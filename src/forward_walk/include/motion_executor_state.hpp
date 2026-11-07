#ifndef MOTION_EXECUTOR_STATE_HPP
#define MOTION_EXECUTOR_STATE_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace motion_executor {
using RawPositions = std::unordered_map<int, int32_t>;
using Clock = std::chrono::steady_clock;
inline constexpr std::chrono::milliseconds kPeriod{20};

// READY is emitted for 11/12/13, 21..25 and LINE units 32..34; the controller's
// current-action allowlist determines whether it sends a queued successor.
inline bool IsReadyWalkAction(uint16_t action)
{
    return action == 11 || action == 12 || action == 13 ||
        (action >= 21 && action <= 25) || (action >= 32 && action <= 34) ||
        action == 36;
}
inline bool IsLongLineAction(uint8_t mission, uint16_t action)
{
    return mission == 1 && IsReadyWalkAction(action);
}

class CompletedHistory {
public:
    bool Contains(uint64_t id) const {
        return id != 0 && std::find(ids_.begin(), ids_.end(), id) != ids_.end();
    }
    void Remember(uint64_t id) {
        if (id == 0 || Contains(id)) return;
        ids_.push_back(id);
        if (ids_.size() > 32) ids_.pop_front();
    }
private:
    std::deque<uint64_t> ids_;
};

enum class Admission { Reject, Start, Queue, ActiveDuplicate, QueuedDuplicate, CompletedDuplicate };

// 실제 측정한 초기 자세를 기준으로 양쪽 팔을 같은 각도만큼 벌린다.
// 바깥 방향의 부호는 다리 directions 배열에서 가져오면 안 된다(팔 ID가 그 배열에 없음).
struct LineArmConfig {
    bool enabled{true};
    int reference_15{1071};
    int reference_16{3073};
    int direction_15{1};
    int direction_16{-1};
    // 15번의 측정 이동량 +308 tick을 양쪽에 대칭 적용한다.
    double angle_deg{308.0 * 360.0 / 4096.0};
    void Validate() const {
        if (reference_15 < 0 || reference_15 > 4095 || reference_16 < 0 || reference_16 > 4095 ||
            !std::isfinite(angle_deg) || angle_deg < 0.0 || angle_deg > 180.0 ||
            (direction_15 != -1 && direction_15 != 0 && direction_15 != 1) ||
            (direction_16 != -1 && direction_16 != 0 && direction_16 != 1)) {
            throw std::invalid_argument("invalid LINE arm pose config");
        }
        if (enabled && (direction_15 == 0 || direction_16 == 0)) {
            throw std::invalid_argument("LINE arm outward directions must be configured as +1/-1");
        }
        if (enabled) Targets();
    }
    RawPositions Targets() const {
        const auto ticks = static_cast<int32_t>(std::nearbyint(angle_deg * 4096.0 / 360.0));
        const int32_t tick_15 = reference_15 + direction_15 * ticks;
        const int32_t tick_16 = reference_16 + direction_16 * ticks;
        if (tick_15 < 0 || tick_15 > 4095 || tick_16 < 0 || tick_16 > 4095) {
            throw std::invalid_argument("LINE arm target outside 0..4095");
        }
        return {{15, tick_15}, {16, tick_16}};
    }
};

// Evaluating an incoming ID never mutates state. Commit only after preparation.
class ActionTransactions {
public:
    Admission Evaluate(uint64_t id, uint8_t mission, uint16_t action) const {
        if (id == 0) return Admission::Reject;
        if (id == active_id) return Admission::ActiveDuplicate;
        if (id == queued_id) return Admission::QueuedDuplicate;
        if (completed.Contains(id)) return Admission::CompletedDuplicate;
        if (queued_id != 0) return Admission::Reject;
        if (active_id == 0) return Admission::Start;
        // Only current A's READY and the empty one-slot queue govern admission.
        // B may be any valid action or mission; full JSON validation precedes ACK.
        if (ready_sent && queued_id == 0) return Admission::Queue;
        return Admission::Reject;
    }
    void Activate(uint64_t id, uint8_t mission, uint16_t action) {
        active_id = id;
        active_mission = mission;
        active_action = action;
        ready_sent = false;
        queued_id = 0;
    }
    bool MarkReady() {
        if (active_id == 0 || ready_sent || !IsReadyWalkAction(active_action)) return false;
        ready_sent = true;
        return true;
    }
    void CompleteActive() {
        completed.Remember(active_id);
        active_id = 0;
        ready_sent = false;
    }
    void Abort() {
        active_id = 0;
        queued_id = 0;
        ready_sent = false;
    }
    uint64_t active_id{0};
    uint64_t queued_id{0};
    uint8_t active_mission{0};
    uint16_t active_action{0};
    bool ready_sent{false};
    CompletedHistory completed;
};

class CameraTransactions {
public:
    Admission Evaluate(uint64_t id) const {
        if (id == 0) return Admission::Reject;
        if (id == active_id) return Admission::ActiveDuplicate;
        if (completed.Contains(id)) return Admission::CompletedDuplicate;
        return active_id == 0 ? Admission::Start : Admission::Reject;
    }
    void CompleteActive() { completed.Remember(active_id); active_id = 0; }
    uint64_t active_id{0};
    CompletedHistory completed;
};

// Minimum body hold after ordinary WALK playback. DONE is never postponed.
class WalkInterActionHold {
public:
    void SetDuration(double sec) {
        if (!std::isfinite(sec) || sec < 0) throw std::invalid_argument("invalid walk hold duration");
        duration_ = sec;
    }
    void Complete(uint16_t action, Clock::time_point now) {
        const bool walk = (action >= 10 && action <= 13) ||
            (action >= 21 && action <= 28) || (action >= 31 && action <= 34) ||
            action == 36;
        until_ = walk ? now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(duration_)) : now;
    }
    bool Waiting(Clock::time_point now) const { return now < until_; }
    void Reset() { until_ = {}; }
private:
    double duration_{0.0};
    Clock::time_point until_{};
};

struct CameraPose { int32_t yaw; int32_t pitch; };
struct CameraConfig {
    int yaw_id{21};
    int pitch_id{22};
    CameraPose forward{2077, 1537};
    CameraPose down{2036, 2013};
    CameraPose goal{2054, 966};
    CameraPose ball_view{2036, 1750};
    double move_sec{0.5};
    double settle_sec{0.0};
    double position_tolerance_ratio{0.10};
    void Validate() const {
        if (yaw_id < 0 || yaw_id > 22 || pitch_id < 0 || pitch_id > 22 || yaw_id == pitch_id ||
            !std::isfinite(move_sec) || move_sec <= 0 ||
            move_sec * 50.0 >= static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
            !std::isfinite(settle_sec) || settle_sec < 0 ||
            !std::isfinite(position_tolerance_ratio) || position_tolerance_ratio<0 || position_tolerance_ratio>1) {
            throw std::invalid_argument("invalid camera IDs/timing");
        }
        for (const auto pose : {forward, down, goal, ball_view}) {
            if (pose.yaw < 0 || pose.yaw > 4095 || pose.pitch < 0 || pose.pitch > 4095) {
                throw std::invalid_argument("camera tick outside 0..4095");
            }
        }
    }
    CameraPose Target(uint8_t request) const {
        switch (request) {
        case 1: return down;
        case 2: return forward;
        case 3: return goal;
        case 9: case 10: return ball_view;
        default: throw std::invalid_argument("unknown camera request");
        }
    }
};

class CameraMotion {
public:
    enum class Result { Idle, Running, Command, Verify };
    enum class ScanPhase : uint8_t { None=0, Outbound=1, Dwell=2, Paused=3, Returning=4, Verify=5, Settling=6 };
    ScanPhase ScanState() const { return scan_phase_; }
    uint32_t ScanStopId() const { return scan_stop_id_; }
    bool ScanActive() const { return scan_phase_ != ScanPhase::None; }
    double ScanCommandedYawDeg() const {
        return (static_cast<double>(current_.yaw)-scan_origin_.yaw)*360.0/4096.0;
    }
    void StartScan(bool left, int max_yaw_deg, double move_sec,
                   double hold_sec, double return_sec,
                   const RawPositions& base, Clock::time_point now,
                   bool goal_view = false, double observation_timeout_sec = 0.0) {
        if(active_) throw std::runtime_error("camera already active");
        if(max_yaw_deg<1 || max_yaw_deg>90 || move_sec<=0 || hold_sec<0 || return_sec<=0 ||
            !std::isfinite(move_sec) || !std::isfinite(hold_sec) || !std::isfinite(return_sec))
            throw std::invalid_argument("invalid scan configuration");
        scan_origin_=goal_view ? config_.goal : config_.forward;
        const int sign=left ? 1 : -1; // Robot-mounted yaw: positive tick delta scans left.
        const int proposed=scan_origin_.yaw + static_cast<int>(
            std::lround(sign*max_yaw_deg*4096.0/360.0));
        if(proposed<0 || proposed>4095)
            throw std::invalid_argument("scan yaw target outside 0..4095");
        scan_far_={proposed,scan_origin_.pitch};
        scan_move_sec_=move_sec;scan_hold_sec_=hold_sec;scan_return_sec_=return_sec;
        point_scan_=observation_timeout_sec>0;
        observation_timeout_sec_=observation_timeout_sec;
        point_index_=left ? 0 : 1;
        scan_amplitude_=std::abs(proposed-scan_origin_.yaw);
        if(point_scan_ && (scan_origin_.yaw-scan_amplitude_<0 ||
                          !std::isfinite(observation_timeout_sec) || observation_timeout_sec<hold_sec))
            throw std::invalid_argument("invalid stationary scan range/timeout");
        scan_stop_id_=0;
        paused_was_dwell_=paused_was_returning_=false;
        current_={base.at(config_.yaw_id),base.at(config_.pitch_id)};
        active_=override_enabled_=true;
        scan_phase_=ScanPhase::Outbound;
        BeginSegment(scan_far_,scan_move_sec_,now);
    }
    bool HoldScan() {
        if(point_scan_ && scan_phase_!=ScanPhase::Dwell && scan_phase_!=ScanPhase::Paused)
            return false; // Never freeze a point scan halfway through a movement.
        if(scan_phase_!=ScanPhase::Outbound && scan_phase_!=ScanPhase::Dwell &&
           scan_phase_!=ScanPhase::Returning)
            return scan_phase_==ScanPhase::Paused;
        paused_was_dwell_=scan_phase_==ScanPhase::Dwell;
        paused_was_returning_=scan_phase_==ScanPhase::Returning;
        scan_phase_=ScanPhase::Paused;
        start_=target_=current_;
        return true;
    }
    bool ResumeScan(Clock::time_point now) {
        if(point_scan_) {
            if(scan_phase_!=ScanPhase::Dwell && scan_phase_!=ScanPhase::Paused) return false;
            if(++point_index_>2) { scan_phase_=ScanPhase::Verify; return true; }
            const CameraPose next{point_index_==1 ? scan_origin_.yaw-scan_amplitude_ :
                                                     scan_origin_.yaw,scan_origin_.pitch};
            const double distance=std::abs(double(next.yaw-current_.yaw));
            scan_phase_=ScanPhase::Outbound;
            BeginSegment(next,std::max(.02,scan_move_sec_*distance/std::max(1,scan_amplitude_)),now);
            return true;
        }
        if(scan_phase_!=ScanPhase::Paused) return false;
        if(paused_was_dwell_) {
            scan_phase_=ScanPhase::Dwell;
            dwell_until_=now + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(scan_hold_sec_));
        } else if(paused_was_returning_) {
            scan_phase_=ScanPhase::Returning;
            const double full=std::max(1.0,std::abs(double(scan_far_.yaw-scan_origin_.yaw)));
            const double remaining=std::abs(double(current_.yaw-scan_origin_.yaw))/full;
            BeginSegment(scan_origin_,std::max(0.02,scan_return_sec_*remaining),now);
        } else {
            scan_phase_=ScanPhase::Outbound;
            const double full=std::max(1.0,std::abs(double(scan_far_.yaw-scan_origin_.yaw)));
            const double remaining=std::abs(double(scan_far_.yaw-current_.yaw))/full;
            BeginSegment(scan_far_,std::max(0.02,scan_move_sec_*remaining),now);
        }
        return true;
    }
    bool ReturnScan(Clock::time_point now) {
        if(scan_phase_==ScanPhase::None || scan_phase_==ScanPhase::Returning ||
           scan_phase_==ScanPhase::Verify) return false;
        scan_phase_=ScanPhase::Returning;
        BeginSegment(scan_origin_,scan_return_sec_,now);
        return true;
    }
    explicit CameraMotion(CameraConfig config = {}) : config_(config) { config_.Validate(); }
    void Start(uint8_t request, const RawPositions& base, Clock::time_point now) {
        point_scan_=false;
        const auto target = config_.Target(request);
        current_ = {base.at(config_.yaw_id), base.at(config_.pitch_id)};
        scan_phase_=ScanPhase::None;
        active_ = override_enabled_ = true;
        BeginSegment(target,config_.move_sec,now);
    }
    Result Update(Clock::time_point now) {
        if (!active_) return Result::Idle;
        if(point_scan_ && (scan_phase_==ScanPhase::Dwell || scan_phase_==ScanPhase::Paused)) {
            if(now>=observation_until_) ResumeScan(now); // Camera cannot wait forever for vision.
            return scan_phase_==ScanPhase::Verify ? Result::Verify : Result::Running;
        }
        if(point_scan_ && scan_phase_==ScanPhase::Settling) return Result::Verify;
        if (scan_phase_==ScanPhase::Paused) return Result::Running;
        if (scan_phase_==ScanPhase::Dwell) {
            if(now<dwell_until_) return Result::Running;
            scan_phase_=ScanPhase::Returning;
            BeginSegment(scan_origin_,scan_return_sec_,now);
        }
        if (scan_phase_==ScanPhase::Verify) return Result::Verify;
        const auto result=UpdateSegment(now);
        if(result==Result::Verify && scan_phase_==ScanPhase::Outbound) {
            if(point_scan_) { scan_phase_=ScanPhase::Settling; return Result::Verify; }
            scan_phase_=ScanPhase::Dwell;
            dwell_until_=now + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(scan_hold_sec_));
            return Result::Running;
        }
        if(result==Result::Verify && scan_phase_==ScanPhase::Returning)
            scan_phase_=ScanPhase::Verify;
        return result;
    }
    void OnCommandWritten(Clock::time_point started, Clock::time_point written) {
        next_at_ = std::max(started + kPeriod, written);
    }
    void Merge(RawPositions& full_command) const {
        if (!override_enabled_) return;
        full_command.at(config_.yaw_id) = current_.yaw;
        full_command.at(config_.pitch_id) = current_.pitch;
    }
    bool Active() const { return active_; }
    CameraPose TargetPose() const { return target_; }
    bool ConfirmArrival(CameraPose measured, Clock::time_point now=Clock::now()) {
        if (!active_ || step_!=steps_ || !settling_) return false;
        const auto close=[this](int32_t actual,int32_t target) {
            return actual>=0 && actual<=4095 &&
                std::abs(static_cast<double>(actual)-target)<=
                    std::abs(static_cast<double>(target))*config_.position_tolerance_ratio;
        };
        if (!close(measured.yaw,target_.yaw) || !close(measured.pitch,target_.pitch)) return false;
        if(point_scan_ && scan_phase_==ScanPhase::Settling) {
            scan_phase_=ScanPhase::Dwell;
            ++scan_stop_id_;
            observation_until_=now+std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(observation_timeout_sec_));
            return false; // An intermediate arrival is not transaction DONE.
        }
        active_=false;
        scan_phase_=ScanPhase::None;
        return true;
    }
    bool OverrideEnabled() const { return override_enabled_; }
    // 시작 자세 완료 후 기본 FORWARD 목표를 유지한다. 외부 카메라 명령/상태 ID는 만들지 않는다.
    // 실제 전송은 MainNode의 단일 writer가 수행하고, 이후 DOWN/GOAL 명령은 이 자세를 변경할 수 있다.
    void HoldForward() {
        HoldPose(config_.forward);
    }
    // Adopt an already transmitted head pose, without a new camera command.
    void HoldPose(CameraPose pose) {
        start_ = target_ = current_ = pose;
        step_ = steps_ = 0;
        active_ = settling_ = false;
        scan_phase_=ScanPhase::None;
        override_enabled_ = true;
    }
    // Retain the last successfully transmitted override if the writer fails.
    void Abort(const RawPositions& last_sent) {
        active_ = false;
        scan_phase_=ScanPhase::None;
        if (override_enabled_ && !last_sent.empty()) {
            current_ = {last_sent.at(config_.yaw_id), last_sent.at(config_.pitch_id)};
        }
    }
private:
    void BeginSegment(CameraPose goal,double seconds,Clock::time_point now) {
        start_=current_;target_=goal;
        steps_=static_cast<std::size_t>(std::max(2.0,std::nearbyint(seconds*50.0)));
        step_=0;next_at_=now;settling_=false;
    }
    Result UpdateSegment(Clock::time_point now) {
        if(now<next_at_) return Result::Running;
        if(step_==steps_) {
            if(!settling_) { settling_=true;settle_started_at_=now; }
            if(std::chrono::duration<double>(now-settle_started_at_).count()<config_.settle_sec)
                return Result::Running;
            return Result::Verify;
        }
        const double x=static_cast<double>(++step_)/steps_;
        const double alpha=x*x*(3.0-2.0*x); // Exactly the existing camera interpolation.
        current_.yaw=static_cast<int32_t>(std::nearbyint(start_.yaw+alpha*(target_.yaw-start_.yaw)));
        current_.pitch=static_cast<int32_t>(std::nearbyint(start_.pitch+alpha*(target_.pitch-start_.pitch)));
        next_at_=now+kPeriod;
        return Result::Command;
    }
    ScanPhase scan_phase_{ScanPhase::None};
    bool point_scan_{false};
    int point_index_{0},scan_amplitude_{0};
    uint32_t scan_stop_id_{0};
    double observation_timeout_sec_{0};
    Clock::time_point observation_until_{};
    CameraPose scan_far_{},scan_origin_{};
    double scan_move_sec_{0.5},scan_hold_sec_{0.5},scan_return_sec_{0.5};
    bool paused_was_dwell_{false},paused_was_returning_{false};
    Clock::time_point dwell_until_{};
    CameraConfig config_;
    CameraPose start_{0, 0}, target_{0, 0}, current_{0, 0};
    std::size_t step_{0}, steps_{0};
    Clock::time_point next_at_{}, settle_started_at_{};
    bool active_{false}, settling_{false}, override_enabled_{false};
};
}
#endif

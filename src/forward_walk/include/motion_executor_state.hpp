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

inline bool IsLongLineAction(uint8_t mission, uint16_t action)
{
    return mission == 1 && (action == 11 || action == 12 || action == 13);
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

// Evaluating an incoming ID never mutates state. Commit only after preparation.
class ActionTransactions {
public:
    Admission Evaluate(uint64_t id, uint8_t mission, uint16_t action) const {
        if (id == 0) return Admission::Reject;
        if (id == active_id) return Admission::ActiveDuplicate;
        if (id == queued_id) return Admission::QueuedDuplicate;
        if (completed.Contains(id)) return Admission::CompletedDuplicate;
        if (active_id == 0) return Admission::Start;
        if (ready_sent && queued_id == 0 && IsLongLineAction(active_mission, active_action) &&
            IsLongLineAction(mission, action)) return Admission::Queue;
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
        if (active_id == 0 || ready_sent || !IsLongLineAction(active_mission, active_action)) return false;
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

struct CameraPose { int32_t yaw; int32_t pitch; };
struct CameraConfig {
    int yaw_id{21};
    int pitch_id{22};
    CameraPose forward{2077, 1537};
    CameraPose down{2036, 2013};
    CameraPose goal{2054, 966};
    double move_sec{0.5};
    double settle_sec{0.2};
    void Validate() const {
        if (yaw_id < 0 || yaw_id > 22 || pitch_id < 0 || pitch_id > 22 || yaw_id == pitch_id ||
            !std::isfinite(move_sec) || move_sec <= 0 ||
            move_sec * 50.0 >= static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
            !std::isfinite(settle_sec) || settle_sec < 0) {
            throw std::invalid_argument("invalid camera IDs/timing");
        }
        for (const auto pose : {forward, down, goal}) {
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
        default: throw std::invalid_argument("unknown camera request");
        }
    }
};

class CameraMotion {
public:
    enum class Result { Idle, Running, Command, Finished };
    explicit CameraMotion(CameraConfig config = {}) : config_(config) { config_.Validate(); }
    void Start(uint8_t request, const RawPositions& base, Clock::time_point now) {
        const auto target = config_.Target(request);
        const CameraPose start{base.at(config_.yaw_id), base.at(config_.pitch_id)};
        target_ = target;
        start_ = current_ = start;
        steps_ = static_cast<std::size_t>(std::max(2.0, std::nearbyint(config_.move_sec * 50.0)));
        step_ = 0;
        next_at_ = now;
        settling_ = false;
        active_ = true;
        override_enabled_ = true;
    }
    Result Update(Clock::time_point now) {
        if (!active_) return Result::Idle;
        if (now < next_at_) return Result::Running;
        if (step_ == steps_) {
            if (!settling_) { settling_ = true; settle_started_at_ = now; }
            if (std::chrono::duration<double>(now - settle_started_at_).count() < config_.settle_sec) {
                return Result::Running;
            }
            active_ = false;
            return Result::Finished;
        }
        const double x = static_cast<double>(++step_) / steps_;
        const double alpha = x*x*(3.0 - 2.0*x);
        current_.yaw = static_cast<int32_t>(std::nearbyint(start_.yaw + alpha*(target_.yaw - start_.yaw)));
        current_.pitch = static_cast<int32_t>(std::nearbyint(start_.pitch + alpha*(target_.pitch - start_.pitch)));
        next_at_ = now + kPeriod;
        return Result::Command;
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
    bool OverrideEnabled() const { return override_enabled_; }
    // Retain the last successfully transmitted override if the writer fails.
    void Abort(const RawPositions& last_sent) {
        active_ = false;
        if (override_enabled_ && !last_sent.empty()) {
            current_ = {last_sent.at(config_.yaw_id), last_sent.at(config_.pitch_id)};
        }
    }
private:
    CameraConfig config_;
    CameraPose start_{0, 0}, target_{0, 0}, current_{0, 0};
    std::size_t step_{0}, steps_{0};
    Clock::time_point next_at_{}, settle_started_at_{};
    bool active_{false}, settling_{false}, override_enabled_{false};
};
}
#endif

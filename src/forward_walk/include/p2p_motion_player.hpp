#ifndef P2P_MOTION_PLAYER_HPP
#define P2P_MOTION_PLAYER_HPP

#include "dynamixel.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class P2PMotionPlayer
{
    struct Motion;
public:
    using Clock = std::chrono::steady_clock;
    using RawPositions = std::unordered_map<int, int32_t>;
    class PreparedProgram {
        friend class P2PMotionPlayer;
        std::vector<Motion> motions;
    };
    // main.cpp 타이머가 Update()를 호출하고, player가 50 Hz 전송을 조절한다.
    // kFinished일 때만 해당 ActionCommand에 대한 DONE을 보내야 한다.
    enum class UpdateResult { kIdle, kRunning, kFinished, kError };

    struct StartOptions {
        std::size_t repeat_count{1};
        std::unordered_map<int, int32_t> position_offsets;
        bool restore_offsets{false};
        std::vector<std::string> trailing_program_paths;
        // 노드 시작 시 전체 WALK_MODE 모션을 재생하지 않고 마지막 자세로만 이동한다.
        bool final_keyframe_only{false};
        std::optional<double> duration_override_sec;
    };

    explicit P2PMotionPlayer(Dxl* dxl);

    // motion 또는 mission JSON 파싱 → 현재 encoder 위치 읽기 → 첫 keyframe
    // 준비를 한 번에 수행한다. mission의 motion들은 순서대로 이어서 실행한다.
    // 실제 반복 재생은 Start() 내부에서 돌지 않고 Update()가 한 step씩 담당한다.
    bool Start(const std::string& json_path,
               const std::string& motion_directory,
               const StartOptions& options);

    // Parse and apply options to a separate local program. No hardware/state changes.
    PreparedProgram PrepareProgram(const std::string& json_path,
                                   const std::string& motion_directory,
                                   const StartOptions& options) const;
    bool StartPrepared(const PreparedProgram& program);
    // Explicit initial pose/clock permits hardware-free verification of fixed steps.
    bool StartPrepared(const PreparedProgram& program, const RawPositions& initial_pose,
                       Clock::time_point now);

    // ROS timer에서 주기적으로 호출한다. 사이트와 같은 step/steps 보간으로
    // 한 번에 한 단계만 전송한다. 지연되어도 단계를 건너뛰거나 몰아서 보내지 않는다.
    UpdateResult Update(RawPositions* output, Clock::time_point now = Clock::now());
    // Only MainNode writes the merged packet; account for its actual write time.
    void OnCommandWritten(Clock::time_point write_started, Clock::time_point written_at);
    double RemainingNominalSec(Clock::time_point now = Clock::now()) const;
    bool IsNearCompletion(double lead_sec, Clock::time_point now = Clock::now()) const;

    // 재생 상태만 정지/초기화한다. Torque OFF 명령은 수행하지 않는다.
    void Stop();

    bool IsPlaying() const { return playing_; }
    const std::string& MotionName() const { return motion_name_; }
    const std::string& Error() const { return error_; }

private:
    // JSON의 interpolation 문자열과 대응한다.
    enum class Interpolation { kLinear, kSmoothStep, kMinimumJerk };

    // JSON keyframes 배열의 원소 하나를 C++에서 보관하는 형태다.
    struct Keyframe {
        std::string name;
        // 사용자가 지정한 기본 이동시간
        double duration_sec{1.0};
        // 목표 자세 도착 후 그대로 유지할 시간
        double hold_sec{0.0};
        // 가장 많이 움직이는 관절도 이 속도를 넘지 않도록 이동시간을 늘린다.
        double max_speed_deg_s{30.0};
        Interpolation interpolation{Interpolation::kSmoothStep};
        // key=motor ID, value=Dynamixel raw encoder tick
        std::unordered_map<int, int32_t> positions;
    };

    struct Motion {
        std::string name;
        std::vector<Keyframe> keyframes;
    };

    Motion LoadMotion(const std::filesystem::path& json_path) const;
    void LoadProgram(const std::filesystem::path& json_path,
                     const std::filesystem::path& motion_directory);
    void ApplyStartOptions(const StartOptions& options,
                           const std::filesystem::path& motion_directory);
    void AppendProgram(const std::filesystem::path& json_path,
                       const std::filesystem::path& motion_directory);
    void SelectMotion(std::size_t index);
    void BeginKeyframe(Clock::time_point now);     // 현재 keyframe 시간/속도 계산
    static double Interpolate(Interpolation kind, double x);

    Dxl* dxl_{nullptr};
    std::string motion_name_;
    std::string error_;
    std::vector<Motion> motions_;
    std::vector<Keyframe> keyframes_;
    std::size_t motion_index_{0};         // mission 내에서 현재 실행 중인 motion
    RawPositions start_positions_;       // 현재 구간을 시작한 실제/직전 목표 위치
    std::size_t keyframe_index_{0};       // 지금 실행 중인 JSON keyframe 번호
    double effective_duration_sec_{0.0};
    static constexpr std::chrono::milliseconds kCommandPeriod{20};
    std::size_t total_steps_{0};
    std::size_t completed_steps_{0};
    Clock::time_point next_command_at_{};
    Clock::time_point phase_started_at_{}; // hold를 시작한 시각
    bool holding_{false};                  // false=이동 중, true=hold 중
    bool playing_{false};                  // 전체 모션 재생 여부
};

#endif

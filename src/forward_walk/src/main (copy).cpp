#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "dynamixel.hpp"
#include "p2p_motion_player.hpp"
#include "vision/msg/action_command.hpp"
#include "vision/msg/camera_command.hpp"
#include "vision/msg/command_status.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class MainNode : public rclcpp::Node
{
public:
    MainNode() : Node("main_node")
    {
        // ==================== P2P 전체 실행 흐름 ====================
        // 1) /g1_vision/action_cmd에서 ActionCommand 수신
        // 2) ProgramFileForAction()으로 action → JSON 파일명 변환
        // 3) P2PMotionPlayer::Start()가 JSON과 현재 encoder 위치를 준비
        // 4) 준비 성공 후 같은 action_id로 ACK publish
        // 5) 10 ms MotionLoop에서 Update()를 반복해 raw tick 전송
        // 6) 마지막 keyframe/hold 완료 후 같은 action_id로 DONE publish

        // motions/와 missions/를 포함한 패키지 share 루트다.
        declare_parameter<std::string>(
            "p2p_asset_directory",
            ament_index_cpp::get_package_share_directory("forward_walk"));
        asset_directory_ = get_parameter("p2p_asset_directory").as_string();

        turn_step_deg_ = declare_parameter<double>("turn_step_deg", 15.0);
        max_turn_repetitions_ =
            declare_parameter<int>("max_turn_repetitions", 12);
        waist_motor_id_ = declare_parameter<int>("waist_motor_id", 12);
        waist_max_yaw_deg_ =
            declare_parameter<double>("waist_max_yaw_deg", 30.0);
        waist_ticks_per_revolution_ =
            declare_parameter<double>("waist_ticks_per_revolution", 4096.0);
        waist_yaw_direction_ =
            declare_parameter<int>("waist_yaw_direction", 1);
        waist_restore_after_shoot_ =
            declare_parameter<bool>("waist_restore_after_shoot", true);
        startup_pose_duration_sec_ =
            declare_parameter<double>("startup_pose_duration_sec", 3.0);

        if (turn_step_deg_ <= 0.0 || max_turn_repetitions_ <= 0 ||
            waist_max_yaw_deg_ <= 0.0 || waist_ticks_per_revolution_ <= 0.0 ||
            startup_pose_duration_sec_ <= 0.0 ||
            (waist_yaw_direction_ != 1 && waist_yaw_direction_ != -1)) {
            throw std::invalid_argument("invalid P2P parameter");
        }

        // P2P는 JSON에 완성된 관절 raw tick이 있으므로 Trajectory/IK/Callback이
        // 필요 없다. Dxl과 JSON player만 생성하고 첫 ActionCommand를 기다린다.
        // USB latency_timer는 프로그램 안에서 sudo를 실행하지 않고 udev 규칙으로
        // 설정해야 한다(설정 방법은 README/실행 환경에서 별도로 관리).
        dxl_ = std::make_shared<Dxl>();
        p2p_player_ = std::make_unique<P2PMotionPlayer>(dxl_.get());

        OpenTorqueLogFile();

        // 새 Vision 프로토콜은 /g1_vision/action_status의 ACK/DONE을 사용한다.
        action_status_pub_ = create_publisher<vision::msg::CommandStatus>(
            "/jandi_vision/action_status", 10);
        // Camera status는 별도 ID 공간과 토픽을 사용하므로 publisher도 분리한다.
        camera_status_pub_ = create_publisher<vision::msg::CommandStatus>(
            "/jandi_vision/camera_status", 10);

        // JSON 보간과 전송을 callback에서 block하지 않고 100 Hz로 수행한다.
        motion_loop_timer_ = create_wall_timer(
            20ms, std::bind(&MainNode::MotionLoop, this));
        motion_loop_timer_->cancel();

        // WALK_MODE 마지막 자세에 도착하기 전에는 명령 topic을 구독하지 않는다.
        StartStartupPose();
    }

private:
    static constexpr int kLegJointCount = 12;
    static constexpr double kTorquePerMilliAmp =
        1.0 / (TORQUE_TO_VALUE_MX_106 * 3.36);
    static constexpr const char* kTorqueLogDir = "/tmp/forward_walk_logs";

    void CreateCommandSubscriptions()
    {
        action_cmd_sub_ = create_subscription<vision::msg::ActionCommand>(
            "/jandi_vision/action_cmd", 10,
            std::bind(&MainNode::ActionCommandCallback, this,
                      std::placeholders::_1));
        camera_cmd_sub_ = create_subscription<vision::msg::CameraCommand>(
            "/jandi_vision/camera_cmd", 10,
            std::bind(&MainNode::CameraCommandCallback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(
            get_logger(),
            "Startup WALK_MODE pose reached and held; ready for command topics");
    }

    void StartStartupPose()
    {
        const std::filesystem::path asset_root(asset_directory_);
        const std::filesystem::path path =
            asset_root /
            ProgramFileForAction(vision::msg::ActionCommand::WALK_MODE);

        P2PMotionPlayer::StartOptions options;
        options.final_keyframe_only = true;
        options.duration_override_sec = startup_pose_duration_sec_;
        if (!p2p_player_->Start(path.string(),
                                (asset_root / "motions").string(), options)) {
            RCLCPP_FATAL(get_logger(),
                         "Startup WALK_MODE pose failed; command topics will not be enabled: %s",
                         p2p_player_->Error().c_str());
            return;
        }

        startup_pose_in_progress_ = true;
        current_action_ = vision::msg::ActionCommand::WALK_MODE;
        motion_loop_timer_->reset();
        RCLCPP_INFO(
            get_logger(),
            "Moving to final WALK_MODE pose over %.2f seconds before accepting commands",
            startup_pose_duration_sec_);
    }

    // 이 함수는 action의 의미와 실제 JSON 파일을 연결하는 유일한 표다.
    // action_id는 명령 인스턴스 ID이므로 절대 이 switch에 사용하지 않는다.
    // 새 JSON을 추가할 때 action 번호와 파일의 대응은 여기만 수정하면 된다.
    std::string ProgramFileForAction(std::uint16_t action) const
    {
        switch (action) {
        // TODO: 실제 로봇에서 검증한 action과 JSON 파일만 추가한다.
        // 예시:
        case vision::msg::ActionCommand::DEFAULT_POSITION:
            return "motions/초기자세_다소곳.json";

        case vision::msg::ActionCommand::DEFAULT_POSE_MODE:
            return "motions/미세보행_초기자세가기.json";

        case vision::msg::ActionCommand::STEP_FORWARD_HALF:
            return "motions/미세보행_미세전진.json";

        case vision::msg::ActionCommand::STEP_BACK:
            return "motions/미세보행_후진.json";

        case vision::msg::ActionCommand::LEFT_SIDE_STEP:
            return "motions/미세보행_좌로이동.json";

        case vision::msg::ActionCommand::RIGHT_SIDE_STEP:
            return "motions/미세보행_우로이동.json";

        case vision::msg::ActionCommand::TURN_LEFT:
            return "motions/미세보행_좌회전.json";

        case vision::msg::ActionCommand::TURN_RIGHT:
            return "motions/미세보행_우회전.json";

        case vision::msg::ActionCommand::TURN_LEFT_AND_STEP:
            return "motions/미세보행_좌회전.json";

        case vision::msg::ActionCommand::TURN_RIGHT_AND_STEP:
            return "motions/미세보행_우회전.json";

        case vision::msg::ActionCommand::WALK_MODE:
            return "motions/보행_왼발만앞에.json";

        case vision::msg::ActionCommand::STEP_FORWARD_ONE:
            return "motions/보행_1걸음_초기X.json";

        case vision::msg::ActionCommand::STEP_FORWARD_LEFT:
            return "motions/보행_좌회전전진_20도_최종_초기X.json";

        case vision::msg::ActionCommand::STEP_FORWARD_RIGHT:
            return "motions/보행_우회전전진_20도_최종_초기X.json";

        case vision::msg::ActionCommand::STEP_FORWARD_FIVE:
            return "missions/연속걷기.json";

        case vision::msg::ActionCommand::PICK_BALL:
            return "motions/공_줍고일어나기.json";

        case vision::msg::ActionCommand::RECATCH:
            return "motions/공_재그립.json";

        case vision::msg::ActionCommand::HUDDLE:
            return "missions/허들넘기.json";
            
        case vision::msg::ActionCommand::SHOOT:
            return "motions/공_던지기.json";
        
        default:
            return {};
        }
    }

    bool IsLeftTurnAction(std::uint16_t action) const
    {
        return action == vision::msg::ActionCommand::TURN_LEFT ||
               action == vision::msg::ActionCommand::TURN_LEFT_AND_STEP;
    }

    bool IsRightTurnAction(std::uint16_t action) const
    {
        return action == vision::msg::ActionCommand::TURN_RIGHT ||
               action == vision::msg::ActionCommand::TURN_RIGHT_AND_STEP;
    }

    bool IsTurnAction(std::uint16_t action) const
    {
        return IsLeftTurnAction(action) || IsRightTurnAction(action);
    }

    bool IsTurnAndStepAction(std::uint16_t action) const
    {
        return action == vision::msg::ActionCommand::TURN_LEFT_AND_STEP ||
               action == vision::msg::ActionCommand::TURN_RIGHT_AND_STEP;
    }

    int CalculateTurnCount(int target_yaw_deg) const
    {
        const int requested = static_cast<int>(std::lround(
            target_yaw_deg / turn_step_deg_));
        return std::clamp(requested, 0, max_turn_repetitions_);
    }

    void PublishActionStatus(std::uint64_t command_id, std::uint8_t status)
    {
        // ActionCommand.action_id를 CommandStatus.command_id에 그대로 돌려줘야
        // Vision이 어느 명령의 ACK/DONE인지 대응시킬 수 있다.
        vision::msg::CommandStatus message;
        message.command_id = command_id;
        message.status = status;
        action_status_pub_->publish(message);
    }

    void ActionCommandCallback(
        const vision::msg::ActionCommand::SharedPtr message)
    {
        RCLCPP_INFO(get_logger(),
                    "Action received: id=%llu mission=%u action=%u target_yaw_deg=%d",
                    static_cast<unsigned long long>(message->action_id),
                    static_cast<unsigned int>(message->mission),
                    static_cast<unsigned int>(message->action),
                    static_cast<int>(message->target_yaw_deg));

        // Vision 코드에서 0은 "pending action 없음"이므로 실행 명령으로 받지 않는다.
        if (message->action_id == 0) {
            RCLCPP_WARN(get_logger(), "action_id 0 is invalid");
            return;
        }
        // Vision은 ACK 전까지 같은 ID를 재전송하므로 다시 실행하지 않는다.
        if (motion_in_progress_ && message->action_id == active_action_id_) {
            RCLCPP_WARN(
                get_logger(),
                "Duplicate action id=%llu is already in progress; NOT executed again, ACK re-published",
                static_cast<unsigned long long>(message->action_id));
            PublishActionStatus(message->action_id,
                                vision::msg::CommandStatus::ACK);
            return;
        }
        // DONE 유실 뒤 같은 ID가 다시 오면 완료 상태만 재전송한다.
        if (message->action_id == last_completed_action_id_) {
            RCLCPP_WARN(
                get_logger(),
                "Duplicate action id=%llu was already completed; NOT executed again, DONE re-published",
                static_cast<unsigned long long>(message->action_id));
            PublishActionStatus(message->action_id,
                                vision::msg::CommandStatus::DONE);
            return;
        }
        // 다른 ID의 모션이 실행 중이면 ACK하지 않는다. ACK하면 Vision은 이 명령이
        // 접수됐다고 믿기 때문이다. 현재 CommandStatus에는 BUSY/REJECT가 없다.
        if (motion_in_progress_) {
            RCLCPP_WARN(get_logger(),
                        "Motion is busy; action id=%llu was not accepted",
                        static_cast<unsigned long long>(message->action_id));
            return;
        }

        const std::string filename = ProgramFileForAction(message->action);
        if (filename.empty()) {
            RCLCPP_ERROR(get_logger(), "No JSON mapping for action=%u",
                         static_cast<unsigned int>(message->action));
            return;
        }
        const int requested_target_yaw_deg =
            static_cast<int>(message->target_yaw_deg);
        int effective_target_yaw_deg = 0;
        P2PMotionPlayer::StartOptions start_options;

        if (IsTurnAction(message->action)) {
            // 회전 방향은 TURN_LEFT/TURN_RIGHT action으로만 정한다.
            // Vision이 전달한 양수 회전량은 별도 부호 변환 없이 그대로 사용한다.
            const int turn_count =
                CalculateTurnCount(requested_target_yaw_deg);
            if (turn_count == 0 &&
                !IsTurnAndStepAction(message->action)) {
                // 15도 단위 반올림 결과가 0이면 모터를 움직이지 않고 정상 완료한다.
                PublishActionStatus(message->action_id,
                                    vision::msg::CommandStatus::ACK);
                last_completed_action_id_ = message->action_id;
                PublishActionStatus(message->action_id,
                                    vision::msg::CommandStatus::DONE);
                RCLCPP_INFO(get_logger(),
                            "Turn rounded to zero: target_yaw_deg=%d",
                            requested_target_yaw_deg);
                return;
            }

            start_options.repeat_count = static_cast<std::size_t>(turn_count);
            const int direction = IsLeftTurnAction(message->action) ? 1 : -1;
            effective_target_yaw_deg = static_cast<int>(std::lround(
                direction * turn_count * turn_step_deg_));
            RCLCPP_INFO(
                get_logger(),
                "Turn plan: requested=%d deg step=%.2f deg count=%d estimated=%d deg",
                requested_target_yaw_deg, turn_step_deg_, turn_count,
                effective_target_yaw_deg);
        } else if (message->action == vision::msg::ActionCommand::SHOOT) {
            const double clamped_yaw_deg = std::clamp(
                static_cast<double>(requested_target_yaw_deg),
                -waist_max_yaw_deg_, waist_max_yaw_deg_);
            effective_target_yaw_deg =
                static_cast<int>(std::lround(clamped_yaw_deg));
            if (effective_target_yaw_deg != requested_target_yaw_deg) {
                RCLCPP_WARN(
                    get_logger(),
                    "Shoot waist yaw clamped: requested=%d deg applied=%d deg",
                    requested_target_yaw_deg, effective_target_yaw_deg);
            }

            const int32_t waist_offset_ticks = static_cast<int32_t>(
                std::lround(
                    effective_target_yaw_deg * waist_ticks_per_revolution_ /
                    360.0 * waist_yaw_direction_));
            if (waist_offset_ticks != 0) {
                start_options.position_offsets.emplace(
                    waist_motor_id_, waist_offset_ticks);
                start_options.restore_offsets = waist_restore_after_shoot_;
            }
            RCLCPP_INFO(
                get_logger(),
                "Shoot waist plan: yaw=%d deg motor_id=%d offset=%d ticks",
                effective_target_yaw_deg, waist_motor_id_, waist_offset_ticks);
        } else if (requested_target_yaw_deg != 0) {
            RCLCPP_WARN(
                get_logger(),
                "target_yaw_deg=%d ignored for action=%u",
                requested_target_yaw_deg,
                static_cast<unsigned int>(message->action));
        }

        std::filesystem::path relative_path(filename);
        // 기존 매핑처럼 파일명만 있으면 motion으로 간주한다. 신규 매핑은
        // motions/... 또는 missions/... 경로를 명시한다.
        if (!relative_path.has_parent_path()) {
            relative_path = std::filesystem::path("motions") / relative_path;
        }
        const std::filesystem::path asset_root(asset_directory_);
        const std::filesystem::path path = asset_root / relative_path;
        if (IsTurnAndStepAction(message->action)) {
            start_options.trailing_program_paths.push_back(
                (asset_root / "missions" / "연속걷기.json").string());
            RCLCPP_INFO(
                get_logger(),
                "Turn-and-step sequence: turn_count=%zu then continuous walk",
                start_options.repeat_count);
        }
        // 파일 파싱, motor ID 누락 검사, 현재 encoder 읽기가 모두 성공해야 접수한다.
        if (!p2p_player_->Start(
                path.string(), (asset_root / "motions").string(), start_options)) {
            RCLCPP_ERROR(get_logger(), "P2P start failed: %s",
                         p2p_player_->Error().c_str());
            return;
        }

        // ACK보다 먼저 active ID를 보관해야 이후 중복 수신을 같은 명령으로 판별한다.
        active_action_id_ = message->action_id;
        current_action_ = message->action;
        active_target_yaw_deg_ = effective_target_yaw_deg;
        motion_in_progress_ = true;
        PublishActionStatus(active_action_id_, vision::msg::CommandStatus::ACK);
        motion_loop_timer_->reset();
    }

    void CameraCommandCallback(
        const vision::msg::CameraCommand::SharedPtr message)
    {
        // TODO: DOWN/FORWARD/GOAL의 목 관절 JSON을 정한 뒤 별도 player로 연결한다.
        RCLCPP_WARN(get_logger(),
                    "Camera command id=%llu request=%u: mapping is not implemented",
                    static_cast<unsigned long long>(message->command_id),
                    static_cast<unsigned int>(message->request));
    }

    void MotionLoop()
    {
        if (!startup_pose_in_progress_ && !motion_in_progress_) return;

        // 기존 SelectMotion → Write_All_Theta → SetThetaRef 경로를 대체한다.
        // Update() 안에서 JSON 보간과 raw tick GroupSyncWrite가 한 번 수행된다.
        const auto result = p2p_player_->Update();
        if (result == P2PMotionPlayer::UpdateResult::kRunning) {
           //  LogLegJointTorqueStep();
            return;
        }
        if (result == P2PMotionPlayer::UpdateResult::kError) {
            if (startup_pose_in_progress_) {
                RCLCPP_FATAL(
                    get_logger(),
                    "Startup WALK_MODE pose failed; command topics will not be enabled: %s",
                    p2p_player_->Error().c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "P2P playback failed: %s",
                             p2p_player_->Error().c_str());
            }
            startup_pose_in_progress_ = false;
            motion_in_progress_ = false;
            current_action_ = 0;
            active_target_yaw_deg_ = 0;
            motion_loop_timer_->cancel();
            return;
        }
        if (result != P2PMotionPlayer::UpdateResult::kFinished) return;

        if (startup_pose_in_progress_) {
            startup_pose_in_progress_ = false;
            current_action_ = 0;
            motion_loop_timer_->cancel();
            CreateCommandSubscriptions();
            return;
        }

        // JSON의 마지막 keyframe 이동과 hold까지 모두 끝난 시점에만 DONE을 보낸다.
        PublishActionStatus(active_action_id_, vision::msg::CommandStatus::DONE);
        last_completed_action_id_ = active_action_id_;

        RCLCPP_INFO(get_logger(), "P2P program completed: action=%u id=%llu",
                    static_cast<unsigned int>(current_action_),
                    static_cast<unsigned long long>(active_action_id_));
        active_action_id_ = 0;
        current_action_ = 0;
        active_target_yaw_deg_ = 0;
        motion_in_progress_ = false;
        motion_loop_timer_->cancel();
    }

    // 로그를 저장할 폴더와 CSV 파일을 준비하는 것
    void OpenTorqueLogFile()
    {
        namespace fs = std::filesystem;
        fs::create_directories(kTorqueLogDir);
        const auto now = std::chrono::system_clock::now();
        const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now = *std::localtime(&now_c);
        std::ostringstream filename;
        filename << kTorqueLogDir << "/leg_torque_"
                 << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << ".csv";
        torque_log_stream_.open(filename.str(), std::ios::out | std::ios::trunc);
        if (!torque_log_stream_.is_open()) return;
        torque_log_stream_ << std::fixed << std::setprecision(6)
            << "ros_time_sec,command,"
            << "R0_torque_nm,R1_torque_nm,R2_torque_nm,R3_torque_nm,R4_torque_nm,R5_torque_nm,"
            << "L0_torque_nm,L1_torque_nm,L2_torque_nm,L3_torque_nm,L4_torque_nm,L5_torque_nm,"
            << "R0_current_mA,R1_current_mA,R2_current_mA,R3_current_mA,R4_current_mA,R5_current_mA,"
            << "L0_current_mA,L1_current_mA,L2_current_mA,L3_current_mA,L4_current_mA,L5_current_mA\n";
    }

    // LogLegJointTorqueStep() 자체가 다이나믹셀 통신으로 전류만 읽는 함수라기보다는, 전류를 읽은 뒤 토크를 계산하고 CSV 파일에 기록하는 함수
    void LogLegJointTorqueStep()
    {
        if (!torque_log_stream_.is_open()) return;
        const Eigen::VectorXd current_mA = dxl_->GetCurrent();
        torque_log_stream_ << get_clock()->now().seconds() << "," << current_action_;
        for (int i = 0; i < kLegJointCount; ++i)
            torque_log_stream_ << "," << current_mA[i] * kTorquePerMilliAmp;
        for (int i = 0; i < kLegJointCount; ++i)
            torque_log_stream_ << "," << current_mA[i];
        torque_log_stream_ << "\n";
    }

    std::shared_ptr<Dxl> dxl_;
    std::unique_ptr<P2PMotionPlayer> p2p_player_;
    rclcpp::Publisher<vision::msg::CommandStatus>::SharedPtr action_status_pub_;
    rclcpp::Publisher<vision::msg::CommandStatus>::SharedPtr camera_status_pub_;
    rclcpp::Subscription<vision::msg::ActionCommand>::SharedPtr action_cmd_sub_;
    rclcpp::Subscription<vision::msg::CameraCommand>::SharedPtr camera_cmd_sub_;
    rclcpp::TimerBase::SharedPtr motion_loop_timer_;
    std::ofstream torque_log_stream_;
    std::string asset_directory_;
    double turn_step_deg_{15.0};
    int max_turn_repetitions_{12};
    int waist_motor_id_{12};
    double waist_max_yaw_deg_{30.0};
    double waist_ticks_per_revolution_{4096.0};
    int waist_yaw_direction_{1};
    bool waist_restore_after_shoot_{true};
    double startup_pose_duration_sec_{3.0};
    bool startup_pose_in_progress_{false};   // 명령 수신 전 WALK_MODE 자세 이동 여부
    bool motion_in_progress_{false};          // 현재 P2P JSON 실행 여부
    std::uint64_t active_action_id_{0};       // 현재 실행 중인 transaction ID
    std::uint64_t last_completed_action_id_{0}; // DONE 재전송용 최근 완료 ID
    std::uint16_t current_action_{0};         // 로그 및 JSON 종류를 나타내는 action 값
    int active_target_yaw_deg_{0};             // 현재 명령에 실제 적용한 목표각
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}

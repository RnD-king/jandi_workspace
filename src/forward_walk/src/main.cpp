#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "dynamixel.hpp"
#include "p2p_motion_player.hpp"
#include "motion_executor_state.hpp"
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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

static_assert(vision::msg::ActionCommand::MISSION_LINE == 1 &&
              vision::msg::ActionCommand::STEP_FORWARD_LEFT == 11 &&
              vision::msg::ActionCommand::STEP_FORWARD_RIGHT == 12 &&
              vision::msg::ActionCommand::STEP_FORWARD_FIVE == 13);
static_assert(vision::msg::CommandStatus::READY == 3 &&
              vision::msg::CameraCommand::DOWN == 1 &&
              vision::msg::CameraCommand::FORWARD == 2 &&
              vision::msg::CameraCommand::GOAL == 3);

class MainNode : public rclcpp::Node
{
public:
    MainNode() : Node("main_node")
    {
        // ==================== P2P 전체 실행 흐름 ====================
        // 1) 시작 자세로 이동한 뒤 몸 동작/카메라 명령 구독을 연다.
        // 2) action(동작 종류)을 JSON 경로로 바꾸고, action_id(명령 식별자)로 중복을 확인한다.
        // 3) PrepareProgram()으로 JSON 전체를 검증한 뒤 즉시 시작하거나 대기열에 저장한다.
        // 4) 명령 접수가 성공하면 같은 ID로 ACK(접수됨)를 Vision에 보낸다.
        // 5) MotionLoop()가 몸/카메라 목표 tick을 합쳐 20 ms 간격으로 한 패킷씩 전송한다.
        // 6) 지원되는 LINE 동작은 종료 직전에 READY(다음 명령 접수 가능)를 보낸다.
        // 7) 마지막 이동/유지가 끝나면 DONE(재생 완료)을 보내고 대기 동작을 이어 시작한다.
        // DONE은 재생 시간표 완료를 뜻하며, encoder로 실제 도착을 재확인한 결과는 아니다.

        // motions/와 missions/를 포함한 패키지 share 루트다.
        declare_parameter<std::string>(
            "p2p_asset_directory",
            ament_index_cpp::get_package_share_directory("forward_walk"));
        asset_directory_ = get_parameter("p2p_asset_directory").as_string();

        // 회전 JSON 한 번의 예상 회전각과 최대 반복 횟수(도 단위 → 반복 횟수 변환용).
        turn_step_deg_ = declare_parameter<double>("turn_step_deg", 15.0);
        max_turn_repetitions_ =
            declare_parameter<int>("max_turn_repetitions", 12);
        // 던지기(SHOOT) 때 허리 관절에 적용할 각도 제한, tick 환산값, 회전 부호.
        waist_motor_id_ = declare_parameter<int>("waist_motor_id", 12);
        waist_max_yaw_deg_ =
            declare_parameter<double>("waist_max_yaw_deg", 30.0);
        waist_ticks_per_revolution_ =
            declare_parameter<double>("waist_ticks_per_revolution", 4096.0);
        waist_yaw_direction_ =
            declare_parameter<int>("waist_yaw_direction", 1);
        waist_restore_after_shoot_ =
            declare_parameter<bool>("waist_restore_after_shoot", true);
        // 프로그램 시작 시 WALK_MODE의 마지막 자세로 이동하는 시간(초).
        startup_pose_duration_sec_ =
            declare_parameter<double>("startup_pose_duration_sec", 3.0);

        // READY를 보낼 종료 전 여유 시간(초). 실제 통신 지연을 예측하는 값은 아니다.
        ready_lead_sec_ = declare_parameter<double>("ready_lead_sec", 0.25);
        // 카메라 yaw=좌우 회전, pitch=상하 회전. 자세 값은 각도가 아니라 encoder tick이다.
        camera_config_.yaw_id = declare_parameter<int>("camera_yaw_motor_id", 21);
        camera_config_.pitch_id = declare_parameter<int>("camera_pitch_motor_id", 22);
        camera_config_.forward.yaw = declare_parameter<int>("camera_forward_yaw_tick", 2077);
        camera_config_.forward.pitch = declare_parameter<int>("camera_forward_pitch_tick", 1537);
        camera_config_.down.yaw = declare_parameter<int>("camera_down_yaw_tick", 2036);
        camera_config_.down.pitch = declare_parameter<int>("camera_down_pitch_tick", 2013);
        camera_config_.goal.yaw = declare_parameter<int>("camera_goal_yaw_tick", 2054);
        camera_config_.goal.pitch = declare_parameter<int>("camera_goal_pitch_tick", 966);
        // move_sec 동안 보간 이동하고 settle_sec 동안 기다린 뒤 카메라 DONE을 보낸다.
        camera_config_.move_sec = declare_parameter<double>("camera_move_duration_sec", 0.5);
        camera_config_.settle_sec = declare_parameter<double>("camera_settle_sec", 0.2);
        camera_config_.Validate();
        camera_motion_ = std::make_unique<motion_executor::CameraMotion>(camera_config_);

        if (!std::isfinite(ready_lead_sec_) || ready_lead_sec_ < 0.0 ||
            !std::isfinite(startup_pose_duration_sec_) || turn_step_deg_ <= 0.0 || max_turn_repetitions_ <= 0 ||
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

        // Vision 프로토콜의 ACK/READY/DONE을 기존 action_status에 전송한다.
        action_status_pub_ = create_publisher<vision::msg::CommandStatus>(
            "/jandi_vision/action_status", 10);
        // Camera status는 별도 ID 공간과 토픽을 사용하므로 publisher도 분리한다.
        camera_status_pub_ = create_publisher<vision::msg::CommandStatus>(
            "/jandi_vision/camera_status", 10);

        // 1 ms마다 상태를 확인하고, 실제 전송은 MotionLoop의 공통 마감 시각으로 50 Hz 제한한다.
        // polling과 전송 주기를 같게 두면 timer jitter로 한 주기를 더 쉴 수 있다.
        motion_loop_timer_ = create_wall_timer(
            1ms, std::bind(&MainNode::MotionLoop, this));
        motion_loop_timer_->cancel();

        // WALK_MODE 마지막 자세에 도착하기 전에는 명령 topic을 구독하지 않는다.
        StartStartupPose();
    }

private:
    // 전류/추정 토크 로그에서 사용하는 다리 관절 수(오른쪽 6개 + 왼쪽 6개).
    static constexpr int kLegJointCount = 12;
    // 모터 전류(mA)를 추정 토크(N·m)로 바꾸는 계수. 직접 측정한 토크는 아니다.
    static constexpr double kTorquePerMilliAmp =
        1.0 / (TORQUE_TO_VALUE_MX_106 * 3.36);
    static constexpr const char* kTorqueLogDir = "/tmp/forward_walk_logs";

    // 시작 자세 재생이 끝난 뒤 호출한다. 이후 들어오는 Vision 명령을 콜백으로 받는다.
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

    // WALK_MODE JSON의 마지막 키프레임만 선택해 시작 자세로 이동한다.
    void StartStartupPose()
    {
        const std::filesystem::path asset_root(asset_directory_);  // 모션 파일 기준 폴더
        const std::filesystem::path path =
            asset_root /
            ProgramFileForAction(vision::msg::ActionCommand::WALK_MODE);

        P2PMotionPlayer::StartOptions options;  // 시작 자세에만 적용할 재생 옵션
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
        last_motion_tick_at_ = std::chrono::steady_clock::now();
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
            return "motions/초기자세_다소곳_.json";

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

    // 요청 각도를 회전 JSON 반복 횟수로 반올림하고 0~최대 반복 횟수로 제한한다.
    int CalculateTurnCount(int target_yaw_deg) const
    {
        const int requested = static_cast<int>(std::lround(  // 제한 전 계산된 반복 횟수
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

    // 몸 동작 명령 수신: 중복/접수 가능 여부 확인 → 모션 준비 → ACK → 실행 또는 대기.
    void ActionCommandCallback(
        const vision::msg::ActionCommand::SharedPtr message)
    {
        RCLCPP_INFO(get_logger(),
                    "Action received: id=%llu mission=%u action=%u target_yaw_deg=%d",
                    static_cast<unsigned long long>(message->action_id),
                    static_cast<unsigned int>(message->mission),
                    static_cast<unsigned int>(message->action),
                    static_cast<int>(message->target_yaw_deg));

        // admission은 접수 판정이다. Start=즉시 실행, Queue=다음 동작으로 저장,
        // *Duplicate=같은 ID 재전송, Reject=접수 불가. Evaluate 자체는 상태를 바꾸지 않는다.
        const auto admission = action_transactions_.Evaluate(
            message->action_id, message->mission, message->action);
        using motion_executor::Admission;
        if (admission == Admission::ActiveDuplicate || admission == Admission::QueuedDuplicate) {
            PublishActionStatus(message->action_id, vision::msg::CommandStatus::ACK);
            return;
        }
        if (admission == Admission::CompletedDuplicate) {
            PublishActionStatus(message->action_id, vision::msg::CommandStatus::DONE);
            return;
        }
        if (admission == Admission::Reject) {
            RCLCPP_WARN(get_logger(), "Action rejected (zero ID, busy before READY, queue full, or non-LINE): id=%llu",
                        static_cast<unsigned long long>(message->action_id));
            return;
        }

        const std::string filename = ProgramFileForAction(message->action);  // 동작 종류의 JSON 경로
        if (filename.empty()) {
            RCLCPP_ERROR(get_logger(), "No JSON mapping for action=%u",
                         static_cast<unsigned int>(message->action));
            return;
        }
        // requested는 Vision 요청 각도, effective는 반복/각도 제한을 반영한 계획상 각도다.
        // effective는 센서로 측정한 실제 회전각이 아니다. 회전 action이 좌우 방향을 정한다.
        const int requested_target_yaw_deg =
            static_cast<int>(message->target_yaw_deg);
        int effective_target_yaw_deg = 0;
        P2PMotionPlayer::StartOptions start_options;  // 반복, 관절 offset, 뒤에 붙일 모션 등의 옵션

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
                action_transactions_.completed.Remember(message->action_id);
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
            const double clamped_yaw_deg = std::clamp(  // 허리 허용 범위 안으로 제한한 각도
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

            // 허리 목표 각도를 JSON의 허리 목표 tick에 더할 offset으로 환산한다.
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

        std::filesystem::path relative_path(filename);  // 패키지 폴더 기준 상대 경로
        // 기존 매핑처럼 파일명만 있으면 motion으로 간주한다. 신규 매핑은
        // motions/... 또는 missions/... 경로를 명시한다.
        if (!relative_path.has_parent_path()) {
            relative_path = std::filesystem::path("motions") / relative_path;
        }
        const std::filesystem::path asset_root(asset_directory_);
        if (IsTurnAndStepAction(message->action) && start_options.repeat_count == 0) {
            start_options.repeat_count = 1;
            relative_path = "missions/연속걷기.json";
        }
        if (IsTurnAndStepAction(message->action) && relative_path == std::filesystem::path(filename)) {
            start_options.trailing_program_paths.push_back(
                (asset_root / "missions" / "연속걷기.json").string());
            RCLCPP_INFO(
                get_logger(),
                "Turn-and-step sequence: turn_count=%zu then continuous walk",
                start_options.repeat_count);
        }
        const std::filesystem::path path = asset_root / relative_path;  // 실제로 읽을 전체 경로
        try {
            // program은 검증을 마친 모션 데이터다. 준비만 할 때는 현재 모션/모터를 바꾸지 않는다.
            auto program = p2p_player_->PrepareProgram(
                path.string(), (asset_root / "motions").string(), start_options);
            if (admission == Admission::Queue) {
                // 대기열은 한 칸이다. 검증된 데이터까지 저장한 뒤 ACK하므로 나중에 다시 읽지 않는다.
                queued_action_.emplace(QueuedAction{message->action_id, message->mission,
                    message->action, effective_target_yaw_deg, std::move(program)});
                action_transactions_.queued_id = message->action_id;
                PublishActionStatus(message->action_id, vision::msg::CommandStatus::ACK);
                return;
            }
            if (!p2p_player_->StartPrepared(program)) {
                RCLCPP_ERROR(get_logger(), "P2P start failed: %s", p2p_player_->Error().c_str());
                return;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Action preparation failed, no ACK: %s", e.what());
            return;
        }
        // 실제 시작 성공 뒤 활성 ID를 등록한다. 실패한 명령에는 ACK를 보내지 않는다.
        action_transactions_.Activate(message->action_id, message->mission, message->action);
        current_action_ = message->action;
        active_target_yaw_deg_ = effective_target_yaw_deg;
        motion_in_progress_ = true;
        PublishActionStatus(message->action_id, vision::msg::CommandStatus::ACK);
        EnsureMotionLoop();
    }

    // 몸 동작과 별도인 카메라 command_id에 ACK/DONE을 대응시켜 전송한다.
    void PublishCameraStatus(std::uint64_t id, std::uint8_t status)
    {
        vision::msg::CommandStatus message;
        message.command_id = id;
        message.status = status;
        camera_status_pub_->publish(message);
    }

    // DOWN/FORWARD/GOAL 요청을 머리 관절 모션으로 만든다. 몸 동작과 동시에 실행할 수 있다.
    void CameraCommandCallback(const vision::msg::CameraCommand::SharedPtr message)
    {
        try { camera_config_.Target(message->request); }
        catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Camera request rejected: %s", e.what());
            return;
        }
        using motion_executor::Admission;
        // 카메라는 별도 중복 기록을 사용하며, 이동 중인 새 ID를 대기열에 넣거나 선점하지 않는다.
        const auto admission = camera_transactions_.Evaluate(message->command_id);
        if (admission == Admission::ActiveDuplicate) {
            PublishCameraStatus(message->command_id, vision::msg::CommandStatus::ACK);
            return;
        }
        if (admission == Admission::CompletedDuplicate) {
            PublishCameraStatus(message->command_id, vision::msg::CommandStatus::DONE);
            return;
        }
        if (admission != Admission::Start) {
            RCLCPP_WARN(get_logger(), "Camera command rejected (zero ID or busy): id=%llu",
                        static_cast<unsigned long long>(message->command_id));
            return;
        }
        try {
            // 시작 자세 이동 때 저장한 마지막 전송 목표를 카메라의 출발점으로 사용한다.
            // 추가 encoder 읽기 통신을 하지 않는다. 이 값은 실제 측정 위치가 아니라 목표 위치다.
            camera_motion_->Start(message->request, last_sent_positions_,
                                  std::chrono::steady_clock::now());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Camera start failed, no ACK: %s", e.what());
            return;
        }
        camera_transactions_.active_id = message->command_id;
        PublishCameraStatus(message->command_id, vision::msg::CommandStatus::ACK);
        EnsureMotionLoop();
    }

    // 유휴 상태에서 꺼져 있던 타이머를 다시 켠다. 이미 동작 중이면 주기를 재시작하지 않는다.
    void EnsureMotionLoop()
    {
        if (motion_loop_timer_->is_canceled()) {
            last_motion_tick_at_ = std::chrono::steady_clock::now();
            motion_loop_timer_->reset();
        }
    }

    // 준비/전송 오류 시 몸·카메라 재생과 대기열을 정리한다. 완료되지 않았으므로 DONE은 보내지 않는다.
    // 이 함수는 재생 상태를 중단하는 것이며 모터 Torque OFF를 수행하지 않는다.
    void AbortPlayback(const std::string& reason)
    {
        // 상태를 지우기 전에 실패한 활성/예약/카메라 ID와 시작 자세 여부를 남긴다.
        RCLCPP_ERROR(get_logger(),
            "Playback stopped without DONE: action_id=%llu queued_id=%llu camera_id=%llu action=%u startup=%d reason=%s",
            static_cast<unsigned long long>(action_transactions_.active_id),
            static_cast<unsigned long long>(action_transactions_.queued_id),
            static_cast<unsigned long long>(camera_transactions_.active_id),
            static_cast<unsigned int>(current_action_),
            static_cast<int>(startup_pose_in_progress_), reason.c_str());
        p2p_player_->Stop();
        camera_motion_->Abort(last_sent_positions_);
        action_transactions_.Abort();
        queued_action_.reset();
        camera_transactions_.active_id = 0;
        startup_pose_in_progress_ = false;
        motion_in_progress_ = false;
        current_action_ = 0;
        active_target_yaw_deg_ = 0;
        motion_loop_timer_->cancel();
    }

    // 몸 목표 계산 → 카메라 목표 계산 → 한 패킷으로 합쳐 전송 → READY/DONE 처리 순서.
    void MotionLoop()
    {
        if (!startup_pose_in_progress_ && !motion_in_progress_ && !camera_motion_->Active()) return;
        const auto now = std::chrono::steady_clock::now();  // 시스템 시각 변경의 영향을 받지 않는 주기 기준
        // gap_ms는 타이머 콜백 사이의 간격이다. 모터 패킷 사이의 간격을 직접 측정한 값은 아니다.
        const double gap_ms = std::chrono::duration<double, std::milli>(now - last_motion_tick_at_).count();
        last_motion_tick_at_ = now;
        if (gap_ms > 40.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Motion update delayed by %.1f ms; preserving interpolation steps", gap_ms);
        }
        // next_write_at_ 이전에는 전송하지 않는다. 몸/카메라가 공통으로 20 ms 간격을 지킨다.
        if (now < next_write_at_) return;
        // body_command: player가 이번 호출에서 만든 전체 모터의 목표 tick(ID → tick).
        // 비어 있으면 이번 호출에서는 몸 목표 패킷을 새로 만들지 않았다는 뜻이다.
        P2PMotionPlayer::RawPositions body_command;
        // body_result: 몸 모션의 재생 상태이며, 목표 위치 데이터인 body_command와 구분한다.
        // kIdle=재생 없음, kRunning=이동/유지 진행 중(새 목표가 없을 수도 있음),
        // kFinished=마지막 이동/유지 완료, kError=오류로 중단해야 함.
        // Update()는 목표를 계산하고, 실제 모터 전송은 아래 SyncWriteRawPositions()가 담당한다.
        const auto body_result = (startup_pose_in_progress_ || motion_in_progress_)
            ? p2p_player_->Update(&body_command, now) : P2PMotionPlayer::UpdateResult::kIdle;
        if (body_result == P2PMotionPlayer::UpdateResult::kError) {
            AbortPlayback(p2p_player_->Error());
            return;
        }
        const bool startup_completed = startup_pose_in_progress_ &&
            body_result == P2PMotionPlayer::UpdateResult::kFinished;
        // 시작 자세가 끝나면 FORWARD를 전송·유지한 뒤 구독을 연다.
        // 이후 몸 JSON에 머리 tick이 있어도 Merge()가 이 기본 자세로 덮어쓴다.
        if (startup_completed) camera_motion_->HoldForward();
        // camera_result: Idle=재생 없음, Running=다음 단계/안정화 대기,
        // Command=새 카메라 목표 계산됨, Finished=이동 및 안정화 시간 완료.
        const auto camera_result = camera_motion_->Update(now);
        // camera_command는 위치 데이터가 아니라 "카메라 목표를 전송해야 하는가"라는 bool이다.
        // 시작 자세 완료 시 FORWARD 유지 목표도 이 경로로 한 번 전송한다.
        const bool camera_command = startup_completed ||
            camera_result == motion_executor::CameraMotion::Result::Command;
        if (!body_command.empty() || camera_command) {
            // command는 최종 전송용 전체 목표다. 몸 목표가 없으면 마지막 전체 목표를 복사해
            // 몸 자세를 유지하면서 카메라 관절만 바꾼다.
            auto command = body_command.empty() ? last_sent_positions_ : body_command;
            try {
                // 시작 시 FORWARD 또는 이후 카메라 명령의 yaw/pitch 목표를 덮어쓴다.
                // 카메라 DONE 뒤에도 그 자세를 유지한다.
                camera_motion_->Merge(command);
                // write_started/written_at: SDK 전송 호출의 시작/반환 시각. 실제 관절 도착 시각은 아니다.
                const auto write_started = std::chrono::steady_clock::now();
                // 몸과 카메라 Goal Position을 쓰는 단일 전송 지점.
                dxl_->SyncWriteRawPositions(command);
                const auto written_at = std::chrono::steady_clock::now();
                // 전송 호출 성공 뒤에만 캐시를 갱신한다. 다음 카메라 이동/몸 유지의 기준이 된다.
                last_sent_positions_ = std::move(command);
                // 전송 시작 + 20 ms와 전송 종료 중 늦은 시각까지 기다려 몰아서 전송하지 않는다.
                next_write_at_ = std::max(write_started + motion_executor::kPeriod, written_at);
                // 실제 전송 시각을 각 재생기에 알려 다음 보간 단계의 마감 시각을 맞춘다.
                if (!body_command.empty()) p2p_player_->OnCommandWritten(write_started, written_at);
                if (camera_command) camera_motion_->OnCommandWritten(write_started, written_at);
            } catch (const std::exception& e) {
                AbortPlayback(e.what());
                return;
            }
        }
        // READY는 종료 예정 시간에 가까워진 지원 LINE 동작에서 한 번만 보낸다.
        // READY 뒤에는 다음 지원 LINE 명령 한 개를 검증·저장하고 미리 ACK할 수 있다.
        if (motion_in_progress_ && !action_transactions_.ready_sent &&
            p2p_player_->IsNearCompletion(ready_lead_sec_, std::chrono::steady_clock::now()) &&
            action_transactions_.MarkReady()) {
            PublishActionStatus(action_transactions_.active_id, vision::msg::CommandStatus::READY);
        }
        // 카메라 이동과 settle 대기가 끝나면 카메라 ID만 완료 처리한다. 몸 동작은 계속될 수 있다.
        if (camera_result == motion_executor::CameraMotion::Result::Finished) {
            PublishCameraStatus(camera_transactions_.active_id, vision::msg::CommandStatus::DONE);
            camera_transactions_.CompleteActive();
        }
        if (body_result == P2PMotionPlayer::UpdateResult::kFinished) {
            if (startup_pose_in_progress_) {
                startup_pose_in_progress_ = false;
                current_action_ = 0;
                CreateCommandSubscriptions();
            } else {
                const auto completed_id = action_transactions_.active_id;  // 방금 끝난 몸 명령의 ID
                // A의 DONE을 먼저 보낸 뒤, 이미 ACK한 대기 동작 B를 활성 동작으로 올린다.
                PublishActionStatus(completed_id, vision::msg::CommandStatus::DONE);
                action_transactions_.CompleteActive();
                motion_in_progress_ = false;
                current_action_ = 0;
                active_target_yaw_deg_ = 0;
                if (queued_action_) {
                    auto queued = std::move(*queued_action_);  // 대기열에서 꺼낸 다음 명령과 검증된 모션
                    queued_action_.reset();
                    action_transactions_.Activate(queued.id, queued.mission, queued.action);
                    current_action_ = queued.action;
                    active_target_yaw_deg_ = queued.effective_target_yaw_deg;
                    if (!p2p_player_->StartPrepared(queued.program)) {
                        AbortPlayback("Queued action start failed: " + p2p_player_->Error());
                        return;
                    }
                    motion_in_progress_ = true;
                    // B는 대기열 저장 시 ACK했으므로 여기서는 다시 ACK하지 않는다.
                    RCLCPP_INFO(get_logger(), "Queued action started automatically: id=%llu",
                                static_cast<unsigned long long>(queued.id));
                }
            }
        }
        // 시작 자세/몸/카메라가 모두 유휴 상태일 때만 타이머를 끈다.
        if (!startup_pose_in_progress_ && !motion_in_progress_ && !camera_motion_->Active()) {
            motion_loop_timer_->cancel();
        }
    }

    // 로그를 저장할 폴더와 CSV 파일을 준비하는 것
    void OpenTorqueLogFile()
    {
        namespace fs = std::filesystem;
        fs::create_directories(kTorqueLogDir);
        const auto now = std::chrono::system_clock::now();  // 파일명에 넣을 실제 날짜/시각
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

    // 전류를 읽고 추정 토크와 함께 CSV에 기록하는 보조 함수.
    // 현재 main.cpp에는 이 함수의 호출이 없으므로 CSV는 헤더만 생성되고 주기 기록은 하지 않는다.
    void LogLegJointTorqueStep()
    {
        if (!torque_log_stream_.is_open()) return;
        const Eigen::VectorXd current_mA = dxl_->GetCurrent();  // 모터별 읽은 전류, 단위 mA
        torque_log_stream_ << get_clock()->now().seconds() << "," << current_action_;
        for (int i = 0; i < kLegJointCount; ++i)
            torque_log_stream_ << "," << current_mA[i] * kTorquePerMilliAmp;
        for (int i = 0; i < kLegJointCount; ++i)
            torque_log_stream_ << "," << current_mA[i];
        torque_log_stream_ << "\n";
    }

    // 이름 끝의 '_'는 콜백 호출 사이에도 이 노드가 계속 보관하는 멤버 변수라는 표기다.
    std::shared_ptr<Dxl> dxl_;                         // 모터 통신 객체: encoder 읽기와 목표 tick 쓰기
    std::unique_ptr<P2PMotionPlayer> p2p_player_;      // 몸 JSON의 키프레임·보간·유지 시간을 관리하는 재생기
    rclcpp::Publisher<vision::msg::CommandStatus>::SharedPtr action_status_pub_;  // 몸 ACK/READY/DONE 송신
    rclcpp::Publisher<vision::msg::CommandStatus>::SharedPtr camera_status_pub_;  // 카메라 ACK/DONE 송신
    rclcpp::Subscription<vision::msg::ActionCommand>::SharedPtr action_cmd_sub_; // 몸 명령 수신 연결
    rclcpp::Subscription<vision::msg::CameraCommand>::SharedPtr camera_cmd_sub_; // 카메라 명령 수신 연결
    rclcpp::TimerBase::SharedPtr motion_loop_timer_;   // 1 ms마다 MotionLoop를 호출하는 ROS 타이머
    std::chrono::steady_clock::time_point last_motion_tick_at_{}; // 직전 유효 콜백 시각: 지연 경고 계산용
    std::ofstream torque_log_stream_;                 // 전류/추정 토크 CSV 출력 스트림
    std::string asset_directory_;                     // motions/와 missions/가 들어 있는 기준 폴더
    double turn_step_deg_{15.0};                      // 회전 JSON 1회당 계획상 회전각(도)
    int max_turn_repetitions_{12};                     // 한 회전 요청의 최대 JSON 반복 횟수
    int waist_motor_id_{12};                          // SHOOT 각도 offset을 적용할 허리 모터 ID
    double waist_max_yaw_deg_{30.0};                   // 허리 회전 요청 제한: -이 값 ~ +이 값(도)
    double waist_ticks_per_revolution_{4096.0};        // 허리 360도 회전에 대응하는 tick 수
    int waist_yaw_direction_{1};                       // 허리 각도 → tick 변환 부호(+1 또는 -1)
    bool waist_restore_after_shoot_{true};            // 던지기 후 적용한 허리 offset을 복원할지 여부
    double startup_pose_duration_sec_{3.0};           // 시작 자세 이동 시간(초)
    bool startup_pose_in_progress_{false};            // 명령 구독을 열기 전 시작 자세를 재생 중인지
    bool motion_in_progress_{false};                  // Vision에서 받은 몸 모션을 실행 중인지(시작 자세와 구분)
    // READY 뒤 미리 ACK한 다음 몸 동작 한 개. 모터를 아직 움직이지 않고 준비 데이터를 보관한다.
    struct QueuedAction {
        std::uint64_t id;                             // 원래 ActionCommand.action_id: ACK/DONE 대응용
        std::uint8_t mission;                         // 미션 종류: LINE 등 접수 조건 판단용
        std::uint16_t action;                         // 동작 종류: JSON 매핑과 READY 지원 여부 판단용
        int effective_target_yaw_deg;                 // 제한/반복 계산 뒤 계획한 각도(도)
        P2PMotionPlayer::PreparedProgram program;     // 접수 시 검증해 둔 모션 데이터
    };
    // 몸의 활성 ID, 대기 ID, READY 전송 여부, 최근 완료 ID 32개를 관리한다.
    // ID 중복이면 재실행하지 않고 진행 중 명령에는 ACK, 완료 명령에는 DONE을 다시 보낸다.
    motion_executor::ActionTransactions action_transactions_;
    std::optional<QueuedAction> queued_action_;        // 비어 있으면 다음 동작 없음, 값이 있으면 1개 대기
    motion_executor::CameraTransactions camera_transactions_; // 몸과 독립인 카메라 활성/완료 ID 기록
    motion_executor::CameraConfig camera_config_;     // 카메라 모터 ID, 세 자세 tick, 이동/안정화 시간
    std::unique_ptr<motion_executor::CameraMotion> camera_motion_; // 머리 보간 및 완료 후 자세 유지 상태
    // 마지막으로 전송 호출에 성공한 전체 목표 tick. 실제 encoder 측정값과 구분해야 한다.
    P2PMotionPlayer::RawPositions last_sent_positions_;
    std::chrono::steady_clock::time_point next_write_at_{}; // 몸/카메라 공통으로 다음 전송이 허용되는 시각
    double ready_lead_sec_{0.25};                      // 종료 몇 초 전부터 READY를 허용할지
    std::uint16_t current_action_{0};                  // 현재 동작 종류(명령 고유 ID 아님), 0은 유휴 상태
    int active_target_yaw_deg_{0};                     // 현재 계획한 각도 저장용; 이 값 자체가 모터를 제어하진 않음
};

int main(int argc, char** argv)
{
    // ROS 초기화 → 노드 생성/시작 자세 재생 → 콜백·타이머 처리 → 종료 순서.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}

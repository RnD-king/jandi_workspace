#include "p2p_motion_player.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
int ParseMotorId(const std::string& text)
{
    std::size_t consumed = 0;
    const int id = std::stoi(text, &consumed);
    if (consumed != text.size() ||
        std::find(Dxl::MotorIds().begin(), Dxl::MotorIds().end(), id) == Dxl::MotorIds().end()) {
        throw std::runtime_error("invalid motor ID: " + text);
    }
    return id;
}
}

P2PMotionPlayer::P2PMotionPlayer(Dxl* dxl) : dxl_(dxl) {}

// motion JSON을 읽어서 실행에 필요한 keyframe으로 변환한다.
P2PMotionPlayer::Motion P2PMotionPlayer::LoadMotion(
    const std::filesystem::path& json_path) const
{
    using boost::property_tree::ptree;
    ptree root;
    boost::property_tree::read_json(json_path.string(), root);

    Motion motion;
    motion.name = root.get<std::string>("name", json_path.string());

    // Optional editor PD metadata is validated, but the verified P850/I0/D0
    // initialization is preserved. Preparation never applies hardware gains.
    if (const auto gains = root.get_child_optional("pd_gains")) {
        if (!gains->data().empty()) throw std::runtime_error("pd_gains must be an object");
        std::unordered_map<int, bool> seen;
        for (const auto& entry : *gains) {
            const int id = ParseMotorId(entry.first);
            if (!seen.emplace(id, true).second || entry.second.empty()) {
                throw std::runtime_error("invalid/duplicate PD gain motor ID");
            }
            const int p = entry.second.get<int>("p_gain");
            const int d = entry.second.get<int>("d_gain");
            if (p < 0 || p > 16383 || d < 0 || d > 16383) {
                throw std::runtime_error("PD gain outside 0..16383");
            }
        }
    }

    const auto frames = root.get_child_optional("keyframes");
    if (!frames) {
        throw std::runtime_error(
            "motion JSON has no keyframes array: " + json_path.string());
    }

    for (const auto& frame_entry : *frames) {
        if (!frame_entry.first.empty()) throw std::runtime_error("keyframes must be an array");
        const ptree& source = frame_entry.second;
        Keyframe frame;
        frame.name = source.get<std::string>("name", "unnamed");
        // get(path, default) also defaults on conversion failure. Only absent
        // fields may use defaults; malformed present fields must reject before ACK.
        const auto timing = [&](const std::string& key, double fallback) {
            return source.get_child_optional(key) ? source.get<double>(key) : fallback;
        };
        frame.duration_sec = timing("duration_sec", 1.0);
        frame.hold_sec = timing("hold_sec", 0.0);
        frame.max_speed_deg_s = timing("max_speed_deg_s", 30.0);

        const std::string kind =
            source.get<std::string>("interpolation", "smoothstep");
        if (kind == "linear") frame.interpolation = Interpolation::kLinear;
        else if (kind == "smoothstep") frame.interpolation = Interpolation::kSmoothStep;
        else if (kind == "minimum_jerk") frame.interpolation = Interpolation::kMinimumJerk;
        else throw std::runtime_error("unknown interpolation: " + kind);

        if (!std::isfinite(frame.duration_sec) || !std::isfinite(frame.hold_sec) ||
            !std::isfinite(frame.max_speed_deg_s) || frame.duration_sec <= 0.0 ||
            frame.duration_sec * 50.0 >= static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
            frame.hold_sec < 0.0 || frame.max_speed_deg_s <= 0.0) {
            throw std::runtime_error("invalid timing/speed in keyframe: " + frame.name);
        }

        const auto positions = source.get_child_optional("positions");
        if (!positions) throw std::runtime_error("keyframe has no positions: " + frame.name);
        for (const auto& position : *positions) {
            const int id = ParseMotorId(position.first);
            const auto tick = position.second.get_value<int32_t>();
            if (tick < 0 || tick > 4095 || !frame.positions.emplace(id, tick).second) {
                throw std::runtime_error("invalid/duplicate position for ID " + position.first);
            }
        }
        for (const int id : Dxl::MotorIds()) {
            if (frame.positions.find(id) == frame.positions.end()) {
                throw std::runtime_error("keyframe '" + frame.name +
                                         "' is missing motor ID " + std::to_string(id));
            }
        }
        motion.keyframes.push_back(std::move(frame));
    }
    if (motion.keyframes.empty()) {
        throw std::runtime_error(
            "motion JSON contains no keyframes: " + json_path.string());
    }
    return motion;
}

// program JSON이 motion이면 하나만, mission이면 참조하는 motion 전체를
// 미리 파싱한다. 그래야 중간 파일 오타를 로봇이 움직인 뒤에 발견하지 않는다.
void P2PMotionPlayer::LoadProgram(
    const std::filesystem::path& json_path,
    const std::filesystem::path& motion_directory)
{
    using boost::property_tree::ptree;
    ptree root;
    boost::property_tree::read_json(json_path.string(), root);

    motions_.clear();
    if (root.get_child_optional("keyframes")) {
        motions_.push_back(LoadMotion(json_path));
        return;
    }

    const auto mission_motions = root.get_child_optional("motions");
    if (!mission_motions) {
        throw std::runtime_error(
            "JSON is neither a motion nor a mission: " + json_path.string());
    }
    for (const auto& entry : *mission_motions) {
        if (!entry.first.empty()) throw std::runtime_error("mission motions must be an array");
        const std::string filename = entry.second.get<std::string>("filename");
        const std::filesystem::path motion_path =
            motion_directory / std::filesystem::path(filename);
        motions_.push_back(LoadMotion(motion_path));
    }
    if (motions_.empty()) {
        throw std::runtime_error(
            "mission JSON contains no motions: " + json_path.string());
    }
}

void P2PMotionPlayer::AppendProgram(
    const std::filesystem::path& json_path,
    const std::filesystem::path& motion_directory)
{
    std::vector<Motion> preceding_motions = std::move(motions_);
    LoadProgram(json_path, motion_directory);
    preceding_motions.insert(preceding_motions.end(),
                             motions_.begin(), motions_.end());
    motions_ = std::move(preceding_motions);
}

void P2PMotionPlayer::ApplyStartOptions(
    const StartOptions& options,
    const std::filesystem::path& motion_directory)
{
    if (options.repeat_count == 0) {
        throw std::runtime_error("repeat_count must be greater than zero");
    }
    if (options.duration_override_sec &&
        (!std::isfinite(*options.duration_override_sec) || *options.duration_override_sec <= 0.0 ||
         *options.duration_override_sec * 50.0 >= static_cast<double>(std::numeric_limits<std::size_t>::max()))) {
        throw std::runtime_error(
            "duration_override_sec must be greater than zero");
    }

    if (options.final_keyframe_only) {
        if (motions_.empty() || motions_.back().keyframes.empty()) {
            throw std::runtime_error(
                "final keyframe requested from an empty program");
        }
        Motion final_pose;
        final_pose.name = motions_.back().name + "_final_pose";
        final_pose.keyframes.push_back(motions_.back().keyframes.back());
        motions_.clear();
        motions_.push_back(std::move(final_pose));
    }

    const std::vector<Motion> original_motions = motions_;
    motions_.clear();
    for (std::size_t repeat = 0; repeat < options.repeat_count; ++repeat) {
        motions_.insert(motions_.end(), original_motions.begin(),
                        original_motions.end());
    }

    if (!options.position_offsets.empty()) {
        if (motions_.empty()) {
            throw std::runtime_error(
                "position offsets require at least one primary motion");
        }

        Keyframe restore_frame;
        if (options.restore_offsets) {
            restore_frame = motions_.back().keyframes.back();
            restore_frame.name += "_offset_restore";
            restore_frame.duration_sec = 0.5;
            restore_frame.hold_sec = 0.0;
            restore_frame.max_speed_deg_s = 30.0;
            restore_frame.interpolation = Interpolation::kSmoothStep;
        }

        for (auto& motion : motions_) {
            for (auto& frame : motion.keyframes) {
                for (const auto& offset : options.position_offsets) {
                    const auto position = frame.positions.find(offset.first);
                    if (position == frame.positions.end()) {
                        throw std::runtime_error(
                            "keyframe '" + frame.name +
                            "' is missing offset motor ID " +
                            std::to_string(offset.first));
                    }
                    const std::int64_t target =
                        static_cast<std::int64_t>(position->second) +
                        offset.second;
                    if (target < 0 || target > 4095) {
                        throw std::runtime_error(
                            "offset target is outside 0..4095 for motor ID " +
                            std::to_string(offset.first));
                    }
                    position->second = static_cast<int32_t>(target);
                }
            }
        }

        if (options.restore_offsets) {
            motions_.back().keyframes.push_back(std::move(restore_frame));
        }
    }

    for (const auto& trailing_path : options.trailing_program_paths) {
        AppendProgram(trailing_path, motion_directory);
    }
    if (options.duration_override_sec) {
        for (auto& motion : motions_) {
            for (auto& frame : motion.keyframes) {
                frame.duration_sec = *options.duration_override_sec;
            }
        }
    }
    if (motions_.empty()) {
        throw std::runtime_error("program contains no motions after options");
    }
}

void P2PMotionPlayer::SelectMotion(std::size_t index)
{
    motion_index_ = index;
    motion_name_ = motions_.at(index).name;
    keyframes_ = motions_.at(index).keyframes;
    keyframe_index_ = 0;
}

bool P2PMotionPlayer::Start(
    const std::string& json_path,
    const std::string& motion_directory,
    const StartOptions& options)
{
    try {
        return StartPrepared(PrepareProgram(json_path, motion_directory, options));
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
}

P2PMotionPlayer::PreparedProgram P2PMotionPlayer::PrepareProgram(
    const std::string& json_path, const std::string& motion_directory,
    const StartOptions& options) const
{
    P2PMotionPlayer parser(nullptr);
    parser.LoadProgram(json_path, motion_directory);
    parser.ApplyStartOptions(options, motion_directory);
    PreparedProgram prepared;
    prepared.motions = std::move(parser.motions_);
    return prepared;
}

bool P2PMotionPlayer::StartPrepared(const PreparedProgram& program)
{
    try {
        if (dxl_ == nullptr) throw std::runtime_error("Dxl is null");
        const auto initial_pose = dxl_->GetRawPositions();
        return StartPrepared(program, initial_pose, Clock::now());
    } catch (const std::exception& e) {
        Stop();
        error_ = e.what();
        return false;
    }
}

bool P2PMotionPlayer::StartPrepared(const PreparedProgram& program,
                                   const RawPositions& initial_pose,
                                   Clock::time_point now)
{
    Stop();
    error_.clear();
    try {
        if (program.motions.empty()) throw std::runtime_error("empty prepared program");
        for (const int id : Dxl::MotorIds()) {
            if (initial_pose.find(id) == initial_pose.end()) {
                throw std::runtime_error("initial pose missing motor ID " + std::to_string(id));
            }
        }
        motions_ = program.motions;
        SelectMotion(0);
        start_positions_ = initial_pose;
        playing_ = true;
        BeginKeyframe(now);
        return true;
    } catch (const std::exception& e) {
        error_ = e.what();
        Stop();
        return false;
    }
}

void P2PMotionPlayer::BeginKeyframe(Clock::time_point now)
{
    const Keyframe& frame = keyframes_.at(keyframe_index_);
    double max_delta_deg = 0.0;

    // 모든 관절 중 이동량이 가장 큰 관절을 기준으로 최소 필요시간을 구한다.
    // MX 계열 1회전=4096 tick이므로 tick 차이를 degree로 변환한다.
    for (const auto& motor : start_positions_) {
        const int32_t target = frame.positions.at(motor.first);
        const double delta_deg =
            std::abs(static_cast<double>(target) - motor.second) * 360.0 / 4096.0;
        max_delta_deg = std::max(max_delta_deg, delta_deg);
    }
    // 사용자가 지정한 시간보다 속도 제한에 필요한 시간이 길면 자동으로 늘린다.
    effective_duration_sec_ = std::max(
        std::max(0.02, frame.duration_sec),
        max_delta_deg / std::max(1.0, frame.max_speed_deg_s));
    // Python round()와 같은 ties-to-even 반올림을 사용한다.
    total_steps_ = static_cast<std::size_t>(std::max(
        2.0, std::nearbyint(effective_duration_sec_ * 50.0)));
    completed_steps_ = 0;
    next_command_at_ = now;
    holding_ = false;
}

double P2PMotionPlayer::Interpolate(Interpolation kind, double x)
{
    // 보간 입력은 반드시 0~1로 제한한다.
    x = std::clamp(x, 0.0, 1.0);
    if (kind == Interpolation::kLinear) return x;
    if (kind == Interpolation::kMinimumJerk) {
        return 10.0*x*x*x - 15.0*x*x*x*x + 6.0*x*x*x*x*x;
    }
    return x*x*(3.0 - 2.0*x);
}

P2PMotionPlayer::UpdateResult P2PMotionPlayer::Update(RawPositions* output,
                                                    Clock::time_point now)
{
    if (output == nullptr) throw std::invalid_argument("P2P output is null");
    output->clear();
    if (!playing_) return UpdateResult::kIdle;
    try {
        const Keyframe& frame = keyframes_.at(keyframe_index_);

        // 사이트는 마지막 목표 전송 뒤에도 한 주기를 쉰 다음 hold를 시작한다.
        // hold/구간 전환을 마친 호출에서 다음 구간의 첫 단계를 바로 전송한다.
        if (completed_steps_ == total_steps_) {
            if (now < next_command_at_) return UpdateResult::kRunning;
            if (frame.hold_sec > 0.0) {
                if (!holding_) {
                    holding_ = true;
                    phase_started_at_ = now;
                    return UpdateResult::kRunning;
                }
                const double held_sec =
                    std::chrono::duration<double>(now - phase_started_at_).count();
                if (held_sec < frame.hold_sec) return UpdateResult::kRunning;
            }
            start_positions_ = frame.positions;
            ++keyframe_index_;
            if (keyframe_index_ >= keyframes_.size()) {
                if (motion_index_ + 1 >= motions_.size()) {
                    Stop();
                    return UpdateResult::kFinished;
                }
                SelectMotion(motion_index_ + 1);
            }
            BeginKeyframe(now);
        }

        if (now < next_command_at_) return UpdateResult::kRunning;
        const Keyframe& active_frame = keyframes_.at(keyframe_index_);
        const double progress =
            static_cast<double>(completed_steps_ + 1) / total_steps_;
        const double alpha = Interpolate(active_frame.interpolation, progress);
        RawPositions command;

        // 각 motor ID별로 start와 target 사이의 이번 timer tick 목표를 만든다.
        for (const auto& motor : start_positions_) {
            const int32_t target = active_frame.positions.at(motor.first);
            const double value = static_cast<double>(motor.second) +
                alpha * (static_cast<double>(target) - motor.second);
            command.emplace(motor.first, static_cast<int32_t>(std::nearbyint(value)));
        }
        // MainNode merges the camera override and owns the only packet writer.
        *output = std::move(command);
        ++completed_steps_;
        // 다음 deadline은 이번 전송 기준이다. 지연 후 과거 deadline을 따라잡으려
        // 여러 단계를 압축하지 않는다. 통신 자체가 20 ms 이상 걸리면 sleep은 없다.
        next_command_at_ = now + kCommandPeriod;
        return UpdateResult::kRunning;
    } catch (const std::exception& e) {
        error_ = e.what();
        playing_ = false;
        return UpdateResult::kError;
    }
}

void P2PMotionPlayer::OnCommandWritten(Clock::time_point write_started,
                                      Clock::time_point written_at)
{
    next_command_at_ = std::max(write_started + kCommandPeriod, written_at);
}

double P2PMotionPlayer::RemainingNominalSec(Clock::time_point now) const
{
    if (!playing_) return 0.0;
    // READY only applies to the final frame of the entire program.
    if (motion_index_ + 1 != motions_.size() || keyframe_index_ + 1 != keyframes_.size()) {
        return std::numeric_limits<double>::infinity();
    }
    const auto& frame = keyframes_.at(keyframe_index_);
    if (holding_) {
        return std::max(0.0, frame.hold_sec -
            std::chrono::duration<double>(now - phase_started_at_).count());
    }
    // Move-phase readiness depends only on successfully generated steps, never
    // elapsed wall time. The final packet's wait is still honored by Update().
    return (total_steps_ - completed_steps_) * 0.02 + frame.hold_sec;
}

bool P2PMotionPlayer::IsNearCompletion(double lead_sec, Clock::time_point now) const
{
    return playing_ && std::isfinite(lead_sec) && lead_sec >= 0.0 &&
           RemainingNominalSec(now) <= lead_sec;
}

void P2PMotionPlayer::Stop()
{
    // 통신 포트나 Torque 상태는 건드리지 않고 player 내부 상태만 초기화한다.
    playing_ = false;
    holding_ = false;
    motion_index_ = 0;
    keyframe_index_ = 0;
    effective_duration_sec_ = 0.0;
    total_steps_ = 0;
    completed_steps_ = 0;
}

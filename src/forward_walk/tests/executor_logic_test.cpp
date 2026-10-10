#include "p2p_motion_player.hpp"
#include "motion_executor_state.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <unistd.h>

using namespace motion_executor;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void Rejects(const std::function<void()>& operation) {
    bool rejected = false;
    try { operation(); } catch (const std::exception&) { rejected = true; }
    Require(rejected, "expected rejection");
}
RawPositions InitialPose() {
    RawPositions pose;
    for (int id : Dxl::MotorIds()) pose.emplace(id, 2048);
    return pose;
}

int main() {
    const fs::path assets(FORWARD_WALK_SOURCE_DIR);
    const fs::path temporary = fs::temp_directory_path() / ("jandi_executor_test_" + std::to_string(getpid()));
    fs::create_directories(temporary);
    int groups = 0;
    auto pass = [&](const std::string& label) { ++groups; std::cout << "PASS " << label << '\n'; };
    try {
        P2PMotionPlayer player(nullptr); // No Dxl instance/constructor or device access.
        const auto left = assets / "motions/보행_좌회전전진_20도_최종_초기X.json";
        const auto right = assets / "motions/보행_우회전전진_20도_최종_초기X.json";
        const auto five = assets / "missions/연속걷기.json";
        for (const auto& path : {left, right, five}) {
            const auto program = player.PrepareProgram(path.string(), (assets / "motions").string(), {});
            Require(player.StartPrepared(program, InitialPose(), Clock::time_point{}), player.Error());
            player.Stop();
        }
        pass("11/12/13 actual motion/mission preparation");

        const auto fixture = temporary / "fixture.json";
        auto write_fixture = [&](double duration, double hold) {
            std::ofstream stream(fixture);
            stream << "{\"name\":\"test\",\"keyframes\":[{\"name\":\"final\",\"duration_sec\":"
                   << duration << ",\"hold_sec\":" << hold
                   << ",\"max_speed_deg_s\":4000,\"interpolation\":\"linear\",\"positions\":{";
            bool first = true;
            for (int id : Dxl::MotorIds()) {
                if (!first) stream << ',';
                first = false;
                stream << '"' << id << "\":2148";
            }
            stream << "}}]}";
        };
        write_fixture(0.5, 0);
        const auto valid = player.PrepareProgram(fixture.string(), temporary.string(), {});
        const auto t0 = Clock::time_point{} + 1s;
        Require(player.StartPrepared(valid, InitialPose(), t0), player.Error());
        P2PMotionPlayer control(nullptr);
        Require(control.StartPrepared(valid, InitialPose(), t0), control.Error());
        RawPositions actual, expected;
        player.Update(&actual, t0);
        control.Update(&expected, t0);
        const auto before = player.RemainingNominalSec(t0);
        player.PrepareProgram(left.string(), (assets / "motions").string(), {});
        Require(player.IsPlaying() && player.MotionName() == "test" &&
                player.RemainingNominalSec(t0) == before, "prepare mutated active state");
        player.Update(&actual, t0 + 90ms);
        control.Update(&expected, t0 + 90ms);
        Require(actual == expected && actual.at(0) == 2056, "delayed update skipped a step");
        Require(!player.IsNearCompletion(0.25, t0 + 90ms), "wall delay caused early READY");
        pass("non-destructive preparation + 90ms delay preserves fixed step");

        using boost::property_tree::ptree;
        ptree root;
        boost::property_tree::read_json(fixture.string(), root);
        auto reject_root = [&](ptree bad) {
            boost::property_tree::write_json(fixture.string(), bad);
            Rejects([&] { player.PrepareProgram(fixture.string(), temporary.string(), {}); });
            Require(player.IsPlaying(), "invalid preparation stopped active playback");
        };
        auto bad = root;
        bad.get_child("keyframes").front().second.get_child("positions").erase("21"); reject_root(bad);
        for (const auto& value : {"-1", "4096", "2000.5", "not_a_tick"}) {
            bad = root; bad.get_child("keyframes").front().second.put("positions.21", value); reject_root(bad);
        }
        for (const auto& key : {"duration_sec", "max_speed_deg_s"}) {
            bad = root; bad.get_child("keyframes").front().second.put(key, 0); reject_root(bad);
            bad = root; bad.get_child("keyframes").front().second.put(key, "nan"); reject_root(bad);
        }
        bad = root; bad.get_child("keyframes").front().second.put("hold_sec", -1); reject_root(bad);
        bad = root; bad.get_child("keyframes").front().second.put("duration_sec", "1e308"); reject_root(bad);
        bad = root; bad.get_child("keyframes").front().second.put("interpolation", "invalid"); reject_root(bad);
        bad = root; bad.put("pd_gains.21.p_gain", 16384); bad.put("pd_gains.21.d_gain", 0); reject_root(bad);
        bad = root; bad.put("pd_gains.21.p_gain", "850.5"); bad.put("pd_gains.21.d_gain", 0); reject_root(bad);
        bad = root; bad.put("pd_gains.21.p_gain", 850); reject_root(bad);
        bad = root; bad.put("pd_gains", "invalid"); reject_root(bad);
        std::ofstream(fixture) << "invalid JSON";
        Rejects([&] { player.PrepareProgram(fixture.string(), temporary.string(), {}); });
        std::ofstream(fixture) << "{\"motions\":[{\"filename\":\"missing.json\"}]}";
        Rejects([&] { player.PrepareProgram(fixture.string(), temporary.string(), {}); });
        write_fixture(0.5, 0);
        P2PMotionPlayer::StartOptions options;
        options.trailing_program_paths = {(temporary / "missing.json").string()};
        Rejects([&] { player.PrepareProgram(fixture.string(), temporary.string(), options); });
        options = {}; options.position_offsets = {{21, 4095}};
        Rejects([&] { player.PrepareProgram(fixture.string(), temporary.string(), options); });
        pass("invalid JSON, motor, timing, interpolation, PD, references, offsets rejection");

        Require(player.StartPrepared(valid, InitialPose(), t0), player.Error());
        for (int step = 0; step < 14; ++step) player.Update(&actual, t0 + step * 20ms);
        Require(player.IsNearCompletion(0.25, t0 + 260ms), "READY not based on remaining steps");
        Require(std::abs(player.RemainingNominalSec(t0 + 260ms) - 0.22) < 1e-8, "wrong final-step remaining time");
        write_fixture(0.04, 0.2);
        const auto hold_program = player.PrepareProgram(fixture.string(), temporary.string(), {});
        Require(player.StartPrepared(hold_program, InitialPose(), t0), player.Error());
        player.Update(&actual, t0); player.Update(&actual, t0 + 20ms);
        Require(player.IsPlaying(), "finished before final period/hold");
        player.Update(&actual, t0 + 40ms);
        Require(std::abs(player.RemainingNominalSec(t0 + 140ms) - 0.1) < 1e-8, "hold remaining calculation");
        Require(player.Update(&actual, t0 + 240ms) == P2PMotionPlayer::UpdateResult::kFinished, "hold DONE timing");
        auto repeated = P2PMotionPlayer::StartOptions{}; repeated.repeat_count = 2;
        auto multiple = player.PrepareProgram(fixture.string(), temporary.string(), repeated);
        Require(player.StartPrepared(multiple, InitialPose(), t0), player.Error());
        Require(!player.IsNearCompletion(100, t0), "READY allowed before final motion");
        pass("READY remaining steps/hold and final-program-only condition");

        ActionTransactions actions;
        Require(actions.Evaluate(0,1,11) == Admission::Reject, "zero action ID accepted");
        Require(actions.Evaluate(100,1,11) == Admission::Start, "idle did not accept");
        actions.Activate(100,1,11);
        Require(actions.Evaluate(100,1,11) == Admission::ActiveDuplicate, "active duplicate");
        Require(actions.Evaluate(101,1,12) == Admission::Reject, "accepted before READY");
        Require(actions.MarkReady() && !actions.MarkReady(), "READY not exactly once");
        Require(actions.Evaluate(101,1,12) == Admission::Queue, "LINE queue rejected");
        Require(actions.Evaluate(101,2,12) == Admission::Reject, "non-LINE mission queued");
        Require(actions.Evaluate(101,1,18) == Admission::Reject, "18 queued");
        actions.queued_id = 101;
        Require(actions.Evaluate(101,1,12) == Admission::QueuedDuplicate, "queued duplicate");
        Require(actions.Evaluate(102,1,13) == Admission::Reject, "queue overflow accepted");
        actions.CompleteActive();
        Require(actions.completed.Contains(100) && actions.queued_id == 101, "completion lost queue");
        actions.Activate(101,1,12);
        Require(!actions.ready_sent && actions.queued_id == 0, "promotion did not reset state");
        actions.CompleteActive();
        Require(actions.Evaluate(100,1,11) == Admission::CompletedDuplicate, "older completed ID executed again");
        for (int action : {10,18,19,20}) {
            actions.Activate(200,1,action);
            Require(!actions.MarkReady(), "READY on unsupported action");
        }
        CompletedHistory history;
        for (uint64_t id=1; id<=33; ++id) history.Remember(id);
        Require(!history.Contains(1) && history.Contains(2) && history.Contains(33), "history is not bounded at 32");
        pass("action duplicates, exactly-once READY, one-slot queue, promotion, bounded history");

        CameraTransactions cameras;
        Require(cameras.Evaluate(0) == Admission::Reject, "zero camera ID accepted");
        cameras.active_id = 100;
        Require(cameras.Evaluate(100) == Admission::ActiveDuplicate &&
                cameras.Evaluate(101) == Admission::Reject, "camera active semantics");
        cameras.CompleteActive();
        Require(cameras.Evaluate(100) == Admission::CompletedDuplicate &&
                cameras.Evaluate(101) == Admission::Start, "camera completed semantics");
        actions.Abort();
        Require(actions.Evaluate(101,1,11) == Admission::CompletedDuplicate &&
                cameras.Evaluate(101) == Admission::Start, "action/camera histories not independent");
        pass("camera duplicate/busy semantics and independent ID space");

        CameraConfig config;
        auto overflowing_config = config;
        overflowing_config.move_sec = 1e308;
        Rejects([&] { overflowing_config.Validate(); });
        Require(config.Target(1).yaw==2036 && config.Target(1).pitch==2013 &&
                config.Target(2).yaw==2077 && config.Target(2).pitch==1537 &&
                config.Target(3).yaw==2054 && config.Target(3).pitch==966, "calibration mapping");
        Rejects([&] { config.Target(0); });
        CameraMotion camera(config);
        auto base = InitialPose();
        auto unmodified = base;
        camera.Merge(base);
        Require(base == unmodified, "camera changed startup before first command");
        camera.HoldForward();
        camera.Merge(base);
        Require(!camera.Active() && camera.OverrideEnabled(), "FORWARD hold started an active camera command");
        for (int id : Dxl::MotorIds()) {
            Require(base.at(id)==(id==21?2077:id==22?1537:2048), "FORWARD hold changed body or missed calibrated head pose");
        }
        auto startup_body = InitialPose();
        camera.Merge(startup_body);
        Require(startup_body.at(21)==2077 && startup_body.at(22)==1537, "body overwrote startup FORWARD hold");
        Require(camera.Update(t0)==CameraMotion::Result::Idle, "FORWARD hold generated fake camera activity");
        camera.Start(3, base, t0);
        Require(camera.Update(t0) == CameraMotion::Result::Command, "camera did not start");
        camera.Merge(base);
        Require(camera.Update(t0+90ms) == CameraMotion::Result::Command, "delayed camera step");
        camera.Merge(base);
        const double x=2.0/25.0, alpha=x*x*(3-2*x);
        Require(base.at(22)==static_cast<int32_t>(std::nearbyint(1537+alpha*(966-1537))), "camera skipped interpolation step from FORWARD");
        auto now=t0+90ms;
        for (int i=2;i<25;++i) { now+=20ms; camera.Update(now); }
        camera.Merge(base);
        Require(base.at(21)==2054 && base.at(22)==966, "camera target not reached");
        Require(camera.Update(now+20ms)==CameraMotion::Result::Running, "settle skipped");
        Require(camera.Update(now+220ms)==CameraMotion::Result::Finished, "camera DONE missing after settle");
        auto next_body = InitialPose(); camera.Merge(next_body);
        for (int id : Dxl::MotorIds()) {
            Require(next_body.at(id)==(id==21?2054:id==22?966:2048), "override changed body motor or was cleared after DONE");
        }
        Require(!camera.Active() && camera.OverrideEnabled(), "persistent override lost");
        pass("camera calibration, delayed interpolation, settle/DONE, persistent full-packet merge");

        fs::remove_all(temporary);
        std::cout << "All " << groups << " test groups passed without hardware.\n";
        return 0;
    } catch (const std::exception& e) {
        fs::remove_all(temporary);
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}

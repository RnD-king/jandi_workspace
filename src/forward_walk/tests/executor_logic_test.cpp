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

void TestWalkInterActionHold() {
    WalkInterActionHold hold;
    const auto t=Clock::time_point{}+1s;
    hold.Complete(13,t);
    Require(!hold.Waiting(t), "zero gap changed legacy execution");
    hold.SetDuration(.6);
    for(uint16_t action : {10,11,12,13,21,22,23,24,25,26,27,28,31}) {
        hold.Complete(action,t);
        Require(hold.Waiting(t+599ms), "WALK hold ended early");
        Require(!hold.Waiting(t+600ms), "WALK hold extended past configured duration");
    }
    hold.Complete(3,t);
    Require(!hold.Waiting(t), "WALK hold applied to FINE motion");
    Rejects([&]{hold.SetDuration(-.1);});
    ActionTransactions state;
    state.Activate(1,1,13);state.MarkReady();state.queued_id=2;state.CompleteActive();
    Require(state.Evaluate(2,1,13)==Admission::QueuedDuplicate, "delayed B lost idempotency");
    Require(state.Evaluate(3,1,13)==Admission::Reject, "third command bypassed delayed one-slot queue");
}

int main() {
    TestWalkInterActionHold();
    const fs::path temporary = fs::temp_directory_path() / ("jandi_executor_test_" + std::to_string(getpid()));
    fs::create_directories(temporary);
    int groups = 0;
    auto pass = [&](const std::string& label) { ++groups; std::cout << "PASS " << label << '\n'; };
    try {
        P2PMotionPlayer player(nullptr); // No Dxl instance/constructor or device access.
        // Real operator assets are checked separately by motion_assets_check.
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
        boost::property_tree::ptree queued;
        boost::property_tree::read_json(fixture.string(), queued);
        queued.put("name", "queued");
        queued.get_child("keyframes").front().second.put("positions.0", 2248);
        const auto queued_path=temporary/"queued.json";
        boost::property_tree::write_json(queued_path.string(),queued);
        player.PrepareProgram(queued_path.string(), temporary.string(), {});
        Require(player.IsPlaying() && player.MotionName() == "test" &&
                player.RemainingNominalSec(t0) == before, "prepare mutated active state");
        player.Update(&actual, t0 + 90ms);
        control.Update(&expected, t0 + 90ms);
        Require(actual == expected && actual.at(0) == 2056, "delayed update skipped a step");
        Require(!player.IsNearCompletion(0.25, t0 + 90ms), "wall delay caused early READY");
        pass("non-destructive preparation + 90ms delay preserves fixed step");

        // Verify mission ordering/repetition with fixed test assets rather than
        // freezing the operator's live mission contents or timing.
        const auto sequence_path=temporary/"sequence.json";
        std::ofstream(sequence_path) <<
            "{\"motions\":[{\"filename\":\"fixture.json\"},"
            "{\"filename\":\"queued.json\"},{\"filename\":\"fixture.json\"}]}";
        P2PMotionPlayer sequence(nullptr);
        const auto sequence_program=sequence.PrepareProgram(sequence_path.string(),temporary.string(),{});
        Require(sequence.StartPrepared(sequence_program,InitialPose(),t0),sequence.Error());
        for(int step=0;step<75;++step) {
            RawPositions output;
            sequence.Update(&output,t0+step*20ms);
            if(step==24 || step==74) Require(output.at(0)==2148,"mission skipped/reordered repeated motion");
            if(step==49) Require(output.at(0)==2248,"mission did not follow referenced motion order");
        }
        Require(sequence.Update(&actual,t0+1500ms)==P2PMotionPlayer::UpdateResult::kFinished,
                "mission length did not reflect its references");
        pass("fixed mission reference order, repeated references and completion");

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

        uint8_t health_data[83]={0};
        health_data[0]=0; health_data[6]=0x25; health_data[80]=118; health_data[82]=82;
        const auto health=MotorHealth::Decode(15,health_data,0x80);
        Require(health.valid && health.voltage_raw==118 && health.temperature==82,
                "health alert packet must retain valid voltage and temperature");
        Require(health.Reason().find("토크 OFF")!=std::string::npos &&
                health.Reason().find("과열")!=std::string::npos &&
                health.Reason().find("과부하")!=std::string::npos &&
                health.Reason().find("전압")!=std::string::npos,"hardware bits not decoded together");
        Require(!MotorHealth::Decode(15,health_data,4).valid,"packet access error treated as valid data");
        MotorHealth missing; missing.communication_error="timeout";
        Require(missing.Reason().find("원인 미확정")!=std::string::npos,"missing reply must not claim power failure");
        health_data[0]=1; health_data[6]=0;
        Require(MotorHealth::Decode(15,health_data,0).Reason().empty(),"healthy motor reported fault");
        pass("motor health bit decoding, Alert data, unknown cause and normal status");

        LineArmConfig arm;
        arm.Validate();
        Require(arm.enabled && arm.Targets().size()==2 && arm.Targets().count(13)==0,
                "measured defaults must target only arms 15/16");
        Require(arm.Targets().at(15)==1379 && arm.Targets().at(16)==2765,
                "15-based measured symmetric targets");
        Require(arm.Targets().at(15)-arm.reference_15 == arm.reference_16-arm.Targets().at(16),
                "arm displacement must be symmetric about measured baselines");
        arm.angle_deg = 45.0;
        arm.Validate();
        Require(arm.Targets().at(15)==1583 && arm.Targets().at(16)==2561, "45-degree arm tick conversion");
        arm.direction_15 = -1; arm.direction_16 = 1;
        Require(arm.Targets().at(15)==559 && arm.Targets().at(16)==3585, "opposite arm direction conversion");
        arm.direction_15 = 0;
        Rejects([&] { arm.Validate(); });
        arm.direction_15 = 1; arm.reference_15 = 4000;
        Rejects([&] { arm.Validate(); });

        write_fixture(0.5, 0);
        P2PMotionPlayer::StartOptions arm_options;
        arm_options.repeat_count = 2;
        arm_options.trailing_program_paths = {fixture.string()};
        const auto arm_baseline = player.PrepareProgram(fixture.string(), temporary.string(), arm_options);
        arm_options.position_overrides = {{15,1379},{16,2765}};
        const auto arm_program = player.PrepareProgram(fixture.string(), temporary.string(), arm_options);
        P2PMotionPlayer arm_player(nullptr), baseline_player(nullptr);
        Require(arm_player.StartPrepared(arm_program, InitialPose(), t0), arm_player.Error());
        Require(baseline_player.StartPrepared(arm_baseline, InitialPose(), t0), baseline_player.Error());
        for (int step=0;step<100;++step) {
            RawPositions arm_output, original_output;
            const auto at=t0+step*20ms;
            const auto arm_result=arm_player.Update(&arm_output,at);
            Require(arm_result==baseline_player.Update(&original_output,at), "arm override changed fixture replay schedule");
            if (step==0) Require(arm_output.at(15)<2048 && arm_output.at(15)>1379, "arm jumped to target without P2P interpolation");
            for (const auto& target:original_output) {
                if (target.first!=15 && target.first!=16)
                    Require(arm_output.at(target.first)==target.second,"arm override changed another motor");
            }
            if (step==24 || step==49 || step==74)
                Require(arm_output.at(15)==1379 && arm_output.at(16)==2765,"arm override missed repeated/trailing motion");
        }
        arm_options.position_overrides={{99,2048}};
        Rejects([&] { player.PrepareProgram(fixture.string(), temporary.string(), arm_options); });
        arm_options.position_overrides={{15,4096}};
        Rejects([&] { player.PrepareProgram(fixture.string(), temporary.string(), arm_options); });
        pass("arm direction/range validation, smooth selective overrides, repeats/trailing, other motors preserved");

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

        for(uint16_t code=21;code<=25;++code) {
            for(uint8_t mission:{1,2,3,4}) {
                ActionTransactions fresh;
                fresh.Activate(6000,mission,code);
                Require(fresh.MarkReady() && !fresh.MarkReady(),"new walk action READY not exactly once");
                Require(fresh.Evaluate(6001,1,21)==Admission::Queue,
                        "new walk READY admission rejected a cross-mission successor");
            }
            ActionTransactions old_to_new;old_to_new.Activate(7000,1,11);old_to_new.MarkReady();
            Require(old_to_new.Evaluate(7001,1,code)==Admission::Queue,"old walk could not queue new action");
        }
        Require(!IsReadyWalkAction(20)&&!IsReadyWalkAction(26)&&!IsReadyWalkAction(27),"READY extended outside valid new action range");
        pass("new action READY and current-action queue policy");
        const auto held=InitialPose();
        auto transition_start=held;transition_start.at(15)=1379;transition_start.at(16)=2765;
        const auto transition=P2PMotionPlayer::PreparePoseTransition(held,{{15,1071},{16,3073}},1.0);
        P2PMotionPlayer arm_transition(nullptr);
        Require(arm_transition.StartPrepared(transition,transition_start,t0),arm_transition.Error());
        RawPositions transition_output;
        for(int step=0;step<50;++step) {
            arm_transition.Update(&transition_output,t0+step*20ms);
            Require(transition_output.at(15)<=transition_start.at(15)&&transition_output.at(15)>=1071,
                    "arm restoration was not monotonic");
            for(const auto& value:held) if(value.first!=15&&value.first!=16)
                Require(transition_output.at(value.first)==value.second,"arm transition changed held body");
            if(step==0) Require(transition_output.at(15)>1071,"arm restore jumped without interpolation");
            transition_start=transition_output;
        }
        Require(transition_output.at(15)==1071&&transition_output.at(16)==3073,"arm transition missed restore targets");
        Rejects([&]{P2PMotionPlayer::PreparePoseTransition(held,{{15,4096}},1.0);});
        Rejects([&]{P2PMotionPlayer::PreparePoseTransition(held,{{99,2048}},1.0);});
        Rejects([&]{P2PMotionPlayer::PreparePoseTransition(held,{},0);});
        pass("arm transition P2P interpolation, held body and target validation");

        ActionTransactions actions;
        Require(actions.Evaluate(0,1,11) == Admission::Reject, "zero action ID accepted");
        Require(actions.Evaluate(100,1,11) == Admission::Start, "idle did not accept");
        actions.Activate(100,1,11);
        Require(actions.Evaluate(100,1,11) == Admission::ActiveDuplicate, "active duplicate");
        Require(actions.Evaluate(101,1,12) == Admission::Reject, "accepted before READY");
        Require(actions.MarkReady() && !actions.MarkReady(), "READY not exactly once");
        Require(actions.Evaluate(101,1,12) == Admission::Queue, "LINE queue rejected");
        Require(actions.Evaluate(101,2,12) == Admission::Queue, "cross-mission successor rejected");
        Require(actions.Evaluate(101,1,18) == Admission::Queue, "non-READY successor rejected");
        actions.queued_id = 101;
        Require(actions.Evaluate(101,1,12) == Admission::QueuedDuplicate, "queued duplicate");
        Require(actions.Evaluate(102,1,13) == Admission::Reject, "queue overflow accepted");
        actions.CompleteActive();
        Require(actions.completed.Contains(100) && actions.queued_id == 101, "completion lost queue");
        actions.Activate(101,1,12);
        Require(!actions.ready_sent && actions.queued_id == 0, "promotion did not reset state");
        actions.CompleteActive();
        Require(actions.Evaluate(100,1,11) == Admission::CompletedDuplicate, "older completed ID executed again");
        for (int action : {10,18,19,20,26,27}) {
            actions.Activate(200,1,action);
            Require(!actions.MarkReady(), "READY on unsupported action");
        }
        // Queue admission depends on the current action's READY, not B's mission or number.
        for (uint8_t mission : {uint8_t{1}, uint8_t{2}, uint8_t{3}, uint8_t{4}}) {
            for (uint16_t action : {uint16_t{11}, uint16_t{12}, uint16_t{13}, uint16_t{32}, uint16_t{33}, uint16_t{34}}) {
                actions.Activate(300 + mission*10 + action, mission, action);
                Require(actions.MarkReady() && !actions.MarkReady(),
                        "READY missing or duplicated for long walk");
                Require(actions.Evaluate(900, 1, 13)==Admission::Queue,
                        "READY admission unexpectedly restricted by mission");
                actions.CompleteActive();
            }
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
        Require(camera.Update(now+20ms)==CameraMotion::Result::Verify && camera.Active(), "camera completed without encoder verification");
        Require(!camera.ConfirmArrival({2054,1200}) && camera.Active(),"out-of-range pitch was accepted");
        Require(!camera.ConfirmArrival({1800,966}),"out-of-range yaw was accepted");
        Require(!camera.ConfirmArrival({2054,-1}),"invalid encoder value was accepted");
        Require(camera.ConfirmArrival({2259,1062}),"target-value ten-percent tolerance was not inclusive");
        Require(camera.Update(now+220ms)==CameraMotion::Result::Idle, "completed camera emitted completion again");
        auto next_body = InitialPose(); camera.Merge(next_body);
        for (int id : Dxl::MotorIds()) {
            Require(next_body.at(id)==(id==21?2054:id==22?966:2048), "override changed body motor or was cleared after DONE");
        }
        Require(!camera.Active() && camera.OverrideEnabled(), "persistent override lost");
        pass("camera calibration, encoder-gated completion, target-relative tolerance and persistent merge");

        fs::remove_all(temporary);
        std::cout << "All " << groups << " test groups passed without hardware.\n";
        return 0;
    } catch (const std::exception& e) {
        fs::remove_all(temporary);
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}

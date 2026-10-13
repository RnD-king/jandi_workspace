// Pre-include MainNode's dependencies before exposing its private state to this
// test translation unit. Production main.cpp and its visibility are unchanged.
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
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
#include <iostream>
#include <thread>
#include <functional>
#include <unistd.h>
#define private public
#define main forward_walk_production_entry
#include "../src/main.cpp"
#undef main
#undef private

namespace fs = std::filesystem;
using Status = vision::msg::CommandStatus;
using Events = std::vector<std::pair<uint64_t,uint8_t>>;
void Check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int Count(const Events& events, uint64_t id, uint8_t status) {
    return std::count(events.begin(),events.end(),std::make_pair(id,status));
}
void WriteTripleMission(const fs::path& path, const std::string& filename, int count=3) {
    std::ofstream stream(path);
    stream << "{\"motions\":[";
    for (int i=0; i<count; ++i) {
        if (i) stream << ',';
        stream << "{\"filename\":\"" << filename << "\"}";
    }
    stream << "]}";
}
void WriteMotion(const fs::path& path, bool startup=false, int gripper=2048) {
    std::ofstream stream(path);
    stream << "{\"keyframes\":[";
    for(int frame=0;frame<(startup?1:2);++frame) {
        if(frame)stream<<',';
        stream << "{\"duration_sec\":0.2,\"hold_sec\":" << (frame==1?0.08:0.0)
               << ",\"max_speed_deg_s\":4000,\"interpolation\":\"smoothstep\",\"positions\":{";
        bool first=true;
        for(int id:Dxl::MotorIds()) {
            if(!first)stream<<',';first=false;
            stream<<'"'<<id<<"\":"<<(id==19?gripper:(id==0&&!startup?2148+frame*100:2048));
        }
        stream<<"}}";
    }
    stream<<"]}";
}

int main() {
    const auto directory=fs::temp_directory_path()/("jandi_main_test_"+std::to_string(getpid()));
    fs::create_directories(directory/"motions");
    fs::create_directories(directory/"missions");
    const auto startup=directory/"motions/보행_왼발만앞에.json";
    const auto left=directory/"motions/보행_좌회전전진_20도_최종_초기X.json";
    const auto right=directory/"motions/보행_우회전전진_20도_최종_초기X.json";
    WriteMotion(startup,true);WriteMotion(left);WriteMotion(right);
    WriteMotion(directory/"motions/공_재그립.json",false,1968);
    WriteTripleMission(directory/"missions/직진_좌회전_3번.json", left.filename().string());
    WriteTripleMission(directory/"missions/직진_우회전_3번.json", right.filename().string());
    WriteTripleMission(directory/"missions/직진_우회전_2번.json", right.filename().string(),2);
    const std::vector<std::string> options={"executor_integration_test","--ros-args",
        "-p","p2p_asset_directory:="+directory.string(),
        "-p","motor_health_log_directory:="+(directory/"log").string(),
        "-p","startup_pose_duration_sec:=0.04",
        "-p","mission_arm_transition_duration_sec:=0.04",
        // 실제 측정한 15번 기준 대칭 팔 목표를 기본 설정으로 검증한다.
        "-p","camera_move_duration_sec:=0.04",
        "-p","camera_settle_sec:=0.02"};
    std::vector<const char*> args;for(const auto& s:options)args.push_back(s.c_str());
    try {
        rclcpp::init(static_cast<int>(args.size()),args.data());
        auto observer=std::make_shared<rclcpp::Node>("executor_test_observer");
        Events actions,cameras;
        auto action_sub=observer->create_subscription<Status>("/jandi_vision/action_status",100,
            [&](Status::SharedPtr m){actions.emplace_back(m->command_id,m->status);});
        auto camera_sub=observer->create_subscription<Status>("/jandi_vision/camera_status",100,
            [&](Status::SharedPtr m){cameras.emplace_back(m->command_id,m->status);});
        auto node=std::make_shared<MainNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(observer);executor.add_node(node);
        auto pump=[&](const std::function<bool()>& done, double seconds=3.0) {
            const auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
            while(!done()&&std::chrono::steady_clock::now()<until) {
                executor.spin_some();std::this_thread::sleep_for(1ms);
            }
            Check(done(),"integration wait timed out");
        };
        auto flush=[&] {
            const auto until=std::chrono::steady_clock::now()+30ms;
            pump([&]{return std::chrono::steady_clock::now()>=until;});
        };
        auto action=[&](uint64_t id,uint16_t code,uint8_t mission=1) {
            auto m=std::make_shared<vision::msg::ActionCommand>();
            m->action_id=id;m->action=code;m->mission=mission;
            node->ActionCommandCallback(m);
        };
        auto camera=[&](uint64_t id,uint8_t request) {
            auto m=std::make_shared<vision::msg::CameraCommand>();
            m->command_id=id;m->request=request;node->CameraCommandCallback(m);
        };
        pump([&]{return !node->startup_pose_in_progress_;});flush();
        Check(node->action_cmd_sub_&&node->camera_cmd_sub_,"subscriptions missing after startup");
        Check(actions.empty()&&cameras.empty()&&node->camera_motion_->OverrideEnabled()&&
              !node->camera_motion_->Active(),"startup status or FORWARD hold state mismatch");
        for(std::size_t i=0;i+1<Dxl::packets.size();++i)
            Check(Dxl::packets[i].at(21)==2048&&Dxl::packets[i].at(22)==2048,"camera override began before startup completed");
        Check(Dxl::current.at(21)==2077&&Dxl::current.at(22)==1537,"startup FORWARD goal was not transmitted");
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"startup missed symmetric arm targets");
        Check(Dxl::packets.front().at(15)>1379&&Dxl::packets.front().at(15)<2048&&
              Dxl::packets.front().at(16)>2048&&Dxl::packets.front().at(16)<2765,
              "startup arms jumped instead of using P2P interpolation");
        for(int id:Dxl::MotorIds())if(id!=15&&id!=16&&id!=19&&id!=21&&id!=22)
            Check(Dxl::current.at(id)==2048,"startup FORWARD hold changed body");
        Check(Dxl::current.at(19)==1968,"startup missed RECATCH gripper target");
        // A normal body JSON contains different head ticks; the automatic FORWARD hold wins.
        action(9001,11);
        // 실제 모터 없이 오류/통신 단절/복구와 활성 동작의 로그를 검증한다.
        const auto health_motor=std::find(Dxl::ids.begin(),Dxl::ids.end(),15);
        const auto poll_health=[&] {
            const auto write_at=node->next_write_at_;
            node->next_write_at_=std::chrono::steady_clock::now()+20ms;
            node->health_index_=std::distance(Dxl::ids.begin(),health_motor);
            node->MotorHealthLoop();
            node->next_write_at_=write_at;
        };
        uint8_t health_bytes[83]={0}; health_bytes[6]=4; health_bytes[80]=120; health_bytes[82]=81;
        Dxl::health[15]=MotorHealth::Decode(15,health_bytes,0x80);
        const auto writes_before_health=Dxl::packets.size();
        poll_health();
        Check(node->health_states_.at(15).reason.find("과열")!=std::string::npos,"overheat fault not recorded");
        std::ifstream health_stream(node->health_log_path_);
        Check(fs::path(node->health_log_path_).parent_path()==directory/"log" && health_stream.is_open(),
              "health logger did not create and write the configured log directory");
        const std::string health_text((std::istreambuf_iterator<char>(health_stream)),{});
        Check(health_text.find("motor_id=15")!=std::string::npos &&
              health_text.find("mission=1 action=11 action_id=9001")!=std::string::npos,
              "health log lost motor or active action context");
        Dxl::health[15]=MotorHealth{}; Dxl::health[15].communication_error="injected timeout";
        poll_health();
        Check(node->health_states_.at(15).failures==1,"communication failure not counted");
        Dxl::health.erase(15); poll_health();
        Check(node->health_states_.at(15).reason.empty(),"recovered motor remains marked failed");
        Check(Dxl::packets.size()==writes_before_health && node->motion_in_progress_,
              "health logger changed goals or stopped playback");
        std::cout<<"PASS health fault context, communication loss, recovery and read-only monitoring\n";

        pump([&]{return node->action_transactions_.completed.Contains(9001);});flush();
        Check(Dxl::current.at(21)==2077&&Dxl::current.at(22)==1537,"ordinary P2P JSON overwrote FORWARD head pose");
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"LINE walking missed arm pose override");
        Check(Dxl::current.at(13)==2048,"LINE arm override changed ID 13");
        Check(Dxl::current.at(19)==1968,"walking did not override ID19");
        Check(cameras.empty(),"automatic FORWARD hold emitted a camera command status");
        std::cout<<"PASS real MainNode startup FORWARD transmission/hold and ordinary P2P override\n";

        Check(MainNode::UsesLineMissionArmPose(1),"LINE mission must override arms");
        for(uint8_t mission:{2,3,4})
            Check(!MainNode::UsesLineMissionArmPose(mission),"non-LINE mission must retain JSON arms");
        for(uint8_t mission:{2,3,4}) {
            const uint64_t id=9100+mission;
            const auto held=Dxl::current;
            const auto packet_start=Dxl::packets.size();
            action(id,11,mission);
            if (mission==2) {
                Check(node->arm_transition_in_progress_,"LINE exit did not start arm transition");
                action(id,11,mission); // active duplicate during transition must not restart it
                pump([&]{return !node->arm_transition_in_progress_;});
                Check(Count(actions,id,Status::DONE)==0&&Count(actions,id,Status::READY)==0,
                      "arm-only transition emitted premature DONE/READY");
                Check(Dxl::current.at(15)==1071&&Dxl::current.at(16)==3073,"restoration did not precede action");
                for(std::size_t index=packet_start;index<Dxl::packets.size();++index)
                    for(int motor:Dxl::ids) if(motor!=15&&motor!=16)
                        Check(Dxl::packets[index].at(motor)==held.at(motor),"arm transition moved another joint");
            } else Check(!node->arm_transition_in_progress_,"non-LINE to non-LINE unnecessarily changed arms");
            pump([&]{return node->action_transactions_.completed.Contains(id);});flush();
            Check(Dxl::current.at(15)==2048&&Dxl::current.at(16)==2048,"non-LINE walking did not retain JSON arm pose");
            Check(Count(actions,id,Status::READY)==1,"non-LINE long walk must send READY once");
            Check(!node->queued_action_,"non-LINE READY erroneously queued another action");
        }
        for(uint16_t code:{14,15,16,17}) Check(!MainNode::UsesFixedGripperPose(code),"task gripper exclusion missing");
        for(uint16_t code:{1,2,3,7,11,13,18,21,22,23,24,25,26})
            Check(MainNode::UsesFixedGripperPose(code),"normal action gripper override missing");
        // 공을 향해 걷기 → 공 잡기 → 다시 걷기에서 action별 팔 목표 전환을 검증한다.
        WriteMotion(directory/"motions/공_줍고일어나기.json");
        action(9190,14,2);
        pump([&]{return node->action_transactions_.completed.Contains(9190);});flush();
        Check(Dxl::current.at(15)==2048&&Dxl::current.at(16)==2048,"PICK_BALL did not restore JSON arm pose");
        Check(Dxl::current.at(19)==2048,"PICK_BALL gripper was overridden");
        action(9191,11,2);
        pump([&]{return node->action_transactions_.completed.Contains(9191);});flush();
        Check(Dxl::current.at(15)==2048&&Dxl::current.at(16)==2048,"BALL walking incorrectly overrode arms");
        Check(Dxl::current.at(19)==1968,"non-LINE walking missed gripper override");
        action(9200,11);
        Check(node->arm_transition_in_progress_,"return to LINE missed arm opening transition");
        pump([&]{return !node->arm_transition_in_progress_;});
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"LINE action began before arms opened");
        Check(Count(actions,9200,Status::DONE)==0&&Count(actions,9200,Status::READY)==0,
              "LINE arm opening completed the action prematurely");
        pump([&]{return node->action_transactions_.completed.Contains(9200);});flush();
        // mission만으로 판별하므로 LINE의 PICK_BALL action도 팔 목표를 유지한다.
        action(9201,14,1);pump([&]{return node->action_transactions_.completed.Contains(9201);});flush();
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"LINE arm pose still depends on action type");
        Check(Count(actions,9200,Status::READY)==1,"LINE READY missing after non-LINE walks");
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"return to LINE did not reapply arm pose");
        Check(Dxl::current.at(19)==2048,"LINE task action incorrectly overrode gripper");
        std::cout<<"PASS mission-only arm scope, restoration/opening before action, duplicate ACK and LINE task action\n";

        action(1001,11);action(1001,11);action(1002,12);action(0,11);
        camera(1001,3);camera(1001,3);camera(2002,2);camera(0,2);camera(9000,9);
        pump([&]{return node->camera_transactions_.completed.Contains(1001);});
        Check(node->motion_in_progress_,"body stopped during camera transition");
        pump([&]{return node->action_transactions_.ready_sent;});
        const int reads=Dxl::reads;
        std::ofstream(right)<<"bad JSON";
        action(1002,12);
        Check(!node->queued_action_&&node->p2p_player_->IsPlaying(),"invalid queued preparation altered A");
        WriteMotion(right);
        action(1002,12);action(1002,12);action(1003,13);action(1004,18);action(1005,12,2);
        Check(Dxl::reads==reads,"queued preparation touched hardware");
        Check(node->queued_action_.has_value(),"prepared B not queued");
        // Starting B must use the prepared object even if its file changes.
        std::ofstream(right)<<"changed after ACK";
        pump([&]{return node->action_transactions_.completed.Contains(1002);});flush();
        Check(Count(actions,1001,Status::ACK)==2&&Count(actions,1001,Status::READY)==1&&
              Count(actions,1001,Status::DONE)==1,"A lifecycle/duplicate mismatch");
        Check(Count(actions,1002,Status::ACK)==2&&Count(actions,1002,Status::READY)==1&&
              Count(actions,1002,Status::DONE)==1,"B auto-start or extra ACK mismatch");
        for(uint64_t id:{0,1003,1004,1005})Check(Count(actions,id,Status::ACK)==0,"rejected action ACKed");
        Check(Count(cameras,1001,Status::ACK)==2&&Count(cameras,1001,Status::DONE)==1,"camera duplicate lifecycle mismatch");
        Check(Count(cameras,2002,Status::ACK)==0&&Count(cameras,0,Status::ACK)==0&&
              Count(cameras,9000,Status::ACK)==0,"invalid/busy camera ACKed");
        Check(Dxl::current.at(21)==2054&&Dxl::current.at(22)==966,"persistent GOAL overwritten by body");
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"queued LINE program lost arm targets");
        action(1001,11);camera(1001,3);flush();
        Check(Count(actions,1001,Status::DONE)==2&&Count(cameras,1001,Status::DONE)==2,"old completed IDs restarted");
        std::cout<<"PASS real MainNode A/READY/queue/B lifecycle, duplicates, rejection, prepared-file persistence\n";

        // 신규 6 action은 준비된 JSON/팔 목표를 큐에 저장하고 DONE 직후 자동 실행한다.
        for(uint16_t code=21;code<=26;++code) {
            const auto file=directory/node->ProgramFileForAction(code);
            WriteMotion(file);
            WriteMotion(directory/node->ProgramFileForAction(21));
            const uint64_t first=30000+code*2, second=first+1;
            action(first,code);
            pump([&]{return node->action_transactions_.ready_sent;});
            action(second,21);
            Check(node->queued_action_.has_value(),"new action failed to queue next walk");
            pump([&]{return node->action_transactions_.completed.Contains(second);});flush();
            Check(Count(actions,first,Status::ACK)==1&&Count(actions,first,Status::READY)==1&&
                  Count(actions,first,Status::DONE)==1&&Count(actions,second,Status::ACK)==1&&
                  Count(actions,second,Status::READY)==1&&Count(actions,second,Status::DONE)==1,
                  "new action READY/queue/auto-start lifecycle mismatch");
            Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"new queued walk lost symmetric arms");
        }
        std::cout<<"PASS all six new action ACK/READY/queue/DONE and arm targets\n";

        // 시작 시 읽은 고정값과 다른 RECATCH fixture로 제외 규칙을 검증한다.
        WriteMotion(directory/"motions/공_재그립.json",false,2168);
        // 제외된 action 모두 원래 JSON의 집게손을 사용한다.
        fs::create_directories(directory/"missions");
        for(uint16_t code:{14,15,16,17}) {
            const auto file=directory/node->ProgramFileForAction(code);
            if(code!=15) WriteMotion(file);
            const uint64_t id=45000+code;
            action(id,code,2);pump([&]{return node->action_transactions_.completed.Contains(id);});flush();
            Check(Dxl::current.at(19)==(code==15?2168:2048),"task action lost original gripper JSON pose");
        }
        // 돌아오는 팔 전환에서도 집게손 기준이 유지되어야 한다.
        action(45050,21,1);pump([&]{return node->action_transactions_.completed.Contains(45050);});flush();
        Check(Dxl::current.at(19)==1968,"LINE return/queued walking missed gripper target");
        const auto body_before=Dxl::current;
        camera(2002,2);
        pump([&]{return node->camera_transactions_.completed.Contains(2002);});flush();
        for(int id:Dxl::MotorIds())if(id!=21&&id!=22)Check(Dxl::current.at(id)==body_before.at(id),"idle camera changed body");
        Check(Dxl::current.at(21)==2077&&Dxl::current.at(22)==1537&&
              node->motion_loop_timer_->is_canceled(),"idle camera final pose/timer mismatch");
        for(std::size_t i=1;i<Dxl::packet_times.size();++i)
            Check(Dxl::packet_times[i]-Dxl::packet_times[i-1]>=19ms,"body/camera writer compressed packets");
        std::cout<<"PASS body/camera simultaneous execution and idle camera single-writer spacing\n";

        // 0도 회전도 미션 경계에서는 팔 전환을 생략하지 않는다.
        action(2801,7,2);
        Check(node->arm_transition_in_progress_,"zero-turn LINE exit skipped restoration");
        pump([&]{return node->action_transactions_.completed.Contains(2801);});flush();
        Check(Dxl::current.at(15)==1071&&Dxl::current.at(16)==3073&&
              Count(actions,2801,Status::ACK)==1&&Count(actions,2801,Status::DONE)==1,
              "zero-turn restoration lifecycle mismatch");
        action(2802,8,1);
        pump([&]{return node->action_transactions_.completed.Contains(2802);});flush();
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"zero-turn return to LINE skipped opening");

        WriteMotion(right);action(3001,11);
        pump([&]{return node->action_transactions_.ready_sent;});action(3002,12);
        Dxl::fail_read=true;
        pump([&]{return !node->motion_in_progress_;});Dxl::fail_read=false;flush();
        Check(Count(actions,3001,Status::DONE)==1&&Count(actions,3002,Status::ACK)==1&&
              Count(actions,3002,Status::DONE)==0&&!node->queued_action_&&
              node->action_transactions_.active_id==0,"failed queued start produced fake DONE or stale state");
        camera(4001,1);Dxl::fail_write=true;
        pump([&]{return !node->camera_motion_->Active();});Dxl::fail_write=false;flush();
        Check(Count(cameras,4001,Status::ACK)==1&&Count(cameras,4001,Status::DONE)==0&&
              node->camera_transactions_.active_id==0,"failed writer produced fake camera DONE");
        // 팔 전환 전송 실패 시 요청 action을 시작하지 않고 DONE도 보내지 않는다.
        WriteMotion(directory/node->ProgramFileForAction(21));
        action(4101,21,2);
        Check(node->arm_transition_in_progress_,"arm transition did not restart after abort");
        Dxl::fail_write=true;
        pump([&]{return !node->motion_in_progress_;});Dxl::fail_write=false;flush();
        Check(!node->pending_mission_program_&&!node->arm_transition_in_progress_&&
              Count(actions,4101,Status::ACK)==1&&Count(actions,4101,Status::DONE)==0,
              "failed arm transition started action or emitted DONE");
        action(4102,21,1);
        Check(node->arm_transition_in_progress_,"partial arm pose was not revalidated on next action");
        pump([&]{return node->action_transactions_.completed.Contains(4102);});flush();
        Check(Dxl::current.at(15)==1379&&Dxl::current.at(16)==2765,"arm transition recovery missed LINE targets");
        std::cout<<"PASS queued-start/write failures stop consistently without fake DONE\n";
        rclcpp::shutdown();fs::remove_all(directory);return 0;
    } catch(const std::exception& e) {
        if(rclcpp::ok())rclcpp::shutdown();fs::remove_all(directory);
        std::cerr<<"FAIL "<<e.what()<<'\n';return 1;
    }
}

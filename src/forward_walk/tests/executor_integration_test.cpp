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
void WriteMotion(const fs::path& path, bool startup=false) {
    std::ofstream stream(path);
    stream << "{\"keyframes\":[";
    for(int frame=0;frame<(startup?1:2);++frame) {
        if(frame)stream<<',';
        stream << "{\"duration_sec\":0.2,\"hold_sec\":" << (frame==1?0.08:0.0)
               << ",\"max_speed_deg_s\":4000,\"interpolation\":\"smoothstep\",\"positions\":{";
        bool first=true;
        for(int id:Dxl::MotorIds()) {
            if(!first)stream<<',';first=false;
            stream<<'"'<<id<<"\":"<<(id==0&&!startup?2148+frame*100:2048);
        }
        stream<<"}}";
    }
    stream<<"]}";
}

int main() {
    const auto directory=fs::temp_directory_path()/("jandi_main_test_"+std::to_string(getpid()));
    fs::create_directories(directory/"motions");
    const auto startup=directory/"motions/보행_왼발만앞에.json";
    const auto left=directory/"motions/보행_좌회전전진_20도_최종_초기X.json";
    const auto right=directory/"motions/보행_우회전전진_20도_최종_초기X.json";
    WriteMotion(startup,true);WriteMotion(left);WriteMotion(right);
    const std::vector<std::string> options={"executor_integration_test","--ros-args",
        "-p","p2p_asset_directory:="+directory.string(),
        "-p","startup_pose_duration_sec:=0.04",
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
        auto pump=[&](const std::function<bool()>& done, double seconds=2.0) {
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
        for(int id:Dxl::MotorIds())if(id!=21&&id!=22)
            Check(Dxl::current.at(id)==2048,"startup FORWARD hold changed body");
        // A normal body JSON contains different head ticks; the automatic FORWARD hold wins.
        action(9001,11);
        pump([&]{return node->action_transactions_.completed.Contains(9001);});flush();
        Check(Dxl::current.at(21)==2077&&Dxl::current.at(22)==1537,"ordinary P2P JSON overwrote FORWARD head pose");
        Check(cameras.empty(),"automatic FORWARD hold emitted a camera command status");
        std::cout<<"PASS real MainNode startup FORWARD transmission/hold and ordinary P2P override\n";

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
        action(1001,11);camera(1001,3);flush();
        Check(Count(actions,1001,Status::DONE)==2&&Count(cameras,1001,Status::DONE)==2,"old completed IDs restarted");
        std::cout<<"PASS real MainNode A/READY/queue/B lifecycle, duplicates, rejection, prepared-file persistence\n";

        const auto body_before=Dxl::current;
        camera(2002,2);
        pump([&]{return node->camera_transactions_.completed.Contains(2002);});flush();
        for(int id:Dxl::MotorIds())if(id!=21&&id!=22)Check(Dxl::current.at(id)==body_before.at(id),"idle camera changed body");
        Check(Dxl::current.at(21)==2077&&Dxl::current.at(22)==1537&&
              node->motion_loop_timer_->is_canceled(),"idle camera final pose/timer mismatch");
        for(std::size_t i=1;i<Dxl::packet_times.size();++i)
            Check(Dxl::packet_times[i]-Dxl::packet_times[i-1]>=19ms,"body/camera writer compressed packets");
        std::cout<<"PASS body/camera simultaneous execution and idle camera single-writer spacing\n";

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
        std::cout<<"PASS queued-start/write failures stop consistently without fake DONE\n";
        rclcpp::shutdown();fs::remove_all(directory);return 0;
    } catch(const std::exception& e) {
        if(rclcpp::ok())rclcpp::shutdown();fs::remove_all(directory);
        std::cerr<<"FAIL "<<e.what()<<'\n';return 1;
    }
}

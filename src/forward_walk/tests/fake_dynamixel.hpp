#ifndef JANDI_TEST_FAKE_DYNAMIXEL_HPP
#define JANDI_TEST_FAKE_DYNAMIXEL_HPP
// This is force-included only in the integration test target.
#define DYNAMIXEL_H
#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#define TORQUE_TO_VALUE_MX_106 183.7155
class Dxl {
public:
    using Positions = std::unordered_map<int,int32_t>;
    inline static constexpr std::array<uint8_t,23> ids = {
        10,8,6,4,2,0,11,9,7,5,3,1,12,13,14,15,16,17,18,19,20,21,22};
    static const auto& MotorIds() { return ids; }
    inline static Positions current = [] {
        Positions p; for (int id:ids) p[id]=2048; return p;
    }();
    inline static std::vector<Positions> packets;
    inline static std::vector<std::chrono::steady_clock::time_point> packet_times;
    inline static int reads{0};
    inline static bool fail_read{false}, fail_write{false};
    Positions GetRawPositions() {
        ++reads;
        if (fail_read) throw std::runtime_error("injected read failure");
        return current;
    }
    void SyncWriteRawPositions(const Positions& p) {
        if (fail_write) throw std::runtime_error("injected write failure");
        if (p.size()!=23) throw std::runtime_error("incomplete packet");
        current=p;
        packets.push_back(p);
        packet_times.push_back(std::chrono::steady_clock::now());
    }
    Eigen::VectorXd GetCurrent() { return Eigen::VectorXd::Zero(23); }
};
#endif

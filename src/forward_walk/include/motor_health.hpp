#pragma once
#include <cstdint>
#include <string>

// MX(Protocol 2.0) RAM 64..146을 한 번 읽은 진단 결과. 모터 설정은 변경하지 않는다.
struct MotorHealth {
    int id{0};
    bool valid{false};
    int torque{-1}, hardware_error{-1}, temperature{-1}, voltage_raw{-1}, packet_error{0};
    std::string communication_error;
    static MotorHealth Decode(int id, const uint8_t* data, int packet_error) {
        MotorHealth h; h.id=id; h.packet_error=packet_error;
        // Alert(0x80)는 데이터가 유효하다. 나머지 packet error는 값으로 오인하지 않는다.
        if (packet_error & 0x7f) { h.communication_error="장치가 진단 읽기를 거부함"; return h; }
        h.valid=true; h.torque=data[0]; h.hardware_error=data[6];
        h.voltage_raw=data[80] | (data[81]<<8); h.temperature=data[82]; return h;
    }
    std::string Reason() const {
        if (!valid) return "진단 읽기 실패(전원/케이블/USB 지연 등 원인 미확정): " + communication_error;
        std::string s;
        const auto add=[&](const std::string& reason) { if (!s.empty()) s+="; "; s+=reason; };
        if (torque==0) add("토크 OFF");
        else if (torque!=1) add("비정상 Torque Enable 값");
        if (hardware_error&0x01) add("입력 전압 범위 오류");
        if (hardware_error&0x04) add("과열");
        if (hardware_error&0x08) add("모터 엔코더 오류");
        if (hardware_error&0x10) add("전기적 오류/전력 공급 부족");
        if (hardware_error&0x20) add("과부하");
        if (hardware_error&~0x3d) add("알 수 없는 하드웨어 오류 비트");
        if ((packet_error&0x80) && hardware_error==0) add("장치 Alert(하드웨어 원인 미확정)");
        if (torque==0 && hardware_error==0) add("원인 미확정(재부팅/외부 토크 해제 등 가능)");
        return s;
    }
};

#include "dynamixel.hpp"
#include <iostream>  
#include <stdexcept>
#include <string>

// const char* getAvailableDeviceName()
// {
//     std::vector<const char*> candidates = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};
//     for (auto dev : candidates) {
//         auto portHandler = dynamixel::PortHandler::getPortHandler(dev);
//         if (portHandler->openPort()) {
//             portHandler->setBaudRate(BAUDRATE);
//             auto packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//             uint16_t model_number;
//             uint8_t dxl_error = 0;
//             int dxl_comm_result = packetHandler->ping(portHandler, 1, &model_number, &dxl_error);
//             // ⚠️ 여기서 1은 네 다이나믹셀 ID. 네 환경에 맞게 바꿔줘야 함.

//             if (dxl_comm_result == COMM_SUCCESS) {
//                 std::cout << "[Info] Dynamixel found on " << dev << std::endl;
//                 portHandler->closePort(); // 다시 열기 위해 닫아줌
//                 return dev;
//             } else {
//                 portHandler->closePort();
//             }
//         }
//     }
//     std::cerr << "[Error] No valid Dynamixel port found" << std::endl;
//     exit(1);
// }


Dxl::Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;


    // const char* device_name = getAvailableDeviceName();   // ✅ 자동 선택
    // portHandler = dynamixel::PortHandler::getPortHandler(device_name);
    // packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
        throw std::runtime_error("Failed to open Dynamixel port: " + std::string(DEVICE_NAME));
    else 
        std::cout << "[Info] Succeeded to open the port!" << std::endl;

    if (!portHandler->setBaudRate(BAUDRATE)) {
        portHandler->closePort();
        throw std::runtime_error("Failed to set Dynamixel baudrate: " + std::to_string(BAUDRATE));
    }
    else 
        std::cout << "[Info] Succeeded to set the baudrate!" << std::endl;

    int16_t current_mode = SetPresentMode(Mode);

    if (current_mode == Current_Control_Mode)
    {
        for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_OperatingMode, Current_Control_Mode, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                std::cerr << "[Error] Failed to set current control mode for ID: " << int(dxl_id[i]) << std::endl;
            else
                std::cout << "[Info] Set current control mode for ID: " << int(dxl_id[i]) << std::endl;
        }
    }
    else if (current_mode == Position_Control_Mode)
    {
        for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_OperatingMode, Position_Control_Mode, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                std::cerr << "[Error] Failed to set position control mode for ID: " << int(dxl_id[i]) << std::endl;
            else
                std::cout << "[Info] Set position control mode for ID: " << int(dxl_id[i]) << std::endl;
        }
    }
    else
    {
        std::cerr << "[Error] Invalid mode set." << std::endl;
    }




    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to enable torque for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] Torque enabled for ID: " << int(dxl_id[i]) << std::endl;
    }


    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to enable LED for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] LED enabled for ID: " << int(dxl_id[i]) << std::endl;
    }




    // VectorXd theta_zero = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    // MoveToTargetSmoothCos(theta_zero, 150, 10);

    // VectorXd theta_goal = Eigen::Map<VectorXd>(Set_D, NUMBER_OF_DYNAMIXELS);
    // MoveToTargetSmoothCos(theta_goal, 150, 10);
    // std::cout << "[Info] Start is half!!!!!!" << std::endl;


    VectorXd PID_Gain(3);
    PID_Gain << 850, 0, 0;
    SetPIDGain(PID_Gain);
}

Dxl::~Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to disable torque for ID: " << int(i) << std::endl;
        else
            std::cout << "[Info] Torque disabled for ID: " << int(dxl_id[i]) << std::endl;
    }

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to disable LED for ID: " << int(i) << std::endl;
        else
            std::cout << "[Info] LED disabled for ID: " << int(dxl_id[i]) << std::endl;
    }

    portHandler->closePort();
}







// ************************************ GETTERS ***************************************** //

//Getter() : 각도 읽기(raw->rad)
void Dxl::syncReadTheta()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    const int result = groupSyncRead.txRxPacket();
    if (result != COMM_SUCCESS) {
        throw std::runtime_error(packetHandler->getTxRxResult(result));
    }
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) {
        if (!groupSyncRead.isAvailable(dxl_id[i], DxlReg_PresentPosition, 4)) {
            throw std::runtime_error("Present Position unavailable for ID " + std::to_string(dxl_id[i]));
        }
        position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
    }
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
}

//Getter() : 각도 getter() [rad]
VectorXd Dxl::GetThetaAct()
{
    syncReadTheta();
    return th_;
}

//Getter() : velocity 읽기 (raw data)
void Dxl::syncReadThetaDot()
{
    dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, DxlReg_PresentVelocity, 4);
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) groupSyncReadThDot.addParam(dxl_id[i]);
    groupSyncReadThDot.txRxPacket();
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) velocity[i] = groupSyncReadThDot.getData(dxl_id[i], DxlReg_PresentVelocity, 4);
    groupSyncReadThDot.clearParam();
}

//Getter() : 각속도 getter() [rad/s] 
//0.0239868240
VectorXd Dxl::GetThetaDot()
{
    VectorXd vel_(NUMBER_OF_DYNAMIXELS);
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        if(velocity[i] > 4294900000) vel_[i] = (velocity[i] - 4294967295) * 0.003816667; //4,294,967,295 = 0xFFFFFFFF   // 1 = 0.229rpm   // 1 = 0.003816667
        else vel_[i] = velocity[i] * 0.003816667;
    }
    return vel_;
}

//Getter() : About dynamixel packet data
void Dxl::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

//Getter() : 추정계산 (이전 세타값 - 현재 세타값 / 시간) [rad/s]
void Dxl::CalculateEstimatedThetaDot(int dt_us)
{
    th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001);
    th_last_ = th_;
}

//Getter() : 각속도 추정계산 getter() [rad/s] 
VectorXd Dxl::GetThetaDotEstimated()
{
    return th_dot_est_;
}


//Getter() : PID gain getter()
// VectorXd Dxl:: GetPIDGain()
// {

// }

//Getter() : 전류값 [mA] 
void Dxl::SyncReadCurrent()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentCurrent, 2);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) current[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentCurrent, 2);
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        const int16_t signed_current_raw = static_cast<int16_t>(current[i] & 0xFFFF);
        cur_[i] = convertValue2Current(signed_current_raw);
    }
}

VectorXd Dxl::GetCurrent()
{
    SyncReadCurrent();
    return cur_;
}


//Getter() : 현재 모드 getter()
int16_t Dxl::GetPresentMode()
{
    return this->Mode;
}


// **************************** SETTERS ******************************** //

//setter() : 각도 setter() [rad]
void Dxl::syncWriteTheta()
{
  dynamixel::GroupSyncWrite gSyncWriteTh(portHandler, packetHandler, DxlReg_GoalPosition, 4);

  uint8_t parameter[NUMBER_OF_DYNAMIXELS] = {0};

  for (uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++){
    ref_th_value_ = ref_th_ * RAD_TO_VALUE;
    getParam(ref_th_value_[i], parameter);
    gSyncWriteTh.addParam(dxl_id[i], (uint8_t *)&parameter);
  }
  gSyncWriteTh.txPacket();
  gSyncWriteTh.clearParam();
}

std::unordered_map<int, int32_t> Dxl::GetRawPositions()
{
  // 기존 syncReadTheta()가 Present Position을 position[]에 채운다.
  // P2P에서는 radian 변환 결과가 아닌 원래 encoder tick을 motor ID별로 반환한다.
  syncReadTheta();
  std::unordered_map<int, int32_t> result;
  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; ++i) {
    result.emplace(static_cast<int>(dxl_id[i]),
                   static_cast<int32_t>(position[i]));
  }
  return result;
}

void Dxl::SyncWriteRawPositions(
    const std::unordered_map<int, int32_t>& raw_positions)
{
  // JSON positions의 key는 실제 motor ID이고 Dxl 내부 배열 순서는
  // {10,8,6,...}이므로 배열 index로 복사하지 않고 ID로 매번 찾는다.
  dynamixel::GroupSyncWrite writer(
      portHandler, packetHandler, DxlReg_GoalPosition, 4);
  uint8_t parameter[4] = {0};

  for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; ++i) {
    const int motor_id = static_cast<int>(dxl_id[i]);
    const auto target = raw_positions.find(motor_id);
    if (target == raw_positions.end()) {
      throw std::invalid_argument(
          "P2P target is missing Dynamixel ID " + std::to_string(motor_id));
    }
    // int32 raw tick을 Dynamixel Protocol 2.0 Goal Position용 4 byte로 변환한다.
    getParam(target->second, parameter);
    if (!writer.addParam(dxl_id[i], parameter)) {
      throw std::runtime_error(
          "GroupSyncWrite addParam failed for Dynamixel ID " +
          std::to_string(motor_id));
    }
  }

  // 모든 모터의 parameter를 추가한 뒤 패킷은 한 번만 전송한다.
  const int result = writer.txPacket();
  writer.clearParam();
  if (result != COMM_SUCCESS) {
    throw std::runtime_error(packetHandler->getTxRxResult(result));
  }
}




//Setter() : 목표 세타값 설정 [rad]
void Dxl::SetThetaRef(VectorXd theta)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS;i++) 
    {
        ref_th_[i] = theta[i]+PI;
        // std::cout << ref_th_[i] << std::endl;
    }
}

//setter() : 토크 setter() [Nm]
void Dxl::syncWriteTorque()
{
    dynamixel::GroupSyncWrite groupSyncWriter(portHandler, packetHandler, DxlReg_GoalCurrent, 2);
    uint8_t parameter[NUMBER_OF_DYNAMIXELS] = {0};
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        ref_torque_value[i] = torqueToValue(ref_torque_[i], i);
        if(ref_torque_value[i] > 1000) ref_torque_value[i] = 1000; //상한값
        else if(ref_torque_value[i] < -1000) ref_torque_value[i] = -1000; //하한값
    }
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        getParam(ref_torque_value[i], parameter);
        groupSyncWriter.addParam(dxl_id[i], (uint8_t *)&parameter);
    }
    groupSyncWriter.txPacket();
    groupSyncWriter.clearParam();
}

//Setter() : 목표 토크 설정 [Nm]
void Dxl::SetTorqueRef(VectorXd a_torque)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) ref_torque_[i] = a_torque[i];
}

// Setter() : PID gain setter()
void Dxl::SetPIDGain(VectorXd PID_Gain)
{    
    uint8_t dxl_error = 0;
    
    if (PID_Gain.size() != 3)
    {
        std::cerr << "PID_Gain should have exactly 3 elements: P, I, and D gains." << std::endl;
        return;
    }
    
    uint16_t P_gain = static_cast<uint16_t>(PID_Gain(0));
    uint16_t I_gain = static_cast<uint16_t>(PID_Gain(1));
    uint16_t D_gain = static_cast<uint16_t>(PID_Gain(2));

    // P, I, D Gain을 각각의 레지스터에 설정
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        // P Gain 설정
        int result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionPGain, P_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set P gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }

        // I Gain 설정
        result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionIGain, I_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set I gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }

        // D Gain 설정
        result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set D gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }
    }
}

//Setter() : 현재 모드 설정
int16_t Dxl::SetPresentMode(int16_t Mode)
{
    if (Mode == 0)
    {
        this->Mode = Current_Control_Mode;
        return Current_Control_Mode;
    }
    else if (Mode == 1)
    {
        this->Mode = Position_Control_Mode;
        return Position_Control_Mode;
    }
    else
    {
        std::cerr << "[Error] Invalid mode requested. Defaulting to Current Control Mode." << std::endl;
        this->Mode = Current_Control_Mode;
        return Current_Control_Mode;
    }
}

// **************************** Function ******************************** //

//Torque2Value : 토크 -> 로우 data
int32_t Dxl::torqueToValue(double torque, uint8_t index)
{
    int32_t value_ = int(torque * torque2value[index]); //MX-64
    return value_;
}

//Value2Radian (Raw data -> Radian)
float Dxl::convertValue2Radian(int32_t value)
{
    float radian = value / RAD_TO_VALUE;
    return radian;
}

//Value2Curret (Raw data -> Current)
// 1raw  = 3.36[mA]
// Range = 0 ~ 1941 (raw)
float Dxl::convertValue2Current(int32_t value)
{
    float current_ = value *3.36;
    return current_;
}

//각도(rad), 각속도(rad/s) 읽고, torque(Nm->raw) 쓰기 
//제어 주파수(전류제어 : 300, 위치제어 : ?)
void Dxl::Loop(bool RxTh, bool RxThDot, bool TxTorque)
{
    if(RxTh) syncReadTheta();
    if(RxThDot) syncReadThetaDot();
    // if(TxTorque) syncWriteTorque();
    
}

//dxl 초기 세팅
void Dxl::initActuatorValues()
{
    for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++)
    {
        torque2value[i] = TORQUE_TO_VALUE_MX_106;
    }


    
    for (int i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    zero_manual_offset[i] = 0;
}










// portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error


VectorXd Dxl::read_rad()
{
    VectorXd rdl_(NUMBER_OF_DYNAMIXELS);
    int32_t present_position = 0;
    for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->read4ByteTxRx(portHandler, dxl_id[i], DxlReg_PresentPosition,(uint32_t*)&present_position);
        rdl_[i] = (present_position - 2048) * (2.0 * M_PI / 4096.0);
    }

    return rdl_;
}

void Dxl::MoveToTargetSmoothCos(const VectorXd& theta_goal, int steps, int delay_ms)
{
    VectorXd theta_now = read_rad();

    for (int s = 1; s <= steps; ++s)
    {
        double rate = 0.5 * (1 - cos(M_PI * double(s) / steps));
        VectorXd theta_interp = theta_now + (theta_goal - theta_now) * rate;
        SetThetaRef(theta_interp);
        syncWriteTheta();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    SetThetaRef(theta_goal);
    syncWriteTheta();
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

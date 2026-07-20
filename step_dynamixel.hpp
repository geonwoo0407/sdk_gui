#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <thread>
#include <chrono>
#include "dynamixel_sdk/dynamixel_sdk.h"

// #include <unordered_map> //자료구조 중 더 빠른 map탐색 key:value


//Protocol version
#define PROTOCOL_VERSION         2.0

//Default setting
#define NUMBER_OF_DYNAMIXELS     23//모터 갯수
#define BAUDRATE                 4000000 // GUI와 동일한 통신 속도
// #define DEVICE_NAME              "/dev/ttyU2D2"
#define DEVICE_NAME              "/dev/ttyUSB0" // 터미널로 몇번인지 확인해야함
// #define DEVICE_NAME              "/dev/ttyUSB1"

// const char* getAvailableDeviceName();

#define PI                       3.141592
#define TORQUE_TO_VALUE_MX_64    267.094     //mx-64 e-manual plot(not considering about efficiency)
#define TORQUE_TO_VALUE_MX_106   183.7155         
#define RAD_TO_VALUE             651.89878   //1rev = 4096 --> 4096/(2*PI) mx 모델 공통
#define RAD2DEG                  57.2958
#define DEG2RAD                  0.0174533
#define DRIVE_MODE_TIME_BASED_BIT 0x04
#define MIN_TIME_PROFILE_FIRMWARE 42
#define MAX_TIME_PROFILE_MS       32737


using Eigen::VectorXd;

// Operating Mode
enum DynamixelOperatingMode
{
    Current_Control_Mode = 0,
    Velocity_Control_Mode = 1,
    Position_Control_Mode = 3,
    Extended_Position_Control_Mode = 4,
    Current_based_Position_Control_Mode = 5,
    PWM_Control_Mode = 16
};

// Control table address
enum DynamixelStandardRegisterTable
{
  // EEPROM
  DxlReg_ModelNumber        = 0,    // 모델 번호 (제품 고유번호)
  DxlReg_ModelInfo          = 2,    // 모델 정보 (추가 설명 등)
  DxlReg_FirmwareVersion    = 6,    // 펌웨어 버전
  DxlReg_ID                 = 7,    // 다이나믹셀 ID
  DxlReg_BaudRate           = 8,    // 통신 속도(baudrate)
  DxlReg_ReturnDelayTime    = 9,    // Status 패킷 딜레이 시간 설정
  DxlReg_DriveMode          = 10,   // 드라이브 모드 (CW/CCW 방향, 토크 모드 등)
  DxlReg_OperatingMode      = 11,   // 작동 모드 (0: 전류제어, 1: 위치제어 등)
  DxlReg_ShadowID           = 12,   // ID 복제/백업용
  DxlReg_ProtocolVersion    = 13,   // 프로토콜 버전 (1.0/2.0)
  DxlReg_HomingOffset       = 20,   // 영점 위치 오프셋
  DxlReg_MovingThreshold    = 24,   // 이동으로 인식하는 임계 속도
  DxlReg_TemperatureLimit   = 31,   // 온도 제한값 (최대 허용 온도)
  DxlReg_MaxVoltageLimit    = 32,   // 최대 허용 전압
  DxlReg_MinVoltageLimit    = 34,   // 최소 허용 전압
  DxlReg_PWMLimit           = 36,   // PWM 최대 제한값 (PWM 제어용)
  DxlReg_CurrentLimit       = 38,   // 최대 전류 제한값 (토크 한계)
  DxlReg_AccelerationLimit  = 40,   // 최대 가속도 제한
  DxlReg_VelocityLimit      = 44,   // 최대 속도 제한
  DxlReg_MaxPositionLimit   = 48,   // 허용 가능한 최대 위치
  DxlReg_MinPositionLimit   = 52,   // 허용 가능한 최소 위치
  DxlReg_DataPort1Mode      = 56,   // 데이터 포트 1 모드
  DxlReg_DataPort2Mode      = 57,   // 데이터 포트 2 모드
  DxlReg_DataPort3Mode      = 58,   // 데이터 포트 3 모드
  DxlReg_Shutdown           = 63,   // 오류 시 셧다운 조건 설정

  // RAM
  DxlReg_TorqueEnable       = 64,   // 토크 ON/OFF (1: 사용, 0: 미사용)
  DxlReg_LED                = 65,   // 내장 LED ON/OFF
  DxlReg_StatusReturnLevel  = 68,   // Status 패킷 응답 레벨 설정
  DxlReg_RegisteredInstruction = 69, // 등록된 명령 여부
  DxlReg_HardwareErrorStatus   = 70, // 하드웨어 오류 상태(알람)
  DxlReg_VelocityIGain      = 76,   // 속도 제어기 I게인
  DxlReg_VelocityPGain      = 78,   // 속도 제어기 P게인
  DxlReg_PositionDGain      = 80,   // 위치 제어기 D게인
  DxlReg_PositionIGain      = 82,   // 위치 제어기 I게인
  DxlReg_PositionPGain      = 84,   // 위치 제어기 P게인
  DxlReg_Feedforward2ndGain = 88,   // 2차 feedforward 게인
  DxlReg_Feedforward1stGain = 90,   // 1차 feedforward 게인
  DxlReg_BusWatchdog        = 98,   // 버스 감시 기능 (Watchdog)
  DxlReg_GoalPWM            = 100,  // 목표 PWM 값 설정
  DxlReg_GoalCurrent        = 102,  // 목표 전류(토크) 설정
  DxlReg_GoalVelocity       = 104,  // 목표 속도 설정
  DxlReg_ProfileAcceleration = 108, // 가속 프로파일 설정
  DxlReg_ProfileVelocity    = 112,  // 속도 프로파일 설정
  DxlReg_GoalPosition       = 116,  // 목표 위치 설정
  DxlReg_RealtimeTick       = 120,  // 실시간 Tick 값(단위: ms)
  DxlReg_Moving             = 122,  // 현재 이동 중 여부(1: 이동, 0: 정지)
  DxlReg_MovingStatus       = 123,  // 이동 상태(디테일)
  DxlReg_PresentPWM         = 124,  // 현재 PWM 출력 값
  DxlReg_PresentCurrent     = 126,  // 현재 전류(토크) 값
  DxlReg_PresentVelocity    = 128,  // 현재 속도 값
  DxlReg_PresentPosition    = 132,  // 현재 위치 값
  DxlReg_VelocityTrajectory = 136,  // 목표 속도 트래젝토리
  DxlReg_PositionTrajectory = 140,  // 목표 위치 트래젝토리
  DxlReg_PresentInputVoltage = 144, // 현재 입력 전압
  DxlReg_PresentTemperature = 146,  // 현재 온도
  DxlReg_DataPort1          = 152,  // 데이터 포트 1
  DxlReg_DataPort2          = 154,  // 데이터 포트 2
  DxlReg_DataPort3          = 156,  // 데이터 포트 3
  DxlReg_IndirectAddress1   = 168,  // 간접주소 1 (Indirect Addressing)
  DxlReg_IndirectData1      = 224   // 간접데이터 1 (Indirect Data)
};


class Dxl
{
    //Member Variable
    private:
        dynamixel::PortHandler* portHandler;
        dynamixel::PacketHandler* packetHandler;
        // const uint8_t dxl_id[NUMBER_OF_DYNAMIXELS] = {12,18,2};
        const uint8_t dxl_id[NUMBER_OF_DYNAMIXELS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        float zero_manual_offset[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t position[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t velocity[NUMBER_OF_DYNAMIXELS] = { 0 };
        int32_t ref_torque_value[NUMBER_OF_DYNAMIXELS] = { 0 };
        int32_t torque2value[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t current[NUMBER_OF_DYNAMIXELS] = { 0 };

        VectorXd ref_th_value_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_th_dot_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_torque_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        // VectorXd th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_last_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_est_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);

        int16_t Mode = Position_Control_Mode;
        bool port_opened_ = false;

        VectorXd Start_set = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);



        // double Set_D[NUMBER_OF_DYNAMIXELS] = 
        // {
        //         0.0, -0.0, 
        //         0.90776, -0.90776, 
        //         -0.18327, -0.03491,
        //         0.0, 0.03491, 
        //         0.90776, 0.52363, 
        //         0.24036, 0.0,
        //         0.0, 1.57080, 
        //         -1.57080, -1.04720, 
        //         1.04720, -1.57080, 
        //         1.57080,0.0, 
        //         0.0, 0.0, 
        //         0.10472
        // };
//초기 자세 코드 0부터 시작해서 우리 로봇이랑 맞춰서 값 수정 0으로 했을때 어떤 자세인지 확인




// ************************************ GETTERS ***************************************** //

        virtual bool syncReadThetaDot(); //모터 속도를 동시에 읽음
        virtual void getParam(int32_t data, uint8_t *param);

// **************************** SETTERS ******************************** //

        virtual void syncWriteTorque();

// **************************** Function ******************************** //

        // virtual void initActuatorValues();
        float convertValue2Radian(int32_t value);
        int32_t torqueToValue(double torque, uint8_t index);

    // Member Function
    public:
        Dxl(); //생성자
        ~Dxl(); //소멸자

// ************************************ GETTERS ***************************************** //

        virtual VectorXd GetThetaAct();
        virtual VectorXd GetThetaDot();
        virtual VectorXd GetThetaDotEstimated();
        // virtual VectorXd GetPIDGain();
        virtual int16_t GetPresentMode();
        virtual bool syncReadTheta();  // rad_pos = (count-count_initial_position) * (range/360) * (2*PI/encoder_cpr)
        VectorXd th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        virtual void SyncReadCurrent();
        VectorXd cur_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        virtual VectorXd GetCurrent();

        VectorXd read_D = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);





// **************************** SETTERS ******************************** //

        virtual void SetTorqueRef(VectorXd);
        virtual void SetThetaRef(const VectorXd&);
        virtual int16_t SetPresentMode(int16_t Mode); 
        virtual bool syncWriteTheta();
        virtual bool ConfigureTimeBasedProfile();
        virtual bool syncWriteTimeBasedTheta(
            const VectorXd& theta, const std::vector<int>& motor_ids,
            uint32_t duration_ms);
        virtual bool SetTorqueEnabled(bool enabled);
        virtual bool IsReady() const;
        void SetPIDGain(VectorXd PID_Gain);


        

// **************************** Function ******************************** //

        virtual void Loop(bool RxTh, bool RxThDot, bool TxTorque);
        virtual void CalculateEstimatedThetaDot(int);
        virtual void initActuatorValues();
        // virtual void FSR_flag();
        // virtual void Quaternino2RPY();
        virtual float convertValue2Current(int32_t value);

        virtual VectorXd read_rad();
        virtual void MoveToTargetSmoothCos(const VectorXd& theta_goal, int steps, int delay_ms);





};


#endif

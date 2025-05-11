// Copyright 2025 Electrified Autonomy, LLC  // NOLINT
//
// SMSCL.hpp
// 飞特SMSCL系列串行舵机应用层程序
// 日期: 2020.6.17
// 作者:

#ifndef FEETECH_LIB__SMSCL_HPP_
#define FEETECH_LIB__SMSCL_HPP_

#define SMSCL_1M     0
#define SMSCL_0_5M   1
#define SMSCL_250K   2
#define SMSCL_128K   3
#define SMSCL_115200 4
#define SMSCL_76800  5
#define SMSCL_57600  6
#define SMSCL_38400  7

// 内存表定义
// -------EPROM(只读)--------
#define SMSCL_VERSION_L 3
#define SMSCL_VERSION_H 4

// -------EPROM(读写)--------
#define SMSCL_ID 5
#define SMSCL_BAUD_RATE 6
#define SMSCL_RETURN_DELAY_TIME 7
#define SMSCL_RETURN_LEVEL 8
#define SMSCL_MIN_ANGLE_LIMIT_L 9
#define SMSCL_MIN_ANGLE_LIMIT_H 10
#define SMSCL_MAX_ANGLE_LIMIT_L 11
#define SMSCL_MAX_ANGLE_LIMIT_H 12
#define SMSCL_LIMIT_TEMPERATURE 13
#define SMSCL_MAX_LIMIT_VOLTAGE 14
#define SMSCL_MIN_LIMIT_VOLTAGE 15
#define SMSCL_MAX_TORQUE_L 16
#define SMSCL_MAX_TORQUE_H 17
#define SMSCL_ALARM_LED 19
#define SMSCL_ALARM_SHUTDOWN 20
#define SMSCL_COMPLIANCE_P 21
#define SMSCL_COMPLIANCE_D 22
#define SMSCL_COMPLIANCE_I 23
#define SMSCL_PUNCH_L 24
#define SMSCL_PUNCH_H 25
#define SMSCL_CW_DEAD 26
#define SMSCL_CCW_DEAD 27
#define SMSCL_OFS_L 33
#define SMSCL_OFS_H 34
#define SMSCL_MODE 35
#define SMSCL_MAX_CURRENT_L 36
#define SMSCL_MAX_CURRENT_H 37

// -------SRAM(读写)--------
#define SMSCL_TORQUE_ENABLE 40
#define SMSCL_ACC 41
#define SMSCL_GOAL_POSITION_L 42
#define SMSCL_GOAL_POSITION_H 43
#define SMSCL_GOAL_TIME_L 44
#define SMSCL_GOAL_TIME_H 45
#define SMSCL_GOAL_SPEED_L 46
#define SMSCL_GOAL_SPEED_H 47
#define SMSCL_LOCK 48

// -------SRAM(只读)--------
#define SMSCL_PRESENT_POSITION_L 56
#define SMSCL_PRESENT_POSITION_H 57
#define SMSCL_PRESENT_SPEED_L 58
#define SMSCL_PRESENT_SPEED_H 59
#define SMSCL_PRESENT_LOAD_L 60
#define SMSCL_PRESENT_LOAD_H 61
#define SMSCL_PRESENT_VOLTAGE 62
#define SMSCL_PRESENT_TEMPERATURE 63
#define SMSCL_REGISTERED_INSTRUCTION 64
#define SMSCL_MOVING 66
#define SMSCL_PRESENT_CURRENT_L 69
#define SMSCL_PRESENT_CURRENT_H 70

#include <cstdint>

#include "feetech_lib/SCSerial.hpp"

class SMSCL : public SCSerial
{
public:
  SMSCL();
  explicit SMSCL(uint8_t End);
  SMSCL(uint8_t End, uint8_t Level);
  virtual int32_t WritePosEx(
                               // 普通写单个舵机位置指令
    uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0);
  virtual int32_t RegWritePosEx(
                                  // 异步写单个舵机位置指令(RegWriteAction生效)
    uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0);
  virtual void SyncWritePosEx(
                                  // 同步写多个舵机位置指令
    uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);
  virtual int32_t WheelMode(uint8_t ID);       // 恒速模式
  virtual int32_t WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC = 0);  // 恒速模式控制指令
  virtual int32_t EnableTorque(uint8_t ID, uint8_t Enable);  // 扭力控制指令
  virtual int32_t unLockEprom(uint8_t ID);     // eprom解锁
  virtual int32_t LockEprom(uint8_t ID);       // eprom加锁
  virtual int32_t CalibrationOfs(uint8_t ID);  // 中位校准
  virtual int32_t FeedBack(int32_t ID);        // 反馈舵机信息
  virtual int32_t ReadPos(int32_t ID);         // 读位置
  virtual int32_t ReadSpeed(int32_t ID);       // 读速度
  virtual int32_t ReadLoad(int32_t ID);        // 读输出至电机的电压百分比(0~1000)
  virtual int32_t ReadVoltage(int32_t ID);     // 读电压
  virtual int32_t ReadTemper(int32_t ID);      // 读温度
  virtual int32_t ReadMove(int32_t ID);        // 读移动状态
  virtual int32_t ReadCurrent(int32_t ID);     // 读电流

private:
  uint8_t Mem[SMSCL_PRESENT_CURRENT_H - SMSCL_PRESENT_POSITION_L + 1];
};

#endif  // FEETECH_LIB__SMSCL_HPP_

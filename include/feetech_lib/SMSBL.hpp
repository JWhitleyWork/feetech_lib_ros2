// Copyright 2025 Electrified Autonomy, LLC  // NOLINT
//
// SMSBL.hpp
// 飞特SMSBL系列串行舵机应用层程序
// 日期: 2020.6.17
// 作者:

#ifndef FEETECH_LIB__SMSBL_HPP_
#define FEETECH_LIB__SMSBL_HPP_

// 波特率定义
#define SMSBL_1M 0
#define SMSBL_0_5M 1
#define SMSBL_250K 2
#define SMSBL_128K 3
#define SMSBL_115200 4
#define SMSBL_76800     5
#define SMSBL_57600     6
#define SMSBL_38400     7

// 内存表定义
// -------EPROM(只读)--------
#define SMSBL_MODEL_L 3
#define SMSBL_MODEL_H 4

// -------EPROM(读写)--------
#define SMSBL_ID 5
#define SMSBL_BAUD_RATE 6
#define SMSBL_MIN_ANGLE_LIMIT_L 9
#define SMSBL_MIN_ANGLE_LIMIT_H 10
#define SMSBL_MAX_ANGLE_LIMIT_L 11
#define SMSBL_MAX_ANGLE_LIMIT_H 12
#define SMSBL_CW_DEAD 26
#define SMSBL_CCW_DEAD 27
#define SMSBL_OFS_L 31
#define SMSBL_OFS_H 32
#define SMSBL_MODE 33

// -------SRAM(读写)--------
#define SMSBL_TORQUE_ENABLE 40
#define SMSBL_ACC 41
#define SMSBL_GOAL_POSITION_L 42
#define SMSBL_GOAL_POSITION_H 43
#define SMSBL_GOAL_TIME_L 44
#define SMSBL_GOAL_TIME_H 45
#define SMSBL_GOAL_SPEED_L 46
#define SMSBL_GOAL_SPEED_H 47
#define SMSBL_LOCK 55

// -------SRAM(只读)--------
#define SMSBL_PRESENT_POSITION_L 56
#define SMSBL_PRESENT_POSITION_H 57
#define SMSBL_PRESENT_SPEED_L 58
#define SMSBL_PRESENT_SPEED_H 59
#define SMSBL_PRESENT_LOAD_L 60
#define SMSBL_PRESENT_LOAD_H 61
#define SMSBL_PRESENT_VOLTAGE 62
#define SMSBL_PRESENT_TEMPERATURE 63
#define SMSBL_MOVING 66
#define SMSBL_PRESENT_CURRENT_L 69
#define SMSBL_PRESENT_CURRENT_H 70

#include <cstdint>

#include "feetech_lib/SCSerial.hpp"

class SMSBL : public SCSerial
{
public:
  SMSBL();
  explicit SMSBL(uint8_t End);
  SMSBL(uint8_t End, uint8_t Level);
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
  uint8_t Mem[SMSBL_PRESENT_CURRENT_H - SMSBL_PRESENT_POSITION_L + 1];
};

#endif  // FEETECH_LIB__SMSBL_HPP_

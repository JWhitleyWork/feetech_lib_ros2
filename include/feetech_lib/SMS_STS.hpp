/*
 * SMS_STS.hpp
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2021.12.8
 * 作者:
 */

#ifndef FEETECH_LIB__SMS_STS_HPP_
#define FEETECH_LIB__SMS_STS_HPP_

//波特率定义
#define SMS_STS_1M 0
#define SMS_STS_0_5M 1
#define SMS_STS_250K 2
#define SMS_STS_128K 3
#define SMS_STS_115200 4
#define SMS_STS_76800 5
#define SMS_STS_57600 6
#define SMS_STS_38400 7

//内存表定义
//-------EPROM(只读)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(读写)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM(读写)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMSBL_TORQUE_LIMIT_L 48
#define SMSBL_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

//-------SRAM(只读)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

#include "feetech_lib/SCSerial.hpp"

#include <cstdint>

class SMS_STS : public SCSerial
{
public:
  SMS_STS();
  SMS_STS(uint8_t End);
  SMS_STS(uint8_t End, uint8_t Level);
  virtual int32_t WriteID(uint8_t ID, uint8_t NewID);
  virtual int32_t WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0);  //普通写单个舵机位置指令
  virtual int32_t RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0);  //异步写单个舵机位置指令(RegWriteAction生效)
  virtual void SyncWritePosEx(
    uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[],
    uint8_t ACC[]);                                                                                             //同步写多个舵机位置指令
  virtual int32_t WriteTorqueLimit(uint8_t ID, uint16_t TorqueLimit);
  virtual int32_t WheelMode(uint8_t ID);  //恒速模式
  virtual int32_t WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC = 0);  //恒速模式控制指令
  virtual int32_t PWMMode(uint8_t ID);  //PWM模式
  virtual int32_t PositionMode(uint8_t ID);  //Position模式
  virtual int32_t WritePWM(uint8_t ID, int16_t pwmOut);  //PWM模式控制指令
  virtual int32_t EnableTorque(uint8_t ID, uint8_t Enable);  //扭力控制指令
  virtual int32_t unLockEprom(uint8_t ID);  //eprom解锁
  virtual int32_t LockEprom(uint8_t ID);  //eprom加锁
  virtual int32_t CalibrationOfs(uint8_t ID);  //中位校准
  virtual int32_t FeedBack(int32_t ID);  //反馈舵机信息
  virtual int32_t ReadPos(int32_t ID);  //读位置
  virtual int32_t ReadSpeed(int32_t ID);  //读速度
  virtual int32_t ReadLoad(int32_t ID);  //读输出至电机的电压百分比(0~1000)
  virtual int32_t ReadVoltage(int32_t ID);  //读电压
  virtual int32_t ReadTemper(int32_t ID);  //读温度
  virtual int32_t ReadMove(int32_t ID);  //读移动状态
  virtual int32_t ReadCurrent(int32_t ID);  //读电流

private:
  uint8_t Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1];
};

#endif  // FEETECH_LIB__SMS_STS_HPP_

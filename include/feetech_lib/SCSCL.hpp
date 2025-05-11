/*
 * SCSCL.hpp
 * 飞特SCSCL系列串行舵机应用层程序
 * 日期: 2020.6.17
 * 作者:
 */

#ifndef FEETECH_LIB__SCSCL_HPP_
#define FEETECH_LIB__SCSCL_HPP_

#define SCSCL_1M 0
#define SCSCL_0_5M 1
#define SCSCL_250K 2
#define SCSCL_128K 3
#define SCSCL_115200 4
#define SCSCL_76800     5
#define SCSCL_57600     6
#define SCSCL_38400     7

//内存表定义
//-------EPROM(只读)--------
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4

//-------EPROM(读写)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM(读写)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM(只读)--------
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

#include "feetech_lib/SCSerial.hpp"

#include <cstdint>

class SCSCL : public SCSerial
{
public:
  SCSCL();
  SCSCL(uint8_t End);
  SCSCL(uint8_t End, uint8_t Level);
  virtual int WritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed = 0);      //普通写单个舵机位置指令
  virtual int RegWritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed = 0);      //异步写单个舵机位置指令(RegWriteAction生效)
  virtual void SyncWritePos(
    uint8_t ID[], uint8_t IDN, uint16_t Position[], uint16_t Time[],
    uint16_t Speed[]);                                                                                               //同步写多个舵机位置指令
  virtual int PWMMode(uint8_t ID);      //PWM输出模式
  virtual int WritePWM(uint8_t ID, int16_t pwmOut);      //PWM输出模式指令
  virtual int EnableTorque(uint8_t ID, uint8_t Enable);      //扭矩控制指令
  virtual int unLockEprom(uint8_t ID);      //eprom解锁
  virtual int LockEprom(uint8_t ID);      //eprom加锁
  virtual int FeedBack(int ID);      //反馈舵机信息
  virtual int ReadPos(int ID);      //读位置
  virtual int ReadSpeed(int ID);      //读速度
  virtual int ReadLoad(int ID);      //读输出至电机的电压百分比(0~1000)
  virtual int ReadVoltage(int ID);      //读电压
  virtual int ReadTemper(int ID);      //读温度
  virtual int ReadMove(int ID);      //读移动状态
  virtual int ReadCurrent(int ID);      //读电流

private:
  uint8_t Mem[SCSCL_PRESENT_CURRENT_H - SCSCL_PRESENT_POSITION_L + 1];
};

#endif  // FEETECH_LIB__SCSCL_HPP_

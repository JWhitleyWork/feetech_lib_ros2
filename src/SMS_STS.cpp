/*
 * SMS_STS.cpp
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2021.12.8
 * 作者:
 */

#include "feetech_lib/SMS_STS.hpp"

#include <cstdint>

SMS_STS::SMS_STS()
{
  End = 0;
}

SMS_STS::SMS_STS(uint8_t End)
:SCSerial(End)
{
}

SMS_STS::SMS_STS(uint8_t End, uint8_t Level)
:SCSerial(End, Level)
{
}

int32_t SMS_STS::WriteID(uint8_t ID, uint8_t NewID)
{
  return writeByte(ID, SMS_STS_ID, NewID);
}

int32_t SMS_STS::WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
  if(Position < 0) {
    Position = -Position;
    Position |= (1 << 15);
  }
  uint8_t bBuf[7];
  bBuf[0] = ACC;
  Host2SCS(bBuf + 1, bBuf + 2, Position);
  Host2SCS(bBuf + 3, bBuf + 4, 0);
  Host2SCS(bBuf + 5, bBuf + 6, Speed);

  return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

int32_t SMS_STS::WriteTorqueLimit(uint8_t ID, uint16_t TorqueLimit)
{
  uint8_t bBuf[2];
  Host2SCS(bBuf, bBuf + 1, TorqueLimit);
  return genWrite(ID, SMSBL_TORQUE_LIMIT_L, bBuf, 2);
}

int32_t SMS_STS::RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
  if(Position < 0) {
    Position = -Position;
    Position |= (1 << 15);
  }
  uint8_t bBuf[7];
  bBuf[0] = ACC;
  Host2SCS(bBuf + 1, bBuf + 2, Position);
  Host2SCS(bBuf + 3, bBuf + 4, 0);
  Host2SCS(bBuf + 5, bBuf + 6, Speed);

  return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

void SMS_STS::SyncWritePosEx(
  uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[],
  uint8_t ACC[])
{

  uint8_t offbuf[IDN][7];
  for(uint8_t i = 0; i < IDN; i++) {
    if(Position[i] < 0) {
      Position[i] = -Position[i];
      Position[i] |= (1 << 15);
    }
    uint8_t bBuf[7];
    uint16_t V;
    if(Speed) {
      V = Speed[i];
    } else {
      V = 0;
    }
    if(ACC) {
      bBuf[0] = ACC[i];
    } else {
      bBuf[0] = 0;
    }
    Host2SCS(bBuf + 1, bBuf + 2, Position[i]);
    Host2SCS(bBuf + 3, bBuf + 4, 0);
    Host2SCS(bBuf + 5, bBuf + 6, V);
    memcpy(offbuf[i], bBuf, 7);
  }
  snycWrite(ID, IDN, SMS_STS_ACC, (uint8_t *)offbuf, 7);
}

int32_t SMS_STS::WheelMode(uint8_t ID)
{
  return writeByte(ID, SMS_STS_MODE, 1);
}

int32_t SMS_STS::WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
  if(Speed < 0) {
    Speed = -Speed;
    Speed |= (1 << 15);
  }
  uint8_t bBuf[2];
  bBuf[0] = ACC;
  genWrite(ID, SMS_STS_ACC, bBuf, 1);
  Host2SCS(bBuf + 0, bBuf + 1, Speed);

  return genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
}

int32_t SMS_STS::PWMMode(uint8_t ID)
{
  return writeByte(ID, SMS_STS_MODE, 2);
}

int32_t SMS_STS::PositionMode(uint8_t ID)
{
  return writeByte(ID, SMS_STS_MODE, 4);
}

int32_t SMS_STS::WritePWM(uint8_t ID, int16_t pwmOut)
{
  if(pwmOut < 0) {
    pwmOut = -pwmOut;
    pwmOut |= (1 << 10);
  }
  uint8_t bBuf[2];
  Host2SCS(bBuf + 0, bBuf + 1, pwmOut);
  return genWrite(ID, SMS_STS_GOAL_TIME_L, bBuf, 2);
}

int32_t SMS_STS::EnableTorque(uint8_t ID, uint8_t Enable)
{
  return writeByte(ID, SMS_STS_TORQUE_ENABLE, Enable);
}

int32_t SMS_STS::unLockEprom(uint8_t ID)
{
  return writeByte(ID, SMS_STS_LOCK, 0);
}

int32_t SMS_STS::LockEprom(uint8_t ID)
{
  return writeByte(ID, SMS_STS_LOCK, 1);
}

int32_t SMS_STS::CalibrationOfs(uint8_t ID)
{
  return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

int32_t SMS_STS::FeedBack(int32_t ID)
{
  int32_t nLen = Read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
  if(nLen != sizeof(Mem)) {
    Err = 1;
    return -1;
  }
  Err = 0;
  return nLen;
}

int32_t SMS_STS::ReadPos(int32_t ID)
{
  int32_t Pos = -1;
  if(ID == -1) {
    Pos = Mem[SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L];
    Pos <<= 8;
    Pos |= Mem[SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Pos = readWord(ID, SMS_STS_PRESENT_POSITION_L);
    if(Pos == -1) {
      Err = 1;
    }
  }
  if(!Err && (Pos & (1 << 15))) {
    Pos = -(Pos & ~(1 << 15));
  }

  return Pos;
}

int32_t SMS_STS::ReadSpeed(int32_t ID)
{
  int32_t Speed = -1;
  if(ID == -1) {
    Speed = Mem[SMS_STS_PRESENT_SPEED_H - SMS_STS_PRESENT_POSITION_L];
    Speed <<= 8;
    Speed |= Mem[SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Speed = readWord(ID, SMS_STS_PRESENT_SPEED_L);
    if(Speed == -1) {
      Err = 1;
      return -1;
    }
  }
  if(!Err && (Speed & (1 << 15))) {
    Speed = -(Speed & ~(1 << 15));
  }
  return Speed;
}

int32_t SMS_STS::ReadLoad(int32_t ID)
{
  int32_t Load = -1;
  if(ID == -1) {
    Load = Mem[SMS_STS_PRESENT_LOAD_H - SMS_STS_PRESENT_POSITION_L];
    Load <<= 8;
    Load |= Mem[SMS_STS_PRESENT_LOAD_L - SMS_STS_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Load = readWord(ID, SMS_STS_PRESENT_LOAD_L);
    if(Load == -1) {
      Err = 1;
    }
  }
  if(!Err && (Load & (1 << 10))) {
    Load = -(Load & ~(1 << 10));
  }
  return Load;
}

int32_t SMS_STS::ReadVoltage(int32_t ID)
{
  int32_t Voltage = -1;
  if(ID == -1) {
    Voltage = Mem[SMS_STS_PRESENT_VOLTAGE - SMS_STS_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Voltage = readByte(ID, SMS_STS_PRESENT_VOLTAGE);
    if(Voltage == -1) {
      Err = 1;
    }
  }
  return Voltage;
}

int32_t SMS_STS::ReadTemper(int32_t ID)
{
  int32_t Temper = -1;
  if(ID == -1) {
    Temper = Mem[SMS_STS_PRESENT_TEMPERATURE - SMS_STS_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Temper = readByte(ID, SMS_STS_PRESENT_TEMPERATURE);
    if(Temper == -1) {
      Err = 1;
    }
  }
  return Temper;
}

int32_t SMS_STS::ReadMove(int32_t ID)
{
  int32_t Move = -1;
  if(ID == -1) {
    Move = Mem[SMS_STS_MOVING - SMS_STS_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Move = readByte(ID, SMS_STS_MOVING);
    if(Move == -1) {
      Err = 1;
    }
  }
  return Move;
}

int32_t SMS_STS::ReadCurrent(int32_t ID)
{
  int32_t Current = -1;
  if(ID == -1) {
    Current = Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L];
    Current <<= 8;
    Current |= Mem[SMS_STS_PRESENT_CURRENT_L - SMS_STS_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Current = readWord(ID, SMS_STS_PRESENT_CURRENT_L);
    if(Current == -1) {
      Err = 1;
      return -1;
    }
  }
  if(!Err && (Current & (1 << 15))) {
    Current = -(Current & ~(1 << 15));
  }
  return Current;
}

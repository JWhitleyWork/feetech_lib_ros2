// Copyright 2025 Electrified Autonomy, LLC  // NOLINT
//
// SCSCL.cpp
// 飞特SCSCL系列串行舵机应用层程序
// 日期: 2020.6.17
// 作者:

#include "feetech_lib/SCSCL.hpp"

#include <cstdint>
#include <cstring>

SCSCL::SCSCL()
{
  End = 1;
}

SCSCL::SCSCL(uint8_t End)
:SCSerial(End)
{
}

SCSCL::SCSCL(uint8_t End, uint8_t Level)
:SCSerial(End, Level)
{
}

int32_t SCSCL::WritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
  uint8_t bBuf[6];
  Host2SCS(bBuf + 0, bBuf + 1, Position);
  Host2SCS(bBuf + 2, bBuf + 3, Time);
  Host2SCS(bBuf + 4, bBuf + 5, Speed);

  return genWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

int32_t SCSCL::RegWritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
  uint8_t bBuf[6];
  Host2SCS(bBuf + 0, bBuf + 1, Position);
  Host2SCS(bBuf + 2, bBuf + 3, Time);
  Host2SCS(bBuf + 4, bBuf + 5, Speed);

  return regWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

void SCSCL::SyncWritePos(
  uint8_t ID[], uint8_t IDN, uint16_t Position[], uint16_t Time[],
  uint16_t Speed[])
{
  uint8_t offbuf[IDN][6];
  for(uint8_t i = 0; i < IDN; i++) {
    uint8_t bBuf[6];
    uint16_t T, V;
    if(Time) {
      T = Time[i];
    } else {
      T = 0;
    }
    if(Speed) {
      V = Speed[i];
    } else {
      V = 0;
    }
    Host2SCS(bBuf + 0, bBuf + 1, Position[i]);
    Host2SCS(bBuf + 2, bBuf + 3, T);
    Host2SCS(bBuf + 4, bBuf + 5, V);
    memcpy(offbuf[i], bBuf, 6);
  }
  syncWrite(ID, IDN, SCSCL_GOAL_POSITION_L, reinterpret_cast<uint8_t *>(offbuf), 6);
}

int32_t SCSCL::PWMMode(uint8_t ID)
{
  uint8_t bBuf[4];
  bBuf[0] = 0;
  bBuf[1] = 0;
  bBuf[2] = 0;
  bBuf[3] = 0;
  return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

int32_t SCSCL::WritePWM(uint8_t ID, int16_t pwmOut)
{
  if(pwmOut < 0) {
    pwmOut = -pwmOut;
    pwmOut |= (1 << 10);
  }
  uint8_t bBuf[2];
  Host2SCS(bBuf + 0, bBuf + 1, pwmOut);

  return genWrite(ID, SCSCL_GOAL_TIME_L, bBuf, 2);
}

int32_t SCSCL::EnableTorque(uint8_t ID, uint8_t Enable)
{
  return writeByte(ID, SCSCL_TORQUE_ENABLE, Enable);
}

int32_t SCSCL::unLockEprom(uint8_t ID)
{
  return writeByte(ID, SCSCL_LOCK, 0);
}

int32_t SCSCL::LockEprom(uint8_t ID)
{
  return writeByte(ID, SCSCL_LOCK, 1);
}

int32_t SCSCL::FeedBack(int32_t ID)
{
  int32_t nLen = Read(ID, SCSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
  if(nLen != sizeof(Mem)) {
    Err = 1;
    return -1;
  }
  Err = 0;
  return nLen;
}

int32_t SCSCL::ReadPos(int32_t ID)
{
  int32_t Pos = -1;
  if(ID == -1) {
    Pos = Mem[SCSCL_PRESENT_POSITION_L - SCSCL_PRESENT_POSITION_L];
    Pos <<= 8;
    Pos |= Mem[SCSCL_PRESENT_POSITION_H - SCSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Pos = readWord(ID, SCSCL_PRESENT_POSITION_L);
    if(Pos == -1) {
      Err = 1;
    }
  }
  return Pos;
}

int32_t SCSCL::ReadSpeed(int32_t ID)
{
  int32_t Speed = -1;
  if(ID == -1) {
    Speed = Mem[SCSCL_PRESENT_SPEED_L - SCSCL_PRESENT_POSITION_L];
    Speed <<= 8;
    Speed |= Mem[SCSCL_PRESENT_SPEED_H - SCSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Speed = readWord(ID, SCSCL_PRESENT_SPEED_L);
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

int32_t SCSCL::ReadLoad(int32_t ID)
{
  int32_t Load = -1;
  if(ID == -1) {
    Load = Mem[SCSCL_PRESENT_LOAD_L - SCSCL_PRESENT_POSITION_L];
    Load <<= 8;
    Load |= Mem[SCSCL_PRESENT_LOAD_H - SCSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Load = readWord(ID, SCSCL_PRESENT_LOAD_L);
    if(Load == -1) {
      Err = 1;
    }
  }
  if(!Err && (Load & (1 << 10))) {
    Load = -(Load & ~(1 << 10));
  }
  return Load;
}

int32_t SCSCL::ReadVoltage(int32_t ID)
{
  int32_t Voltage = -1;
  if(ID == -1) {
    Voltage = Mem[SCSCL_PRESENT_VOLTAGE - SCSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Voltage = readByte(ID, SCSCL_PRESENT_VOLTAGE);
    if(Voltage == -1) {
      Err = 1;
    }
  }
  return Voltage;
}

int32_t SCSCL::ReadTemper(int32_t ID)
{
  int32_t Temper = -1;
  if(ID == -1) {
    Temper = Mem[SCSCL_PRESENT_TEMPERATURE - SCSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Temper = readByte(ID, SCSCL_PRESENT_TEMPERATURE);
    if(Temper == -1) {
      Err = 1;
    }
  }
  return Temper;
}

int32_t SCSCL::ReadMove(int32_t ID)
{
  int32_t Move = -1;
  if(ID == -1) {
    Move = Mem[SCSCL_MOVING - SCSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Move = readByte(ID, SCSCL_MOVING);
    if(Move == -1) {
      Err = 1;
    }
  }
  return Move;
}

int32_t SCSCL::ReadCurrent(int32_t ID)
{
  int32_t Current = -1;
  if(ID == -1) {
    Current = Mem[SCSCL_PRESENT_CURRENT_L - SCSCL_PRESENT_POSITION_L];
    Current <<= 8;
    Current |= Mem[SCSCL_PRESENT_CURRENT_H - SCSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Current = readWord(ID, SCSCL_PRESENT_CURRENT_L);
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

/*
 * SMSCL.cpp
 * 飞特SMSCL系列串行舵机应用层程序
 * 日期: 2020.6.17
 * 作者:
 */

#include "feetech_lib/SMSCL.hpp"

#include <cstdint>
#include <cstring>

SMSCL::SMSCL()
{
  End = 0;
}

SMSCL::SMSCL(uint8_t End)
:SCSerial(End)
{
}

SMSCL::SMSCL(uint8_t End, uint8_t Level)
:SCSerial(End, Level)
{
}

int32_t SMSCL::WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
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

  return genWrite(ID, SMSCL_ACC, bBuf, 7);
}

int32_t SMSCL::RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
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

  return regWrite(ID, SMSCL_ACC, bBuf, 7);
}

void SMSCL::SyncWritePosEx(
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
  snycWrite(ID, IDN, SMSCL_ACC, (uint8_t *)offbuf, 7);
}

int32_t SMSCL::WheelMode(uint8_t ID)
{
  return writeByte(ID, SMSCL_MODE, 1);
}

int32_t SMSCL::WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
  if(Speed < 0) {
    Speed = -Speed;
    Speed |= (1 << 15);
  }
  uint8_t bBuf[2];
  bBuf[0] = ACC;
  genWrite(ID, SMSCL_ACC, bBuf, 1);
  Host2SCS(bBuf + 0, bBuf + 1, Speed);

  return genWrite(ID, SMSCL_GOAL_SPEED_L, bBuf, 2);
}

int32_t SMSCL::EnableTorque(uint8_t ID, uint8_t Enable)
{
  return writeByte(ID, SMSCL_TORQUE_ENABLE, Enable);
}

int32_t SMSCL::unLockEprom(uint8_t ID)
{
  return writeByte(ID, SMSCL_LOCK, 0);
}

int32_t SMSCL::LockEprom(uint8_t ID)
{
  return writeByte(ID, SMSCL_LOCK, 1);
}

int32_t SMSCL::CalibrationOfs(uint8_t ID)
{
  return writeByte(ID, SMSCL_TORQUE_ENABLE, 128);
}

int32_t SMSCL::FeedBack(int32_t ID)
{
  int32_t nLen = Read(ID, SMSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
  if(nLen != sizeof(Mem)) {
    Err = 1;
    return -1;
  }
  Err = 0;
  return nLen;
}

int32_t SMSCL::ReadPos(int32_t ID)
{
  int32_t Pos = -1;
  if(ID == -1) {
    Pos = Mem[SMSCL_PRESENT_POSITION_H - SMSCL_PRESENT_POSITION_L];
    Pos <<= 8;
    Pos |= Mem[SMSCL_PRESENT_POSITION_L - SMSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Pos = readWord(ID, SMSCL_PRESENT_POSITION_L);
    if(Pos == -1) {
      Err = 1;
    }
  }
  if(!Err && (Pos & (1 << 15))) {
    Pos = -(Pos & ~(1 << 15));
  }

  return Pos;
}

int32_t SMSCL::ReadSpeed(int32_t ID)
{
  int32_t Speed = -1;
  if(ID == -1) {
    Speed = Mem[SMSCL_PRESENT_SPEED_H - SMSCL_PRESENT_POSITION_L];
    Speed <<= 8;
    Speed |= Mem[SMSCL_PRESENT_SPEED_L - SMSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Speed = readWord(ID, SMSCL_PRESENT_SPEED_L);
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

int32_t SMSCL::ReadLoad(int32_t ID)
{
  int32_t Load = -1;
  if(ID == -1) {
    Load = Mem[SMSCL_PRESENT_LOAD_H - SMSCL_PRESENT_POSITION_L];
    Load <<= 8;
    Load |= Mem[SMSCL_PRESENT_LOAD_L - SMSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Load = readWord(ID, SMSCL_PRESENT_LOAD_L);
    if(Load == -1) {
      Err = 1;
    }
  }
  if(!Err && (Load & (1 << 10))) {
    Load = -(Load & ~(1 << 10));
  }
  return Load;
}

int32_t SMSCL::ReadVoltage(int32_t ID)
{
  int32_t Voltage = -1;
  if(ID == -1) {
    Voltage = Mem[SMSCL_PRESENT_VOLTAGE - SMSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Voltage = readByte(ID, SMSCL_PRESENT_VOLTAGE);
    if(Voltage == -1) {
      Err = 1;
    }
  }
  return Voltage;
}

int32_t SMSCL::ReadTemper(int32_t ID)
{
  int32_t Temper = -1;
  if(ID == -1) {
    Temper = Mem[SMSCL_PRESENT_TEMPERATURE - SMSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Temper = readByte(ID, SMSCL_PRESENT_TEMPERATURE);
    if(Temper == -1) {
      Err = 1;
    }
  }
  return Temper;
}

int32_t SMSCL::ReadMove(int32_t ID)
{
  int32_t Move = -1;
  if(ID == -1) {
    Move = Mem[SMSCL_MOVING - SMSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Move = readByte(ID, SMSCL_MOVING);
    if(Move == -1) {
      Err = 1;
    }
  }
  return Move;
}

int32_t SMSCL::ReadCurrent(int32_t ID)
{
  int32_t Current = -1;
  if(ID == -1) {
    Current = Mem[SMSCL_PRESENT_CURRENT_H - SMSCL_PRESENT_POSITION_L];
    Current <<= 8;
    Current |= Mem[SMSCL_PRESENT_CURRENT_L - SMSCL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Current = readWord(ID, SMSCL_PRESENT_CURRENT_L);
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

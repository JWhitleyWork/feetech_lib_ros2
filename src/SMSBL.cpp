/*
 * SMSBL.cpp
 * 飞特SMSBL系列串行舵机应用层程序
 * 日期: 2020.6.17
 * 作者:
 */

#include "feetech_lib/SMSBL.hpp"

#include <cstdint>
#include <cstring>

SMSBL::SMSBL()
{
  End = 0;
}

SMSBL::SMSBL(uint8_t End)
:SCSerial(End)
{
}

SMSBL::SMSBL(uint8_t End, uint8_t Level)
:SCSerial(End, Level)
{
}

int32_t SMSBL::WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
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

  return genWrite(ID, SMSBL_ACC, bBuf, 7);
}

int32_t SMSBL::RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
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

  return regWrite(ID, SMSBL_ACC, bBuf, 7);
}

void SMSBL::SyncWritePosEx(
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
  snycWrite(ID, IDN, SMSBL_ACC, (uint8_t *)offbuf, 7);
}

int32_t SMSBL::WheelMode(uint8_t ID)
{
  return writeByte(ID, SMSBL_MODE, 1);
}

int32_t SMSBL::WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
  if(Speed < 0) {
    Speed = -Speed;
    Speed |= (1 << 15);
  }
  uint8_t bBuf[2];
  bBuf[0] = ACC;
  genWrite(ID, SMSBL_ACC, bBuf, 1);
  Host2SCS(bBuf + 0, bBuf + 1, Speed);

  return genWrite(ID, SMSBL_GOAL_SPEED_L, bBuf, 2);
}

int32_t SMSBL::EnableTorque(uint8_t ID, uint8_t Enable)
{
  return writeByte(ID, SMSBL_TORQUE_ENABLE, Enable);
}

int32_t SMSBL::unLockEprom(uint8_t ID)
{
  return writeByte(ID, SMSBL_LOCK, 0);
}

int32_t SMSBL::LockEprom(uint8_t ID)
{
  return writeByte(ID, SMSBL_LOCK, 1);
}

int32_t SMSBL::CalibrationOfs(uint8_t ID)
{
  return writeByte(ID, SMSBL_TORQUE_ENABLE, 128);
}

int32_t SMSBL::FeedBack(int32_t ID)
{
  int32_t nLen = Read(ID, SMSBL_PRESENT_POSITION_L, Mem, sizeof(Mem));
  if(nLen != sizeof(Mem)) {
    Err = 1;
    return -1;
  }
  Err = 0;
  return nLen;
}

int32_t SMSBL::ReadPos(int32_t ID)
{
  int32_t Pos = -1;
  if(ID == -1) {
    Pos = Mem[SMSBL_PRESENT_POSITION_H - SMSBL_PRESENT_POSITION_L];
    Pos <<= 8;
    Pos |= Mem[SMSBL_PRESENT_POSITION_L - SMSBL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Pos = readWord(ID, SMSBL_PRESENT_POSITION_L);
    if(Pos == -1) {
      Err = 1;
    }
  }
  if(!Err && (Pos & (1 << 15))) {
    Pos = -(Pos & ~(1 << 15));
  }

  return Pos;
}

int32_t SMSBL::ReadSpeed(int32_t ID)
{
  int32_t Speed = -1;
  if(ID == -1) {
    Speed = Mem[SMSBL_PRESENT_SPEED_H - SMSBL_PRESENT_POSITION_L];
    Speed <<= 8;
    Speed |= Mem[SMSBL_PRESENT_SPEED_L - SMSBL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Speed = readWord(ID, SMSBL_PRESENT_SPEED_L);
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

int32_t SMSBL::ReadLoad(int32_t ID)
{
  int32_t Load = -1;
  if(ID == -1) {
    Load = Mem[SMSBL_PRESENT_LOAD_H - SMSBL_PRESENT_POSITION_L];
    Load <<= 8;
    Load |= Mem[SMSBL_PRESENT_LOAD_L - SMSBL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Load = readWord(ID, SMSBL_PRESENT_LOAD_L);
    if(Load == -1) {
      Err = 1;
    }
  }
  if(!Err && (Load & (1 << 10))) {
    Load = -(Load & ~(1 << 10));
  }
  return Load;
}

int32_t SMSBL::ReadVoltage(int32_t ID)
{
  int32_t Voltage = -1;
  if(ID == -1) {
    Voltage = Mem[SMSBL_PRESENT_VOLTAGE - SMSBL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Voltage = readByte(ID, SMSBL_PRESENT_VOLTAGE);
    if(Voltage == -1) {
      Err = 1;
    }
  }
  return Voltage;
}

int32_t SMSBL::ReadTemper(int32_t ID)
{
  int32_t Temper = -1;
  if(ID == -1) {
    Temper = Mem[SMSBL_PRESENT_TEMPERATURE - SMSBL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Temper = readByte(ID, SMSBL_PRESENT_TEMPERATURE);
    if(Temper == -1) {
      Err = 1;
    }
  }
  return Temper;
}

int32_t SMSBL::ReadMove(int32_t ID)
{
  int32_t Move = -1;
  if(ID == -1) {
    Move = Mem[SMSBL_MOVING - SMSBL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Move = readByte(ID, SMSBL_MOVING);
    if(Move == -1) {
      Err = 1;
    }
  }
  return Move;
}

int32_t SMSBL::ReadCurrent(int32_t ID)
{
  int32_t Current = -1;
  if(ID == -1) {
    Current = Mem[SMSBL_PRESENT_CURRENT_H - SMSBL_PRESENT_POSITION_L];
    Current <<= 8;
    Current |= Mem[SMSBL_PRESENT_CURRENT_L - SMSBL_PRESENT_POSITION_L];
  } else {
    Err = 0;
    Current = readWord(ID, SMSBL_PRESENT_CURRENT_L);
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

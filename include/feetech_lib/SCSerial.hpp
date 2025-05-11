/*
 * SCSerial.hpp
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者:
 */

#ifndef FEETECH_LIB__SCSERIAL_HPP_
#define FEETECH_LIB__SCSERIAL_HPP_

#include "feetech_lib/SCS.hpp"

#include <cstdint>
#include <cstdio>
#include <string>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

class SCSerial : public SCS
{
public:
  SCSerial();
  SCSerial(uint8_t End);
  SCSerial(uint8_t End, uint8_t Level);

protected:
  int32_t writeSCS(uint8_t *nDat, int32_t nLen);  //输出nLen字节
  int32_t readSCS(uint8_t *nDat, int32_t nLen);  //输入nLen字节
  int32_t writeSCS(uint8_t bDat);  //输出1字节
  void rFlushSCS();
  void wFlushSCS();

public:
  unsigned long int32_t IOTimeOut;  //输入输出超时
  int32_t Err;

public:
  virtual int32_t getErr() {return Err;}
  virtual int32_t setBaudRate(int32_t baudRate);
  virtual bool begin(int32_t baudRate, const int8_t * serialPort);
  virtual void end();

protected:
  int32_t fd;  //serial port handle
  struct termios orgopt;  //fd ort opt
  struct termios curopt;  //fd cur opt
  uint8_t txBuf[255];
  int32_t txBufLen;
};

#endif  // FEETECH_LIB__SCSERIAL_HPP_

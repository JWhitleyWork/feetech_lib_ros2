/*
 * SCSerial.cpp
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者:
 */

#include "feetech_lib/SCSerial.hpp"

#include <cstdint>

SCSerial::SCSerial()
{
  IOTimeOut = 100;
  fd = -1;
  txBufLen = 0;
}

SCSerial::SCSerial(uint8_t End)
:SCS(End)
{
  IOTimeOut = 100;
  fd = -1;
  txBufLen = 0;
}

SCSerial::SCSerial(uint8_t End, uint8_t Level)
:SCS(End, Level)
{
  IOTimeOut = 100;
  fd = -1;
  txBufLen = 0;
}

bool SCSerial::begin(int32_t baudRate, const int8_t * serialPort)
{
  if(fd != -1) {
    close(fd);
    fd = -1;
  }
  //printf("servo port:%s\n", serialPort);
  if(serialPort == NULL) {
    return false;
  }
  fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if(fd == -1) {
    perror("open:");
    return false;
  }
  fcntl(fd, F_SETFL, FNDELAY);
  tcgetattr(fd, &orgopt);
  tcgetattr(fd, &curopt);
  speed_t CR_BAUDRATE;
  switch(baudRate) {
    case 9600:
      CR_BAUDRATE = B9600;
      break;
    case 19200:
      CR_BAUDRATE = B19200;
      break;
    case 38400:
      CR_BAUDRATE = B38400;
      break;
    case 57600:
      CR_BAUDRATE = B57600;
      break;
    case 115200:
      CR_BAUDRATE = B115200;
      break;
    case 500000:
      CR_BAUDRATE = B500000;
      break;
    case 1000000:
      CR_BAUDRATE = B1000000;
      break;
    default:
      CR_BAUDRATE = B115200;
      break;
  }
  cfsetispeed(&curopt, CR_BAUDRATE);
  cfsetospeed(&curopt, CR_BAUDRATE);

  printf("serial speed %d\n", baudRate);
    //Mostly 8N1
  curopt.c_cflag &= ~PARENB;
  curopt.c_cflag &= ~CSTOPB;
  curopt.c_cflag &= ~CSIZE;
  curopt.c_cflag |= CS8;
  curopt.c_cflag |= CREAD;
  curopt.c_cflag |= CLOCAL;  //disable modem statuc check
  cfmakeraw(&curopt);  //make raw mode
  curopt.c_iflag &= ~(BRKint32_t | ICRNL | INPCK | ISTRIP | IXON);
  if(tcsetattr(fd, TCSANOW, &curopt) == 0) {
    return true;
  } else {
    perror("tcsetattr:");
    return false;
  }
}

int32_t SCSerial::setBaudRate(int32_t baudRate)
{
  if(fd == -1) {
    return -1;
  }
  tcgetattr(fd, &orgopt);
  tcgetattr(fd, &curopt);
  speed_t CR_BAUDRATE;
  switch(baudRate) {
    case 9600:
      CR_BAUDRATE = B9600;
      break;
    case 19200:
      CR_BAUDRATE = B19200;
      break;
    case 38400:
      CR_BAUDRATE = B38400;
      break;
    case 57600:
      CR_BAUDRATE = B57600;
      break;
    case 115200:
      CR_BAUDRATE = B115200;
      break;
    case 230400:
      CR_BAUDRATE = B230400;
      break;
    case 500000:
      CR_BAUDRATE = B500000;
      break;
    default:
      break;
  }
  cfsetispeed(&curopt, CR_BAUDRATE);
  cfsetospeed(&curopt, CR_BAUDRATE);
  return 1;
}

int32_t SCSerial::readSCS(uint8_t *nDat, int32_t nLen)
{
  int32_t fs_sel;
  fd_set fs_read;
  int32_t rvLen = 0;

  struct timeval time;

  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);

  time.tv_sec = 0;
  time.tv_usec = IOTimeOut * 1000;

    //使用select实现串口的多路通信
  while(1) {
    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
    if(fs_sel) {
      rvLen += read(fd, nDat + rvLen, nLen - rvLen);
      //printf("nLen = %d rvLen = %d\n", nLen, rvLen);
      if(rvLen < nLen) {
        continue;
      } else {
        return rvLen;
      }
    } else {
      //printf("serial read fd read return 0\n");
      return rvLen;
    }
  }
}

int32_t SCSerial::writeSCS(uint8_t *nDat, int32_t nLen)
{
  while(nLen--) {
    txBuf[txBufLen++] = *nDat++;
  }
  return txBufLen;
}

int32_t SCSerial::writeSCS(uint8_t bDat)
{
  txBuf[txBufLen++] = bDat;
  return txBufLen;
}

void SCSerial::rFlushSCS()
{
  tcflush(fd, TCIFLUSH);
}

void SCSerial::wFlushSCS()
{
  if(txBufLen) {
    txBufLen = write(fd, txBuf, txBufLen);
    txBufLen = 0;
  }
}

void SCSerial::end()
{
  fd = -1;
  close(fd);
}

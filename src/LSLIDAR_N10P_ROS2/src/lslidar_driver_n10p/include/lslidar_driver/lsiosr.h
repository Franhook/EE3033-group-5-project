/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSM10_N10
@filename: lsiosr.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           22-10-24      li          new
*******************************************************/
#ifndef LSIOSR_H
#define LSIOSR_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <iostream>

//Baud rate
#define     BAUD_230400     230400
#define     BAUD_460800     460800
#define     BAUD_500000     500000
#define     BAUD_921600     921600

//Parity bit
#define     PARITY_ODD    	'O' //Odd
#define     PARITY_EVEN   	'E' //Even
#define     PARITY_NONE   	'N' //No parity

//Stop bit
#define     STOP_BIT_1     	1
#define     STOP_BIT_2     	2

//Data bits
#define     DATA_BIT_7     	7
#define     DATA_BIT_8     	8

namespace lslidar_driver
{
class LSIOSR{
public:
  static LSIOSR* instance(std::string name, int speed, int fd = 0);

  ~LSIOSR();

  /* Read data from the serial port */
  int read(unsigned char *buffer, int length, int timeout = 30);

  /* Send data through the serial port */
  int send(const char* buffer, int length, int timeout = 30);

  /* Empty serial port input buffer */
  void flushinput();

  /* Initialize the serial port */
  int init();

  int close();

  /* Get the serial-port number */
  std::string getPort();

  /* Set the serial-port number */
  int setPortName(std::string name);

private:
  LSIOSR(std::string name, int speed, int fd);

  int waitWritable(int millis);
  int waitReadable(int millis);

  /* Serial-port configuration function */
  int setOpt(int nBits, uint8_t nEvent, int nStop);

  std::string port_;
  int baud_rate_;

  int fd_;
};
}
#endif


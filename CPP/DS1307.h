/*
  DS1307.h - Real-Time Clock/Calendar library for DefendLineII

  Copyright (c) 2012 Dmitry Pakhomenko.
  pahomenko@gmail.com
  http://atmega.magictale.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef DS1307_h
#define DS1307_h

#include <inttypes.h>

#include "Wire.h"

#define DS1307_I2C_ADDR 0x68

typedef struct struct_RtcTime
{	
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t weekday;
	uint8_t month;
	uint16_t year;
} RtcTimeType;

class DS1307
{
  private:
  public:
    DS1307();
	uint8_t init(uint8_t i2cAddr);
	uint8_t getTime(uint8_t i2cAddr, RtcTimeType* rtc);
	void setTime(uint8_t i2cAddr, RtcTimeType* rtc);
};

extern DS1307 RTClock;

#endif


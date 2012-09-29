/*
  DS1307.cpp - Real-Time Clock/Calendar library for DefendLineII

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

#include "DS1307.h"

// Initialize Class Variables //////////////////////////////////////////////////

// Constructors ////////////////////////////////////////////////////////////////

DS1307::DS1307()
{
}

// Public Methods //////////////////////////////////////////////////////////////
uint8_t DS1307::init(uint8_t i2cAddr)
{
	Wire.beginTransmission(i2cAddr);
	return Wire.endTransmission();
}

uint8_t DS1307::getTime(uint8_t i2cAddr, RtcTimeType* rtc)
{
	Wire.beginTransmission(i2cAddr);
	//Wire.send(2);
	Wire.send(0);
	Wire.endTransmission();

	Wire.requestFrom(i2cAddr, (uint8_t)7);

	uint8_t res = 0; 	

	if(7 <= Wire.available())
	{
        rtc->seconds = Wire.receive() & 0x7F;
		rtc->minutes = Wire.receive() & 0x7F;
		rtc->hours = Wire.receive() & 0x3F;
		rtc->weekday = Wire.receive() & 0x7;
		rtc->day = Wire.receive() & 0x3F;
		rtc->month = Wire.receive() & 0x1F;
		rtc->year = 0x2000 + Wire.receive();
  	}else res = -1;

	return res;
}

void DS1307::setTime(uint8_t i2cAddr, RtcTimeType* rtc)
{
	Wire.beginTransmission(i2cAddr);
        Wire.send(0);

   	Wire.send(rtc->seconds & 0x7F);
	Wire.send(rtc->minutes & 0x7F);
	Wire.send(rtc->hours & 0x3F);
	Wire.send(rtc->weekday & 0x7);
	Wire.send(rtc->day & 0x3F);
	Wire.send(rtc->month & 0x1F);
	Wire.send((uint8_t)(rtc->year - 0x2000));

	Wire.endTransmission();
}

// Preinstantiate Objects //////////////////////////////////////////////////////

DS1307 RTClock = DS1307();

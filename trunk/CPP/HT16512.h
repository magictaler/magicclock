/*
  HT16512.h - Driver for HT16512 VFD Controller

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

#ifndef HT16512_H
#define HT16512_H

#include  <inttypes.h>
#include "Print.h"

#define VFD_SEGMENTS 15
#define VFD_DIGITS 7
#define VFD_BYTES_PER_DIGIT 2
#define VFD_DIMMING_MAX 7
#define VFD_MAX_TEST_DELAY 100
#define VFD_FLASHING_DELAY 2
#define VFD_ROTOR_TEST_REVOLUTIONS 10
#define VFD_ROTOR_FRAMES 4
#define VFD_SCROLL_FRAMES 3

#define VFD_FLASH_NONE 0
#define VFD_FLASH_ALL 0xFF


class HT16512 : public Print 
{
    public:
        enum enum_TestState 
        {   
            NOT_STARTED,    
            COLUMN_TEST,
            SEGMENT_TEST,
            DIMMING_TEST,
            GLOWING_TEST,
            CHARSET_TEST,
            CHARSET_TEST2,
            ROTOR_TEST,
            SLASH_EFFECT,
            SCROLL_EFFECT,
            COMPLETED    
        };

        HT16512(uint8_t cs, uint8_t sclk, uint8_t data);
        //Harware commands
        void reset();
        void displayOnCmd(uint8_t dimming);
        void displayOffCmd();
        void displayWriteCmd(uint8_t addr_inc, uint8_t nodata);
        void addrSetCmd(uint8_t addr);
        void readKeyDataCmd(uint8_t nodata);
        void readSwDataCmd(uint8_t nodata);
        //Low level function for sending command
        void command(uint8_t value, uint8_t nodata);
        //Low level function for sending data
        void data_out(uint8_t value, uint8_t init_cs, uint8_t finalise_cs);
        //Low level function for receiving data
        uint8_t data_in(uint8_t init_cs, uint8_t finalise_cs);

        uint8_t readKeyData();
        uint8_t readSwData();
        void setLEDoutputs(uint8_t value);

        //Send ASCII value to display
        void write(uint8_t value);
        //Send ASCII value to intermediate buffer
        void write_f(uint8_t value, uint8_t dstIndex, uint8_t dispColumn);

        //Transfer array to display
        uint8_t write(uint8_t* buffer, uint8_t dstIndex, uint8_t len);
        //Transfer array to intermediate buffer
        uint8_t write_f(uint8_t* buffer, uint8_t dstIndex, uint8_t len);
        //Transfer string from program memory to intermediate buffer
        void print_f_p(const prog_char str[]);
        //Transfer whole intermediate buffer to display
        void flipFrame();
        void flipFlashState();
	void clearFrame();

        uint8_t getFlashAttr(uint8_t index);
        void setFlashAttr(uint8_t index, uint8_t value);

        uint8_t columnTest();
        uint8_t segmentTest();
        uint8_t dimmingTest();
        uint8_t glowingTest();
        uint8_t charsetTest();
        uint8_t charsetTest2();
        uint8_t rotorTest();
        inline void renderRotor(uint8_t addr, uint8_t idx);

        uint8_t slashEffect();
        uint8_t scrollEffect();

        uint8_t testStep();

protected: 
        uint8_t _cs, _sclk, _data, _rotorState, _flashAttr;
        uint16_t _testBt;
        enum_TestState _tstState;
        uint8_t _vfdFrame[VFD_BYTES_PER_DIGIT * VFD_DIGITS];
};
#endif

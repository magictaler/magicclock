/*
  HT16512.cpp - Driver for HT16512 VFD Controller

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

#include <util/delay.h>

#include "HT16512.h"
#include "WProgram.h"

HT16512::HT16512(uint8_t cs, uint8_t sclk, uint8_t data)
{
    _cs = cs;
    _sclk = sclk;
    _data = data;
}

void HT16512::reset() 
{
    pinMode(_cs, OUTPUT);
    pinMode(_sclk, OUTPUT);
    pinMode(_data, OUTPUT);
    digitalWrite(_cs, HIGH);
    digitalWrite(_sclk, HIGH);

    //Set display mode
    command(0x3, true); //7 digits, 15 segments
    displayOnCmd(VFD_DIMMING_MAX);//maximum brightness

    _testBt = 0;
    _tstState = NOT_STARTED;

    _flashAttr = 0;
}

void HT16512::displayOnCmd(uint8_t dimming)
{
    command(0x88 | (dimming & VFD_DIMMING_MAX), true);
}

void HT16512::displayOffCmd()
{
    command(0x80, true);
}

void HT16512::displayWriteCmd(uint8_t addr_inc, uint8_t nodata)
{
    uint8_t cmd = 0x40;
    if (addr_inc == 0) cmd |= (1 << 2);
    command(cmd, nodata);
}

void HT16512::readKeyDataCmd(uint8_t nodata)
{
    command(0x42, nodata);
}

void HT16512::readSwDataCmd(uint8_t nodata)
{
    command(0x43, nodata);
}

void HT16512::addrSetCmd(uint8_t addr)
{
    command(0xC0 | (addr & 0x1F), true);
}

void HT16512::command(uint8_t value, uint8_t nodata) 
{
    digitalWrite(_sclk, HIGH);
    digitalWrite(_cs, LOW);

    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (( value >> i ) & 0x1)
            digitalWrite(_data, HIGH);
        else 
            digitalWrite(_data, LOW);
        digitalWrite(_sclk, LOW);
        digitalWrite(_sclk, HIGH);
    }
    if (nodata > 0) digitalWrite(_cs, HIGH);
}

const PROGMEM uint8_t SCROL_FRAMES[3][14] = 
    {
        {
        0x01, 0x10, //bits 8, 4
        0x00, 0x00, //bits -
        0x00, 0x00, //bits -
        0x01, 0x10, //bits 8, 4
        0x00, 0x00, //bits -
        0x00, 0x00, //bits -
        0x01, 0x10, //bits 8, 4
        },
        {
        0x00, 0x01, //bits 0
        0x01, 0x00, //bits 8
        0x00, 0x10, //bits 4
        0x00, 0x00, //bits -
        0x01, 0x00, //bits 8
        0x00, 0x10, //bits 4
        0x20, 0x00, //bits 13
        },
        {
        0x10, 0x00, //bits 12
        0x00, 0x10, //bits 4
        0x01, 0x00, //bits 8
        0x00, 0x00, //bits -
        0x00, 0x10, //bits 4
        0x01, 0x00, //bits 8
        0x00, 0x02, //bits 1
        }
    };

const PROGMEM uint8_t ROTOR_PGM[4][2] = {
    {0x02,0x40}, // | (bits 9, 6)
    {0x04,0x48}, // \ (bits 10, 6, 3)
    {0xC0,0x40}, // - (bits 15, 14, 6)
    {0x08,0x44}  // / (bits 11, 6, 2)
};

const PROGMEM uint8_t FONT_PGM[63][2] = {
    {0x00,0x00}, // space 0x20
    {0xC0,0x60}, // ! -: (bits 15, 14, 6, 5)
    {0x00,0x00}, // " N/A
    {0x00,0x00}, // # N/A
    {0xE3,0x51}, // $ (bits 15, 14, 13, 9, 8, 6, 4, 0)
    {0x00,0x00}, // % N/A
    {0x00,0x00}, // & N/A
    {0x00,0x00}, // ' N/A
    {0x04,0x04}, // ( (bits 10, 2)
    {0x08,0x08}, // ) (bits 11, 3)
    {0xCE,0x4C}, // * (bits 15, 14, 11, 10, 9, 6, 3, 2)
    {0xC2,0x40}, // + (bits 15, 14, 9, 6)
    {0x00,0x00}, // , N/A
    {0xC0,0x40}, // - (bits 15, 14, 6)
    {0x00,0x40}, // . (bit 6)
    {0x08,0x44}, // / (bits 11, 6, 2)
    {0x39,0x17}, // 0 (bits 13, 12, 11, 8, 4, 2, 1, 0)
    {0x20,0x6},  // 1 (bits 13, 2, 1)
    {0x89,0x52}, // 2 (bits 15, 11, 8, 6, 4, 1)
    {0x05,0x54}, // 3 (bits 10, 8, 6, 4, 2)
    {0xC2,0x41}, // 4 (bits 15, 14, 9, 6, 0)
    {0xE1,0x51}, // 5 (bits 15, 14, 13, 8, 6, 4, 0)
    {0xF1,0x51}, // 6 (bits bits 15, 14, 13, 12, 8, 6, 4, 0)
    {0x08,0x54}, // 7 (bits 11, 6, 4, 2)
    {0xF1,0x53}, // 8 (bits 15, 14, 13, 12, 8, 6, 4, 1, 0)
    {0xE1,0x53}, // 9 (bits 15, 14, 13, 8, 6, 4, 1, 0)
    {0x00,0x20}, // : (bit 5)
    {0x00,0x00}, // ; N/A
    {0x04,0x04}, // < (bits 10, 2)
    {0x00,0x00}, // = N/A
    {0x08,0x08}, // > (bits 11, 3)
    {0x00,0x00}, // ? N/A
    {0x00,0x00}, // @ N/A
    {0xF0,0x53}, // A (bits 15, 14, 13, 12, 6, 4, 1, 0)
    {0xA3,0x52}, // B (bits 15, 13, 9, 8, 6, 4, 1 )
    {0x11,0x11}, // C (bits 12, 8, 4, 0)
    {0x23,0x52}, // D (bits 13, 9, 8, 6, 4, 1 )
    {0xD1,0x51}, // E (bits 15, 14, 12, 8, 6, 4, 0)
    {0x50,0x51}, // F (bits 14, 12, 6, 4, 0)
    {0xB1,0x11}, // G (bits 15, 13, 12, 8, 4, 0)
    {0xF0,0x43}, // H (bits 15, 14, 13, 12, 6, 1, 0)
    {0x02,0x40}, // I (bits 9, 6)
    {0x31,0x02}, // J (bits 13, 12, 8, 1)
    {0x54,0x45}, // K (bits 14, 12, 10, 6, 2, 0)
    {0x11,0x01}, // L (bits 12, 8, 0)
    {0x30,0x4F}, // M (bits 13, 12, 6, 3, 2, 1, 0)
    {0x34,0x4B}, // N (bits 13, 12, 10, 6, 3, 1, 0)
    {0x31,0x13}, // O (bits 13, 12, 8, 4, 1, 0)
    {0xD0,0x53}, // P (bits 15, 14, 12, 6, 4, 1, 0)
    {0x35,0x13}, // Q (bits 13, 12, 10, 8, 4, 1, 0)
    {0xD4,0x53}, // R (bits 15, 14, 12, 10, 6, 4, 1, 0)
    {0xE1,0x51}, // S (bits 15, 14, 13, 8, 6, 4, 0)
    {0x02,0x50}, // T (bits 9, 6, 4)
    {0x31,0x03}, // U (bits 13, 12, 8, 1, 0)
    {0x18,0x45}, // V (bits 12, 11, 6, 2, 0)
    {0x3C,0x03}, // W (bits 13, 12, 11, 10, 1, 0)
    {0x0C,0x4C}, // X (bits 11, 10, 6, 3, 2)
    {0xE1,0x43}, // Y (bits 15, 14, 13, 8, 6, 1, 0)
    {0x09,0x54}, // Z (bits 11, 8, 6, 4, 2)
    {0x04,0x04}, // [ (bits 10, 2)
    {0x04,0x48}, // \ (bits 10, 6, 3)
    {0x08,0x08}, // ] (bits 11, 3)
    
    //Special non-standard symbols
    {0xC0, 0x53}  // degree (bits 15, 14, 6, 4, 1, 0)
}; // DEL


uint8_t HT16512::readKeyData()
{
    readKeyDataCmd(false);
    _delay_us(1);
    return data_in(false, true);
}

uint8_t HT16512::readSwData()
{
    readSwDataCmd(false);
    _delay_us(1);
    return data_in(false, true);
}

void HT16512::setLEDoutputs(uint8_t value)
{
     command(0x41, false);
     data_out(value , false, true);
}

void HT16512::write(uint8_t value) 
{
    displayWriteCmd(true, false);

    uint8_t chrset = pgm_read_byte(&FONT_PGM[(value - 0x20)][0]);
    data_out(chrset, false, false);
    chrset = pgm_read_byte(&FONT_PGM[(value - 0x20)][1]);
    data_out(chrset, false, true);
}

void HT16512::write_f(uint8_t value, uint8_t dstIndex, uint8_t dispColumn) 
{
    if (dstIndex > (VFD_DIGITS - 1) * VFD_BYTES_PER_DIGIT) return;
    uint8_t chrset = pgm_read_byte(&FONT_PGM[(value - 0x20)][0]);
    _vfdFrame[dstIndex] = chrset;
    chrset = pgm_read_byte(&FONT_PGM[(value - 0x20)][1]);
    if (dispColumn) chrset |= 1 << 5;
    _vfdFrame[dstIndex+1] = chrset;
}

void HT16512::data_out(uint8_t value, uint8_t init_cs, uint8_t finalise_cs) 
{
    digitalWrite(_sclk, HIGH);
    if (init_cs > 0) digitalWrite(_cs, LOW);

    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (( value >> i ) & 0x1)
            digitalWrite(_data, HIGH);
        else 
            digitalWrite(_data, LOW);
        digitalWrite(_sclk, LOW);
        digitalWrite(_sclk, HIGH);
    }
    if (finalise_cs > 0) digitalWrite(_cs, HIGH);
}

uint8_t HT16512::data_in(uint8_t init_cs, uint8_t finalise_cs)
{
    pinModePullup(_data, INPUT, true);

    digitalWrite(_sclk, HIGH);
    if (init_cs > 0) digitalWrite(_cs, LOW);

    uint8_t i;
    uint8_t value = 0xFF;
    for (i = 0; i < 8; i++)
    {
        digitalWrite(_sclk, LOW);

        if (digitalRead(_data) == HIGH)
           value |= (1 < i);
        else
           value &= ~(1 << i);

        digitalWrite(_sclk, HIGH);
    }

    pinMode(_data, OUTPUT);
    if (finalise_cs > 0) digitalWrite(_cs, HIGH);

    return value;
}

uint8_t HT16512::write(uint8_t* buffer, uint8_t dstIndex, uint8_t len)
{
    if (len == 0 || (dstIndex >= VFD_DIGITS * VFD_BYTES_PER_DIGIT)) return 0;
    if (len > VFD_DIGITS * VFD_BYTES_PER_DIGIT - dstIndex) 
        len = VFD_DIGITS * VFD_BYTES_PER_DIGIT - dstIndex;

    addrSetCmd(dstIndex);

    displayWriteCmd(true, false);
    for (uint8_t i = 0; i < len-1; i++)
    {
         data_out(buffer[i], false, false);
    } 
    data_out(buffer[len-1], false, true);

    return len;
}

uint8_t HT16512::write_f(uint8_t* buffer, uint8_t dstIndex, uint8_t len)
{
    if (len == 0 || (dstIndex >= VFD_DIGITS * VFD_BYTES_PER_DIGIT)) return 0;
    if (len > VFD_DIGITS * VFD_BYTES_PER_DIGIT - dstIndex) 
        len = VFD_DIGITS * VFD_BYTES_PER_DIGIT - dstIndex;

    memcpy(&_vfdFrame[dstIndex], buffer, len);

    return len;
}

void HT16512::print_f_p(const prog_char str[])
{
    char c;
    uint8_t idx = 0;
    if(!str) return;

    clearFrame();
   
    while((c = pgm_read_byte(str++)))
    {
        write_f(c, idx, false);
        idx += 2;    
    }
}


uint8_t HT16512::testStep()
{
    switch (_tstState)
    {
        case NOT_STARTED:
            {
	        _tstState = COLUMN_TEST;
                _testBt = 0;
            }
            break;

        case COLUMN_TEST:
            if (!columnTest()) 
            {
                _testBt = 1;
                _tstState = SEGMENT_TEST;
            }
            break;

        case SEGMENT_TEST:
            if (!segmentTest()) 
            {
                _tstState = DIMMING_TEST;
                _testBt = VFD_DIMMING_MAX;
            }
            break;

        case DIMMING_TEST:
            if (!dimmingTest())
            {
                _tstState = GLOWING_TEST;
                _testBt = 0;
            }
            break;

        case GLOWING_TEST:
            if (!glowingTest())
            {
                _tstState = CHARSET_TEST;
                _testBt = (VFD_MAX_TEST_DELAY / 2);
            }
            break;

        case CHARSET_TEST:
            if (!charsetTest())
            {
                //_tstState = CHARSET_TEST2;
                //_testBt = VFD_MAX_TEST_DELAY;
                _tstState = ROTOR_TEST;
                _rotorState = 0;
                _testBt = VFD_ROTOR_FRAMES * VFD_ROTOR_TEST_REVOLUTIONS;
            }
            break;

        case CHARSET_TEST2:
            if (!charsetTest2())
            {
                _tstState = COMPLETED;
            }
            break;

        case ROTOR_TEST:
            if (!rotorTest())
            {
                _tstState = SCROLL_EFFECT;
                _testBt = (VFD_MAX_TEST_DELAY / 2);
                _rotorState = 0;
            }
            break;

        case SCROLL_EFFECT:
            if (!scrollEffect())
            {
                _tstState = COMPLETED;
                //_tstState = SLASH_EFFECT;
                //_testBt = VFD_DIGITS;
            }
            break;

        case SLASH_EFFECT:
            if (!slashEffect())
                _tstState = COMPLETED;
            break;

        case COMPLETED:
            _tstState = NOT_STARTED;
            break;
        default:
            _tstState = NOT_STARTED;
    }
    return _tstState;
}

uint8_t HT16512::columnTest()
{
    if (_testBt < VFD_DIGITS)
    {
        if (_testBt == 0)
        {
            addrSetCmd(0);
            for (uint8_t i = 0; i < VFD_DIGITS; i++)
            {
                displayWriteCmd(true, false);
                if (i == 0) 
                {
                    data_out(0xFF, false, false);
                    data_out(0xFF, false, true);
                }else{
                    data_out(0, false, false);
                    data_out(0, false, true);
                }
            }
        }else{
            addrSetCmd((_testBt - 1) * 2);
            displayWriteCmd(true, false);
            data_out(0, false, false);
            data_out(0, false, false);
            data_out(0xFF, false, false);
            data_out(0xFF, false, true);
        }

        _testBt++;
        return true;
    }else return false;
}

uint8_t HT16512::segmentTest()
{
    if (_testBt < (uint16_t)(1 << VFD_SEGMENTS))
    {
        addrSetCmd(0);
        for (uint8_t i = 0; i < VFD_DIGITS; i++)
        {
            displayWriteCmd(true, false);
            data_out(_testBt & 0xFF, false, false);
            data_out((_testBt >> 8) & 0xFF, false, true);
        }
        _testBt = _testBt << 1;
        return true;
    }else return false;
}

uint8_t HT16512::dimmingTest()
{
    if (_testBt == VFD_DIMMING_MAX)
    {
        addrSetCmd(0);
        for (uint8_t i = 0; i < VFD_DIGITS; i++)
        {
            displayWriteCmd(true, false);
            data_out(0xFF, false, false);
            data_out(0xFF, false, true);
        }

        displayOnCmd(_testBt);
        _testBt--;
        return true;
    }else
    {
        displayOnCmd(_testBt);
        if (_testBt != 0)
        {
            _testBt--;
            return true;
        }
    }
    return false;
}

uint8_t HT16512::glowingTest()
{
    if (_testBt == 0)
    {
        addrSetCmd(0);
        for (uint8_t i = 0; i < VFD_DIGITS; i++)
        {
            displayWriteCmd(true, false);
            data_out(0xFF, false, false);
            data_out(0xFF, false, true);
        }

        displayOnCmd(_testBt);
        _testBt++;
        return true;
    }else
    {
        displayOnCmd(_testBt);
        if (_testBt < VFD_DIMMING_MAX)
        {
            _testBt++;
            return true;
        }
    }
    return false;
}


uint8_t HT16512::charsetTest()
{
    if (_testBt != 0)
    {
        if (_testBt == (VFD_MAX_TEST_DELAY / 2))
        {
            //Sending characters one by one

            //addrSetCmd(0);
            //for (uint8_t testChar = 0x30; testChar < (0x30 + VFD_DIGITS); testChar++)
            //    write(testChar);
    
            //Sending characters to intermediate buffer and
            //then send video data in one go
            for (uint8_t testChar = 0x57; testChar < (0x57 + VFD_DIGITS); testChar++)
                write_f(testChar, ( testChar - 0x57 ) * VFD_BYTES_PER_DIGIT, false);
            
            _flashAttr = 0;
            flipFrame();

            setFlashAttr(VFD_DIGITS + 1, (1 << 6));
        }else
        {
            //Using flashing attributes
            if (_testBt % VFD_FLASHING_DELAY == 0)
            {
                flipFlashState();
                flipFrame();
            }
            if ((_testBt % (VFD_FLASHING_DELAY * 4)) == 0)
            {
                uint8_t flashVal = (getFlashAttr(VFD_DIGITS + 1) >> 1);
                if (flashVal == 0) flashVal = (1 << 6);
                setFlashAttr(VFD_DIGITS + 1, flashVal);
            }
        }
        _testBt--;
        return true;
    }
    return false;
}

uint8_t HT16512::charsetTest2()
{
    if (_testBt != 0)
    {
        if (_testBt == VFD_MAX_TEST_DELAY)
        {
            addrSetCmd(0);
            for (uint8_t testChar = 0x37; testChar < 0x37 + VFD_DIGITS; testChar++)
                write(testChar);
        }
        _testBt--;
        return true;
    }
    return false;
}

inline void HT16512::renderRotor(uint8_t addr, uint8_t idx)
{
    addrSetCmd(addr);
    displayWriteCmd(true, false);
    uint8_t chrset = pgm_read_byte(&ROTOR_PGM[idx][0]);
    data_out(chrset, false, false);
    chrset = pgm_read_byte(&ROTOR_PGM[idx][1]);
    data_out(chrset, false, true);
}

uint8_t HT16512::rotorTest()
{
    if (_testBt!= 0)
    {
        if (_testBt == VFD_ROTOR_FRAMES * VFD_ROTOR_TEST_REVOLUTIONS)
        {
            addrSetCmd(0);
            for (uint8_t i = 0; i < VFD_DIGITS; i++)
            {
                displayWriteCmd(true, false);
                data_out(0, false, false);
                data_out(0, false, true);
            }
        }

        renderRotor(0, (_rotorState & 0xF));

        uint8_t bckRotSt = VFD_ROTOR_FRAMES - ((_rotorState >> 4) & 0xF);
        if (bckRotSt == VFD_ROTOR_FRAMES) bckRotSt = 0;
        renderRotor(2 * VFD_BYTES_PER_DIGIT, bckRotSt);

        renderRotor(4 * VFD_BYTES_PER_DIGIT, ((_rotorState >> 4) & 0xF));

        bckRotSt = VFD_ROTOR_FRAMES - (_rotorState & 0xF);
        if (bckRotSt == VFD_ROTOR_FRAMES) bckRotSt = 0;
        renderRotor((VFD_DIGITS - 1) * VFD_BYTES_PER_DIGIT, bckRotSt);

        _rotorState = (_rotorState & 0xF0) | ((_rotorState & 0xF) + 1);
        if ((_rotorState & 0xF) >= VFD_ROTOR_FRAMES) _rotorState &= 0xF0;

        if (_testBt % 2 == 0)
        {
            _rotorState = ((_rotorState + 0x10) & 0xF0) | (_rotorState & 0xF);
            if ((( _rotorState >> 4 ) & 0xF) >= VFD_ROTOR_FRAMES) _rotorState &= 0xF;
        }

        _testBt--;
        return true;
    }   
    return false;
}

void HT16512::flipFrame()
{
    if (_flashAttr == 0 || ((_flashAttr >> 7 ) & 0x1) == 0)
    {
        write((uint8_t*)&_vfdFrame, 0, VFD_BYTES_PER_DIGIT * VFD_DIGITS);
        return;
    }

    for (uint8_t i = 0; i < VFD_DIGITS; i++)
    {
        if ((( _flashAttr >> (VFD_DIGITS - i - 1) ) & 0x1) == 0)
        {
            //Send bytes representing single digit to VFD
            write(&_vfdFrame[i * VFD_BYTES_PER_DIGIT], i * VFD_BYTES_PER_DIGIT, 
                VFD_BYTES_PER_DIGIT);
        }else{
            //The digit is not visible during this period in time
            addrSetCmd(i * VFD_BYTES_PER_DIGIT);
            //Display space char
            write(0x20);    
        }
    }
}

void HT16512::clearFrame()
{
    memset(&_vfdFrame, 0, sizeof(_vfdFrame));
}

uint8_t HT16512::getFlashAttr(uint8_t index)
{
    if (index >= VFD_DIGITS) return (_flashAttr & ~(1 << 7));
    else return (( _flashAttr >> index ) & 0x1);
}
     
void HT16512::setFlashAttr(uint8_t index, uint8_t value)
{
    if (index >= VFD_DIGITS) 
        _flashAttr = (_flashAttr & (1 << 7)) | (value & ~(1 << 7));
    else _flashAttr |= 1 << index;
}

void HT16512::flipFlashState()
{
    if (((_flashAttr >> 7 ) & 0x1) == 0)
    {
        _flashAttr |= 1 << 7;
    }else{
        _flashAttr &= ~(1 << 7);
    }
}

uint8_t HT16512::slashEffect()
{
    if (_testBt != 0)
    {
        if (_testBt == VFD_DIGITS)
        {
            memset(&_vfdFrame, 0, sizeof(_vfdFrame));
            write_f(0x2F, 0, false);
            write_f(0x5C, (VFD_DIGITS - 1) * VFD_BYTES_PER_DIGIT, false );
        }else{
            //Move everything to the center
            uint16_t* vfdPntr = (uint16_t*)&_vfdFrame;
            uint16_t leftVal, rightVal;
            for (uint8_t i = ((VFD_DIGITS - 1) / 2 + 1); i < VFD_DIGITS; i++)
            {
                leftVal = vfdPntr[VFD_DIGITS - 1 - i];
                rightVal = vfdPntr[i];
                
                vfdPntr[i - 1] |= rightVal;
                vfdPntr[VFD_DIGITS - i] |= leftVal;
            }
            write_f(0x2F, 0, false);
            write_f(0x5C, (VFD_DIGITS - 1) * VFD_BYTES_PER_DIGIT, false);
        }

        flipFrame();

        _testBt--;
        return true;
    }   
    return false;
}

uint8_t HT16512::scrollEffect()
{
    if (_testBt != 0)
    {
        for (uint8_t i = 0; i < sizeof(_vfdFrame); i++)
            _vfdFrame[i] = pgm_read_byte(&SCROL_FRAMES[_rotorState][i]);

        _flashAttr = 0;  
        flipFrame();

        _rotorState++;
        if (_rotorState >= VFD_SCROLL_FRAMES) _rotorState = 0;

        _testBt--;
        return true;    
    }
    return false;
}



/*
  VFDClock.c - multifunctional clock and thermometer based on 
  ATMega32 and 15-segment 7-digit VFD tube from old DVD player
  driven by HT16512 VFD controller.

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

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

#include "Wire.h"
#include "HardwareSerial.h"
#include "Messages.h"
#include "HT16512.h"
#include "IRremote.h"
#include "DS1307.h"
#include "SoundHelper.h"
#include "Board.h"

#include "OneWire.h"

volatile uint16_t sysCntr;      //incremented every timer tick
volatile uint8_t sysState;      //current system state (mode)
volatile uint8_t stateCntr;     //counter for displaying current state (in sec) - when becomes
                                //zero display state changes to the next one
volatile uint8_t soft10Cntr;	//software freq divider by 10
volatile uint8_t dsState;       //temperature sensor state (to avoid explicit usage of delays)  
volatile RtcTimeType rtc;       //current time
volatile float temperature;     //current temperature
volatile uint8_t actModeTimer;  //seconds remaining before going to standby mode

EepromParamsType eepromParams EEMEM;    //RC keys allocations

IRrecv irrecv(IR_RECV_PIN);     //IR receiver
decode_results IR_results;      //Buffer for storing received & decoded IR packets

HT16512 vfd(VFD_CS_PIN, VFD_SCLK_PIN, VFD_DATA_PIN);    //VFD display
OneWire ds(TEMP_SNSR_PIN);      //1Wire temperature sensor from Dallas Semiconductor

void (*boot_loader_start)(void) = (void(*)(void))0x7800; 

inline volatile void onSerialReceive(unsigned char c, ring_buffer* buf)
{
    while (Serial.available() > 0)
    {
        //dump the received byte for now
        Serial.read();
    }
}

void initVFD(uint8_t state)
{
    cli();
    soft10Cntr = 0;
    sysCntr = 0;
    sysState = state;
    stateCntr = STD_DISP_TIME;

    //Enable VFD power supply
    digitalWrite(STANDBY_PIN, HIGH);
    _delay_ms(100);

    //Initialise VFD tube
    vfd.reset();
    vfd.addrSetCmd(0);
    vfd.clearFrame();
    vfd.flipFrame();


//vfd.setLEDoutputs(0xFF);
    
    actModeTimer = SECONDS_IN_ACTIVE_MODE;
    sei();
}

void setup()
{
    //Define powersave configuration
    set_sleep_mode(SLEEP_MODE_IDLE);

    pinMode(STANDBY_PIN, OUTPUT);
    digitalWrite(STANDBY_PIN, HIGH);

    pinModePullup(PIR_SNSR_PIN, INPUT, true);

    //Set up 8 bit Timer0
    cbi(TIMSK0,TOIE0);
    sbi(TIFR0, TOV0);

    TCCR0A = 0;
    outb(TCCR0B, (inb(TCCR0B) & ~TIMER_PRESCALE_MASK) | TIMER_CLK_DIV1024);
    cbi(TCCR0B, WGM12);
    cbi(TCCR0B, WGM13);
    TCNT0 = SYS_TIMER_CNTR;

    sbi(TIMSK0,TOIE0);

//#if defined(__AVR_ATmega328P__)
//    cbi(TIMSK1,TOIE1);
//    sbi(TIFR1, TOV1);
//#else
//    cbi(TIMSK,TOIE1);
//    sbi(TIFR, TOV1);
//#endif

//    TCCR1A = 0;
//    outb(TCCR1B, (inb(TCCR1B) & ~TIMER_PRESCALE_MASK) | TIMER_CLK_DIV64);
//    cbi(TCCR1B, WGM12);
//    cbi(TCCR1B, WGM13);
//    TCNT1 = SYS_TIMER_CNTR;

//#if defined(__AVR_ATmega328P__)
//    sbi(TIMSK1,TOIE1);
//#else
//    sbi(TIMSK, TOIE1);
//#endif
    
    rtc.year = -1;

#if defined(__AVR_ATmega328P__)
//    sbi(EIMSK, INT0);
//    sbi(EIMSK, INT1);
#else
//    sbi(GICR, INT0);
//    sbi(GICR, INT1);
#endif

    initVFD(sysGreetings);
    cli();
              
    //join i2c bus as a master
    Wire.begin();           

    //Initialize temperature sensor variables
    temperature = FAILED_TEMPERATURE;
    dsState = dsIdle;

    //Enable IR receiver
    irrecv.blink13(true);
    irrecv.enableIRIn();

    initSpeaker(SPKR_PIN);

    Serial.begin(57600);
    Serial.attachRxEvent(onSerialReceive);

    Serial.print_p(PRODUCT_NAME);
    Serial.print_p(SPACE_CHAR);
    Serial.print_p(FIRMWARE_REV);
    Serial.print_p(SPACE_CHAR);
    Serial.println_p(FIRMWARE_DATE);

    sei();
}

void toStandBy()
{
    cli();
    sysState = sysStandby;

    //Disable VFD power supply
    digitalWrite(STANDBY_PIN, LOW);
    _delay_ms(100);

    sei();
}

void bcdToVFD(uint8_t value, uint8_t dstIndex, uint8_t dispColumn, uint8_t suppLeadZero)
{
    uint8_t chrset = ((value >> 4) & 0xF) + 0x30;
    if ((chrset == 0x30) && suppLeadZero) chrset = 0x20;
    vfd.write_f(chrset, dstIndex, dispColumn);
    vfd.write_f((value &0xF) + 0x30, dstIndex + 2, dispColumn);
}

uint8_t bcdToBin(uint8_t value)
{
   uint8_t hex = 10;
   hex *= (value & 0xF0) >> 4;
   hex += (value & 0x0F);
   return hex; 
}

uint8_t binToBcd(uint8_t value)
{
   uint8_t MSD = 0;
   while (value >= 10)
   {
      value -= 10;
      MSD += 0x10;
   }
   return MSD + value;
}

uint8_t isCodeEqual(decode_results *results, uint32_t* addr)
{
    eeprom_busy_wait();
    uint16_t rcType = eeprom_read_word(&eepromParams.rcType);
    if (rcType != results->decode_type)
        return false;

    eeprom_busy_wait();
    uint32_t rcCmd = eeprom_read_dword(addr);
    return (rcCmd == results->value);
}

void updateRCCmdInFlash(decode_results *results, uint32_t* addr)
{
    eeprom_busy_wait();
    uint16_t rcType = eeprom_read_word(&eepromParams.rcType);
    if (rcType != results->decode_type)
    {
        eeprom_busy_wait();
        eeprom_write_word(&eepromParams.rcType, results->decode_type);
    }

    eeprom_busy_wait();
    uint32_t rcCmd = eeprom_read_dword(addr);
    if (rcCmd != results->value)
    {
        eeprom_busy_wait();
        eeprom_write_dword(addr, results->value);
    }

    _delay_ms(RCCMD_UPDATE_DELAY);
}

void defaultClockSettings()
{
    if (RTClock.init(DS1307_I2C_ADDR) == 0)
    {
        RTClock.getTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);
        rtc.seconds = MIN_SECONDS;
        rtc.minutes = MIN_MINUTES;
        rtc.hours = MIN_HOURS;
        rtc.day = MIN_DAY;
        rtc.weekday = MIN_WEEKDAY;
        rtc.month = MIN_MONTH;
        rtc.year = DEF_YEAR;
        RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);
    }
}

void dispTime()
{
    vfd.clearFrame();
    bcdToVFD(rtc.hours, 0, false, false);
    bcdToVFD(rtc.minutes, 2 * VFD_BYTES_PER_DIGIT, true, false);
    bcdToVFD(rtc.seconds, 4 * VFD_BYTES_PER_DIGIT, true, false);
    vfd.flipFrame();
}

void dispDate()
{
    vfd.clearFrame();
    bcdToVFD(rtc.day, 0, false, true);

    vfd.write_f(0x20, 4, false);

    uint8_t mon = ((rtc.month >> 4 && 0xF) * 10) + (rtc.month & 0xF);

    vfd.write_f(pgm_read_byte(&MONTHS[mon-1][0]), 6, false);
    vfd.write_f(pgm_read_byte(&MONTHS[mon-1][1]), 8, false);
    vfd.write_f(pgm_read_byte(&MONTHS[mon-1][2]), 10, false);
    vfd.write_f(0x20, 12, false);
    vfd.flipFrame();
}

void dispDayOfWeek()
{
    vfd.clearFrame();
    vfd.print_f_p((PGM_P)pgm_read_word(&DAYSOFWEEK[rtc.weekday - 1]));
    vfd.flipFrame();
}

void IRDecode(decode_results *results) 
{
    if (sysState == sysWaitForSetup)
    {
        sysState = sysRCSetupLeft;
        vfd.print_f_p(RC_LEFT);
        stateCntr = STD_DISP_TIME * 5;
        _delay_ms(RCCMD_UPDATE_DELAY);
    }else if (sysState == sysRCSetupLeft)
    {
        sysState = sysRCSetupRight;
        vfd.print_f_p(RC_RIGHT);
        stateCntr = STD_DISP_TIME * 5;
        updateRCCmdInFlash(results, &eepromParams.rcAction_Left_Code);
    }else if (sysState == sysRCSetupRight)
    {
        sysState = sysRCSetupUp;
        vfd.print_f_p(RC_UP);
        stateCntr = STD_DISP_TIME * 5;
        updateRCCmdInFlash(results, &eepromParams.rcAction_Right_Code);
    }else if (sysState == sysRCSetupUp)
    {
        sysState = sysRCSetupDown;
        vfd.print_f_p(RC_DOWN);
        stateCntr = STD_DISP_TIME * 5;
        updateRCCmdInFlash(results, &eepromParams.rcAction_Up_Code);
    }else if (sysState == sysRCSetupDown)
    {
        sysState = sysRCSetupOnOff;
        vfd.print_f_p(RC_ON_OFF);
        stateCntr = STD_DISP_TIME * 5;
        updateRCCmdInFlash(results, &eepromParams.rcAction_Down_Code);
    }else if (sysState == sysRCSetupOnOff)
    {
        sysState = sysRCSetupMode;
        vfd.print_f_p(RC_MODE);
        stateCntr = STD_DISP_TIME * 5;
        updateRCCmdInFlash(results, &eepromParams.rcAction_OnOff_Code);
    }else if (sysState == sysRCSetupMode)
    {
        sysState = sysRCSetupSettings;
        vfd.print_f_p(RC_SETTINGS);
        stateCntr = STD_DISP_TIME * 5;
        updateRCCmdInFlash(results, &eepromParams.rcAction_Mode_Code);
    }else if (sysState == sysRCSetupSettings)
    {
        sysState = sysTimeDisp;
        stateCntr = STD_DISP_TIME;
        vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);

        updateRCCmdInFlash(results, &eepromParams.rcAction_Settings_Code);
    }else if (sysState == sysTimeDisp || 
              sysState == sysDayOfWeekDisp ||
              sysState == sysExtTempDisp ||
              sysState == sysDateDisp)
    {
        if (isCodeEqual(results, &eepromParams.rcAction_Settings_Code))
        {
            sysState = sysSetHours;
            stateCntr = STD_DISP_TIME * 5;
            if (rtc.year == (uint16_t)-1) defaultClockSettings();
            dispTime();
            vfd.setFlashAttr(6, 1);
            vfd.setFlashAttr(5, 1);
        }else if (isCodeEqual(results, &eepromParams.rcAction_OnOff_Code))
        {
            if (sysState != sysStandby)
            {
                cli();
                actModeTimer = 0;
                sei();
            }else if (sysState != sysStandby)
            {
                cli();
                actModeTimer = SECONDS_IN_ACTIVE_MODE;
                sei();
                initVFD(sysTimeDisp);
            }
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Mode_Code))
        {
            sysState = sysSelfTest;
            _delay_ms(RCCMD_DEBOUNCER);
        }
    }else if (sysState == sysSetHours)
    {
        stateCntr = STD_DISP_TIME * 5;
        if (isCodeEqual(results, &eepromParams.rcAction_Up_Code))
        {
            uint8_t hours = bcdToBin(rtc.hours) + 1;
            if (hours > MAX_HOURS) hours = MIN_HOURS;
            rtc.hours = binToBcd(hours);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispTime();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Down_Code))
        {
            uint8_t hours = bcdToBin(rtc.hours);
            if (hours > MIN_HOURS) hours--; else hours = MAX_HOURS;
            rtc.hours = binToBcd(hours);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispTime();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Right_Code))
        {
            sysState = sysSetMinutes;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(4, 1);
            vfd.setFlashAttr(3, 1);
            _delay_ms(RCCMD_DEBOUNCER);
        }
    }else if (sysState == sysSetMinutes)
    {
        stateCntr = STD_DISP_TIME * 5;
        if (isCodeEqual(results, &eepromParams.rcAction_Up_Code))
        {
            uint8_t minutes = bcdToBin(rtc.minutes) + 1;
            if (minutes > MAX_MINUTES) minutes = MIN_MINUTES;
            rtc.minutes = binToBcd(minutes);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispTime();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Down_Code))
        {
            uint8_t minutes = bcdToBin(rtc.minutes);
            if (minutes > MIN_MINUTES) minutes--; else minutes = MAX_MINUTES;
            rtc.minutes = binToBcd(minutes);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispTime();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Left_Code))
        {
            sysState = sysSetHours;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(6, 1);
            vfd.setFlashAttr(5, 1);
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Right_Code))
        {
            sysState = sysResetSeconds;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(2, 1);
            vfd.setFlashAttr(1, 1);
            _delay_ms(RCCMD_DEBOUNCER);
        }
    }else if (sysState == sysResetSeconds)
    {
        stateCntr = STD_DISP_TIME * 5;
        if (isCodeEqual(results, &eepromParams.rcAction_Up_Code) ||
            isCodeEqual(results, &eepromParams.rcAction_Down_Code))
        {
            rtc.seconds = MIN_SECONDS;
            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispTime();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Left_Code))
        {
            sysState = sysSetMinutes;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(4, 1);
            vfd.setFlashAttr(3, 1);
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Right_Code))
        {
            sysState = sysSetDayOfWeek;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_ALL);
            dispDayOfWeek();
        }
    }else if (sysState == sysSetDayOfWeek)
    {
        stateCntr = STD_DISP_TIME * 5;
        if (isCodeEqual(results, &eepromParams.rcAction_Up_Code))
        {
            if (rtc.weekday == MAX_WEEKDAY) rtc.weekday = MIN_WEEKDAY;
            else rtc.weekday++;

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispDayOfWeek();
            _delay_ms(RCCMD_DEBOUNCER);
        }if (isCodeEqual(results, &eepromParams.rcAction_Down_Code))
        {
            if (rtc.weekday == MIN_WEEKDAY) rtc.weekday = MAX_WEEKDAY;
            else rtc.weekday--;

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispDayOfWeek();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Left_Code))
        {
            sysState = sysResetSeconds;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(2, 1);
            vfd.setFlashAttr(1, 1);
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Right_Code))
        {
            sysState = sysSetDay;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(6, 1);
            vfd.setFlashAttr(5, 1);
            dispDate();
        }
    }else if (sysState == sysSetDay)
    {
        stateCntr = STD_DISP_TIME * 5;
        if (isCodeEqual(results, &eepromParams.rcAction_Up_Code))
        {
            uint8_t day = bcdToBin(rtc.day) + 1;
            if (day > 31) day = MIN_DAY;
            rtc.day = binToBcd(day);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispDate();
            _delay_ms(RCCMD_DEBOUNCER);
        }if (isCodeEqual(results, &eepromParams.rcAction_Down_Code))
        {
            uint8_t day = bcdToBin(rtc.day);
            if (day > MIN_DAY) day--; else day = 31;
            rtc.day = binToBcd(day);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispDate();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Left_Code))
        {
            sysState = sysSetDayOfWeek;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_ALL);
            dispDayOfWeek();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Right_Code))
        {
            sysState = sysSetMonth;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(3, 1);
            vfd.setFlashAttr(2, 1);
            vfd.setFlashAttr(1, 1);
            dispDate();
            _delay_ms(RCCMD_DEBOUNCER);
        }
    }else if (sysState == sysSetMonth)
    {
        stateCntr = STD_DISP_TIME * 5;
        if (isCodeEqual(results, &eepromParams.rcAction_Up_Code))
        {
            uint8_t month = bcdToBin(rtc.month) + 1;
            if (month > MAX_MONTH) month = MIN_MONTH;
            rtc.month = binToBcd(month);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispDate();
            _delay_ms(RCCMD_DEBOUNCER);
        }if (isCodeEqual(results, &eepromParams.rcAction_Down_Code))
        {
            uint8_t month = bcdToBin(rtc.month);
            if (month > MIN_MONTH) month--; else month = MAX_MONTH;
            rtc.month = binToBcd(month);

            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.setTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);

            dispDate();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Left_Code))
        {
            sysState = sysSetDay;
            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
            vfd.setFlashAttr(6, 1);
            vfd.setFlashAttr(5, 1);
            dispDate();
            _delay_ms(RCCMD_DEBOUNCER);
        }else if (isCodeEqual(results, &eepromParams.rcAction_Right_Code))
        {
        }
    }
    irrecv.resume();
}

void NoTempToVFD()
{
    vfd.addrSetCmd(0);
    vfd.write(DASH);
    vfd.write(DASH);
    vfd.write(DEGREE);
    vfd.write('C');
    vfd.write(SPACE_CHR);
    vfd.write(SPACE_CHR);
    vfd.write(SPACE_CHR);
}

float getTemperature(uint8_t* data)
{
    int HighByte, LowByte, TReading, SignBit, Tc_100;
    float res;
    
    LowByte = data[0];
    HighByte = data[1];
    TReading = (HighByte << 8) + LowByte;
    SignBit = TReading & 0x8000;  // test most sig bit
    if (SignBit) // negative
        TReading = (TReading ^ 0xffff) + 1; // 2's comp

    Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

    res = Tc_100 / 100;
    if (SignBit) res *= -1;

    return res;
}

void noDataToVFD()
{
    vfd.addrSetCmd(0);
    for (uint8_t i = 0; i < VFD_DIGITS-1; i++) 
        vfd.write(DASH);
    vfd.write(SPACE_CHR);
}

//===============================================================
//===============================================================
/* main program starts here */
int main(void)
{    
    uint8_t dsAddr[8];  //temperature sensor address
    uint8_t dsData[12]; //temperature sensor data buffer

    setup();
    
    while (1)
    {
        {
            switch (sysState)
            {
                case sysSelfTest:
                    {
                        if (vfd.testStep() == vfd.COMPLETED)
                        {
                            //sysState = sysTimeDisp;
                            //stateCntr = STD_DISP_TIME;
                            boot_loader_start();
                        }
                    }
                    break;
                case sysGreetings:
                        if (stateCntr == 0)
                        {
                            sysState = sysWaitForSetup;
                            stateCntr = STD_DISP_TIME * 2;
                            vfd.print_f_p(RC_SETUP);
                            vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_ALL);
                        }else
                        {
                            vfd.addrSetCmd(0);
                            if (stateCntr > (STD_DISP_TIME / 3 * 2))
                                vfd.print_p(HELLO);
                            else if (stateCntr > STD_DISP_TIME / 3)
                                vfd.print_p(MAGIC);
                            else if  (stateCntr > 0)
                                vfd.print_p(CLOCK);
                        }
                    break;
                case sysWaitForSetup:
                case sysRCSetupLeft:
                case sysRCSetupRight:
                case sysRCSetupUp:
                case sysRCSetupDown:
                case sysRCSetupOnOff:
                case sysRCSetupMode:
                case sysRCSetupSettings:
                case sysSetHours:
                case sysSetMinutes:
                case sysResetSeconds:
                case sysSetYear:
                case sysSetDayOfWeek:
                case sysSetMonth:
                case sysSetDay:

                    if (stateCntr == 0)
                    {
                        sysState = sysTimeDisp;
                        stateCntr = STD_DISP_TIME;
                        vfd.setFlashAttr(VFD_DIGITS, VFD_FLASH_NONE);
                    }else
                    {

                        if (sysState == sysSetHours ||
                            sysState == sysSetMinutes ||
                            sysState == sysResetSeconds)
                        {
                            dispTime();
                        }
                        vfd.flipFlashState();
                        vfd.flipFrame();
                    }
                    break;

                case sysTimeDisp:
                    {
                        if (rtc.year == (uint16_t)-1) noDataToVFD();
                        else dispTime();

                        if (stateCntr == 0)
                        {
                            sysState = sysDateDisp;
                            stateCntr = STD_DISP_TIME;
                        }
                    }
                    break;

                case sysDayOfWeekDisp:
                    if (rtc.year == (uint16_t)-1) noDataToVFD();
                    else 
                    {
                        vfd.addrSetCmd(0);
                        vfd.print_p((PGM_P)pgm_read_word(&DAYSOFWEEK[rtc.weekday - 1]));
                        if (stateCntr == 0)
                        {
                            sysState = sysExtTempDisp;
                            stateCntr = STD_DISP_TIME;
                        }
                    }
                    break;

                case sysExtTempDisp:
                        if (temperature == FAILED_TEMPERATURE)NoTempToVFD();
                        else 
                        {
                            vfd.addrSetCmd(0);
                            if (temperature > 0) vfd.write('+');
                            vfd.print((long)temperature, DEC);
                            vfd.write(DEGREE);
                            vfd.write('C');

                            for (uint8_t i = 0; i < VFD_DIGITS-4; i++) 
                                vfd.write(SPACE_CHR);
                        }
                        if (stateCntr == 0)
                        {
                            sysState = sysTimeDisp;
                            stateCntr = STD_DISP_TIME;
                        }
                    break;
    
                case sysIntTempDisp:

                    break;

                case sysDateDisp:
                        if (rtc.year == (uint16_t)-1) noDataToVFD();
                        else 
                        {
                            dispDate();
                        }
                        if (stateCntr == 0)
                        {
                            sysState = sysDayOfWeekDisp;
                            stateCntr = STD_DISP_TIME;
                        }
                    break;

                case sysIntroEffect:

                    break;
    
                case sysStandby:

                    break;

                case sysFailure:
                    stateCntr = STD_DISP_TIME * 2;
                    vfd.print_f_p(FAILURE);
                    break;
            
                default:
                    ;       
            }
        }

        //Getting current time
        if ((sysCntr % ((SYS_TIMER_FREQ / 10) / 4)) == 0)
        {
            if (RTClock.init(DS1307_I2C_ADDR) == 0)
                RTClock.getTime(DS1307_I2C_ADDR, (RtcTimeType*)&rtc);
        }

        //Getting current temperature
        if ((sysCntr % ((SYS_TIMER_FREQ / 10) * 5)) == 0)
        {
            if (dsState == dsIdle)
            {
                if ( ds.search(dsAddr)) 
                {
                    if (ds.crc8(dsAddr, 7) == dsAddr[7])
                    {
                        if (dsAddr[0] == 0x28) 
                        {
                            ds.reset();
                            ds.select(dsAddr);
                            ds.write(0x44);
                            dsState = dsTempConversion;
                        }
                    }
                }else 
                {
                    ds.reset_search();
                    dsState = dsInitDelay;
                }
            }else if (dsState == dsTempConversion)
            {
                ds.reset();
                ds.select(dsAddr);
                ds.write(0xBE);
                for (uint8_t i = 0; i < 9; i++) 
                    dsData[i] = ds.read();
  
                if (ds.crc8(dsData, 8) == dsData[8])
                {
                    temperature = getTemperature((uint8_t*)&dsData);
                }
                dsState = dsIdle;
            }else if (dsState == dsInitDelay)
            {
                dsState = dsIdle;
            }

            
            if (temperature != FAILED_TEMPERATURE)
            {
                if (temperature > 0) Serial.write('+');
                else if (temperature < 0) Serial.write('-');
                  Serial.print((long)temperature, DEC);
                Serial.write('C');
            }
            Serial.println();
        }

        //Handling RC commands
        if (irrecv.decode(&IR_results)) 
        {
            cli();
            actModeTimer = SECONDS_IN_ACTIVE_MODE;
            sei();

            if (sysState == sysStandby)
            {
                initVFD(sysTimeDisp);
            }

            shortBeep();
            IRDecode(&IR_results);
            irrecv.resume();
        }
        
        if (actModeTimer == 0 &&  sysState != sysStandby)
        {
            uint8_t brightness = VFD_DIMMING_MAX;
            while (brightness != 0)
	    {            
                vfd.displayOnCmd(brightness--);
                _delay_ms(500);
            }
            toStandBy();
        }

        if (digitalRead(PIR_SNSR_PIN) == LOW)
        {
            cli();
            actModeTimer = SECONDS_IN_ACTIVE_MODE;
            sei();

            if (sysState == sysStandby)
            {
                initVFD(sysTimeDisp);
            }
        }

        if (sysState == sysGreetings)
            longBeepAsync();
 
//        sleep_mode();
        _delay_ms(MAIN_LOOP_DELAY);
    }
}

/* IR receiver */
ISR(INT0_vect)
{
    //just for waking up
}

/* PIR sensor */
ISR(INT1_vect)
{
    //just for waking up
}

/* 100Hz system timer */
ISR(TIMER0_OVF_vect)
//ISR(TIMER1_OVF_vect)
{
    TCNT0 = SYS_TIMER_CNTR;
    //TCNT1 = SYS_TIMER_CNTR;
    
    if (soft10Cntr >= 10)
    {
        sysCntr++;

        if (sysCntr % (SYS_TIMER_FREQ / 10) == 0)
        {
	    if (stateCntr != 0) stateCntr--; 
            if (actModeTimer != 0) actModeTimer--;
        }
        soft10Cntr = 0;
    }else soft10Cntr++;
}

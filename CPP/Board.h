#ifndef Board_h
#define Board_h

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef outb
	#define	outb(addr, data)	addr = (data)
#endif
#ifndef inb
	#define	inb(addr)			(addr)
#endif

#define TIMER_CLK_STOP			0x00	///< Timer Stopped
#define TIMER_CLK_DIV1			0x01	///< Timer clocked at F_CPU
#define TIMER_CLK_DIV8			0x02	///< Timer clocked at F_CPU/8
#define TIMER_CLK_DIV64			0x03	///< Timer clocked at F_CPU/64
#define TIMER_CLK_DIV256		0x04	///< Timer clocked at F_CPU/256
#define TIMER_CLK_DIV1024		0x05	///< Timer clocked at F_CPU/1024
#define TIMER_CLK_T_FALL		0x06	///< Timer clocked at T falling edge
#define TIMER_CLK_T_RISE		0x07	///< Timer clocked at T rising edge
#define TIMER_PRESCALE_MASK		0x07	///< Timer Prescaler Bit-Mask

#define LED_PORT	PORTB
#define LED_DDR		DDRB
#define LED_YELLOW	2

#define LED_YELLOW_0 cbi(LED_PORT,LED_YELLOW)
#define LED_YELLOW_1 sbi(LED_PORT,LED_YELLOW)

#define SYS_TIMER_FREQ  100
#define SYS_TIMER_CNTR 255 - (F_CPU/1024/SYS_TIMER_FREQ)

//#define SYS_TIMER_FREQ  10
//#define SYS_TIMER_CNTR  65535 - (F_CPU/64/SYS_TIMER_FREQ)

#define SPKR_PIN 17     //PB1

#define VFD_CS_PIN   15 //PD7
#define VFD_SCLK_PIN 14 //PD6
#define VFD_DATA_PIN 13 //PD5

#define STANDBY_PIN  12 //PD4
#define PIR_SNSR_PIN 11 //PD3
#define IR_RECV_PIN  10 //PD2
#define TEMP_SNSR_PIN 24//PC2

#define STD_DISP_TIME 3

#define FAILED_TEMPERATURE -999

#define SPACE_CHR 0x20
#define DASH 0x21
#define DEGREE 0x5E

#define MAX_HOURS 23
#define MAX_MINUTES 59
#define MAX_MONTH 12
#define MIN_HOURS 0
#define MIN_MINUTES 0
#define MIN_MONTH 1
#define MAX_WEEKDAY 7
#define MIN_WEEKDAY 1
#define MIN_DAY 1
#define DEF_YEAR 0x2012
#define MIN_SECONDS 0

#define RCCMD_UPDATE_DELAY 1000
#define RCCMD_DEBOUNCER 300
#define MAIN_LOOP_DELAY 200

#define SECONDS_IN_ACTIVE_MODE 180


const PROGMEM char TWO_POS_HEX_FORMAT[] = "%02X";
const PROGMEM char TWO_POS_INT_FORMAT[] = "%02i";

enum enum_SysState
{
    sysSelfTest,
    sysGreetings,
    sysTimeDisp,
    sysDayOfWeekDisp,
    sysExtTempDisp,
    sysIntTempDisp,
    sysDateDisp,
    sysSetHours,
    sysSetMinutes,
    sysResetSeconds,
    sysSetYear,
    sysSetMonth,
    sysSetDay,
    sysSetDayOfWeek,
    sysIntroEffect,
    sysStandby,
    sysWaitForSetup,
    sysRCSetupLeft,
    sysRCSetupRight,
    sysRCSetupUp,
    sysRCSetupDown,
    sysRCSetupOk,
    sysRCSetupOnOff,
    sysRCSetupSettings,
    sysRCSetupMode,
    sysFailure
};

enum enum_DsState
{
    dsIdle,
    dsTempConversion,
    dsInitDelay
};

enum enum_RCAction
{
    rcAction_OK,
    rcAction_Left,
    rcAction_Right,
    rcAction_Up,
    rcAction_Down,
    rcAction_OnOff,
    rcAction_Mode,
    rcAction_Settings
};

const PROGMEM uint8_t MONTHS[12][3] = {
    {'J','A','N'},
    {'F','E','B'},
    {'M','A','R'},
    {'A','P','R'},
    {'M','A','Y'},
    {'J','U','N'},
    {'J','U','L'},
    {'A','U','G'},
    {'S','E','P'},
    {'O','C','T'},
    {'N','O','V'},
    {'D','E','C'}
};

const PROGMEM char MON[] = "MONDAY ";
const PROGMEM char TUE[] = "TUESDAY";
const PROGMEM char WED[] = "WED-DAY";
const PROGMEM char THU[] = "THU-DAY";
const PROGMEM char FRI[] = "FRIDAY ";
const PROGMEM char SAT[] = "SAT-DAY";
const PROGMEM char SUN[] = "SUNDAY ";

static PGM_P DAYSOFWEEK[] PROGMEM = {
    MON,
    TUE,
    WED,
    THU,
    FRI,
    SAT,
    SUN
};

typedef struct struct_EepromParams
{       
    uint16_t rcType;
    uint32_t rcAction_OK_Code;
    uint32_t rcAction_Left_Code;
    uint32_t rcAction_Right_Code;
    uint32_t rcAction_Up_Code;
    uint32_t rcAction_Down_Code;
    uint32_t rcAction_OnOff_Code;
    uint32_t rcAction_Mode_Code;
    uint32_t rcAction_Settings_Code;
} EepromParamsType;

extern EepromParamsType eepromParams;

#endif

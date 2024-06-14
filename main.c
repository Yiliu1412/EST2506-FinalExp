#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"
#include "ctype.h"
#include "string.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/eeprom.h"

////////////////////////////////////////// DEFINES //////////////////////////////////////////

//I2C GPIO chip address and resigster define
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT						0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06

#define SYSTICK_FREQUENCY		20000000 // 时钟频率 20Mhz

////////////////////////////////////////// GLOBAL VARIABLES //////////////////////////////////////////

volatile uint8_t result;

int data[8];
uint16_t t = 10000;

char numbers[10] = {'0','1','2','3','4','5','6','7','8','9'};
uint8_t StudentID[9] = {3,1,9,1,0,6,0,3};

uint8_t seg7[] = {
    0x3f,
    0x06,
    0x5b,
    0x4f,
    0x66,
    0x6d,
    0x7d,
    0x07,
    0x7f,
    0x6f,
    0x77,
    0x7c,
    0x58,
    0x5e,
    0x079,
    0x71,
    0x5c
};

uint8_t seg71[] = {
	0x3f,
	0x30,
	0x5b,
	0x79,
	0x74,
	0x69,
	0x6f,
	0x38,
	0x7f,
	0x7d
};

uint8_t Serial[8];
char State_Get[8];
char RxBuf[256]; 
uint8_t RxEndFlag = 0;

uint8_t isReverse = 0;
uint8_t seg_pos = 0;
uint8_t seg_blink = 1;
uint8_t seg_bit = 8;
uint8_t cnt = 0;
uint8_t isboot = 1;
uint8_t isSoftReset = 0;
uint8_t isSet = 0;
uint8_t set_flag;
uint8_t yes,cancel;
uint8_t isStartTiming = 0;
uint8_t isRing = 0;
uint32_t BeepPeriod = 5000;
uint8_t Alarm_isEnable = 0;
uint8_t ringtone = 1;
uint8_t SetStopwatch = 0;
uint32_t ui32SysClock;
uint32_t ui32EEPROMInit;

uint16_t ms;

uint8_t clock10ms = 0;
uint8_t clock10ms_flag = 0;

uint8_t clock20ms = 0;
uint8_t clock20ms_flag = 0;

uint16_t clock100ms = 0;
uint8_t clock100ms_flag = 0;

uint16_t clock800ms = 0;

uint16_t clock1s = 0;
uint8_t clock1s_flag = 0;

uint16_t year = 24;
uint8_t year_high = 2;
uint8_t year_low = 4;

uint8_t month = 6;
uint8_t month_high = 0;
uint8_t month_low = 6;

uint8_t day = 16;
uint8_t day_high = 1;
uint8_t day_low = 6;

uint8_t hour = 10;
uint8_t hour_high = 1;
uint8_t hour_low = 0;

uint8_t minute = 0;
uint8_t minute_high = 0;
uint8_t minute_low = 0;

uint8_t second = 0;
uint8_t second_high = 0;
uint8_t second_low = 0;

uint8_t Alarm_hour = 0;
uint8_t Alarm_minute = 0;
uint8_t Alarm_second = 0;
uint8_t Alarm_hour_H = 0;
uint8_t Alarm_hour_L = 0;
uint8_t Alarm_min_H = 0;
uint8_t Alarm_min_L = 0;
uint8_t Alarm_sec_H = 0;
uint8_t Alarm_sec_L = 0;


int Record_min_high = 0;
int Record_min_low = 0;
int Record_second_high = 0;
int Record_second_low = 0;
int Record_milisecond_high = 0;
int Record_milisecond_low = 0;

uint32_t Dispalydata[16];
uint32_t Dispalydata_read[16];


uint8_t change_Key = 0; // 检测按键状态的变化
uint8_t Key, usr_Key1, usr_Key2, lastKey, lastusr_Key1, lastusr_Key2; 
uint8_t isPressUsrkey = 0; 
uint8_t isReleaseUsrkey = 0;
int press, release, duration; 
uint8_t Key_flag[8], Key_count[8];
uint8_t isShortPress[8], isLongPress[8], isRelease[8]; 
uint8_t mode_key, set_key, key_number[8];



////////////////////////////////////////// FUNCTION DECLARATIONS //////////////////////////////////////////

void SysTickInit(void);
void Reset(void);
void Delay(uint32_t value);
void Key_detect(void);
void Boot(void);
void Trim(char *str);

void 		S800_GPIO_Init(void);
void    	S800_PWM_Init(void);
void		S800_Timer0_Init(void);

void 		S800_UART_Init(void);
void 		UART_INIT(void);
void 		UARTStringPut(const char *cMessage);

void		S800_I2C0_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);

void Press_to_set(void);
void Time_display(void);
void Time_set(void);
void Date_display(void);
void Date_set(void);
void Alarm_display(void);
void Alarm_set(void);
void Record_display(void);
void Record_set(void);

void Ringtone(void);

//////////////////////////////////////////// MAIN PROGRAM ////////////////////////////////////////////////////


int main(void){
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000); 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)){
	}

	ui32EEPROMInit = EEPROMInit();

	if(ui32EEPROMInit != EEPROM_INIT_OK){
		while(1){}
	}

	EEPROMRead(Dispalydata_read, 0x400, sizeof(Dispalydata_read));

	if(Dispalydata_read[12] == 1){
		int i;
		year_high = Dispalydata_read[0];
		year_low = Dispalydata_read[1];
		month_high = Dispalydata_read[2];
		month_low = Dispalydata_read[3];
		day_high = Dispalydata_read[4];
		day_low = Dispalydata_read[5];
		hour_high = Dispalydata_read[6];
		hour_low = Dispalydata_read[7];
		minute_high = Dispalydata_read[8];
		minute_low = Dispalydata_read[9];
		second_high = Dispalydata_read[10];
		second_low = Dispalydata_read[11];
		for(i = 0; i < 13; i++){
			Dispalydata[i] = 0;
		}
		EEPROMProgram(Dispalydata, 0x400, sizeof(Dispalydata));
	}
	else{
		year_high = 2;
		year_low = 4;
		month_high = 0;
		month_low = 6;
		day_high = 1;
		day_low = 6;
		hour_high = 1;
		hour_low = 2;
		minute_high = 0;
		minute_low = 0;
		second_high = 0;
		second_low = 70;
	}

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	S800_PWM_Init();
	S800_Timer0_Init();
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT );
	IntEnable(INT_TIMER0A);
	SysTickInit();   
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	IntEnable(INT_UART0);
  	IntMasterEnable();

	UARTStringPut((char*)"\n启动成功，输入“?”查看所有命令。\n");
	
	while (1){
		if(!isboot) Key_detect();
		if(Alarm_isEnable) Ringtone();
		UART_INIT();
		if(isboot) Boot();
		else{		
			switch(mode_key){
				case 1:					
					if(isSet) Time_set();
					else Time_display();
					break;
				case 2:					
					if(isSet) Date_set();
					else Date_display();
					break;
				case 3:
					if(isSet) Alarm_set();
					else Alarm_display();
					break;
				case 4:
					if(isSet) Record_set();
					else{
						Record_display();
						if(yes) isStartTiming = 1;
						if(cancel) isStartTiming = 0;
					}
					break;
				default:
					Time_display();
			}
		}
	}
}

void Boot(void){ 
	int i;
	for(i = 0; i < clock100ms_flag%8+1; i++)
	{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,256-pow(2,clock100ms_flag%8+1));
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[StudentID[i]]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<i);
		Delay(t);
	}
}//从左到右显示学号


void Trim(char *str) {
  int i, j;
	  for (i = 0, j = 0; str[i]; i++) {
			  	if (str[i] != ' ') 
					str[j++] = str[i];
	}
	str[j] = '\0'; 
}//去除字符串中的空格


void SysTick_Handler(void){
	ms++;
	if(ms >= 1000) ms = 0;
	if(isboot){
		if(++clock100ms >= 100){
			clock100ms_flag++;
			clock100ms = 0;
		}
		if(clock100ms_flag == 8) {
			isboot = 0;
		}
	} // clock100ms_flag除8为显示学号次数

	if(++clock800ms >= 800){
		seg_blink = 1 - seg_blink;
	}

	if(++clock1s >= 1000){
		second_low++;
		clock1s = 0;
	}

	if(second_low >= 10){
			second_high++;
			second_low = 0;
	}

	if(second_high >= 6){
		minute_low++;
		second_high = 0;
	}

	if(minute_low >= 10){
		minute_high++;
		minute_low = 0;
	}

	if(minute_high >= 6){
		hour_low++;
		minute_high = 0;
	}

	if(hour_low >= 10){
		hour_high++;
		hour_low = 0;
	}

	hour = hour_high*10 + hour_low;
	minute = minute_high*10 + minute_low;
	second = second_high*10 + second_low;

	usr_Key1 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0);
	usr_Key2 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1);
}

void SysTickInit(void){
	SysTickPeriodSet(ui32SysClock/1000);
	SysTickEnable();  			
	SysTickIntEnable();		
}


/////////////////////////////////////////  FUNCTION ABOUT CLOCK //////////////////////////////////////////

void Reset(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)){
	}

	ui32EEPROMInit = EEPROMInit();
	
	if(ui32EEPROMInit != EEPROM_INIT_OK){
		while(1){}
	}

	Dispalydata[0] = year_high;
	Dispalydata[1] = year_low;
	Dispalydata[2] = month_high;
	Dispalydata[3] = month_low;
	Dispalydata[4] = day_high;
	Dispalydata[5] = day_low;
	Dispalydata[6] = hour_high;
	Dispalydata[7] = hour_low;
	Dispalydata[8] = minute_high;
	Dispalydata[9] = minute_low;
	Dispalydata[10] = second_high;
	Dispalydata[11] = second_low;
	Dispalydata[12] = 1;

	EEPROMProgram(Dispalydata, 0x400, sizeof(Dispalydata));
	SysCtlReset();
}


void Key_detect(void){
	int i,j;
	Key = ~I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
	change_Key = Key ^ lastKey; 
	
	if(usr_Key1 == lastusr_Key1 && usr_Key1 == 0){
		yes = 1;
		cancel = 0;
	}
	else if(usr_Key2 == lastusr_Key2 && usr_Key2 == 0){
		yes = 0;
		cancel = 1;
	}
	else{
		yes = 0;
		cancel = 0;
	}

	if(change_Key != 0 && Key != 0) {
		for(i = 3; i >= 0; i--) {
			if(change_Key & (1 << i)) {
				if(Key & (1 << i)){
					mode_key = i+1;
				}
				else {
					mode_key = 1;
				}
			}
		}

		for(j = 4; j < 8; j++) {
			if(change_Key & (1 << j)) {
				if(Key & (1 << j)){
					set_key = j+1;
				}
				else {
					set_key = 0;
				}
			}
		}
	}
	lastKey = Key; 
	lastusr_Key1 = usr_Key1;
	lastusr_Key2 = usr_Key2;
}


void Time_display(void){
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~1);

	if(isReverse){
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,127);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[hour_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[hour_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[minute_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[minute_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[second_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[second_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
		Delay(t);
		}
	else{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~1);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[minute_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[minute_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[second_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
		Delay(t);		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[second_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
		Delay(t);
	}
}


void Date_display(void){
	if(isReverse){
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~64);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[year_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[year_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[month_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[month_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[day_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[day_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
		Delay(t);	
	}
	else{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~2);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[year_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[year_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[month_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
		Delay(t);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[day_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
		Delay(t);	
	}
}


void Alarm_display(void){
	if(isReverse){
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~32);

		if(Alarm_isEnable){
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_hour_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_hour_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_min_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_min_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_sec_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_sec_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
			Delay(t);
		}
		else{
			if(seg_blink){
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);
				Delay(8*t);
			}
			else{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_hour_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_hour_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_min_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_min_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_sec_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Alarm_sec_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));		
				Delay(t);
			}
		}
	}
	else{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~4);
		
		if(Alarm_isEnable){
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_hour_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_hour_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
			Delay(t);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_min_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_min_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_sec_H]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
			Delay(t);		
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_sec_L]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
			Delay(t);
		}
		else{
			if(seg_blink){
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);
				Delay(8*t);
			}
			else{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_hour_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(1));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_hour_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(2));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(4));
				Delay(t);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_min_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(8));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_min_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(16));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);	
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(32));
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_sec_H]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(64));		
				Delay(t);		
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Alarm_sec_L]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(128));		
				Delay(t);
			}
		}
	}
}


void Record_display(void){
	if(isReverse){
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~16);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Record_min_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(64));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Record_min_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0X40);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(16));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Record_second_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(8));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Record_second_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Record_milisecond_high]|0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(2));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[Record_milisecond_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(1));
		Delay(t);		
	}
	else{
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~8);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Record_min_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(1));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Record_min_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(2));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0X40);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(4));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Record_second_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(8));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Record_second_low]|0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(16));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Record_milisecond_high]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(32));
		Delay(t);	
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[Record_milisecond_low]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, (uint8_t)(64));
		Delay(t);	
	}
}
	
void Press_to_set(void){
	int i;

	if(set_key == 8 && isShortPress[7] && isRelease[7]){
		if(seg_pos > 0){
			if(seg_pos == 3 || seg_pos == 6){
				seg_pos -= 2;
			}
			else{
				seg_pos--;
			}
		}
		else{
			seg_pos = 7;
		}
		isShortPress[7] = 0;
	}
	
	if(set_key == 7 && isShortPress[6] && isRelease[6]){
		if(seg_pos < 7){
			if(seg_pos == 1 || seg_pos == 4){
				seg_pos += 2;
			}
			else{
				seg_pos++;
			}
		}
		else{
			seg_pos = 0;
		}
		isShortPress[6] = 0;
	}

	if(set_key == 6 && isShortPress[5] && isRelease[5]){
		data[seg_pos]--; 
		isShortPress[5] = 0;
	}

	if(set_key == 5 && isShortPress[4] && isRelease[4]){
		data[seg_pos]++; 
		isShortPress[4] = 0;
	}
	
	for(i = 0; i < seg_bit; i++){
		if(i == seg_pos && seg_blink){
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);	
			Delay(t);
		}
		else if(i == 2 || i == 5){
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);
			Delay(t);
		}
		else{
			if(isReverse){
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg71[data[i]]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);	
				Delay(t);
			}
			else{
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[data[i]]);
				result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1U<<i);	
				Delay(t);
			}
		}
	}
}

void Time_set(void){
	seg_bit = 8;

	if(isReverse){
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,127);
	}
	else{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~1);
	}

	if(set_flag){
		data[0] = hour_high;
		data[1] = hour_low;
		data[3] = minute_high;
		data[4] = minute_low;
		data[6] = second_high;
		data[7] = second_low;
		set_flag = 0;
	}

	Press_to_set();
	if(data[0] > 2 && data[1] != 3) data[0] = 2;
	
	if(data[1] > 9){
		data[1] = 0;
		data[0]++;
	}

	if(data[0] == 2 && data[1] > 3){
		data[0] = 0;
		data[1] = 0;
	}

	if(data[3] > 5){
		data[3] = 0;
	}

	if(data[4] > 9){
		data[3]++;
		data[4] = 0;
	}

	if(data[6] > 5){
		data[6] = 0;
	}

	if(data[7] > 9){
		data[6]++;
		data[7] = 0;
	}
	
	if(data[0] < 0 && data[1] != 0){
		data[0] = 0;
	}

	if(data[1] < 0){
		data[1] = 9;
		data[0]--;
	}

	if(data[0] == 0 && data[1] < 0){
		data[0] = 2;
		data[1] = 3;
	}

	if(data[3] < 0){
		data[3] = 5;
	}

	if(data[4] < 0){
		data[3]--;
		data[4] = 9;
	}

	if(data[6] < 0){
		data[6] = 5;
	}

	if(data[7] < 0){
		data[6]--;
		data[7] = 9;
	}

	if(yes){
		hour_high = data[0];
		hour_low = data[1];
		minute_high = data[3];
		minute_low = data[4];
		second_high = data[6];
		second_low = data[7];
		isSet = 0;
	}

	if(cancel){
		isSet = 0;
	}
}

void Date_set(void){
	seg_bit = 8;

	if(isReverse){
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~64);
	}
	else{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~2);
	}

	if(set_flag){
		data[0] = year_high;
		data[1] = year_low;
		data[3] = month_high;
		data[4] = month_low;
		data[6] = day_high;
		data[7] = day_low;
		set_flag = 0;
	}

	Press_to_set();

	if(data[0] > 9) data[0] = 0;
	if(data[1] > 9) data[1] = 0;
	if(data[0] < 0) data[0] = 9;
	if(data[1] < 0) data[1] = 9;

	year = 2000 + data[0]*10 + data[1];

	if(data[3] > 1){
		data[3] = 0;
	}

	if(data[4] > 9){
		data[3]++;
		data[4] = 0;
	}

	if(data[3] == 1 && data[4] > 2){
		data[3] = 0;
		data[4] = 0;
	}

	if(data[3] < 0){
		data[3] = 1;
	}

	if(data[4] < 0){
		data[3]--;
		data[4] = 9;
	}

	if(data[3] == 0 && data[4] < 1){
		data[3] = 1;
		data[4] = 2;
	}

	month = data[3]*10 + data[4];
	
	if(month != 2){
		if(data[6] > 3){
			data[6] = 0;
		}
		if(data[7] > 9 && data[6] != 3){
			data[6]++;
			data[7] = 0;
		}
		if(data[6] == 3){
			if(month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12){
				if(data[7] > 1){
					data[6] = 0;
					data[7] = 1;
				}
			}
			else{
				if(data[7] > 0){
					data[6] = 0;
					data[7] = 1;
				}
			}
		}
	}
	else if(month == 2){
		if(data[6] > 2) data[6] = 0;

		if(data[7] > 9 && data[6] != 2){
			data[6]++;
			data[7] = 0;
		}
		
		if(data[6] == 2){
			if(year%400 == 0 ||(year%100 != 0 && year%4 == 0)){
				if(data[7] > 9){
					data[6] = 0;
					data[7] = 1;
				}
			}
			else{
				if(data[7] > 8){
					data[6] = 0;
					data[7] = 0;
				}
			}
		}
	}

	if(yes){
		year_high = data[0];
		year_low = data[1];
		month_high = data[3];
		month_low = data[4];
		day_high = data[6];
		day_low = data[7];
		isSet = 0;
	}

	if(cancel){
		isSet = 0;
	}
}

	
void Alarm_set(void){
	seg_bit = 8;

	if(isReverse){
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~32);
	}
	else{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~4);
	}
	
	if(set_flag){
		data[0] = hour_high;
		data[1] = hour_low;
		data[3] = minute_high;
		data[4] = minute_low;
		data[6] = second_high;
		data[7] = second_low;
		set_flag = 0;
	}

	Press_to_set();
	if(data[0] > 2 && data[1] != 3) data[0] = 2;

	if(data[1] > 9) {
		data[1] = 0;
		data[0]++;
	}

	if(data[0] == 2 && data[1] > 3){
		data[0] = 0;
		data[1] = 0;
	}

	if(data[3] > 5){
		data[3] = 0;
	}

	if(data[4] > 9){
		data[3]++;
		data[4] = 0;
	}

	if(data[6] > 5){
		data[6] = 0;
	}

	if(data[7] > 9){
		data[6]++;
		data[7] = 0;
	}
	
	if(data[0] < 0 && data[1] != 0) data[0] = 0;
	
	if(data[1] < 0) {
		data[1] = 9;
		data[0]--;
	}

	if(data[0] == 0 && data[1] < 0){
		data[0] = 2;
		data[1] = 3;
	}

	if(data[3] < 0){
		data[3] = 5;
	}

	if(data[4] < 0){
		data[3]--;
		data[4] = 9;
	}

	if(data[6] < 0){
		data[6] = 5;
	}

	if(data[7] < 0){
		data[6]--;
		data[7] = 9;
	}

	if(yes){
		Alarm_hour_H = data[0];
		Alarm_hour_L = data[1];
		Alarm_min_H = data[3];
		Alarm_min_L = data[4];
		Alarm_sec_H = data[6];
		Alarm_sec_L = data[7];
		isSet = 0;
		Alarm_hour = Alarm_hour_H*10 + Alarm_hour_L;
		Alarm_minute = Alarm_min_H*10 + Alarm_min_L;
		Alarm_second = Alarm_sec_H*10 + Alarm_sec_L;
		Alarm_isEnable = 1;
	}

	if(cancel){
		Alarm_hour_H = data[0];
		Alarm_hour_L = data[1];
		Alarm_min_H = data[3];
		Alarm_min_L = data[4];
		Alarm_sec_H = data[6];
		Alarm_sec_L = data[7];
		isSet = 0;
		Alarm_isEnable = 0;
	}
}


void Record_set(void){
	int i;		
	seg_bit = 5;

	if(set_flag){
		data[0] = 0;
		data[1] = 0;
		data[3] = 0;
		data[4] = 0;
		set_flag = 0;
	}

	if(isReverse){
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~16);
	}

	else{
		if(seg_blink) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,255);
		else result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~8);
	}

	Press_to_set();

	if(data[0] > 9) data[i] = 0;
	if(data[0] < 0) data[i] = 9;
	if(data[1] > 9) data[i] = 0;
	if(data[1] < 0) data[i] = 9;
	if(data[3] > 6) data[i] = 0;
	if(data[3] < 0) data[i] = 5;

	if(data[4] > 9){
		data[3]++;
		data[i] = 0;
	}

	if(data[4] < 0){
		data[3]--;
		data[i] = 9;
	}
	
	if(yes){
		Record_min_high = data[0];
		Record_min_low = data[1];
		Record_second_high = data[3];
		Record_second_low = data[4];
		isSet = 0;
		isStartTiming = 1;
	}

	if(cancel){
		isSet = 0;
		isStartTiming = 0;
	}
}

void Ringtone(void){
	int i;
	if(Alarm_hour == hour && Alarm_minute == minute && Alarm_second == second){
		isRing = 1;
	}

	if(yes || minute - Alarm_minute >= 1){
		isRing = 0;
	}

	if(isRing){
		UARTStringPut((char*)"\n时间到了！\n");
	}
}

/////////////////////////////// UART SETTINGS ///////////////////////////////
void UART_INIT(void){
	int i,j;
	int k = 0;
	Trim(RxBuf);
	if (RxEndFlag){
		for(i = 0; i < sizeof(RxBuf); i++){
			for(j = 0; j < 10; j++){
				if(numbers[j]==RxBuf[i+1]){
					Serial[k]=j;
					k++;
				}
			}	
		}

		if(strncmp(RxBuf, "INITCLOCK", 5) == 0){
			Reset();
		}
		
		else if(strncmp(RxBuf,"SET",3) == 0){
			if(strncmp(RxBuf,"SETTIME",7) == 0){
				hour_high = Serial[0];
				hour_low = Serial[1];
				minute_high = Serial[2];
				minute_low = Serial[3];
				second_high = Serial[4];
				second_low = Serial[5];
				UARTStringPut((char*)"\n设置时间成功\n");
			}
			else if(strncmp(RxBuf,"SETDATE",7) == 0){
				year_high = Serial[2];
				year_low = Serial[3];
				month_high = Serial[4];
				month_low = Serial[5];
				day_high = Serial[6];
				day_low = Serial[7];
				UARTStringPut((char*)"\n设置日期成功\n");
			}
			else if(strncmp(RxBuf,"SETALARM",8) == 0){
				Alarm_hour_H = Serial[0];
				Alarm_hour_L = Serial[1];
				Alarm_min_H = Serial[2];
				Alarm_min_L = Serial[3];
				Alarm_sec_H = Serial[4];
				Alarm_sec_L = Serial[5];
				UARTStringPut((char*)"\n设置闹钟时间成功\n");
			}
			else if(strncmp(RxBuf,"SETSTWATCH",10) == 0){
				Record_min_high = Serial[0];
				Record_min_low = Serial[1];
				Record_second_high = Serial[2];
				Record_second_low = Serial[3];
				UARTStringPut((char*)"\n设置倒计时成功\n");
			}
			else{
				UARTStringPut((char*)"\n输入命令有误！请输入“?”来查看所有命令\n" );
			}
			RxEndFlag=0; 
		}
		else if(strncmp(RxBuf,"GET",3) == 0){
			if(strncmp(RxBuf,"GETTIME",7) == 0){
				State_Get[0] = hour_high + '0';
				State_Get[1] = hour_low + '0';
				State_Get[2] = ':';
				State_Get[3] = minute_high + '0';
				State_Get[4] = minute_low + '0';
				State_Get[5] = ':';
				State_Get[6] = second_high + '0';
				State_Get[7] = second_low + '0';
				UARTStringPut((char*)"\n当前时间为：\n");
				UARTStringPut(State_Get);
			}
			else if(strncmp(RxBuf,"GETDATE",7) == 0){
				State_Get[0] = year_high + '0';
				State_Get[1] = year_low + '0';
				State_Get[2] = '-';
				State_Get[3] = month_high + '0';
				State_Get[4] = month_low + '0';
				State_Get[5] = '-';
				State_Get[6] = day_high + '0';
				State_Get[7] = day_low + '0';
				UARTStringPut((char*)"\n当前日期为：\n");
				UARTStringPut(State_Get);
			}
			else if(strncmp(RxBuf,"GETALARM",8) == 0){
				State_Get[0] = Alarm_hour_H + '0';
				State_Get[1] = Alarm_hour_L + '0';
				State_Get[2] = ':';
				State_Get[3] = Alarm_min_H + '0';
				State_Get[4] = Alarm_min_L + '0';
				State_Get[5] = ':';
				State_Get[6] = Alarm_sec_H + '0';
				State_Get[7] = Alarm_sec_L + '0';
				UARTStringPut((char*)"\n设定的闹钟时间为：\n");
				UARTStringPut(State_Get);
			}
			else{
				UARTStringPut((char*)"输入命令有误！请输入“?”来查看所有命令\n" );
			}
			RxEndFlag = 0;
		}
		else if(strncmp(RxBuf, "OPENALARM", 9) == 0){
			Alarm_isEnable = 1;
			RxEndFlag = 0;
			UARTStringPut((char*)"\n闹钟已开启\n" );
		}
		else if(strncmp(RxBuf, "CLOSEALARM", 10) == 0){
			Alarm_isEnable = 0;
			RxEndFlag = 0;
			UARTStringPut((char*)"\n闹钟已关闭\n" );
		}
		else if(strncmp(RxBuf, "PAUSE", 5) == 0){
			isStartTiming = 0;
			RxEndFlag = 0;
			UARTStringPut((char*)"\n倒计时暂停\n" );
		}
		else if(RxBuf[0] == '?')
		{
			UARTStringPut((char*)"本数字钟可使用命令如下：\n");
			UARTStringPut((char*)"1.重启命令：\n");
			UARTStringPut((char*)"INITCLOCK			软重启数字钟，保留当前时间日期\n\n");
			UARTStringPut((char*)"2.设置命令：\n");
			UARTStringPut((char*)"SET TIME XX:YY:ZZ	设置时间为XX时YY分ZZ秒\n");
			UARTStringPut((char*)"SET DATE XXXX-YY-ZZ	设置日期为XXXX年YY月ZZ日\n");
			UARTStringPut((char*)"SET ALARM XX:YY:ZZ	设置闹钟时间为XX时YY分ZZ秒\n");
			UARTStringPut((char*)"OPENALARM		打开闹钟\n");
			UARTStringPut((char*)"CLOSEALARM		关闭闹钟\n\n");
			UARTStringPut((char*)"3.获取状态命令：\n");
			UARTStringPut((char*)"GET TIME		获取当前时间\n");
			UARTStringPut((char*)"GET DATE		获取当前日期\n");
			UARTStringPut((char*)"GET ALARM		获取当前闹钟时间\n\n");
			UARTStringPut((char*)"注：以上命令均不区分大小写且有空格容错功能。");
			RxEndFlag = 0;
		}
		else
		{
			UARTStringPut((char*)"输入命令有误！请输入“?”来查看所有命令\n" );
			RxEndFlag = 0;
			}
	}
}

//////////////////////////////////////// BASIC FUNCTION FROM PREVIOUS EXP ///////////////////////////////////
void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){}
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_5);		
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPinConfigure(GPIO_PK5_M0PWM7);
	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
}

void S800_PWM_Init(void)  
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) { } 
	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); 
	PWMGenConfigure(PWM0_BASE,PWM_GEN_3, PWM_GEN_MODE_DOWN 
	|PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,BeepPeriod);
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,BeepPeriod/4);
	PWMGenEnable(PWM0_BASE,PWM_GEN_3); 
}

void S800_Timer0_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, (ui32SysClock/100));
	TimerEnable(TIMER0_BASE, TIMER_A);
}

// I2C0 

void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}


uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
  //	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(200);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
	Delay(200);
	return value;
}

// URAT

void S800_UART_Init(void) 
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello, world! \n Version 20240614 \n 姓名：刘璜逸 学号：522031910603\r\n");
}

void UART0_Handler(void) 
{ 
	uint32_t ulStatus;
	ulStatus = UARTIntStatus(UART0_BASE, true); 
	UARTIntClear(UART0_BASE, ulStatus); 
	while( UARTCharsAvail(UART0_BASE) )
	{ 
		RxBuf[cnt] = toupper(UARTCharGetNonBlocking(UART0_BASE));
		if(RxBuf[cnt] != '\n')
		{
			cnt++;
		}
		else
		{
			RxBuf[cnt] = '\0';
			RxEndFlag=1; 
			cnt = 0;
			break;
		}
	}
} 

void UARTStringGet(char *msg)
{
	while(1)
	{
		*msg = UARTCharGet(UART0_BASE); 
		if (*msg =='\r') 
		{ 
		 *msg = UARTCharGet(UART0_BASE); 
		 break; 
		}
		msg++; 
	}
	*msg = '\0';
}

void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}

//TIMER0
void TIMER0A_Handler(void)
{	
	int i;
	uint32_t intStatus;
	intStatus = TimerIntStatus(TIMER0_BASE, true);
	TimerIntClear(TIMER0_BASE, intStatus ); 
	for(i = 0; i < 8; i++)
	{
		if(Key & (1 << i))
		{
			Key_flag[i] = 1;
			Key_count[i]++;
			isRelease[i] = 0;
		}
		else
		{
			Key_flag[i] = 0;
			Key_count[i] = 0;
			isRelease[i] = 1;
		}
		if(Key_flag[i] && Key_count[i] > 2 && Key_count[i] < 50)
		{
			isShortPress[i] = 1;
			isLongPress[i] = 0;
		}
		else if(Key_flag[i] && Key_count[i] > 100)
		{
			isShortPress[i] = 0;
			isLongPress[i] = 1;
			Key_flag[i] = 0;
			Key_count[i] = 0;
		}
		
		if(isLongPress[i] && i<4)
		{
			isSet = 1 - isSet;
			set_flag = isSet;
			isLongPress[i] = 0;
		}
		if(isLongPress[6]&&isLongPress[7]) Reset();
		if(isLongPress[4]&&isLongPress[5]) isReverse = 1 - isReverse;
	}
		if(isStartTiming && !(Record_milisecond_low == 0 && Record_milisecond_high == 0 && Record_second_low == 0 && Record_second_high == 0 && Record_min_low == 0 && Record_min_high == 0))
	{
		Record_milisecond_low--;
		if(Record_milisecond_low < 0)
		{
			Record_milisecond_high--;
			Record_milisecond_low = 9;
		}
		if(Record_milisecond_high < 0)
		{
			Record_second_low--;
			Record_milisecond_high = 9;
		}
		if(Record_second_low < 0)
		{
			Record_second_high--;
			Record_second_low = 9;
		}
		if(Record_second_high < 0)
		{
			Record_min_low--;
			Record_second_high = 5;
		}
		if(Record_min_low < 0)
		{
			Record_min_high--;
			Record_min_low = 9;
		}
		if(Record_milisecond_low == 0 && Record_milisecond_high == 0 && Record_second_low == 0 && Record_second_high == 0 && Record_min_low == 0 && Record_min_high == 0)
		{
			isStartTiming = 0;
		}
	}
}


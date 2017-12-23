// ***********************************************************************
// STM32F103_RTC  - demonstration of using the RTC component
// derived from for USART Menu with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX, SysTick interrupts every 10us
// Added support for read/write to flash (for storing startup date/time)
// and support for configuring, reading and writing to/from the RTC
//
// Option 1 will get the time & date from the user, store it in flash then
//          configure and set the RTC.(resetting it to zero)
// Option 2 will read the saved setup time & date and add the elapsed time
//          read from the RTC counter then display the updated values.
// Option 5 Just writes the saved starting time & date to flash
// Option 6 Just reads the saved starting time & date from flash and displays it.
//
// By Eric M. Klaus   2/2016
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_flash.h"
#include "stdlib.h"

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

#define STORAGE_START_ADDRESS    ((uint32_t)0x08007000) // flash storage start address(last 4K of flash)
#define PAGE_SIZE  (uint16_t)0x400  // Page size = 1KByte - for Low and Medium Density Devices
#define PAGE0_BASE_ADDRESS      ((uint32_t)(STORAGE_START_ADDRESS + 0x000))

// Application specific time structure
typedef struct mytim {
   uint8_t tim_tic;         // 1/16th second tick counts
   uint8_t tim_sec;         // seconds (0 to 59)
   uint8_t tim_min;         // minutes (0 to 59)
   uint8_t tim_hour;        // hours (0 to 23)
   uint8_t tim_mday;        // day of the month (1 to 31)
   uint8_t tim_mon;         // month, (0 to 11)
   uint16_t tim_year;       // full year eg: 2016
}mytim;

// ***********************************************************************
// * Declare function prototypes
//
void showMenu(void);
void processMenuCmd(char cmd);

// ***********************************************************************
//  RTC / Date Time Functions
// ***********************************************************************
void readTimeFromFlash(mytim *date_tim);
void writeTimeToFlash(mytim *date_tim);
void showTime(mytim *date_tim);
void getTime(mytim *date_time);
void calcNewDate(mytim *date_time, uint32_t RTCval);
void addSecond(mytim *date_time);
void addMinute(mytim *date_time);
void addHour(mytim *date_time);
void addDay(mytim *date_time);
int daysInMonth(int month, int year);

void RTC_Configuration(void);
// ***********************************************************************


// SysTick based delay routines
void SysTick_Handler(void);

// Looping style delay routines (still controlled by SysTick)
//    NOTE: provided as examples, none used in this example
void Delay_us(const uint32_t usec);
void Delay_ms(const uint32_t msec);

// USART routines ** specific to USART1
void SetupUSART(void);
void USART_PutChar(char c);
void USART_PutStr(char *str);
void USART_PutHexByte(unsigned char byte);
int USART_GetStr(char *buf, int len);

// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  // state of LED
char msgBuf[80]; // general purpose string buffer

mytim timCurrent, timNew;  // time & date buckets

int main(void)
{
 int k=0;
 timCurrent.tim_tic=16;  // Un-initialized date time

 // At this point startup has executed invoking SystemInit()
 // which selects the HSE clock and configures it to run at 74MHz.

 // ** Configures the SysTick event to fire every 10us	**
 SysTick_Config(SystemCoreClock / 100000);

 // Configure the GPIO pin for the LED (PC13)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTB Enable
 GPIO_InitStruct.GPIO_Pin = LED_PIN; // Configure Led pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LED_PORT, &GPIO_InitStruct);

 SetupUSART();

 GPIO_ResetBits(LED_PORT, LED_PIN);  //LED ON
 TimingDelay = 50000;                // Blink every 1/2 sec.

 showMenu();  // Display the user menu

 while(1)
   {
    if(TimingDelay==0)
	  {
	   TimingDelay = 50000;  //Reset the 1/2 sec. delay
	   if(led1_val)
	     {
          GPIO_ResetBits(LED_PORT, LED_PIN); // LED1 OFF
		  led1_val=0;
	     }
	   else
	     {
		  GPIO_SetBits(LED_PORT, LED_PIN); // LED1 ON
		  led1_val=1;
	     }

	   // ** add your code here to run every 1/2 second... ***
	  }

	// Is a byte available from UART?
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	  {
		k = USART_ReceiveData(USART1);
		USART_PutChar(k);
		processMenuCmd(k);
	  }

	// ** add your code to do something useful here .. ***
   }
}

// ***********************************************************************
// showMenu()    Output the user menu to the USART
// ***********************************************************************
void showMenu(void)
{
  USART_PutStr("\r\n **** USARTx Menu **** \r\n");
  USART_PutStr(" 1. Set Time & Date\r\n");
  USART_PutStr(" 2. Read Time & Date\r\n");
  USART_PutStr(" 3. Option Three \r\n");
  USART_PutStr(" 4. Blink LED\r\n");
  USART_PutStr(" 5. Flash Write Test \r\n");
  USART_PutStr(" 6. Flash Read Test \r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;
  uint32_t RTCVal;
  switch(cmd)
  {
   case '1':
	   getTime(&timCurrent);  // Ask the user for date & time
	   USART_PutStr("\r\n  ");
	   if(timCurrent.tim_tic < 16)
	     {
		  showTime(&timCurrent);
		  writeTimeToFlash(&timCurrent);
	      //RTC_run();
		  RTC_Configuration();
		  USART_PutStr("\r\n RTC Config. Complete...");
	      /* Change the current time */
	      RTC_SetCounter(15);
	      /* Wait until last write operation on RTC registers has finished */
	      RTC_WaitForLastTask();
	      USART_PutStr(" Timer Reset..");

	      BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
	      USART_PutStr(" Done\r\n");
	     }
   break;
   case '2':
	   USART_PutStr("\r\n  RCT Value: \r\n");
	   RTCVal = RTC_GetCounter();
	   utoa(RTCVal, msgBuf, 10);
	   USART_PutStr(msgBuf);

	   USART_PutStr("\r\n  Date: ");
	   readTimeFromFlash(&timNew);
	   calcNewDate(&timNew, RTCVal);
	   showTime(&timNew);
	   USART_PutStr("\r\n");
   break;
   case '3':
	   USART_PutStr("\r\n  You Selected Option Three !!! \r\n");
   break;
   case '4':
	   USART_PutStr("\r\n  Blink LED...  \r\n");
	   for(x=1; x<20; x++)
	     {
		   GPIO_SetBits(LED_PORT, LED_PIN); // LED1 ON
		   Delay_ms(100);
		   GPIO_ResetBits(LED_PORT, LED_PIN); // LED1 OFF
		   Delay_ms(100);
	     }
	   USART_PutStr(" Done...\r\n");
   break;

   case '5':  // Flash Write test
	   if(timCurrent.tim_tic > 15)
	     {
	      getTime(&timCurrent);  // Ask the user for date & time
	     }

	   if(timCurrent.tim_tic < 16)  // Valid date & time entered
	     {
		   writeTimeToFlash(&timCurrent);  // Save it in flash memory
	     }

	   USART_PutStr("\r\n Done \r\n");
   break;
   case '6':  // Flash Read test
	   x = 0;
	   USART_PutStr(" Reading Flash ...");
	   readTimeFromFlash(&timNew);
	   showTime(&timNew);
	   USART_PutStr("\r\n");
   break;


   case 'M':
   case 'm':
	   showMenu();
   break;
  }

}


// ***********************************************************************
// RTC_Configuration()
// Configure the RTC and prepare it for writing.
// Counter will be running at 16HZ (each count = 1/16 second)
// ***********************************************************************
void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second *** NOT using interrupts for this demo *** */
    //RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    //RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    //RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Set RTC prescaler: set RTC period to 1/16 sec */
    RTC_SetPrescaler(2047); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(2047+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

// ***********************************************************************
// readTimeFromFlash(mytim *date_tim)
// ***********************************************************************
void readTimeFromFlash(mytim *date_tim)
{
 uint32_t  Address = PAGE0_BASE_ADDRESS;

 date_tim->tim_tic = (*(__IO uint8_t*)Address);
 Address ++;

 date_tim->tim_sec = (*(__IO uint8_t*)Address);
 Address ++;

 date_tim->tim_min = (*(__IO uint8_t*)Address);
 Address ++;

 date_tim->tim_hour = (*(__IO uint8_t*)Address);
 Address ++;

 date_tim->tim_mday = (*(__IO uint8_t*)Address);
 Address++;

 date_tim->tim_mon = (*(__IO uint8_t*)Address);
 Address++;

 date_tim->tim_year = (*(__IO uint16_t*)Address);
}


// ***********************************************************************
// writeTimeToFlash(mytim *date_tim)
// ***********************************************************************
void writeTimeToFlash(mytim *date_tim)
{
 uint16_t  FlashStatus, x;
 uint32_t  Address = PAGE0_BASE_ADDRESS;

 FLASH_Unlock();

 FlashStatus = FLASH_ErasePage(PAGE0_BASE_ADDRESS);
 //FlashStatus = FLASH_WaitForLastOperation(1000);
 if(FlashStatus != FLASH_COMPLETE)
   {
	   USART_PutStr("\r\n!!! Page Erase ERROR !!!\r\n");
	   return;
   }


 x = (date_tim->tim_sec << 8 ) + date_tim->tim_tic;

 FlashStatus = FLASH_ProgramHalfWord(Address, x);
 //FlashStatus = FLASH_WaitForLastOperation(1000);
 if(FlashStatus != FLASH_COMPLETE)
   {
	   USART_PutStr("\r\n!!! Write ERROR1 !!!\r\n");
	   return;
   }

 Address += sizeof(uint16_t);
 x = (date_tim->tim_hour << 8 ) + date_tim->tim_min;

 FlashStatus = FLASH_ProgramHalfWord(Address, x);
 //FlashStatus = FLASH_WaitForLastOperation(1000);
 if(FlashStatus != FLASH_COMPLETE)
   {
	   USART_PutStr("\r\n!!! Write ERROR2 !!!\r\n");
	   return;
   }

 Address += sizeof(uint16_t);
 x = (date_tim->tim_mon << 8 ) + date_tim->tim_mday;

 FlashStatus = FLASH_ProgramHalfWord(Address, x);
 //FlashStatus = FLASH_WaitForLastOperation(1000);
 if(FlashStatus != FLASH_COMPLETE)
   {
	   USART_PutStr("\r\n!!! Write ERROR3 !!!\r\n");
	   return;
   }

 x = date_tim->tim_year;
 Address += sizeof(uint16_t);
 FlashStatus = FLASH_ProgramHalfWord(Address, x);
 //FlashStatus = FLASH_WaitForLastOperation(1000);
 if(FlashStatus != FLASH_COMPLETE)
   {
	   USART_PutStr("\r\n!!! Write ERROR4 !!!\r\n");
	   return;
   }

}

// ***********************************************************************
//  showTime(mytim *date_tim)
// ***********************************************************************
void showTime(mytim *date_tim)
{
  char valBuf[5];

   utoa(date_tim->tim_mon, valBuf, 10);
   USART_PutStr(valBuf);
   USART_PutChar('\\');
   utoa(date_tim->tim_mday, valBuf, 10);
   USART_PutStr(valBuf);
   USART_PutChar('\\');
   utoa(date_tim->tim_year, valBuf, 10);
   USART_PutStr(valBuf);
   USART_PutChar(' ');
   utoa(date_tim->tim_hour, valBuf, 10);
   USART_PutStr(valBuf);
   USART_PutChar(':');
   utoa(date_tim->tim_min, valBuf, 10);
   USART_PutStr(valBuf);
   USART_PutChar(':');
   utoa(date_tim->tim_sec, valBuf, 10);
   USART_PutStr(valBuf);
   USART_PutChar(':');
   utoa(date_tim->tim_tic, valBuf, 10);
   USART_PutStr(valBuf);
   USART_PutChar(' ');
}


// ***********************************************************************
//  getTime(mytim *date_time)
// ***********************************************************************
void getTime(mytim *date_time)
{
 int x;
 char inBuf[5];

 date_time->tim_tic=16;  //Indicated input error

  USART_PutStr("\r\n Enter Year (4 digits): ");
  USART_GetStr(inBuf, 4);
  x = (int)strtol(inBuf, 0, 10);
  if((x < 1900)||(x > 9999))
    {
	  USART_PutStr("\r\n Invalid Year Entered\r\n ");
	  return;
    }

  date_time->tim_year = x;

  USART_PutStr("\r\n Enter Month (0-11): ");
  USART_GetStr(inBuf, 2);
  x = (int)strtol(inBuf, 0, 10);
  if(x > 11)
    {
	  USART_PutStr("\r\n Invalid Month Entered\r\n ");
	  return;
    }

  date_time->tim_mon = x;

  USART_PutStr("\r\n Enter Date (1-31): ");
  USART_GetStr(inBuf, 2);
  x = (int)strtol(inBuf, 0, 10);
  if((x < 1)||(x > daysInMonth(date_time->tim_mon, date_time->tim_year)))
    {
	  USART_PutStr("\r\n Invalid Date\r\n ");
	  return;
    }

  date_time->tim_mday = x;

  USART_PutStr("\r\n Enter Hour (0-23): ");
  USART_GetStr(inBuf, 2);
  x = (int)strtol(inBuf, 0, 10);
  if(x > 23)
    {
	  USART_PutStr("\r\n Invalid Hour\r\n ");
	  return;
    }

  date_time->tim_hour = x;

  USART_PutStr("\r\n Enter Minutes (0-59): ");
  USART_GetStr(inBuf, 2);
  x = (int)strtol(inBuf, 0, 10);
  if(x > 59)
    {
	  USART_PutStr("\r\n Invalid Minutes\r\n ");
	  return;
    }

  date_time->tim_min = x;

  USART_PutStr("\r\n Enter Seconds (0-59): ");
  USART_GetStr(inBuf, 2);
  x = (int)strtol(inBuf, 0, 10);
  if(x > 59)
    {
	  USART_PutStr("\r\n Invalid Seconds\r\n ");
	  return;
    }

  date_time->tim_sec = x;

  date_time->tim_tic=0;  // Success
}


// ***********************************************************************
// calcNewDate(mytim *date_time, uint32_t RTCval)
// Add the elapsed time RTC counter to the current date value.
// Each count in RTCval = 1/16 second.
// ***********************************************************************
void calcNewDate(mytim *date_time, uint32_t RTCval)
{
  while(RTCval > 1382399)   // 16*60*60*24 = 1382400 (counts/DAY)
   {
    addDay(date_time);      // add 1 Day, adjust for months & year
    RTCval -= 1382400;
   }

  while(RTCval > 57599)    // 16*60*60 = 57600 counts/HOUR
   {
  	addHour(date_time);    //add 1 hour to (date_time) handle any overflow
	  RTCval -= 57600;
   }

  while(RTCval > 959)    //16*60=960  counts/MINUTE
   {
  	addMinute(date_time);  //add 1 minute to (date_time) handle any overflow
	  RTCval -= 960;
   }

  while(RTCval > 15)   //16 counts/SECOND
   {
	  addSecond(date_time);  //add 1 second to (date_time) handle any overflow
	  RTCval -= 16;
   }

  date_time->tim_tic = (uint8_t)RTCval;
}


// ***********************************************************************
//  addSecond(*date_time)  add 1 second to (date_time) handle any overflow
// ***********************************************************************
void addSecond(mytim *date_time)
{
 date_time->tim_sec++;
 if(date_time->tim_sec > 59)
   {
  	date_time->tim_sec=0;
   	addMinute(date_time);
   }
}

// ***********************************************************************
//  addMinute(*date_time) add 1 minute to (date_time) handle any overflow
// ***********************************************************************
void addMinute(mytim *date_time)
{
 date_time->tim_min++;
 if(date_time->tim_min > 59)
   {
  	date_time->tim_min=0;
   	addHour(date_time);
   }
}

// ***********************************************************************
//  addHour(*date_time) add 1 hour to (date_time) handle any overflow
// ***********************************************************************
void addHour(mytim *date_time)
{
 date_time->tim_hour++;
 if(date_time->tim_hour > 23)
   {
  	date_time->tim_hour=0;
   	addDay(date_time);
   }
}

// ***********************************************************************
//  addDay(*date_time) add 1 day to (date_time) handle any overflow
// ***********************************************************************
void addDay(mytim *date_time)
{
 date_time->tim_mday++;
 if(date_time->tim_mday > daysInMonth(date_time->tim_mon, date_time->tim_year))
   {
	 date_time->tim_mday=1;
	 date_time->tim_mon++;
	 if(date_time->tim_mon > 11)
	   {
		 date_time->tim_mon = 0;
		 date_time->tim_year++;
	   }
   }
}

// ***********************************************************************
//  daysInMonth(int month, int year)
// ***********************************************************************
int daysInMonth(int month, int year)
{
  int days=31;
  char leapyear = 0;

  if(month == 4 || month == 6 || month == 9 ||month == 11)
    {
	 days = 30;
    }
  else
    {
	  if(month == 2)
	    {
		 leapyear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);

		 if(leapyear == 0)
			days = 29;
		else
			days = 29;
	    }
    }

 return days;
}

// ***********************************************************************
//  SetupUSART()
//
// USART1 configured as follow:
// BaudRate = 9600 baud, Word Length = 8 Bits, One Stop Bit, No parity
// Hardware flow control disabled (RTS and CTS signals)
// Receive and transmit enabled
// USART Clock disabled
// USART CPOL: Clock is active low
// USART CPHA: Data is captured on the middle
// USART LastBit: The clock pulse of the last data bit is not output to
//                           the SCLK pin
// ***********************************************************************
void SetupUSART()
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 USART_InitTypeDef USART_InitStructure;

 // Enable USART1 and GPIOA clock
 RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

 // Configure USART1 Rx (PA10) as input floating
 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 // Configure USART1 Tx (PA9) as alternate function push-pull            */
 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 USART_InitStructure.USART_BaudRate            = 9600;
 USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
 USART_InitStructure.USART_StopBits            = USART_StopBits_1;
 USART_InitStructure.USART_Parity              = USART_Parity_No ;
 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
 USART_Init(USART1, &USART_InitStructure);
 USART_Cmd(USART1, ENABLE);
}

// ***********************************************************************
// ******** USART Utility functions **************************************
// ***********************************************************************
// ***********************************************************************
// USART_PutChar(char *c)
// ***********************************************************************
void USART_PutChar(char c)
{
  // write a character to the USART
  USART_SendData(USART1, (uint8_t) c);

  ///Loop until the end of transmission
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

// ***********************************************************************
// USART_PutStr(char *str)
// ***********************************************************************
void USART_PutStr(char *str)
{
  while(*str)
  {
    USART_PutChar(*str);
	str++;
  }
}

// ***********************************************************************
// USART_PutHexByte(char byte)
// ***********************************************************************
void USART_PutHexByte(unsigned char byte)
{
  char n = (byte >> 4) & 0x0F;
  // Write high order digit
  if(n < 10)
 	USART_PutChar(n + '0');
  else
	USART_PutChar(n - 10 + 'A');

  // Write low order digit
  n = (byte & 0x0F);
  if(n < 10)
 	USART_PutChar(n + '0');
  else
	USART_PutChar(n - 10 + 'A');
}

// ***********************************************************************
//	USART_GetStr(char *buf, int len)
//  Return length of input string
// ***********************************************************************
int USART_GetStr(char *buf, int len)
{
 int i=0;
 char k = 0;

 while(k != 0x0D)
    {
	  // Is a byte available from UART?
	 if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	   {
		k = USART_ReceiveData(USART1);  // get input char
		USART_PutChar(k);               // echo input char
		buf[i]=k;                       // store input char

		if(++i==len)  // Buffer Full = EXIT
	      break;

		if((k==0x0A)||(k==0x1B)) // LF or Esc = EXIT
			break;

		if((k==0x7F)&&(i>0))  // Backspace
			i--;
	   }
    }

  buf[i]=0;
  return i;
}


// ***********************************************************************
// ******* SysTick based Delay functions  ********************************
// ***********************************************************************
// ***********************************************************************
//  Simple microsecond delay routine
// ***********************************************************************
void Delay_us(const uint32_t usec)
{
  TimingDelay = usec/10;
  while(TimingDelay != 0);
}

// ***********************************************************************
//  Simple millisecond delay routine
// ***********************************************************************
void Delay_ms(const uint32_t msec)
{
  Delay_us(msec * 1000);
}


// ***********************************************************************
//  SystemTick IRQ handler
//  Decrements TimingDelay, increments usCounter & msCounter;
// ***********************************************************************
void SysTick_Handler(void)
{
 if (TimingDelay != 0x00)
   {
     TimingDelay--;
   }

 usCounter++;
 if(++ctMs > 99)
   {
	msCounter++;
	ctMs=0;
   }
}




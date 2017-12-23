// ***********************************************************************
// STM32F103_5110  - Nokia 5110 84,48 LCD display demo
// derived from template for USART Menu with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX and SysTick interrupts every 10us
//
// NOTE: LCD5110.c contains a useful core counter based timing delay routine
// By Eric M. Klaus   2/2016
// ***********************************************************************
#include "stdio.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "LCD5110.h"
#include "stdlib.h"     /* strtol */
#include "STlogo.h"

#define LED_PIN  GPIO_Pin_13
#define BUTTON_PIN GPIO_Pin_14
#define LED_PORT GPIOC

//******************************************************************************

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);


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

//static const char buffer[10000] = { 't', 'e', 's', 't',0 };
//static const char buffer1[20000] = { 'M', 'A', 'X', 'M','E','M',0};

// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  // state of LED
int counter1=0;  // blink counter
char msgBuf[80]; // general purpose string buffer
//unsigned int bufEnd;

int main(void)
{
 int k=0;
 // At this point startup has executed invoking SystemInit()
 // which selects the HSE clock and configures it to run at 74MHz.

 // ** Configures the SysTick event to fire every 100us	**
 SysTick_Config(SystemCoreClock / 10000);

 // Configure the GPIO pin for the LED (PC13)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTB Enable
 GPIO_InitStruct.GPIO_Pin = LED_PIN; // Configure Led pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LED_PORT, &GPIO_InitStruct);

 // Configure the GPIO pin for the Button (PC14)
  GPIO_InitStruct.GPIO_Pin = BUTTON_PIN;     // Configure button pin
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // Set as INPUT with Pull-up
  GPIO_Init(LED_PORT, &GPIO_InitStruct);

 //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

 SetupUSART();

 LcdInitialize();  // Initialize the LCD

 GPIO_ResetBits(LED_PORT, LED_PIN);  //LED ON
 TimingDelay = 50000;                // Blink every 1/2 sec.

 LcdClear();

 showMenu();  // Display the user menu

 LcdString("*Hello LCD**");
 LcdStringX2("!RESET!", 2, 5);

 while(1)
   {
    if(TimingDelay==0)
	  {
	   TimingDelay = 5000;  //Reset the 1/2 sec. delay
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
	    if(!GPIO_ReadInputDataBit(LED_PORT, BUTTON_PIN)) // Is button pushed?
	      {
	    	counter1++;
	    	itoa(counter1, msgBuf, 10);
	    	LCD_GotoXY(0,5);
	    	LcdString(msgBuf);
	      }

	    TimingDelay = 5000;  //Reset the 1/2 sec. delay
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
  USART_PutStr(" 1. Clear LCD \r\n");
  USART_PutStr(" 2. Send 12 Us \r\n");
  USART_PutStr(" 3. Init. LCD \r\n");
  USART_PutStr(" 4. Blink Backlight \r\n");
  USART_PutStr(" 5. Toggle GPIOB \r\n");
  USART_PutStr(" 6. LCD Message \r\n");
  USART_PutStr(" 7. Show Logo\r\n");
  USART_PutStr(" 8. Core Timing Test \r\n");
  USART_PutStr(" 9. MaxMem Test\r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x,v;
  unsigned int uVal;
  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n  Clear \r\n");
	   LcdClear();
   break;
   case '2':
	   USART_PutStr("\r\n  12 Us row 4 \r\n");
	   LCD_GotoXY(0,4);   // Col 0, Row 4
	   LcdString("UUUUUUUUUUU");
   break;
   case '3':
	   USART_PutStr("\r\n  LCD Init \r\n");
	   LcdInitialize();  // Initialize the LCD
	   LcdClear();
	   LCD_GotoXY(0,0);
	   LcdString("*Hello LCD**");
	   LcdStringX2("*RESET*", 2, 5);
   break;
   case '4':
	   USART_PutStr("\r\n  Blink Backlight\r\n");
	   for(x=1; x<20; x++)
	     {
		   GPIO_SetBits(LCD_PORT, PIN_BKLT); // Backlight Off
		   Delay_ms(100);
		   GPIO_ResetBits(LCD_PORT, PIN_BKLT); // Backlight On
		   Delay_ms(100);
	     }
	   USART_PutStr(" Done...\r\n");
   break;
   case '5':
	   USART_PutStr("\r\n  Blink GPIOB  \r\n");
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	   GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_All;
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
	    GPIO_Init(GPIOB, &GPIO_InitStruct);

		   for(x=1; x<20; x++)
		     {
			   GPIO_SetBits(GPIOB, GPIO_Pin_All);
			   Delay_ms(100);
			   GPIO_ResetBits(GPIOB, GPIO_Pin_All);
			   Delay_ms(100);
		     }

		   LcdInitialize();  // Initialize the LCD
		   USART_PutStr(" Done...\r\n");
   break;
   case '6':
	   USART_PutStr("\r\n  Enter Message[12 chars max] :");
	   x = USART_GetStr(msgBuf, 12);
	   USART_PutStr("\r\n  Enter Row#[0-5]:");
	   USART_GetStr(msgBuf+20, 1);
	   v = msgBuf[20]-'0';
	   LCD_GotoXY(0,v);
	   LcdString(msgBuf);
	   USART_PutStr("Done...\r\n");
   break;
   case '7':
	   LCD_GotoXY(0,0);
	   LcdBmp(STlogo);
	   USART_PutStr("Done...\r\n");
   break;
   case '8':
	   USART_PutStr("\r\n  Enter Delay Loop Count :");
	   x = USART_GetStr(msgBuf, 10);
	   while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET);
	   uVal = (unsigned int)strtol(msgBuf, NULL, 10);
	   EnableCoreTiming();
	   usCounter=0;
	   for(x=1; x<10; x++)
	     {
		   GPIO_ResetBits(LED_PORT, LED_PIN); // LED1 ON
		   CoreTimingDelay(uVal);
		   GPIO_SetBits(LED_PORT, LED_PIN); // LED1 OFF
		   CoreTimingDelay(uVal);
	     }
       itoa(usCounter, msgBuf, 10);
       USART_PutStr("\r\n  usCounter:");
       USART_PutStr(msgBuf);
	   USART_PutStr("\r\n  Press a key...");
	   while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != SET);
	   showMenu();
       break;

   case '9': //Max Mem test
	   //USART_PutStr((char *)buffer);
	   //USART_PutStr((char *)buffer1);
	   //bufEnd = (unsigned int)&buffer1[19999];
	   //sprintf(msgBuf, "\r\n End of buffer1= %06X \r\n", bufEnd);
	   //USART_PutStr(msgBuf);
   break;
   case 'M':
   case 'm':
	   showMenu();
   break;
  }

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

		if((k==0x0A)||(k==0x0D)||(k==0x1B)) // LF,CR or Esc = EXIT
			break;

		if(++i==len)  // Buffer Full = EXIT
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
  TimingDelay = usec/100;
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






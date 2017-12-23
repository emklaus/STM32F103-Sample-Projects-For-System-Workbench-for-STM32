// ***********************************************************************
// STM32F103_VGA1
// derived from STMF103_USARTx template with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX, SysTick interrupts every 1ms
//
// Added VGA Output Support
//      ------------------------
//    (   1   2   3   4   5      )     1,2,3  Red Green Blue signals PA7 (Red)*(connector wire colors not color signals)
//     \    6   7   8   9   10  /         13  HSYNC                  PA8 (green)
//      \  11 12  13  14  15   /          14  VSYNC                  PA1 (yellow)
//        --------------------         5,6,10 GND                         
//
//
// Based on a project by Ruben H. Meleca  http://www.artekit.eu
// Modified by Eric M. Klaus   2/2016
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "stm32f10x_dma.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"

#include "sys.h"
#include "video.h"
#include "gdi.h"

extern void demoInit(uint16_t demo_no);

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);



// USART routines ** specific to USART1
void SetupUSART(void);
void USART_PutChar(char c);
void USART_PutStr(char *str);
void USART_PutHexByte(unsigned char byte);
int USART_GetStr(char *buf, int len);

// Global timing counters
extern volatile uint32_t sysTiming;

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  // state of LED
char msgBuf[80]; // general purpose string buffer

int main(void)
{
 int k=0;
 // At this point startup has executed invoking SystemInit()
 // which selects the HSE clock and configures it to run at 74MHz.

/* TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable */
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA |
						RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);



 sysInitSystemTimer();

 // Configure the GPIO pin for the LED (PC13)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTB Enable
 GPIO_InitStruct.GPIO_Pin = LED_PIN; // Configure Led pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LED_PORT, &GPIO_InitStruct);

 SetupUSART();

 GPIO_ResetBits(LED_PORT, LED_PIN);  //LED ON
 sysTiming = 500;                // Blink every 1/2 sec.

 showMenu();  // Display the user menu

 while(1)
   {

    if(sysTiming==0)
	  {
    	sysTiming = 500;  //Reset the 1/2 sec. delay
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
  USART_PutStr(" 1. Run Text Demo \r\n");
  USART_PutStr(" 2. Run Select Demo \r\n");
  USART_PutStr(" 3. Option Three \r\n");
  USART_PutStr(" 4. Blink LED \r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;
  uint32_t i;
  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n Starting VGA Demo... \r\n");

	   vidInit();

		vidClearScreen();
		gdiRectangle(0,0,(VID_PIXELS_X - 1),VID_VSIZE - 1,0);
		gdiDrawTextEx(160, 40, "STM32F103C8T6", GDI_ROP_COPY);
		gdiDrawTextEx(170, 55, "ARM CORTEX", GDI_ROP_COPY);
		gdiDrawTextEx(100, 75, "MINIMUM SYSTEM DEVELOPMENT BOARD", GDI_ROP_COPY);
		gdiDrawTextEx(145, 90, "VGA DEMO PROJECT", GDI_ROP_COPY);

		i = 50000000;
		while(i--);

	   TIM_DeInit(TIM1);   // TIM1 used PA9 also used by USART
                           // need to stop video to use USART
	   USART_DeInit(USART1);  // De & Re-Initialize USART
	   SetupUSART();
	   USART_PutStr("\r\n Done\r\n");
   break;
   case '2':
	   USART_PutStr("\r\n  Enter Demo# [1-6]:");
	   USART_GetStr(msgBuf, 1);
	   x = (int)msgBuf[0]-'0';
	   vidInit();

	   demoInit(x);

		i = 40000000;
		while(i--);

	   TIM_DeInit(TIM1);

	   USART_DeInit(USART1);
	   SetupUSART();
	   USART_PutStr("\r\n Done\r\n");
   break;
   case '3':
	   USART_PutStr("\r\n  You Selected Option Three !!! \r\n");
   break;
   case '4':
	   USART_PutStr("\r\n  Blink LED...  \r\n");
	   for(x=1; x<20; x++)
	     {
		   GPIO_SetBits(LED_PORT, LED_PIN); // LED1 ON
		   sysDelayMs(100);
		   GPIO_ResetBits(LED_PORT, LED_PIN); // LED1 OFF
		   sysDelayMs(100);
	     }
	   USART_PutStr(" Done...\r\n");
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
 USART_ClockInitTypeDef USART_ClockInitStruct;

 // Enable USART1 and GPIOA clock
 RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

 USART_ClockStructInit(&USART_ClockInitStruct);
 USART_ClockInit(USART1, &USART_ClockInitStruct);

 // Configure USART1 Rx (PA10) as input floating
 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 // Configure USART1 Tx (PA9) as alternate function push-pull            */
 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
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




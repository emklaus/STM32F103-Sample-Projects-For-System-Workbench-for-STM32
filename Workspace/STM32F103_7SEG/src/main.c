// ***********************************************************************
// STM32F103_7SEG  - & Segment LED Display demo, speaker and pushbutton
// derived from template for USART Menu with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX and SysTick interrupts every 10us
//
//
//****************7 Segment Map ********************************
//      -a-             -1-                 -x01-
//   f |    |b      32 |    | 2        x20 |     | x02
//      -g-             -64-                -x40-
//   e |    |c      16 |    |4         x10 |     | x04
//      -d-   (dp)      -8-   (128)         -x08-   (x80)
//
//  GPIO Pins: a=PA0, b=PA1, c=PA2, d=PA3, e=PA4, f=PA5, g=PA6, dp=PA7
// wire colors: black, red,  green, yellow, red,  yellow, black, green
//
//  pushbutton PB12, speaker PB13
//
// By Eric M. Klaus   2/2016
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

#define SPEAKER_PIN GPIO_Pin_13
#define BUTTON_PIN  GPIO_Pin_12
#define BUTTON_PORT GPIOB

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);

void beep(int ct, int delay);
char AsciiToHexVal(char b);
unsigned char HexStrToByte(char *buf);

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

int led1_val=0, digitNo=0, digit_count=0;
char msgBuf[80]; // general purpose string buffer
// 7 segment pattern table for hex digits 0-F
const uint8_t segval[16] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
					        0x7f, 0x6f, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

int main(void)
{
 int k=0;
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

 // Configure the GPIO pins for the 7 Segment LED Display(PA0-PA7)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// Clock PORTA Enable
 GPIO_InitStruct.GPIO_Pin = 0x00FF; // Configure PA0-PA7
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(GPIOA, &GPIO_InitStruct);

 // Configure the GPIO pins for the button & speaker(PB12 & PB13)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// Clock PORTB Enable
 GPIO_InitStruct.GPIO_Pin = SPEAKER_PIN; // Configure Speaker Pin As Output
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

 GPIO_InitStruct.GPIO_Pin = BUTTON_PIN; // Configure Speaker Pin As Output
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;// Set Input-PullUp
 GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

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
	   if(digit_count) // Counting turned on?
	     {
	      GPIO_ResetBits(GPIOA, 0x00FF);  // Clear PA0-PA7
	      GPIO_SetBits(GPIOA, (uint16_t)segval[digitNo]); //Set PA0-PA7
	      if(++digitNo >15) //Increment digit#
	         digitNo=0;
	     }

	   // Is button pressed?
	   if(! GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN))
		   beep(250, 500); //1Khz  1/4 sec.
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
  USART_PutStr(" 1. Output Byte \r\n");
  USART_PutStr(" 2. Counting On/Off \r\n");
  USART_PutStr(" 3. Beep 400hz\r\n");
  USART_PutStr(" 4. Option Four \r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;
  uint8_t data;

  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n  Enter Data byte[2 hex chars]: ");
	   USART_GetStr(msgBuf, 2);
	   data = HexStrToByte(msgBuf);
	   GPIO_ResetBits(GPIOA, 0x00FF);
	   GPIO_SetBits(GPIOA, (uint16_t)data);
   break;
   case '2':
	   if(digit_count)
	   {
		digit_count=0;
		USART_PutStr("\r\n Counting OFF\r\n");
	   }
	   else
	   {
		digit_count=1;
		USART_PutStr("\r\n Counting ON\r\n");
	   }
   break;
   case '3':
	   USART_PutStr("\r\n  You Selected Option Three !!! \r\n");
	   beep(400, 1250);  // 400Hz for 1 sec.
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
   case 'M':
   case 'm':
	   showMenu();
   break;
  }

}

//************************************************************************
//
//************************************************************************
void beep(int ct, int delay)
{
 int x;
 for(x=0; x< ct; x++)
   {
    GPIO_SetBits(BUTTON_PORT, SPEAKER_PIN);
	Delay_us(delay);
	GPIO_ResetBits(BUTTON_PORT, SPEAKER_PIN);
	Delay_us(delay);
   }

}
//************************************************************************
// AsciiToHexVal(char b)
// Convert a single ASCII character to it's 4 bit value 0-9 or A-F
// Note: no value error checking is done. Valid HEX characters is assumed
//************************************************************************
char AsciiToHexVal(char b)
{
 char v= b & 0x0F;  // '0'-'9' simply mask high 4 bits
 if(b>'9')
   v+=9;           // 'A'-'F' = low 4 bits +9
 return v;
}

//************************************************************************
// HexStrToByte(char *buf)
// Convert a 2 character ASCII string to a 8 bit unsigned value
//************************************************************************
unsigned char HexStrToByte(char *buf)
{
 unsigned char v;
 v= AsciiToHexVal(buf[0]) * 16;
 v+= AsciiToHexVal(buf[1]);
 return v;
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




// ***********************************************************************
// STM32F103_EXTIx  External Interrupt demo
// derived from template for USART Menu with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX,  SysTick interrupts every 10us
// Added support for generating an external interrupt with pushbutton on PB12
//
// ** Use this project as a template for other work ***
// By Eric M. Klaus   3/2016
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);

// EXTI Demo routines
void NVIC_Configuration(void);

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
void USART_PutHexWord(uint16_t word);
int USART_GetStr(char *buf, int len);
void USART_PutMsgAndVal(char *msg, uint16_t val, char addNL);

// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

GPIO_InitTypeDef GPIO_InitStruct;
EXTI_InitTypeDef EXTI_InitStructure;

int led1_val=0;  // state of LED
char msgBuf[80]; // general purpose string buffer
static uint16_t __IO ctIRQ=0;

int main(void)
{
 int k=0;

 SystemInit();

 // At this point startup has executed invoking SystemInit()
 // which selects the HSE clock and configures it to run at 74MHz.

 // ** Configures the SysTick event to fire every 10us	**
 SysTick_Config(SystemCoreClock / 100000);


 // Enable GPIOA, GPIOB, GPIOC and AFIO clock
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
         | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

 // Configure the GPIO pin for the On-Board LED (PC13)
 GPIO_InitStruct.GPIO_Pin = LED_PIN; // Configure Led pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LED_PORT, &GPIO_InitStruct);

 // Configure the GPIO pin for the BUTTON (PB12)
 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12; // Configure Button pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;// Set Input with Pull-Up
 GPIO_Init(GPIOB, &GPIO_InitStruct);

 // Configure the GPIO pin for the RED LED (PB13)
 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; // Configure Red LED pin (PB13)
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(GPIOB, &GPIO_InitStruct);


 SetupUSART();

 GPIO_ResetBits(LED_PORT, LED_PIN);  //LED ON
 TimingDelay = 25000;                // Blink every 1/2 sec.

 showMenu();  // Display the user menu

 //NVIC_Configuration();

 while(1)
   {
    if(TimingDelay==0)
	  {
	   TimingDelay = 25000;  //Reset the 1/2 sec. delay
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

	   // ** add your code here to run every 1/4 second... ***

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
  USART_PutStr("\r\n **** EXTI DEmo Menu **** \r\n");
  USART_PutStr(" 1. Configure Interrupts \r\n");
  USART_PutStr(" 2. Display Counts \r\n");
  USART_PutStr(" 3. Option Three \r\n");
  USART_PutStr(" 4. Option Four \r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;
  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n  Configuring IRQ for PB12  ");

	   NVIC_Configuration();

	   // Connect EXTI Line12 to PB12
	   GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

	   // Configure EXTI Line12 to generate an interrupt on rising or falling edge
	   // Pin should have been previously configures and an INPUT pin with pull-up enabled
	   EXTI_InitStructure.EXTI_Line = EXTI_Line12;                //Pin Px12
	   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    // High to Low change will generate an Interrupt
	   EXTI_InitStructure.EXTI_LineCmd = ENABLE;                  // Enable Interrupt
	   EXTI_Init(&EXTI_InitStructure);

	   ctIRQ=0;
	   USART_PutStr(" Done\r\n");
   break;
   case '2':
	   USART_PutMsgAndVal("\r\n IRQ Counts: ", ctIRQ, 1);

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
   case 'M':
   case 'm':
	   showMenu();
   break;
  }

}



//***************************************************************************
//Configure the nested vectored interrupt controller.
//***************************************************************************
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // Set the Vector Table base location at 0x08000000
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    // Configure one bit for preemption priority
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    // Enable the EXTI15_10 Interrupt (IRQ vector for Pins Px10-Px15)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
// USART_PutHexWord(word)
// ***********************************************************************
void USART_PutHexWord(uint16_t word)
{
 uint8_t byte;
 byte = (word >> 8)& 0x00FF;
 USART_PutHexByte(byte);
 byte = word & 0x00FF;
 USART_PutHexByte(byte);
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

		if((k=='\r')||(k=='\n')||(k==0x1B)) // CR,LF or Esc = EXIT
			break;

		if((k==0x7F)&&(i>1))  // Backspace (maybe specific to PuTTY)
		  {
			i--;
			continue;
		  }

		if(++i==len)  // Buffer Full = EXIT
	      break;
	   }
    }

  buf[i]=0;
  return i;
}

// ***********************************************************************
// USART_PutMsgAndVal(char *msg, int val, uint8_t addNL)
// output a string followed by a HEX value.
// NOTE: useful for error message and error code
// ***********************************************************************
void USART_PutMsgAndVal(char *msg, uint16_t val, char addNL)
{
  USART_PutStr("\r\n");
  USART_PutStr(msg);
  if(val <256)
	USART_PutHexWord((uint8_t)val);
  else
	USART_PutHexWord((uint16_t)val);

  if(addNL)
    USART_PutStr("\r\n");
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

 usCounter+=10;
 if(++ctMs > 99)
   {
	msCounter++;
	ctMs=0;
   }
}

// ***********************************************************************
//  EXTI15_10_IRQHandler(void)
//  IRQ handler for Pins Px10-Px15 (wher x can be ANY port A,B,C,D...)
// ***********************************************************************
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
     ctIRQ++;

     if(ctIRQ & 0x0001)
       GPIO_SetBits(GPIOB, GPIO_Pin_13);
     else
       GPIO_ResetBits(GPIOB, GPIO_Pin_13);

     //Clear the EXTI line 9 pending bit
     EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

void HardFault_Handler(void)
{
	TimingDelay--;
}

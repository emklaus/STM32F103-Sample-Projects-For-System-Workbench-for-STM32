// ***********************************************************************
// STM32F103_TIMx
// derived from template for USART Menu with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX,  SysTick interrupts every 10us
//
// Use TIM3 to produce PWM controlling an LED on PA6
//
// ** Use this project as a template for other work ***
// By Eric M. Klaus   2/2016
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "stm32f10x_tim.h"
#include "misc.h"
#include "stdlib.h"  //for itoa(), atoi()

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

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
void USART_PutHexWord(uint16_t word);
int USART_GetStr(char *buf, int len);
void USART_PutMsgAndVal(char *msg, uint16_t val, char addNL);

// Timer3 PWM Demo Setup Routine
void Setup_TIM3_PWMOutput(void);
void ResetTIM3_CH1_PWM(int pwm);

// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

static __IO unsigned int TIM2Counter;

GPIO_InitTypeDef GPIO_InitStruct;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

int led1_val=0;  // state of LED
int pwm_val=333, pwm_dir=0, pwm_fade=0; // PWM control variables
char msgBuf[80]; // general purpose string buffer

int main(void)
{
 int k=0;
 TIM2Counter = 0;
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


 // *********************************************************************
 // *********** TIMER ROUTINES ******************************************
 // *********************************************************************
 // ******* Enable the TIM2 global Interrupt  **********
 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

 NVIC_Init(&NVIC_InitStructure);

 /* TIM2 clock enable */
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


 // Timer2  Time base configuration - System clock is 72MHz
 TIM_TimeBaseStructure.TIM_Period = 1000 - 1;  // 1 MHz down to 1 KHz (1 ms)
 TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 72 MHz Clock down to 1 MHz (adjust per your clock)
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

 /* TIM IT enable */
 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

 /* TIM2 enable counter */
 TIM_Cmd(TIM2, ENABLE);
 // *********************************************************************
 // *********** END - TIMER ROUTINES ************************************
 // *********************************************************************


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

	   // ** If Fade is enabled adjust PWM value every 1/2 sec. ***
	      if(pwm_fade)  // If fade demo is enabled
	        {
	         if(pwm_dir)  // fade UP
	           {
	    	    pwm_val+=10;
	    	    if(pwm_val > 300)  //if max fade up
	    	        pwm_dir=0;     // switch to fade down

	           }
	         else  // fade down
	           {
	    	    pwm_val-=10;
	    	    if(pwm_val < 20)  // if min fade down
	    	       pwm_dir=1;     // switch to fade up
	           }

	          ResetTIM3_CH1_PWM(pwm_val); //update pwm output
	         }

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
  USART_PutStr("\r\n **** TIMx Demo Menu **** \r\n");
  USART_PutStr(" 1. Reset TIM2 to 10us\r\n");
  USART_PutStr(" 2. Verify TIM2Counter\r\n");
  USART_PutStr(" 3. Show TIM2Counter\r\n");
  USART_PutStr(" 4. Option Four \r\n");
  USART_PutStr(" 5. Start PWM Demo \r\n");
  USART_PutStr(" 6. Set TIM3_CH1 PWM\r\n");
  USART_PutStr(" 7. Toggle PWM Fade Demo\r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;

  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n Reset TIM2 to 500KHz 1 (count/2MHZ)\r\n");
	   /* Time base configuration */
	   TIM_TimeBaseStructure.TIM_Period = 10-1; // 1MHZ to 100KHz
	   TIM_TimeBaseStructure.TIM_Prescaler = 72-1; // 72 MHz Clock to 1MHz
	   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
   break;
   case '2':
	   USART_PutStr("\r\n  Verify TIM2Counter: ");
	   TIM2Counter=0;
	   Delay_ms(1000);
	   utoa(TIM2Counter, (char *)msgBuf, 10);
	   USART_PutStr((char *)msgBuf);
	   USART_PutStr(" Counts in 1 Sec.\r\n");
   break;
   case '3':
	   USART_PutStr("\r\n  TIM2Counter: ");
	   utoa(TIM2Counter, (char *)msgBuf, 10);
	   USART_PutStr((char *)msgBuf);
	   USART_PutStr("\r\n Done...\r\n");
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
   case '5':
	   USART_PutStr("\r\n  Starting PWM Demo \r\n");
	   Setup_TIM3_PWMOutput();
   break;
   case '6':
	   USART_PutStr("\r\n  Input CH1 PWM Value [0-655]: ");
	   USART_GetStr((char *)msgBuf, 5);
	   pwm_val = atoi((char *)msgBuf);
	    /* PWM1 Mode configuration: Channel1 */
	   ResetTIM3_CH1_PWM(pwm_val);
       USART_PutStr("\n OK\r\n");
   break;
   case '7':  // toggle pwm fade demo flag
	   if(pwm_fade)
	     {
		  pwm_fade = 0;
		  USART_PutStr("\r\n PWM Fade OFF\r\n");
	     }
	   else
	     {
		  pwm_fade=1;
		  USART_PutStr("\r\n PWM Fade ON\r\n");
	     }
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
// ******* Setup_TIM3_PWMOutput()  ********************************
// TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
// The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
// clock at 24 MHz the Prescaler is computed as following:
// - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
// SystemCoreClock is set to 72 MHz for STM32F103C
//
// The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
//                                                 = 24 MHz / 666 = 36 KHz
// TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
// TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
// TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
// TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
// ***********************************************************************
void Setup_TIM3_PWMOutput(void)
{
 //GPIO_InitTypeDef GPIO_InitStruct;                *use global variable
 //TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  *use global variable
 //TIM_OCInitTypeDef  TIM_OCInitStructure;          *use global variable

 uint16_t CCR1_Val = 333;
 uint16_t CCR2_Val = 249;
 uint16_t CCR3_Val = 166;
 uint16_t CCR4_Val = 83;
 uint16_t PrescalerValue = 0;

 /* TIM3 clock enable */
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

 /* GPIOA and GPIOB clock enable */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                      RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

 // *********** TIM3 PWM GPIO SETUP *********************
 // PA6=TIM3_CH1, PA7=TIM3_CH2, PB0=TIM3_CH3, PB1=TIM3_CH4
 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

 GPIO_Init(GPIOA, &GPIO_InitStruct);

 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
 GPIO_Init(GPIOB, &GPIO_InitStruct);

 // Compute the prescaler value for 24MHZ
 PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;

 // Time base configuration
 TIM_TimeBaseStructure.TIM_Period = 665;               // 24MHz/666=36KHz
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

 // PWM1 Mode configuration: Channel1
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

 TIM_OC1Init(TIM3, &TIM_OCInitStructure);

 TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

 // PWM1 Mode configuration: Channel2
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

 TIM_OC2Init(TIM3, &TIM_OCInitStructure);

 TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

 // PWM1 Mode configuration: Channel3 */
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

 TIM_OC3Init(TIM3, &TIM_OCInitStructure);

 TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

 // PWM1 Mode configuration: Channel4 */
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

 TIM_OC4Init(TIM3, &TIM_OCInitStructure);

 TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

 TIM_ARRPreloadConfig(TIM3, ENABLE);

 /// TIM3 enable counter
 TIM_Cmd(TIM3, ENABLE);

}

// ***********************************************************************
// ResetTIM3_CH1_PWM(int pwm)
// ***********************************************************************
void ResetTIM3_CH1_PWM(int pwm)
{
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 TIM_OCInitStructure.TIM_Pulse = pwm;
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

 TIM_OC1Init(TIM3, &TIM_OCInitStructure);
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

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM2Counter++;

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  }
}



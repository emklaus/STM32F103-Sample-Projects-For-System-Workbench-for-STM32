// ***********************************************************************
// STM32F103_WAV
// derived from Template for USART Menu with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX, SysTick interrupts every 10us
// Play a sound file using PCM/PWM on pin PA6
// Here's how it works:
// 1. When TIM3 count reaches it's period count of 255+1 it generates an interrupt
// 2. The IRQ routine loads a sound file value into the TIM3 CH1 compare register
// 3. That will set the TIM3 CH1 pin (PA6) HIGH until TIM3 counter matches the
//    compare count value then it will go low.
//    (so the larger the sound byte value the higher the amplitude of the output)
// 4. Each loaded value is output 4 times (see sample_count) then the next
//    sound file byte is loaded and the process repeats until all bytes are played.
//
//    the sound files were recorded using a sample rate of 8.0kHz, 8 bits, mono
//    There's a Wav2c utility that creates the .h files from a .wav sound file.
//
// By Eric M. Klaus   2/2016
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "stdlib.h"  //for itoa(), atoi()
#include "helloComputer02.h"
#include "one.h"
#include "two.h"
#include "three.h"
#include "four.h"
#include "five.h"
#include "six.h"
#include "seven.h"
#include "eight.h"
#include "nine.h"
#include "noclick.h"

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);


// SysTick based delay routines
void SysTick_Handler(void);
void TIM3_IRQHandler(void);

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

// Timer3 PWM Demo Setup Routine
void playNumber(int num);
void Setup_TIM3_PWMOutput(void);

// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

static __IO uint16_t sample, pcm_length;
static __IO uint16_t sample_count;
static __IO const uint8_t *sound_data;
static __IO uint8_t playSound = 0;

GPIO_InitTypeDef GPIO_InitStruct;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

int led1_val=0;  // state of LED
int pwm_val=333, pwm_dir=0, pwm_fade=0;
char msgBuf[80]; // general purpose string buffer

int main(void)
{
 int k=0;
 // At this point startup has executed invoking SystemInit()
 // which selects the HSE clock and configures it to run at 74MHz.

 // ** Configures the SysTick event to fire every 125us	**
 SysTick_Config(SystemCoreClock / 8000);

 // Configure the GPIO pin for the LED (PC13)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTB Enable
 GPIO_InitStruct.GPIO_Pin = LED_PIN | GPIO_Pin_14 | GPIO_Pin_15; // Configure Led pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LED_PORT, &GPIO_InitStruct);

 SetupUSART();

 GPIO_ResetBits(LED_PORT, LED_PIN | GPIO_Pin_14 | GPIO_Pin_15);  //LED ON
 TimingDelay = 4000;                // Blink every 1/2 sec.

 showMenu();  // Display the user menu

 Setup_TIM3_PWMOutput();

 while(1)
   {
    if(TimingDelay==0)
	  {
	   TimingDelay = 4000;  //Reset the 1/2 sec. delay
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
  USART_PutStr(" 1. Play Sound\r\n");
  USART_PutStr(" 2. Play Number\r\n");
  USART_PutStr(" 3. Count\r\n");
  USART_PutStr(" 4. Blink LED\r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;
  switch(cmd)
  {
  case '1':
	   USART_PutStr("\r\n  Hello... \r\n");

	   pcm_length = hello_computer_length;
	   sound_data= hello_computer_data;

	   sample=0;
	   sample_count=1;
	   playSound=1;
	   TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	   while(sample<pcm_length);
	   USART_PutStr(" Done...\r\n");
  break;
  case '2':
	   x=10;
	   USART_PutStr("\r\n Enter the number to play:[1-9] (0=Exit)");
	    while(x)
	      {
	       USART_GetStr(msgBuf, 1);
	       x = msgBuf[0]-'0';
	       if((x<10)&&(x>0))
	         {
	          playNumber(x);
	         }
	      }
	   USART_PutStr(" Done...\r\n");
  break;
  case '3':
	    USART_PutStr("\r\n counting...");
	    x = msgBuf[0]-'0';
	    for(x=1; x<10; x++)
	      {
	       Delay_ms(500);
	       USART_PutChar('0'+x);
	       playNumber(x);
	      }

	   USART_PutStr(" Done...\r\n");
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

// ***********************************************************************
// playNumber(int num)
// ***********************************************************************
void playNumber(int num)
{

 pcm_length = pre_play_length;
 sound_data= pre_play_data;
 sample=0;
 sample_count=1;
 playSound=1;
 TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
 while(sample<pcm_length);

 switch(num)
 {
  case 1:
    pcm_length = one_length;
    sound_data= one_data;
  break;
  case 2:
    pcm_length = two_length;
    sound_data= two_data;
  break;
  case 3:
    pcm_length = three_length;
    sound_data= three_data;
  break;
  case 4:
    pcm_length = four_length;
    sound_data= four_data;
  break;
  case 5:
    pcm_length = five_length;
    sound_data= five_data;
  break;
  case 6:
    pcm_length = six_length;
    sound_data= six_data;
  break;
  case 7:
    pcm_length = seven_length;
    sound_data= seven_data;
  break;
  case 8:
    pcm_length = eight_length;
    sound_data= eight_data;
  break;
  case 9:
    pcm_length = nine_length;
    sound_data= nine_data;
  break;
  default:
	  pcm_length = hello_computer_length;
	  sound_data= hello_computer_data;
  break;
 }

  sample=0;
  sample_count=1;
  playSound=1;
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  while(sample<pcm_length);

  pcm_length = post_play_length;
  sound_data= post_play_data;
  sample=0;
  sample_count=1;
  playSound=1;
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  while(sample<pcm_length);
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
// ******* Setup_TIM3_PWMOutput()  ***************************************
// TIM3 Configuration: generate a PWM pulse proportional to signal amplitude.
// The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
// clock at 8 MHz the Prescaler is computed as following:
// - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
// SystemCoreClock is set to 72 MHz for STM32F103C
//
// The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
//                                                 = 8MHz/256 = 31.25KHz
// ***********************************************************************
void Setup_TIM3_PWMOutput(void)
{
 GPIO_InitTypeDef GPIO_InitStruct;
 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 TIM_OCInitTypeDef  TIM_OCInitStructure;

 uint16_t PrescalerValue = 0;

 /* TIM3 clock enable */
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

 /* GPIOA,GPIOB and GPIOC clock enable */
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

 // Compute the prescaler value for 8MHZ
 PrescalerValue = (uint16_t) (SystemCoreClock / 8000000) - 1;

 // Time base configuration
 TIM_TimeBaseStructure.TIM_Period = 255;               // 8MHz/256 = 31.25KHz
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

 // PWM1 Mode configuration: Channel1
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 TIM_OCInitStructure.TIM_Pulse = sound_data[0];
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

 TIM_OC1Init(TIM3, &TIM_OCInitStructure);

 TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

 // ******* Enable the TIM3 global Interrupt  **********
 NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);

 // TIM3 IT enable  ** For now disable the overflow IRQ
 TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);

 // TIM3 enable counter
 TIM_Cmd(TIM3, ENABLE);

}

// ***********************************************************************
// ******* SysTick based Delay functions  ********************************
// ***********************************************************************
// ***********************************************************************
//  Simple microsecond delay routine
// ***********************************************************************
void Delay_us(const uint32_t usec)
{
  TimingDelay = usec/125;
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

 usCounter+=125;
 if(++ctMs > 8)
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
void TIM3_IRQHandler(void)
{
 if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  // when the 256 count is reached
  {
	 if(playSound)
	   {
	    sample_count--;
	    if(sample_count == 0)
	      {
	       sample_count = 4;
	       TIM_SetCompare1(TIM3, sound_data[sample++]);
	       if(sample>pcm_length)
	         {
	          playSound=0;  // Turn off Timer0 Overflow Int.
	          TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
	         }
	      }
	   }

	 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);   // reset the IRQ flag
  }

}


// ***********************************************************************
// STM32F103_GPIO1  - Minimum GPIO support with a SysTick Timer (blink)
// Using SysTick interrupts every 10us
//
// Blinks the on-board LED connected to PC13
// Optionally connect a pushbutton to PC14 & gnd which will cause the LED
//    to flash quickly.
//
// *** Use this as a starting point for other projects
// By Eric M. Klaus
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define BUTTON_PIN GPIO_Pin_14
#define LED_PIN    GPIO_Pin_13
#define LED_PORT   GPIOC


// ***********************************************************************
//* Declare function prototypes
// ***********************************************************************

// SysTick based delay routines
void SysTick_Handler(void);

// Looping style delay routines (still controlled by SysTick)
//    NOTE: provided as examples, none used in this example
void Delay_us(const uint32_t usec);
void Delay_ms(const uint32_t msec);

// GPIO Demo functions
void flashLED(int count, int delay);

// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  // state of LED


// ***********************************************************************
// main()
// ***********************************************************************
int main(void)
{

 // At this point startup has executed invoking SystemInit()
 // which selects the HSE clock and configures it to run at 74MHz.

 // ** Configures the SysTick event to fire every 10us	**
 SysTick_Config(SystemCoreClock / 100000);

 // Configure the GPIO pin for the LED (PC13)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // Clock PORTC Enable

 GPIO_InitStruct.GPIO_Pin = LED_PIN;            // Configure Led pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;   // Set Output Push-Pull
 GPIO_Init(LED_PORT, &GPIO_InitStruct);

 // Configure the GPIO pin for the Button (PC14)
 GPIO_InitStruct.GPIO_Pin = BUTTON_PIN;     // Configure button pin
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // Set as INPUT with Pull-up
 GPIO_Init(LED_PORT, &GPIO_InitStruct);


 flashLED(20, 50);  //Startup blink fast for 2 Sec.

 GPIO_ResetBits(LED_PORT, LED_PIN);  //LED ON
 TimingDelay = 50000;                // Blink every 1/2 sec.


 while(1)
   {
    if(TimingDelay==0)
	  {
	   TimingDelay = 50000;  //Reset the 1/2 sec. delay
	   if(led1_val)
	     {
		  GPIO_SetBits(LED_PORT, LED_PIN); // LED1 OFF
		  led1_val=0;
	     }
	   else
	     {
		  GPIO_ResetBits(LED_PORT, LED_PIN); // LED1 ON
		  led1_val=1;
	     }

	    // Do something else that needs to happen every 1/2 second here.....

	    if(!GPIO_ReadInputDataBit(LED_PORT, BUTTON_PIN)) // Is button pushed?
	      {
	       flashLED(20, 25); //Very Fast blink 1/2 sec.
	      }
	  }

     // Do something useful here...

   }
}

// ***********************************************************************
// flashLED(count, delay)
// Blink the LED (count) times using the specified ms delay (delay).
// ***********************************************************************
void flashLED(int count, int delay)
{
  while(count--)
    {
     GPIO_ResetBits(LED_PORT, LED_PIN);  // LED1 ON
     Delay_ms(delay);
     GPIO_SetBits(LED_PORT, LED_PIN);    // LED1 OFF
     Delay_ms(delay);
    }
}

// ***********************************************************************
// ******* SysTick based Delay functions  ********************************
// ***********************************************************************
// ***********************************************************************
//  Simple microsecond delay routine (* 10us resolution)
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
//  SystemTick IRQ handler (invoked every 10us)
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

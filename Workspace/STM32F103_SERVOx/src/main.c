// ***********************************************************************
// STM32F103_Servo1   Timer4 counting at 1ms/count Timer3 used for PWM
//
// derived from Template for UART Menu with GPIO & SysTick support
// Using UART1 PA10=RX PA9=TX. SysTick interrupts every 10us
// Appends TIM2 support & IRQ Handler
//
// This example demonstrates how to support almost any number of Servos
// It is based on the arrays servoPins[], servoLoadVals[] and servoRunVals[]
// Here's how it works:
// 1. At the 20ms interrupt, copy servoLoadVals[] to servoRunVals[] and set
//   all servo signal pins HIGH, then find the LOWEST value in servoRunVals[]
//   and load that into TIM2 Compare1 value.
// 2. When the Compare1 count is reached (a servo pulse needs to end)
//    check all the servo value and end(set pin LOW) ANY servo pulse
//    that is less than or equal to the current Compare1 count.
//    At this time we will also set the servoRunVal[] element of any
//    expired servo to the 20ms count so it will be ignored by the lowest
//    value selection routine.
// 3. Finally search for the lowest servo value(if any) and load Compare1
//     with that value and exit waiting for that count to be reached(see step#2)
// 4. Once all servos have been processed the 20ms event will eventually be
//    reached and the process will repeat.(see step#1)
//
//  Your code will adjust the value in the servoLoadVal[] array to set the
//  servo positions. This prevents the user from changing a value while it's
//  being serviced by the IRQ routine.
//  NOTE: For this demonstration all servos must be connected to GPIO pins
//     on the SAME GPIO PORT it could easily be expanded to support other
//     ports by adding a servoPorts[] array similar to the servoPins[] array.
//
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

#define MAX_SERVOS   4
#define TIMER_VAL_20MS 20000
#define SERVO_PORT GPIOB
#define SERVO_PORT_PERIPH RCC_APB2Periph_GPIOB

// ** Typically, SERVO_MIN_COUNTS would be 1000 and SERVO_MAX_COUNTS would be 2000 **
// ** I found my servos (HS-322HD, HS425BB) responded differently - adjust accordingly ***
#define SERVO_MIN_COUNTS 650
#define SERVO_MAX_COUNTS 2450
#define SERVO_RANGE_COUNTS (SERVO_MAX_COUNTS - SERVO_MIN_COUNTS)
/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);

void loadServoRunValues(void);
int findLowestServoRunVal(void);
void resetAllExpiredServos(uint16_t cc1Val);
void setAllServos(void);
void configureAllServoPins(void);
void setupTIM2(void);

// SysTick based delay routines
void SysTick_Handler(void);

// Looping style delay routines (still controlled by SysTick)
void Delay_us(const uint32_t usec);
void Delay_ms(const uint32_t msec);

// USART routines ** specific to USART1
void SetupUSART(void);
void USART_PutChar(char c);
void USART_PutStr(char *str);
void USART_PutHexByte(char byte);
int USART_GetStr(char *buf, int len);


// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;
static __IO unsigned int TIM2Counter, TIM2CC1;  //for TIM2 demo only...

// servoPins[] hold the pinIDs for servos 0-4
static __IO uint16_t servoPins[MAX_SERVOS]={GPIO_Pin_12,GPIO_Pin_13,GPIO_Pin_14,GPIO_Pin_15};
// servoLoadVals[] hold the pulse width counts for servos 0-4
static __IO uint16_t servoLoadVals[MAX_SERVOS]={SERVO_MAX_COUNTS,1975,1500,SERVO_MIN_COUNTS};
// servoRunVals[] holds the pulse width value being processed.
static __IO uint16_t servoRunVals[MAX_SERVOS];

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  //state of LED
unsigned char msgBuf[80]; // General purpose buffer

int main(void)
{
 int k=0;
 TIM2Counter = 0;
 TIM2CC1=0;

 // At this point startup has executed invoking SystemInit()
 // which selects the HSE clock and configures it to run at 74MHz.

 // ** Configures the SysTick event to fire every 10us	**
 SysTick_Config(SystemCoreClock / 100000);

 // Configure the GPIO pin for the LED (PC13)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTC Enable
 GPIO_InitStruct.GPIO_Pin = LED_PIN; // Configure Led pin
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LED_PORT, &GPIO_InitStruct);

 SetupUSART(); // Configure USART for 9600 baud

 configureAllServoPins(); // Setup GPIO for all defined servo pins

 setupTIM2();             // Configure and start Timer2

 GPIO_SetBits(LED_PORT, LED_PIN);  //LED ON
 TimingDelay = 50000;              // Blink every 1/2 sec.

 USART_PutStr("\r\n Timer Test Menu\r\n");

 while(1)
   {
    if(TimingDelay==0) //Time to blink LED
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
     }

    // Is a byte available from UART?
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
      {
	   k = USART_ReceiveData(USART1);
	   USART_PutChar(k);
	   processMenuCmd(k);
	  }
   }
}


//***************************************************************************
//  showMenu()
//***************************************************************************
void showMenu(void)
{
  USART_PutStr("\r\n **** UART2 Menu **** \r\n");
  USART_PutStr(" 1. Option One\r\n");
  USART_PutStr(" 2. Verify TIM2Counter\r\n");
  USART_PutStr(" 3. Show TIM2Counter\r\n");
  USART_PutStr(" 4. Option Four \r\n");
  USART_PutStr(" 5. Set Servo Pos\r\n");
  USART_PutStr(" 6. Show Servo Positions\r\n");
  USART_PutStr(" 7. Set Servo 0 count value\r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

//***************************************************************************
// processMenuCmd(cmd)
//***************************************************************************
void processMenuCmd(char cmd)
{
  int x, pos;

  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n Option One\r\n");
	   //{add your code here}
   break;
   case '2': // This verifies the TIM2 frequency and 20ms timer
	   USART_PutStr("\r\n  Verify TIM2Counter: ");
	   TIM2Counter=0;
	   TIM2CC1=0;
	   Delay_ms(1000);
	   utoa(TIM2Counter, (char *)msgBuf, 10);
	   USART_PutStr((char *)msgBuf);
	   USART_PutStr(" TIM2CC1: ");
	   utoa(TIM2CC1, (char *)msgBuf, 10);
	   USART_PutStr((char *)msgBuf);
	   USART_PutStr(" Counts in 1 Sec.\r\n");
   break;
   case '3':  // This proves things are working....
	   USART_PutStr("\r\n  TIM2Counter: ");
	   utoa(TIM2Counter, (char *)msgBuf, 10);
	   USART_PutStr((char *)msgBuf);
	   USART_PutStr(" TIM2CC1: ");
	   utoa(TIM2CC1, (char *)msgBuf, 10);
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

   case '5':  // Set Servo Position
	   USART_PutStr("\r\n  Input Servo #[0-3]: ");
	   USART_GetStr((char *)msgBuf, 2);
	   x = atoi((char *)msgBuf);
	   USART_PutStr("\r\n  Input Servo Pos [0-180]: ");
	   USART_GetStr((char *)msgBuf, 3);
	   pos = atoi((char *)msgBuf);
	   if(x < MAX_SERVOS)
	     {
		   servoLoadVals[x]=(int)(float)pos * ((float)SERVO_RANGE_COUNTS/180)+SERVO_MIN_COUNTS;
	       USART_PutStr("\n OK\r\n");
	     }
   break;

   case '6':  // Show Servo Positions
	   USART_PutStr("\r\n Servo Positions: ");
	   for(x=0; x < MAX_SERVOS; x++)
	     {
	 	  pos = (int)((float)servoLoadVals[x] - SERVO_MIN_COUNTS)/((float)SERVO_RANGE_COUNTS/180);

	 	  itoa(pos, (char *)msgBuf, 10);
	 	  USART_PutStr((char *)msgBuf);
	 	  USART_PutStr(" ");
	     }

	   USART_PutStr("\r\n");
   break;

   case '7': // Set Servo 0 Counts
	         // (use this to test range values for SERVO_MIN_COUNTS & SERVO_MAX_COUNTS)
	   USART_PutStr("\r\n  Input Servo 0  Count Val: ");
	   USART_GetStr((char *)msgBuf, 4);
	   pos = atoi((char *)msgBuf);
       servoLoadVals[0]=pos;
	   USART_PutStr("\r\n Set Val To :");
   	   itoa(pos, (char *)msgBuf, 10);
 	   USART_PutStr((char *)msgBuf);
 	   USART_PutStr("\r\n");
   break;

   case 'M':
   case 'm':
	   showMenu();
   break;

  }
}



//***************************************************************************
//****** Servo Routines *****************************************************
//***************************************************************************

//***************************************************************************
// loadServoRunValues() copy all values from servoLoadVals to servoRunVals
//***************************************************************************
void loadServoRunValues(void)
{
  int x;
  for(x=0; x < MAX_SERVOS; x++)
    {
	  servoRunVals[x]= servoLoadVals[x];
    }
}

//***************************************************************************
// findLowestServoRunVal()
// find the lowest value in  servoRunVals[} and return it's index
// Returns -1 if no value < TIMER_VAL_20MS is found
//***************************************************************************
int findLowestServoRunVal(void)
{
 int x, servoIndex = -1;
 uint16_t lowVal = TIMER_VAL_20MS;

 for(x=0; x < MAX_SERVOS; x++)
    {
	  if(servoRunVals[x] < lowVal)
	    {
		  lowVal = servoRunVals[x];
		  servoIndex=x;
	    }
    }
 return servoIndex;
}

//***************************************************************************
// resetAllExpiredServos(cc1Val)    (end the pulse)
// find any value in servoRunVals[] <= cc1Val (TM2 Compare1 count)
// and set the corresponding GPIO pin LOW also replace the value with
// the 20ms value.
//***************************************************************************
void resetAllExpiredServos(uint16_t cc1Val)
{
 int x;

 for(x=0; x < MAX_SERVOS; x++)
    {
	  if(servoRunVals[x] <= cc1Val)
		{
		 servoRunVals[x] = TIMER_VAL_20MS;         // Set RunVal = TIMER_VAL_20MS
		 GPIO_ResetBits(SERVO_PORT, servoPins[x]); // Set output pin LOW
	    }
    }
}

//***************************************************************************
// setAllServos()  Set the GPIO pin for each servo HIGH (start a pulse)
//***************************************************************************
void setAllServos(void)
{
 int x;

 for(x=0; x < MAX_SERVOS; x++)
    {
     servoRunVals[x] = servoLoadVals[x];      // Copy LoadVal to RunVal
	 GPIO_SetBits(SERVO_PORT, servoPins[x]);  // Set Output pin HIGH
    }
}

//***************************************************************************
// configureAllServoPins()   setup the GPIO pins for each pin in servoPins[].
//***************************************************************************
void configureAllServoPins(void)
{
 int x;
 GPIO_InitTypeDef GPIO_InitStruct;

 RCC_APB2PeriphClockCmd(SERVO_PORT_PERIPH, ENABLE);// Clock SERVO_PORT Enable

 for(x=0; x < MAX_SERVOS; x++)
    {
	 GPIO_InitStruct.GPIO_Pin = servoPins[x]; // Configure ServoPin[x]
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
	 GPIO_Init(SERVO_PORT, &GPIO_InitStruct);

	 GPIO_ResetBits(SERVO_PORT, servoPins[x]); // Set output pin LOW
    }
}

// *********************************************************************
// *********** TIMER2 ROUTINES *****************************************
// *********************************************************************
//***************************************************************************
// setupTIM2(void)
// Configure and start Timer2 counting @ 1us/count
// It will generate a TIM_IT_Update interrupt at 20ms
// and a TIM_IT_CC1 interrupt each time the Compare1 count is reached
//***************************************************************************
void setupTIM2(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // ******* Enable the TIM2 gloabal Interrupt  **********
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


  // Timer2  Time base configuration - System clock is 72MHz
  TIM_TimeBaseStructure.TIM_Period = TIMER_VAL_20MS - 1; // TIM_IT_Update will occur every 20ms
  TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 72 MHz Clock down to 1 MHz (adjust per your clock)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // Output Compare Mode configuration: Channel1
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_SetCompare1(TIM2, TIMER_VAL_20MS + 1); // First time compare value will not be hit.
                                             // When the 20ms time is reached this value will
                                             // be loaded correctly by the ISR

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
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
	// USART_PutChar(char *c)
	// NOTE: this is a simple example and will block other processing
	//       until the byte is sent.
	// ***********************************************************************
	void USART_PutChar(char c)
	{
		//write a character to the USART
		USART_SendData(USART1, (uint8_t) c);

		// Loop until the end of transmission
		  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		  {}
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
	void USART_PutHexByte(char byte)
	{
	 char n = (byte >> 4) & 0x0F;
	 /* Write high order digit */
	 if (n < 10)
	 	USART_PutChar(n + '0');
	 else
		USART_PutChar(n - 10 + 'A');

	 /* Write low order digit */
	 n = (byte & 0x0F);
	 if (n < 10)
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

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
  int x;
  uint16_t cc1Val;

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  // when the 20ms count is reached
  {
	setAllServos();                  // Set all servo pins HIGH (start of the pulse)
	loadServoRunValues();            // copy value from servoLoadVals[] to servoRunVals[]
	x = findLowestServoRunVal();     // find servo# with lowest value
	cc1Val = servoRunVals[x];        // get the value found
	TIM_SetCompare1(TIM2, cc1Val);   // load it to the TIM2 compare register

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   // reset the IRQ flag
    TIM2Counter++;                                // just a demo counter..
  }

  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // when the Compare1 count is reached
  {
    cc1Val = TIM_GetCapture1(TIM2);  // read the compare value we just hit
    resetAllExpiredServos(cc1Val);   // reset any servo with time <= cc1Val
    x = findLowestServoRunVal();     // find the lowest value that has not yet been reset
    if(x != -1)                      //  -1 is returned when no more servos need service
      {
    	cc1Val = servoRunVals[x];    // get the count value for the lowest found

        if(cc1Val < TIMER_VAL_20MS)        // if it's valid
          TIM_SetCompare1(TIM2, cc1Val);   // set that value to Compare1
      }

    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);   // Clear the IRQ flag
    TIM2CC1++;                                 // just a demo counter
  }

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


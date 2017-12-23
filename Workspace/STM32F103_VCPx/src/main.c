//************************************************************************
// STM32F103_VCPx
// Template for use to implement a basic USB VCP (Virtual COM Port)
// sends a message to the VCP and echos bytes received + hex value to terminal.
// Designed for the STM32F103C8T6 Minimum System Development Board (ebay)
// Download the VCP driver from http://www.st.com/web/en/catalog/tools/PF257938
// NOTES:based on example file STSW-STM32121 from www.st.com
//       references to USART were removed from those files.
//       look for modifications in hw_config.*,usb_endp.c,stm32_it.*,usb_prop.c
//
// ** project changes needed to be made to include the USB_FS_Driver folder **
//
//************************************************************************

#include "stm32f10x.h"
#include "usb_init.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "misc.h"

#define LED_PIN  GPIO_Pin_13
#define LED2_PIN GPIO_Pin_14
#define LED_PORT GPIOC


// Global buffers and pointers (defined in hw_config.c )
extern uint8_t  VCP_Tx_Buffer [VCP_TX_DATA_SIZE];
extern uint8_t  VCP_Rx_Buffer [VCP_RX_DATA_SIZE];
extern uint32_t VCP_Tx_ptr_in;
extern uint32_t VCP_Tx_ptr_out;
extern uint32_t VCP_Tx_length;
extern uint32_t VCP_Rx_ptr_in;
extern uint32_t VCP_Rx_ptr_out;
extern uint32_t VCP_Rx_length;
// Global timing counters
extern uint32_t TimingDelay;

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  //state of LED

// Private function prototypes
void showMenu(void);
void processMenuCmd(char cmd);
// VCP utility functions
void VCP_init(void);
char VCP_GetChar(void);
void VCP_PutChar(char c);
void VCP_PutStr(char *str);
void VCP_PutHexByte(unsigned char byte);

// SysTick based delay routines
// Looping style delay routines (still controlled by SysTick)
//    NOTE: provided as examples, none used in this example
void Delay_us(const uint32_t usec);
void Delay_ms(const uint32_t msec);



//************************************************************************
// main()   Program entry point.
//************************************************************************
int main(void)
{
  char k;
  Set_System();
  //SetupClock();
  SystemCoreClockUpdate(); // MUST be called whenever the core clock is changed
  // ** Configures the SysTick event to fire every 10us	**
  SysTick_Config(SystemCoreClock / 100000);

  // Configure the GPIO pin for the LED (PC13)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTB Enable
  GPIO_InitStruct.GPIO_Pin = LED_PIN | LED2_PIN; // Configure Led pins
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
  GPIO_Init(LED_PORT, &GPIO_InitStruct);

  VCP_init();  // Init. the Virtual COM Port

  GPIO_ResetBits(LED_PORT, LED2_PIN);  //RED LED OFF
  GPIO_SetBits(LED_PORT, LED_PIN);  //LED ON
  TimingDelay = 50000;              // Blink every 1/2 sec.

  VCP_PutStr("\r\n Hello From VCP...\r\n");
  
  while (1)
  {
   if(VCP_Rx_length > 0)
   {
	 k = VCP_GetChar();
	 VCP_PutChar(k);
	 processMenuCmd(k);
   }

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
     }

  }
}


void showMenu(void)
{
  VCP_PutStr("\r\n **** UART2 Menu **** \r\n");
  VCP_PutStr(" 1. Option One \r\n");
  VCP_PutStr(" 2. Option Two \r\n");
  VCP_PutStr(" 3. Option Three \r\n");
  VCP_PutStr(" 4. Option Four \r\n");
  VCP_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;
  switch(cmd)
  {
   case '1':
	   VCP_PutStr("\r\n  You pressed ONE \r\n");
	   GPIO_SetBits(LED_PORT, LED2_PIN);  //RED LED ON
   break;
   case '2':
	   VCP_PutStr("\r\n  You pressed TWO !! \r\n");
	   GPIO_ResetBits(LED_PORT, LED2_PIN);  //RED LED OFF
   break;
   case '3':
	   VCP_PutStr("\r\n  You Selected Option Three !!! \r\n");
   break;
   case '4':
	   VCP_PutStr("\r\n  Blink LED...  \r\n");
	   for(x=1; x<20; x++)
	     {
		   GPIO_SetBits(LED_PORT, LED_PIN); // LED1 ON
		   Delay_ms(100);
		   GPIO_ResetBits(LED_PORT, LED_PIN); // LED1 OFF
		   Delay_ms(100);
	     }
	   VCP_PutStr(" Done...\r\n");
   break;
   case 'M':
   case 'm':
	   showMenu();
   break;

  }

}

// ***********************************************************************
// VCP_init()
// ***********************************************************************
void VCP_init(void)
{
 VCP_Tx_ptr_in = 0;
 VCP_Tx_ptr_out = 0;
 VCP_Tx_length = 0;
 VCP_Rx_ptr_in = 0;
 VCP_Rx_ptr_out = 0;
 VCP_Rx_length = 0;

 Set_USBClock();
 USB_Interrupts_Config();
 USB_Init();
}

// ***********************************************************************
// VCP_GetChar()
// ***********************************************************************
char VCP_GetChar(void)
{
  char outChr;
  outChr = VCP_Rx_Buffer[VCP_Rx_ptr_out];
  VCP_Rx_ptr_out++;

  if(VCP_Rx_ptr_out == VCP_RX_DATA_SIZE)
    {
	  VCP_Rx_ptr_out = 0;
	  VCP_Rx_length = 0;
    }
  else
  {
   VCP_Rx_length = VCP_Rx_ptr_in - VCP_Rx_ptr_out;
  }

 return outChr;
}

// ***********************************************************************
// VCP_PutChar(c)
// ***********************************************************************
void VCP_PutChar(char c)
{
  VCP_Tx_Buffer[VCP_Tx_ptr_in] = c;

  VCP_Tx_ptr_in++;

  if(VCP_Tx_ptr_in == VCP_TX_DATA_SIZE)
    {
	  VCP_Tx_ptr_in = 0;
	  VCP_Tx_length = 0;
    }
  else
  {
   VCP_Tx_length = VCP_Tx_ptr_in - VCP_Tx_ptr_out;
  }

}

// ***********************************************************************
// VCP_PutStr(char *str)
// ***********************************************************************
void VCP_PutStr(char *str)
{
  while(*str)
  {
    VCP_PutChar(*str);
	str++;
  }
}

// ***********************************************************************
// VCP_PutHexByte(char byte)
// ***********************************************************************
void VCP_PutHexByte(unsigned char byte)
{
 char n = (byte >> 4) & 0x0F;
 /* Write high order digit */
 if (n < 10)
 	VCP_PutChar(n + '0');
 else
	VCP_PutChar(n - 10 + 'A');

 /* Write low order digit */
 n = (byte & 0x0F);
 if (n < 10)
 	VCP_PutChar(n + '0');
 else
	VCP_PutChar(n - 10 + 'A');
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
// ***********************************************************************

//*****END OF FILE****

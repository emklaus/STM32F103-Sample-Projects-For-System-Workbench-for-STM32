// ***************************************************************
// LCD5110.c
// General routines for controlling the NOKIA 5110 48x84 LCD
// ***************************************************************

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "LCD5110.h"
#include "Font.h"
//void Delay_us(const uint32_t usec);

//******************************************************************************
// Used by delay functions using the core's cycle counter in the trace unit.
// EnableCoreTiming() and CoreTimingDelay()
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC;

// ***************************************************************
//  LcdInitialize()  Setup the LCD
// ***************************************************************
void LcdInitialize(void)
{
 GPIO_InitTypeDef GPIO_InitStruct;

 // Configure the GPIO pin for the LCD
 RCC_APB2PeriphClockCmd(LCD_PERIPH | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
 //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //Disable JTAG so we can use PB4
 GPIO_StructInit(&GPIO_InitStruct);
 GPIO_InitStruct.GPIO_Pin = PIN_SCLK | PIN_RESET | PIN_SDIN | PIN_SCE | PIN_DC | PIN_BKLT;
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LCD_PORT, &GPIO_InitStruct);
 GPIO_ResetBits(LCD_PORT, PIN_BKLT | PIN_SDIN | PIN_DC);
 GPIO_SetBits(LCD_PORT, PIN_SCLK | PIN_RESET | PIN_SCE);    // Reset = HIGH

 us_delay(200000);
 GPIO_SetBits(LCD_PORT, PIN_BKLT);    // Backlight OFF
 us_delay(200000);
 
 GPIO_ResetBits(LCD_PORT, PIN_RESET);  // Reset = LOW
 us_delay(20000);
 GPIO_SetBits(LCD_PORT, PIN_RESET);    // Reset = HIGH
 us_delay(100);
 
  //Note: sometimes Contrast, Temp Coefficent and bias values need to be adjusted **
  LcdWrite(LCD_C, 0x21 );  // LCD Extended Commands.
  LcdWrite(LCD_C, 0xC1 );  // Set LCD Vop (Contrast). //0xB1
  LcdWrite(LCD_C, 0x06 );  // Set Temp coefficent. //0x04
  LcdWrite(LCD_C, 0x13 );  // LCD bias mode 1:48. //0x13
  LcdWrite(LCD_C, 0x20 );  // LCD Basic Commands
  LcdWrite(LCD_C, 0x0C );  // LCD in normal mode.
  LCD_GotoXY(0, 0);
  GPIO_ResetBits(LCD_PORT, PIN_BKLT);    // Backlight ON
}

// ***************************************************************
//  LcdCharacter(character)
//  Display character at the current output position
// ***************************************************************
void LcdCharacter(char character)
{
  unsigned char data;
  int offset;
  LcdWrite(LCD_D, 0x00);
  
  offset = (int)(character - 0x20)*5;
  
  for (int index = 0; index < 5; index++)
  {
   // ** access data from font array ** 
   data = charSet[offset + index];
   
   LcdWrite(LCD_D, data);
  }
  LcdWrite(LCD_D, 0x00);
}

// ***************************************************************
//  LcdCharacterX2(character)  10x13
//  Display character at the specified output position
// ***************************************************************
void LcdCharacterX2(char character, char row, char col)
{
  unsigned char data, upper, lower;
  int offset, x;
  
  offset = (int)(character - 0x20)*5;
  
  for (int index = 0; index < 5; index++)
  {
   // ** access data from font array ** 
   data = charSet[offset + index];
   upper=0;
   lower=0;
   for(x=0; x<4; x++)
      {
       upper = upper >> 2;   
       lower = lower >> 2;
        
       if(data & 0x01)
          upper |= 0xC0;  

       if(data & 0x10)
          lower |= 0xC0;  
          
       data = data >> 1;
      }

   LCD_GotoXY(col, row);  
   LcdWrite(LCD_D, upper);
   LcdWrite(LCD_D, upper);
   LCD_GotoXY(col, row+1);
   LcdWrite(LCD_D, lower);
   LcdWrite(LCD_D, lower);
   col+=2;
  }
  
  LCD_GotoXY(col, row);
  LcdWrite(LCD_D, 0x00);     //Blank Spacer between chars        
  LCD_GotoXY(col, row+1);
  LcdWrite(LCD_D, 0x00);     //Blank Spacer between chars        
}

// ***************************************************************
//  LcdStrinfX2(str, row, col)
//  Display character at the specified output position
// ***************************************************************
void LcdStringX2(char *str, char row, char col)
{
  while (*str)
  {
    LcdCharacterX2(*str, row, col);
    str++;
    col+=11;
  }
}

// ***************************************************************
//  LcdClear()
//  Clear the LCD Display
// ***************************************************************
void LcdClear(void)
{
  for (int index = 0; index < LCD_X * LCD_Y / 8; index++)
  {
    LcdWrite(LCD_D, 0x00);
  }
}

// ***************************************************************
//  LcdString(characters)
//  Display the string characters at the current output position
// ***************************************************************
void LcdString(char *characters)
{
  while (*characters)
  {
    LcdCharacter(*characters++);
  }
}

// ***************************************************************
//  LcdWrite(dc, data)
//  Write 1 byte (data or command) to the LCD
// ***************************************************************
void LcdWrite(uint8_t dc, uint8_t data)
{
  int x;

  // Set DC pin for Data or Command   
  if(dc == LCD_D)
    GPIO_SetBits(LCD_PORT, PIN_DC);    // Data
  else  
    GPIO_ResetBits(LCD_PORT, PIN_DC);  // Command
  

  GPIO_ResetBits(LCD_PORT, PIN_SCE);  // CE = LOW
  
  us_delay(10);

  //shiftOut 8 bits from data (MSB -> LSB)
  for(x=0; x < 8; x++)
     {
      GPIO_ResetBits(LCD_PORT, PIN_SCLK);  // Set CLK LOW
      us_delay(10);

      if(data & 0x80)  // Set DIN pin to match MSB
        GPIO_SetBits(LCD_PORT, PIN_SDIN);    
      else  
        GPIO_ResetBits(LCD_PORT, PIN_SDIN);

      GPIO_SetBits(LCD_PORT, PIN_SCLK);  // Set CLK HIGH
      us_delay(10);
      
      data = data<<1;  
     }
     
  GPIO_SetBits(LCD_PORT, PIN_SCE);    // CE = HIGH
  
}

// ***************************************************************
//  LcdBmp(*my_array)
//  Display the bitmap array
// ***************************************************************
void LcdBmp(const uint8_t *my_array)
{
  unsigned char data;
  for (int index = 0; index < 504; index++)
  {
    data = my_array[index];
    LcdWrite(LCD_D, data);
  }
}

// *************************************************************************
//  LCD_gotoXY
//  Sets cursor location to xy location corresponding to basic font size.
//  x - range: 0 to 84  y -> range: 0 to 5
// *************************************************************************
void LCD_GotoXY(uint8_t x, uint8_t y )
{
 //Send command to display with XY values we want to draw at
  LcdWrite(LCD_C, 0x80 | x); // Col
  LcdWrite(LCD_C, 0x40 | y); // Row
}

// *************************************************************************
//  Draw a Horizontal bar at the specified position for the specified length
// *************************************************************************
void LcdHBar(uint8_t row, uint8_t start, uint8_t len, uint8_t bClear)
{
 char p=0;
 unsigned char d=0x7E;
 
 if(bClear)
   d=0;
   
 LCD_GotoXY(start, row);
 while(p < len)
   {
    LcdWrite(LCD_D, d); 
    p++;
   }
}

//**** Looping delay functions ********************
//******************************************************************************
// us_delay(uint32_t usec)   Delay usec  microseconds
//******************************************************************************
void us_delay(uint32_t usec)
{
 EnableCoreTiming();
 CoreTimingDelay(usec * 4);
 //Delay_us(usec);
}

//******************************************************************************
// EnableCoreTiming()
// Enable the core's cycle counter in the trace unit.
// From http://forums.arm.com/index.php?showtopic=13949
// DWT_CYCCNT = 0xE0001004   address of the DWT Cycle Count register
// DWT_CONTROL = 0xE0001000  address of the DTW register
// SCB_DEMCR   = 0xE000EDFC  address of the DEMCR register
//******************************************************************************
void EnableCoreTiming(void)
{
 static int enabled = 0;
 if(!enabled)
   {
     *SCB_DEMCR = *SCB_DEMCR | 0x01000000;//Set TRCENA bit of DEMCR ( Debug Exception and Monitor Control Register)
     *DWT_CYCCNT = 0; // reset the counter DWT (debug watchdog timer?)
     *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
     enabled = 1;
   }
}

//******************************************************************************
// CoreTimingDelay(unsigned int tick)
// Approximately  0.25 us/tick
// looping delay using the core's cycle counter in the trace unit.
//******************************************************************************
void CoreTimingDelay(unsigned int tick)
{
 unsigned int start, current;
 start = *DWT_CYCCNT;
 do
   {
    current = *DWT_CYCCNT;
   }while((current - start) < tick);
}

//******************************************************************************


/*
 *	Basic LCD interface example
 *	This code will interface to a standard LCD controller
 *	like the Hitachi HD44780. It uses it in 4 bit mode, with
 *	the hardware connected as follows (the standard 14 pin 
 *	LCD connector is used):
 *	
 *	PORTA bits 0-3 are connected to the LCD data bits 4-7 (high nibble)
 *	PORTA bit 4 is connected to the LCD RS input (register select)
  *	PORTA bit 5 is connected to the LCD RW (Read(1)/Write(0))
 *	PORTA bit 6 is connected to the LCD EN bit (enable)
 *	
 *	To use these routines, set up the port I/O  then
 *	call lcd_init(), then other routines as required.
 *	
 */
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include	"lcd.h"

extern void Delay_us(const uint32_t usec);


// ***********************************************************************
// Toggle the EN line Low to High then Low
// ***********************************************************************
void toggle_en(void)
{
 GPIO_SetBits(LCD_CTL_PORT, LCD_EN);   // Set EN Bit HIGH
 Delay_us(10);                         // short delay
 GPIO_ResetBits(LCD_CTL_PORT, LCD_EN); // Set EN Bit LOW
}

// ***********************************************************************
// write a byte to the LCD in 4 bit mode 
// NOTE: in this case both the 4 Data Bits and the 
//       3 control signals are on PORTA so we need to 
//       preserve the state of the high 4 bits (control) 
//       while sending data to the low 4 bits
// ***********************************************************************
void lcd_write(unsigned char c)
{
  uint16_t dataVal;
  Delay_us(40);    // Internally each read/write take 40us so wait
                    // to be sure the previous operation is complete.  

  dataVal = GPIO_ReadOutputData(LCD_DATA_PORT); //get the initial state of GPIOA
  dataVal &= 0xFFF0;
  dataVal |= ( c >> 4 );     // Output low 4 Data bits
  GPIO_Write(LCD_DATA_PORT, dataVal);
  toggle_en();                // clock EN pin

  dataVal &= 0xFFF0;         // Clear the low 4 bits
  dataVal |= ( c & 0x0F );   // set high 4 Data bits
  GPIO_Write(LCD_DATA_PORT, dataVal);
  toggle_en();                // clock EN pin
}

// ***********************************************************************
// read a byte from the LCD in 4 bit mode
// NOTE: in this case both the 4 Data Bits and the
//       3 control signals are on PORTA so we need to
//       preserve the state of the high 4 bits (control)
//       while reading data from the low 4 bits
// ***********************************************************************
uint8_t lcd_read(char rs)
{
  uint8_t d=0;
  uint16_t dataVal;
  GPIO_InitTypeDef GPIO_InitStruct;

  //All LCD Data Pins on GPIOA = INPUT
  // Configure the GPIO pin for the LCD Data (PA0-PA3)
  GPIO_InitStruct.GPIO_Pin = LCD_DATA_PINS; // Configure LCD Data pins
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;// Set Input PullUP
  GPIO_Init(LCD_DATA_PORT, &GPIO_InitStruct);

  if(rs)
	GPIO_SetBits(LCD_CTL_PORT, LCD_RS);    // Set LCD_RS=1 - Read Data
  else
    GPIO_ResetBits(LCD_CTL_PORT, LCD_RS); // Set LCD_RS=0 - Read Busy Flag

  GPIO_SetBits(LCD_CTL_PORT, LCD_RW); // Set LCD_RW=1 - Read/Write=Read

  GPIO_SetBits(LCD_CTL_PORT, LCD_EN);   // Set EN Bit HIGH
  Delay_us(10);                         // short delay
  dataVal = GPIO_ReadInputData(LCD_DATA_PORT);
  d = (uint8_t)(dataVal << 4) & 0x00F0; // Read High 4 bits.
  GPIO_ResetBits(LCD_CTL_PORT, LCD_EN); // Set EN Bit LOW

  Delay_us(10);                         // short delay

  GPIO_SetBits(LCD_CTL_PORT, LCD_EN);   // Set EN Bit HIGH
  Delay_us(10);                         // short delay
  dataVal = GPIO_ReadInputData(LCD_DATA_PORT);
  d |= (uint8_t)(dataVal & 0x000F);      // Read Low 4 bits.

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Data Pins back to OUTPUTs
  GPIO_Init(LCD_DATA_PORT, &GPIO_InitStruct);

  GPIO_ResetBits(LCD_CTL_PORT, LCD_EN | LCD_RW | LCD_RS); // Set EN,RW,RS Bits LOW

  return(d);     // return data read (bit 7 = Busy Flag, bits 6:0=address)
}

// ***********************************************************************
// lcd_busy()
// Return non-zero if busy
// ***********************************************************************
uint8_t lcd_busy(void)
{
 uint8_t d;
 d = lcd_read(0);     // Read the busy flag
 return(d & 0x80);    // mask and return it
}


// ***********************************************************************
// 	Clear and home the LCD
// ***********************************************************************
void lcd_clear(void)
{
  while(lcd_busy());    // Wait till not busy
  
  GPIO_ResetBits(LCD_CTL_PORT, LCD_RS); // Set LCD_RS = 0;
  lcd_write(0x1);            // send Clear command
  Delay_us(2000);            // short delay
}

// ***********************************************************************
// write a string of chars to the LCD 
// ***********************************************************************
void lcd_puts(const char * s)
{
 while(*s)
	 lcd_putch(*s++);   // Send each byte in string till NULL encountered
}

// ***********************************************************************
// write one character to the LCD 
// ***********************************************************************
void lcd_putch(char c)
{
 while(lcd_busy());    // Wait till not busy
 
 GPIO_SetBits(LCD_CTL_PORT, LCD_RS); // Set LCD_RS = 1;
 lcd_write( c );         // Send byte
}


// ***********************************************************************
// Go to the specified position
// ***********************************************************************
void lcd_goto(unsigned char pos)
{
 while(lcd_busy());    // Wait till not busy
 
 GPIO_ResetBits(LCD_CTL_PORT, LCD_RS); // Set LCD_RS = 0;
 lcd_write(0x80+pos);       // Set DDRAM address command
}
	
// ***********************************************************************
// Initialize the LCD - put into 4 bit mode
// ***********************************************************************
void lcd_init()
{
 char init_value;
 uint16_t dataVal;
 GPIO_InitTypeDef GPIO_InitStruct;

 init_value = 0x3;

 //All LCD Pins on GPIOA = OUTPUT
 // Configure the GPIO pin for the LCD Data (PA0-PA3)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// Clock PORTA Enable
 GPIO_InitStruct.GPIO_Pin = LCD_DATA_PINS; // Configure LCD Data pins
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LCD_DATA_PORT, &GPIO_InitStruct);

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// Clock PORTA Enable
 GPIO_InitStruct.GPIO_Pin = LCD_RS | LCD_RW | LCD_EN; // Configure LCD Ctrl pins
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
 GPIO_Init(LCD_CTL_PORT, &GPIO_InitStruct);

 GPIO_ResetBits(LCD_CTL_PORT, LCD_RS | LCD_RW | LCD_EN);
	
 Delay_us(15000);	// wait 15mSec after power applied,
 dataVal = GPIO_ReadOutputData(LCD_DATA_PORT);
 dataVal &= 0xFFF0;
 dataVal |= init_value;
 GPIO_Write(LCD_DATA_PORT, dataVal);

 toggle_en(); 
 Delay_us(5000);
 toggle_en(); 
 Delay_us(200);
 toggle_en(); 
 Delay_us(200);

 dataVal = GPIO_ReadOutputData(LCD_DATA_PORT);
 dataVal &= 0xFFF0;
 dataVal |= 2;  // Output Four bit mode command
 GPIO_Write(LCD_DATA_PORT, dataVal);

 toggle_en();       // clock EN pin

 lcd_write(0x28); // Set interface length
 lcd_write(0x0C); // Display On, Cursor Off, Cursor Blink Off
 lcd_clear();	    // Clear screen
 lcd_write(0x6);  // Set entry Mode
}

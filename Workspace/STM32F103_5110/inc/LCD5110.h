// ***************************************************************
// LCD5110.h
// General routine for controlling the NOKIA 5110 48x84 LCD
// 
// PA0=Backlight (violet)     PA1=DC (orange)    PA2=CE (Blue)
// PA3=DIN (Green)            PA4=RST (Brown)    PA5=CLK (Yellow)
// ***************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#define PIN_SCLK  GPIO_Pin_5
#define PIN_RESET GPIO_Pin_4
#define PIN_SDIN  GPIO_Pin_3
#define PIN_SCE   GPIO_Pin_2
#define PIN_DC    GPIO_Pin_1
#define PIN_BKLT  GPIO_Pin_0
#define LCD_PORT  GPIOA
#define LCD_PERIPH  RCC_APB2Periph_GPIOA

#define LCD_C     0    //LCD Command
#define LCD_D     1    //LCD Data

#define LCD_X     84   // Dot Cols
#define LCD_Y     48   // Dot rows

void LcdInitialize(void);      // Initialize   the LCD
void LcdCharacter(char character);  // Display character at the current output position
void LcdCharacterX2(char character, char row, char col); 
void LcdStringX2(char *str, char row, char col);
void LcdClear(void);
void LcdString(char *str);
void LcdWrite(uint8_t dc, uint8_t data);
void LcdBmp(const uint8_t *my_array);
void LCD_GotoXY(uint8_t x, uint8_t y );
void LcdHBar(uint8_t row, uint8_t start, uint8_t len, uint8_t bClear);
void us_delay(uint32_t usec);
void EnableCoreTiming(void);
void CoreTimingDelay(unsigned int tick);


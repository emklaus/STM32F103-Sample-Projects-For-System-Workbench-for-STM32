// ***********************************************************************
// STM32F103_24C02  - derived from UART Menu with GPIO & SysTick support
// Using UART1 PA10=RX PA9=TX, SysTick interrupts every 10us
// Adding I2C Peripheral for accessing ST 24C02 EEPROM 2k (256x8)
//
// Circuit       24C02  1,8 (E0, VCC)                    - 3.3V
// Connections:  24C02  2,3,4,7 (E1,E2,GND, Mode/WC)     - GND
//               24C02  5 (SDA)  *add 10k pullup on SDA  - PB9 (SDA1)
//               24C02  6 (SCL)  *add 10k pullup on SCL  - PB8 (SCL1)
//
// By Eric M. Klaus
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

#define I2C_EE             I2C1
#define I2C_EE_CLK         RCC_APB1Periph_I2C1
#define I2C_EE_GPIO        GPIOB
#define I2C_EE_GPIO_CLK    RCC_APB2Periph_GPIOB
#define I2C_EE_SCL         GPIO_Pin_8
#define I2C_EE_SDA         GPIO_Pin_9

// ** demo used a ST 24C02 2k(256x8) EEPROM ***
#define I2C_Speed              200000
#define I2C_SLAVE_ADDRESS7     0xA0
#define I2C_FLASH_PAGESIZE     8
#define EEPROM_HW_ADDRESS      0xA2   /* E0=1  E1=0 E2=0 (shift 1 pos. left)*/


/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);

void dump16(unsigned char *buf);
char AsciiToHexVal(char b);
unsigned char HexStrToByte(char *buf);

void SetupClock(void);

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

void I2C_Configuration(void);
void I2C_EE_Init(void);
void I2C_EE_ByteWrite(uint8_t* pBuffer, uint16_t WriteAddr);
void I2C_EE_PageWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint8_t NumByteToWrite);
void I2C_EE_BufferWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
void I2C_EE_BufferRead(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
void I2C_EE_WaitEepromStandbyState(void);


// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  // state of LED
char msgBuf[80]; // general purpose string buffer
uint16_t EEPROM_ADDRESS;
uint8_t eeBuf[17];

int main(void)
{
 int k=0;

 SetupClock();

 SystemCoreClockUpdate(); // MUST be called whenever the core clock is changed

  // ** Configures the SysTick event to fire every 10us	**
  SysTick_Config(SystemCoreClock / 100000);

 // Configure the GPIO pin for the LED (PC13)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// Clock PORTB Enable
  GPIO_InitStruct.GPIO_Pin = LED_PIN; // Configure Led pin
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// Set Output Push-Pull
  GPIO_Init(LED_PORT, &GPIO_InitStruct);

  SetupUSART();

  GPIO_ResetBits(LED_PORT, LED_PIN);  //LED ON
  TimingDelay = 50000;                // Blink every 1/2 sec.

  showMenu();

  I2C_EE_Init();

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

// ***********************************************************************
// showMenu()    Output the user menu to the USART
// ***********************************************************************
void showMenu(void)
{
  USART_PutStr("\r\n *** I2C 24C02 Menu *** \r\n");
  USART_PutStr(" 1. Read  1 Byte \r\n");
  USART_PutStr(" 2. Write 1 Byte \r\n");
  USART_PutStr(" 3. Read 16 Bytes\r\n");
  USART_PutStr(" 4. Write String \r\n");
  USART_PutStr(" 5. Blink LED \r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x;
  unsigned char addr, data;

  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n  You pressed ONE \r\n");
   break;
   case '2':
	   USART_PutStr("\r\n  Enter Write Start Address[2 hex chars]: ");
	   USART_GetStr(msgBuf, 3);
	   addr = HexStrToByte(msgBuf);

	   USART_PutStr("\r\n  Enter Data[2 hex chars]: ");
	   USART_GetStr(msgBuf, 3);
	   data = HexStrToByte(msgBuf);

	   I2C_EE_ByteWrite(&data, (uint16_t)addr);

	   USART_PutStr("\r\n  Wrote: ");
	   USART_PutHexByte(data);
	   USART_PutStr(" to Address: ");
	   USART_PutHexByte(addr);
	   USART_PutStr("\r\n");
   break;
   case '3':
	   USART_PutStr("\r\n  Enter Read Start Address[2 hex chars]: ");
	   USART_GetStr(msgBuf, 3);
	   addr = HexStrToByte(msgBuf);

	   USART_PutStr("\r\n  ");
	   USART_PutHexByte(addr);

	   I2C_EE_BufferRead(eeBuf, (uint16_t)addr, 16);

	   USART_PutStr("  ");
	   dump16(eeBuf);
   break;
   case '4':
	   USART_PutStr("\r\n  Enter Write Start Address[2 hex chars]: ");
	   USART_GetStr(msgBuf, 3);
	   addr = HexStrToByte(msgBuf);

	   USART_PutStr("\r\n  Enter Data[16 chars max]: ");
	   x = USART_GetStr((char *)eeBuf, 16);

	   I2C_EE_BufferWrite(eeBuf, addr, x);

	   USART_PutStr("\r\n  Wrote:' ");
	   USART_PutStr((char *)eeBuf);
	   USART_PutStr("' to Address: ");
	   USART_PutHexByte(addr);
	   USART_PutStr("\r\n");
   break;
   case '5':
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
// dump16(unsigned char *buf)
// Display 16 bytes as HEX and ASCII
//************************************************************************
void dump16(unsigned char *buf)
{
 int x;
 unsigned char k;

 for(x=0; x<16; x++)
   {
	 if(x==8)
	   USART_PutChar('-');
	 else
	   USART_PutChar(' ');

	 USART_PutHexByte(buf[x]);
   }

 USART_PutStr("     ");
 for(x=0; x<16; x++)
   {
	k = buf[x];
	if((k < 0x20)||(k > 0x7E))
		k= '.';

	 USART_PutChar(k);
   }

 USART_PutStr("\r\n");
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


/***************************************************************************//**
 *  @brief  I2C Configuration
 ******************************************************************************/
void I2C_Configuration(void)
{
    I2C_InitTypeDef  I2C_InitStructure;

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C_EE, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C_EE, &I2C_InitStructure);
}

/***************************************************************************//**
 * @brief  Initializes peripherals used by the I2C EEPROM driver.
 ******************************************************************************/
void I2C_EE_Init()
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 /* I2C Periph clock enable */
 RCC_APB1PeriphClockCmd(I2C_EE_CLK, ENABLE);

 /* GPIO Periph clock enable */
 RCC_APB2PeriphClockCmd(I2C_EE_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

 /* GPIO configuration */
 GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);

 /* Configure I2C_EE pins: SCL and SDA */
 GPIO_InitStructure.GPIO_Pin =  I2C_EE_SCL | I2C_EE_SDA;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
 GPIO_Init(I2C_EE_GPIO, &GPIO_InitStructure);

 /* I2C configuration */
 I2C_Configuration();

 /* Select the EEPROM address according to the state of E0, E1, E2 pins */
 EEPROM_ADDRESS = EEPROM_HW_ADDRESS;
}

/***************************************************************************//**
 * @brief      Writes one byte to the I2C EEPROM.
 * @param[in]  pBuffer   : pointer to the buffer  containing the data to be
 *                         written to the EEPROM.
 * @param[in]  WriteAddr : EEPROM's internal address to write to.
 * @return     None
 ******************************************************************************/
void I2C_EE_ByteWrite(uint8_t* pBuffer, uint16_t WriteAddr)
{
    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


    //** for parts with > 256 bytes **
    /* Send the EEPROM's internal address to write to : MSB of the address first */
    //I2C_SendData(I2C_EE, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /* Test on EV8 and clear it */
    //while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the EEPROM's internal address to write to : LSB of the address */
    I2C_SendData(I2C_EE, (uint8_t)(WriteAddr & 0x00FF));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(I2C_EE, *pBuffer);

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);
}

/***************************************************************************//**
 * @brief      Reads a block of data from the EEPROM.
 * @param[in]  pBuffer : pointer to the buffer that receives the data read
 *                       from the EEPROM.
 * @param[in]  ReadAddr : EEPROM's internal address to read from.
 * @param[in]  NumByteToRead : number of bytes to read from the EEPROM.
 * @return     None
 ******************************************************************************/
void I2C_EE_BufferRead(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
{
	// __disable_irq();


    /* While the bus is busy */
    while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    //*** For Parts with >256 bytes ********
    /* Send the EEPROM's internal address to read from: MSB of the address first */
    //I2C_SendData(I2C_EE, (uint8_t)((ReadAddr & 0xFF00) >> 8));

    /* Test on EV8 and clear it */
    //while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the EEPROM's internal address to read from: LSB of the address */
    I2C_SendData(I2C_EE, (uint8_t)(ReadAddr & 0x00FF));

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send START condition a second time */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for read */
    I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
    while(NumByteToRead)
    {
        if(NumByteToRead == 1)
        {
            /* Disable Acknowledgment */
            I2C_AcknowledgeConfig(I2C_EE, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(I2C_EE, ENABLE);
        }

        /* Test on EV7 and clear it */
        if(I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the EEPROM */
            *pBuffer = I2C_ReceiveData(I2C_EE);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C_EE, ENABLE);

    //__enable_irq();
}

/***************************************************************************//**
 * @brief      Writes buffer of data to the I2C EEPROM.
 * @param[in]  pBuffer : pointer to the buffer  containing the data to be
 *                       written to the EEPROM.
 * @param[in]  WriteAddr : EEPROM's internal address to write to.
 * @param[in]  NumByteToWrite : number of bytes to write to the EEPROM.
 * @return     None
 ******************************************************************************/
void I2C_EE_BufferWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, count = 0;
    uint16_t Addr = 0;

    Addr = WriteAddr % I2C_FLASH_PAGESIZE;
    count = I2C_FLASH_PAGESIZE - Addr;
    NumOfPage =  NumByteToWrite / I2C_FLASH_PAGESIZE;
    NumOfSingle = NumByteToWrite % I2C_FLASH_PAGESIZE;

    /* If WriteAddr is I2C_FLASH_PAGESIZE aligned  */
    if(Addr == 0)
    {
        /* If NumByteToWrite < I2C_FLASH_PAGESIZE */
        if(NumOfPage == 0)
        {
            I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
            I2C_EE_WaitEepromStandbyState();
        }
        /* If NumByteToWrite > I2C_FLASH_PAGESIZE */
        else
        {
            while(NumOfPage--)
            {
                I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_FLASH_PAGESIZE);
                I2C_EE_WaitEepromStandbyState();
                WriteAddr +=  I2C_FLASH_PAGESIZE;
                pBuffer += I2C_FLASH_PAGESIZE;
            }

            if(NumOfSingle!=0)
            {
                I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
                I2C_EE_WaitEepromStandbyState();
            }
        }
    }
    /* If WriteAddr is not I2C_FLASH_PAGESIZE aligned  */
    else
    {
        /* If NumByteToWrite < I2C_FLASH_PAGESIZE */
        if(NumOfPage== 0)
        {
            /* If the number of data to be written is more than the remaining space
              in the current page: */
            if (NumByteToWrite > count)
            {
                /* Write the data conained in same page */
                I2C_EE_PageWrite(pBuffer, WriteAddr, count);
                I2C_EE_WaitEepromStandbyState();

                /* Write the remaining data in the following page */
                I2C_EE_PageWrite((uint8_t*)(pBuffer + count), (WriteAddr + count), (NumByteToWrite - count));
                I2C_EE_WaitEepromStandbyState();
            }
            else
            {
                I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
                I2C_EE_WaitEepromStandbyState();
            }
        }
        /* If NumByteToWrite > I2C_FLASH_PAGESIZE */
        else
        {
            NumByteToWrite -= count;
            NumOfPage =  NumByteToWrite / I2C_FLASH_PAGESIZE;
            NumOfSingle = NumByteToWrite % I2C_FLASH_PAGESIZE;

            if(count != 0)
            {
                I2C_EE_PageWrite(pBuffer, WriteAddr, count);
                I2C_EE_WaitEepromStandbyState();
                WriteAddr += count;
                pBuffer += count;
            }

            while(NumOfPage--)
            {
                I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_FLASH_PAGESIZE);
                I2C_EE_WaitEepromStandbyState();
                WriteAddr +=  I2C_FLASH_PAGESIZE;
                pBuffer += I2C_FLASH_PAGESIZE;
            }
            if(NumOfSingle != 0)
            {
                I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
                I2C_EE_WaitEepromStandbyState();
            }
        }
    }
}

/***************************************************************************//**
 * @brief      Writes more than one byte to the EEPROM with a single WRITE cycle.
 *             Note: The number of byte can't exceed the EEPROM page size.
 * @param[in]  pBuffer : pointer to the buffer containing the data to be
 *                       written to the EEPROM.
 * @param[in]  WriteAddr : EEPROM's internal address to write to.
 * @param[in]  NumByteToWrite : number of bytes to write to the EEPROM.
 * @return     None
 ******************************************************************************/
void I2C_EE_PageWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint8_t NumByteToWrite)
{
    /* While the bus is busy */
    while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // ** for parts with > 265 bytes un-comment the 2 commands below **
    /* Send the EEPROM's internal address to write to : MSB of the address first */
    //I2C_SendData(I2C_EE, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /* Test on EV8 and clear it */
    //while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the EEPROM's internal address to write to : LSB of the address */
    I2C_SendData(I2C_EE, (uint8_t)(WriteAddr & 0x00FF));

    /* Test on EV8 and clear it */
    while(! I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* While there is data to be written */
    while(NumByteToWrite--)
    {
        /* Send the current byte */
        I2C_SendData(I2C_EE, *pBuffer);

        /* Point to the next byte to be written */
        pBuffer++;

        /* Test on EV8 and clear it */
        while (!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);
}

/***************************************************************************//**
 * @brief  Wait for EEPROM Standby state
 ******************************************************************************/
void I2C_EE_WaitEepromStandbyState(void)
{

 do
  {
   /* Send START condition */
   I2C_GenerateSTART(I2C_EE, ENABLE);

   /* Read I2C_EE SR1 register to clear pending flags */
   I2C_ReadRegister(I2C_EE, I2C_Register_SR1);

   /* Send EEPROM address for write */
   I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);
  }while(!(I2C_ReadRegister(I2C_EE, I2C_Register_SR1) & 0x0002));

    /* Clear AF flag */
 I2C_ClearFlag(I2C_EE, I2C_FLAG_AF);

    /* STOP condition */
 I2C_GenerateSTOP(I2C_EE, ENABLE);
}



// ***********************************************************************
//  SetupClock()
// Configure the System clock to 72MHz
// ***********************************************************************
void SetupClock()
{
  RCC_DeInit ();                    // RCC system reset(for debug purpose)
  RCC_HSEConfig (RCC_HSE_ON);       // Enable HSE

  // Wait till HSE is ready
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

  RCC_HCLKConfig   (RCC_SYSCLK_Div1);   // HCLK   = SYSCLK
  RCC_PCLK2Config  (RCC_HCLK_Div1);     // PCLK2  = HCLK
  RCC_PCLK1Config  (RCC_HCLK_Div2);     // PCLK1  = HCLK/2
  RCC_ADCCLKConfig (RCC_PCLK2_Div4);    // ADCCLK = PCLK2/4

  //PLLCLK = 8MHz * 9 = 72 MHz
  RCC_PLLConfig (0x00010000, RCC_PLLMul_9);

  RCC_PLLCmd (ENABLE);                  // Enable PLL

  // Wait till PLL is ready
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

  // Select PLL as system clock source
  RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);

  // Wait till PLL is used as system clock source
  while (RCC_GetSYSCLKSource() != 0x08);
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

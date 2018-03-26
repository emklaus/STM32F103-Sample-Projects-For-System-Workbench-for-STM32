// ***********************************************************************
// STM32F103_SDCard1
// derived from template for USART Menu with GPIO & SysTick support
// Using USART1 PA10=RX PA9=TX, SysTick interrupts every 10us
// Added files from the FatFS Library http://elm-chan.org/fsw/ff/00index_e.html
// Added stm32_spi_usd.*  with support routines for SD Card Read/Write via SPI
// Included SPI driver stm32f10x_spi.*
// Added basic functions to Init, Read & Write blocks of data on SD card.
// Linked FatFS functions to read & write files on SD Card
// NOTE: Tested with a 1GB SD card (2GB or larger may require changes)
//
//            +-------------------------------------------------------+
//            |                     Pin assignment                    |
//            +-------------------------+---------------+-------------+
//            |  STM32xx SPI Pins       | SD Signal     | SD Card Pin |
//            +-------------------------+---------------+-------------+
//            | SPI_CS_PIN         PB5  |   ChipSelect  |    1        |
//            | SPI_MOSI_PIN/MOSI  PA7  |   DataIn      |    2        |
//            |                         |   GND         |    3 (0 V)  |
//            |                         |   VDD         |    4 (3.3 V)|
//            | SPI_SCK_PIN/SCLK   PA5  |   Clock       |    5        |
//            |                         |   GND         |    6 (0 V)  |
//            | SPI_MISO_PIN/MISO  PA6  |   DataOut     |    7        |
//            +-------------------------+---------------+-------------+
//
//
// By Eric M. Klaus   2/2016
// ***********************************************************************
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stdlib.h"
#include "string.h"

#include "stm32f10x_spi.h"
#include "stm32_spi_usd.h"

#include "ff.h"
#include "ffconf.h"
#include "diskio.h"

#define LED_PIN  GPIO_Pin_13
#define LED_PORT GPIOC

/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void showMenu(void);
void processMenuCmd(char cmd);

// **** FatFS Demo Routines ********
uint32_t fatfs_ListDirectoryFiles(char* DirName);
uint32_t fatfs_ListFile(char* filename);
int InitCheck(uint8_t doFS);


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
void dump16(unsigned char *buf);
char AsciiToHexVal(char b);
unsigned char HexStrToByte(char *buf);

// Global timing counters
static __IO uint32_t TimingDelay, ctMs;
static __IO uint32_t usCounter, msCounter;

GPIO_InitTypeDef GPIO_InitStruct;

int led1_val=0;  // state of LED
char msgBuf[80]; // general purpose string buffer

uint8_t dataBuf[SD_BLOCK_SIZE];
uint8_t SD_Init_flag=0;

// ******** FatFS Variables **************************************
FATFS myFs;  // Global FileSystem object (pass this to f_mount() )

// Some global FatFS related variables
uint8_t aBuffer[513];
FILINFO MyFileInfo;
DIR MyDirectory;
UINT BytesWritten;
UINT BytesRead;
FRESULT res;
FIL file1;
uint8_t fatFS_Init=0, fatFS_Mount=0;
char fileNameBuf[13];


int main(void)
{
 int k=0;
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
  USART_PutStr(" 1. Init & Read SD Card\r\n");
  USART_PutStr(" 2. Read Block \r\n");
  USART_PutStr(" 3. Write Block \r\n");
  USART_PutStr(" 4. Option Four \r\n");
  USART_PutStr(" 5. Dir List\r\n");
  USART_PutStr(" 6. Read   A File\r\n");
  USART_PutStr(" 7. Create A File\r\n");
  USART_PutStr(" 8. Delete A File\r\n");
  USART_PutStr(" M. Re-Display Menu \r\n");
}

void processMenuCmd(char cmd)
{
  int x, len;
  uint8_t sdError;
  uint16_t status;
  uint32_t ReadAddr;

  switch(cmd)
  {
   case '1':
	   USART_PutStr("\r\n  Read SD Card   ");

	   if(InitCheck(0))
	      break;

	   status = SD_GetStatus();
	   if(status != 0xFF)
	     {
	      USART_PutMsgAndVal("  SD_GetStatus():", status, 0);
	     }


	   sdError = SD_ReadBlock(dataBuf, 1024, SD_BLOCK_SIZE);
	   if(sdError != 0)
	     {
	       USART_PutMsgAndVal("  SD_ReadBlock(1024):", sdError, 1);
	     }
	   else
	     {
		   USART_PutStr("\r\n\n");
  	       for(x=0; x<SD_BLOCK_SIZE; x+=16)
	          {
		        dump16((unsigned char *)&dataBuf[x]);
	          }
	     }

   break;
   case '2':
	   USART_PutStr("\r\n  Enter Read Block#:");
	   USART_GetStr(msgBuf, 4);
	   x = (int)strtol(msgBuf,0,10);
	   ReadAddr = x * SD_BLOCK_SIZE;
	   itoa(ReadAddr, msgBuf, 10);
	   USART_PutStr("\r\n Address: ");
	   USART_PutStr(msgBuf);

	   if(InitCheck(0))
	      break;

	   sdError = SD_ReadBlock(dataBuf, ReadAddr,SD_BLOCK_SIZE);
	   if(sdError != 0)
	     {
	       USART_PutMsgAndVal("  SD_ReadBlock():", sdError, 1);
	     }
	   else
	     {
		  USART_PutStr("\r\n\n");
	      for(x=0; x<SD_BLOCK_SIZE; x+=16)
	         {
		      dump16((unsigned char *)&dataBuf[x]);
	         }
	     }

   break;
   case '3':
	   USART_PutStr("\r\n  Enter Block#:");
	   USART_GetStr(msgBuf, 4);
	   x = (int)strtol(msgBuf,0,10);
	   ReadAddr = x * SD_BLOCK_SIZE;
	   itoa(ReadAddr, msgBuf, 10);
	   USART_PutStr("\r\n Address: ");
	   USART_PutStr(msgBuf);

	   if(InitCheck(0))
	      break;

	   USART_PutStr("\r\n Enter Data:");
	   len = USART_GetStr(msgBuf, 80);
	   USART_PutMsgAndVal(" GetStr len:", len, 1);
	   for(x=0; x<SD_BLOCK_SIZE-len; x+=len)
	      {
		   memcpy(&dataBuf[x], msgBuf, len);
	      }

	   sdError = SD_WriteBlock(dataBuf, ReadAddr,SD_BLOCK_SIZE);
	   USART_PutMsgAndVal("  SD_WriteBlock():", sdError, 1);

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
	   USART_PutStr("\r\n  List Root Dir \r\n");

	   if(InitCheck(1))
	      break;

	   fatfs_ListDirectoryFiles("/");

	   USART_PutStr("\n\n[Done]\r\n");
   break;
   case '6':
	   USART_PutStr("\r\n  Enter Read Filename[8.3 format]:");
	   len = USART_GetStr(msgBuf, 13);

	   if(InitCheck(1))
	      break;

	   USART_PutStr("\r\n\n");

	   fatfs_ListFile(msgBuf);

	   USART_PutStr("\r\n\n[Done]\r\n");
   break;
   case '7':
	   USART_PutStr("\r\n  Enter Create Filename[8.3 format]:");
	   len = USART_GetStr(fileNameBuf, 13);

	   if(InitCheck(1))
	      break;

	   res = f_open(&file1, fileNameBuf, FA_CREATE_ALWAYS | FA_WRITE);
	   USART_PutStr(" f_open(): ");
	   USART_PutHexByte((unsigned char) res);

	   USART_PutStr("\r\n  Enter file data:");
	   len = USART_GetStr(msgBuf, 79);
	   USART_PutMsgAndVal("\n Msg Len:", len, 1);

	   res = f_write(&file1, msgBuf, len, &BytesRead);
	   USART_PutStr(" f_write(): ");
	   USART_PutHexByte((unsigned char) res);
	   USART_PutStr(" BytesRead: 0x");
	   USART_PutHexWord(BytesRead);

       f_close(&file1);

       USART_PutStr("\r\n [Reading ");
       USART_PutStr(fileNameBuf);
       USART_PutStr(" ]\r\n\n");

       fatfs_ListFile(fileNameBuf);

	   USART_PutStr("\r\n\n[Done]\r\n");
   break;
   case '8':
	   USART_PutStr("\r\n  Enter DELETE Filename[8.3 format]:");
	   len = USART_GetStr(fileNameBuf, 13);

	   if(InitCheck(1))
	      break;

	   res = f_unlink (fileNameBuf);
	   USART_PutMsgAndVal("\n f_unlink(): ", res, 1);

	   USART_PutStr("\r\n\n[Done]\r\n");
   break;

   case 'M':
   case 'm':
	   showMenu();
   break;
  }

}

// ***********************************************************************
// fatfs_ListDirectoryFiles(char* DirName)
// List entries in the root directory
// Return number of entries.
// ***********************************************************************
uint32_t fatfs_ListDirectoryFiles(char* DirName)
{
  uint32_t j = 0;
  FRESULT res;

  USART_PutStr("\r\n\n  Directory Listing of: ");
  USART_PutStr(DirName);
  USART_PutStr("\r\n");

  res = f_opendir(&MyDirectory, DirName);

  if(res == FR_OK)
  {
	USART_PutStr("\r\n  ");
    for (;;)
    {
      res = f_readdir(&MyDirectory, &MyFileInfo);
      if(res != FR_OK || MyFileInfo.fname[0] == 0) break;
      //if(MyFileInfo.fname[0] == '.') continue;

      USART_PutStr(MyFileInfo.fname);
      USART_PutStr("\r\n  ");
      j++;
    }
  }

  return j;
}

// ***********************************************************************
// fatfs_ListFile(char* filename)
// Open and list a file to the serial port.
// Return the number of bytes read.
// ***********************************************************************
uint32_t fatfs_ListFile(char* filename)
{
 uint32_t fileLen=0;
 uint8_t readBuf[513];
 FIL fp;
 UINT BytesRead;
 FRESULT res;


 res = f_open(&fp, filename, FA_READ);
 if(res!=FR_OK)
   {
    USART_PutMsgAndVal(" f_open() ERROR: ", res, 1);
    return 0;
   }

 while(1)
   {
    res = f_read(&fp, readBuf, 512, &BytesRead);
    if(res!=FR_OK)
      {
       USART_PutMsgAndVal(" f_read() ERROR: ", res, 1);
       break;
      }

    readBuf[BytesRead]=0;
 	USART_PutStr((char *)readBuf);
 	fileLen+=BytesRead;

 	if(BytesRead < 512)
 	  break;
   }

 f_close(&fp);
 USART_PutStr("\r\n");
 return fileLen;
}

// ***********************************************************************
// InitCheck(uint8_t doFS)
// Check for initialization of DS card and FatFS volume
// Execute the initialization if it has not already been done
// Returns zero on success.
// ***********************************************************************
int InitCheck(uint8_t doFS)
{
 FRESULT res=0;

 if((!SD_Init_flag)|(!fatFS_Init))
   {
    STM_SPI_Init();
    if(! doFS)
      {
	   res =  SD_Init();
	   if(res)
	    {
	      USART_PutMsgAndVal(" SD_init():", res, 0);
	      fatFS_Init=0;
	      return (int)res;
	    }
      }
    else
      {
	   res = disk_initialize(0);

	  if(res)
	    {
		  USART_PutMsgAndVal(" disk_initialize(): ", res, 0);
		  fatFS_Init=0;
		  SD_Init_flag=0;
		  return (int)res;
	    }

	   fatFS_Init=1;
       SD_Init_flag=1;
      }
   }


 if((!fatFS_Mount)&(doFS))
   {
    res = f_mount(&myFs,"/",0);
	if(res)
	  {
	   USART_PutStr(" f_mount(): ");
	   USART_PutHexByte((unsigned char) res);
	   fatFS_Mount=0;
	  }
	else
	 {
      fatFS_Mount=1;
	 }
   }

 return (int)res;
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

		if((k=='\r')||(k=='\n')||(k==0x1B)) // LF CR or Esc = EXIT
			break;

		if((k==0x7F)&&(i>1))  // Backspace
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

 usCounter++;
 if(++ctMs > 99)
   {
	msCounter++;
	ctMs=0;
   }
}




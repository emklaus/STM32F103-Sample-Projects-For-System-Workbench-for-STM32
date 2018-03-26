/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

// ** Modified for Single SD Card demo implementation ***

#include "stm32_spi_usd.h"	 /* Example: MMC/SDC control */
#include "diskio.h"		       /* FatFs lower layer API */


#define BLOCK_SIZE 512 /* Block Size in Bytes */

/* Definitions of physical drive number for each media */
#define MMC		0    // Always use this...

static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(uint8_t pdrv)
{
  Stat = STA_NOINIT;
  
  if (pdrv == 0)  // Always = 0 for SD Card Demo
  {
    if (SD_Init() == SD_RESPONSE_NO_ERROR)
    {
      Stat &= ~STA_NOINIT;
    }
  }
  
  return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (uint8_t pdrv)
{
  if (pdrv == 0) /* Supports only single drive */
  {
    Stat = STA_NOINIT;
    
    if(SD_GetStatus() == 0xFF)
    {
      Stat &= ~STA_NOINIT;
    }
  }
  
  return Stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (uint8_t pdrv, uint8_t *buff, uint32_t sector, uint16_t count)
{
  if (pdrv == 0)
  {
    if(SD_GetStatus() != 0xFF)
    {
      return(RES_NOTRDY);
    }
    
    SD_ReadBlock(buff, sector << 9, BLOCK_SIZE);
  }
  return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
  if (pdrv == 0)
  {
    if(SD_GetStatus() != 0xFF)
    {
      return(RES_NOTRDY);
    }
    
    SD_WriteBlock((BYTE *)buff, sector << 9, BLOCK_SIZE); 
  }
  return RES_OK;
}
#endif


/**
  * @brief  Get current time 
  * @param  none
  * @retval DWORD: Current time
  */
DWORD get_fattime ()
{
  return(0);
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  DRESULT res = RES_OK;
  SD_CardInfo cardinfo; 
  
  if (pdrv) return RES_PARERR;
  
  res = RES_ERROR;
  
  if (Stat & STA_NOINIT) return RES_NOTRDY;
  
  switch (cmd) {
    /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;
    
    /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    if(pdrv == 0)
    {
      SD_GetCardInfo(&cardinfo);  
      *(DWORD*)buff = cardinfo.CardCapacity / 512; 
    }
    
    res = RES_OK;
    break;
    
    /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :	 
    *(WORD*)buff = BLOCK_SIZE;
    res = RES_OK;
    break;
    
    /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    if(pdrv == 0)
    {
      SD_GetCardInfo(&cardinfo); 
      *(DWORD*)buff = cardinfo.CardBlockSize;
    }
    else
    {
      *(DWORD*)buff = 32;
    }
    res = RES_OK;
     break;
  default:
    res = RES_PARERR;
  }
  
  return res;
}
#endif

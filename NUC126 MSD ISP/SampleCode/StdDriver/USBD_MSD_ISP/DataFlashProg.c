/****************************************************************************
 * @file     DataFlashProg.c
 * @brief    NUC126 Series Data Flash Access API
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/


/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "NUC126.h"
#include "DataFlashProg.h"

#if 0
# define dbg     printf
#else
# define dbg(...)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

uint32_t g_sectorBuf[FLASH_PAGE_SIZE / 4];

uint8_t u8FormatData[62] = 
{
    0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53,
    0x35, 0x2E, 0x30, 0x00, 0x02, 0x01, 0x06, 0x00,
    0x02, 0x00, 0x02, 0xA8, 0x00, 0xF8, 0x01, 0x00,
    0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0xB9,
    0xC1, 0xAA, 0x42, 0x4E, 0x4F, 0x20, 0x4E, 0x41,
    0x4D, 0x45, 0x20, 0x20, 0x20, 0x20, 0x46, 0x41,
    0x54, 0x31, 0x32, 0x20, 0x20, 0x20
};


uint8_t u8RootDirData[96] = 
{
    0x42, 0x20, 0x00, 0x49, 0x00, 0x6E, 0x00, 0x66,
    0x00, 0x6F, 0x00, 0x0F, 0x00, 0x72, 0x72, 0x00,
    0x6D, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00,
    0x6F, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x00, 0x00,
    0x01, 0x53, 0x00, 0x79, 0x00, 0x73, 0x00, 0x74,
    0x00, 0x65, 0x00, 0x0F, 0x00, 0x72, 0x6D, 0x00,
    0x20, 0x00, 0x56, 0x00, 0x6F, 0x00, 0x6C, 0x00,
    0x75, 0x00, 0x00, 0x00, 0x6D, 0x00, 0x65, 0x00,
    0x53, 0x59, 0x53, 0x54, 0x45, 0x4D, 0x7E, 0x31,
    0x20, 0x20, 0x20, 0x16, 0x00, 0x99, 0x0D, 0x5C,
    0x6D, 0x43, 0x6D, 0x43, 0x00, 0x00, 0x0E, 0x5C,
    0x6D, 0x43, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#define FAT_SECTORS             u8FormatData[14]
#define ROOT_DIR_SECTORS        ((((u8FormatData[18] << 8) + u8FormatData[17]) * 32) / 512)  
#define DATA_SECTOR_ADDRESS     ((FAT_SECTORS + 2 + ROOT_DIR_SECTORS) * 512)
#define FLASH_OFFSET  0x0
void my_memcpy(void *dest, void *src, int32_t size)
{
    int32_t i;

    for (i = 0; i < size; i++)
       *((uint8_t *)dest + i) = *((uint8_t *)src + i);
}

/* This is low level read function of USB Mass Storage */
void DataFlashRead(uint32_t addr, uint32_t size, uint32_t buffer)
{
    uint32_t i;
	 uint32_t *pu32;
    pu32 = (uint32_t *)buffer;
    for (i = 0; i < FLASH_PAGE_SIZE/4; i++)
    {
        pu32[i] = 0;
    }                

if (addr == 0x00000000)
    {
        my_memcpy((uint8_t *)pu32, u8FormatData, 62);
        
        pu32[FLASH_PAGE_SIZE/4-1] = 0xAA550000;            
    }
    else
    {
        if ( (addr == (FAT_SECTORS * 512)) || (addr == ((FAT_SECTORS+1) * 512)) )
        {
            pu32[0] = 0x00FFFFF8;
        }
        else if (addr == (8 * 512)) /* root dir */
        {
            my_memcpy((uint8_t *)pu32, u8RootDirData, 96);
        }
    }
}


uint32_t FMC_ProgramPage(uint32_t u32StartAddr, uint32_t * u32Buf)
{
    uint32_t i;

    for (i = 0; i < FLASH_PAGE_SIZE/4; i++)
    {
        FMC_Write(u32StartAddr + i*4, u32Buf[i]);
    }

    return 0;
}

volatile unsigned int  BIN_CHECK=0;
void DataFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer)
{
	uint32_t *pu32;
	uint32_t i;
	pu32 = (uint32_t *)buffer;
	if ( (addr >= DATA_SECTOR_ADDRESS) && (addr < (DATA_SECTOR_ADDRESS + DATA_FLASH_STORAGE_SIZE)) )
    {
        //UNLOCKREG();
        //FMC->ISPCON.ISPEN = 1;
      addr -= DATA_SECTOR_ADDRESS;

			if(addr==0)
			{
				if((pu32[0]>0x20000000)&&pu32[0]<0x20005000)
				{
				//printf("data0:0x%x\n\r",pu32[0]);			
				BIN_CHECK=1;
				}
			
			}
      if(BIN_CHECK==1) 
			{
			 
		   	if((addr+FLASH_OFFSET)%2048==0)
				{
					//printf("Address:0x%x\n\r",addr+FLASH_OFFSET);	
				  FMC_Erase(addr+FLASH_OFFSET);
					}
			 //FMC_ProgramPage(addr+FLASH_OFFSET, pu32);
				for (i = 0; i < FLASH_PAGE_SIZE/4; i++)
        {
				//	 printf("data:0x%x\n\r",pu32[i]);
        FMC_Write((addr+FLASH_OFFSET) + (i*4), pu32[i]);
        }
			}       
   }
	
	
	

}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB Mass-Storage.
 *           It uses embedded data flash as storage.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"
#include "massstorage.h"

/* If crystal-less is enabled, system won't use any crystal as clock source
   If using crystal-less, system will be 48MHz, otherwise, system is 72MHz
*/
#define PLLCTL_SETTING  CLK_PLLCTL_144MHz_HXT
#define PLL_CLOCK       144000000
#define CRYSTAL_LESS        1
#define HIRC48_AUTO_TRIM    0x412   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x118)



/*--------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);
	
#ifndef CRYSTAL_LESS
    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    CLK->CLKSEL3 |= CLK_CLKSEL3_USBDSEL_PLL;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_USBDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_USB(3);

    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 2;        // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
#else
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48EN_Msk;
    while(!(CLK->STATUS & CLK_STATUS_HIRC48STB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC48;

    CLK->CLKSEL3 &= ~CLK_CLKSEL3_USBDSEL_Msk;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_USBDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_USB(1);

    SystemCoreClock = 48000000;        // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
#endif

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;

    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
 // SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
 //   SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;
}
#if 0
void UART0_Init(void)
{
//    /*---------------------------------------------------------------------------------------------------------*/
//    /* Init UART                                                                                               */
//    /*---------------------------------------------------------------------------------------------------------*/
//    /* Reset IP */
   SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

//    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TrimInit;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
  // UART0_Init();

//    printf("+-------------------------------------------------------+\n");
//    printf("|          NuMicro USB MassStorage Sample Code          |\n");
//    printf("+-------------------------------------------------------+\n");

    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();
	  if(PB15==1)
		{		

                /* Set VECMAP to LDROM for booting from LDROM */
                FMC_SetVectorPageAddr(FMC_APROM_BASE);

                /* Software reset to boot to LDROM */
                NVIC_SystemReset();
		            while(1);
		}
	
	
    FMC_EnableAPUpdate();
    USBD_Open(&gsInfo, MSC_ClassRequest, NULL);

    USBD_SetConfigCallback(MSC_SetConfig);

    /* Endpoint configuration */
    MSC_Init();
    USBD_Start();

#ifdef CRYSTAL_LESS
    /* Backup init trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less */
    SYS->IRCTCTL1 = HIRC48_AUTO_TRIM;
#endif

    NVIC_EnableIRQ(USBD_IRQn);

    while(1)
    {
#ifdef CRYSTAL_LESS
        /* Re-start crystal-less when any error found */
        if(SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk))
        {
            /* USB clock trim fail. Just retry */
            SYS->IRCTCTL1 = 0;  /* Disable crystal-less */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk;
            
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;
            
            /* Waiting for USB bus stable */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
            while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);
            
            SYS->IRCTCTL1 = HIRC48_AUTO_TRIM; /* Re-enable crystal-less */
           // printf("USB trim fail. Just retry.\n");
        }
#endif

        MSC_ProcessCmd();
    }
}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


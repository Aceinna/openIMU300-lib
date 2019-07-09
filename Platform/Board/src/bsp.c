/******************************************************************************
* @file bsp.c
* @brief
* File description:
*		Use the file to Configure Cortex M3
*
* $Rev: 16166 $
* @date: 2011-03-09 11:53:45 -0800 (Wed, 09 Mar 2011) $
* @author: whpeng $
******************************************************************************/
#include "stm32f4xx_conf.h"
#include "bsp.h"
#include "configureGPIO.h"
#include "stdio.h"
#include "boardAPI.h"
#include "platformAPI.h"
#include "BITStatus.h"


 void InitSystemTimer(void)
 {
     /// need 1s interrupt as well from GPS, hooked in to the system to force
     ///   sampling on that boundary.
     SysTick_Config(SystemCoreClock / 1000);
 }


/** ****************************************************************************
 * @name BSP_init
 * @brief This function should be called by your application code before you make
 *        use of any of the functions found in this module
 *
 * Trace:
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void BSP_init(void)
{
    int i;
    //    RCC_config(); /// System clocks configuration
	//NVIC_config();
	GPIO_config();
	InitSystemTimer();
    for(i = WWDG_IRQn; i <= FPU_IRQn; i++){
         NVIC_SetPriority((IRQn_Type)i, 0x02);
    }

    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
}

/** ****************************************************************************
 * @name GPIO_config
 * W@brief configuration GPIO
 *
 * Trace:
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void GPIO_config(void)
{
  ;
}

/** ****************************************************************************
 * @name RCC_config
 * @brief Configures the different system clocks and Enable peripherals.
 *
 * Trace:
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void RCC_config(void)
{
#define SYSTICK_RCC_CONF
#if defined(SYSTICK_RCC_CONF) /// temporary reserves
	ErrorStatus HSEStartUpStatus;

	RCC_DeInit(); /// RCC system reset(for debug purpose)
	RCC_HSEConfig(RCC_HSE_ON); /// Enable HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); /// Wait till HSE is ready

	if(HSEStartUpStatus == SUCCESS)
	{
		/// Enable Prefetch Buffer
        FLASH_PrefetchBufferCmd(ENABLE);
        FLASH_InstructionCacheCmd(ENABLE);
        FLASH_DataCacheCmd(ENABLE);

		FLASH_SetLatency(FLASH_Latency_3);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);

        //RCC_PLLConfig( RCC_PLLSource_HSE, 25, 240, 2, 5 );
        RCC_PLLConfig( RCC_PLLSource_HSE, 24, 232, 2, 5 );

		RCC_PLLCmd(ENABLE); /// Enable PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) /// Wait till PLL is ready
		{ /* spin */ }

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); /// Select PLL as system clock

		/// Wait till PLL is used as system clock source
		while(RCC_GetSYSCLKSource() != 0x08)
		{ /* spin */ }
	}
// (FIXME) JSM - HSE enable in the following function
   SystemCoreClockUpdate();
#elif defined(SYSTICK_STM32_CONF)
	SystemInit(); /// temporary reserves
#endif

}

/** ****************************************************************************
 * @name NVIC_config
 * @brief Configures Vector Table base location.
 *
 * Trace:
 *
 * @param N/A
 * @retval
 ******************************************************************************/
void NVIC_config(void)
{
#if defined (VECT_TAB_RAM)
	/// Set the Vector Table base location at 0x20000000
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#elif defined(VECT_TAB_FLASH_IAP)
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, NVIC_FLASH_IAP);
#else  /// VECT_TAB_FLASH
	/// Set the Vector Table base location at 0x08000000
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
	/// Configure the NVIC Preemption Priority Bits
}

void SetIntVectorOffset(u32 offset)
{
	 NVIC_SetVectorTable(NVIC_VectTab_FLASH, offset);   
}


/** ***************************************************************************
 * @name BoardInit The DMU380 board initialization
 * @param [in] N/A
 * @param [out] "#> " out serial to console
 * @retval N/A
 ******************************************************************************/

void BoardInit(void)  
{
    // Initialize the system clock, PLL, etc
    SystemInit();            // system_stm32f2xx.c
    SetIntVectorOffset(APP_NVIC_OFFSET);

    /*int tmp = */SystemCoreClockUpdate(); // system_stm32f2xx.c

    // In the case of the HSI clock configuration, change "system_stm32fxx.c"
    //   based on the Excel-generated file and comment out RCC_config() in
    //   BSP_init.  The variable 'tmp' is set based on whether the internal or
    //   external clock is used by the processor.  Both will generate
    //   approximately a 120 MHz system clock using PLLs.
    BSP_init();              // bsp.c
    
    BoardDetectTestMode();

    ControlPortInit();
    RCC_ClearFlag(); ///< reset flags - stm32f2xx_rccc.c
    // ---------------------------- BOOT CAUSE MESSAGING -----------------------
    /// Initialize data-ready (DR), configuration, and (1-PPS) pins as input pins
    ///   (the board is configured according to the signal levels on these pins)
    InitBoardConfiguration_GPIO();  // configureGPIO.c
    ReadUnitHwConfiguration();   // configureGPIO.c

    // Debugging pin IO3 will be used to indicate to the user when the rate-
    //   sensor is read (with a rising edge).  Start operation by setting this
    //   pin low.
    GPIOB->BSRRH = GPIO_Pin_11;  // Set IO3 low

}

void BoardGetResetStatus(char *destination, int len)
{
    if( RCC_GetFlagStatus( RCC_FLAG_PORRST ) ) { // Power-on/power-down reset
        sprintf(destination, "\r\nNormal power-on.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_BORRST ) ) { // Brown-out reset
        sprintf(destination,"\r\nReset due to power brown-out.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_IWDGRST ) ) { // Watchdog reset
        sprintf(destination,"\r\nReset due to watch dog timeout.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_LPWRRST ) ) { // Low power
        sprintf(destination,"\r\nReset due to low-power.\r\n");
    } else if( RCC_GetFlagStatus( RCC_FLAG_SFTRST ) ) {  // Software reset
        sprintf(destination,"\r\nReset due to software reset.\r\n");
    }

}  

void set_DebugI2CPin(BOOL High)
{

}

BOOL testMode = FALSE;

BOOL BoardDetectTestMode()
{
    testMode = GPIO_DetectTestMode();
    return testMode;
}

void BoardClearTestMode()
{
    testMode = FALSE;
}

BOOL  BoardIsTestMode()
{
    return testMode;
}

int drdyStatus = 1, mosiStatus = 1;

void BoardInitPinsForTestMode()
{
    if(!testMode){
        return;
    }

    GPIO_InitPinsForTestMode();
    setDataReadyPin(1);
    setMosiPin(1);

    drdyStatus = 1; 
    mosiStatus = 1;
}

BOOL BoardPerformSelfTest()
{
    BOOL error = 0;
    int ppsStatus, nssStatus;

    if(!testMode){
        return TRUE;
    }

    ppsStatus = GPIO_ReadInputDataBit(ONE_PPS_PORT, ONE_PPS_PIN);
    nssStatus = GPIO_ReadInputDataBit(SPI3_SLAVE_SELECT_PORT, SPI3_SLAVE_SELECT_PIN);

    if (ppsStatus != drdyStatus) {
        gBitStatus.hwBIT.bit.ppsError = 1;
        error = TRUE;
    }
    
    if (nssStatus != mosiStatus) {
        gBitStatus.hwBIT.bit.nssError = 1;
        error = TRUE;
    }
    
    if (drdyStatus) {
        setDataReadyPin(0);
        drdyStatus = 0;
    }else {
        setDataReadyPin(1);
        drdyStatus = 1;
    }
    
    if (mosiStatus){
        setMosiPin(0);
        mosiStatus = 0;
    }else {
        setMosiPin(1);
        mosiStatus = 1;
    }

    return error;
}


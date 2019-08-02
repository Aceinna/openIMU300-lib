/** ***************************************************************************
 * @file   taskDataAcquisitionSupport.c
 * @Author
 * @copyright (c) 2019 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 200Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#include <stdlib.h>
#include "stm32f4xx_conf.h"
#include "boardDefinition.h" ///< For ONE_PPS_PIN, etc

#include "AlgorithmLimits.h"
#include "WorldMagneticModel.h"
#include "watchdog.h"
#include "bitSelfTest.h"
#include "osapi.h"
#include "osresources.h"
#include "platformAPI.h"
#include "BITStatus.h"
#include "uart.h"
#include "sensorsAPI.h"
#include "spiAPI.h"
#include "boardAPI.h"
#include "commAPI.h"



void    _InitExternalSync( FunctionalState NewState );

// todo tm20160603 - PUT IN A BETTER PLACE!  TEMPORARY TO GET THIS COMPILING AND RUNNING!!!
#include "TimingVars.h"
TimingVars           timer;   // for InitTimingVars


typedef struct {
    union {
       uint64_t time;
       struct {
           uint32_t timeLo;
           uint32_t timeHi;
       };
    };
}stime_t;

uint64_t currSystemTime = 0;
uint32_t prevTim5Val    = 0;
uint64_t ppsTstamp      = 0;

stime_t tStamp;

uint64_t platformGetCurrTimeStamp()
{
    OSDisableHook();
    uint32_t cur =  TIM_GetCounter(TIM5);
    if(cur < prevTim5Val){
        tStamp.timeHi++;
    }
    tStamp.timeLo = cur; 
    prevTim5Val   = cur;
    uint64_t res = tStamp.time / 60;
    OSEnableHook();
    return res;
}

uint64_t platformGetCurrTimeStampFromIsr()
{
    uint32_t cur =  TIM_GetCounter(TIM5);
    if(cur < prevTim5Val){
        tStamp.timeHi++;
    }
    tStamp.timeLo = cur; 
    prevTim5Val   = cur;
    return tStamp.time / 60;
}


int platformGetPpsToDrdyDelay()
{
    return 0;
}

int platformGetSensToPpsDelay()
{
    return 0;
}

uint64_t platformGetPpsTimeStamp()
{
    return ppsTstamp;
}



uint32_t prevTs = 0;

static uint8_t  ppsDetected  =  0; // 0 not synced, 1 synced
uint8_t getPpsFlag( void ) { return ppsDetected; } // dmu.h
void    setPpsFlag( uint8_t gotPpsFlag ) { ppsDetected = gotPpsFlag; }
uint8_t platformGetPpsFlag( void ) { return ppsDetected; } // dmu.h
void    platformSetPpsFlag( uint8_t gotPpsFlag ) { ppsDetected = gotPpsFlag; }

/** ***************************************************************************
 * @name ONE_PPS_EXTI_IRQHandler() LOCAL TIM2 global interrupt request handler.
 * @brief The value of the timer that triggers the interrupt is based on the
 *        input to 'InitDataAcquisitionTimer()'. Upon TIM2 interrupt, the rate
 *        sensor data buffers are reset when the function GyroStartReading() is
 *        called. The sensor data-ready interrupt starts the data transfer to
 *        the data buffers.
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void ONE_PPS_EXTI_IRQHandler(void)
{

    OSEnterISR();

    if(platformIsGpsPPSUsed()){
        ppsTstamp     = platformGetCurrTimeStampFromIsr();
        ppsDetected   = 1;
    }

    EXTI->PR = ONE_PPS_EXTI_LINE;  ///< Clear the interrupt bit
    
    OSExitISR();
}

/** ***************************************************************************
 * @name InitDataAcquisitionTimer() Set up and initialize the timer interrupt
 *       that drives data acquisition.
 * @brief After timeout, TIM2_IRQHandler, resets the rate-sensor data buffers.
 *        The individual sensor interrupts populates the data buffers when the
 *        sensors indicate 'data-ready'.
 *
 *        Using 32 bit TIM2. Get a sync signal from GPS or the user on TIM2.
 *        use that to trim the TIM2 reload value to have a timer that is the
 *        same rate as the desired outputDataRate
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void InitDataAcquisitionTimer(uint32_t outputDataRate)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    double period;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    TIM_Cmd( TIM2, DISABLE);       ///< TIM disable counter
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    /// Set the timer interrupt period (counter value at interrupt).  Alter this
    /// to sync with a 1PPS or 1kHz signal. ==> SystemCoreClock = 120MHz
    period = (double)( SystemCoreClock ); // >> 1; ///< period = 120 MHz / 2 = 60 MHz
    period = 0.5 * period / (double)outputDataRate;    ///< = period / ODR ==> 60 MHz / 500 Hz = 120,000

    /// Time base configuration
    TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseStructure.TIM_Period      = (uint32_t)(period+0.5);
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
    TIM_ARRPreloadConfig( TIM2, ENABLE );

    /// Enable the TIM2 global interrupt, TIM2_IRQn
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE ); ///< TIM Interrupts enable
    /// Ensure an UEV generates an interrupt by deselecting the URS bit in TIM2_CR1
    TIM2->CR1 &= (uint16_t)~TIM_CR1_URS;
    
    TIM_Cmd( TIM2, ENABLE ); ///< TIM enable counter
}





/** ***************************************************************************
 * @name TIM2_IRQHandler()
 * @brief The value of the timer that triggers the interrupt (period) is based
 *        on the input to the function 'InitDataAcquisitionTimer'. Upon TIM2
 *        interrupt, the rate sensor data buffers are reset when the function
 *        GyroStartReading() is called. The sensor data-ready interrupt handles
 *        the data transfer to the data buffers.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
uint16_t N_per = 0;

// Leave this in here for now so the change can be easily rolled back if
//   something going forward doesn't work quite right.
#define  TIM2_800Hz  1

static uint8_t TIM2_Cntr      = 0;
static uint8_t TIM2_CntrLimit = 3;  // 200 Hz Sampling
uint32_t       syncPulseTimeStamp = 0, syncPulsePeriod = 0, dacqTickTimeStamp = 0;
double         cntPeriod800;
BOOL   ppsSync            = FALSE;
int    numAdjustments     = 0;
int    adjustmentVal      = 0; 
int    adjustmentValLast  = 0; 
int    rem    = 0;
int    divv   = 0;
int    divva  = 0;
int    ratioP = 0, ratioM = 0, adjStep = 0, adj1 = 0;
BOOL   dir;
BOOL   syncP  = FALSE;


int  nestedTim2  = 0;
int  nestCntTim2 = 0;
void TIM2_IRQHandler(void)
{
    nestedTim2 ++;
    if(nestedTim2 > 1){
        nestCntTim2++;
    }
    OSEnterISR();
    dacqTickTimeStamp = TIM5->CNT;
    
    
#if TIM2_800Hz
// timer runs at 800 Hz
    N_per = N_per + 1;
    if( N_per >= 800 ) {
        N_per = 0;
    }

    // New sampling method: 800 Hz Accel/800 Hz Rate-Sensor
    TIM2_Cntr++;
    if( TIM2_Cntr > TIM2_CntrLimit ) {
        TIM2_Cntr = 0;

        // Upon TIM2 timeout, signal taskDataAcquisition() to continue
        osSemaphoreRelease(dataAcqSem);
#if defined(DBC_FILE) || defined(SAE_J1939)
        // Upon TIM2 timeout, signal taskDataCANComminication() to continue
        osSemaphoreRelease(canDataSem);
#endif
    }

#else
    // timer runs at 200 Hz
    // Upon TIM2 timeout, signal taskDataAcquisition() to continue
    osSemaphoreRelease(dataAcqSem);
#if defined(DBC_FILE) || defined(SAE_J1939)
        // Upon TIM2 timeout, signal taskDataCANComminication() to continue
        osSemaphoreRelease(canDataSem);
#endif
#endif

    // reset the interrupt flag
    TIM2->SR = (uint16_t)~TIM_IT_Update;

    OSExitISR();
    nestedTim2--;
}

#define  NUMBER_OF_FREQ_MATCHES     5
#define  EXT_CLK_TEST_PRECISION     0.0001 //  (1/100 of %)

static uint8_t   TIM5_Cntr      = 0;
static uint8_t   TIM5_CntrLimit = 4;  // 200 Hz Sampling
static uint32_t  ref_min, ref_max;
typedef struct{
    uint32_t ppsTS;
    uint32_t dacqTs;
    int      delta;
}pll_debug_t;

pll_debug_t pllDbg[32];
int pllDbgIdx = 0;

#define  TS_FILT_SIZE 8
uint32_t ppsFiltr[TS_FILT_SIZE] = {0,0,0,0,0,0,0,0};
int      ppsFltIdx;
/** ***************************************************************************
 * @name TIM5_IRQHandler()
 * @brief The value of the timer that triggers the interrupt (period) is based
 *        on the input to the function 'InitDataAcquisitionTimer'. Upon TIM5
 *        interrupt, the rate sensor data buffers are reset when the function
 *        GyroStartReading() is called. The sensor data-ready interrupt handles
 *        the data transfer to the data buffers.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TIM5_IRQHandler(void)
{
    static uint8_t    freqValid = 0;
    static uint32_t   cap1 = 0, match = 0;
    static int        delta1 = 0;
    uint16_t          sr5  = TIM5->SR;
    uint32_t          cap  = TIM5->CCR1;

    OSEnterISR();
 
    if(sr5 & TIM_IT_CC1){
        while(1){
            if(freqValid){
                TIM5_Cntr++;   // Incremented at 1000 Hz
                if( TIM5_Cntr > TIM5_CntrLimit ) {
                    // This loop is done at 200 Hz
                    TIM5_Cntr = 0;
                    // signal taskDataAcquisition() to continue
                    osSemaphoreRelease(dataAcqSem);
#ifdef CAN_BUS_COMM
                    // Upon TIM5 timeout, signal taskDataCANComminication() to continue
                    osSemaphoreRelease(canDataSem);
#endif
                }
                break;
            }
            delta1 = cap - cap1;
            cap1   = cap; 
            if (delta1 < ref_min || delta1 > ref_max){
                // disregard pulse and use previous numbers; 
                match = 0;
                break;
            }
            match ++;
            if (match >= NUMBER_OF_FREQ_MATCHES) {
                freqValid  = 1;
                match      = 0;
            }
            break;
        }
    }
    
    if(sr5 & TIM_IT_Update){
// do something
    }

    // reset the interrupt flag
    TIM5->SR &= ~sr5;

    OSExitISR();
}



/** ***************************************************************************
 * @name InitClockMeasurementTimer() Set up and initialize the timer interrupt
 *       that drives data acquisition.
 *        Using 32 bit TIM2 measure period of input clock signal on 1PPS pin
 *        1000 or 1 Hz
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void InitClockMeasurementTimer(uint32_t freq)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef       TIM_ICInitStruct;
    NVIC_InitTypeDef        NVIC_InitStructure;

    if(freq != 1000){
        // so far just 1000 Hz supported 
        return;
    }

    ref_min = (uint32_t)( ((float)SystemCoreClock/(freq*2))  * (1.0 - EXT_CLK_TEST_PRECISION));
    ref_max = (uint32_t)( ((float)SystemCoreClock/(freq*2))  * (1.0 + EXT_CLK_TEST_PRECISION));

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );
    TIM_Cmd(TIM5, DISABLE);                       ///< TIM disable counter
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);   ///< Clear interrupt

    /// Time base configuration
    TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure );   // init with default values - max clock
    TIM_ARRPreloadConfig( TIM5, ENABLE );

    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInit(TIM5, &TIM_ICInitStruct);                // init with defaults - on rising edge
    
//    Enable the TIM5 global interrupt, TIM5_IRQn
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
//    TIM_ITConfig( TIM5, TIM_IT_Update, ENABLE ); ///< TIM Interrupts enable
//    TIM_ITConfig( TIM5, TIM_IT_Update | TIM_IT_CC1, ENABLE ); ///< TIM Interrupts enable
    TIM_ITConfig( TIM5, TIM_IT_CC1, ENABLE ); ///< TIM Interrupts enable
//    Ensure an UEV generates an interrupt by deselecting the URS bit in TIM5_CR1
    TIM5->CR1 &= (uint16_t)~TIM_CR1_URS;
    TIM_Cmd(TIM5, ENABLE ); ///< TIM enable counter
}


/** ***************************************************************************
 * @name DataAquisitionStart() API
 * @brief used in commands.c to test sensors
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DataAquisitionStart(void)
{
    // Configure and enable timers
    InitClockMeasurementTimer(1000);
    InitDataAcquisitionTimer(800);

     /// Enable external sync 1 PPS A0 interrupt
    if(!BoardIsTestMode()){
    _InitExternalSync( ENABLE );
    }

    ActivateSensors();

    /// Select UART or SPI (if DR pin is pulled LOW then select UART, else SPI).
    if(platformGetUnitCommunicationType() != SPI_COMM ) {
        uart_init(userSerialChan, platformGetBaudRate());
    } 
    else { /// SPI
#ifdef SPI_BUS_COMM
        BoardConfigureDataReadyPinForSpi();
        InitCommunication_UserSPI();
#endif
    }
}


void TaskDataAcquisition_Init(void)
{

    BITInit(); // BIT.c

    gBitStatus.hwStatus.bit.unlockedEEPROM = !eepromLocked();

    /// reset upon entry into main-loop and while doing EEPROM reads/writes.
    InitTimer_Watchdog( DISABLE ); // FIXME turned off for now

    // Initialize the timing variables and set the odr based on the system type
    Initialize_Timing();
    
    // initialize sensors
    InitSensors();      

}


uint32_t imuCounter = 0;

void PrepareToNewDacqTick()
{
    static BOOL firstTime = TRUE;

    imuCounter += 5;   // miliseconds considering 200Hz tick 

    if(BoardIsTestMode()){
    if(firstTime){
        BoardInitPinsForTestMode();
        firstTime = FALSE;
    }else{
        BoardPerformSelfTest();
    }
    }

      // Increment once TMR5 triggers
      TimingVars_Increment();
      // state machine to configure, restore and analyze the Self Test results
      BITStartStop();
      // kick the watchdog timer
      PetWatchdog(); 
      //
}

void PrepareToNewDacqTickAndProcessUartMessages()
{
    // Process commands and  output continuous packets to UART
    // Processing of user commands always goes first
    ProcessUserCommands ();
    SendContinuousPacket(200);
    PrepareToNewDacqTick();
}

uint32_t   platformGetIMUCounter()
{
    return imuCounter;
}


/** ***************************************************************************
 * @name DataAquisitionStop() API
 * @brief Called from user command interface. Once running, the data acquisition
 *   process should not stop (unless commanded).
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DataAquisitionStop(void)
{
    TIM_Cmd(TIM2, DISABLE);
}



/** ***************************************************************************
 * @name _InitExternalSync() LOCAL sets up the sync timer
 * @brief  perform input capture on a clock signal using TIM2
 *
 * @param [in] NewState - interupt enable or disable
 * @retval N/A
 ******************************************************************************/
void _InitExternalSync( FunctionalState NewState )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /// Enable the peripheral clocks
    RCC_AHB1PeriphClockCmd( ONE_PPS_CLK, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE ); // for interrupts

    /// Configure the one-PPS pin (A0) as input
    GPIO_StructInit( &GPIO_InitStructure );
    GPIO_InitStructure.GPIO_Pin  = ONE_PPS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init( ONE_PPS_PORT, &GPIO_InitStructure );
    GPIO_PinAFConfig(ONE_PPS_PORT, ONE_PPS_SOURCE, GPIO_AF_TIM5);

    /// Configure EXTI line
    EXTI_StructInit( &EXTI_InitStructure );
    EXTI_InitStructure.EXTI_Line    = ONE_PPS_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = NewState;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init( &EXTI_InitStructure );

    gBitStatus.hwStatus.bit.unlocked1PPS = TRUE;

    /// Connect EXTI Line to GPIO Pin
    SYSCFG_EXTILineConfig( ONE_EXTI_PORT_SOURCE, ONE_PPS_EXTI_PIN_SOURCE );

    /// Enable and set EXTI Interrupt to the highest priority
    NVIC_InitStructure.NVIC_IRQChannel                   = ONE_PPS_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = NewState;
    NVIC_Init( &NVIC_InitStructure );
}



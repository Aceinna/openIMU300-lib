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
BOOL dacqInitialized = FALSE;


typedef struct {
    union {
       uint64_t time;
       struct {
           uint32_t timeLo;
           uint32_t timeHi;
       };
    };
}stime_t;

static uint32_t prevTim5Val    = 0;
static uint64_t ppsTstamp      = 0;
static int64_t  ppsTstampFull      = 0;
static int64_t  dacqTstampFull     = 0;
static uint64_t solutionTstamp = 0;
static int64_t  solutionTstampFull    = 0;
static int64_t  solutionPpsTstampFull  = 0;
static uint64_t iTowTstamp     = 0;
static uint64_t prevItow       = 0;
static uint64_t iTow = 0;
static uint32_t numTicksInPps         = 59947833;
static BOOL     iTowUpdated           = FALSE;
stime_t tStamp;
static  uint8_t  ppsDetected           =  0; // 0 not synced, 1 synced
static  uint8_t  solutionPpsDetected   =  0; // 0 not synced, 1 synced
static uint32_t  gpsItow;
// value in microseconds
uint64_t platformGetCurrTimeStamp()
{
    OSDisableHook();
    uint32_t cur =  TIM5->CNT;
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
    uint32_t cur =  TIM5->CNT;
    if(cur < prevTim5Val){
        tStamp.timeHi++;
    }
    tStamp.timeLo = cur; 
    prevTim5Val   = cur;
    return tStamp.time / 60;
}

uint64_t platformGetFullTimeStampFromIsr()
{
    uint32_t cur =  TIM5->CNT;
    if(cur < prevTim5Val){
        tStamp.timeHi++;
    }
    tStamp.timeLo = cur; 
    prevTim5Val   = cur;
    return tStamp.time / 60;
}


uint64_t platformGetCurrTicksFromIsr()
{
    uint32_t cur =  TIM5->CNT;
    if(cur < prevTim5Val){
        tStamp.timeHi++;
    }
    tStamp.timeLo = cur; 
    prevTim5Val   = cur;
    return tStamp.time;
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

void  platformSetPpsTimeStamp(uint32_t tmr)
{
    if(tmr < prevTim5Val){
        tStamp.timeHi++;
    }
    tStamp.timeLo = tmr; 
    prevTim5Val   = tmr;
    ppsTstamp     = tStamp.time / 60;
    if(prevItow){
        iTow     = prevItow;
        prevItow = 0;
    }
    ppsTstampFull = tStamp.time;
    iTow += 1000000;
    ppsDetected  = 1;
//    if(ppsTstampFull > solutionTstampFull && (ppsTstampFull - solutionTstampFull) < 600){   // 10 uS
//        solutionPpsTstampFull = ppsTstampFull; 
//        solutionPpsDetected   = TRUE;
//    }
}


BOOL platformGetPpsFlag( bool fClear) { 
    BOOL detected;
    OSDisableHook();
    detected    = solutionPpsDetected;
    if(fClear){
        solutionPpsDetected = FALSE;
    }
    OSEnableHook();
    return detected; 
}


int    updErrCnt = 0;

void    platformUpdateITOW(uint32_t itow)
{
    OSDisableHook();

    uint64_t tStamp        = platformGetCurrTicksFromIsr();
    int64_t  ddd           = (tStamp - ppsTstampFull); 
    double   numTicksPerUs = (double)numTicksInPps/1000000;
    double   offset        = (double)ddd/numTicksPerUs;

    if(offset < 900000.0){
        iTowUpdated  = TRUE;
    prevItow = (uint64_t)itow * 1000;     // microsecond resolution
        gpsItow      = itow;
    }else{
        updErrCnt++;
        updErrCnt &= 0x0f;
        prevItow = 0;
    }

    OSEnableHook();

}

uint64_t platformGetEstimatedITOW()
{
    OSDisableHook();
    uint64_t tStamp = platformGetCurrTimeStampFromIsr();
    uint64_t ddd;; 

    if(!ppsTstamp){
        ddd = tStamp - iTowTstamp; 
    }else{
        ddd = tStamp - ppsTstamp; 
    }
    OSEnableHook();

    return iTow + ddd;
}

uint32_t platformGetItow(BOOL *detected, BOOL *updated)
{
    OSDisableHook();
    uint32_t res         = iTow/1000;
    *detected            = solutionPpsDetected;
    *updated             = iTowUpdated;
    iTowUpdated          = 0;
    OSEnableHook();
    return res;
}

uint32_t platformGetGpsItow()
{
    return gpsItow;
}


uint64_t platformGetEstimatedITOWFromIsr()
{
    uint64_t tStamp = platformGetCurrTimeStampFromIsr();
    uint64_t ddd; 

    if(!ppsTstamp){
        ddd = tStamp - iTowTstamp; 
    }else{
        ddd = tStamp - ppsTstamp; 
    }
    
    return iTow + ddd;
}


int      itowErrCnt = 0; 
uint64_t lastItow   = 0;

double platformGetSolutionTstampAsDouble()
{
    if(ppsTstampFull == 0){
        return 0;
    }
    double res;
    OSDisableHook();
    int64_t  ddd = solutionTstampFull - solutionPpsTstampFull;
    double   numTicksPerUs = (double)numTicksInPps/1000000;
    double   offset = (double)ddd/numTicksPerUs;
    if((offset > 1006000.0 || offset < 0 )&& numTicksPerUs != 0 && iTow != lastItow){
        itowErrCnt++;
    }
    lastItow = iTow;
    res      = (double)iTow + offset; 
    OSEnableHook();
    return res;
}

BOOL   syncP  = FALSE;
BOOL   resync = FALSE;
int    rem    = 17;
int    divv   = 299739;
int    divva  = 0;
int    ratioP = 0, ratioM = 0, adjStep = 0, adj1 = 0;
BOOL   dir;

void adjustDacqSyncPhase()
{
    int ddd;

    // no valid sync pulse detected
    // keep old settings

    if(!syncP){
        return;
    }

    if(resync){
        resync  = FALSE;
        ratioP  = rem;
        ratioM  = 200 - ratioP;
        dir     = 1;
        divva   = divv; 
        adj1    = adjStep;
        adjStep = 0;
    }
    
    if(ratioP && dir){
        ddd  = divva + adj1;
        if(ratioM){
            dir ^= 1;
        }
        TIM_SetAutoreload(TIM2, ddd);
        ratioP -= 1;
    }

    if(!dir && ratioM){
        if(ratioP){
            dir ^= 1;
        }
        ratioM -= 1; 
        TIM_SetAutoreload(TIM2, divva-1);
    }
}


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
//         ppsDetected  = 1;
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
    numTicksInPps =  SystemCoreClock/2;                

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


int deltaBuf[256] = {0};
int deltaIdx = 0;


void calculateSyncPhaseShift()
{
    int64_t        delta;
    static int     lock1 = 0, lock2 = 0, lock3 = 0;

    if(!ppsDetected){
        return; 
    }

    delta = solutionTstampFull - solutionPpsTstampFull; 
    deltaBuf[deltaIdx++] = delta;
    deltaIdx &= 0xff;

    if(!lock1){
        if(delta < 10000){
            // try to properly position phase shift
            adjStep = 50;
    }else{
        lock1    = 1;
            adjStep = -20;
    }
    }else if(!lock2){
        if(delta > 6000){
           adjStep = -20;
        }else {
            lock2   = 1;
            adjStep = -10;
        }
    }else if(!lock3){
        if(delta > 3000){
           adjStep = -5;
        }else {
            lock3   = 1;
        adjStep = 0;
    }
    }else {
        if(delta > 2400){
            rem -= 3;
    }else{
            rem += 3;
        }
    }

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

//double         cntPeriod800;
uint32_t    syncPulsePeriod = 0;


uint32_t syncOffset  = 0;//, tmr;
int  nestedTim2  = 0;
int  nestCntTim2 = 0;
int  nestedTim5  = 0;
int  nestCntTim5 = 0;
int  numDacqCycles   = 0;  
void TIM2_IRQHandler(void)
{
    nestedTim2 ++;
    if(nestedTim2 > 1){
        nestCntTim2++;
    }

    OSEnterISR();
    syncOffset = 0;
    

    if(!ppsTstamp){
        iTow          += 5000;    // in ms
        iTowTstamp     = platformGetCurrTimeStampFromIsr();
        solutionTstamp = iTow;
    }else{
        solutionTstamp = platformGetEstimatedITOWFromIsr();
    }

    solutionTstampFull   = tStamp.time;
    dacqTstampFull       = tStamp.time;

    if(platformIsGpsPPSUsed() && syncP){
        if(ppsDetected){
            solutionPpsTstampFull   = ppsTstampFull;
            solutionPpsDetected     = ppsDetected;
            calculateSyncPhaseShift();
            resync = 1; 
            numDacqCycles = 0;
        }else{
            numDacqCycles++;
            numDacqCycles %= 200;
            if(numDacqCycles == 0){
                resync = 1;
            }
        }
    adjustDacqSyncPhase();
        ppsDetected = FALSE;
    }
    
//  timer runs at 200 Hz
   N_per = N_per + 4;
    if( N_per >= 800 ) {
        N_per = 0;
    }

        // Upon TIM2 timeout, signal taskDataAcquisition() to continue
        osSemaphoreRelease(dataAcqSem);
#if defined(CAN_BUS_COMM)
        // Upon TIM2 timeout, signal taskDataCANComminication() to continue
        if(dacqInitialized){
        osSemaphoreRelease(canDataSem);
        }
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

#define  TS_FILT_SIZE 8
uint32_t ppsFiltr[TS_FILT_SIZE] = {0,0,0,0,0,0,0,0};
int      syncFltIdx;
int      syncFreq    = 0;
int      missedPulseCnt = 0;
int      missedPulseIdx = 0;
uint32_t missedPulseTab[1024] = {0};




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
    static uint32_t   cap2, cnt = 0;
    static uint32_t   syncPeriod = 0, syncAvg = 0;
    static int        delta, err = 0, syncCnt = 0;
    static uint16_t   sr5;
    static uint32_t   ts;
    static uint32_t   cap;


    nestedTim5 ++;

    if(nestedTim5 > 1){
        nestCntTim5++;
    }

    OSEnterISR();
 
    sr5  = TIM5->SR;

    if(sr5 & TIM_IT_CC1){
        //    ts   = TIM5->CNT; 
        cap  = TIM5->CCR1;
        ts   = cap; 
        while(1){
            if(freqValid){
            // Here sync achieved - keep synced
                if(syncFreq == 1000){
                TIM5_Cntr++;   // Incremented at 1000 Hz
                if( TIM5_Cntr > TIM5_CntrLimit ) {
                    // This loop is done at 200 Hz
                    TIM5_Cntr = 0;
                        //  timer runs at 200 Hz
                        N_per = N_per + 4;
                        if( N_per >= 800 ) {
                            N_per = 0;
                        }
                    // signal taskDataAcquisition() to continue
                    osSemaphoreRelease(dataAcqSem);
#if defined (CAN_BUS_COMM)
                        // Upon TIM2 timeout, signal taskDataCANComminication() to continue
                    if(dacqInitialized){
                    osSemaphoreRelease(canDataSem);
                    }
#endif
                }
                break;
            }
                // delta between two periods of sync signal 
            delta1 = cap - cap1;
            cap1   = cap; 
            if (delta1 < ref_min || delta1 > ref_max){
                // disregard pulse and use previous numbers; 
                    missedPulseCnt++;
                    missedPulseTab[missedPulseIdx++] = delta1;
//                    DEBUG_INT("Missed sync\n", missedPulseIdx);
                    missedPulseIdx &= 0x3ff;
                    break;
                }
                platformSetPpsTimeStamp(ts);
                syncAvg             -= ppsFiltr[syncFltIdx];
                ppsFiltr[syncFltIdx] = delta1;
                syncFltIdx++;
                syncFltIdx          &= 0x7;
                syncAvg             += delta1;
                if(syncCnt < TS_FILT_SIZE){
                    syncCnt ++;
                    syncPeriod = delta1;
                }else {
                    syncPeriod = syncAvg >> 3;    // divide by 8
                }
                numTicksInPps = syncPeriod; 
                divv   = syncPeriod/200;
                rem    = syncPeriod%200;
                syncP  = TRUE; 
                resync = 1;
                cnt ++;
//                DEBUG_UINT("\r\n", cnt);
//                DEBUG_UINT(" ,"  , cap);
//                DEBUG_INT(" ,"  ,  syncPeriod);
//                DEBUG_INT(" ,"   , delta);
//                DEBUG_INT(" ,"   , refDelta);
//                DEBUG_INT(" , "  , divv);
//                DEBUG_INT(" , "  , rem);
                break;
            }
// Here sync not achieved yet
            cap2               = cap1; 
            cap1               = cap;
            delta              = cap1 - cap2;
            if(delta < 0){
                err++;
            }
            if (delta > ref_min && delta < ref_max){
                syncPulsePeriod = delta;
                match++;
                syncOffset++;
            }else{
                match = 0;
            }
            if (match >= NUMBER_OF_FREQ_MATCHES) {
                freqValid  = 1;
                match      = 0;
                if(syncFreq == 1000){
                    TIM_Cmd(TIM2, DISABLE);  ///< TIM2 disable, start to sync from here
                    uint16_t sr2  = TIM2->SR;
                    // reset the interrupt flag
                    TIM2->SR &= ~sr2;
                    TIM5_Cntr = syncOffset;    
                }
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
    nestedTim5--;
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


    if(platformIsGpsPPSUsed()){
        freq     = 1;       // 1Hz sync
        syncFreq = 1;
    }else{
        freq = 1000;    // 1000 Hz sync
        syncFreq = 1000;
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
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
    InitClockMeasurementTimer(1);
    InitDataAcquisitionTimer(200);

     /// Enable external sync 1 PPS A0 interrupt
    if(!BoardIsTestMode()){
    _InitExternalSync( ENABLE );
    }

    ActivateSensors();

    /// Select UART or SPI (if DR pin is pulled LOW then select UART, else SPI).
    if(platformGetUnitCommunicationType() != SPI_COMM ) {
        uart_init(userSerialChan, platformGetBaudRate(0xFFFF));
    } 
    else { /// SPI
#ifdef SPI_BUS_COMM
        BoardConfigureDataReadyPinForSpi();
        InitCommunication_UserSPI();
#endif
    }

    dacqInitialized = TRUE;
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



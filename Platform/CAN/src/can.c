/** ***************************************************************************
 * @file can.c the definitions of basic functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#if defined(DBC_FILE) || defined(SAE_J1939)
#include <stdio.h>
#include <stdlib.h>

#include "sae_j1939.h"
#include "GlobalConstants.h"
#include "can.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "boardDefinition.h"
#include "osapi.h"
#include "UserConfiguration.h"
#include "canAPI.h"

void (*gCANTxCompleteCallback)(void) = NULL;    // callback function pointer of CAN transmit 
void (*gCANRxCompleteCallback)(void) = NULL;    // callback function pointer of CAN receive

int     gCANSleep = FALSE;                      // CAN interface sleep variable
uint32_t canRxIntCounter = 0;                   // counter of CAN receiver
uint32_t canStartDetectRxIntCounter = 0;        // counter of CAN receiver for auto detection
CAN_FilterInitTypeDef FILTER_InitStructure;
int filterNum = 1;

/** ***************************************************************************
 * @name get_can_sleep() check CAN interface sleep
 * @brief  perform an API of checking CAN active or not
 *
 * @param [in] CANx, where x can be 1 or 2 to select the CAN peripheral.
 * @retval TRUE or FAlSE
 ******************************************************************************/
uint8_t get_can_sleep(CAN_TypeDef* CANx) 
{ 
      return gCANSleep; 
}

/** ***************************************************************************
 * @name sleep_can() CAN interface goes to sleep
 * @brief Enter sleep mode
 *
 * @param [in] CANx, where x can be 1 or 2 to select the CAN peripheral.
 * @retval 1 sleep or 0 fails
 ******************************************************************************/
uint8_t sleep_can(CAN_TypeDef* CANx) 
{
  gCANSleep = TRUE;
  
  return CAN_Sleep(CANx);
}

/** ***************************************************************************
 * @name wake_up_can() CAN interface wakes up
 * @brief Wakes up the CAN peripheral from sleep mode .
 *
 * @param [in] CANx, where x can be 1 or 2 to select the CAN peripheral.
 * @retval 1 leaves sleep mode or 0 fails
 ******************************************************************************/
uint8_t wake_up_can(CAN_TypeDef* CANx) 
{
  return CAN_WakeUp(CANx);
}

/** ***************************************************************************
 * @name CAN_Set_BTR() Set up bit timing
 * @brief Provides a function to set bit timing according to the expected baud rate
 *
 * @param [in] CAN_InitStructure CAN init structure definition
 *             br, baud rate
 * @retval N/A
 ******************************************************************************/
void CAN_Set_BTR(_ECU_BAUD_RATE br, CAN_InitTypeDef* CAN_InitStructure)
{
  switch (br) {
      // 500Kbps, time quantum 10, segment1 4, segment2 1.
      case _ECU_500K:
        CAN_InitStructure->CAN_Prescaler = 6;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      // 250Kbps, time quantum 24, segment1 3, segment2 1.
      case _ECU_250K:
        CAN_InitStructure->CAN_Prescaler = 12;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      // 125Kbps, time quantum 24, segment1 7, segment2 2.
      case _ECU_125K:
        CAN_InitStructure->CAN_Prescaler = 24;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      // 1000Kbps, time quantum 2, segment1 7, segment2 2.
      case _ECU_1000K:
        CAN_InitStructure->CAN_Prescaler = 3;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      default:
        break;
  }
  
  return;
}

/** ***************************************************************************
 * @name _CAN_Init() CAN interface initializes
 * @brief Provides a function of CAN's initialization
 *
 * @param [in] CANx, where x can be 1 or 2 to select the CAN peripheral.
 *             mode, normal mode
 * @retval CAN_NO_ERROR
 ******************************************************************************/
uint8_t _CAN_Init(CAN_TypeDef* CANx, uint8_t mode, int baudRate)
{
    CAN_InitTypeDef  CAN_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
        
    CAN_DeInit(CANx);
    
    CAN_StructInit(&CAN_InitStructure);
        
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = mode;
   
    
    CAN_InitStructure.CAN_SJW = 0;
    // set to default value, 250kbps, if out of supported range
    if ( baudRate > _ECU_1000K)
        CAN_Set_BTR(_ECU_250K, &CAN_InitStructure);
    else
        CAN_Set_BTR(baudRate, &CAN_InitStructure);

    // Initialize CAN1 of Version 3rd PCB and CAN2 of version 1st PCB  
    if (CANx == CAN1) {
        // initialize CAN1 clock
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); 
    
        // initialize CAN1 tx/rx clock
        RCC_AHB1PeriphClockCmd(CAN1_RX_GPIO_CLK, ENABLE);
        RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK, ENABLE);
    
        // configure CAN1 tx/rx pins
        GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_RX_SOURCE, GPIO_AF_CAN1);
        GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_TX_SOURCE, GPIO_AF_CAN1);
    
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
        GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
        GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);
                   
        CAN_Init(CAN1, &CAN_InitStructure);
#ifndef FL        
         /// Enable the CAN1 global interrupt, CAN1_TX_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_TX_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x9;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
         
         /// Enable the CAN1 global interrupt, CAN1_RX0_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xa;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
         
         // Enable the CAN1 global interrupt, CAN2_RX1_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX1_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xa;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x2;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
#endif        
        
    } else {
        // initialize CAN1 and CAN2 clocks
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); 
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); 
    
        // initialize CAN2 tx/rx clocks
        RCC_AHB1PeriphClockCmd(CAN2_RX_GPIO_CLK, ENABLE);
        RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK, ENABLE);
    
        // initialize CAN2 tx/rx pins
        GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_RX_SOURCE, GPIO_AF_CAN2);
        GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_TX_SOURCE, GPIO_AF_CAN2);
    
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN;
        GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
        GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
        
        CAN_Init(CAN2, &CAN_InitStructure);
#ifdef FL        
         /// Enable the CAN2 global interrupt, CAN2_TX_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_TX_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x9;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
         
         /// Enable the CAN2 global interrupt, CAN2_RX0_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xa;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
         NVIC_Init( &NVIC_InitStructure );
#endif
    }
    
    CAN_DBGFreeze(CANx, DISABLE);
    
    return CAN_NO_ERROR;
}

/** ***************************************************************************
 * @name_CAN_Init_Filter_Engine () CAN's filter initialization
 * @brief Performs the filter of CAN interface 
 *
 * @param [in]
 *         
 * @retval
 ******************************************************************************/
void _CAN_Init_Filter_Config_Structure(void)
{
  // initialize common parameters
  FILTER_InitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  FILTER_InitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
  FILTER_InitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
  FILTER_InitStructure.CAN_FilterActivation     = ENABLE;
}

void ConfigureCANMessageFilter(uint32_t baseID, uint32_t baseMask)
{
    uint32_t mask;
    uint32_t filtr;
  // initialize filter ECU ID
    filtr = (baseID   << 3 ) | (USER_CAN_IDE << 2) | (USER_CAN_RTR << 1);
    mask  = (baseMask << 3 ) | (USER_CAN_IDE << 2) | (USER_CAN_RTR << 1);
  FILTER_InitStructure.CAN_FilterIdHigh     =  filtr >> 16;
  FILTER_InitStructure.CAN_FilterIdLow      =  filtr;
  FILTER_InitStructure.CAN_FilterMaskIdHigh =  mask >> 16;
  FILTER_InitStructure.CAN_FilterMaskIdLow  =  mask;
    FILTER_InitStructure.CAN_FilterNumber     =  filterNum++;
  CAN_FilterInit(&FILTER_InitStructure);

}

void _CAN_Activate_Filters()
{
  CAN_SlaveStartBank(28);
}


/** ***************************************************************************
 * @name _CAN_Configure() assign callback funtion's pointers
 * @brief Performs tx/rx callback functions 
 *
 * @param [in] callback1 -- transmit function
 *             callback2 -- receive function
 * @retval N/A
 ******************************************************************************/
void _CAN_Configure(void (*callback1)(void), void(*callback2)(void))
{
   gCANTxCompleteCallback = callback1;
   gCANRxCompleteCallback = callback2;
   
   return;
}


/** ***************************************************************************
 * @name _CAN_Init_IT() interrupt initialization
 * @brief Set interrupts' bits on CAN interface
 *
 * @param [in] CANx, where x can be 1 or 2 to select the CAN peripheral.
 * @retval N/A
 ******************************************************************************/
void _CAN_Init_IT(CAN_TypeDef* CANx)
{
  uint32_t int_bits = 0;
  
  int_bits = CAN_IT_TME | CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1;
  
  CAN_ITConfig(CANx, int_bits, ENABLE);
  
  return;
}

/** ***************************************************************************
 * @name _CAN_IT_Enable() interrupt enable function
 * @brief Turn on the interrupt of CAN interface
 *
 * @param [in] CANx, where x can be 1 or 2 to select the CAN peripheral.
 *             in_bits, indicates interrupt's bits
 * @retval N/A
 ******************************************************************************/
void _CAN_IT_Enable(CAN_TypeDef* CANx, uint32_t int_bits)
{
   CAN_ITConfig(CANx, int_bits, ENABLE);
  
  return;
}

/** ***************************************************************************
 * @name _CAN_IT_Disable() interrupt disable function
 * @brief Turn off the interrupt of CAN interface
 *
 * @param [in] CANx, where x can be 1 or 2 to select the CAN peripheral.
 *             in_bits, indicates interrupt's bits
 * @retval N/A
 ******************************************************************************/

void _CAN_IT_Disable(CAN_TypeDef* CANx, uint32_t int_bits)
{
  CAN_ITConfig(CANx, int_bits, DISABLE);
  
  return;
}

/** ***************************************************************************
 * @name CAN1_TX_IRQHandler() CAN1 transmit IRQ handler
 * @brief Handle transmitting interrupt of CAN1
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
void CAN1_TX_IRQHandler(void)
{
  ITStatus ItRslt;
  
  OSEnterISR();
  
  ItRslt = CAN_GetITStatus(CAN1, CAN_IT_TME);
  
  if(ItRslt == SET) {
    gCANTxCompleteCallback();
    CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
  }
  
  OSExitISR();
  
  return;
}

/** ***************************************************************************
 * @name CAN1_RX0_IRQHandler CAN1 receive buffer 0 IRQ handler
 * @brief Handle receiving interrupt of CAN1 buffer 0
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
#ifdef SAE_J1939
  ITStatus ItRslt;
  uint8_t fifoPending;
    
	OSEnterISR();
  
  canRxIntCounter++;
  ItRslt =  CAN_GetITStatus(CAN1, CAN_IT_FMP0);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN1, 0);
    do {
        CAN_Receive(CAN1, 0, &(gEcuInst.curr_rx_desc->rx_buffer));
        gCANRxCompleteCallback();
        CAN_FIFORelease(CAN1, 0);
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
  }
  
	OSExitISR();
#endif
  return;
}

/** ***************************************************************************
 * @name CAN1_RX1_IRQHandler() CAN1 receive buffer 1 IRQ handler
 * @brief Handle receiving interrupt of CAN1 buffer 1
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
  
void CAN1_RX1_IRQHandler(void)
{
#ifdef SAE_J1939
  ITStatus ItRslt;
  uint8_t fifoPending;
  
	OSEnterISR();
  
  ItRslt =  CAN_GetITStatus(CAN1, CAN_IT_FMP1);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN1, 1);
    do {
        CAN_Receive(CAN1, 1, &(gEcuInst.curr_rx_desc->rx_buffer));
        CAN_FIFORelease(CAN1, 1);
        gCANRxCompleteCallback();
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF1);
  }
  
	OSExitISR();
#endif
  
  return;
}

/** ***************************************************************************
 * @name CAN2_TX_IRQHandler() CAN2 transmit IRQ handler
 * @brief Handle transmitting interrupt of CAN2
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
void CAN2_TX_IRQHandler(void)
{
#ifdef FL
  ITStatus ItRslt;
  
  OSDisableHook();
  
  ItRslt = CAN_GetITStatus(CAN2, CAN_IT_TME);
  
  if(ItRslt == SET) {
    gCANTxCompleteCallback();
    CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
  }
  
  OSEnableHook(); 
#endif
  return;
}

/** ***************************************************************************
 * @name CAN2_RX0_IRQHandler() CAN2 receive buffer 0 IRQ handler
 * @brief Handle receiving interrupt of CAN1 buffer 0
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
#ifdef FL
  ITStatus ItRslt;
  uint8_t fifoPending;
    
  OSDisableHook();
  
  ItRslt =  CAN_GetITStatus(CAN2, CAN_IT_FMP0);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN2, 0);
    do {
        CAN_Receive(CAN2, 0, &(gEcuInst.curr_rx_desc->rx_buffer));
        CAN_FIFORelease(CAN2, 0);
        gCANRxCompleteCallback();
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
  }
  
  OSEnableHook(); 
#endif
  
  return;
}

/** ***************************************************************************
 * @name CAN2_RX1_IRQHandler() CAN2 receive buffer 1 IRQ handler
 * @brief Handle receiving interrupt of CAN2 buffer 1
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
  
void CAN2_RX1_IRQHandler(void)
{
#ifdef FL
  ITStatus ItRslt;
  uint8_t fifoPending;
  
  OSDisableHook();
  
  ItRslt =  CAN_GetITStatus(CAN2, CAN_IT_FMP1);
  if (ItRslt == SET) {
    fifoPending = CAN_MessagePending(CAN2, 1);
    do {
        CAN_Receive(CAN2, 1, &(gEcuInst.curr_rx_desc->rx_buffer));
        CAN_FIFORelease(CAN2, 1);
        gCANRxCompleteCallback();
        fifoPending--;
    } while (fifoPending > 0);
    
    CAN_ClearITPendingBit(CAN2, CAN_IT_FF1);
  }
  
  OSEnableHook(); 
#endif  
  return;
}


/** ***************************************************************************
 * @name InitCommunication_UserCAN() user interface, CAN, initialization 
 * @brief Provide an API of CAN initialization to system
 *        initialize CAN1 of version 3rd PCB
 * @param [in]  
 * @retval N/A
 ******************************************************************************/
void InitCommunication_UserCAN(int baudRate)
{
  
  _CAN_Init(CAN1, CAN_Mode_Normal, baudRate); 
  
    _CAN_Init_Filter_Config_Structure();
    
    // configure CAN controller for selective reception of CAN messages
    ConfigureCANMessageFilters();

    _CAN_Activate_Filters();
  
  _CAN_Init_IT(CAN1);
 
  
  return;
}


/** ***************************************************************************
 * @name CAN_Detect_Baudrate() Baud rate auto detection
 * @brief Traverse all of supported channels and find out the baud rate used 
 *        by host CAN device
 * @param [in]  CANx, where x can be 1 or 2 to select the CAN peripheral.
 *              rate, 500kbps, 250kbps or 125kbps
 * @retval _ECU_500K, _ECU_250K or _ECU_125K
 ******************************************************************************/
_ECU_BAUD_RATE _Detect_Baudrate(CAN_TypeDef* CANx, _ECU_BAUD_RATE rate)
{
   CAN_InitTypeDef  CAN_InitStructure;
   _ECU_BAUD_RATE found_rate;
     
  if (CANx != CAN1) 
    return _ECU_1000K;
  
  
  if (rate == _ECU_1000K)
      return rate;
  
  CAN_StructInit(&CAN_InitStructure);
        
  CAN_InitStructure.CAN_ABOM = ENABLE;
  CAN_InitStructure.CAN_AWUM = ENABLE;
  CAN_InitStructure.CAN_TXFP = ENABLE;
  CAN_InitStructure.CAN_SJW = 0;
  CAN_Set_BTR(rate, &CAN_InitStructure);
   
  CAN_Init(CAN1, &CAN_InitStructure);
    
  OS_Delay(CAN_DETECT_TIME);
    
  if (canRxIntCounter > canStartDetectRxIntCounter + 1) {
      found_rate = rate;
      return found_rate;
  }
   
  return _ECU_1000K;
}

BOOL CAN_Detect_Baudrate(_ECU_BAUD_RATE *rate)
{
    CAN_InitTypeDef  CAN_InitStructure;
    static int8_t retry_time = CAN_BAUD_RATE_RETRY; 
    int    result;
        // Auto-detection 
        if (gEcuInst.state == _ECU_BAUDRATE_DETECT) {
            canStartDetectRxIntCounter =  canRxIntCounter;
            
            result = _Detect_Baudrate(CAN1, *rate);
            
            if (result != _ECU_1000K) {
                gEcuConfig.baudRate = result;
                
                CAN_StructInit(&CAN_InitStructure);
                
                CAN_InitStructure.CAN_ABOM = ENABLE;
                CAN_InitStructure.CAN_AWUM = ENABLE;
                CAN_InitStructure.CAN_TXFP = ENABLE;
                CAN_InitStructure.CAN_SJW = 0;
                CAN_Set_BTR(gEcuConfig.baudRate, &CAN_InitStructure);
                
                CAN_Init(CAN1, &CAN_InitStructure);
                
                SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN;
                
                SetEcuBaudRate(result);
                
            }
            else {
                if (retry_time-- > 0) {
                    (*rate)++;
                    if (*rate >= _ECU_1000K){
                        *rate = _ECU_500K;
                    }
                    return FALSE;
                }
                
                CAN_DeInit(CAN1);
                
                CAN_StructInit(&CAN_InitStructure);
                
                CAN_InitStructure.CAN_ABOM = ENABLE;
                CAN_InitStructure.CAN_AWUM = ENABLE;
                CAN_InitStructure.CAN_TXFP = ENABLE;
                CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
                
                CAN_InitStructure.CAN_SJW = 0;
                CAN_Set_BTR(gEcuConfig.baudRate, &CAN_InitStructure);
                
                CAN_Init(CAN1, &CAN_InitStructure);
            }
            
            gEcuInst.state = _ECU_CHECK_ADDRESS; 
        }
        return TRUE;
}

#endif

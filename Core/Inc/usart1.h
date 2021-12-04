/**
  ******************************************************************************
  * @file    usart1.h
  * @author  Marco, Roldan L.
  * @version v1.0
  * @date    August 14, 2021
  * @brief   USART peripheral driver
  *
  *          Device used: Bluepill (STM32F103C8)
  ******************************************************************************
  *
  * Copyright (C) 2021  Marco, Roldan L.
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.en.html.
  *
  *
  * https://github.com/rmarco30
  *
  ******************************************************************************
**/


#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"


/**
 * Configuration Guide
 *
 * Setting macro to 1 enables it, 0 otherwise
 *
 * USE_USART1_REMAP         set this to 1 to remap the USART1's TX/RX from PA9/PA10 to PB6/PB7
 * USE_USART1_TX            Master switch to enable USART1 TX, setting this to 0 disables all USART1 TX functionality
 * USE_USART1_RX            Master switch to enable USART1 RX, setting this to 0 disables all USART1 RX functionality
 * USE_USART1_DMA_TX        set this to 1 to use USART1's TX with DMA handling the data transfer, 0 uses byte by byte TX
 * USE_USART1_DMA_RX        set this to 1 to use USART1's RX with DMA handling the data transfer, 0 uses byte by byte RX with interrupt
 * USE_USART1_DMA_TX_INT    set this to 1 to use DMA related interrupts for TX
 * USE_USART1_DMA_RX_INT    set this to 1 to use DMA related interrupts for RX
 */

#define USE_USART1_REMAP                0
#define USE_USART1_TX                   1
#define USE_USART1_RX                   0


#if ( USE_USART1_TX )
    #define USE_USART1_DMA_TX           0
#endif

#if ( USE_USART1_DMA_TX )
    #define USE_USART1_DMA_TX_INT       0
#endif

#if ( USE_USART1_RX )
    #define USE_USART1_DMA_RX           0
#endif

#if ( USE_USART1_DMA_RX )
    #define USE_USART1_DMA_RX_INT       0
#endif



/* Function Prototypes */

/**
 * @brief	Initializes the USART1. See usart1.h for config options.
 * @param	none
 * @retval	none
 */
void usart1_init(void);



/**
 * @brief	Transmit data through DMA or byte by byte transfer method
 *          see usart1.h for configuration.
 * @param	none
 * @retval	none
 */
void usart1_write(char* ch);



/**
 * @brief    Receive a multibyte data through DMA.
 * @param    buffer: pointer to array where DMA will store the received data
 * @param    len: size of expected data to be received in byte. Must be <= to the size of destination array.
 * @retval   none
 */
void usart1_read(char * buffer, uint32_t len);


#endif /* __USART1_H */
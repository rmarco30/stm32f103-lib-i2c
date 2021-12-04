/**
  ******************************************************************************
  * @file    usart1.c
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


#include "usart1.h"

static void usart1_gpio(void);
static void usart1_config(void);


#if ( USE_USART1_DMA_TX || USE_USART1_DMA_RX )
    static void usart1_dma_config(void);
#endif

#if ( USE_USART1_RX )
    char usart1_rdata[10];
#endif



#if ( USE_USART1_TX )
/**
 * @brief	Transmit data through DMA or byte by byte transfer method
 * @brief   see usart1.h for configuration.
 * @param	none
 * @retval	none
 */
void usart1_write(char* ch)
{

#if USE_USART1_DMA_TX

    /* Calculate string length */
    uint8_t len;
    for( len = 0; ch[len] != '\0'; len++ );

    /* DMA1 memory address (source) */
    DMA1_Channel4->CMAR = (uint32_t)ch;

    /* Load the data length */
    DMA1_Channel4->CNDTR = (uint32_t)len;

     /* Clear TC bit */
    USART1->SR &= ~( USART_SR_TC );

    /* Enable DMA1 Channel 4 to start transfer */
    DMA1_Channel4->CCR |= DMA_CCR4_EN;

     /* Wait until Transmission is complete */
    while(  !( USART1->SR & USART_SR_TC) );

#if ( !USE_USART1_DMA_TX_INT )

    /* Disable DMA1 Channel 4 */
    DMA1_Channel4->CCR &= ~( DMA_CCR4_EN );

#endif

#endif


#if ( !USE_USART1_DMA_TX )

    /* Move data to USART1 data register byte by byte */
    for ( uint8_t i = 0; ch[i] != '\0'; i++ )
    {
        USART1->DR = ch[i];
        /* Wait until transmit data register is empty */
        while( !( USART1->SR & USART_SR_TXE ) );
    }

#endif

}
#endif



#if ( USE_USART1_DMA_TX_INT )
/**
 * @brief    DMA1 Channel4 IRQ Handler
 * @param    none
 * @retval   none
 */
void DMA1_Channel4_IRQHandler( void )
{
    /* Check if full transfer complete */
    if( ( ( DMA1->ISR >> 12 ) & 0x2 ) )
    {
        /* Transfer complete */
        /* Disable DMA1 Channel 4 */
        DMA1_Channel4->CCR &= ~( DMA_CCR4_EN );

        /* Clear channel 4 interrupts */
        DMA1->IFCR |= ( 0xf << 12 );

        /* Do interrupt routine here */
    }
    /* Check if transfer error */
    else if( ( ( DMA1->ISR >> 12 ) & 0x8 ) )
    {
        /* Transfer error occurs */
        /* Clear channel 4 interrupts */
        DMA1->IFCR |= ( 0xf << 12 );
        /* Do error handling here */
    }
}
#endif



#if ( USE_USART1_RX )

#if ( USE_USART1_DMA_RX )
/**
 * @brief    Receive a multibyte data through DMA.
 * @param    buffer: pointer to array where DMA will store the received data
 * @param    len: size of expected data to be received in byte. Must be <= to the size of destination array.
 * @retval   none
 */
void usart1_read(char * buffer, uint32_t len)
{
    /* Disable DMA before configuring */
    DMA1_Channel5->CCR &= ~( DMA_CCR1_EN );
    /* Clear DMA1 Channel 5 interrupts */
    DMA1->IFCR |= 0x00070000UL;
    /* DMA memory address for RX (destination) */
    DMA1_Channel5->CMAR = (uint32_t)buffer;
    /* Size of data to be read */
    DMA1_Channel5->CNDTR = len;
    /* Enable DMA1 Channel 5 to start reading data */
    DMA1_Channel5->CCR |= DMA_CCR1_EN;
}
#endif

#if ( !USE_USART1_DMA_RX )
/**
 * @brief    USART1 interrupt service routine
 * @param    none
 * @retval   none
 */
void USART1_IRQHandler(void)
{
    /* If a byte is received */
    if( USART1->SR & USART_SR_RXNE )
    {
        /* Implement desired functionality here */
        char *ptr = usart1_rdata;
        /* Store the data to a global array */
        *ptr = (char)USART1->DR;
        /* Echo back the received character */
        USART1->DR = *ptr;
    }
}
#endif

#endif



/**
 * @brief	Initializes the USART1. See usart1.h for config options.
 * @param	none
 * @retval	none
 */
void usart1_init(void)
{
    usart1_gpio();
    usart1_config();

#if ( USE_USART1_DMA_TX ||  USE_USART1_DMA_RX )
    usart1_dma_config();
#endif

}



/**
 * @brief	Helper function to initialize USART1's TX/RX pins.
 *          See usart1.h for config options.
 * @param	none
 * @retval	none
 */
static void usart1_gpio(void)
{

#if ( USE_USART1_REMAP )

    /* Enable required clocks */
    RCC->APB2ENR |= ( RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN );
    /* Remap USART1's TX/RX from PA9/PA10 to PB6/PB5 */
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

#if ( USE_USART1_TX )

    /* Clear required bits for TX pin */
    GPIOB->CRL &= ~( GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0 |
                     GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 );
    /* Configure PB6(TX) as Alternate function push-pull, 50 MHz */
    GPIOB->CRL |= ( GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 |
                    GPIO_CRL_CNF6_1 );

#endif

#if ( USE_USART1_RX )

    /* Clear required bits for RX pin */
    GPIOB->CRL &= ~( GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0 |
                     GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 );

    /* Configure PB7(RX) as Input with pull-up/pull-down */
    GPIOB->CRL |= GPIO_CRL_CNF7_1;

    /* Enable pull-up */
    GPIOB->BSRR |= GPIO_BSRR_BS7;

#endif

#else

    /* Enable required clocks */
    RCC->APB2ENR |= ( RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN );

#if ( USE_USART1_TX )

    /* Clear required bits for TX pin */
    GPIOA->CRH &= ~( GPIO_CRH_CNF9_1 | GPIO_CRH_CNF9_0 |
                     GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0 );

    /* Configure PA9(TX) as Alternate function push-pull, 50 MHz */
    GPIOA->CRH |= ( GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0 |
                    GPIO_CRH_CNF9_1 );

#endif

#if ( USE_USART1_RX )

    /* Clear required bits for RX pin */
    GPIOA->CRH &= ~( GPIO_CRH_CNF10_1 | GPIO_CRH_CNF10_0 |
                     GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0 );

    /* Configure PA10 (RX) as Input with pull-up/pull-down */
    GPIOA->CRH |= GPIO_CRH_CNF7_1;

    /* Enable pull-up */
    GPIOA->BSRR |= GPIO_BSRR_BS7;

#endif

#endif

}



/**
 * @brief	Helper function to configure USART1.
 * @brief   See usart1.h for config options.
 * @param	none
 * @retval	none
 */
static void usart1_config(void)
{
    /* Enable USART1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Set baud rate to 115200 */
    USART1->BRR = 0x271;

#if ( USE_USART1_TX )
    /* Enable USART1 transmitter */
    USART1->CR1 |= USART_CR1_TE;
#endif

#if ( USE_USART1_RX )

    /* Enable USART1 receiver */
    USART1->CR1 |= USART_CR1_RE;

    /* RXNE interrupt enable */
    USART1->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ( USART1_IRQn );
#endif

#if ( USE_USART1_DMA_TX )
    /* Enable USART1 TX with DMA transfer */
    USART1->CR3 |= USART_CR3_DMAT;
#endif

#if ( USE_USART1_DMA_RX )
    /* Enable USART1 RX with DMA transfer */
    USART1->CR3 |= USART_CR3_DMAR;
#else


#endif

    /* Enable USART1 */
    USART1->CR1 |= USART_CR1_UE;
}



#if ( USE_USART1_DMA_TX || USE_USART1_DMA_RX )

/**
 * @brief	Helper function to configure USART1 with DMA data transfer
 * @brief   See usart1.h for config options.
 * @param	none
 * @retval	none
 */
static void usart1_dma_config(void)
{
    /* Enable DMA1 clock */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

#if ( USE_USART1_DMA_TX )

    /* DMA peripheral address for TX (destination) */
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;
    /* Data direction from memory to peripheral */
    DMA1_Channel4->CCR |= DMA_CCR4_DIR;
    /* Channel priority for TX. Low priority */
    DMA1_Channel4->CCR &= ~( DMA_CCR4_PL_1 | DMA_CCR4_PL_0 );
    /* Memory auto increment enabled */
    DMA1_Channel4->CCR |= DMA_CCR4_MINC;

#if ( USE_USART1_DMA_TX_INT )
    /* Enable Transfer complete and Transfer Error interrupts */
    DMA1_Channel4->CCR |= ( DMA_CCR4_TCIE | DMA_CCR4_TEIE );
    /* Enable DMA1 Channel4 interrupt at NVIC */
    NVIC_EnableIRQ( DMA1_Channel4_IRQn );

#endif

#endif

#if ( USE_USART1_DMA_RX )
    /* DMA peripheral address for RX (source) */
    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;
    /* Data direction from peripheral to memory */
    DMA1_Channel5->CCR &= ~( DMA_CCR5_DIR );
    /* Channel priority for RX. Medium priority */
    DMA1_Channel5->CCR |= DMA_CCR5_PL_1;
    /* Memory auto increment enabled */
    DMA1_Channel5->CCR |= DMA_CCR5_MINC;

#if ( USE_USART1_DMA_RX_INT )

    /* Enable interrupts */
    DMA1_Channel5->CCR |= DMA_CCR5_TCIE | DMA_CCR5_TEIE;
    /* Enable DMA1 Channel5 interrupt at NVIC */
    NVIC_EnableIRQ( DMA1_Channel5_IRQn );

#endif

    /* Enable DMA1 Channel 5 */
    DMA1_Channel5->CCR |= DMA_CCR5_EN;

#endif

}

#endif
/**
  ******************************************************************************
  * @file    main.c
  * @author  Marco, Roldan L.
  * @version v1.0
  * @date    August 27, 2021
  * @brief   Demo codes for I2C driver
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

#include "stm32f10x.h"
#include <stdint.h>
#include <stdio.h>
#include "i2c.h"
#include "usart1.h"

/* Set only one macro at a time to test the functionality,
   the other device must be configured to master/slave
   appropriately */

#define I2C_TEST_MASTER_RX        0
#define I2C_TEST_MASTER_TX        0
#define I2C_TEST_SLAVE_RX         0
#define I2C_TEST_SLAVE_TX         0

#define PICO_SLAVE_ADDR           0x30
#define PICO_SLAVE_ADDR_W         ( PICO_SLAVE_ADDR << 1 )
#define PICO_SLAVE_ADDR_R         ( (PICO_SLAVE_ADDR << 1) | 0x01 )



char print_buf[30];

uint8_t master_tx_buffer[5] = { 0x41, 0x42, 0x43, 0x44};
uint8_t master_rx_buffer[10];
uint8_t slave_tx_buffer[5] = { 0xde, 0xad, 0xbe, 0xef };
uint8_t slave_rx_buffer[5];

void i2c_event_handler(I2C_TypeDef* I2Cx, i2c_events_vars_t* i2c_event_vars);



uint8_t global_flag = 0;

i2c_events_vars_t i2c_events;

int main()
{
    I2C_Init_t I2C_test_conf;
    i2c_structInit(&I2C_test_conf);
    i2c_init(I2C1, &I2C_test_conf);

    // usart1_init();
    // usart1_write("I2C Driver Test\n");

    // I2C1->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
    // NVIC_DisableIRQ(I2C1_EV_IRQn);

    // NVIC_SetPriority( USART1_IRQn, 5 );
    // NVIC_EnableIRQ( USART1_IRQn );


    while(1)
    {
        i2c_start_it(I2C1, &i2c_events, PICO_SLAVE_ADDR_W, 4, master_tx_buffer);
    }
}


void I2C1_EV_IRQHandler(void)
{
    i2c_events_vars_t* i2c_event_vars = &i2c_events;

    if( I2C1->SR1 & I2C_SR1_TXE )
    {
        I2C1->DR = *i2c_event_vars->i2c_buffer_ptr++;

        if(I2C1->SR1 & I2C_SR1_BTF)
        {
            i2c_event_vars->i2c_data_count--;

            if( i2c_event_vars->i2c_data_count == 0 )
            {
                i2c_stop(I2C1);
            }
        }
    }
    if( I2C1->SR1 & I2C_SR1_SB )
    {
        I2C1->DR = (i2c_event_vars->i2c_addr);
    }
    if( I2C1->SR1 & I2C_SR1_ADDR )
    {   
        I2C1->SR1 = I2C1->SR1;
        I2C1->SR2 = I2C1->SR2;
    }
}


// void i2c_event_handler(I2C_TypeDef* I2Cx, i2c_events_vars_t* i2c_event_vars)
// {
//     if( I2Cx->SR1 & I2C_SR1_TXE )
//     {
//         if( (i2c_event_vars->i2c_data_count) != 0 )
//         {
//             I2Cx->DR = *i2c_event_vars->i2c_buffer_ptr++;
//             i2c_event_vars->i2c_data_count--;
//         }
//         else
//         {
//             i2c_stop(I2C1);
//         }
//     }
//     else if( I2Cx->SR1 & I2C_SR1_SB )
//     {
//         I2Cx->DR = (i2c_event_vars->i2c_addr);
//     }
//     else if( I2Cx->SR1 & I2C_SR1_ADDR )
//     {   
//         I2Cx->SR1 = I2Cx->SR1;
//         I2Cx->SR2 = I2Cx->SR2;
//     }
//     else
//     {

//     }
// }





/* Test loop for master receiver */
#if I2C_TEST_MASTER_RX

    /* This code is tested on Arduino Nano acting as a slave transmitter
       because pico's i2c_write API doesn't work in slave mode as for my
       testing. The rest are tested with pico. */

    while(1)
    {
        i2c_start(I2C1);
        i2c_request(I2C1, PICO_SLAVE_ADDR_R);
        i2c_read_burst(I2C1, MASTER, 7, master_rx_buffer);

        uint8_t len;
        for(len = 0; master_rx_buffer[len] != '\0'; len++);

        for(uint8_t i = 0; i != len ; i++)
        {
            /* Print data in hexadecimal format */
            sprintf(print_buf, "%x", master_rx_buffer[i]);
            usart1_write(print_buf);
        }
        usart1_write("\n");
    }    

#endif /* I2C_TEST_MASTER_RX */


/* Test loop for master transmitter */
#if I2C_TEST_MASTER_TX

    /* Transmit 4 bytes of data to the slave */

    while(1)
    {
        i2c_start(I2C1);
        i2c_request(I2C1, PICO_SLAVE_ADDR_W);
        i2c_write_burst(I2C1, MASTER, 4, master_tx_buffer);
        i2c_stop(I2C1);
        usart1_write("done\n");
    }   

#endif /* I2C_TEST_MASTER_TX */


/* Test loop for slave receiver */
#if I2C_TEST_SLAVE_RX

    /* Receive N bytes of data from master and prints
    it to terminal. Maximum bytes that can be received 
    is defined by the buffer size however the real number
    of byte(s) received is determined by how many bytes
    the master transmits */

    while(1)
    {
        i2c_read_burst(I2C1, SLAVE, 0, slave_rx_buffer);

        uint8_t len;
        for(len = 0; slave_rx_buffer[len] != '\0'; len++);

        for(uint8_t i = 0; i != len ; i++)
        {
            /* Print data in hexadecimal format */
            sprintf(print_buf, "%x", slave_rx_buffer[i]);
            usart1_write(print_buf);
        }
        usart1_write("\n");
    }

#endif /* I2C_TEST_SLAVE_RX */


/* Test loop for slave transmitter */
#if I2C_TEST_SLAVE_TX

    /* Transmit 5 bytes of data as slave to the master */

    while(1)
    {
        i2c_write_burst(I2C1, SLAVE, 5, slave_tx_buffer);
        usart1_write("done\n");
    }

#endif /* I2C_TEST_SLAVE_TX */
/*
 * examples.h
 *
 *  Created on: Jan 25, 2024
 *      Author: sf
 */

#ifndef EXAMPLES_EXAMPLES_H_
#define EXAMPLES_EXAMPLES_H_

void blink(void);

void blink_ext(void);

void led_external_button(void);
void led_user_button(void);

void button_interrupt(void);

void spi_send_data(void);

void spi_send_data_arduino(void);

void spi_cmd_handling(void);

void i2c_master_tx(void);
void i2c_master_rx(void);

void i2c_master_rx_it(void);

void usart_tx(void);

void usart_case(void);

#endif /* EXAMPLES_EXAMPLES_H_ */

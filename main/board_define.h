#ifndef MAIN_BOARD_DEFINE_H_
#define MAIN_BOARD_DEFINE_H_

#include "driver/gpio.h"
#include "stdio.h"

#define ESP32_DXXX

#ifdef ESP32_DXXX

#define SPI_MOSI_GPIO GPIO_NUM_32
#define SPI_MISO_GPIO GPIO_NUM_18
#define SPI_SCK_GPIO GPIO_NUM_5
#define SPI_CS_GPIO GPIO_NUM_22

#define UART_2_TX_GPIO GPIO_NUM_2
#define UART_2_RX_GPIO GPIO_NUM_13 //ESP32 MTCK

#define ADC_0_GPIO GPIO_NUM_4 //ESP32 ADC2_CH0
#define ADC_1_GPIO GPIO_NUM_15 //ESP32 ADC2_CH3

#define UART_1_RX_GPIO GPIO_NUM_12
#define UART_1_TX_GPIO GPIO_NUM_14 //EPS32 MTMS

#define GPIO_0_GPIO GPIO_NUM_27
#define GPIO_1_GPIO GPIO_NUM_26
#define GPIO_2_GPIO GPIO_NUM_25
#define GPIO_3_GPIO GPIO_NUM_34 //ESP32 VDET_1

#define I2C_SDA_GPIO GPIO_NUM_35 //ESP32 VDET_2
#define I2C_SCL_GPIO GPIO_NUM_39 //ESP32 SENSOR_VN

#define GPIO_WKUP_GPIO GPIO_NUM_38 //ESP32 SENSOR_CAPN

#endif

//Application Board PinMap
#define GPIO_LED_0 SPI_MISO_GPIO
#define GPIO_LED_1 SPI_SCK_GPIO
#define GPIO_LED_2 SPI_CS_GPIO

#define GPIO_0 GPIO_0_GPIO
#define GPIO_BUTTON_0 GPIO_1_GPIO
#define GPIO_BUTTON_1 GPIO_2_GPIO
#define GPIO_1 GPIO_3_GPIO

#endif /* MAIN_BOARD_DEFINE_H_ */

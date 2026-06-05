/**
 * @file hexnet_io_map.h
 * @brief Hexnet IO Module – ESP32-S3 pin map (single reference)
 *
 *  ┌────────┬──────────────────────────────────────────────────────────────┐
 *  │  GPIO  │  Function                                                    │
 *  ├────────┼──────────────────────────────────────────────────────────────┤
 *  │   1    │  Dimmer PWM #1 (LEDC)                                        │
 *  │   2    │  Dimmer PWM #2 (LEDC)                                        │
 *  │   3    │  ADC – battery sense (ADC1_CH2)                              │
 *  │   4    │  ADC – analog sensor 0 (ADC1_CH3)                            │
 *  │   5    │  ADC – analog sensor 1 (ADC1_CH4)                            │
 *  │   6    │  DHT (reserved / alt)                                        │
 *  │   7    │  DHT sensor data                                             │
 *  │   8    │  RS485 driver enable (DE/RE)                                 │
 *  │  10    │  Digital MUX select S0                                       │
 *  │  11    │  Digital MUX select S1                                       │
 *  │  12    │  Digital MUX select S2                                       │
 *  │  13    │  Digital MUX bank A read (MUX1_Z)                            │
 *  │  14    │  Digital MUX bank B read (MUX2_Z)                            │
 *  │  16    │  Motor driver enable                                         │
 *  │  17    │  CAN / TWAI RX                                               │
 *  │  18    │  CAN / TWAI TX                                               │
 *  │  19    │  UART1 TX – RS485                                            │
 *  │  20    │  UART1 RX – RS485                                            │
 *  │  35    │  Motor direction B                                           │
 *  │  36    │  Motor direction A                                           │
 *  │  37    │  WS2812 RGB strip data                                       │
 *  │  38    │  Dimmer PWM #4 (LEDC) – shares DAC RDY/BSY if I2C dim used   │
 *  │  39    │  Dimmer PWM #3 (LEDC) – shares DAC LDAC if I2C dim used      │
 *  │  40    │  74HC4094 shift-register latch (STB)                         │
 *  │  41    │  74HC4094 shift-register data (DATA)                         │
 *  │  42    │  74HC4094 shift-register clock (CLK)                         │
 *  │  43    │  USB/UART console TX (ROM default)                           │
 *  │  44    │  USB/UART console RX (ROM default)                           │
 *  │  45    │  Run / status LED                                            │
 *  │  47    │  I2C SCL (MCP4728 DAC – optional, not used in PWM profile)   │
 *  │  48    │  I2C SDA (MCP4728 DAC – optional)                            │
 *  └────────┴──────────────────────────────────────────────────────────────┘
 */
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

/* --- Status --- */
#define HEXNET_IO_RUN_LED_GPIO          GPIO_NUM_45

/* --- Shift register (16 digital outputs) --- */
#define HEXNET_IO_SR_CLK_GPIO           GPIO_NUM_42
#define HEXNET_IO_SR_DATA_GPIO          GPIO_NUM_41
#define HEXNET_IO_SR_STB_GPIO           GPIO_NUM_40

/* --- Motor (H-bridge) --- */
#define HEXNET_IO_MOTOR_EN_GPIO         GPIO_NUM_16
#define HEXNET_IO_MOTOR_A_GPIO          GPIO_NUM_36
#define HEXNET_IO_MOTOR_B_GPIO          GPIO_NUM_35

/* --- Dimmer PWM (LEDC) --- */
#define HEXNET_IO_DIM_PWM_GPIO_1        GPIO_NUM_1
#define HEXNET_IO_DIM_PWM_GPIO_2        GPIO_NUM_2
#define HEXNET_IO_DIM_PWM_GPIO_3        GPIO_NUM_39
#define HEXNET_IO_DIM_PWM_GPIO_4        GPIO_NUM_38

/* --- WS2812 RGB matrix --- */
#define HEXNET_IO_RGB_STRIP_GPIO        GPIO_NUM_37
#define HEXNET_IO_RGB_LED_COUNT         65
#define HEXNET_IO_RGB_MATRIX_ROWS       10
#define HEXNET_IO_RGB_MATRIX_COLS       20
#define HEXNET_IO_RGB_MATRIX_LEDS       (HEXNET_IO_RGB_MATRIX_ROWS * HEXNET_IO_RGB_MATRIX_COLS)

/* --- Digital input MUX (16 inputs) --- */
#define HEXNET_IO_MUX_S0_GPIO           GPIO_NUM_10
#define HEXNET_IO_MUX_S1_GPIO           GPIO_NUM_11
#define HEXNET_IO_MUX_S2_GPIO           GPIO_NUM_12
#define HEXNET_IO_MUX_Z_A_GPIO          GPIO_NUM_13
#define HEXNET_IO_MUX_Z_B_GPIO          GPIO_NUM_14
#define HEXNET_IO_MUX_CHANNEL_COUNT     8

/* --- DHT --- */
#define HEXNET_IO_DHT_GPIO_PRIMARY      GPIO_NUM_7
#define HEXNET_IO_DHT_GPIO_ALT          GPIO_NUM_6

/* --- RS485 / UART1 --- */
#define HEXNET_IO_RS485_EN_GPIO         GPIO_NUM_8
#define HEXNET_IO_UART_RS485_PORT       UART_NUM_1
#define HEXNET_IO_UART_RS485_TX_GPIO    GPIO_NUM_19
#define HEXNET_IO_UART_RS485_RX_GPIO    GPIO_NUM_20
#define HEXNET_IO_UART_RS485_BAUD       115200
#define HEXNET_IO_UART_RS485_BUF_SIZE   512

/* --- CAN / TWAI --- */
#define HEXNET_IO_CAN_TX_GPIO           GPIO_NUM_18
#define HEXNET_IO_CAN_RX_GPIO           GPIO_NUM_17

/* --- I2C DAC MCP4728 (optional dim path) --- */
#define HEXNET_IO_I2C_SCL_GPIO          GPIO_NUM_47
#define HEXNET_IO_I2C_SDA_GPIO          GPIO_NUM_48
#define HEXNET_IO_DAC_LDAC_GPIO         GPIO_NUM_39
#define HEXNET_IO_DAC_RDY_GPIO          GPIO_NUM_38
#define HEXNET_IO_I2C_DAC_ADDR          0x60

/* Bit masks for gpio_config */
#define HEXNET_IO_MUX_SEL_MASK \
    ((1ULL << HEXNET_IO_MUX_S0_GPIO) | (1ULL << HEXNET_IO_MUX_S1_GPIO) | (1ULL << HEXNET_IO_MUX_S2_GPIO))
#define HEXNET_IO_MUX_Z_MASK \
    ((1ULL << HEXNET_IO_MUX_Z_A_GPIO) | (1ULL << HEXNET_IO_MUX_Z_B_GPIO))
#define HEXNET_IO_SR_MASK \
    ((1ULL << HEXNET_IO_SR_CLK_GPIO) | (1ULL << HEXNET_IO_SR_DATA_GPIO) | (1ULL << HEXNET_IO_SR_STB_GPIO))
#define HEXNET_IO_MOTOR_MASK \
    ((1ULL << HEXNET_IO_MOTOR_EN_GPIO) | (1ULL << HEXNET_IO_MOTOR_A_GPIO) | (1ULL << HEXNET_IO_MOTOR_B_GPIO))

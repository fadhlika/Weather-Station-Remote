#ifndef DHT22_H
#define DHT22_H

#include "main.h"
#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "tim.h"

static char temp_char1[2], temp_char2, rh_char1[2], rh_char2;
static uint8_t check = 0;
static uint16_t sum, RH, TEMP;
static GPIO_InitTypeDef GPIO_InitStruct;

static void Delay_us(uint32_t us);
static void set_gpio_output(void);
static void set_gpio_input(void);

void DHT22_start(void);

static void check_response(void);

static uint8_t read_data(void);

bool DHT22_sample(void);

uint16_t DHT22_getTemperature(void);
uint16_t DHT22_getHumidity(void);
#endif
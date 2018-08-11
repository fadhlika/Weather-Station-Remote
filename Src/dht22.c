#include "dht22.h"

void Delay_us(uint32_t us)
{
    HAL_TIM_Base_Start(&htim17);
    while (__HAL_TIM_GET_COUNTER(&htim17) < us)
        ;
    HAL_TIM_Base_Stop(&htim17);
    __HAL_TIM_SET_COUNTER(&htim17, 0);
}

void set_gpio_output(void)
{
    /*Configure GPIO pin output: PA1 */
    GPIO_InitStruct.Pin = DHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);
}

void set_gpio_input(void)
{
    /*Configure GPIO pin input: PA1 */
    GPIO_InitStruct.Pin = DHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);
}

void DHT22_start(void)
{
    set_gpio_output();                            // set the pin as output
    HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, 0); // pull the pin low
    Delay_us(500);                                // wait for 500us
    HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, 1); // pull the pin high
    Delay_us(30);                                 // wait for 30us
    set_gpio_input();                             // set as input
}

void check_response(void)
{
    Delay_us(40);
    if (!(HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)))
    {
        Delay_us(80);
        if ((HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)))
            check = 1;
    }
    while ((HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)))
        ; // wait for the pin to go low
}

uint8_t read_data(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        while (!(HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)))
            ;                                                // wait for the pin to go high
        Delay_us(40);                                        // wait for 40 us
        if ((HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)) == 0) // if the pin is low
        {
            i &= ~(1 << (7 - j)); // write 0
        }
        else
            i |= (1 << (7 - j)); // if the pin is high, write 1
        while ((HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin)))
            ; // wait for the pin to go low
    }
    return i;
}

bool DHT22_sample(void)
{
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
    DHT22_start();
    check_response();
    Rh_byte1 = read_data();
    Rh_byte2 = read_data();
    Temp_byte1 = read_data();
    Temp_byte2 = read_data();
    sum = read_data();
    //if (sum == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2))
    {
        TEMP = ((Temp_byte1 << 8) | Temp_byte2);
        RH = ((Rh_byte1 << 8) | Rh_byte2);

        return true;
    }
    return false;
}

uint16_t DHT22_getTemperature(void)
{
    return TEMP;
}

uint16_t DHT22_getHumidity(void)
{
    return RH;
}
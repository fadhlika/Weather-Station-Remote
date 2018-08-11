
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include <stdbool.h>
#include "dht22.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBUG 1
volatile bool isAnemometerDone = false;
volatile uint32_t period = 0;
volatile bool isMeasure = false;
volatile bool rainfall = false;
uint16_t last_temp = 0, last_rh = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void LoRa_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    period = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(htim, 0);
    isAnemometerDone = true;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14)
  {
    isMeasure = true;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case RAINGAUGE_Pin:
    rainfall = 1;
    break;
  case DIO0_Pin:
    LoRa_OnDioRise();
    break;
  }
}

void OnReceive(int packetsize)
{
  uint8_t buf[packetsize];

  int i;
  for (i = 0; i < packetsize; i++)
  {
    buf[i] = (char)LoRa_Read();
  }
  HAL_UART_Transmit(&huart1, &buf, packetsize, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 1000);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  uint32_t reset_reson = RCC->CSR;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  LoRa_Init();
  LoRa_onReceive(OnReceive);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim14);

  HAL_UART_Transmit(&huart1, "Boot\n", 6, 1000);
  if (reset_reson != 0x00000000)
  {
    uint8_t packet[6] = {6, 4, (uint8_t)(reset_reson >> 24), (uint8_t)(reset_reson >> 16), (uint8_t)(reset_reson >> 8), (uint8_t)(reset_reson)};
    LoRa_Transmit(packet, 6);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //LoRa_Receive();
    HAL_UART_Transmit(&huart1, "Loop Start\n", 11, 1000);

    if (isMeasure)
    {
      HAL_UART_Transmit(&huart1, "Measuring...\n", 12, 1000);
      DHT22_sample();

      uint16_t TEMP = DHT22_getTemperature();
      uint16_t RH = DHT22_getHumidity();
      char b[100];
      int len = sprintf(b, "last_temp: %u temp: %u last_hum:%u hum: %u\n", last_temp, TEMP, last_rh, RH);
      HAL_UART_Transmit(&huart1, b, len, 1000);

      uint8_t packet[8] = {4, 2, (uint8_t)(TEMP >> 8), (uint8_t)TEMP, 5, 2, (uint8_t)(RH >> 8), (uint8_t)RH};

      LoRa_Transmit(packet, 8);
      isMeasure = false;
    }

    if (isAnemometerDone)
    {
      HAL_GPIO_WritePin(VANE_EN_GPIO_Port, VANE_EN_Pin, GPIO_PIN_RESET);

      uint16_t adc_raw[3];
      HAL_ADCEx_Calibration_Start(&hadc);
      HAL_ADC_Start(&hadc);
      int i;
      for (i = 0; i < 3; i++)
      {
        if (HAL_ADC_PollForConversion(&hadc, 500) == HAL_OK)
        {
          adc_raw[i] = HAL_ADC_GetValue(&hadc);
        }
      }
      HAL_ADC_Stop(&hadc);

      HAL_GPIO_WritePin(VANE_EN_GPIO_Port, VANE_EN_Pin, GPIO_PIN_SET);

      uint8_t dir = 0;
      if (adc_raw[0] > adc_raw[1])
      {
        if (adc_raw[1] > 1800)
        {
          dir = 0;
        }
        else if (adc_raw[1] <= 1800)
        {
          dir = 1;
        }
      }
      else if (adc_raw[0] < adc_raw[1])
      {
        if (adc_raw[1] > 2800)
        {
          dir = 2;
        }
        else if (adc_raw[1] <= 2800)
        {
          dir = 3;
        }
      }

      uint16_t vdd = 3300 * (float)(*((uint16_t *)((uint32_t)0x1FFFF7BA))) / adc_raw[2];
      double vel = 0;
      if (period > 0)
        vel = 0.6142 * (1.0 / ((double)period / 1000.0)) + 0.3098;
      int vel_int = (int)(vel * 1000);

      uint8_t packet[7] = {1, 2, (uint8_t)(vel_int >> 8), (uint8_t)vel_int, 2, 1, (uint8_t)dir};
      LoRa_Transmit(packet, 7);
#if DEBUG
      char buffer[500];
      int len_ = sprintf(buffer, "speed: %d dir: %d adc0: %u adc1:%u vcc: %u rain: %d\n", vel_int, dir, adc_raw[0], adc_raw[1], vdd, rainfall);
      HAL_UART_Transmit(&huart1, &buffer, len_, 1000);
#endif
      isAnemometerDone = false;
    }

    if (rainfall)
    {
      uint8_t packet[3] = {3, 1, 1};
      LoRa_Transmit(packet, 3);
      rainfall = 0;
    }

    HAL_UART_Transmit(&huart1, "Loop Done\n", 10, 1000);

    LoRa_Sleep();
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    HAL_UART_Transmit(&huart1, "Resume Loop\n", 12, 1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

  /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void LoRa_Init(void)
{
  lora.dio0.Port = GPIOA;
  lora.dio0.Pin = GPIO_PIN_0;
  lora.reset.Port = GPIOA;
  lora.reset.Pin = GPIO_PIN_8;
  lora.ss.Port = GPIOA;
  lora.ss.Pin = GPIO_PIN_4;
  lora.hspi = &hspi1;

  if (LoRa_Begin(4333E5) != HAL_OK)
  {
#if DEBUG
    HAL_UART_Transmit(&huart1, (uint8_t *)"Error\r\n", 7, 1000);
#endif
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t captureIndex = 0, 
captureValue1, 
captureValue2, 
period;
uint8_t anemometer_done = 0,
anemometer_timeout = 0;
uint8_t timebuffer[32], buffer[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
void RTC_DateTimeShow(uint8_t* showtime);
void RTC_AlarmConfig(void);
void LoRa_Init(void);
static void SYSCLKConfig_STOP(void);
void RTC_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    if(captureIndex == 0)
    {
      captureValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
      captureIndex = 1;
      //HAL_UART_Transmit(&huart1, (uint8_t*) "Callback1\r\n", 11, 1000);
    } 
    else if(captureIndex == 1)
    {
      captureValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

      if(captureValue2 > captureValue1) {
        period = captureValue2 - captureValue1;
      } else if (captureValue2 < captureValue1) 
      {
        period = ((4999 - captureValue1) + captureValue2) + 1;
      }
      //HAL_UART_Transmit(&huart1, (uint8_t*) "Callback2\r\n", 11, 1000);
      HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);
      anemometer_done = 1;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
  if(htim->Instance == TIM14)
  {
    anemometer_timeout = 1;
  }
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    RTC_DateTimeShow(&timebuffer);
    RTC_AlarmConfig();
    
    anemometer_done = 0;
    anemometer_timeout = 0;
    captureIndex = 0;
    period = 0;
    
    HAL_TIM_Base_Start_IT(&htim14);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

    uint16_t adc_raw[3];
    HAL_ADCEx_Calibration_Start(&hadc);
    HAL_ADC_Start(&hadc);
    int i;
    for(i=0; i<3; i++) {
      if(HAL_ADC_PollForConversion(&hadc, 500) == HAL_OK) {
        adc_raw[i] = HAL_ADC_GetValue(&hadc);
      }
    }
    HAL_ADC_Stop(&hadc);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

    uint16_t vdd = 3300 * (float)(*((uint16_t*) ((uint32_t) 0x1FFFF7BA)))/adc_raw[2];

    while((anemometer_done != 1) && (anemometer_timeout != 1)) {
      HAL_SuspendTick();
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
      HAL_ResumeTick();
    }
    
    HAL_TIM_Base_Stop_IT(&htim14);
    if(anemometer_timeout == 1) {
      HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
    }
  
    uint32_t tip = __HAL_TIM_GET_COUNTER(&htim3);
    int len = sprintf(buffer, "%s;%u;%u;%u;%u;%u", timebuffer, vdd, tip, adc_raw[0], adc_raw[1], period);
    
    HAL_UART_Transmit(&huart1, &buffer, len, 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, 1000);
    
    LoRa_Init();
    LoRa_Transmit(buffer, len);
    LoRa_Sleep();
    
    
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
    SystemClock_Config();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void RTC_DateTimeShow(uint8_t* showtime)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char*)showtime,"20%02d-%02d-%02dT%02d:%02d:%02d",
  sdatestructureget.Year, sdatestructureget.Month, sdatestructureget.Date,
  stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
} 

void RTC_AlarmConfig(void) {
   RTC_TimeTypeDef stimestructureget;
   RTC_DateTypeDef sdatestructureget;
   RTC_AlarmTypeDef salarmstructureget;
 
   /* Get the RTC current Time */

   HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

   HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
   /* Get the RTC current Date */

   HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

   salarmstructureget.AlarmTime.Hours = stimestructureget.Hours;
   salarmstructureget.AlarmTime.Minutes = stimestructureget.Minutes;
   salarmstructureget.AlarmTime.Seconds = (stimestructureget.Seconds + 15) % 60;
   salarmstructureget.AlarmTime.SubSeconds = stimestructureget.SubSeconds;
   salarmstructureget.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
   salarmstructureget.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
   salarmstructureget.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
   salarmstructureget.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
   salarmstructureget.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
   salarmstructureget.AlarmDateWeekDay = sdatestructureget.Date;
   salarmstructureget.Alarm = RTC_ALARM_A;
   if (HAL_RTC_SetAlarm_IT(&hrtc, &salarmstructureget, RTC_FORMAT_BIN) != HAL_OK)
   {
     _Error_Handler(__FILE__, __LINE__);
   }
 }
 

static void SYSCLKConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */

  uint32_t pFLatency = 0;

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from STOP reconfigure the system clock: Enable HSE and PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /*
  uint32_t timer = __HAL_TIM_GET_COUNTER(&htim16);
  __HAL_TIM_SET_COUNTER(&htim16, 0);
  char buffer[16];
  sprintf(buffer, "%u\r\n", timer);*/
  //HAL_UART_Transmit(&huart1, (uint8_t*) "Alarm\r\n", 7, 1000);
  
}

void LoRa_Init(void) {
  lora.dio0.Port = GPIOA;
  lora.dio0.Pin = GPIO_PIN_0;
  lora.reset.Port = GPIOA;
  lora.reset.Pin = GPIO_PIN_8;
  lora.ss.Port = GPIOA;
  lora.ss.Pin = GPIO_PIN_4;
  lora.hspi = &hspi1;

  if(LoRa_Begin(433E6) != HAL_OK) {
    Error_Handler();
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
  while(1)
  {
    
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
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

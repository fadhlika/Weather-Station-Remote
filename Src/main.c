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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBUG 1

uint32_t captureIndex = 0, 
captureValue1, 
captureValue2, 
period;
uint8_t timeout = 0;
uint32_t tip = 0;
uint8_t rainfall = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
void RTC_DateTimeShow(uint8_t* time);
void LoRa_Init(void);
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
      HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);
    }
  }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
  if(htim->Instance == TIM14)
  {
    timeout = 1;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case RAINGAUGE_Pin:
      tip++;
      rainfall = 1;
      break;
    case DIO0_Pin:
      LoRa_OnDioRise();
  }
}

void OnReceive(int packetsize) {
  uint8_t buf[packetsize];

  int i;
  for(i=0; i<packetsize; i++) {
    buf[i] = (char)LoRa_Read();
  }
  HAL_UART_Transmit(&huart1, &buf, packetsize, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, 1000);
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
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_8SEC, RTC_SMOOTHCALIB_PLUSPULSES_SET, 0x02);
  LoRa_Init();
  LoRa_onReceive(OnReceive);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    #if DEBUG
    HAL_UART_Transmit(&huart1, (uint8_t*) "Setting Alarm\r\n", 15, 1000);
    #endif
    
    uint8_t timebuffer[6], buffer[64];
    RTC_DateTimeShow(&timebuffer);
    
    timeout = 0;
    captureIndex = 0;
    period = 0;
    __HAL_TIM_SET_COUNTER(&htim14, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    LoRa_Receive();
    
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

    char dir;
    if(adc_raw[0] > adc_raw[1]) {
      if(adc_raw[1] > 1800) {
        dir = 'N';
      } else if(adc_raw[1] <= 1800) {
        dir = 'W';
      } 
    } else if(adc_raw[0] < adc_raw[1]) {
      if(adc_raw[1] > 2800) {
        dir = 'E';
      } else if(adc_raw[1] <= 2800) {
        dir = 'S';
      }
    }

    uint16_t vdd = 3300 * (float)(*((uint16_t*) ((uint32_t) 0x1FFFF7BA)))/adc_raw[2];

    while(timeout != 1) {
      HAL_SuspendTick();
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
      HAL_ResumeTick();
    }
    
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);    
    HAL_TIM_Base_Stop_IT(&htim14);

    #if DEBUG
    int len_ = sprintf(buffer, "cv1: %u cv2: %u, adc1: %u adc2: %u, period: %u\r\n", 
      captureValue1, captureValue2, adc_raw[0], adc_raw[1], period);
    HAL_UART_Transmit(&huart1, &buffer, len_, 1000);
    #endif

    double vel = 0;
    if (period > 0) vel = 0.6142 * (1.0/((double)period/1000.0)) + 0.3098;
    int vel_int = (int)(vel*1000);

    #if DEBUG
    int len = sprintf(buffer, "1;20%.2u-%.2u-%.2uT%.2u:%.2u:%.2u;%c;%u;%d;%u", 
                      timebuffer[0], timebuffer[1], timebuffer[2], 
                      timebuffer[3], timebuffer[4], timebuffer[5],
                      dir, vdd, vel_int, tip);
    HAL_UART_Transmit(&huart1, &buffer, len, 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, 1000);
    #endif
    
    LoRa_Transmit(buffer, len);
    LoRa_Sleep();
    tip = 0;

    stop:
    #if DEBUG
    HAL_UART_Transmit(&huart1, (uint8_t*) "Enter Stop\r\n", 12, 1000);
    #endif
    __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    rainfall = 0;
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
    SystemClock_Config();

    #if DEBUG
    HAL_UART_Transmit(&huart1, (uint8_t*) "Wake Up\r\n", 9, 1000);
    #endif

    if(rainfall == 1) {
      rainfall = 0;
      goto stop;
    }
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
void RTC_DateTimeShow(uint8_t* time)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  time[0] = sdatestructureget.Year;
  time[1] = sdatestructureget.Month;
  time[2] = sdatestructureget.Date;
  time[3] = stimestructureget.Hours;
  time[4] = stimestructureget.Minutes;
  time[5] = stimestructureget.Seconds;

  RTC_AlarmTypeDef salarmstructureget;
 
  /* Get the RTC current Time */

  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */

  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

  salarmstructureget.AlarmTime.Hours = stimestructureget.Hours;
  salarmstructureget.AlarmTime.Minutes = (stimestructureget.Minutes + 1) % 60;
  salarmstructureget.AlarmTime.Seconds = (stimestructureget.Seconds);
  salarmstructureget.AlarmTime.SubSeconds = stimestructureget.SubSeconds;
  salarmstructureget.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  salarmstructureget.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  salarmstructureget.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS;
  salarmstructureget.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  salarmstructureget.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  salarmstructureget.AlarmDateWeekDay = sdatestructureget.Date;
  salarmstructureget.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &salarmstructureget, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    #if DEBUG
    HAL_UART_Transmit(&huart1, (uint8_t*) "Error\r\n", 7, 1000);
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "./BSP/LCD/lcd.h"
#include "./BSP/pidw/pid.h"
#include "./BSP/OLED/OLED.h"
#include "stdio.h"
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t size=200;
float ceshi=0;
uint32_t ADC_Value[ADC_SIZE];
float va,vb,va1;
uint32_t ad1,ad2,i;
int count=0;
__IO uint8_t AdcConvEnd = 0;

float fft_inputbuf[FFT_LENGTH * 2];  
float fft_outputbuf[FFT_LENGTH];  


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
	
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_ms(uint16_t t)
{
	while(t--);
}
uint16_t sin1[200] = {3657,3770,3882,3995,4107,4219,4330,4440,4550,4659,4766,4873,4978,5081,5184,5285,5384,
5481,5576,5670,5762,5851,5938,6023,6105,6185,6263,6337,6410,6479,6545,6609,6670,
6727,6782,6833,6881,6926,6968,7006,7041,7072,7101,7125,7146,7164,7178,7189,7196,
7200,7200,7196,7189,7178,7164,7146,7125,7101,7072,7041,7006,6968,6926,6881,6833,
6782,6727,6670,6609,6545,6479,6410,6337,6263,6185,6105,6023,5938,5851,5762,5670,
5576,5481,5384,5285,5184,5081,4978,4873,4766,4659,4550,4440,4330,4219,4107,3995,
3882,3770,3657,3543,3430,3318,3205,3093,2981,2870,2760,2650,2541,2434,2327,2222,
2119,2016,1915,1816,1719,1624,1530,1438,1349,1262,1177,1095,1015,937,863,790,
721,655,591,530,473,418,367,319,274,232,194,159,128,99,75,54,
36,22,11,4,0,0,4,11,22,36,54,75,99,128,159,194,
232,274,319,367,418,473,530,591,655,721,790,863,937,1015,1095,1177,
1262,1349,1438,1530,1624,1719,1816,1915,2016,2119,2222,2327,2434,2541,2650,2760,
2870,2981,3093,3205,3318,3430,3543}; 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int k = 0;
	if(++k == size)k = 0;
	if(htim->Instance == TIM1)
	{
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sin1[k]); 
		TIM1->CCR1=sin1[k];
	}
	else if(htim->Instance == TIM3)
	{

	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  OLED_init();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim1); //开启定时器
  HAL_ADCEx_Calibration_Start(&hadc1);                  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, ADC_SIZE);
  HAL_TIM_Base_Start_IT(&htim3); //开启定时器
  ceshi=156.0212;
  while (!AdcConvEnd)                                   //等待转化完毕
    ;
  for (uint16_t i = 0; i < ADC_SIZE; i+=2)
  {
	printf("%.3f\n", ADC_Value[i] * 3.3 / 4095); //打印ADC_Value
  }
  /**********************进行傅里叶变换*******************************/
  AdcConvEnd=0;  
  for (int i = 0; i < FFT_LENGTH; i++)
  {
    fft_inputbuf[i * 2] = ADC_Value[2*i] * 3.3 / 4096;
    fft_inputbuf[i * 2 + 1] = 0;
  }
  arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);
  arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH);
  /**********************等待转化完毕*******************************/
  fft_outputbuf[0] /= 1024;

    for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
    {
        fft_outputbuf[i] /= 512;
    }

    /***********************打印结果**********************************/
    printf("FFT Result:\r\n");

    for (int i = 0; i < FFT_LENGTH; i++)//
    {
        printf("%d:\t%.2f\r\n", i, fft_outputbuf[i]);
    }    
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  OLED_operate_gram(PEN_CLEAR);
  OLED_printf(0,0,"HUIHUI");
  OLED_printf(1,0,"%.2f",ceshi);
  OLED_refresh_gram();
  ceshi++;
  HAL_Delay(100);
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
float va,vb,va1;
uint32_t ad1,ad2,i;
int count=0;
__IO uint8_t AdcConvEnd = 0;
uint32_t ADC_count=0,eff_measure=0;
uint32_t ADC_Value[ADC_SIZE];

float adc_buff[FFT_LENGTH];
float fft_inputbuf[FFT_LENGTH * 2];  
float fft_outputbuf[FFT_LENGTH];  
float effect[FFT_LENGTH];
float UI[FFT_LENGTH];

float Voltage_REF=0.514;
double effective_value;
double effective_value_all;
double uk=0.7;
CNTL_PI_F U_pi;              //U_pi

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
void key_scan()
{
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3) == GPIO_PIN_RESET)
	{
		delay_ms(1000);
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3) == GPIO_PIN_RESET)    //KEY1_Pin
		{
			HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);              //PB5 0
//            Voltage_REF-=0.01;
			while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3) == GPIO_PIN_RESET );
		}
	}
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4) == GPIO_PIN_RESET)       //KEY0_Pin
	{ 
		delay_ms(1000);
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4) == GPIO_PIN_RESET)
		{
			Voltage_REF+=0.001;
			uk+=0.003;
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);               //PE5  0
			while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4) == GPIO_PIN_RESET );
		}
	}
}
uint16_t sin1[200] = {3599,3709,3820,3931,4041,4150,4260,4368,4476,4583,4689,4794,4897,5000,5101,5200,
5298,5394,5489,5582,5672,5761,5847,5932,6014,6093,6170,6245,6317,6386,6453,6516,
6577,6635,6690,6742,6791,6836,6879,6918,6954,6986,7016,7042,7064,7083,7099,7111,
7120,7125,7127,7125,7120,7111,7099,7083,7064,7042,7016,6986,6954,6918,6879,6836,
6791,6742,6690,6635,6577,6516,6453,6386,6317,6245,6170,6093,6014,5932,5847,5761,
5672,5582,5489,5394,5298,5200,5101,5000,4897,4794,4689,4583,4476,4368,4260,4150,
4041,3931,3820,3709,3599,3488,3377,3266,3156,3047,2937,2829,2721,2614,2508,2403,
2300,2197,2096,1997,1899,1803,1708,1615,1525,1436,1350,1265,1183,1104,1027,952,
880,811,744,681,620,562,507,455,406,361,318,279,243,211,181,155,
133,114,98,86,77,72,71,72,77,86,98,114,133,155,181,211,
243,279,318,361,406,455,507,562,620,681,744,811,880,952,1027,1104,
1183,1265,1350,1436,1525,1615,1708,1803,1899,1997,2096,2197,2300,2403,2508,2614,
2721,2829,2937,3047,3156,3266,3377,3488}; 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int k = 0;
	if(htim->Instance == TIM1)
	{
//    	if(U_pi.Out>0&&U_pi.Out<1)
//		{
			TIM1->CCR1=3600+(sin1[k]-3600)*U_pi.Out*0.98;
//		}
//		TIM1->CCR1=3600+(sin1[k]-3600)*uk*0.98;
//		TIM1->CCR1=sin1[k];
		k++;
		if(k == size)k = 0;
	}
	else if(htim->Instance == TIM3)
	{
//		adc_buff[ADC_count]=ADC_Value[0]*3.3/4096+0.00;
//		printf("%d",ADC_Value[0]);
//		printf("%.3f\n",adc_buff[ADC_count]);
//		ADC_count++;
//		if(ADC_count>1023) ADC_count=0;
//		printf("%.3f\n",adc_buff[ADC_count]);
//		for (uint16_t i = 0; i < FFT_LENGTH; i++)
//		{
//		    printf("%.3f\n", adc_buff[i]); 
//		}
		
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
  HAL_TIM_Base_Start_IT(&htim1); //�??启定时器
  HAL_ADCEx_Calibration_Start(&hadc1);                  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, ADC_SIZE);
  HAL_TIM_Base_Start_IT(&htim3); //�??启定时器
  ceshi=156.0212;
  CNTL_PI_F_init(&U_pi);//U_pi初始值化
  U_pi.Kp = 1;    U_pi.Ki = 0.5;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    key_scan();
//	for (uint16_t i = 0; i < FFT_LENGTH; i++)
//	{
//     printf("%.3f\n", adc_buff[i]); //打印ADC_Value
//	}
//  printf("%.3f,%d\n",adc_buff[ADC_count],ADC_count);
//  OLED_operate_gram(PEN_CLEAR);   
//  OLED_printf(0,0,"HUIHUI");
//  OLED_printf(1,0,"%.2f",ceshi);
//  OLED_refresh_gram();
	ceshi++;
	if(ceshi>=65536) ceshi=0;
//  HAL_Delay(100);
	  
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

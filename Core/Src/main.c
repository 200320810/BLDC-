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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_2.8_lcd.h"
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
int ha=0;
int hb=0;
int hc=0;
int i=0;
int m=0;
uint32_t pwmzkb[1];
uint32_t pwmv;
uint32_t voltva[1];
uint32_t voval;
float angle;
float speed;
int cPeriod;
int t=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(TIM3==htim->Instance)
	//cPeriod++;
	{
		HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_1);
		speed = 60*t/(180*4);
		t=0;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)//
	{
		if(m==0)
		{
			return;
		}
			/*		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmv);
__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmv);
__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmv);*/
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(HALLA_GPIO_Port,HALLA_Pin)==SET)
		{
			ha=1;
		}
		else
		{
			ha=0;
		}
		if(HAL_GPIO_ReadPin(HALLB_GPIO_Port,HALLB_Pin)==SET)
		{
			hb=1;
		}
		else
		{
			hb=0;
		}
		if(HAL_GPIO_ReadPin(HALLC_GPIO_Port,HALLC_Pin)==SET)
		{
			hc=1;
		}
		else
		{
			hc=0;
		}
		if(ha==1&&hb==0&&hc==1)
		{
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		else if(ha==1&&hb==0&&hc==0)
		{
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		}
		else if(ha==1&&hb==1&&hc==0)
		{
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		}
		else if(ha==0&&hb==1&&hc==0)
		{
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		else if(ha==0&&hb==1&&hc==1)
		{
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		else if(ha==0&&hb==0&&hc==1)
		{
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
			
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		t++;
		/*if(t>=1000)
		{
		speed = 6000/((cPeriod*0.001)*360*3);
		cPeriod = 0;
			t=0;
		}*/
		//angle += 60;
			/*if(cPeriod >= 50){
				speed = angle/((cPeriod*0.001)*360*3);
				cPeriod = 0;
				angle = 0;
				//LCD_DisplayStringLine(2,(u8 *)"BLDC speed:");
				//LCD_Draw_NUM(70,300,(int)speed);
			}*/
	/*	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);*/
	}
}

	void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
	{
		if(htim->Instance==htim8.Instance)
		{
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			__HAL_TIM_DISABLE_IT(&htim8,TIM_IT_BREAK);
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
  MX_ADC3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	STM32_LCD_Init();
	LCD_Clear(BackColor);
	LCD_SetTextColor(Blue);
HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
/*HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);*/
__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
HAL_TIM_Base_Start_IT(&htim3);
HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
//HAL_TIM_Base_Start_IT(&htim2);
HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
//HAL_TIM_Base_Start_IT(&htim2);
HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		int j=0;
	if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_10)==GPIO_PIN_RESET)
			{
		for(j=0;j<10000;j++){}
				if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_10)==GPIO_PIN_RESET){
					m = 1;
				}
			}
			if(m==1)
			{
		if(i==0)
		{
			HAL_TIM_IC_CaptureCallback(&htim2);
		HAL_Delay(4);
		HAL_TIM_IC_CaptureCallback(&htim2);
			i=1;
		}
		//if(cPeriod >= 50){
				//speed = 60/((cPeriod*0.001)*360*3);
				//cPeriod = 0;
				//angle = 0;
				//LCD_DisplayStringLine(2,(u8 *)"BLDC speed:");
				//LCD_Draw_NUM(70,300,(int)speed);
			//}
		HAL_ADC_Start_DMA(&hadc3,pwmzkb,1);
		pwmv=(pwmzkb[0]*8000)/4095;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmv);
__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmv);
__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmv);
		HAL_Delay(50);
		HAL_ADC_Start_DMA(&hadc1,voltva,1);
		voval=(voltva[0]*3300)/4095;
			voval=voval*1.12732/7.32;
		LCD_SetTextColor(Blue);
				LCD_DisplayStringLine(2,(u8 *)"BLDC speed:");
				LCD_Draw_NUM(70,300,(int)speed);

				LCD_DisplayStringLine(5,(u8 *)"PWM:");
				LCD_Draw_NUM(70+70,300,(int)(pwmv/80));
				LCD_DisplayStringLine(7,(u8 *)"Voltage:");
				LCD_Draw_NUM(70+70+70,300,(int)voval);
	}
			else
			{
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			}
		
		
		
   //__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,6000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef rcvStat;
static float IMU_sensor[9];
static uint8_t Rx_buffer[100] = {0,}; // �?? ?��보�?? ???��?�� 배열
static int USART_CNT = 0; // ???���?? 주기
static uint8_t Rx_data; //1바이?��?�� 받아?��?��?��.
int Rx_indx=0;
static int flag = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	if(huart->Instance == USART6)
	{

		HAL_UART_Receive_IT(&huart6, &Rx_data, 1);
		Rx_buffer[Rx_indx++] = Rx_data;
	}

}
void IMU(void)
{
	int i = 1; //전체적인 순환 변수
	int j =0; // 소수점 전까지 정수 담당 변수
	int k =1; //소수점 후 부터 컴마까지 소수점 담당 변수
	int index=0; //센서 안에 넣을 변수
	double sensor_temp;
	//double pointertemp;
	uint8_t temp[10]={0,};
	for(i; Rx_buffer[i]!=10; i++) //문장?�� ?��?�� 경우 ?��?�� ?��?��?��
	{
		for(j=0; Rx_buffer[i+j]!=46 ; j++) // ?�� ?��?���? ?��까�? �? ?�� 까�??�� 기록?�� temp?�� ???��
		{
			temp[j] = Rx_buffer[i+j];
		}
		sensor_temp = atoi(temp); //?��?���? atoi�? �??��

		for(k = 1; Rx_buffer[i+j+k]!=44 && Rx_buffer[i+j+k]!=13; k++) //?��?��?�� 추�??��?�� for�?
		{
			if (temp[0] == 45) // 음수일 경우
				{
				//pointertemp = pow(0.1, k) * (Rx_buffer[i+j+k] - 48);
				sensor_temp = sensor_temp - pow(0.1, k) * (Rx_buffer[i+j+k] - 48);
				}
			else // 양수일경우
				{
				//pointertemp = pow(0.1, k) * (Rx_buffer[i+j+k] - 48);
				sensor_temp = sensor_temp + pow(0.1, k) * (Rx_buffer[i+j+k] - 48);
				}

		}
		i = i+j+k ;
		for(int temp_size=0; temp_size<sizeof(temp);temp_size++) // 배열 초기화
		{
			temp[temp_size] = 0;
		}
		IMU_sensor[index++] = sensor_temp;

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	double Sensor1;
	double Sensor2;
	uint8_t star[2]={0X2A,};
	uint8_t str[10]={0,};

	if(htim->Instance == TIM1)
	{
		USART_CNT++;

		if(USART_CNT == 50)
		{
			
			__enable_irq();
			if(Rx_indx ==0)
			{

				for(int i=0;i<99;i++)
				{
					Rx_buffer[i] =0X00;
				}

			}
			HAL_UART_Transmit(&huart6, star, 1, 1000);
			IMU();
		}

		if(USART_CNT == 100)

		{
			USART_CNT = 0;
			Rx_indx=0;
			
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_UART_Transmit(&huart2, Rx_buffer,100,1000);
			__disable_irq();





		}
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
  HAL_UART_Receive_IT (&huart6, &Rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

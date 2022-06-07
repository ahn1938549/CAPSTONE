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
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int TimerCnt = 0;           // 1�???????????? 주기 USART

static uint8_t Rx_data;             // USART2 ?��?��?�� 받아?��?�� 버퍼
static char Rx_buffer[100]= {0,};   // USART2 버퍼
static int Rx_indx = 0;				// Rx_buffer?�� 배열 ?�� 조정 ?��?�� ?��?��?��

static int SONIC_FLAG = 0;          // 초음?�� 배열 ?��?��?��
static int SONIC_BUFFER[3] = {0,};  // 초음?�� 계산 �???????????? ???�� �?????????????�� 3번이 진짜 �???????????? ???��

static float IMU_sensor[9] = {0,};  // IMU ?��?�� �?????????????��

volatile double g_Ppre = 0; // ?��?�� Position
volatile double g_Pcur = 0; // ?��?�� Position
volatile double g_Pdes = 0.; // 목표 Position
volatile double g_Perr = 0; // ?��?��
volatile double g_Pvcur = 0;
volatile double g_Ppre_err = 0;
volatile double g_Perr_dif = 0;

// Velocity
volatile double g_Vpre = 0; // ?��?�� ?��?��
volatile double g_Vcur = 0; // ?��?�� ?��?��
volatile double g_Vdes = 0; // 목표 ?��?��
volatile double g_Verr = 0; // ?��?��
volatile double g_Verr_sum = 0;
volatile double g_Vlimit = 1.; // ?��?�� ?��?��

// Current

volatile double g_Cdes = 0;
volatile double g_Cerr[4] = {0,};
volatile double cur_control[4] = {0,};
volatile double g_Cerr_sum[4] = {0,}; // ?��?�� ?�� for I Control
volatile double g_Climit = 1.; // ?��?�� ?���??????

volatile double g_vel_control = 0;
volatile double g_pos_control = 0;
volatile unsigned char g_TimerCnt = 0; // For ?��?��주기

volatile double pos_P = 1;
volatile double pos_D = 0.1;
volatile double vel_P = 2;
volatile double vel_I = 0.1;
volatile double cur_P = 1.0;
volatile double cur_I = 0.1;

static _Bool dir_1 = 0;
static _Bool dir_2 = 0;
static _Bool dir_3 = 0;
static _Bool dir_4 = 0;

static uint16_t Motor_CCR_9_ch1 = 3000;
static uint16_t Motor_CCR_9_ch2 = 0;
static uint16_t Motor_CCR_10 = 0;
static uint16_t Motor_CCR_11 = 0;

// encoder
static int32_t encoder_cnt_2;
static int32_t encoder_cnt_3;
static int32_t encoder_cnt_4;
static int32_t encoder_cnt_5;

static int32_t encoder_pre_cnt_2;
static int32_t encoder_pre_cnt_3;
static int32_t encoder_pre_cnt_4;
static int32_t encoder_pre_cnt_5;

static int32_t encoder_err_cnt_2;
static int32_t encoder_err_cnt_3;
static int32_t encoder_err_cnt_4;
static int32_t encoder_err_cnt_5;

static int16_t cnt_2;
static int16_t cnt_3;
static int16_t cnt_4;
static int16_t cnt_5;
// radian
static float rad_2;
static float rad_3;
static float rad_4;
static float rad_5;

static float old_rad_2 = 0;
static float old_rad_3 = 0;
static float old_rad_4 = 0;
static float old_rad_5 = 0;
// degree
static float deg_2;
static float deg_3;
static float deg_4;
static float deg_5;
// speed
static float speed_2;
static float speed_3;
static float speed_4;
static float speed_5;

static int old_speed_2 = 0;
static int old_speed_3 = 0;
static int old_speed_4 = 0;
static int old_speed_5 = 0;
// acceleration
static int ac_2;
static int ac_3;
static int ac_4;
static int ac_5;

static float dt = 0.0001;
static float pi = 3.14;
////////////////////////////////////Current Fir Filter
static float current_x[4][4];
static float current_y[4];
static uint16_t adc_read[4] = {0,};


float floatFIR(float* x)
{
    float y = 0.0;
    float coef[4] = {0.3, 0.2, 0.2, 0.3};
    for (int i = 0 ; i <=4 ; i++)
    {
         y = y + (coef[i] * x[i]);
    }
    return y;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // 초음?�� ?��?��
{
	if(htim->Instance == TIM1)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			SONIC_BUFFER[SONIC_FLAG] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // 초음?�� �???????????? 받기 /58?? 거리계산
			if(SONIC_FLAG ==1)
			{
				SONIC_FLAG = 0;

				if(SONIC_BUFFER[0] > SONIC_BUFFER[1])                                      // 만약 값이 65536?�� ?��?���???????????? 경우 ?��?��처리
				{
					SONIC_BUFFER[2] = (TIM1->ARR - SONIC_BUFFER[0] + SONIC_BUFFER[1]) / 58;
				}
				else                                                                       // ?��?��처리 ?��?��?��?�� ?��?��
					SONIC_BUFFER[2] = (SONIC_BUFFER[1] - SONIC_BUFFER[0]) / 58;
			}
			else
			{
				SONIC_FLAG = 1;
			}
		}
	}

	/*if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
			encoder_cnt_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			}

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
			encoder_pre_cnt_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			}
	}*/

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //USART6�???????????? IMU ?��?�� �???????????? 받아?��?�� �????????????
{

	if(huart->Instance == USART6)
	{
		HAL_UART_Receive_IT(&huart6, &Rx_data, 1);  //IMU�???????????? ?��?��?�� ?��?��줍니?��.
		Rx_buffer[Rx_indx++] = Rx_data;
	}
}
void IMU(void)
{
	int i = 1; //?��반적?�� 배열 ?��?�� �?????????????��
	int j =0; // IMU?�� ?��?�� �????????????�????????????
	int k =1; // IMU?�� ?��?�� �????????????�????????????
	int index=0; // IMU ?��?�� 값의 ?��?��?��
	double sensor_temp;
	uint8_t temp[10]={0,};
	for(i; Rx_buffer[i]!=10; i++) //문장 ?��반적?�� �????????????�????????????
	{
		for(j=0; Rx_buffer[i+j]!=46 ; j++) // .?�� 만나�???????????? ?��까�? ?��?���????????????�????????????
		{
			temp[j] = Rx_buffer[i+j];
		}
		sensor_temp = atoi(temp); // ?��?�� �????????????�???????????? atoi ?��?��

		for(k = 1; Rx_buffer[i+j+k]!=44 && Rx_buffer[i+j+k]!=13; k++) //,�????????????�???????????? �???????????? \n 만나�???????????? ?��까�? 받아?��?�� �????????????�????????????
		{
			if (temp[0] == 45) // ?��?��?�� 경우
				{
				sensor_temp = sensor_temp - pow(0.1, k) * (Rx_buffer[i+j+k] - 48);
				}
			else // ?��?��?�� 경우
				{
				sensor_temp = sensor_temp + pow(0.1, k) * (Rx_buffer[i+j+k] - 48);
				}
		}
		i = i+j+k;
		for(int temp_size=0; temp_size<sizeof(temp);temp_size++) // 받아?��?�� 배열 초기?��
		{
			temp[temp_size] = 0;
		}
		IMU_sensor[index++] = sensor_temp; // ?��?���???????????? ???��
	}
}

void SetDutyCW(){ // ?���???? 출력 값을

	//while(TCNT1  == 0);

	Motor_CCR_9_ch1 = cur_control[1] / 24 + 500; // ?��?��?�� ?��?�� OCR �???? 처리
	Motor_CCR_9_ch2 = cur_control[1] / 24 + 500;
	Motor_CCR_10 = cur_control[1] / 24 + 500;
	Motor_CCR_11 = cur_control[1] / 24 + 500;
	// OCR_MAX : 390
	// OCR_MIN : 10
	if(Motor_CCR_9_ch1 > 4200)	Motor_CCR_9_ch1 = 4200;
	//else if(Motor_CCR_9_ch1 < 10)	Motor_CCR_9_ch1 = 10;
	//OCR1A = OCR1B = ocr;


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	char star[2]={0X2A,}; // IMU ?���???????????? 모드
	uint8_t str[20]={0,}; // 초음?�� ?��?�� 받아?��?�� 모드
	//////////////////////////////////FIR FIlter

	encoder_cnt_2 = TIM2->CNT;
	encoder_cnt_3 = TIM3->CNT;
	encoder_cnt_4 = TIM4->CNT;
	encoder_cnt_5 = TIM5->CNT;

	/*for(int i =0; i<4; i++)
	{
		current_x[0][i]=adc_read[0];
		current_x[1][i]=adc_read[1];
		current_x[2][i]=adc_read[2];
		current_x[3][i]=adc_read[3];

		current_y[i] = floatFIR(current_x[i]);
		current_y[i] = (current_y[i] - 2048) * 9.4 / 4096 * 10;
	}*/


    /////////////////////////////////////////////////////////////////////////////////////////////////////// encoder

   old_rad_2 = rad_2;
   //encoder_pre_cnt_2 = encoder_cnt_2;

   old_rad_3 = rad_3;
   //encoder_pre_cnt_3 = encoder_cnt_3;

   old_rad_4 = rad_4;
   //encoder_pre_cnt_4 = encoder_cnt_4;

   old_rad_5 = rad_5;
   //encoder_pre_cnt_5 = encoder_cnt_5;


   encoder_err_cnt_2 = encoder_cnt_2 - encoder_pre_cnt_2;
   encoder_err_cnt_3 = encoder_cnt_3 - encoder_pre_cnt_3;
   encoder_err_cnt_4 = encoder_cnt_4 - encoder_pre_cnt_4;
   encoder_err_cnt_5 = encoder_cnt_5 - encoder_pre_cnt_5;

   //if (encoder_cnt_2 > 65000 && encoder_pre_cnt_2 < 500)  encoder_err_cnt_2 -= 65535;
   //else if (encoder_cnt_2 < 500 && encoder_pre_cnt_2 > 65000)  encoder_err_cnt_2 += 65535;

   //if (encoder_cnt_3 > 65000 && encoder_pre_cnt_3 < 500)  encoder_err_cnt_3 -= 65535;
   //else if (encoder_cnt_3 < 500 && encoder_pre_cnt_3 > 65000)  encoder_err_cnt_3 += 65535;

   if (encoder_cnt_4 > 65000 && encoder_pre_cnt_4 < 500)  encoder_err_cnt_4 -= 65535;
   else if (encoder_cnt_4 < 500 && encoder_pre_cnt_4 > 65000)  encoder_err_cnt_4 += 65535;

   if (encoder_cnt_5 > 65000 && encoder_pre_cnt_5 < 500)  encoder_err_cnt_5 -= 65535;
   else if (encoder_cnt_5 < 500 && encoder_pre_cnt_5 > 65000)  encoder_err_cnt_5 += 65535;

   ///////////////////////////////////////////////////////////////////////////////////////////////// rad and deg and speed

   rad_2 += (encoder_err_cnt_2 / 3600.) * 2. * pi;
   speed_2 = (rad_2 - old_rad_2) / dt;

   rad_3 += (encoder_err_cnt_3 / 3600.) * 2. * pi;
   speed_3 = (rad_3 - old_rad_3) / dt;

   rad_4 += (encoder_err_cnt_4 / 3600.) * 2. * pi;
   speed_4 = (rad_4 - old_rad_4) / dt;

   rad_5 += (encoder_err_cnt_5 / 3600.) * 2. * pi;
   speed_5 = (rad_5 - old_rad_5) / dt;

   deg_2 = rad_2 * 180 / pi;
   deg_3 = rad_3 * 180 / pi;
   deg_4 = rad_4 * 180 / pi;
   deg_5 = rad_5 * 180 / pi;

   //////////////////////////////////////////////////////////////////////////////////////////////////// Timer Pulse

   TIM9->CCR1 = Motor_CCR_9_ch1;                       // motor 1 _ Duty  ??? 조절  ????
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, !dir_1);   // pin ?  버링, pin 번호, 방향?  ?   (set/reset)

   TIM9->CCR2 = Motor_CCR_9_ch2;                       // motor 2 _ Duty  ??? 조절  ????
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, dir_2);  // pin ?  버링, pin 번호, 방향?  ?   (set/reset)

   TIM10->CCR1 = Motor_CCR_10;                          // motor 3 _ Duty  ??? 조절  ????
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, !dir_3); // pin ?  버링, pin 번호, 방향?  ?   (set/reset)

   TIM11->CCR1 = Motor_CCR_11;                         // motor 4 _ Duty  ??? 조절  ????
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, dir_4); // pin ?  버링, pin 번호, 방향?  ?   (set/reset)

	if(htim->Instance == TIM1)
	{

		TimerCnt++;
		//UASRT
		if(TimerCnt == 10000)
		{
			TimerCnt = 0;
			//Rx_indx=0;
			//IMU();
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			sprintf(str,"SONIC %06d \n", adc_read[0]);
		}

		if((TimerCnt % 100) == 0)  // ?��?��주기 : 0.01 �??
		{

			g_Vcur = (speed_5 - g_Ppre) / 0.01;
			g_Ppre = speed_2;
			g_Verr = g_Vdes - g_Vcur;

			g_Verr_sum += g_Verr; // ?��?�� ?��분값
			vel_P = 1;
			vel_I = 1;
			g_vel_control = g_Verr * vel_P + g_Verr_sum * vel_I * 0.01;
		}

		if(TimerCnt % 10 == 0) // ?��?�� 주기 : 0.001�?????
		{
			g_Cdes = 4000;
			cur_P = 1.8;
			cur_I = 0.18;
			SetDutyCW();
			 // ?��?��
			for(int i =0; i <4; i++) //////Filtered Current Sesnor
			{

				g_Cerr[i] = g_Cdes - current_y[i];
				g_Cerr_sum[i] = g_Cerr_sum[i] + g_Cerr[i]; // ?��?��?��
				cur_control[i] = g_Cerr[i] * cur_P + g_Cerr_sum[i] * cur_I * 0.001;

			}

		}




		//?��류센?��


		/*if(USART_CNT == 50)
		{
			if(Rx_indx ==0)
			{

				for(int i=0;i<99;i++)
				{
					Rx_buffer[i] =0X00;
				}

			}
			HAL_UART_Transmit(&huart6, star, 1, 1000);

		}*/

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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM9_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start_IT(&htim9, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim9, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);             // MOTOR 2
  HAL_TIMEx_PWMN_Start(&htim9, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);            // MOTOR 3
  HAL_TIMEx_PWMN_Start(&htim10, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);            // MOTOR 4
  HAL_TIMEx_PWMN_Start(&htim11, TIM_CHANNEL_1);

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);

  HAL_UART_Receive_IT (&huart6, &Rx_data, 1);

  HAL_ADC_Start_DMA(&hadc1, &adc_read[0], 4);


  //
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2- 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3600 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3600 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3600 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3600 - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4200 - 1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4200 - 1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 4200 - 1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1050;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Dir1_Pin|Dir4_Pin|Dir3_Pin|Dir2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Dir1_Pin Dir4_Pin Dir3_Pin Dir2_Pin */
  GPIO_InitStruct.Pin = Dir1_Pin|Dir4_Pin|Dir3_Pin|Dir2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

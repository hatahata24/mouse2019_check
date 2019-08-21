/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include <string.h>
#include "math.h"

#define MAIN_C_
#include "global.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DIAMETER 24.2//22.5//23.4
#define TREAD 66
#define sensor_wait 3000
#define log_allay 200

//gyro
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define GYRO_FACTOR 16.4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

float cnt_r = 0;
float dist_r = 0;
float speed_r = 0;
float cnt_l = 0;
float dist_l = 0;
float speed_l = 0;

int mode = 0;
int cnt = 0;
int get_cnt = 0;

float target_speed_min_l = 0;
float target_speed_max_l = 0;
float target_speed_min_r = 0;
float target_speed_max_r = 0;
float pulse_l, pulse_r;

float accel_l = 0;
float accel_r = 0;
float target_speed_l = 0;
float target_speed_r = 0;

float degree_z = 0;
float target_degree_z = 0;
float target_dist = 0;

int get_speed_l[log_allay];
int get_speed_r[log_allay];

int value1, value2, value3, value4;

float epsilon_sum = 0; 	//dif sum
float old_epsilon = 0; 	//
float epsilon_dif = 0;	//dif of dif
float epsilon_l = 0; 	//dif between target & current
float epsilon_r = 0;	//dif between target & current
float Kp = 4;
float Ti = 1000;
float Td = 0;

//#define NUMBER_OF_VRS 4
//static uint16_t vr_values[NUMBER_OF_VRS];

//#define ADC_CONVERTED_DATA_BUFFER_SIZE ((uint32_t) 4)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void buzzer(int, int);
int get_adc_value(ADC_HandleTypeDef*, uint32_t);
void icm20689_init(void);
uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t val);
float icm20689_read_gyro_z(void);
void drive_dir(uint8_t, uint8_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int c){
  if(c == '\n'){
    int _c = '\r';
    HAL_UART_Transmit(&huart1, &_c, 1, 1);
  }
  HAL_UART_Transmit(&huart1, &c, 1, 1);
  return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	TIM_OC_InitTypeDef ConfigOC;
	ConfigOC.OCMode = TIM_OCMODE_PWM1;
	ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if(htim == &htim6){
/*	  ledOn ++;
	  ledOn = ledOn % 2;
	  switch(ledOn){
	  case 0:
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);   //
		  break;

	  case 1:
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);   //
		  break;
	  }
*/

		cnt_l = TIM4 -> CNT;
		cnt_r = TIM8 -> CNT;
		if(cnt_l > 40000) cnt_l = cnt_l - 65535;
		if(cnt_r > 40000) cnt_r = cnt_r - 65535;
		cnt_r = cnt_r * -1;

		dist_l = dist_l + cnt_l * (DIAMETER * M_PI * 11 / 40 / 4096 / 4);
		dist_r = dist_r + cnt_r * (DIAMETER * M_PI * 11 / 40 / 4096 / 4);

		speed_l = cnt_l * (DIAMETER * M_PI * 11 / 40 / 4096 / 4) / 0.001;
		speed_r = cnt_r * (DIAMETER * M_PI * 11 / 40 / 4096 / 4) / 0.001;

		TIM4 -> CNT = 0;
		TIM8 -> CNT = 0;

		if(MF.FLAG.DRV){
			target_speed_l += accel_l * 0.001;
			target_speed_l = max(min(target_speed_l, target_speed_max_l), target_speed_min_l);
			epsilon_l = target_speed_l - speed_l;
			pulse_l = Kp * epsilon_l;
			if(pulse_l > 0){
				drive_dir(0, 0);
				ConfigOC.Pulse = pulse_l;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			}
			else if(pulse_l < 0){
				drive_dir(0, 1);
				ConfigOC.Pulse = -pulse_l;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			}

			target_speed_r += accel_r * 0.001;
			target_speed_r = max(min(target_speed_r, target_speed_max_r), target_speed_min_r);
			epsilon_r = target_speed_r - speed_r;
			pulse_r = Kp * epsilon_r;
			if(pulse_r > 0){
				drive_dir(1, 0);
				ConfigOC.Pulse = pulse_r;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
			}
			else if(pulse_r < 0){
				drive_dir(1, 1);
				ConfigOC.Pulse = -pulse_r;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
			}
			if(cnt >= 5 && MF.FLAG.LOG){
				cnt = 0;
				if(get_cnt < log_allay){
					get_speed_l[get_cnt] = speed_l;
					get_speed_r[get_cnt] = speed_r;
					get_cnt++;
				}
			}
		}else{
			drive_dir(0, 3);
			drive_dir(1, 3);
		}

		//gyro interrupt
		degree_z += icm20689_read_gyro_z() * 0.001;

		if(MF.FLAG.GYR){
			target_dist = TREAD*M_PI/360*(degree_z-target_degree_z);
			if(target_dist > 0){
				target_speed_l = sqrt(2*accel_l*target_dist);
				target_speed_r = -1 * target_speed_l;
			}else{
				target_speed_l = sqrt(2*accel_l*target_dist*-1)*-1;
				target_speed_r = -1 * target_speed_l;
			}

			epsilon_l = target_speed_l - speed_l;
			pulse_l = Kp * epsilon_l;
			if(pulse_l > 0){
				drive_dir(0, 0);
				ConfigOC.Pulse = pulse_l;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			}
			else if(pulse_l < 0){
				drive_dir(0, 1);
				ConfigOC.Pulse = -pulse_l;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
			}

			epsilon_r = target_speed_r - speed_r;
			pulse_r = Kp * epsilon_r;
			if(pulse_r > 0){
				drive_dir(1, 0);
				ConfigOC.Pulse = pulse_r;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
			}
			else if(pulse_r < 0){
				drive_dir(1, 1);
				ConfigOC.Pulse = -pulse_r;
				HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
			}
		}


		//ADchange interrupt
		uint16_t delay = 0;
		cnt++;
		mode = (mode+1)%2;

		switch(mode){
		  case 0:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); 	//L
				for(delay=0; delay<sensor_wait; delay++);
				value4 = get_adc_value(&hadc1, ADC_CHANNEL_3);			//L
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   	//R
				for(delay=0; delay<sensor_wait; delay++);
				value2 = get_adc_value(&hadc1, ADC_CHANNEL_1);			//R
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			break;

		  case 1:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  	//FL
				for(delay=0; delay<sensor_wait; delay++);
				value3 = get_adc_value(&hadc1, ADC_CHANNEL_2);			//FL
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   	//FR
				for(delay=0; delay<sensor_wait; delay++);
				value1 = get_adc_value(&hadc1, ADC_CHANNEL_0);			//FR
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  icm20689_init();

  printf("Welcome to WMMC !\n");

  int val = 0;

  setbuf(stdout, NULL);

  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);

  int pulse = 0;

  TIM_OC_InitTypeDef ConfigOC;
  ConfigOC.OCMode = TIM_OCMODE_PWM1;
  ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	int cnt_r2, cnt_l2, dist_r2, dist_l2, speed_r2, speed_l2;
	cnt_r2 = cnt_r * 10;
	cnt_l2 = cnt_l * 10;
	dist_r2 = dist_r * 10;
	dist_l2 = dist_l * 10;
	speed_r2 = speed_r * 10;
	speed_l2 = speed_l * 10;

/*LED check
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    HAL_Delay(1000);
*/

/*full color LED check
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_Delay(1000);
*/

/*push switch check
    if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET ) {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    } else {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
*/

/*mode select
    if(mode & 0b001){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    if(mode & 0b010){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    if(mode & 0b100){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }


    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
      HAL_Delay(100);
      while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
      mode++;
      if(mode > 7){
        mode = 0;
      }
      printf("Mode : %d\n", mode);
    }

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
      HAL_Delay(1000);
      while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
      switch(mode){
        case 0:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 1:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

        case 2:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

        case 3:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

        case 4:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 5:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 6:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 7:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

      }
    }
*/

/*sensor ON&OFF
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_Delay(1000);
*/

/*AD change check
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100); //ADC
	val = HAL_ADC_GetValue(&hadc1);

	printf("%d\n",val);
	HAL_Delay(10);
	mode++;
	mode = mode%2;

	switch(mode){
	  case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   //FR
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //R
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  //L
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  //FL
		break;

	  case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		break;
	}
*/

/*AD change check
	HAL_Delay(1);

	mode++;
	cnt++;
	mode = mode%2;

	switch(mode){
	  case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   //FR
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //R
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  //L
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  //FL
		break;

	  case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		break;
	}

	value1 = get_adc_value(&hadc1, ADC_CHANNEL_0);	//FR
	value2 = get_adc_value(&hadc1, ADC_CHANNEL_1);	//R
	value3 = get_adc_value(&hadc1, ADC_CHANNEL_2);	//FL
	value4 = get_adc_value(&hadc1, ADC_CHANNEL_3);	//L
	if(cnt >= 101){
		printf("FR:%3d, R:%3d, FL:%3d, L:%3d\n", value1, value2, value3, value4);
		cnt = 0;
	}
*/

/*AD change interrupt check
if(cnt >= 101){
	printf("FR:%3d, R:%3d, FL:%3d, L:%3d\n", value1, value2, value3, value4);
	cnt = 0;
}
*/

/*AD change x4??
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   //FR
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //R
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  //L
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  //FL

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100); //ADC

    value1 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100); //ADC

    value2 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100); //ADC

    value3 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100); //ADC

    value4 = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    printf("FR:%4d,  R:%4d,  FL:%4d,  L:%4d \n", value1, value2, value3, value4);

    HAL_Delay(100);
*/

/*AD change DMA
	printf("FR: %u, R: %u, FL: %u, L: %u\n\n", g_ADCBuffer[0], g_ADCBuffer[1], g_ADCBuffer[2], g_ADCBuffer[3]); //FR
	//printf("R: %d\n", g_ADCBuffer[1]); //R
	//printf("FL: %d\n", g_ADCBuffer[2]); //FL
	//printf("L: %d\n", g_ADCBuffer[3]); //L

    mode++;
    mode = mode%2;

    switch(mode){
      case 0:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   //FR
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   //R
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  //L
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);  //FL
        printf("ON \n");
        break;

      case 1:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        printf("OFF \n");
        break;
    }

	HAL_Delay(500);
*/

/*Timer LED ON&OFF
 * Written in Interrupt
 */

/*encoder R check
	cnt = TIM8 -> CNT;

	printf("%d\n", cnt);

	HAL_Delay(100);
*/

/*encoder L check
    cnt = TIM4 -> CNT;

    printf("%d\n", cnt);

    HAL_Delay(100);
*/

/*encoder R mode select
    cnt = TIM8 -> CNT;

    printf("%d\n", cnt);

    HAL_Delay(100);

    if(mode & 0b001){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    if(mode & 0b010){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    if(mode & 0b100){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    }else{
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    val = cnt - cnt_old;

    if(val >= 40000){
      mode++;
      if(mode > 7){
        mode = 0;
      }
      printf("Mode : %d\n", mode);
    }
    if(val <= -40000){
      mode--;
      if(mode < 0){
        mode = 7;
      }
      printf("Mode : %d\n", mode);
    }

    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
      HAL_Delay(1000);
      while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
      switch(mode){
        case 0:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 1:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

        case 2:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

        case 3:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

        case 4:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 5:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 6:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          break;

        case 7:
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          break;

      }
    }

    cnt_old = cnt;

    //HAL_TIM_Encorder_Stop(&htim4, TIM_CHANNEL_ALL);
*/

/*Timer Encoder Count
	printf("L  cnt:%3d, dist: %3d, speed:%3d\n", cnt_l2, dist_l2, speed_l2);
	printf("R  cnt:%3d, dist: %3d, speed:%3d\n", cnt_r2, dist_r2, speed_r2);

	HAL_Delay(333);
*/

/*PWM check
    pulse = pulse + 3000;

    ConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    ConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    if(pulse >= 60000)
    {
      pulse = 0;
    }

    HAL_Delay(100);
*/

/*Motor R rotate forward & back

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//R_CCW

    ConfigOC.Pulse = 100;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    printf("PWM 10% \n");

    HAL_Delay(500);

    ConfigOC.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    printf("PWM 0% \n");

    HAL_Delay(500);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);		//R_CW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW

    ConfigOC.Pulse = 100;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    printf("PWM -10% \n");

    HAL_Delay(500);

    ConfigOC.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    printf("PWM 0% \n");

    HAL_Delay(500);
*/

/*Motor L rotate forward & back

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//L_CCW

	ConfigOC.Pulse = 100;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM 10% \n");

	HAL_Delay(500);

	ConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM 0% \n");

	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	//R_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//R_CCW

	ConfigOC.Pulse = 100;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM -10% \n");

	HAL_Delay(500);

	ConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM 0% \n");

	HAL_Delay(500);
*/

/*Motor R&L rotate forward & back

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//R_CCW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//L_CCW

	ConfigOC.Pulse = 100;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM 10% \n");

	HAL_Delay(500);

	ConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM 0% \n");

	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	//R_CW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//L_CCW

	ConfigOC.Pulse = 100;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM -10% \n");

	HAL_Delay(500);

	ConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM 0% \n");

	HAL_Delay(500);
*/

/*Motor R rotate PWM
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//R_CCW

    pulse = pulse + 100;

    ConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    printf("PWM %4d \n", pulse);

    HAL_Delay(500);

    if(pulse >= 500)
    {
      pulse = 0;
    }
*/

/*Motor L rotate PWM
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//L_CCW

	pulse = pulse + 100;

	ConfigOC.Pulse = pulse;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM %4d \n", pulse);

	HAL_Delay(500);

	if(pulse >= 500)
	{
		pulse = 0;
	}
*/

/*Motor R&L rotate PWM
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//R_CCW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//L_CCW

	pulse = pulse + 100;

	ConfigOC.Pulse = pulse;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	ConfigOC.Pulse = pulse;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	printf("PWM %4d \n", pulse);

	HAL_Delay(500);

	if(pulse >= 500)
	{
	pulse = 0;
	}
*/

/*Motor R&L rotate certain distance
	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//L_CCW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//R_CCW

	pulse = 50;

	ConfigOC.Pulse = pulse;
	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	HAL_TIM_PWM_ConfigChannel(&htim2, &ConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	while(dist_l < 150 && dist_r < 150);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//L_CCW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	//R_CW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW

	while(dist_l > 0 && dist_r > 0);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//L_CCW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW

	while(1);
*/

/*speed control
	MF.FLAG.DRV = 1;
	for(int i = 0; i < 1; i++){
		HAL_Delay(500);
		target_speed_l = 600;
		target_speed_r = 600;
		while(dist_l < 1000 && dist_r < 1000);

		target_speed_l = -200;
		target_speed_r = -200;
		while(dist_l > 0 && dist_r > 0);

		target_speed_l = 0;
		target_speed_r = 0;
	}
	while(1);MF.FLAG.DRV = 0;
*/

//accel & speed control & log get
	HAL_Delay(500);
	for(int i = 0; i < 1; i++){
		MF.FLAG.DRV = 1;
		MF.FLAG.LOG = 1;
		accel_l = 1000;
		accel_r = 1000;
		target_speed_max_l = 500;
		target_speed_max_r = 500;
		while(dist_l < 300 && dist_r < 300);

		accel_l = 1000;
		accel_r = 1000;
		target_speed_max_l = 0;
		target_speed_max_r = 0;
		target_speed_min_l = 0;
		target_speed_min_r = 0;
	}
	MF.FLAG.DRV = 0;

//log print
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);

	for(int i=0; i<log_allay; i++){
		printf("l:	%d\n", get_speed_l[i]);
		HAL_Delay(5);
	}
	for(int i=0; i<log_allay; i++){
		printf("r:	%d\n", get_speed_r[i]);
		HAL_Delay(5);
	}


/*accel & speed control & log get +deaccel
	HAL_Delay(500);
	for(int i = 0; i < 1; i++){
		MF.FLAG.DRV = 1;
		MF.FLAG.LOG = 1;
		accel_l = 1000;
		accel_r = 1000;
		target_speed_max_l = 500;
		target_speed_max_r = 500;
		while(dist_l < 300 && dist_r < 300);

		accel_l = -1000;
		accel_r = -1000;
		target_speed_min_l = -500;
		target_speed_min_r = -500;
		while(dist_l > -300 && dist_r > -300);

		accel_l = 1000;
		accel_r = 1000;
		target_speed_max_l = 0;
		target_speed_max_r = 0;
		target_speed_min_l = 0;
		target_speed_min_r = 0;
	}
	MF.FLAG.DRV = 0;

//log print
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);

	for(int i=0; i<log_allay; i++){
		printf("l:	%d\n", get_speed_l[i]);
		HAL_Delay(5);
	}
	for(int i=0; i<log_allay; i++){
		printf("r:	%d\n", get_speed_r[i]);
		HAL_Delay(5);
	}
*/

/*turn Right
	MF.FLAG.DRV = 1;
	for(int i = 0; i < 4; i++){
		HAL_Delay(500);
	    if(i == 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	    if(i == 1) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	    if(i == 2) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

		target_speed_l = 200;
		target_speed_r = -200;
		while(dist_l < (TREAD*M_PI/4) && dist_r < (TREAD*M_PI/4));

		target_speed_l = 0;
		target_speed_r = 0;
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	    dist_l = 0;
	    dist_r = 0;
	}
	while(1)MF.FLAG.DRV = 0;
*/

/*turn Right (one wheel active)
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
	MF.FLAG.DRV = 1;
	HAL_Delay(500);
	for(int i = 0; i < 1; i++){
		accel_l = 3000;
		accel_r = 0;
		target_speed_max_l = 200;
		target_speed_max_r = 0;
		while(dist_l < (TREAD*2*M_PI/4)*1.1);

		accel_l = -3000;
		target_speed_max_l = 0;
		//dist_l = 0;
		//dist_r = 0;
	}
	//while(1)MF.FLAG.DRV = 1;
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
	dist_r2 = dist_r * 10;
	dist_l2 = dist_l * 10;
	printf("dist_l*10:	%d\n", dist_l2);
	printf("dist_r*10:	%d\n", dist_r2);
*/

/*turn Right (two wheel active)
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
	MF.FLAG.DRV = 1;
	HAL_Delay(500);
	for(int i = 0; i < 1; i++){
		accel_l = 10000;
		accel_r = -10000;
		target_speed_max_l = 200;
		target_speed_min_r = -200;
		while(dist_l < (TREAD*M_PI/4));

		accel_l = -3000;
		accel_r = 3000;
		//dist_l = 0;
		//dist_r = 0;
	}
	//while(1)MF.FLAG.DRV = 1;
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
	dist_r2 = dist_r * 10;
	dist_l2 = dist_l * 10;
	printf("dist_l*10:	%d\n", dist_l2);
	printf("dist_r*10:	%d\n", dist_r2);
*/

/*slalom R
	MF.FLAG.DRV = 1;
	for(int i = 0; i < 8; i++){
		HAL_Delay(500);

		target_speed_l = 200;
		target_speed_r = 200;
		while(dist_l < 10 && dist_r < 10);

		target_speed_l = 400;
		target_speed_r = 50;
	    dist_l = 0;
	    dist_r = 0;
		while((dist_l+dist_r)/2 < (TREAD*M_PI/2.5));

		target_speed_l = 200;
		target_speed_r = 200;
	    dist_l = 0;
	    dist_r = 0;
		while(dist_l < 10 && dist_r < 10);

		target_speed_l = 0;
		target_speed_r = 0;
	    dist_l = 0;
	    dist_r = 0;
	}
	while(1)MF.FLAG.DRV = 0;
*/

/*slalom R2
	MF.FLAG.DRV = 1;
	HAL_Delay(5000);
	for(int i = 0; i < 4; i++){

		accel_l = 3000;
		accel_r = 3000;
		target_speed_max_l = 400;
		target_speed_max_r = 400;
		dist_l = 0;
		dist_r = 0;
		while(dist_l < 110 && dist_r < 110);

		accel_l = 3000;
		accel_r = -3000;
		target_speed_max_l = 589;
		target_speed_min_r = 211;
		dist_l = 0;
		dist_r = 0;
		while((dist_l+dist_r)/2 < 110*1);//1.2

		accel_l = -3000;
		accel_r = 3000;
		target_speed_min_l = 400;
		target_speed_max_r = 400;
		dist_l = 0;
		dist_r = 0;
		while(dist_l < 110 && dist_r < 110);
	}
	while(1)MF.FLAG.DRV = 0;
*/

/*voltage check
	if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	} else {
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
*/

/*buzzer
	buzzer(DO, 500);
	buzzer(LE, 500);
	buzzer(MI, 500);
	buzzer(FA, 500);
	buzzer(SO, 500);
	buzzer(LA, 500);
	buzzer(SI, 500);
	buzzer(DOO, 500);
*/

/*buzzer pitagola
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);

	buzzer(LE, 200);
	buzzer(MI, 200);
	buzzer(RST, 50);
	buzzer(LE, 200);
	buzzer(MI, 200);
	buzzer(RST, 50);
	buzzer(DOO, 200);
	buzzer(SI, 200);
	buzzer(RST, 120);
	buzzer(SO, 250);
	buzzer(RST, 1000);
*/

/*buzzer pitagola 2
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);

	for(int i=0; i<pita; i++){
		buzzer(pitagola[i][0], pitagola[i][1]);
	}
*/

/*gyro who am i check
	uint8_t who_am_i;

	HAL_Delay(100); // wait start up
	who_am_i = read_byte(WHO_AM_I); // 1. read who am i
	printf("WHO_AM_I:	0x%x\n",who_am_i); // 2. check who am i value
	// 2. error check
	if ( who_am_i != 0x98 ){
		while(1){
			printf( "gyro_error\r");
		}
	}
	HAL_Delay(50); // wait
	write_byte(PWR_MGMT_1, 0x00); // 3. set pwr_might

	HAL_Delay(50);
	write_byte(CONFIG, 0x00); // 4. set config

	HAL_Delay(50);
	write_byte(GYRO_CONFIG, 0x18); // 5. set gyro config

	HAL_Delay(50);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
*/

/*gyro z read
	int gyro_z2, degree_z2;
	gyro_z2 = icm20689_read_gyro_z() * 10;
	degree_z2 = degree_z * 10;
	printf("gyro_z*10: %3d, degree*10: %3d\n", gyro_z2, degree_z2);
	HAL_Delay(5);
*/

/*enkaigei
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
	target_degree_z = icm20689_read_gyro_z();
	accel_l = 3000;

	MF.FLAG.GYR = 1;

	while(1);
*/

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Period = 65535;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_13 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC2 
                           PC3 PC4 PC5 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB13 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_13 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void buzzer(int sound, int length){

	TIM_OC_InitTypeDef ConfigOC;
	ConfigOC.OCMode = TIM_OCMODE_PWM1;
	ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	hz = 1000000 / sound;
	TIM3 -> ARR = hz;

    ConfigOC.Pulse = hz / 2;
    HAL_TIM_PWM_ConfigChannel(&htim3, &ConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_Delay(length);

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//get_adc_value
// ??��?��?定されたチャンネルのアナログ電圧値を取り�???��?��??��?��?
// 引数1???��?��??��?��hadc …… 電圧値を取り�???��?��すチャンネルが属すADCのHandler
// 引数2???��?��??��?��channel …… 電圧値を取り�???��?��すチャンネル
// 戻り�???��?��???��?��??��?��電圧値???��?��?12bit??��?��?解能???��?��?
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_adc_value(ADC_HandleTypeDef *hadc, uint32_t channel){

  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  //sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  //sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  HAL_ADC_ConfigChannel(hadc, &sConfig);

  HAL_ADC_Start(hadc);                    // AD変換を開始す??��?��?
  HAL_ADC_PollForConversion(hadc, 100);   // AD変換終�?まで??��?��?機す??��?��?
  return HAL_ADC_GetValue(hadc);          // AD変換結果を取得す??��?��?
}

/*HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	printf("ADC CH0 Value is %d\r\n",aADCxConvertedData[0]);
	printf("ADC CH1 Value is %d\r\n",aADCxConvertedData[1]);
	printf("ADC CH2 Value is %d\r\n",aADCxConvertedData[2]);
	printf("ADC CH3 Value is %d\r\n",aADCxConvertedData[3]);
}
*/

void icm20689_init(void){
  uint8_t who_am_i;

  HAL_Delay(100); // wait start up
  who_am_i = read_byte(WHO_AM_I); // 1. read who am i
  printf("\r\n0x%x\r\n",who_am_i); // 2. check who am i value

  // 2. error check
  if (who_am_i != 0x98){
    while(1){
      printf( "gyro_error\r");
    }
  }

  HAL_Delay(50); // wait
  write_byte(PWR_MGMT_1, 0x00); // 3. set pwr_might

  HAL_Delay(50);
  write_byte(CONFIG, 0x00); // 4. set config

  HAL_Delay(50);
  write_byte(GYRO_CONFIG, 0x18); // 5. set gyro config

  HAL_Delay(50);
}

uint8_t read_byte(uint8_t reg){
  uint8_t ret,val;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET ); //cs = Low;
  ret = reg | 0x80;  // MSB = 1
  HAL_SPI_Transmit(&hspi3, &ret,1,100); // sent 1byte(address)
  HAL_SPI_Receive(&hspi3,&val,1,100); // read 1byte(read data)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET );  //cs = High;
  return val;
}

void write_byte(uint8_t reg, uint8_t val){
  uint8_t ret;
  ret = reg & 0x7F ; // MSB = 0
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // cs = Low;
  HAL_SPI_Transmit(&hspi3, &ret,1,100); // sent 1byte(address)
  HAL_SPI_Transmit(&hspi3, &val,1,100); // read 1byte(write data)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // cs = High;
}

float icm20689_read_gyro_z(void){
  int16_t gyro_z;
  float omega;

  // H:8bit shift, Link h and l
  gyro_z = (int16_t)((int16_t)(read_byte(GYRO_ZOUT_H) << 8) | read_byte(GYRO_ZOUT_L));

  omega = (float)(gyro_z / GYRO_FACTOR + 1.15); // dps to deg/sec
  return omega;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_dir
// wheel turn dir for each wheel
// hikisuu:1-wheel select(0=>L, 1=>R), 2-dir select(0=>CW, 1=>CWW, 2=>ShortBrake, 3=>free)
// modorichi: nothing
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_dir(uint8_t wheel, uint8_t dir){
	if(wheel == 0){
		if(dir == 0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);	//L_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 1){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);	//L_CW
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//L_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 2){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);		//L_CW
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);		//L_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);	//STBY
		}
	}else{
		if(dir == 0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//R_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 1){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);	//R_CW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else if(dir == 2){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);		//R_CW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//R_CCW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);		//STBY
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);	//STBY
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

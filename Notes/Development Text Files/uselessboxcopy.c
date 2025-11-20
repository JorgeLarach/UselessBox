/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "trapqueen.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* -- PWM Explanation --
 * htim2 handles Servo PWM
 * MG995 Servo requires PWM frequency of 20ms (50 hz)
 * Using Period of 10,000 and Prescaler of 168 achieves this
 * 0.5ms moves to 0 degrees, and 2.5ms moves to 180 degrees
 * Therefore, min pulse width is 250 and max is 1250
 *
 * htim3 handles RGB LED PWM (Red-Green-Blue Light-Emitting-Diode Pulse-Width-Modulation)
 * Using Period of 255 and Prescaler of 80 (don't remember why this prescaler but whatever)
 * Channels 1, 2, and 3 correspond to Red, Green, and Blue
 *
 * htim4 handles speaker playback. WAV file array is 8 bit unsigned at 8000hz
 * 1 second/8000 = 125us. Too small for HAL_Delay, so using DWT to delay this amt of time
 * Using Period of 255 and Prescaler of 0 (~328kHz PWM freq, really really fast. Good for speaker interfacing)
 * Period of 255 bc each sample is at most 255 (8bit unsigned). Already mapped duty cycle.
 * Could use interrupts instead of DWT (decouple sample timing from CPU), but this seemed simpler
 * */

/* -- Used GPIO Pins --
 * PA0 - PWM for Lid servo
 * PA1 - PWM for Arm servo
 * PA6 - PWM for Red channel
 * PA7 - PWM for Green channel
 * PB0 - PWM for Blue channel
 * PB6 - PWM for Speaker playback
 * PA8 - Power/interrupt pin for useless box switch
 * */

#define MIN_SERVO_PW             250  // 0.5ms duty cycle, 0 degrees
#define MAX_SERVO_PW             1250 // 2.5s  duty cycle, 180 degrees
#define MAX_ROTATION_DEGREES     180  // Max angle a servo can rotate
#define LID_MAX_ANGLE			 35   // Lift lid no more than these degrees
#define ARM_MAX_ANGLE			 170  // Lift arm no more than these degrees
#define DEBOUNCE_TIME_MS         100  // Switch debounce safety wait
#define HUE_UPDATE_INTERVAL      50   // Increment hue every 50 iterations in main
#define SPEAKER_SAMPLE_DELAY_US  106  //
#define AUDIO_SAMPLE_RESET_INDEX 44   // Starting index for audio sample wrap-around

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t movement_requested = 0; // Binary semaphore that triggers UselessBox_StandardRoutine()
uint32_t current_time, previous_time;    // Handles switch debouncing in ISR
uint16_t R, G, B, H = 0;                 // Handles LED cycling
uint32_t hue_update_counter = 0;
uint32_t audio_sample_index = 44;        // Music index. Skips WAV header (first 43 elements)

extern const uint8_t trapqueencut_wav[];
extern const unsigned int trapqueencut_wav_len;

typedef enum {
    LID = 0, // PA0, htim2 channel 1
    ARM = 1  // PA1, htim2 channel 2
} ServoID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* Routine Handling */
void UselessBox_RoutineDecider();
void UselessBox_StandardRoutine();
void UselessBox_AngryRoutine();
void UselessBox_WaveringRoutine();

/* Servo Handling */
uint32_t Servo_SetAngle(ServoID id, uint8_t angle);
uint32_t Servo_CalculatePulseWidth(uint8_t angle);
void     Servo_SweepAngle(ServoID id, uint8_t angle1, uint8_t angle2, uint32_t delay_ms);

/* Speaker Handling */
void Speaker_OutputSample(uint8_t freq);
void Speaker_DWT_Init(void);
void Speaker_DWTDelay_us(uint32_t us_delay);

/* RGB LED Handling */
void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue);
void HSV_to_RGB(uint16_t H, uint16_t *R, uint16_t *G, uint16_t *B);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UselessBox_StandardRoutine(){
	Servo_SweepAngle(LID, 0, LID_MAX_ANGLE, 2); // Open lid
	Servo_SweepAngle(ARM, 0, ARM_MAX_ANGLE, 2); // Swing out arm
	HAL_Delay(200);
	Servo_SweepAngle(ARM, ARM_MAX_ANGLE, 0, 2); // Return arm
	Servo_SweepAngle(LID, LID_MAX_ANGLE, 0, 2); // Close lid
}

void UselessBox_AngryRoutine(){
	Servo_SweepAngle(LID, 0, LID_MAX_ANGLE, 2); // Open lid
	Servo_SweepAngle(ARM, 0, ARM_MAX_ANGLE, 2); // Swing out arm
	for(uint8_t i = 0; i < 7; i++){
		Servo_SweepAngle(LID, LID_MAX_ANGLE, 0, 2); // Close lid
		HAL_Delay(100);
		Servo_SweepAngle(LID, 0, LID_MAX_ANGLE, 2); // Open lid
		HAL_Delay(100);
	}
	HAL_Delay(100);
	Servo_SweepAngle(ARM, ARM_MAX_ANGLE, 0, 2); // Return arm
	Servo_SweepAngle(LID, LID_MAX_ANGLE, 0, 2); // Close lid
}

void UselessBox_WaveringRoutine(){
	Servo_SweepAngle(LID, 0, LID_MAX_ANGLE, 10); // Open lid
	Servo_SweepAngle(ARM, 0, ARM_MAX_ANGLE-10, 5); // Close wavering point
	for(uint8_t i = 0; i < 3; i++){
		Servo_SweepAngle(ARM, ARM_MAX_ANGLE-10, ARM_MAX_ANGLE-20, 5); // Far wavering point
		HAL_Delay(100);
		Servo_SweepAngle(ARM, ARM_MAX_ANGLE-20, ARM_MAX_ANGLE-10, 5); // Close wavering point
	}
	Servo_SweepAngle(ARM, ARM_MAX_ANGLE-10, 100, 3); // Fake return arm
	HAL_Delay(100);
	Servo_SweepAngle(ARM, 100, ARM_MAX_ANGLE, 3); // Flip switch
	HAL_Delay(100);
	Servo_SweepAngle(ARM, ARM_MAX_ANGLE, 0, 2); // Return arm
	Servo_SweepAngle(LID, LID_MAX_ANGLE, 0, 2); // Close lid
}

void UselessBox_RoutineDecider(){
	switch(rand()%4){
		case 0:  UselessBox_StandardRoutine(); break;
		case 1:  UselessBox_AngryRoutine();    break;
		case 2:  UselessBox_WaveringRoutine(); break;
		default: UselessBox_StandardRoutine();
	}
}

uint32_t Servo_SetAngle(ServoID id, uint8_t angle){
	uint32_t pulse_width = Servo_CalculatePulseWidth(angle);
	switch(id){
		case LID: htim2.Instance->CCR1 = pulse_width; break;
		case ARM: htim2.Instance->CCR2 = pulse_width; break;
		default:  break;
	}

	return pulse_width;
}

uint32_t Servo_CalculatePulseWidth(uint8_t angle){
	return MIN_SERVO_PW + (((MAX_SERVO_PW - MIN_SERVO_PW) * angle)/MAX_ROTATION_DEGREES);
}

void Servo_SweepAngle(ServoID id, uint8_t angle1, uint8_t angle2, uint32_t delay_ms) {
	// Time in ms it takes to execute this function = abs(angle1-angle2)*delay_ms
    if (angle1 > 180) angle1 = 180;
    if (angle2 > 180) angle2 = 180;

    if (angle1 == angle2) return;

    int8_t sign = (angle1 < angle2) ? 1 : -1;

    while (angle1 != angle2) {
        Servo_SetAngle(id, angle1);
        HAL_Delay(delay_ms);
        angle1 += sign;
    }

    // Ensure the servo reaches the final position
    Servo_SetAngle(id, angle2);
}

void Speaker_OutputSample(uint8_t freq){
	 htim4.Instance->CCR1 = freq;
}

void Speaker_DWT_Init(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enables DWT unit
    DWT->CYCCNT = 0; // Clears DWT counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Starts it counting
}

void Speaker_DWTDelay_us(uint32_t us_delay){
    uint32_t cycles = us_delay * (SystemCoreClock / 1000000); // Cycles per microsecond (84MHz/1,000,000) = 84 cycles per us
    uint32_t start = DWT->CYCCNT;                             // Current cycle count
    while ((DWT->CYCCNT - start) < cycles);                   // Busy-wait until specified cycles have passed
}

void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue){
	htim3.Instance->CCR1 = red;
	htim3.Instance->CCR2 = green;
	htim3.Instance->CCR3 = blue;
}

void HSV_to_RGB(uint16_t H, uint16_t *R, uint16_t *G, uint16_t *B) {
//	https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
	float S = 1, V = 1;
    float C = V * S;
    float X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
    float m = V - C;

    float r, g, b;
    if      (H >= 0   && H <  60) r = C, g = X, b = 0;
    else if (H >= 60  && H < 120) r = X, g = C, b = 0;
    else if (H >= 120 && H < 180) r = 0, g = C, b = X;
    else if (H >= 180 && H < 240) r = 0, g = X, b = C;
    else if (H >= 240 && H < 300) r = X, g = 0, b = C;
    else                          r = C, g = 0, b = X;

    // Convert to 8-bit RGB
    *R = (int)((r + m) * 255);
    *G = (int)((g + m) * 255);
    *B = (int)((b + m) * 255);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* INIT SERVOS */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Lid servo
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Arm servo

  /* INIT RGB LED */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Red channel
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Green channel
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Blue channel

  /* INIT SPEAKER CONTROL */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Speaker control
  Speaker_DWT_Init();                       // 125us delay, 8Khz audio playback

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  Servo_SetAngle(LID, 0);
  Servo_SetAngle(ARM, 0);
  // Idea: Seed RNG with an empty ADC port. Should read garbage data

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(movement_requested){
		  Speaker_OutputSample(0); // Turn off speaker
		  HAL_Delay(100);          // Maybe remove this
		  LED_SetColor(0, 0, 0);  // Turn off lights
		  UselessBox_RoutineDecider();
		  movement_requested = 0;
	  } else {
		  // RGB Cycling
		  if((hue_update_counter++) % 50 == 0) {
			  H = (H + 1) % 360; // Tweak cycle_speed if necessary. Only increment H every 50 iterations
			  hue_update_counter = 1;
		  }
		  HSV_to_RGB(H, &R, &G, &B);
		  LED_SetColor(R,G,B);

		  // Speaker audio playback
		  Speaker_OutputSample(trapqueencut_wav[audio_sample_index++]);
		  if(audio_sample_index > trapqueencut_wav_len) audio_sample_index = AUDIO_SAMPLE_RESET_INDEX;
		  Speaker_DWTDelay_us(SPEAKER_SAMPLE_DELAY_US); // Should be 125us, but its too pitched down. Plays audio at 8000hz, which is how I encoded the WAV file

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
  // Remember, Prescaler is 200-1 and Period is 8400-1.
  // Period gets changed when IOC generates code for some reason
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 80;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // This function handles interrupts by pin number, not by port.


  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  if(GPIO_Pin != GPIO_PIN_8) return;

  current_time = HAL_GetTick();

  if (current_time - previous_time < DEBOUNCE_TIME_MS) return; // Debounce check: Ignore if the debounce time hasn't passed

  // Remember, with the pull-up resistor, switch OFF = High, ON = Low
  // If the switch is ON (GPIO_PIN_RESET, which means Low), request servo movement
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET){
	  previous_time = current_time;
	  movement_requested = 1;  // Trigger servo movement
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

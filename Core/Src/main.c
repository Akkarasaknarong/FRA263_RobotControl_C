/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "kalman_filter.h"
#include "reference_feedforward.h"
#include "disturbance_feedforward.h"
#include "cascade.h"
#include "quintic_traj.h"
#include "qei.h"
#include "md20a.h"
#include "serial_frame.h"
#include "modbus.h"
#include "basesystem_interface.h"
#include "User_Interface.h"
#include "gripper.h"
#include "datatype.h"
#include "Init.h"
#include "Robot_Worker.h"

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
refTarget_t REFdata = { 0 };
QEIstruct_t QEIdata = { 0 };
SSErrorstruct_t SSErrordata = {0};
KALMANstruct_t ESTdata = { 0 };
PIDParam_t PIDparam = { .kp_pos = 1000000, .kd_pos = 0, .ki_pos = 0, .kp_vel = 1, .kd_vel = 0, .ki_vel = 0 };
SerialFrame_t STLINK_UART_frame;

Robot_t Robot = {
	.Robot_Status = Ready_recieve_Basesystem,
	.Robot_Processing = Robot_Idle,
	.Data_from_Basesystem = Do_not_have_data_Basesystem,
	.Mode = Mode_IDLE,

	.Monitor_data = {
		.QEI = &QEIdata,
		.REF = &REFdata,
		.SSErr = &SSErrordata
	}
};

float V_FF = 0 ;
int PWM_FF = 0 ;
float PWM_PID = 0;
float V_DFFW = 0 ;
int PWM_DFFW = 0 ;

ModbusHandleTypedef hmodbus;
u16u8_t reg[MODBUS_REGISTER_COUNT];
volatile uint32_t uart_rx_count = 0;
volatile uint16_t dbg_rx_tail = 0;
volatile uint8_t dbg_crc_fail = 0;
volatile uint16_t dbg_crc_calc = 0;
volatile uint16_t dbg_crc_recv = 0;
volatile uint8_t dbg_tx_count = 0;
volatile uint8_t dbg_worker_state = 0;
float Timer = 0 ;
int state_machine = 1 ;

int Selector_Modbus1_Matlab2 = 1; // Don't forget to change baudrate
int SW_CascadeON1_CascadeOFF2 = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Motor_QEIUpdate();
void MD20A_Control(int pwm);
void Traj_Setup();
void Init_Library();
void Init_TIMER();
void Quintic_List(int selec);
void Gripper_Control_Blocking(int _CCommand);
void Gripper_Control_NonBlocking(int _Command, float GP_Time);
void Robot_Start();
void P2P_Basesystem(float _q_start , float _q_final , float _t);
int check_sum();

void Automode_Update();
void Manualmode_Update();

void UART_Transmit();
void UARTDMAConfig();
void UART_Unpack();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Init_Library();
  Init_TIMER();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		User_Interface_Start();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Period = 57343;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 169;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1145;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim7, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 169;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 399;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 169;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 199;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY_MODE_Pin|RELAY_READY_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_Pin|Gripper_Open_Pin|Gripper_Close_Pin|Gripper_Down_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Gripper_Up_GPIO_Port, Gripper_Up_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_RESET_Pin PROX_Pin E_STOP_Pin */
  GPIO_InitStruct.Pin = BUTTON_RESET_Pin|PROX_Pin|E_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_MODE_Pin RELAY_READY_Pin LD2_Pin */
  GPIO_InitStruct.Pin = RELAY_MODE_Pin|RELAY_READY_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Reed_SW_Close_Pin JS_EMERGENCY_Pin Reed_SW_Up_Pin Reed_SW_Down_Pin */
  GPIO_InitStruct.Pin = Reed_SW_Close_Pin|JS_EMERGENCY_Pin|Reed_SW_Up_Pin|Reed_SW_Down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : JS_Rotate_Right_Pin JS_RESET_Pin JS_Updown_Pin JS_OpenClose_Pin
                           MODE_SELECTOR_Pin */
  GPIO_InitStruct.Pin = JS_Rotate_Right_Pin|JS_RESET_Pin|JS_Updown_Pin|JS_OpenClose_Pin
                          |MODE_SELECTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin Gripper_Open_Pin Gripper_Close_Pin Gripper_Down_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|Gripper_Open_Pin|Gripper_Close_Pin|Gripper_Down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_Pin */
  GPIO_InitStruct.Pin = TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JS_HOME_Pin JS_Rotate_Left_Pin */
  GPIO_InitStruct.Pin = JS_HOME_Pin|JS_Rotate_Left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Gripper_Up_Pin */
  GPIO_InitStruct.Pin = Gripper_Up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Gripper_Up_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART_Transmit(){
	SerialFrame_AddPayload(&STLINK_UART_frame, &QEIdata.q, sizeof(float));
	SerialFrame_AddPayload(&STLINK_UART_frame, &QEIdata.qd, sizeof(float));
	SerialFrame_AddPayload(&STLINK_UART_frame, &ESTdata.q_est, sizeof(float));
	SerialFrame_AddPayload(&STLINK_UART_frame, &ESTdata.qd_est, sizeof(float));
	SerialFrame_AddPayload(&STLINK_UART_frame, &REFdata.ref_q, sizeof(float));
	SerialFrame_AddPayload(&STLINK_UART_frame, &REFdata.ref_qd, sizeof(float));
	SerialFrame_AddPayload(&STLINK_UART_frame, &Timer, sizeof(float));
	SerialFrame_Transmit(&STLINK_UART_frame); // 2Header + 2Single
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	uart_rx_count++;
	dbg_rx_tail = Size;
	if (Size == 0)
		return;
	hmodbus.modbusUartStructure.RxTail = Size;
	hmodbus.Flag_T15TimeOut = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &POS_CTRL_TIM) {
		Robot_Worker();

		// Modbus MATLAB Selector
		if (Selector_Modbus1_Matlab2 == 1){
			Robot_Start();
			Modbus_Protocal_Worker();
			if (reg[REG_HEARTBEAT].U16 == HEARTBEAT_PC) {
				reg[REG_HEARTBEAT].U16 = HEARTBEAT_ROBOT;
			}
			Basesystem_Interface_Decode(reg);
		} else if (Selector_Modbus1_Matlab2 == 2){
			UART_Transmit();
		}


		// Pos Control
		if (SW_CascadeON1_CascadeOFF2 == 1){
			Pos_ctrl_Tunning(PIDparam.kp_pos, PIDparam.kd_pos, PIDparam.ki_pos);
			Pos_ctrl_Compute(REFdata.ref_q, ESTdata.q_est);
		}
	}

	if (htim == &VEL_CTRL_TIM) {
		if (SW_CascadeON1_CascadeOFF2 == 1){
			// Vel Control
			Vel_ctrl_Tunning(PIDparam.kp_vel, PIDparam.kd_vel, PIDparam.ki_vel);
			Vel_ctrl_Compute(REFdata.ref_qd , ESTdata.qd_est,&PWM_PID);

			// Disturbance FFW
			Motor_Disturbance_feedforward_Update(ESTdata.load_est,&V_DFFW);
			PWM_DFFW = V_DFFW * (65535.0f / 24.0f);

			// Refference FFW
			Motor_Ref_feedforward_Update(REFdata.ref_qd,&V_FF);
			PWM_FF = V_FF * (65535.0f / 24.0f);

			// PWM Output
			MD20A_Control(PWM_FF+PWM_PID);
		}
	}

	if (htim == &QEI_UPDATE_TIM) {
		// Global Timer
		Timer += 0.0002 ;

		if (Selector_Modbus1_Matlab2 == 2){
			// Quintic List Tuning (Local Path)
			 Quintic_List(2);
		}

		// QEI
		QEI_Update();

		// Kalman
		Motor_Kalman_Update(V_FF, QEIdata.q, &ESTdata.q_est, &ESTdata.qd_est, &ESTdata.load_est, &ESTdata.i_est);

		// Monitor Steady state Error
		SSErrordata.q_ss = REFdata.ref_q - QEIdata.q ;
		SSErrordata.qd_ss = REFdata.ref_qd - QEIdata.qd ;
		SSErrordata.qd_deg_ss = RadianToDegree(SSErrordata.q_ss);
		// Monitor (Can delete use this for monitor only)
		REFdata.ref_q_index = RadianToIndex(REFdata.ref_q);
		REFdata.ref_q_deg = RadianToDegree(REFdata.ref_q);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &STLINK_UART) {
		if (Selector_Modbus1_Matlab2 == 2){
			UART_Unpack();
		}
	}
}

void Quintic_P2P(float _q_start , float _q_final , float _t){
	QuinticTraj_Init(_q_start, _q_final, _t);
	QuinticTraj_Compute(Timer,&REFdata.ref_q,&REFdata.ref_qd,&REFdata.ref_qdd);

	if (Timer >= _t){
		Timer = 0 ;
		state_machine++ ;
	}
}
void Quintic_List(int selec) {
	float t_slow = 3.25f;
	float tar_deg = -360.0f;

	float t_fast = 3.25f;
	float t_break = 2.0f ;


	// Rotate
	if (selec == 1){
		if (state_machine == 1){
			Quintic_P2P(0,360,t_slow);
		} else if (state_machine == 2) {
			Quintic_P2P(360,360,t_break);
		} else if (state_machine == 3) {
			Quintic_P2P(360,0,t_fast);

		} else if (state_machine == 4) {
			Quintic_P2P(0,0,t_break);
		} else if (state_machine == 5) {
			Quintic_P2P(0,270,t_slow);
		} else if (state_machine == 6) {
			Quintic_P2P(270,270,t_break);
		} else if (state_machine == 7) {
			Quintic_P2P(270,0,t_fast);
		}

		else if (state_machine == 8) {
			Quintic_P2P(0, 0, t_break);
		} else if (state_machine == 9) {
			Quintic_P2P(0, 180, t_slow);
		} else if (state_machine == 10) {
			Quintic_P2P(180, 180, t_break);
		} else if (state_machine == 11) {
			Quintic_P2P(180, 0, t_fast);
		}

		else if (state_machine == 12) {
			Quintic_P2P(0, 0, t_break);
		} else if (state_machine == 13) {
			Quintic_P2P(0, 90, t_slow);
		} else if (state_machine == 14) {
			Quintic_P2P(90, 90, t_break);
		} else if (state_machine == 15) {
			Quintic_P2P(90, 0, t_fast);
		}
	}

	// Rotate with Pick Place
	if (selec == 2) {
		if (state_machine == 1) {
			Quintic_P2P(0, 0, t_break);
		}
		if (state_machine == 2) {
			Quintic_P2P(0, tar_deg, t_slow);
		}
		if (state_machine == 3) {
			Quintic_P2P(tar_deg, tar_deg, t_break);
		}
		if (state_machine == 4) {
			Quintic_P2P(tar_deg, 0, t_slow);

		}
		if (state_machine == 5) {
			state_machine = 1;
		}
	}
}

void Quintic_List_Basesystem(int N_pare) {
	float t_slow = 1.2f;
	float t_fast = 1.25f;
	float t_break = 2.0f;

	float p0 = IndexToDegree(Basesystem_Data._Auto.Sequence[0]);
	float p1 = IndexToDegree(Basesystem_Data._Auto.Sequence[1]);
	float p2 = IndexToDegree(Basesystem_Data._Auto.Sequence[2]);
	float p3 = IndexToDegree(Basesystem_Data._Auto.Sequence[3]);
	float p4 = IndexToDegree(Basesystem_Data._Auto.Sequence[4]);
	float p5 = IndexToDegree(Basesystem_Data._Auto.Sequence[5]);
	float p6 = IndexToDegree(Basesystem_Data._Auto.Sequence[6]);
	float p7 = IndexToDegree(Basesystem_Data._Auto.Sequence[7]);
	float p8 = IndexToDegree(Basesystem_Data._Auto.Sequence[8]);
	float p9 = IndexToDegree(Basesystem_Data._Auto.Sequence[9]);

	float tar = 180.0f ;
	if (N_pare == 1) {
		if (state_machine == 1) Quintic_P2P(0, 0, t_break); 		// Go Pick 1
		else if (state_machine == 2) Quintic_P2P(0, tar, t_slow);	// Pick 1
		else if (state_machine == 3) Quintic_P2P(tar, tar, t_break);	// Go Place 1
		else if (state_machine == 4) Quintic_P2P(tar, 0, t_slow);	// Place 1
		else if (state_machine == 5) Quintic_P2P(0, 0, t_break);
		else if (state_machine == 6) {Robot.Robot_Processing = Robot_Complete ;}
	}

	else if (N_pare == 2) {
		if (state_machine == 1) Quintic_P2P(0, p0, t_fast); 		// Go Pick 1
		else if (state_machine == 2) Quintic_P2P(p0, p0, t_break);	// Pick 1
		else if (state_machine == 3) Quintic_P2P(p0, p1, t_slow);	// Go Place 1
		else if (state_machine == 4) Quintic_P2P(p1, p1, t_break);	// Place 1

		else if (state_machine == 5) Quintic_P2P(p1, p2, t_fast);	// Go Pick 2
		else if (state_machine == 6) Quintic_P2P(p2, p2, t_break);	// Pick 2
		else if (state_machine == 7) Quintic_P2P(p2, p3, t_slow);	// Go Place 2
		else if (state_machine == 8) Quintic_P2P(p3, p3, t_break);	// Place 2
		else if (state_machine == 9) Quintic_P2P(p3, 0, t_slow);	// Place 2
		else if (state_machine == 10) {Robot.Robot_Processing = Robot_Complete ;}
	}

	else if (N_pare == 3) {
		if (state_machine == 1) Quintic_P2P(0, p0, t_fast); 		// Go Pick 1
		else if (state_machine == 2) Quintic_P2P(p0, p0, t_break);
		else if (state_machine == 3) Quintic_P2P(p0, p1, t_slow);
		else if (state_machine == 4) Quintic_P2P(p1, p1, t_break);

		else if (state_machine == 5) Quintic_P2P(p1, p2, t_fast);	// Go Pick 2
		else if (state_machine == 6) Quintic_P2P(p2, p2, t_break);
		else if (state_machine == 7) Quintic_P2P(p2, p3, t_slow);
		else if (state_machine == 8) Quintic_P2P(p3, p3, t_break);

		else if (state_machine == 9) Quintic_P2P(p3, p4, t_fast);	// Go Pick 3
		else if (state_machine == 10) Quintic_P2P(p4, p4, t_break);	// Pick 3
		else if (state_machine == 11) Quintic_P2P(p4, p5, t_slow);	// Go Place 3
		else if (state_machine == 12) Quintic_P2P(p5, p5, t_break);	// Place 3
		else if (state_machine == 13) Quintic_P2P(p5, 0, t_slow);
		else if (state_machine == 14) {Robot.Robot_Processing = Robot_Complete ;}
	}

	else if (N_pare == 4) {
		if (state_machine == 1) Quintic_P2P(0, p0, t_fast);
		else if (state_machine == 2) Quintic_P2P(p0, p0, t_break);
		else if (state_machine == 3) Quintic_P2P(p0, p1, t_slow);
		else if (state_machine == 4) Quintic_P2P(p1, p1, t_break);

		else if (state_machine == 5) Quintic_P2P(p1, p2, t_fast);
		else if (state_machine == 6) Quintic_P2P(p2, p2, t_break);
		else if (state_machine == 7) Quintic_P2P(p2, p3, t_slow);
		else if (state_machine == 8) Quintic_P2P(p3, p3, t_break);

		else if (state_machine == 9) Quintic_P2P(p3, p4, t_fast);
		else if (state_machine == 10) Quintic_P2P(p4, p4, t_break);
		else if (state_machine == 11) Quintic_P2P(p4, p5, t_slow);
		else if (state_machine == 12) Quintic_P2P(p5, p5, t_break);

		else if (state_machine == 13) Quintic_P2P(p5, p6, t_fast);	// Go Pick 4
		else if (state_machine == 14) Quintic_P2P(p6, p6, t_break);	// Pick 4
		else if (state_machine == 15) Quintic_P2P(p6, p7, t_slow);	// Go Place 4
		else if (state_machine == 16) Quintic_P2P(p7, p7, t_break);	// Place 4
		else if (state_machine == 17) Quintic_P2P(p7, 0, t_slow);	// Place 4
		else if (state_machine == 18) {Robot.Robot_Processing = Robot_Complete ;}
	}

	else if (N_pare == 5) {
		if (state_machine == 1) Quintic_P2P(0, p0, t_fast);
		else if (state_machine == 2) Quintic_P2P(p0, p0, t_break);
		else if (state_machine == 3) Quintic_P2P(p0, p1, t_slow);
		else if (state_machine == 4) Quintic_P2P(p1, p1, t_break);

		else if (state_machine == 5) Quintic_P2P(p1, p2, t_fast);
		else if (state_machine == 6) Quintic_P2P(p2, p2, t_break);
		else if (state_machine == 7) Quintic_P2P(p2, p3, t_slow);
		else if (state_machine == 8) Quintic_P2P(p3, p3, t_break);

		else if (state_machine == 9) Quintic_P2P(p3, p4, t_fast);
		else if (state_machine == 10) Quintic_P2P(p4, p4, t_break);
		else if (state_machine == 11) Quintic_P2P(p4, p5, t_slow);
		else if (state_machine == 12) Quintic_P2P(p5, p5, t_break);

		else if (state_machine == 13) Quintic_P2P(p5, p6, t_fast);
		else if (state_machine == 14) Quintic_P2P(p6, p6, t_break);
		else if (state_machine == 15) Quintic_P2P(p6, p7, t_slow);
		else if (state_machine == 16) Quintic_P2P(p7, p7, t_break);

		else if (state_machine == 17) Quintic_P2P(p7, p8, t_fast);	// Go Pick 5
		else if (state_machine == 18) Quintic_P2P(p8, p8, t_break);	// Pick 5
		else if (state_machine == 19) Quintic_P2P(p8, p9, t_slow);	// Go Place 5
		else if (state_machine == 20) Quintic_P2P(p9, p9, t_break);	// Place 5
		else if (state_machine == 21) Quintic_P2P(p9, p9, t_break);	// Place 5
		else if (state_machine == 22) {Robot.Robot_Processing = Robot_Complete ;}
	}
}



void Automode_Update(){
	if (Basesystem_Data.Mode == Mode_AUTO) {
		if (Basesystem_Data._Auto.Type == AUTO_TYPE_SEQUENCE){
			int count = Basesystem_Data._Auto.N_pare;
			if (count == 1) {
				Quintic_List_Basesystem(1);
			} else if (count == 2) {
				Quintic_List_Basesystem(2);
			} else if (count == 3) {
				Quintic_List_Basesystem(3);
			} else if (count == 4) {
				Quintic_List_Basesystem(4);
			} else if (count == 5) {
				Quintic_List_Basesystem(5);
			}
		}
		else if (Basesystem_Data._Auto.Type == AUTO_TYPE_P2P){
			if (Basesystem_Data._Auto.P2P.unit == 1) // Unit Index
			{
				REFdata.ref_q = IndexToRadian(Basesystem_Data._Auto.P2P.value);
			} else if (Basesystem_Data._Auto.P2P.unit == 0) // Unit Degree
			{
				REFdata.ref_q = DegreeToRadian(Basesystem_Data._Auto.P2P.value);
			}
			Basesystem_Data.has_new_data = 0 ;
		}
		}
}

void Manualmode_Update(){
	if (Basesystem_Data.Mode == Mode_MANUAL){
		int gp_pin_state = Basesystem_Data._Manual.Gripper_pin_state ;
		if (gp_pin_state == 0)
			Gripper_Group(1,0,0,0);
		if (gp_pin_state == 1)
			Gripper_Group(0,1,0,0);
		if (gp_pin_state == 2)
			Gripper_Group(0,0,0,1);
		if (gp_pin_state == 4)
			Gripper_Group(0,0,1,0);

		int gp_command = Basesystem_Data._Manual.Gripper_command ;
		if (gp_command == 1) // Pick
			Gripper_Control(1);
		if (gp_command == 2) // Place
			Gripper_Control(0);

		int Jog = Basesystem_Data._Manual.Jog_val ;
		REFdata.ref_q += DegreeToRadian(Jog);
		Basesystem_Data._Manual.Jog_val = 0 ;
	}
}
// ======================================================================= //
// === Operating base system ===
void Robot_Start() {
//	if (Basesystem_Data.has_new_data == 1) {
//		Timer = 0.0f;
//		state_machine = 1;
//		Basesystem_Data.has_new_data = 2;
//	}
//	if (Basesystem_Data.has_new_data == 2){
//		 Automode_Update();
////		 Manualmode_Update();
//	}
//	Basesystem_Interface_Feedback();
}
// ======================================================================= //
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_PRESSED     GPIO_PIN_RESET
#define KEY_NOT_PRESSED GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;	// Timer1 handle for PWM
// Timer handle
// extern TIM_HandleTypeDef htim1;
// PID control constants
#define Kp 1.0f      // Proportional gain
#define Ki 0.1f      // Integral gain
#define Kd 0.05f     // Derivative gain

// Motor Control Parameters
float motor_speed = 0.0f;       // Current motor speed (in RPM or units of choice)
float target_speed = 0.0f;      // Target motor speed set by FDCAN commands
float motor_position = 0.0f;    // Current motor position (degrees or radians)
float target_position = 0.0f;   // Target motor position set by FDCAN commands
bool motor_stop = true;            // Flag to stop or start the motor

float previous_error = 0.0f;    // For derivative term
float integral = 0.0f;          // For integral term

// PWM limits
const float max_pwm = 1000.0f;  // Max PWM value (duty cycle)
const float min_pwm = 0.0f;     // Min PWM value (duty cycle)

// FDCAN Message Buffers
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

/* USER CODE BEGIN PV */
uint8_t ubKeyNumber = 0x0;
uint8_t ubKeyNumberValue = 0x0;
uint8_t ubLedBlinkTime = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN1_Init(void);

// Function prototypes
void MotorControl_Init(void);
void MotorControl_Loop(void);
void FDCAN_ReceiveHandler(void);
void Motor_Stop(void);
void Motor_Move(float position, float speed);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(char ch)
{
	//HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
	return ch;
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
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  // Initialize motor control
  MotorControl_Init();
  // Start PWM for motor control
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // Handle FDCAN messages
	FDCAN_ReceiveHandler();

	// Update motor control logic based on received commands
	if (!motor_stop)
	{
		Motor_Move(target_position, target_speed);
	}
	else
	{
		Motor_Stop();
	}

	// Main control loop for motor
	MotorControl_Loop();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */
	FDCAN_FilterTypeDef sFilterConfig;

	  /* Configure Rx filter */
	  sFilterConfig.IdType = FDCAN_STANDARD_ID;
	  sFilterConfig.FilterIndex = 0;
	  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  sFilterConfig.FilterID1 = 0x321;
	  sFilterConfig.FilterID2 = 0x7FF;
	  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }

    /* Prepare Tx Header */
    TxHeader.Identifier = 0x321;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
  /* USER CODE END FDCAN1_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : Hall_Sensor_A_Pin Hall_Sensor_B_Pin Hall_Sensor_C_Pin */
  GPIO_InitStruct.Pin = Hall_Sensor_A_Pin|Hall_Sensor_B_Pin|Hall_Sensor_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Initialize motor control peripherals
void MotorControl_Init(void)
{
    // Timer1 for PWM control (already initialized in CubeMX)
    htim1.Instance = TIM1;
}

void FDCAN_ReceiveHandler(void)
{
    // Check if a new message is received in RX FIFO 0
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // Extract command from the received data (first two bytes)
        uint16_t command = (RxData[0] << 8) | RxData[1];

        // Process the command based on its value
        switch (command)
        {
            case 0x0001:  // Start motor command
                motor_stop = false;  // Set flag to start the motor
                target_speed = RxData[2];  // Set target speed from the message (byte 3)
                break;

            case 0x0002:  // Stop motor command
                motor_stop = true;  // Set flag to stop the motor
                break;

            case 0x0003:  // Set motor position command
                // Extract a 32-bit float value from RxData bytes (4 to 7) for target position
                target_position = *((float*)&RxData[2]);
                break;

            default:
                break;
        }
    }
}
void FDCAN_TransmitMessage(uint32_t id, uint8_t *data, uint8_t size)
{
    FDCAN_TxHeaderTypeDef TxHeader;  // Define the TxHeader structure

    // Set up the CAN message header
    TxHeader.Identifier = id;                 // Set the message ID
    TxHeader.IdType = FDCAN_STANDARD_ID;       // Use standard 11-bit CAN ID, change to FDCAN_EXTENDED_ID for 29-bit
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // Transmitting a data frame
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;   // Set the Data Length Code (DLC), adjust this based on the actual data size

    // Optional configurations:
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // Set error state (Active or Passive)
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // Set to FDCAN_BRS_ON for CAN FD (faster data rate), leave OFF for standard CAN
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // Set to Classic CAN mode, adjust to FDCAN_FD_CAN for CAN FD
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // Disable Tx events, enable if required

    // Check if the data size is within allowed limits
    if (size <= 8)  // Standard CAN supports up to 8 bytes of data
    {
        // Transmit the message by adding it to the FDCAN Tx FIFO queue
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
        {
            // Transmission failed, handle error
            printf("Error: Failed to add FDCAN message to Tx FIFO queue\n");
        }
    }
    else
    {
        // If data size exceeds 8 bytes, handle the error for CAN FD (Flexible Data Rate CAN)
        printf("Error: Data size exceeds maximum payload for standard CAN (8 bytes)\n");
    }
}

void Motor_Move(float position, float speed)
{
    // Ensure the speed value is within a valid range (0 to 100%)
    if (speed < 0.0f)
        speed = 0.0f;
    else if (speed > 100.0f)
        speed = 100.0f;

    // Ensure the position value is within your motor's limits (this is motor specific)
    // For example, if position ranges from 0 to 360 degrees for a rotary motor
    if (position < 0.0f)
        position = 0.0f;
    else if (position > 360.0f)
        position = 360.0f;

    // Example logic: Adjust PWM duty cycle based on the desired speed (mapped to 0 - 100% duty cycle)
    uint16_t pwm_value = (uint16_t)(speed * (TIM1->ARR + 1) / 100.0f);  // Scale speed to PWM range

    // Set PWM duty cycle for Channel 1 (adjusting speed)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);

    // PID Control (Pseudo-code)
    // float error = target_position - current_position; // Calculate error based on sensor feedback
    // float correction = PID_Compute(error);            // PID correction to reach the desired position
    // Adjust PWM duty cycle based on the correction
}


// Stop the motor
void Motor_Stop(void)
{
    // Disable motor control by setting PWM duty cycle to zero on all necessary channels
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  // Set PWM on Channel 1 to 0
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);  // Set PWM on Channel 2 to 0 (if used)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);  // Set PWM on Channel 3 to 0 (if used)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);  // Set PWM on Channel 4 to 0 (if used)

    // Optionally: Turn off the timer or reset control signals (optional safety step)
    __HAL_TIM_DISABLE(&htim1);  // Disable Timer 1 if no longer needed
}
void MotorControl_Loop(void)
{
    // PID control variables
    float error = 0.0f;         // Error between target and current speed/position
    float derivative = 0.0f;    // Derivative term
    float output = 0.0f;        // PID output
    float dt = 0.01f;           // Time step for derivative and integral calculation

    // --- Motor speed control (can be adapted for position control similarly) ---

    // Calculate the error between target speed and current motor speed
    error = target_speed - motor_speed;

    // Calculate integral term
    integral += error * dt;

    // Calculate derivative term
    derivative = (error - previous_error) / dt;

    // Calculate PID output
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Save the current error for the next derivative calculation
    previous_error = error;

    // Constrain the output to valid PWM range
    if (output > max_pwm)
    {
        output = max_pwm;
    }
    else if (output < min_pwm)
    {
        output = min_pwm;
    }

    // Apply the PWM output to the motor via Timer1 (assuming Channel 1 for motor control)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)output);
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

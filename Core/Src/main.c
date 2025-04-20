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
#include "joystick.h"
#include "swerve_module.h"
#include "swerve_drive.h"
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DEBUG_PRINT
//#define TEST
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim20;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float xSpeed = 0.0f, ySpeed = 0.0f, rot = 0.0f;
uint32_t lastJoystickUpdate = 0;  // Store the last time we received valid joystick data
#define JOYSTICK_TIMEOUT_MS 500   // Timeout after 500ms of no data

bool lastButtonState;
bool throwState = false;
bool lastZalahState;
bool zalahState = false;
bool PASS_State = false;
bool ZALAH_State = false;
bool lastLockModeButtonState = false;
LockMode currentLockMode = LOCK_MODE_DISABLED;

// Swerve drive data structure
SwerveDriveData swerveData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM20_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void calibrate_motor()
{
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_On(LED_GREEN);
    // Send maximum throttle pulse for ESC calibration
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1940-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1940-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1940-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1940-1);

    HAL_Delay(9000);  // Wait 2 seconds (adjust as needed)

    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_On(LED_GREEN);
    // Then send minimum throttle pulse to complete calibration
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1100-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1100-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1100-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100-1);
    HAL_Delay(8000);

    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);
    BSP_LED_Off(LED_GREEN);
}
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
  SwerveModule moduleRB = {	// Configuration moduleRF
		  .steering = {
		      .dira_gpio_port = INA3_GPIO_Port,
		      .dira_gpio_pin = INA3_Pin,
			  .dirb_gpio_port = INB3_GPIO_Port,
			  .dirb_gpio_pin = INB3_Pin,
		      .pwm_tim = &htim3,
		      .pwm_channel = TIM_CHANNEL_3,
	        .encoder_tim = &htim8,
		      .Kp = 5.0f,        // Increased from 2.5f
		      .Ki = 0.2f,        // Increased from 0.1f
		      .Kd = 0.02f,       // Increased from 0.01f
		      .integral_limit = 500.0f,
		      .dt = 0.05f,
			  .max_pwm = 199     // Maximum PWM value
		  },
		  .driving = {
		      .pwm_tim = &htim2,
		      .pwm_channel = TIM_CHANNEL_4,
		      .min_pulse = 1110-1,
		      .max_pulse = 1400-1,
		      .arming_pulse = 1100-1
		  },
		  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };

  SwerveModule moduleLB = {	// Configuration moduleRF
	  .steering = {
	      .dira_gpio_port = INA4_GPIO_Port,
	      .dira_gpio_pin = INA4_Pin,
		  .dirb_gpio_port = INB4_GPIO_Port,
		  .dirb_gpio_pin = INB4_Pin,
	      .pwm_tim = &htim3,
	      .pwm_channel = TIM_CHANNEL_4,
          .encoder_tim = &htim20,
	      .Kp = 5.0f,        // Increased from 2.5f
	      .Ki = 0.2f,        // Increased from 0.1f
	      .Kd = 0.02f,       // Increased from 0.01f
	      .integral_limit = 500.0f,
	      .dt = 0.05f,
		  .max_pwm = 199     // Maximum PWM value
	  },
	  .driving = {
	      .pwm_tim = &htim2,
	      .pwm_channel = TIM_CHANNEL_2,
	      .min_pulse = 1110-1,
	      .max_pulse = 1400-1,
	      .arming_pulse = 1100-1
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };

  SwerveModule moduleRF = {	// Configuration moduleRF
	  .steering = {
	      .dira_gpio_port = INA1_GPIO_Port,
	      .dira_gpio_pin = INA1_Pin,
	      .dirb_gpio_port = INB1_GPIO_Port,
	      .dirb_gpio_pin = INB1_Pin,
	      .pwm_tim = &htim3,
	      .pwm_channel = TIM_CHANNEL_1,
          .encoder_tim = &htim1,
		  .Kp = 5.0f,        // Increased from 2.5f
		  .Ki = 0.2f,        // Increased from 0.1f
		  .Kd = 0.02f,       // Increased from 0.01f
	      .integral_limit = 500.0f,
	      .dt = 0.05f,
		  .max_pwm = 199     // Maximum PWM value
	  },
	  .driving = {
	      .pwm_tim = &htim2,
	      .pwm_channel = TIM_CHANNEL_3,
	      .min_pulse = 1110-1,
	      .max_pulse = 1400-1,
	      .arming_pulse = 1100-1
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };

  SwerveModule moduleLF = {	// Configuration moduleRF
		  .steering = {
		      .dira_gpio_port = INA2_GPIO_Port,
		      .dira_gpio_pin = INA2_Pin,
		      .dirb_gpio_port = INB2_GPIO_Port,
		      .dirb_gpio_pin = INB2_Pin,
		      .pwm_tim = &htim3,
		      .pwm_channel = TIM_CHANNEL_2,
	          .encoder_tim = &htim4,
		      .Kp = 5.0f,        // Increased from 5.0f (already high)
		      .Ki = 0.2f,        // Increased from 0.1f
		      .Kd = 0.02f,       // Increased from 0.01f
		      .integral_limit = 500.0f,
		      .dt = 0.05f,
			  .max_pwm = 199     // Maximum PWM value
		  },
		  .driving = {
		      .pwm_tim = &htim2,
		      .pwm_channel = TIM_CHANNEL_1,
		      .min_pulse = 1110-1,
		      .max_pulse = 1400-1,
		      .arming_pulse = 1100-1
		  },
		  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  JOYSTICK_Init(&huart4);  // Pass your UART handle
  JOYSTICK_SetTimeout(100); // Optional: Set custom timeout
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM20_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize swerve drive
  SD_Init();
  SD_SetLockMode(LOCK_MODE_45_DEG);

  // Initialize swerve modules
  SM_Init(&moduleRF);
  SM_Init(&moduleLF);
  SM_Init(&moduleRB);
  SM_Init(&moduleLB);
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Swerve Drive Robot Initialized\n");
  /* -- Sample board code to switch on led ---- */
  BSP_LED_On(LED_GREEN);

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    /* -- Sample board code for User push-button in interrupt mode ---- */
	    if (BspButtonState == BUTTON_PRESSED)
	    {
	      /* Update button state */
	      BspButtonState = BUTTON_RELEASED;
	      /* -- Sample board code to toggle led ---- */
	      BSP_LED_Toggle(LED_GREEN);

	      calibrate_motor();
	    }
        JOYSTICK_Process();

        // Check for joystick timeout
        if (HAL_GetTick() - lastJoystickUpdate > JOYSTICK_TIMEOUT_MS) {
            // Reset all control values when joystick times out
            xSpeed = 0.0f;
            ySpeed = 0.0f;
            rot = 0.0f;
            
            // Update kinematics with zero values
            SD_UpdateKinematics(xSpeed, ySpeed, rot, &swerveData);
            SD_UpdateModules(&swerveData, &moduleRF, &moduleLF, &moduleRB, &moduleLB);
        }
        // Normal joystick control code
        else if (JOYSTICK_NewDataAvailable()) {
            JoystickData data = JOYSTICK_GetData();
            lastJoystickUpdate = HAL_GetTick();  // Reset timeout timer

            // Scale joystick inputs to [-1, 1] range with deadband
            xSpeed = -(float)data.axisX / 512.0f;
            ySpeed = (float)data.axisY / 512.0f;  
            rot = (float)data.axisRX / 512.0f;

            // Apply deadband to prevent drift
            SD_ApplyDeadband(&xSpeed, &ySpeed, &rot);

//            printf("X: %f, Y: %f, Rot: %f\n", xSpeed, ySpeed, rot);
//            printf("Buttons (Hex): 0x%04X\n", data.buttons);

            // Handle lock mode toggle (using button 0x0010)
            bool currentLockModeButtonState = (data.buttons & 0x0010) != 0;
            if (currentLockModeButtonState && !lastLockModeButtonState) {
                // Toggle through lock modes
                currentLockMode = (currentLockMode + 1) % 3;  // Cycle through 0, 1, 2
                SD_SetLockMode(currentLockMode);
                printf("Lock mode changed to: %d\n", currentLockMode);
            }
            lastLockModeButtonState = currentLockModeButtonState;

            // Update kinematics and modules
            SD_UpdateKinematics(xSpeed, ySpeed, rot, &swerveData);
            SD_UpdateModules(&swerveData, &moduleRF, &moduleLF, &moduleRB, &moduleLB);
        }

//        // Check encoder data
//        SM_GetCurrentAngle(&moduleRF);
//        SM_GetCurrentAngle(&moduleLF);
//        SM_GetCurrentAngle(&moduleRB);
//        SM_GetCurrentAngle(&moduleLB);
//        printf("\r\n");

//        HAL_GPIO_WritePin(moduleRF.steering.dira_gpio_port, moduleRF.steering.dira_gpio_pin, 0);
//        HAL_GPIO_WritePin(moduleRF.steering.dirb_gpio_port, moduleRF.steering.dirb_gpio_pin, 1);
//        __HAL_TIM_SET_COMPARE(moduleRF.steering.pwm_tim, moduleRF.steering.pwm_channel, (uint16_t)30);
//        printf("here\n");
//        HAL_Delay(1000);

        /*
         *steering motor hurd nemj hugtsaag bagsgah
        encoder timer aldaad bn ylanguya 4. Hulhi omron baij magdgu omron solij uzej bolno
        tged validate data bnu gedgiig shalgaj uzeh
        tgku bol subsystem 2 avtomataar ajilaad bn
        swerve baga zereg smooth bolgoh

        tged zalah garj irsen tohioldold zaldag button dahiad darval motor irgdeg nohtsol tavih
         catapult ayulgui ajilgaag hangah davhar nohtsol hynah

         sensor tavij zarim zuilsiig automation juulah
         button evteihen bolgoh
         */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        HAL_Delay(10);
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 170-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  sConfigOC.Pulse = 1099;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim3.Init.Prescaler = 85-1;  // Reduced from 170-1 to increase frequency
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;    // Reduced from 200-1 to increase frequency
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM20 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM20_Init(void)
{

  /* USER CODE BEGIN TIM20_Init 0 */

  /* USER CODE END TIM20_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM20_Init 1 */

  /* USER CODE END TIM20_Init 1 */
  htim20.Instance = TIM20;
  htim20.Init.Prescaler = 0;
  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim20.Init.Period = 65535;
  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim20.Init.RepetitionCounter = 0;
  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim20, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM20_Init 2 */

  /* USER CODE END TIM20_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 500000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INB1_Pin|INB2_Pin|INB3_Pin|INB4_Pin
                          |INA1_Pin|INA2_Pin|INA3_Pin|INA4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INB1_Pin INB2_Pin INB3_Pin INB4_Pin
                           INA1_Pin INA2_Pin INA3_Pin INA4_Pin */
  GPIO_InitStruct.Pin = INB1_Pin|INB2_Pin|INB3_Pin|INB4_Pin
                          |INA1_Pin|INA2_Pin|INA3_Pin|INA4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
//  // Replace existing motor pin configurations with:
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx; // Use correct AF for each timer
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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

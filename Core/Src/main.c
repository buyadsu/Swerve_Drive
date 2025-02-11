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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PRINT
#define TEST

// Robot dimensions (adjust according to actual measurements)
#define ROBOT_LENGTH 0.5f // Distance from front to back wheels (meters)
#define ROBOT_WIDTH 0.5f  // Distance from left to right wheels (meters)
#define ROBOT_STEERING_GEAR_RATIO 2.0f // Gear ratio of steering motors
#define STEERING_ENCODER_RESOLUTION 1000.0 // Encoder resolution
#define DEADZONE 0.1f     // Deadzone to ignore small joystick inputs
#define MAX_CHANGE_RATE 0.05f  // Limits the change per loop iteration
#define TIMEOUT_MS 500         // Timeout for joystick disconnect detection

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
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float xSpeed = 0.0f, ySpeed = 0.0f, rot = 0.0f;
uint32_t lastJoystickUpdate = 0;  // Store the last time we received valid joystick data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  SwerveModule moduleRF = {	// Configuration moduleRF
	  .steering = {
	      .dir_gpio_port = IN1_GPIO_Port,
	      .dir_gpio_pin = IN1_Pin,
	      .pwm_tim = &htim1,
	      .pwm_channel = TIM_CHANNEL_4,
          .encoder_tim = &htim3,
	      .Kp = 2.5f,
	      .Ki = 0.1f,
	      .Kd = 0.01f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199
	  },
	  .driving = {
	      .pwm_tim = &htim2,
	      .pwm_channel = TIM_CHANNEL_1,
	      .min_pulse = 1110-1,
	      .max_pulse = 1915-1,
	      .arming_pulse = 1100-1
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };

  SwerveModule moduleLF = {	// Configuration moduleRF
	  .steering = {
	      .dir_gpio_port = IN2_GPIO_Port,
	      .dir_gpio_pin = IN2_Pin,
	      .pwm_tim = &htim1,
	      .pwm_channel = TIM_CHANNEL_3,
          .encoder_tim = &htim4,
	      .Kp = 2.5f,
	      .Ki = 0.1f,
	      .Kd = 0.01f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199
	  },
	  .driving = {
	      .pwm_tim = &htim2,
	      .pwm_channel = TIM_CHANNEL_2,
	      .min_pulse = 1110-1,
	      .max_pulse = 1915-1,
	      .arming_pulse = 1100-1
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };

  SwerveModule moduleRB = {	// Configuration moduleRF
	  .steering = {
	      .dir_gpio_port = IN3_GPIO_Port,
	      .dir_gpio_pin = IN3_Pin,
	      .pwm_tim = &htim1,
	      .pwm_channel = TIM_CHANNEL_2,
          .encoder_tim = &htim5,
		  .Kp = 2.5f,  // Increase proportional gain
		  .Ki = 0.05f, // Reduce integral to prevent windup
		  .Kd = 0.05f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199
	  },
	  .driving = {
	      .pwm_tim = &htim2,
	      .pwm_channel = TIM_CHANNEL_3,
	      .min_pulse = 1110-1,
	      .max_pulse = 1915-1,
	      .arming_pulse = 1100-1
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };

  SwerveModule moduleLB = {	// Configuration moduleRF
	  .steering = {
	      .dir_gpio_port = IN4_GPIO_Port,
	      .dir_gpio_pin = IN4_Pin,
	      .pwm_tim = &htim1,
	      .pwm_channel = TIM_CHANNEL_1,
          .encoder_tim = &htim8,
	      .Kp = 2.5f,
	      .Ki = 0.1f,
	      .Kd = 0.01f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199
	  },
	  .driving = {
	      .pwm_tim = &htim2,
	      .pwm_channel = TIM_CHANNEL_4,
	      .min_pulse = 1110-1,
	      .max_pulse = 1915-1,
	      .arming_pulse = 1100-1
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8) // Adjust based on encoder
  };
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  JOYSTICK_Init(&huart3);  // Pass your UART handle
  JOYSTICK_SetTimeout(50); // Optional: Set custom timeout
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
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialization
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

//  __HAL_TIM_SET_COMPARE(&moduleRF.driving.pwm_tim, &moduleRF.driving.pwm_channel, &moduleRF.driving.min_pulse);
//  __HAL_TIM_SET_COMPARE(&moduleLF.driving.pwm_tim, &moduleLF.driving.pwm_channel, &moduleLF.driving.min_pulse);
//  __HAL_TIM_SET_COMPARE(&moduleRB.driving.pwm_tim, &moduleRB.driving.pwm_channel, &moduleRB.driving.min_pulse);
//  __HAL_TIM_SET_COMPARE(&moduleLB.driving.pwm_tim, &moduleLB.driving.pwm_channel, &moduleLB.driving.min_pulse);

//  SM_CalibrateESC(&moduleRF.driving);
//  SM_CalibrateESC(&moduleLF.driving);
//  SM_CalibrateESC(&moduleRB.driving);
//  SM_CalibrateESC(&moduleLB.driving);

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
      printf("ESC calibrate starting..\n");

      SM_CalibrateESC(&moduleRF.driving);
      SM_CalibrateESC(&moduleLF.driving);
      SM_CalibrateESC(&moduleRB.driving);
      SM_CalibrateESC(&moduleLB.driving);

      printf("ESC calibrate done.\n");

//      __HAL_TIM_SET_COMPARE(&moduleRF.driving.pwm_tim, &moduleRF.driving.pwm_channel, &moduleRF.driving.min_pulse);
//      __HAL_TIM_SET_COMPARE(&moduleLF.driving.pwm_tim, &moduleLF.driving.pwm_channel, &moduleLF.driving.min_pulse);
//      __HAL_TIM_SET_COMPARE(&moduleRB.driving.pwm_tim, &moduleRB.driving.pwm_channel, &moduleRB.driving.min_pulse);
//      __HAL_TIM_SET_COMPARE(&moduleLB.driving.pwm_tim, &moduleLB.driving.pwm_channel, &moduleLB.driving.min_pulse);

      /* ..... Perform your action ..... */
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    JOYSTICK_Process();

    if (JOYSTICK_NewDataAvailable()) {
    	JoystickData data = JOYSTICK_GetData();
    	lastJoystickUpdate = HAL_GetTick();  // Reset timeout timer


#ifdef DEBUG_PRINT
        printf("X: %ld, Y: %ld, RX: %ld\n", data.axisX, data.axisY, data.axisRX);
#endif

        xSpeed = (float)data.axisX / 512.0f;
        ySpeed = (float)data.axisY / -512.0f;
        rot = (float)data.axisRX / 512.0f;

        // Apply deadzone
        if (fabsf(xSpeed) < DEADZONE) xSpeed = 0.0f;
        if (fabsf(ySpeed) < DEADZONE) ySpeed = 0.0f;
        if (fabsf(rot) < DEADZONE) rot = 0.0f;

        // Smooth input changes to prevent jerky motion
//        xSpeed += fminf(fmaxf(newXSpeed - xSpeed, -MAX_CHANGE_RATE), MAX_CHANGE_RATE);
//        ySpeed += fminf(fmaxf(newYSpeed - ySpeed, -MAX_CHANGE_RATE), MAX_CHANGE_RATE);
//        rot += fminf(fmaxf(newRot - rot, -MAX_CHANGE_RATE), MAX_CHANGE_RATE);
    }

    // If no joystick data received for TIMEOUT_MS, stop the motors
    if (HAL_GetTick() - lastJoystickUpdate > TIMEOUT_MS) {
        xSpeed = 0.0f;
        ySpeed = 0.0f;
        rot = 0.0f;
    }

#ifdef TEST
		float target_angle = 45.0f;  // From kinematics
		float target_speed = 0;

//		printf("RB Encoder: %ld\n", (int32_t)htim5.Instance->CNT); //debug

		SM_UpdateSteering(&moduleRF, target_angle);
		SM_UpdateDriving(&moduleRF, target_speed);

		SM_UpdateSteering(&moduleLF, target_angle);
		SM_UpdateDriving(&moduleLF, target_speed);
//
		SM_UpdateSteering(&moduleRB, target_angle);
		SM_UpdateDriving(&moduleRB, target_speed);
//
		SM_UpdateSteering(&moduleLB, target_angle);
		SM_UpdateDriving(&moduleLB, target_speed);

		if(SM_SteeringAtTarget(&moduleRF, target_angle, 1.0f)) {
			// Reached target angle
			printf("Reached target angle moduleRF\n");
		}

		if(SM_SteeringAtTarget(&moduleLF, target_angle, 1.0f)) {
			// Reached target angle
			printf("Reached target angle moduleLF\n");
		}

		if(SM_SteeringAtTarget(&moduleRB, target_angle, 1.0f)) {
			// Reached target angle
			printf("Reached target angle moduleRB\n");
		}

		if(SM_SteeringAtTarget(&moduleLB, target_angle, 1.0f)) {
			// Reached target angle
			printf("Reached target angle moduleLB\n");
		}

		HAL_Delay(10);
#else
		// Kinematic calculations for each module
	    // Front Right (RF)
	    float rf_x = xSpeed - (rot * (ROBOT_LENGTH / 2.0f));
	    float rf_y = ySpeed + (rot * (ROBOT_WIDTH / 2.0f));
	    float rf_angle = atan2f(rf_y, rf_x) * (180.0f / (float)M_PI);
	    float rf_speed = sqrtf(rf_x * rf_x + rf_y * rf_y);

	    // Front Left (LF)
	    float lf_x = xSpeed - (rot * (ROBOT_LENGTH / 2.0f));
	    float lf_y = ySpeed - (rot * (ROBOT_WIDTH / 2.0f));
	    float lf_angle = atan2f(lf_y, lf_x) * (180.0f / (float)M_PI);
	    float lf_speed = sqrtf(lf_x * lf_x + lf_y * lf_y);

	    // Rear Right (RB)
	    float rb_x = xSpeed + (rot * (ROBOT_LENGTH / 2.0f));
	    float rb_y = ySpeed + (rot * (ROBOT_WIDTH / 2.0f));
	    float rb_angle = atan2f(rb_y, rb_x) * (180.0f / (float)M_PI);
	    float rb_speed = sqrtf(rb_x * rb_x + rb_y * rb_y);

	    // Rear Left (LB)
	    float lb_x = xSpeed + (rot * (ROBOT_LENGTH / 2.0f));
	    float lb_y = ySpeed - (rot * (ROBOT_WIDTH / 2.0f));
	    float lb_angle = atan2f(lb_y, lb_x) * (180.0f / (float)M_PI);
	    float lb_speed = sqrtf(lb_x * lb_x + lb_y * lb_y);

	    // Normalize speeds if any exceeds 1.0
	    float max_speed = fmaxf(fmaxf(rf_speed, lf_speed), fmaxf(rb_speed, lb_speed));
	    if (max_speed > 1.0f && max_speed > 0.0f) {
	    	rf_speed /= max_speed;
	        lf_speed /= max_speed;
	        rb_speed /= max_speed;
	        lb_speed /= max_speed;
	    }

//	    rf_angle = fmodf((rf_angle + 360.0f), 360.0f);
//	    lf_angle = fmodf((lf_angle + 360.0f), 360.0f);
//	    rb_angle = fmodf((rb_angle + 360.0f), 360.0f);
//	    lb_angle = fmodf((lb_angle + 360.0f), 360.0f);

	    // Update modules
	    SM_UpdateSteering(&moduleRF, rf_angle);
	    SM_UpdateDriving(&moduleRF, rf_speed);

	    SM_UpdateSteering(&moduleLF, lf_angle);
	    SM_UpdateDriving(&moduleLF, lf_speed);

	    SM_UpdateSteering(&moduleRB, rb_angle);
	    SM_UpdateDriving(&moduleRB, rb_speed);

	    SM_UpdateSteering(&moduleLB, lb_angle);
	    SM_UpdateDriving(&moduleLB, lb_speed);

#ifdef DEBUG_PRINT
	    // Debug prints for each wheel's speed and angle
	    printf("\n==== Wheel Data ====\n");
	    printf("RF -> Angle: %.2f°, Speed: %.2f\n", rf_angle, rf_speed);
	    printf("LF -> Angle: %.2f°, Speed: %.2f\n", lf_angle, lf_speed);
	    printf("RB -> Angle: %.2f°, Speed: %.2f\n", rb_angle, rb_speed);
	    printf("LB -> Angle: %.2f°, Speed: %.2f\n", lb_angle, lb_speed);
#endif // DEBUG_PRINT

#endif // TEST

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 170-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
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
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
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
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  huart3.Init.BaudRate = 500000;
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
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
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

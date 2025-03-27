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
#include "swerve_drive.h"
#include "swerve_module.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_PRINT
//#define TEST

// Robot dimensions (adjust according to actual measurements)
#define ROBOT_STEERING_GEAR_RATIO 2.0f // Gear ratio of steering motors
#define STEERING_ENCODER_RESOLUTION 1000.0 // Encoder resolution
#define TIMEOUT_MS 500         // Timeout for joystick disconnect detection

// Joystick smoothing parameters
#define JOYSTICK_DEADBAND 0.05f    // 5% deadband for joystick inputs
#define JOYSTICK_SMOOTHING 0.15f   // Smoothing factor (0.0 to 1.0)
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
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
TIM_HandleTypeDef htim20;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;  // For joystick

/* USER CODE BEGIN PV */
float xSpeed = 0.0f, ySpeed = 0.0f, rot = 0.0f;
float target_xSpeed = 0.0f, target_ySpeed = 0.0f, target_rot = 0.0f;
uint32_t lastJoystickUpdate = 0;  // Store the last time we received valid joystick data

bool lastButtonState;
bool throwState = false;
bool lastZalahState;
bool zalahState = false;
bool PASS_State = false;
bool ZALAH_State = false;

// Global state for previous angles
static float prev_angles[4] = {0};  // RF, LF, RB, LB
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
static void MX_UART5_Init(void);
static void MX_UART1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM5_Init(void);
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
	      .dir_gpio_port = IN2_GPIO_Port,
	      .dir_gpio_pin = IN2_Pin,
	      .pwm_tim = &htim3,
	      .pwm_channel = TIM_CHANNEL_2,
          .encoder_tim = &htim1,
	      .Kp = 5.0f,
	      .Ki = 0.1f,
	      .Kd = 0.01f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199,
          .prev_error = 0.0f,
          .integral = 0.0f
	  },
	  .driving = {
	      .uart = &huart4,
	      .axis = 0,
	      .max_velocity = 2.0f,  // 2 turns/s max velocity
	      .max_current = 10.0f   // 10A max current
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8)
  };

  SwerveModule moduleLF = {	// Configuration moduleLF
	  .steering = {
	      .dir_gpio_port = IN4_GPIO_Port,
	      .dir_gpio_pin = IN4_Pin,
	      .pwm_tim = &htim3,
	      .pwm_channel = TIM_CHANNEL_4,
          .encoder_tim = &htim4,
	      .Kp = 2.5f,
	      .Ki = 0.1f,
	      .Kd = 0.01f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199,
          .prev_error = 0.0f,
          .integral = 0.0f
	  },
	  .driving = {
	      .uart = &huart4,
	      .axis = 1,
	      .max_velocity = 2.0f,
	      .max_current = 10.0f
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8)
  };

  SwerveModule moduleRB = {	// Configuration moduleRB
	  .steering = {
	      .dir_gpio_port = IN1_GPIO_Port,
	      .dir_gpio_pin = IN1_Pin,
	      .pwm_tim = &htim3,
	      .pwm_channel = TIM_CHANNEL_1,
          .encoder_tim = &htim8,
		  .Kp = 2.5f,
		  .Ki = 0.1f,
		  .Kd = 0.01f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199,
          .prev_error = 0.0f,
          .integral = 0.0f
	  },
	  .driving = {
	      .uart = &huart5,
	      .axis = 0,
	      .max_velocity = 2.0f,
	      .max_current = 10.0f
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8)
  };

  SwerveModule moduleLB = {	// Configuration moduleLB
	  .steering = {
	      .dir_gpio_port = IN3_GPIO_Port,
	      .dir_gpio_pin = IN3_Pin,
	      .pwm_tim = &htim3,
	      .pwm_channel = TIM_CHANNEL_3,
          .encoder_tim = &htim20,
	      .Kp = 2.5f,
	      .Ki = 0.1f,
	      .Kd = 0.01f,
	      .integral_limit = 500.0f,
	      .dt = 0.1f,
		  .max_pwm = 199,
          .prev_error = 0.0f,
          .integral = 0.0f
	  },
	  .driving = {
	      .uart = &huart5,
	      .axis = 1,
	      .max_velocity = 2.0f,
	      .max_current = 10.0f
	  },
	  .counts_per_degree = ROBOT_STEERING_GEAR_RATIO / (float)(STEERING_ENCODER_RESOLUTION * 8)
  };
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  JOYSTICK_Init(&huart1);  // Pass your UART handle
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
  MX_UART5_Init();
  MX_UART1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // Initialize ODrive UART
  ODrive_UART_Init();
  SwerveDrive_Init();
  // Initialize joystick
  Joystick_Init();

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
  // Before setting PWM values:
  HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_SET);
  // Repeat for other motors as needed
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
//      printf("ESC calibrate starting..\n");
//
//      SM_CalibrateESC(&moduleRF.driving);
//      SM_CalibrateESC(&moduleLF.driving);
//      SM_CalibrateESC(&moduleRB.driving);
//      SM_CalibrateESC(&moduleLB.driving);
//
//      printf("ESC calibrate done.\n");
      /* ..... Perform your action ..... */
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    JOYSTICK_Process();

    if (JOYSTICK_NewDataAvailable()) {
        JoystickData data = JOYSTICK_GetData();
        lastJoystickUpdate = HAL_GetTick();  // Reset timeout timer

        // Convert raw joystick values to normalized values (-1 to 1)
        float raw_x = (float)data.axisX / 512.0f;
        float raw_y = (float)data.axisY / -512.0f;
        float raw_rot = (float)data.axisRX / -512.0f;

        // Apply deadband
        if (fabsf(raw_x) < JOYSTICK_DEADBAND) raw_x = 0.0f;
        if (fabsf(raw_y) < JOYSTICK_DEADBAND) raw_y = 0.0f;
        if (fabsf(raw_rot) < JOYSTICK_DEADBAND) raw_rot = 0.0f;

        // Apply smoothing
        target_xSpeed = target_xSpeed + (raw_x - target_xSpeed) * JOYSTICK_SMOOTHING;
        target_ySpeed = target_ySpeed + (raw_y - target_ySpeed) * JOYSTICK_SMOOTHING;
        target_rot = target_rot + (raw_rot - target_rot) * JOYSTICK_SMOOTHING;

        // Limit rate of change
        float x_diff = target_xSpeed - xSpeed;
        float y_diff = target_ySpeed - ySpeed;
        float rot_diff = target_rot - rot;

        xSpeed += fmaxf(fminf(x_diff, MAX_CHANGE_RATE), -MAX_CHANGE_RATE);
        ySpeed += fmaxf(fminf(y_diff, MAX_CHANGE_RATE), -MAX_CHANGE_RATE);
        rot += fmaxf(fminf(rot_diff, MAX_CHANGE_RATE), -MAX_CHANGE_RATE);

        #ifdef DEBUG_PRINT
            printf("Raw: X=%.2f Y=%.2f R=%.2f\n", raw_x, raw_y, raw_rot);
            printf("Target: X=%.2f Y=%.2f R=%.2f\n", target_xSpeed, target_ySpeed, target_rot);
            printf("Current: X=%.2f Y=%.2f R=%.2f\n", xSpeed, ySpeed, rot);
        #endif

        // Add data validation
        if(xSpeed < -1.0f || xSpeed > 1.0f ||
           ySpeed < -1.0f || ySpeed > 1.0f ||
           rot < -1.0f || rot > 1.0f) {
            xSpeed = 0.0f;
            ySpeed = 0.0f;
            rot = 0.0f;
            target_xSpeed = 0.0f;
            target_ySpeed = 0.0f;
            target_rot = 0.0f;
            printf("Invalid joystick data!\r\n");
        }

        printf("Buttons (Hex): 0x%04X\n", data.buttons);

//        HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, (data.buttons & 0x0001) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, (data.buttons & 0x0002) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // Control relays based on buttons
        if (data.buttons & 0x0001) { // sungah tsylinder
            HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
            printf("relay1\n");
        } else {
            HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
        }

//        if (data.buttons & 0x0002) { // deeshee bombog ogoh tsylinder
//            HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
//            printf("relay2\n");
//            PASS_State = true;
//        } else {
//            HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
//            PASS_State = false;
//        }


        // Assume we toggle using button bit 0.
        bool currentZalahState = (data.buttons & 0x0004) != 0;

        // Detect rising edge: current is pressed and last was not.
        if (currentZalahState && !lastZalahState) {
            zalahState = !zalahState;  // Toggle relay state

            if (zalahState) { // Zalah 		damjuulah tsylinder ajilaagu bh ystoi
                printf("zalah\n");
                HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
                HAL_Delay(3000);
                HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 140);
                HAL_Delay(600);

                HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 140);
                HAL_Delay(700);

                __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CH, 0);
                HAL_Delay(1000);

                HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
                HAL_Delay(4000);
                zalahState = false;
            } else {
                __HAL_TIM_SET_COMPARE(&MOTOR1_TIM, MOTOR1_CH, 0);
            }
        }

        // Update last button state for edge detection in the next loop iteration.
        lastZalahState = currentZalahState;


//            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 100);
//            __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 100);
//            __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 100);

//        if (data.buttons & 0x0008) { // Throw
//            // Set motor speeds
//            printf("shideh\n");
//            __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CH, 150);
//            __HAL_TIM_SET_COMPARE(&MOTOR3_TIM, MOTOR3_CH, 120);
//
////            Motor_SetSpeed(&MOTOR2_TIM, MOTOR2_CH, 150);
////            Motor_SetSpeed(&MOTOR3_TIM, MOTOR3_CH, 120);
//        } else {
//            // Set motor speeds
////            __HAL_TIM_SET_COMPARE(&MOTOR2_TIM, MOTOR2_CH, 0);
////            __HAL_TIM_SET_COMPARE(&MOTOR3_TIM, MOTOR3_CH, 0);
//        }

        // Assume we toggle using button bit 0.
        bool currentButtonState = (data.buttons & 0x0008) != 0;

        // Detect rising edge: current is pressed and last was not.
        if (currentButtonState && !lastButtonState) {
            throwState = !throwState;  // Toggle relay state

            if (throwState) {
                printf("shideh\n");
                HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 150);
                __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 120);
            } else {
                __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
            }
        }

        // Update last button state for edge detection in the next loop iteration.
        lastButtonState = currentButtonState;
    }
    HAL_Delay(10);

    // If no joystick data received for TIMEOUT_MS, stop the motors
    if (HAL_GetTick() - lastJoystickUpdate > TIMEOUT_MS) {
        xSpeed = 0.0f;
        ySpeed = 0.0f;
        rot = 0.0f;
        target_xSpeed = 0.0f;
        target_ySpeed = 0.0f;
        target_rot = 0.0f;

        printf("Joystick connection lost.\n");
        lastJoystickUpdate = HAL_GetTick();
    }

    printf("xSpeed: %f, ySpeed: %f, Rot: %f\n", xSpeed, ySpeed, rot);

#ifdef TEST
//		if(i<=4) {
//			target_angle += 45.0f;
//			i = 0;
//		}
//
//    	int i = 0;
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
		SwerveDrive_Update(xSpeed, ySpeed, rot);

#ifdef DEBUG_PRINT
	    // Debug prints for each wheel's speed and angle
//	    printf("\n==== Wheel Data ====\n");
//	    printf("RF -> Angle: %.2f°, Speed: %.2f\n", rf_angle, rf_speed);
//	    printf("LF -> Angle: %.2f°, Speed: %.2f\n", lf_angle, lf_speed);
//	    printf("RB -> Angle: %.2f°, Speed: %.2f\n", rb_angle, rb_speed);
//	    printf("LB -> Angle: %.2f°, Speed: %.2f\n", lb_angle, lb_speed);
#endif // DEBUG_PRINT

//	    HAL_Delay(10);

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
  htim3.Init.Prescaler = 170-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 170-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 170-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 199;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; // Changed from SET  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 170-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 199;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{
  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
  * @brief UART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART1_Init(void)
{
  /* USER CODE BEGIN UART1_Init 0 */

  /* USER CODE END UART1_Init 0 */

  /* USER CODE BEGIN UART1_Init 1 */

  /* USER CODE END UART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART1_Init 2 */

  /* USER CODE END UART1_Init 2 */
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

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, (IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin | MOTOR3_Pin | MOTOR2_Pin), GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, (RELAY2_Pin | RELAY1_Pin), GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin
                           MOTOR3_Pin MOTOR2_Pin */
  GPIO_InitStruct.Pin = (IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin | MOTOR3_Pin | MOTOR2_Pin);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY2_Pin RELAY1_Pin */
  GPIO_InitStruct.Pin = (RELAY2_Pin | RELAY1_Pin);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR1_Pin */
  GPIO_InitStruct.Pin = MOTOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MOTOR1_GPIO_Port, &GPIO_InitStruct);

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

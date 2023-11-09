/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern BNO055_t IMU_BNO055_struct;
extern COMMAND_t COMMAND_struct;
extern PROPULSION_t HDW_CONTROLLER_struct;

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


/*Structure definition for math.h pid*/
arm_pid_instance_f32 hpid_pitch;
arm_pid_instance_f32 hpid_yaw;
arm_pid_instance_f32 hpid_roll;
arm_pid_instance_f32 hpid_speed;
arm_pid_instance_f32 hpid_altitude;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId LedTaskHandle;
osThreadId Roll_PIDHandle;
osThreadId Pitch_PIDHandle;
osThreadId Yaw_PIDHandle;
osMutexId I2C_ControllerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLedTask(void const * argument);
void StartRoll_PID(void const * argument);
void StartPitch_PID(void const * argument);
void StartYaw_PID(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of I2C_Controller */
  osMutexDef(I2C_Controller);
  I2C_ControllerHandle = osMutexCreate(osMutex(I2C_Controller));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */



  /*Configure output for timer*/
  PropulsionAndControl_Init(&HDW_CONTROLLER_struct, ESC_GPIO_PIN, ESC_GPIO_PORT, SERVO_LEFT_GPIO_PIN, SERVO_LEFT_GPIO_PORT, SERVO_RIGHT_GPIO_PIN, SERVO_RIGHT_GPIO_PORT, ESC_TIMER_CHANNEL_NBR, SERVO_LEFT_TIMER_CHANNEL_NBR, SERVO_RIGHT_TIMER_CHANNEL_NBR, &htim4);

  /*Init for IMU sensors*/
  BNO055_Init(&hi2c2, &IMU_BNO055_struct);







  /* Create the thread(s) */
  /* definition and creation of LedTask */
  osThreadDef(LedTask, StartLedTask, osPriorityNormal, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of Roll_PID */
  osThreadDef(Roll_PID, StartRoll_PID, osPriorityLow, 0, 128);
  Roll_PIDHandle = osThreadCreate(osThread(Roll_PID), NULL);

  /* definition and creation of Pitch_PID */
  osThreadDef(Pitch_PID, StartPitch_PID, osPriorityIdle, 0, 128);
  Pitch_PIDHandle = osThreadCreate(osThread(Pitch_PID), NULL);

  /* definition and creation of Yaw_PID */
  osThreadDef(Yaw_PID, StartYaw_PID, osPriorityIdle, 0, 128);
  Yaw_PIDHandle = osThreadCreate(osThread(Yaw_PID), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  Function implementing the LedTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    vTaskDelay(1000);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartRoll_PID */
/**
* @brief Function implementing the Roll_PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRoll_PID */
void StartRoll_PID(void const * argument)
{
  /* USER CODE BEGIN StartRoll_PID */


	/*PID factor init*/
	Pid_Init(&hpid_roll, PID_KP_ROLL, PID_KI_ROLL, PID_KD_ROLL);

  /* Infinite loop */
	for(;;)
	{	/*Read roll axis data*/
		xSemaphoreTake(I2C_ControllerHandle,25);
		BNO055_ReadEuler_Roll(&hi2c2, &IMU_BNO055_struct);
		xSemaphoreGive(I2C_ControllerHandle);

		/*Compensate PID*/
		Pid_CompensateRoll(&hpid_roll, &COMMAND_struct, &IMU_BNO055_struct, &HDW_CONTROLLER_struct);


		vTaskDelay(150);
	}
  /* USER CODE END StartRoll_PID */
}

/* USER CODE BEGIN Header_StartPitch_PID */
/**
* @brief Function implementing the Pitch_PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPitch_PID */
void StartPitch_PID(void const * argument)
{
  /* USER CODE BEGIN StartPitch_PID */
	Pid_Init(&hpid_pitch, PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH);
  /* Infinite loop */
  for(;;)
  {
	  	/*Read pitch axis data*/
		xSemaphoreTake(I2C_ControllerHandle,25);
		BNO055_ReadEuler_Pitch(&hi2c2, &IMU_BNO055_struct);
		xSemaphoreGive(I2C_ControllerHandle);

		/*Compensate PID */
		Pid_CompensatePitch(&hpid_pitch, &COMMAND_struct, &IMU_BNO055_struct, &HDW_CONTROLLER_struct);

		vTaskDelay(150);
  }
  /* USER CODE END StartPitch_PID */
}

/* USER CODE BEGIN Header_StartYaw_PID */
/**
* @brief Process use to read IMU yaw axis and use PID to compensate plane movement.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartYaw_PID */
void StartYaw_PID(void const * argument)
{
  /* USER CODE BEGIN StartYaw_PID */
	Pid_Init(&hpid_yaw, PID_KP_YAW, PID_KI_YAW, PID_KD_YAW);
  /* Infinite loop */
  for(;;)
  {
	  	/*Read yaw axis data*/
		xSemaphoreTake(I2C_ControllerHandle,25);
		BNO055_ReadEuler_Yaw(&hi2c2, &IMU_BNO055_struct);
		xSemaphoreGive(I2C_ControllerHandle);

		/*Compensate PID step 1 - error calculation */
		Pid_CompensateYaw(&hpid_yaw, &COMMAND_struct, &IMU_BNO055_struct, &HDW_CONTROLLER_struct);

		vTaskDelay(150);
  }
  /* USER CODE END StartYaw_PID */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
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
#include "bsp_dwt.h"
#include "infantry_chassis.h"
#include "ins.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId insTaskHandle;
osThreadId wheelMotorTaskHandle;
osThreadId chassisTaskHandle;
osThreadId legMotorTaskHandle;
osThreadId ctrlcommTaskHandle;
osThreadId capTaskHandle;
osThreadId refereecommTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartInsTask(void const * argument);
void StartWheelMotorTask(void const * argument);
void StartChassisTask(void const * argument);
void StartLegMotorTask(void const * argument);
void StartCtrlCommTask(void const * argument);
void StartCapTask(void const * argument);
void StartRefereeTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of insTask */
  osThreadDef(insTask, StartInsTask, osPriorityRealtime, 0, 256);
  insTaskHandle = osThreadCreate(osThread(insTask), NULL);

  /* definition and creation of wheelMotorTask */
  osThreadDef(wheelMotorTask, StartWheelMotorTask, osPriorityHigh, 0, 128);
  wheelMotorTaskHandle = osThreadCreate(osThread(wheelMotorTask), NULL);

  /* definition and creation of chassisTask */
  osThreadDef(chassisTask, StartChassisTask, osPriorityAboveNormal, 0, 128);
  chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

  /* definition and creation of legMotorTask */
  osThreadDef(legMotorTask, StartLegMotorTask, osPriorityRealtime, 0, 128);
  legMotorTaskHandle = osThreadCreate(osThread(legMotorTask), NULL);

  /* definition and creation of ctrlcommTask */
  osThreadDef(ctrlcommTask, StartCtrlCommTask, osPriorityNormal, 0, 128);
  ctrlcommTaskHandle = osThreadCreate(osThread(ctrlcommTask), NULL);

  /* definition and creation of capTask */
  osThreadDef(capTask, StartCapTask, osPriorityRealtime, 0, 128);
  capTaskHandle = osThreadCreate(osThread(capTask), NULL);

  /* definition and creation of refereecommTask */
  osThreadDef(refereecommTask, StartRefereeTask, osPriorityNormal, 0, 256);
  refereecommTaskHandle = osThreadCreate(osThread(refereecommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartInsTask */
/**
 * @brief Function implementing the insTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInsTask */
void StartInsTask(void const * argument)
{
  /* USER CODE BEGIN StartInsTask */
  static float ins_start;
  static float ins_dt;
  INS_Init();
  /* Infinite loop */
  for (;;) {
    ins_start = DWT_GetTimeline_ms();
    INS_Task();
    ins_dt = DWT_GetTimeline_ms() - ins_start;
    osDelay(1);
  }
  /* USER CODE END StartInsTask */
}

/* USER CODE BEGIN Header_StartWheelMotorTask */
/**
 * @brief Function implementing the wheelMotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartWheelMotorTask */
void StartWheelMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartWheelMotorTask */
  static float wheel_start;
  static float wheel_dt;

  /* Infinite loop */
  for (;;) {
    wheel_start = DWT_GetTimeline_ms();
    WheelMotorTask();
    wheel_dt = DWT_GetTimeline_ms() - wheel_start;
    osDelay(1);
  }
  /* USER CODE END StartWheelMotorTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
 * @brief Function implementing the chassisTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  static float chassis_start;
  static float chassis_dt;

  /* Infinite loop */
  for (;;) {
    chassis_start = DWT_GetTimeline_ms();
    ChassisCalcTask();
    chassis_dt = DWT_GetTimeline_ms() - chassis_start;
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartLegMotorTask */
/**
 * @brief Function implementing the legMotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLegMotorTask */
void StartLegMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartLegMotorTask */
  static float leg_start;
  static float leg_dt;

  /* Infinite loop */
  for (;;) {
    leg_start = DWT_GetTimeline_ms();
    UnitreeMotorTask();
    leg_dt = DWT_GetTimeline_ms() - leg_start;
    osDelay(1);
  }
  /* USER CODE END StartLegMotorTask */
}

/* USER CODE BEGIN Header_StartCtrlCommTask */
/**
 * @brief Function implementing the ctrlcommTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCtrlCommTask */
void StartCtrlCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCtrlCommTask */
  static float ctrl_start;
  static float ctrl_dt;

  /* Infinite loop */
  for (;;) {
    ctrl_start = DWT_GetTimeline_ms();
    CtrlCommTask();
    ctrl_dt = DWT_GetTimeline_ms() - ctrl_start;
    osDelay(5);
  }
  /* USER CODE END StartCtrlCommTask */
}

/* USER CODE BEGIN Header_StartCapTask */
/**
* @brief Function implementing the capTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCapTask */
void StartCapTask(void const * argument)
{
  /* USER CODE BEGIN StartCapTask */
  static float cap_start;
  static float cap_dt;

  /* Infinite loop */
  for(;;)
  {
    cap_start = DWT_GetTimeline_ms();
    CapTask();
    cap_dt = DWT_GetTimeline_ms() - cap_start;
    osDelay(10);
  }
  /* USER CODE END StartCapTask */
}

/* USER CODE BEGIN Header_StartRefereeTask */
/**
* @brief Function implementing the refereecommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRefereeTask */
void StartRefereeTask(void const * argument)
{
  /* USER CODE BEGIN StartRefereeTask */
  /* Infinite loop */
  for(;;)
  {
    UITask();
  }
  /* USER CODE END StartRefereeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/* USER CODE BEGIN Header */
/*
 * 创建后台、通信、IMU、策略推理、执行五个任务，并规定各自等待方式。
 * 执行任务优先级最高，等TIM6基础节拍；推理等分频节拍；其他任务轮询后osDelay。
 * 实际协议/控制代码在imcalib/task，入口循环只负责初始化、等待和调用。
 */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "robot_control.h"
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
osThreadId commTaskHandle;
osThreadId imuTaskHandle;
osThreadId policyTaskHandle;
osThreadId actuationTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void commTask_Entry(void const * argument);
void imuTask_Entry(void const * argument);
void policyTask_Entry(void const * argument);
void actuationTask_Entry(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* 先创建TIM6要释放的两个信号量并清算法状态，之后才创建会启动TIM6的执行任务。 */
  Robot_Control_Init();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of commTask */
  osThreadDef(commTask, commTask_Entry, osPriorityAboveNormal, 0, 512);
  commTaskHandle = osThreadCreate(osThread(commTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, imuTask_Entry, osPriorityHigh, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of policyTask */
  osThreadDef(policyTask, policyTask_Entry, osPriorityHigh, 0, 2048);
  policyTaskHandle = osThreadCreate(osThread(policyTask), NULL);

  /* definition and creation of actuationTask */
  osThreadDef(actuationTask, actuationTask_Entry, osPriorityRealtime, 0, 1024);
  actuationTaskHandle = osThreadCreate(osThread(actuationTask), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    /* 后台诊断和灯效，每轮完成后让出5个OS tick；并非严格5ms周期调度。 */
    background_task_body();
    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_commTask_Entry */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_commTask_Entry */
void commTask_Entry(void const * argument)
{
  /* USER CODE BEGIN commTask_Entry */
  for(;;)
  {
    /* 整理接收反馈、腿解算、故障和使能；让出1个OS tick后再轮询。 */
    comm_task_body();
    osDelay(1);
  }
  /* USER CODE END commTask_Entry */
}

/* USER CODE BEGIN Header_imuTask_Entry */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imuTask_Entry */
void imuTask_Entry(void const * argument)
{
  /* USER CODE BEGIN imuTask_Entry */
  imu_task_init();
  for(;;)
  {
    /* 只处理HI229新样本；模块时间戳相同会直接返回，循环不代表新采样到达。 */
    imu_task_body();
    osDelay(1);
  }
  /* USER CODE END imuTask_Entry */
}

/* USER CODE BEGIN Header_policyTask_Entry */
/**
* @brief Function implementing the policyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_policyTask_Entry */
void policyTask_Entry(void const * argument)
{
  /* USER CODE BEGIN policyTask_Entry */
  ctrl_task_init();
  for(;;)
  {
    /* 等待TIM6每MACHINE_POLICY_DIV拍释放一次，再运行一帧网络推理。 */
    osSemaphoreWait(policy_tick_sem_handle, osWaitForever);
    ctrl_task_body();
    // osDelay(10);
  }
  /* USER CODE END policyTask_Entry */
}

/* USER CODE BEGIN Header_actuationTask_Entry */
/**
* @brief Function implementing the actuationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_actuationTask_Entry */
void actuationTask_Entry(void const * argument)
{
  /* USER CODE BEGIN actuationTask_Entry */
  output_task_init();
  for(;;)
  {
    /* 每个TIM6基础节拍执行一次反馈→选路→算力矩→发送；不叠加osDelay。 */
    osSemaphoreWait(ctrl_tick_sem_handle, osWaitForever);
    output_task_body();
    // osDelay(1);
  }
  /* USER CODE END actuationTask_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

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
#include "shared_variables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Main Error Handler that decides SD opening and notifies with CAN error message */
TaskHandle_t ACCU_Error_Handle;

/* Reading GPIOs AIR or PC or other state handler */
TaskHandle_t Read_Inputs_Handle;

/* TS Handler Tasks */
TaskHandle_t PreCharge_Handle;
TaskHandle_t TS_Active_Handle;
TaskHandle_t TS_OFF_Handle;

/* BMS Task Handler */
TaskHandle_t BMS_Main_TaskHandle;
TaskHandle_t BMS_Communication_TaskHandle;

/* SD Card Handler */
TaskHandle_t SDCard_Handle;

TaskHandle_t Charge_Handle;

TaskHandle_t Canbus_TaskHandle;

/* USB handler */
TaskHandle_t USB_TaskHandle;

/* AIR Situation and Pre-charge Event Group */
EventGroupHandle_t xPreCharge_EventGroup;
EventBits_t PreCharge_EventBits;
PreCharge_EventBits_t PreCharge_EventBits_now_set;

EventGroupHandle_t xTS_Off_EventGroup;
EventBits_t TS_Off_EventBits;
TS_Off_EventBits_t TS_Off_EventBits_now_set;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern const float Em_Ah_LUT[100][2];

/* Variable that selects which CAN Frame will be transmitted each time */
TickType_t SD_tick, CANTx_time, CANTx_tick;
uint8_t    can_msg_counter;

/* USB jSON Transactions with BMS GUI */
uint8_t 		   usb_connected;
USBD_StatusTypeDef usb_result;

/* To check if ELCON was connected & Balancing variables */
TickType_t balancing_update_time;
bool       tsac_timer_stopped = true;

/* Error Variables */
/* bmsError's are found inside: "bmsInfo.state" variable */
//TSAC_Error_t tsacError = TSAC_OK;

/* Notifications that PreCharge was done and/or ACCU error has occurred
 * CAUTION: bms_notification were defined locally to avoid clearanceOnExit */
uint32_t pc_notification = 0,  error_notification = 0;

/* Time variables for efficiency on scheduling */
TickType_t volt_tick, temp_tick, stat_tick, cfgr_tick,
		   volt_read_time, temp_read_time, stat_read_time,
		   ADC_volt_tick, ADC_temp_tick, ADC_stat_tick;

/* Variable flags that allow LTC6811 to make ADC conversions
 * or read from respective register after an appropriate interval */
uint16_t pecErrors[16];
uint8_t first_volt_measurement_taken = 0, first_temp_measurement_taken = 0,
		ADC_Ready = 0, reg_locked = 0;
uint32_t volt_error, temp_error, stat_error;

/* Timing Variables for Error Handling of the BMS */
uint32_t no_overvoltage_time;
uint32_t no_undervoltage_time;
uint32_t no_overvoltage_flag_time;
uint32_t no_undervoltage_flag_time;
uint32_t no_overcurrent_charge_time;
uint32_t no_overcurrent_discharge_time;
uint32_t no_overtemperature_time;
uint32_t no_undertemperature_time;
uint32_t no_isabelle_voltage_error_time;

/* Shows how many times every 100ms PC error persists */
uint8_t counter_pc_error, counter_air_stuck;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osTimerId timCAN1Handle;
osTimerId timTSACBusHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ACCU_Error_Handler();

/* HV TS Checks - Handler Tasks & GPIO Input readings */
void read_Inputs_Task();
void TS_Activation_Task();
void PreCharge_Task();
void TS_OFF_Check();

/* BMS Handler Tasks */
void BMS_Main_Task();
void BMS_Communication_Task();

/* SD Handler Task */
void SDCard_Task();

/* USB Handler Task */
void USB_Task();

/* Charging - Elcon Task */
void Charge_Task();

/* CAN Bus Task */
void Canbus_Task();

/* Ah function in SoC Estimator */
extern float find_ah_from_voltage(float voltage);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void timCAN1_Tx_Callback(void const * argument);
void timTSACBus_Callback(void const * argument);

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

  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of timCAN1 */
//  osTimerDef(timCAN1, timCAN1_Tx_Callback);
//  timCAN1Handle = osTimerCreate(osTimer(timCAN1), osTimerPeriodic, NULL);

  /* definition and creation of timTSACBus */
  osTimerDef(timTSACBus, timTSACBus_Callback);
  timTSACBusHandle = osTimerCreate(osTimer(timTSACBus), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  //osTimerStart(timCAN1Handle, 100);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* ACCU Error Handler */
  xTaskCreate(ACCU_Error_Handler,     "ACCU_Error_Handle Task",  128, NULL, 11, &ACCU_Error_Handle);

  /* Read Input Handler */
  xTaskCreate(read_Inputs_Task,       "Read_Inputs_Handle Task", 128, NULL, 14, &Read_Inputs_Handle);

  /* TS Situation Handlers */
  xTaskCreate(TS_OFF_Check,           "TS_OFF_Handle Task"   ,   128, NULL,  9, &TS_OFF_Handle);
  xTaskCreate(PreCharge_Task,         "PreCharge_Handle Task",   128, NULL, 11, &PreCharge_Handle);
  xTaskCreate(TS_Activation_Task,     "TS_Active_Handle Task",   128, NULL, 14, &TS_Active_Handle);

  /* BMS Task Handling */
  xTaskCreate(BMS_Main_Task,          "BMS_Main_Handle Task",    500, NULL, 14, &BMS_Main_TaskHandle);
  xTaskCreate(BMS_Communication_Task, "BMS_ADC_Handle Task",     256, NULL, 14, &BMS_Communication_TaskHandle);

  /* CAN bus Task Handling */
  xTaskCreate(Canbus_Task,            "Canbus_Handle_Task",      128, NULL, 14, &Canbus_TaskHandle);

  /* Initialization of USB Task */
  xTaskCreate(USB_Task,               "USB_Handle Task",         600, NULL,  8, &USB_TaskHandle);

  /* SD Card writing Task */
  xTaskCreate(SDCard_Task,            "SDCard_Handle Task",      500, NULL,  8, &SDCard_Handle);

  xTaskCreate(Charge_Task,            "Charge_Handle_Task",      128, NULL,  9, &Charge_Handle);

  /* Pre-charge & AIR Status flags Control */
  xPreCharge_EventGroup = xEventGroupCreate();
  xTS_Off_EventGroup    = xEventGroupCreate();

  /* BMS Initializations */
  BMS_Init(&master, slave_array, &hspi2, GPIOC, GPIOC, SPI2_CS_Pin, AMS_OK_Pin, &bmsInfo, 1, 0);
  ADC_Ready = 1;

  // JUST FOR NOW
  //bmsInfo.BalInfo.balancing_enabled = 1;

  /* TSAC basic struct initialization */
  Accu_Struct_Init(&accuInfo, &can_handler_prim,
		  IMD_OK_GPIO_Port, IMD_OK_Pin,
		  PC_STATE_GPIO_Port, PC_STATE_Pin,
		  VS_OVER_60V_GPIO_Port, VS_OVER_60V_Pin,
		  AIR_M_Supp_GPIO_Port, AIR_M_Supp_Pin,
		  AIR_P_Supp_GPIO_Port, AIR_P_Supp_Pin,
		  AIR_M_State_GPIO_Port, AIR_M_State_Pin,
		  AIR_P_State_GPIO_Port, AIR_P_State_Pin,
		  AIR_P_DRIVER_GPIO_Port, AIR_P_DRIVER_Pin,
		  LED1_GPIO_Port, LED1_Pin);

  /* IVT Current Sensor and ELCON Charger struct initialization */
  IVT_Struct_Init(&ivt, &can_handler_tsac);
  Elcon_Struct_Init(&elcon, &can_handler_tsac);

  /* SD Card Mounting */
  sdCard.fresult = SD_Card_init(&sdCard, slave_array, &accuInfo, &bmsInfo, &ivt, &elcon, &imd, &p23status);

  /* ACCU CAN Starting */
  ACCU_CAN_Config();

  /* ADC Inputs */
  Enable_ADC_Conversions();

  /* Timer Configuration */
  Start_Timers();

  /* USB Initialization */
  USB_Init(USB_ENABLE_GPIO_Port, USB_ENABLE_Pin);

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
	  /* IMD status */
	  update_imd_status(&imd);

	  /* Checking how many PEC errors in BMS occur */
	  for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
		  pecErrors[id] = slave_array[id].pec_errors;

	  /* Calculate SoC of P23_ACCU */
//	  if((bmsInfo.max_total_Ah != 0) && (bmsInfo.min_total_Ah != 0))
//	  {
//		  bmsInfo.max_SoC = bmsInfo.max_initial_SoC + (ivt.Ah_consumed / bmsInfo.min_total_Ah) * 100;
//		  bmsInfo.min_SoC = bmsInfo.min_initial_SoC + (ivt.Ah_consumed / bmsInfo.max_total_Ah) * 100;
//	  }
//	  else
//	  {
//		  bmsInfo.min_SoC = 255;	// Error in Ah mapping!
//		  bmsInfo.max_SoC = 255;
//	  }
	  osDelay(50);
  }
  /* USER CODE END StartDefaultTask */
}



void Canbus_Task()
{
	for (;;)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		osDelay(1);

		CANTx_time = xTaskGetTickCount() - CANTx_tick;
		CANTx_tick  = xTaskGetTickCount();

		switch(can_msg_counter)
		{
		case 0:
			Cell_Voltage_CAN_Tx(&bmsInfo, &can_handler_prim);
			break;

		case 1:
			Cell_Temp_CAN_Tx(&bmsInfo, &can_handler_prim);
			break;

		case 2:
			TSAC_Energy_CAN_Tx(&bmsInfo, &ivt, &can_handler_prim);
			break;

		case 3:
			TSAC_Status_CAN_Tx(&bmsInfo, &accuInfo, &imd);
			break;

		case 4:
			Error_Msg_CAN_Tx(&accuInfo, &bmsInfo, &can_handler_prim);
			break;

		default:
			break;
		}

		can_msg_counter++;
		if (can_msg_counter > 4)
			can_msg_counter = 0;

		osDelay(100);
	}
}

/* timTSACBus_Callback function */
void timTSACBus_Callback(void const * argument)
{
	/* USER CODE BEGIN timTSACBus_Callback */
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	if((accuInfo.AIR_M_State == 1) && (accuInfo.AIR_P_State == 1))
	{
		Elcon_CAN_Tx(&elcon, &bmsInfo);
		Charger_CAN_Tx(&bmsInfo, &can_handler_tsac);  // For the charger's LCD display
		bmsInfo.mode = CHARGING;
	}
	else
	{
		Elcon_stop_charging(&elcon, &bmsInfo);
		bmsInfo.mode = STDBY;
	}

  /* USER CODE END timTSACBus_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Task that is notified only when an error from TSAC or AMS occurs */
void ACCU_Error_Handler()
{
  /* Infinite loop */
  for(;;)
  {
	  xTaskNotifyWait(0, 0xFFFFFFFF, &error_notification, portMAX_DELAY);

	  uint8_t error = ((error_notification == PC_RELAY_IMPLAUSIBILITY && accuInfo.precharge_relay_error));

	  if(error)
		  accuInfo.precharge_relay_error = true;

	  error |= (error_notification == (AIR_P_STUCK|AIR_M_STUCK));
	  error |= (error_notification == AIR_P_STUCK);
	  error |= (error_notification == AIR_M_STUCK);
	  error |= (error_notification == AIR_M_STUCK);
	  error |= (error_notification == AIR_M_IMPLAUSIBILITY);
	  error |= (error_notification == AIR_P_IMPLAUSIBILITY);
	  error |= (error_notification == PC_CIRCUIT_ERROR);
	  error |= (bmsInfo.state      != BMS_OK);

	  if(bmsInfo.BalInfo.state == on)
		  stop_balancing(&bmsInfo, &master, slave_array, 3);

	  /* In case of a preCharge relay error, open BMS SD relay and AIR+ driver */
	  if(error)
	  {
		  ams_fault(&master, &accuInfo);
		  HAL_GPIO_WritePin(accuInfo.AIR_P_Driver, accuInfo.AIR_P_Driver_pin, GPIO_PIN_RESET);
		  bmsInfo.charge_flag = 0;
		  bmsInfo.target_charge_current = 0;
		  osDelay(1);
	  }
	  osDelay(10);
  }
}


/* Task that constantly runs on reading AIR status, IMD status & Pre-Charge relay status */
void read_Inputs_Task()
{
  /* Infinite loop */
  for(;;)
  {
	  /* Reading AVI opto-coupler output, concerning if the voltage at vehicle side is over 60V. */
	  accuInfo.over60V_dclink = HAL_GPIO_ReadPin(accuInfo.VS_OVER60V, accuInfo.VS_OVER60V_Pin);

	  /* Reading PreCharge actual state based on PicKering's mechanical state */
	  accuInfo.precharge_actual_state
	  = !(HAL_GPIO_ReadPin(accuInfo.PC_Indicator, accuInfo.PC_Indicator_Pin));

	  /* Reading AIR's status */
	  accuInfo.AIR_M_Supp  = HAL_GPIO_ReadPin(accuInfo.AIR_M_Supp_3V,  accuInfo.AIR_M_Supp_3V_pin);
	  accuInfo.AIR_M_State = HAL_GPIO_ReadPin(accuInfo.AIR_M_State_3V, accuInfo.AIR_M_State_3V_pin);
	  accuInfo.AIR_P_Supp  = HAL_GPIO_ReadPin(accuInfo.AIR_P_Supp_3V,  accuInfo.AIR_P_Supp_3V_pin);
	  accuInfo.AIR_P_State = HAL_GPIO_ReadPin(accuInfo.AIR_P_State_3V, accuInfo.AIR_P_State_3V_pin);

	  /* Check AIR- Supply */
	  if(accuInfo.AIR_M_Supp)
	  {
		  xEventGroupSetBits(xPreCharge_EventGroup, AIR_M_SUPP);
	  }
	  else
	  {
		  /* In case of an open SD flag, we reset the appropriate flags */
		  HAL_GPIO_WritePin(accuInfo.AIR_P_Driver, accuInfo.AIR_P_Driver_pin, GPIO_PIN_RESET);
		  xEventGroupClearBits(xPreCharge_EventGroup, AIR_M_SUPP|PC_DONE);
		  xEventGroupSetBits(xPreCharge_EventGroup, TS_INACTIVE);

		  /* If SD opens the DC_Link will be discharged, so we need to reset all TS flags */
		  accuInfo.ts_active = false;
		  accuInfo.precharge_done = 0;
		  accuInfo.precharge_failed = 0;
	  }

	  /* Check AIR- State */
	  if(accuInfo.AIR_M_State)
	  {
		 xEventGroupSetBits(xPreCharge_EventGroup, AIR_M_STATE);
		 accuInfo.AIR_M_closed_time = xTaskGetTickCount();
	  }
	  else
		  xEventGroupClearBits(xPreCharge_EventGroup, AIR_M_STATE);

	  /* Check AIR+ Supply */
	  if(accuInfo.AIR_P_Supp)
		  xEventGroupSetBits(xPreCharge_EventGroup, AIR_P_SUPP);
	  else
		  xEventGroupClearBits(xPreCharge_EventGroup, AIR_P_SUPP);

	  /* Check AIR+ State */
	  if(accuInfo.AIR_P_State)
	  {
		  xEventGroupSetBits(xPreCharge_EventGroup, AIR_P_STATE);
	  }
	  else
	  {
		  xEventGroupClearBits(xPreCharge_EventGroup, AIR_P_STATE);
	  }
	  /* OVER_60V flag is set when VS voltage reaches at least 60V, so we know that PreCharge
	   * has started. Moreover, we want to show that TS is now active, so we reset TS_OFF flag */
	  if(accuInfo.over60V_dclink)
	  {
		  xEventGroupClearBits(xTS_Off_EventGroup, TS_OFF);
		  xEventGroupSetBits(xPreCharge_EventGroup, OVER_60V);
	  }
	  else
	  {
		  xEventGroupSetBits(xTS_Off_EventGroup, TS_OFF);
		  xEventGroupClearBits(xPreCharge_EventGroup, OVER_60V);
	  }
	  /* Pre-Charge Relay Error Handling:
	   * Must be constantly checked so as to provide that P23 won't start PreCharge,
	   * if the PC relay in on the wrong mechanical state */
	  if(accuInfo.precharge_actual_state != accuInfo.AIR_M_Supp)
	  {
		  if(counter_pc_error == 0)
		  {
			  accuInfo.tick_PC_Error = xTaskGetTickCount();
			  counter_pc_error++;
		  }

		  if(xTaskGetTickCount() - accuInfo.tick_PC_Error > PC_IMPLAUSIBILITY_TIME)
		  {
			  xEventGroupSetBits(xPreCharge_EventGroup, PC_RELAY_ERROR);
			  accuInfo.precharge_relay_error = true;
			  accuInfo.state 	  = PC_RELAY_IMPLAUSIBILITY;
			  accuInfo.last_error = PC_RELAY_IMPLAUSIBILITY;
			  xTaskNotify(ACCU_Error_Handle, PC_RELAY_IMPLAUSIBILITY, eSetValueWithOverwrite);
		  }
	  }
	  else if(accuInfo.precharge_actual_state == accuInfo.AIR_M_Supp && !accuInfo.precharge_relay_error)
	  {
		  counter_pc_error = 0;
		  xEventGroupClearBits(xPreCharge_EventGroup, PC_RELAY_ERROR);
	  }

	  /* Take Actions concerning all possible TSAC Errors:
	   * On the background AIR checks and Pre-Charge procedure will continue */
	  accuInfo.imd_error = !HAL_GPIO_ReadPin(accuInfo.IMD_ok, accuInfo.IMD_ok_Pin);
	  if(accuInfo.imd_error)
	  {
		  accuInfo.state	  = IMD_ERROR;	/* Verify IMD_ERROR */
		  accuInfo.last_error = IMD_ERROR;
		  xTaskNotify(ACCU_Error_Handle, IMD_ERROR, eSetValueWithOverwrite);
	  }
	  else if (accuInfo.state == IMD_ERROR) /* Resets only an IMD_ERROR */
	  {
		  accuInfo.state = TSAC_OK;
	  }

	  /* Check the PreCharge Event_Bits & TS_OFF Event bits that were set at this moment */
	  PreCharge_EventBits_now_set = xEventGroupGetBits(xPreCharge_EventGroup);
	  TS_Off_EventBits_now_set	  = xEventGroupGetBits(xTS_Off_EventGroup);
	  osDelay(100);
  }
}


/* Checking constantly while SD has not closed that an AIR is stuck & notify BMS Errors */
void TS_OFF_Check()
{
	for(;;)
	{
		/* TS_OFF Task is blocked until SD is opened at some point,
		 * so AIR_M_Supply is 0V: Initial or during error situation */
		xEventGroupWaitBits(xTS_Off_EventGroup, TS_OFF, pdTRUE, pdTRUE, portMAX_DELAY);
		xEventGroupWaitBits(xTS_Off_EventGroup, AIRM_STATE|AIRP_STATE, pdTRUE, pdFALSE, portMAX_DELAY);
		osDelay(50);

		/* Don't clear TS_OFF on exit: It must only be cleared from
		 * OVER_60V flag that indicates preCharge has started
		 * When the task is unblocked it will check only the AIR's Stuck situation
		 * If this situation exists for more than STUCK_AIR_INTERVAL, then show
		 * AIR Stuck error */
		if(TS_Off_EventBits == AIRP_STATE || TS_Off_EventBits == AIRM_STATE)
		{
			if(counter_air_stuck == 0)
			{
				counter_air_stuck++;
				accuInfo.tick_AIR_Stuck = xTaskGetTickCount();
			}

			/* In case AIR stuck situation is persistent for enough time */
			if(xTaskGetTickCount() - accuInfo.tick_AIR_Stuck > STUCK_AIR_INTERVAL)
			{
				accuInfo.air_stuck = true;

				if(accuInfo.AIR_M_State)
				{
					accuInfo.state 		= AIR_M_STUCK;
					accuInfo.last_error = AIR_M_STUCK;
				}

				else if(accuInfo.AIR_P_State)
				{
					accuInfo.state 		= AIR_P_STUCK;
					accuInfo.last_error = AIR_P_STUCK;
				}

				xTaskNotify(ACCU_Error_Handle, accuInfo.state, eSetValueWithOverwrite);
			}
		}
		osDelay(10);
	}
}


/* Runs only, when Pre-Charge is notified to start to see if
 * HV voltage is build up on the Vehicle side */
void PreCharge_Task()
{
	for(;;)
	{
		/* Wait for Notification from the PreCharge procedure */
		xTaskNotifyWait(0, 0xFFFFFFFF, &pc_notification, portMAX_DELAY);

		// PreCharge error add
		while(!accuInfo.precharge_done && !accuInfo.precharge_failed)
		{
			if((bmsInfo.accu_voltage > 432.0) && (ivt.voltage_vs > 0.93 * bmsInfo.accu_voltage))
			{
				accuInfo.precharge_failed = 0;
				accuInfo.precharge_voltage = ivt.voltage_U2 - ivt.voltage_vs;
				xEventGroupSetBits(xPreCharge_EventGroup, PC_DONE);
			}
			else
			{
				accuInfo.precharge_done = 0;
				accuInfo.precharge_failed = 0;
				xEventGroupClearBits(xPreCharge_EventGroup, PC_DONE);
			}
		}
		osDelay(5);
	}
}


/* Checks in order to activate/deactivate TS properly with no AIRs or PC errors */
void TS_Activation_Task()
{
	/* Infinite loop */
	for(;;)
	{
		/* Task is blocked until SD is closed and AIR- has 24V supply
		 *  No AIR- problem, so notify the Pre-Charge checks after PC relay
		 *  is OK, else check PreCharge_Event bits */
		PreCharge_EventBits = xEventGroupWaitBits(xPreCharge_EventGroup, AIR_M_SUPP|AIR_M_STATE|OVER_60V|TS_INACTIVE, pdTRUE, pdTRUE, portMAX_DELAY);

		/* Wait until PreCharge has started and DC_Link reaches at least 60V */
		if(PreCharge_EventBits == (AIR_M_SUPP|AIR_M_STATE|OVER_60V|TS_INACTIVE))
		{
			/* Notify PreCharge check task, for PreCharge Voltage control */
			xTaskNotify(PreCharge_Handle, 6969, eSetValueWithOverwrite);

			/* Wait until Pre-charge procedure is done & closing AIR+ can be done */
			PreCharge_EventBits = xEventGroupWaitBits(xPreCharge_EventGroup, AIR_M_SUPP|AIR_M_STATE|PC_DONE|OVER_60V, pdTRUE, pdTRUE, PC_DONE_TIME);

			if(PreCharge_EventBits == (AIR_M_SUPP|AIR_M_STATE|OVER_60V|PC_DONE))
			{
				/* If everything is OK, notify the task that has to close AIR+ */

  				HAL_GPIO_WritePin(accuInfo.AIR_P_Driver, accuInfo.AIR_P_Driver_pin, GPIO_PIN_SET);
				PreCharge_EventBits = xEventGroupWaitBits(xPreCharge_EventGroup, AIR_M_SUPP|AIR_M_STATE|AIR_P_SUPP|AIR_P_STATE|OVER_60V|PC_DONE, pdTRUE, pdTRUE, AIR_P_INTERVAL);

				if(PreCharge_EventBits == (AIR_M_SUPP|AIR_M_STATE|AIR_P_SUPP|AIR_P_STATE|OVER_60V|PC_DONE))
				{
					accuInfo.ts_active = true;
					xEventGroupClearBits(xPreCharge_EventGroup, TS_INACTIVE);

					accuInfo.precharge_done = 1;

					uint8_t data[8];
					data[1] = 0x01;		// To indicate for ASB & VCU control that TS is Active!
					if(CAN_Tx(&can_handler_prim, 0x304, 8, data) != HAL_OK);
				}
				/* Enters the else statement once in each precharge */
				else
					update_TSAC_Errors(PreCharge_EventBits, &accuInfo);
			}
			else
			{
				accuInfo.precharge_failed = 1;
				update_TSAC_Errors(PreCharge_EventBits, &accuInfo);
			}
		}
		else
			update_TSAC_Errors(PreCharge_EventBits, &accuInfo);

		osDelay(5);
	}
}


/* AMS TASK HANDLING */
/* Checks if there is communication timeout / slave-PCB error /
 * overTemp / underTemp /overVoltage / underVoltage / overCurrent */
void BMS_Main_Task()
{
//	uint8_t initial_soc_determined = 0;
	/* Infinite Loop */
	for(;;)
	{
		if (first_volt_measurement_taken && first_temp_measurement_taken)
		{
			/* Update BMS ErrorCode */
			update_BMS_Errors(&master, slave_array, &bmsInfo, &ivt, &accuInfo);

			/* Update only once total_Ah every time BMS opens-up */
//			if(!initial_soc_determined)
//			{
//				bmsInfo.min_total_Ah = find_ah_from_voltage(bmsInfo.min_voltage_cell.value);
//				bmsInfo.max_total_Ah = find_ah_from_voltage(bmsInfo.max_voltage_cell.value);
//				bmsInfo.min_initial_SoC  = 100.0 * bmsInfo.min_total_Ah / 15.4;
//				bmsInfo.max_initial_SoC  = 100.0 * bmsInfo.max_total_Ah / 15.4;
//				initial_soc_determined = 1;
//			}

			/* If the errorCode detects a BMS Error, notify ACCU_ErrorHandler() with the BMS ErrorCode value */
			if (bmsInfo.state == BMS_OK)
			{
				HAL_GPIO_WritePin(master.AMS_OK, master.AMS_OKPin, GPIO_PIN_SET);
				accuInfo.ams_error = 0;
			}
			else
				xTaskNotify(ACCU_Error_Handle, bmsInfo.state, eSetValueWithOverwrite);
		}
		osDelay(10);
	}
}



/* Implement successful ADC Conversion of cell voltages, temperatures and status measurements for BMS:
 * Update functionality of LTC6811 voltage, temperature & status registers  */
void BMS_Communication_Task()
{
	first_volt_measurement_taken = 0;
	first_temp_measurement_taken = 0;

	/* Infinite Loop */
	for(;;)
	{
		if(ADC_Ready && (!reg_locked))
		{
			if (xTaskGetTickCount() - ADC_volt_tick > 100)
			{
				ADC_volt_tick = xTaskGetTickCount();
				update_voltages(&master, slave_array, &bmsInfo, BC, ID_0, MD_0);
				for(uint8_t id = ID_0; id < SLAVES_NUM; id++)
					while (poll_status(&master, NO_BC, id) != OK)
						osDelay(5);
				reg_locked = 1;
				ADC_Ready  = 1;

				for(uint8_t id = ID_0; id < SLAVES_NUM; id++)
				{
					if(read_voltage_registers(&master, slave_array, &bmsInfo, id, 3) != OK)
						volt_error++;
				}
				volt_read_time = xTaskGetTickCount() - volt_tick;
				volt_tick = xTaskGetTickCount();
				reg_locked = 0;
				first_volt_measurement_taken = 1;
			}

			if (xTaskGetTickCount() - ADC_temp_tick > 350)
			{
				ADC_temp_tick = xTaskGetTickCount();
				update_temperatures(&master, BC, ID_0, MD_0);
				for(uint8_t id = ID_0; id < SLAVES_NUM; id++)
					while (poll_status(&master, NO_BC, id) != OK)
						osDelay(5);
				reg_locked = 1;
				ADC_Ready = 1;

				for(uint8_t id = ID_0; id < SLAVES_NUM; id++)
				{
					if(read_temperature_registers(&master, slave_array, id, 3) != OK)
						temp_error++;
				}
				temp_read_time = xTaskGetTickCount() - temp_tick;
				temp_tick = xTaskGetTickCount();
				reg_locked = 0;
				first_temp_measurement_taken = 1;
			}

			if (xTaskGetTickCount() - ADC_stat_tick > 600)
			{
				ADC_stat_tick = xTaskGetTickCount();
				update_status(&master, BC, ID_0, MD_0);
				for(uint8_t id = ID_0; id < SLAVES_NUM; id++)
					while (poll_status(&master, NO_BC, id) != OK)
						osDelay(5);
				reg_locked = 1;
				ADC_Ready = 1;

				for(uint8_t id = ID_0; id < SLAVES_NUM; id++)
				{
					if(read_status_registers(&master, slave_array, id, 3) != OK)
						stat_error++;
				}
				stat_read_time = xTaskGetTickCount() - stat_tick;
				stat_tick = xTaskGetTickCount();
				reg_locked = 0;
			}

			if (xTaskGetTickCount() - cfgr_tick > 2000)
			{
				cfgr_tick = xTaskGetTickCount();
				for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
					read_cfgr_register(&master, slave_array, id, 3);

				if (bmsInfo.state == SLAVE_ERROR)
				{
					uint8_t gpio[5] = {1,1,1,1,1};
					uint8_t no_dcc[9] = {0,0,0,0,0,0,0,0,0};
					safe_write_cfgr(&master, slave_array, BC, 0, gpio, 1, 0, UV_THRESHOLD, OV_THRESHOLD, no_dcc, 0, 3);
				}
			}
		}

		/* Task that handles balancing algorithm: Decides which cells require discharge or not */
		if(bmsInfo.BalInfo.balancing_enabled && (bmsInfo.state == BMS_OK))
		{
			if (balancing_update_time - xTaskGetTickCount() > 3000)
			{
				update_balancing_cells(&bmsInfo,  &master, slave_array, 5);
				balancing_update_time = xTaskGetTickCount();
			}
		}
		else if (bmsInfo.BalInfo.state != off)
			stop_balancing(&bmsInfo, &master, slave_array, 3);

		osDelay(5);
	}
}


/* Task that constantly writes data on SD Card */
void SDCard_Task()
{
	/* Infinite loop */
	for(;;)
	{
		sdCard.time_interval = xTaskGetTickCount() - SD_tick;
		SD_tick = xTaskGetTickCount();

		sdCard.connected = (!HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin));

		if (sdCard.mounted && (!sdCard.read_flag))
		{
			SD_card_write(&sdCard);
		}

		osDelay(200);
	}
}


/* jSON Transactions for GUI-Interface communication */
void USB_Task()
{
	/* Infinite Loop */
	for(;;)
	{
		/* Checks if a GUI device is connected to the USB COM Port */
		usb_connected = HAL_GPIO_ReadPin(VBUS_SENSE_GPIO_Port, VBUS_SENSE_Pin);
		if(usb_connected)
		{
			/* ------------------------- GUI TESTING BEGIN --------------------------- */
//			for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
//			{
//				uint8_t humidity = 0;
//				if ((id == 1) || (id == 3) || (id == 5) || (id == 7) || (id == 8) || (id == 10) || (id == 12) || (id == 14))
//					humidity = 1;
//
//				float v = 3.001;
//				float t = 10.5;
//				for (uint8_t cell = 0; cell < CELLS_NUM; ++cell)
//				{
//					slave_array[id].voltage[cell] = v;
//					v += 0.133;
//				}
//				for (uint8_t ntc = 0; ntc < NTCS_NUM; ++ntc)
//				{
//					if ((ntc == 4) && humidity)
//						slave_array[id].temp[ntc] = 0xFF;
//					else
//					{
//						slave_array[id].temp[ntc] = t;
//						t += 20;
//					}
//				}
//
//				if (humidity)
//					slave_array[id].humidity = 50.11 + id;
//				else
//					slave_array[id].humidity = 0xFF;
//			}
			/* ------------------------- GUI TESTING END --------------------------- */

			if (sdCard.read_flag)
			{
				SD_card_read(&sdCard);
				sdCard.read_flag = 0;
				osDelay(50);
			}
			else  /* Transmits only if the SD card does not use the USB to transmit its content */
			{
				usb_result = voltages_json(slave_array);
				osDelay(50);
				usb_result = balancing_json(slave_array);
				osDelay(50);
				usb_result = temperatures_json(slave_array);
				osDelay(50);
				usb_result = humidities_json(slave_array);
				osDelay(50);
				usb_result = Accu_json(&accuInfo);
				osDelay(50);
				usb_result = Ivt_json(&ivt);
				osDelay(50);
				usb_result = Elcon_json(&elcon);
				osDelay(50);
				usb_result = PECerrors_json(slave_array);
				osDelay(50);
			}
		}
		osDelay(150);
	}
}



void Charge_Task()
{
	for(;;)
	{
		/* Start Timer for TSAC Bus for Charging, only if ELCON charger verifies connection */
		if(elcon.connected)
		{
			if (tsac_timer_stopped)
			{
				/* This timer sends Elcon / Charger PCB the necessary messages */
				osTimerStart(timTSACBusHandle, 950);
				tsac_timer_stopped = false;
			}
		}

		/* If ELCON charger hasn't responded for at least CHARGER_TIMEOUT_MS
		 * stop the Timer that transmits its messages and reset the connection */
		if((xTaskGetTickCount() - elcon.last_msg_received > CHARGER_TIMEOUT_MS) && elcon.connected)
		{
			elcon.connected = false;
			bmsInfo.BalInfo.balancing_enabled = 0;
			tsac_timer_stopped = true;
			osTimerStop(timTSACBusHandle);
		}

		osDelay(100);
	}
}


/* USER CODE END Application */

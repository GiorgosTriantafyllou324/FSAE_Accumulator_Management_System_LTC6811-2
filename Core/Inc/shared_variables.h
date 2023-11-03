/*
 * shared_variables.h
 *
 *  Created on: Mar 31, 2023
 *      Author: gamin
 */

#ifndef INC_SHARED_VARIABLES_H_
#define INC_SHARED_VARIABLES_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"

/* Peripheral Includes */
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "can.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* BMS Includes */
#include <ACCU_base_lib.h>
#include <ACCU_CAN_functions.h>
#include <LTC6811.h>
#include <SD_Card.h>
#include <JsonUsb.h>

/* AIR parameters defines */
#define PC_IMPLAUSIBILITY_TIME 	100000
#define AIR_M_INTERVAL  		100
#define AIR_P_INTERVAL  		100
#define STUCK_AIR_INTERVAL		100000
#define OVER_60V_TIME			100000
#define PC_DONE_TIME			3000

/* BMS structure */
extern MCU 	     master;
extern LTC6811   slave_array[16];
extern BMS_info  bmsInfo;

/* Other TSAC structs */
extern Imd        imd;
extern Ivt 	      ivt;
extern Accu_info  accuInfo;
extern Elcon      elcon;

/* P23 info */
extern P23_status p23status;

/* CAN Handler structure */
extern CAN_Handler can_handler_tsac;  // CAN1
extern CAN_Handler can_handler_prim;  // CAN2

/* SD Card structure and main variables */
extern SD_Card sdCard;

/* FreeRTOS objects required */
extern TaskHandle_t ACCU_Error_Handle;
extern BaseType_t xHigherPriority_DCDCTemp;

/* CAN Variables */
extern CAN_RxHeaderTypeDef RxHeader1, RxHeader2;
extern CAN_TxHeaderTypeDef TxHeader1, TxHeader2;
extern uint8_t RxData1[8], RxData2[8], TxData1[8], TxData2[8];
extern uint32_t canMailbox;

/* Timer for FAN Control*/
extern TIM_HandleTypeDef htim3;

/* Timer for BMS Delays and Timing measurements */
extern TIM_HandleTypeDef htim1;

/* For Debugging */
extern uint32_t debug_mode;

#endif /* INC_SHARED_VARIABLES_H_ */

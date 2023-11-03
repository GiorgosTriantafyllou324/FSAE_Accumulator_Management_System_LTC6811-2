#ifndef LTC6811_BOARD_H_
#define LTC6811_BOARD_H_

#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"

/* INCLUDES GENERAL BMS CONSTANTS USED IN THIS SPECIFIC APPLICATION */

/* General battery pack constants  */
#define SLAVES_NUM 		 16
#define CELLS_NUM 		 9
#define NTCS_NUM 		 5
#define RH_SENSORS_NUM   1   // Humidity Sensors in each PCB
#define RH_GPIO_NUM      4   // Indicates in which slave GPIO (0 to 4) the humidity sensor is connected
#define MAX_ATTEMPTS     10

/*  The operating voltage of the batteries is 3V --> 4.2V  */
#define UV_THRESHOLD 	 3.050f
#define OV_THRESHOLD 	 4.195f
#define UT_THRESHOLD 	 10     /*under temperature threshold */
#define OT_THRESHOLD     60 // - 1

/* Voltage constants used in the balancing process */
#define LOW_V   3.65
#define MID_V   3.85
#define HIGH_V  4.19

/* NTC voltage divider constants */
#define V_REF                   3.003f
#define NTC_CONST_RESISTOR_KOHM 10
#define NTC_LUT_LENGTH          201

/* ShutDown timing constants - Watchdogs */
#define COMM_TIMEOUT_MS    450  /* Time in ms after which the communication to a slave is considered lost and SDC must open */
#define CURRENT_TIMEOUT_MS 450  /* Time in ms after which the current is considered out of range and SDC must open */
#define VOLTAGE_TIMEOUT_MS 450  /* Time in ms after which a cell voltage is considered out of range and SDC must open */
#define TEMP_TIMEOUT_MS    900  /* Time in ms after which an NTC's temperature is considered out of range and SDC must open */
#define CHARGER_TIMEOUT_MS 3000 /* Time in ms after which no message has been received from the ELCON charger */
#define IVT_TIMEOUT_MS     200  /* Time in ms after which no message has been received from the IVT current sensor */

#define DIAGNOSE_ENABLED   0    /* 0: No diagnostic check will run during BMS_Init() */


extern const float NTC_LUT[NTC_LUT_LENGTH];
extern float NTC_voltage[NTC_LUT_LENGTH];
extern const uint8_t ntc_to_cell_position[SLAVES_NUM][NTCS_NUM];

/* BMS Mode */
typedef enum
{
	STDBY 	  = 0,
	CHARGING     ,

}BMS_Mode_t;


#endif /* LTC6811_BOARD_H_ */

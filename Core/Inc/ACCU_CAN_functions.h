#ifndef INC_ACCU_CAN_FUNCTIONS_H_
#define INC_ACCU_CAN_FUNCTIONS_H_

#include <ACCU_base_lib.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#include <LTC6811.h>

/* IsabellenHuette IDs & Configuration messages */
#define IVT_I 	0x521
#define IVT_U1 	0x522
#define IVT_U2 	0x523
#define IVT_U3 	0x524
#define IVT_AH  0x527
#define IVT_WH  0x528

/* ELCON messages and Status defines */
#define SET_OV		1    /* ELCON received greater voltage than supported */
#define SET_OC		2    /* ELCON received greater current than supported */

/* Charger Board ID */
#define CHARGER_ID 0x18FFFFF5

/* Balancing Task Manager */
extern SemaphoreHandle_t xBalancingSemaphore;
extern BaseType_t   	 xHigherPriority_Balancing;

/* CanBus Variables */
typedef struct CAN_Handler
{
	CAN_HandleTypeDef*   hcan;
	CAN_RxHeaderTypeDef* RxHeader;
	CAN_TxHeaderTypeDef* TxHeader;
	uint8_t*			 RxData;
	uint8_t*			 TxData;
	uint32_t*			 canMailbox;
	uint32_t 			 canFIFO;
	uint32_t 			 canRx_errors;
	uint32_t 			 canTx_errors;
	uint32_t 			 can_error_time;
	TickType_t 			 current;

}CAN_Handler;

/* Struct that contains several info concerning P23 (P22.5 gkouxou gkouxou) status */
typedef struct{

	/* EMRAX stuff */
	int16_t actual_torque, requested_torque;
	int16_t motor_rpm;

	/* SD flags */
	uint8_t bspd_status,
			ts_off,
			rtd_done,
			vcu_alive,
			accu_alive;

	uint8_t power_limiter;

	/* Sensor info */
	uint8_t apps1,
			apps2;
	uint8_t brf,
			brr;

	/* TSAL handling errors */
	uint8_t tsal_impl_air_m,
			tsal_impl_air_p,
			tsal_impl_accu,
			tsal_impl_pc_state;

	uint8_t vcu_flags;
	uint8_t vcu_sd_flags;
	uint8_t inv_enabled;
	uint8_t inv_curr_lim_reached;

	int16_t hall_fl,
			hall_fr,
			hall_rl,
			hall_rr;

	/* General info */
	float power_mech;

	/* PDU info */
	float tdk1_current;
	float tdk2_current;
	float lv_max_cell_voltage;
	float lv_min_cell_voltage;

}P23_status;





/* Functions */
void CAN_Handler_Init(CAN_Handler *can_handler, CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxHeader,
			          CAN_TxHeaderTypeDef *TxHeader, uint8_t *RxData, uint8_t *TxData, uint32_t *canMailbox, uint32_t can_fifo);

HAL_StatusTypeDef IVT_I_Configuration(Ivt* ivt);
HAL_StatusTypeDef IVT_U1_Configuration(Ivt* ivt);
HAL_StatusTypeDef IVT_U2_Configuration(Ivt* ivt);
HAL_StatusTypeDef IVT_U3_Configuration(Ivt* ivt);
HAL_StatusTypeDef IVT_Baud_Configuration(Ivt* ivt);
void	          IVT_CAN_Rx(Ivt *ivt);

HAL_StatusTypeDef Elcon_CAN_Tx(Elcon *elcon, BMS_info *bmsInfo);   // Extended CANbus ID TX to ELCON (1Hz)
void       		  Elcon_CAN_Rx(Elcon *elcon);
HAL_StatusTypeDef Elcon_stop_charging(Elcon* elcon, BMS_info* bmsInfo);

HAL_StatusTypeDef Charger_CAN_Tx(BMS_info* bmsInfo, CAN_Handler* canHandle);

/* Main CAN_RX checks */
void TSAC_CAN_Rx(CAN_Handler *can_handler, Elcon *elcon, Ivt *ivt, BMS_info *bmsInfo);

/* CANbus functions for each message */
HAL_StatusTypeDef TSAC_Energy_CAN_Tx(BMS_info* bmsInfo, Ivt* ivt, CAN_Handler *can_handler);
HAL_StatusTypeDef Cell_Temp_CAN_Tx(BMS_info* bmsInfo, CAN_Handler *can_handler);
HAL_StatusTypeDef TSAC_Status_CAN_Tx(BMS_info* bmsInfo, Accu_info* accuInfo, Imd* imd);
HAL_StatusTypeDef Cell_Voltage_CAN_Tx(BMS_info* bmsInfo, CAN_Handler *can_handler);
HAL_StatusTypeDef Error_Msg_CAN_Tx(Accu_info *accuInfo, BMS_info* bmsInfo, CAN_Handler *can_handler);

/* Private functions called by above CAN functions */
HAL_StatusTypeDef CAN_Rx(CAN_Handler *can_handler, uint32_t canFIFO);
HAL_StatusTypeDef CAN_Tx(CAN_Handler *can_handler, uint32_t id, uint8_t dlc, uint8_t *data);
void Error_Handler_CAN(uint64_t x, TickType_t current);

/* Reads all information about P23 status: Mainly used for SD_logging */
void read_P23_messages(CAN_Handler *can_handler, P23_status *P23_status);

#endif /* INC_ACCU_CAN_FUNCTIONS_H_ */

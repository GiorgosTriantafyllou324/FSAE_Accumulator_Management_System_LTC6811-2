#include <ACCU_CAN_functions.h>


/* Constant arrays for the initialization of the IVT */
//uint8_t IVT_STOP[8] 	     = {0x34,0x00,0x01,0x00,0x00,0x00,0x00,0x00};
//uint8_t IVT_U2_CONFIG[8]   = {0x22,0x02,0x00,0x05,0x00,0x00,0x00,0x00}; //3c->60ms
//uint8_t IVT_U1_CONFIG[8]   = {0x21,0x02,0x00,0x0A,0x00,0x00,0x00,0x00}; //0A->10ms
//uint8_t IVT_I_CONFIG[8]    = {0x20,0x02,0x00,0x05,0x00,0x00,0x00,0x00}; //05->5ms
//uint8_t IVT_STORE[8] 	     = {0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//uint8_t IVT_START[8] 	     = {0x34,0x01,0x01,0x00,0x00,0x00,0x00,0x00};
//uint8_t IVT_RESTART[8] 	 = {0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//uint8_t IVT_Baud_CONFIG[8] = {0x3A,0x02,0x00,0x00,0x00,0x00,0x00,0x00};


void CAN_Handler_Init(CAN_Handler *can_handler, CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxHeader,
			          CAN_TxHeaderTypeDef *TxHeader, uint8_t *RxData, uint8_t *TxData, uint32_t *canMailbox, uint32_t can_fifo)
{
	can_handler->hcan 		= hcan;
	can_handler->RxHeader 	= RxHeader;
	can_handler->TxHeader 	= TxHeader;
	can_handler->RxData 	= RxData;
	can_handler->TxData 	= TxData;
	can_handler->canMailbox = canMailbox;
	can_handler->canFIFO	= can_fifo;
	can_handler->canRx_errors = 0x00;
	can_handler->canTx_errors = 0x00;
}



/* Receive data from TSAC bus : IVT & More */
void IVT_CAN_Rx(Ivt *ivt)
{
	switch(ivt->can_handler->RxHeader->StdId)
	{
	case IVT_I:
		ivt->current = 0.001 * (float)(ivt->can_handler->RxData[2] << 24 | ivt->can_handler->RxData[3] << 16 | ivt->can_handler->RxData[4] << 8 | ivt->can_handler->RxData[5]);
		ivt->IVT_I_Time = xTaskGetTickCount() - ivt->IVT_I_Time_Previous;
		ivt->IVT_I_Time_Previous = xTaskGetTickCount();
		break;

	case IVT_U1:
		ivt->voltage_vs = 0.001 * (float)(ivt->can_handler->RxData[2] << 24 | ivt->can_handler->RxData[3] << 16 | ivt->can_handler->RxData[4] << 8 | ivt->can_handler->RxData[5]);
		ivt->IVT_U1_Time = xTaskGetTickCount() - ivt->IVT_U1_Time_Previous;
		ivt->IVT_U1_Time_Previous = xTaskGetTickCount();
		break;

	case IVT_U2:
		ivt->voltage_U2 = 0.001 * (float)(ivt->can_handler->RxData[2] << 24 | ivt->can_handler->RxData[3] << 16 | ivt->can_handler->RxData[4] << 8 | ivt->can_handler->RxData[5]);
		ivt->IVT_U2_Time = xTaskGetTickCount() - ivt->IVT_U2_Time_Previous;
		ivt->IVT_U2_Time_Previous = xTaskGetTickCount();
		break;

	case IVT_AH:
		ivt->Ah_consumed = (float)(ivt->can_handler->RxData[5] | (ivt->can_handler->RxData[4] << 8) | (ivt->can_handler->RxData[3] << 16) | (ivt->can_handler->RxData[2] << 24)) / 3600;
		ivt->IVT_AH_Time = xTaskGetTickCount() - ivt->IVT_AH_Time_Previous;
		ivt->IVT_AH_Time_Previous = xTaskGetTickCount();
		break;

	case IVT_WH:
		ivt->Wh_consumed = (float)(ivt->can_handler->RxData[5] | (ivt->can_handler->RxData[4] << 8) | (ivt->can_handler->RxData[3] << 16) | (ivt->can_handler->RxData[2] << 24));
		ivt->IVT_WH_Time = xTaskGetTickCount() - ivt->IVT_WH_Time_Previous;
		ivt->IVT_WH_Time_Previous = xTaskGetTickCount();
		break;

	default:
		break;
	}
}


HAL_StatusTypeDef Elcon_CAN_Tx(Elcon *elcon, BMS_info *bmsInfo)
{
	if(bmsInfo->charge_flag)
	{
		elcon->target_voltage = 600;
		elcon->target_current = bmsInfo->target_charge_current;
		elcon->N_target_charge_state = 0;
	}
	else
	{
		elcon->target_current = 0;
		elcon->target_voltage = 0;
		elcon->N_target_charge_state = 1;
	}

	/* Current limiter */
	if(elcon->target_current > CHARGE_CURR_THRESHOLD)
		elcon->target_current = CHARGE_CURR_THRESHOLD;

	/* Voltage limiter */
	if(elcon->target_voltage > 600)
		elcon->target_voltage = 600;


	elcon->tx_data[0] = (uint16_t)(elcon->target_voltage * 10) >> 8;
	elcon->tx_data[1] = (uint8_t)(elcon->target_voltage * 10);
	elcon->tx_data[2] = (uint8_t)(elcon->target_current * 10) >> 8;
	elcon->tx_data[3] = (uint8_t)(elcon->target_current * 10);
	elcon->tx_data[4] = elcon->N_target_charge_state;

	elcon->can_handler->TxHeader->DLC 				 = 8;
	elcon->can_handler->TxHeader->ExtId 			 = 0x1806E5F4;
	elcon->can_handler->TxHeader->IDE 		    	 = CAN_ID_EXT;
	elcon->can_handler->TxHeader->RTR 			     = CAN_RTR_DATA;
	elcon->can_handler->TxHeader->TransmitGlobalTime = DISABLE;

	return HAL_CAN_AddTxMessage(elcon->can_handler->hcan, elcon->can_handler->TxHeader, elcon->tx_data, elcon->can_handler->canMailbox);
}



void Elcon_CAN_Rx(Elcon *elcon)
{
	elcon->connected		    = true;
	elcon->last_msg_received    = xTaskGetTickCount();
	elcon->output_voltage 		= 0.1 * (float)(elcon->can_handler->RxData[0] << 8 | elcon->can_handler->RxData[1]);
	elcon->output_current 		= 0.1 * (float)(elcon->can_handler->RxData[2] << 8 | elcon->can_handler->RxData[3]);

	elcon->hw_fail 				= (elcon->can_handler->RxData[4] >> 0) & 0x01;
	elcon->charger_over_temp 	= (elcon->can_handler->RxData[4] >> 1) & 0x01;
	elcon->ac_input_error 		= (elcon->can_handler->RxData[4] >> 2) & 0x01;
	elcon->N_charge_state 		= (elcon->can_handler->RxData[4] >> 3) & 0x01;
	elcon->can_error			= (elcon->can_handler->RxData[4] >> 4) & 0x01;

	for (uint8_t i = 0; i < 5; ++i)
		elcon->rx_data[i] = elcon->can_handler->RxData[i];
}



HAL_StatusTypeDef Elcon_stop_charging(Elcon* elcon, BMS_info* bmsInfo)
{

	bmsInfo->charge_flag = 0;
	return Elcon_CAN_Tx(elcon, bmsInfo);
}



HAL_StatusTypeDef Charger_CAN_Tx(BMS_info* bmsInfo, CAN_Handler* canHandle)
{
	uint16_t max_cell_voltage = (uint16_t)(bmsInfo->max_voltage_cell.value * 10000);
	uint16_t min_cell_voltage = (uint16_t)(bmsInfo->min_voltage_cell.value * 10000);
	uint8_t  max_cell_temp    = (uint8_t)(bmsInfo->max_temp_cell.value * 2);
	uint8_t  min_cell_temp    = (uint8_t)(bmsInfo->min_temp_cell.value * 2);


	uint8_t tx_data[6];
	tx_data[0] = max_cell_voltage >> 8;
	tx_data[1] = max_cell_voltage;
	tx_data[2] = min_cell_voltage >> 8;
	tx_data[3] = min_cell_voltage;
	tx_data[4] = max_cell_temp;
	tx_data[5] = min_cell_temp;

	canHandle->TxHeader->DLC 				= sizeof(tx_data);
	canHandle->TxHeader->StdId 			 	= 0x700;
	canHandle->TxHeader->IDE 		    	= CAN_ID_STD;
	canHandle->TxHeader->RTR 			    = CAN_RTR_DATA;
	canHandle->TxHeader->TransmitGlobalTime = DISABLE;

	return HAL_CAN_AddTxMessage(canHandle->hcan, canHandle->TxHeader, tx_data, canHandle->canMailbox);
}



void TSAC_CAN_Rx(CAN_Handler *can_handler, Elcon *elcon, Ivt *ivt, BMS_info *bmsInfo)
{
	  if((can_handler->RxHeader->IDE == CAN_ID_EXT) && (can_handler->RxHeader->ExtId == 0x18FF50E5))
	  {
		  Elcon_CAN_Rx(elcon);
	  }
	  else if((can_handler->RxHeader->IDE == CAN_ID_EXT) && (can_handler->RxHeader->ExtId == CHARGER_ID))
	  {
		  if(can_handler->RxData[0] == 1)
			  elcon->spare_button_state = 1;
		  else
			  elcon->spare_button_state = 0;
	  }
	  else if (can_handler->RxHeader->IDE == CAN_ID_STD)
		  IVT_CAN_Rx(ivt);
}


HAL_StatusTypeDef TSAC_Energy_CAN_Tx(BMS_info* bmsInfo, Ivt* ivt, CAN_Handler *can_handler)
{
	uint8_t  data[8];
	uint16_t total_voltage_vs = (uint16_t)(ivt->voltage_vs * 100);
	int16_t  accu_current     =  (int16_t)(ivt->current   * 100);
	int16_t  energy_consumed  =  (int16_t)(ivt->Wh_consumed);
	int16_t  ah_consumed      = (int16_t)(ivt->Ah_consumed * 1000);

	data[0] = total_voltage_vs >> 8;
	data[1] = total_voltage_vs;
	data[2] = accu_current >> 8;
	data[3] = accu_current;
	data[4] = energy_consumed >> 8;
	data[5] = energy_consumed;
	data[6] = ah_consumed >> 8;
	data[7] = ah_consumed;

	return CAN_Tx(can_handler, 0x301, sizeof(data), data);
}


HAL_StatusTypeDef Cell_Temp_CAN_Tx(BMS_info* bmsInfo, CAN_Handler *can_handler)
{
	uint8_t data[5];
	data[0] = (uint8_t)(bmsInfo->max_temp_cell.value  * 2); // Maximum temperature that can be measured is 90 deg C
	data[1] = (uint8_t)(bmsInfo->max_temp_cell.pos);
	data[2] = (uint8_t)(bmsInfo->min_temp_cell.value * 2);  // Maximum temperature that can be measured is 0  deg C
	data[3] = (uint8_t)(bmsInfo->min_temp_cell.pos);        // Maximum temperature that can be measured is 0  deg C
	data[4] = (uint8_t)(bmsInfo->avg_cell_temp      * 2);

	return CAN_Tx(can_handler, 0x303, sizeof(data), data);
}


HAL_StatusTypeDef TSAC_Status_CAN_Tx(BMS_info* bmsInfo, Accu_info* accuInfo, Imd* imd)
{
	uint8_t data[7];
	uint8_t max_humidity     = (uint8_t)(bmsInfo->max_humidity.value);
	uint8_t max_humidity_pos = 			 bmsInfo->max_humidity.pos;

	/* Finds greatest humidity percentage from all sensors */
	if (accuInfo->HVroom_humidity > max_humidity)
	{
		max_humidity     = accuInfo->HVroom_humidity;
		max_humidity_pos = 0;
	}

	data[0] = ((accuInfo->precharge_done  & 0x01) << 7) |
			  ((accuInfo->ams_error       & 0x01) << 6) |
			  ((accuInfo->imd_error       & 0x01) << 5) |
			  ((accuInfo->AIR_P_Supp      & 0x01) << 4) |
			  ((accuInfo->AIR_P_State     & 0x01) << 3) |
			  ((accuInfo->AIR_M_Supp      & 0x01) << 2) |
			  ((accuInfo->AIR_M_State     & 0x01) << 1) |
			  ((accuInfo->over60V_dclink  & 0x01) << 0);

	data[1] = ((accuInfo->ts_active      & 0x01) << 0) |
			  ((accuInfo->vicor_overtemp & 0x01) << 1);

	data[2] = (uint8_t)(accuInfo->dc_dc_temp * 2);
	data[3] = max_humidity;
	data[4] = max_humidity_pos;
	data[5] = imd->insulation_kOhm >> 8;
	data[6] = imd->insulation_kOhm;

	return CAN_Tx(accuInfo->can_handler, 0x304, sizeof(data), data);
}



HAL_StatusTypeDef Cell_Voltage_CAN_Tx(BMS_info* bmsInfo, CAN_Handler *can_handler)
{
	uint8_t data[8];
	uint16_t max_cell_voltage = (uint16_t)(bmsInfo->max_voltage_cell.value * 10000);
	uint16_t min_cell_voltage = (uint16_t)(bmsInfo->min_voltage_cell.value * 10000);

	data[0] = max_cell_voltage >> 8;
	data[1] = max_cell_voltage;
	data[2] = bmsInfo->max_voltage_cell.pos;
	data[3] = min_cell_voltage >> 8;
	data[4] = min_cell_voltage;
	data[5] = bmsInfo->min_voltage_cell.pos;
	data[6] = bmsInfo->min_SoC;
	data[7] = bmsInfo->max_SoC;

	return CAN_Tx(can_handler, 0x302, sizeof(data), data);
}



HAL_StatusTypeDef Error_Msg_CAN_Tx(Accu_info *accuInfo, BMS_info* bmsInfo, CAN_Handler *can_handler)
{
	uint8_t data[5];
	uint8_t position_of_error = 0;

	if (bmsInfo->state == BMS_OK && accuInfo->state == TSAC_OK)
		position_of_error = 0xFF;

	else if (bmsInfo->state == UNDERVOLTAGE)
		position_of_error = bmsInfo->min_voltage_cell.pos;

	else if (bmsInfo->state == OVERVOLTAGE)
		position_of_error = bmsInfo->max_voltage_cell.pos;

	else if (bmsInfo->state == OVERTEMP)
		position_of_error = bmsInfo->max_temp_cell.pos;

	else if (bmsInfo->state == UNDERTEMP)
		position_of_error = bmsInfo->min_temp_cell.pos;

	else if (bmsInfo->state == COMMUNICATION_ERROR)
		position_of_error = bmsInfo->comm_error_id;

	else if (bmsInfo->state == SLAVE_ERROR)
		position_of_error = bmsInfo->slave_error_id;

	data[0] = (uint8_t)(bmsInfo->state);
	data[1] = (uint8_t)(bmsInfo->last_error);
	data[2] = (uint8_t)(accuInfo->state);
	data[3] = (uint8_t)(accuInfo->last_error);
	data[4] = position_of_error;

	return CAN_Tx(can_handler, 0x300, sizeof(data), data);
}


HAL_StatusTypeDef CAN_Tx(CAN_Handler *can_handler, uint32_t id, uint8_t dlc, uint8_t *data)
{
	can_handler->TxHeader->StdId = id;
	can_handler->TxHeader->DLC  = dlc;
	can_handler->TxHeader->IDE  = CAN_ID_STD;
	can_handler->TxHeader->RTR  = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(can_handler->hcan, can_handler->TxHeader, data, can_handler->canMailbox) != HAL_OK){
		can_handler->can_error_time = xTaskGetTickCount();
		can_handler->canTx_errors += 1;
		Error_Handler_CAN(can_handler->can_error_time, can_handler->current);
		return HAL_ERROR;
	}

	return HAL_OK;
}


HAL_StatusTypeDef CAN_Rx(CAN_Handler *can_handler, uint32_t canFIFO)
{
	if (HAL_CAN_GetRxMessage(can_handler->hcan, canFIFO, can_handler->RxHeader, can_handler->RxData) != HAL_OK){
		can_handler->can_error_time = xTaskGetTickCount();
		can_handler->canRx_errors += 1;
		Error_Handler_CAN(can_handler->can_error_time, can_handler->current);
		return HAL_ERROR;
	}

	return HAL_OK;
}


void Error_Handler_CAN(uint64_t x, TickType_t current)
{
	while(current < x + 250)
		current = xTaskGetTickCount();
}


/* Logging data on primaryCAN: BrakePressure, APPS, TS_OFF indicator & more */
void read_P23_messages(CAN_Handler *can_handler, P23_status *P23_status)
{
	switch(can_handler->RxHeader->StdId)
	{
		case 0x305:
			P23_status->hall_fl = (can_handler->RxData[0] << 8) | can_handler->RxData[1];
			P23_status->hall_fr = (can_handler->RxData[2] << 8) | can_handler->RxData[3];
			P23_status->hall_rl = (can_handler->RxData[4] << 8) | can_handler->RxData[5];
			P23_status->hall_rr = (can_handler->RxData[6] << 8) | can_handler->RxData[7];
		break;

		case(0x306):
			P23_status->actual_torque    = (int16_t)((can_handler->RxData[0] << 8)|can_handler->RxData[1]);
			P23_status->requested_torque = (int16_t)((can_handler->RxData[2] << 8)|can_handler->RxData[3]);
			P23_status->motor_rpm        = (int16_t)((can_handler->RxData[4] << 8)|can_handler->RxData[5]);
			break;

		case(0x307):
			P23_status->bspd_status = (can_handler->RxData[0] >> 3) & 0x01;
			P23_status->ts_off      = (can_handler->RxData[0] >> 6) & 0x01;
			P23_status->apps1       = (can_handler->RxData[1]);
			P23_status->apps2       = (can_handler->RxData[2]);
			P23_status->brf	        =  can_handler->RxData[4];
			P23_status->brr	        =  can_handler->RxData[5];
			break;

		case(0x308):
			P23_status->rtd_done = can_handler->RxData[0] & 0x02;
			P23_status->vcu_sd_flags = can_handler->RxData[1];
			break;

		case(0x313):
			P23_status->power_limiter = can_handler->RxData[0];
			break;

		case(0x403):
			if(can_handler->RxData[0] != 0x00)
				P23_status->vcu_alive = 1;
			else
				P23_status->vcu_alive = 0;

			if(can_handler->RxData[1] != 0x00)
				P23_status->accu_alive = 1;
			else
				P23_status->accu_alive = 0;


		    P23_status->tsal_impl_air_m    = (can_handler->RxData[2] >> 0) & 0b1111;
		    P23_status->tsal_impl_air_p    = (can_handler->RxData[2] >> 4) & 0b1111;
		    P23_status->tsal_impl_accu     = (can_handler->RxData[3] >> 0) & 0b1111;
		    P23_status->tsal_impl_pc_state = (can_handler->RxData[3] >> 4) & 0b1111;

			break;

		case 0x181:
			if (can_handler->RxData[0] == 0x27)
				P23_status->vcu_flags = can_handler->RxData[1];

			else if (can_handler->RxData[0] == 0x40)
			{
				P23_status->inv_enabled          =  can_handler->RxData[1]       & 0x01;
				P23_status->inv_curr_lim_reached = (can_handler->RxData[4] >> 5) & 0x01;
			}
			break;

		case 0x311:
			P23_status->lv_max_cell_voltage = ((can_handler->RxData[0] << 8) | can_handler->RxData[1]) / 10000.0;
			P23_status->lv_min_cell_voltage = ((can_handler->RxData[2] << 8) | can_handler->RxData[3]) / 10000.0;
			P23_status->tdk1_current = ((can_handler->RxData[6] << 8) | can_handler->RxData[7]) / 1000.0;
			break;

		case 0x312:
			P23_status->tdk2_current = ((can_handler->RxData[0] << 8) | can_handler->RxData[1]) / 1000.0;
			break;

		default:
			break;
	}
}


#include <ACCU_base_lib.h>



extern TaskHandle_t ACCU_Error_Handle;

void Accu_Struct_Init(Accu_info* accuInfo, CAN_Handler* can_handler,
	  	  	  	  	  GPIO_TypeDef *IMD_ok,
					  uint16_t      IMD_ok_Pin,
					  GPIO_TypeDef *PC_Indicator,
					  uint16_t      PC_Indicator_Pin,
	  	  	  	  	  GPIO_TypeDef *VS_OVER60,
					  uint16_t      VS_OVER60_Pin,
					  GPIO_TypeDef *AIR_M_Supp_3V,
					  uint16_t      AIR_M_Supp_3V_pin,
					  GPIO_TypeDef *AIR_P_Supp_3V,
					  uint16_t      AIR_P_Supp_3V_pin,
					  GPIO_TypeDef *AIR_M_State_3V,
					  uint16_t      AIR_M_State_3V_pin,
					  GPIO_TypeDef *AIR_P_State_3V,
					  uint16_t      AIR_P_State_3V_pin,
					  GPIO_TypeDef *AIR_P_Driver,
					  uint16_t      AIR_P_Driver_pin,
					  GPIO_TypeDef *LED,
					  uint16_t      LED_pin)
{
	accuInfo->state 	 = TSAC_OK;
	accuInfo->last_error = TSAC_OK;

	accuInfo->can_handler = can_handler;

	for(uint8_t i = 0; i < 5; i++)
		accuInfo->tx_data[i] = 0x00;           // Data sent from the ACCU board to the Primary CANbus

	/* General Accumulator info */
	accuInfo->precharge_done = 0x0;
	accuInfo->ams_error  	  = 0x0;
	accuInfo->imd_error  	  = 0x0;
	accuInfo->AIR_P_Supp      = 0x0;
	accuInfo->AIR_M_Supp 	  = 0x0;
	accuInfo->AIR_P_State 	  = 0x0;
	accuInfo->AIR_M_State     = 0x0;
	accuInfo->over60V_dclink  = 0x0;
	accuInfo->vicor_overtemp  = 0x0;

	accuInfo->dc_dc_temp = 0x0;

	accuInfo->HVroom_humidity = 0x0;

	accuInfo->precharge_voltage = 0;
	accuInfo->precharge_time    = 0;
	accuInfo->AIR_M_closed_time = 0;
	accuInfo->AIR_P_State_Int   = 0;
	accuInfo->precharge_actual_state = 0;
	accuInfo->precharge_relay_error = false;

	/* Initialization of all the GPIOs */
	accuInfo->IMD_ok 			= IMD_ok;
	accuInfo->IMD_ok_Pin 		= IMD_ok_Pin;
	accuInfo->PC_Indicator 		= PC_Indicator;
	accuInfo->PC_Indicator_Pin 	= PC_Indicator_Pin;
	accuInfo->VS_OVER60V	   	= VS_OVER60;
	accuInfo->VS_OVER60V_Pin  	= VS_OVER60_Pin;
	accuInfo->AIR_M_Supp_3V  	= AIR_M_Supp_3V;
	accuInfo->AIR_P_Supp_3V  	= AIR_P_Supp_3V;
	accuInfo->AIR_M_State_3V	= AIR_M_State_3V;
	accuInfo->AIR_P_State_3V 	= AIR_P_State_3V;
	accuInfo->AIR_P_Driver   	= AIR_P_Driver;
	accuInfo->LED   		 	= LED;

	accuInfo->AIR_M_Supp_3V_pin  = AIR_M_Supp_3V_pin;
	accuInfo->AIR_P_Supp_3V_pin  = AIR_P_Supp_3V_pin;
	accuInfo->AIR_M_State_3V_pin = AIR_M_State_3V_pin;
	accuInfo->AIR_P_State_3V_pin = AIR_P_State_3V_pin;
	accuInfo->AIR_P_Driver_pin   = AIR_P_Driver_pin;
	accuInfo->LED_pin   		 = LED_pin;
}



/* Isabelle IVT information */
void IVT_Struct_Init(Ivt* ivt, CAN_Handler *can_handler)
{
	ivt->can_handler = can_handler;

	for(uint8_t i = 0; i < 8; ++i)
		ivt->rx_data[i] = 0x00;

	ivt->current 		 = 0;
	ivt->voltage_vs 	 = 0;
	ivt->Wh_consumed = 0;
	ivt->Ah_consumed     = 0;
	ivt->voltage_U2		 = 0;

	ivt->IVT_I_Time  = 0;
	ivt->IVT_I_Time_Previous = xTaskGetTickCount();
	ivt->IVT_U1_Time = 0;
	ivt->IVT_U1_Time_Previous = xTaskGetTickCount();
	ivt->IVT_U2_Time = 0;
	ivt->IVT_U2_Time_Previous = xTaskGetTickCount();
	ivt->IVT_U3_Time = 0;
	ivt->IVT_U3_Time_Previous = xTaskGetTickCount();
	ivt->IVT_AH_Time = 0;
	ivt->IVT_AH_Time_Previous = xTaskGetTickCount();
	ivt->IVT_WH_Time = 0;
	ivt->IVT_WH_Time_Previous = xTaskGetTickCount();
}



/* ELCON functionalities */
void Elcon_Struct_Init(Elcon *elcon, CAN_Handler *can_handler)
{
	elcon->can_handler 		 = can_handler;
	elcon->connected   		 = false;
	elcon->last_msg_received = 0;

	for(uint8_t i = 0; i < 5; ++i)
	{
		elcon->tx_data[i] = 0x00;       // Data sent to ELCON with CANbus (ID: 0x1806E5F4)
		elcon->rx_data[i] = 0x00;		// Data received from ELCON with CANbus (ID: 0x18FF50E5)
	}

	elcon->output_voltage 	   = 0;	    // Voltage and current that ELCON outputs
	elcon->output_current 	   = 0;

	elcon->target_current 	   = 0;	    // Voltage and current to set the charger
	elcon->target_voltage 	   = 0;

	elcon->N_target_charge_state = 1;	// 0: Charging is ENABLED    1: Charging is DISABLED

	// Status flags
	elcon->hw_fail           = 0x01;
	elcon->charger_over_temp = 0x01;
	elcon->ac_input_error    = 0x01;
	elcon->N_charge_state    = 0x01;
	elcon->can_error         = 0x01;
}



void update_imd_status(Imd* imd)
{
	if (imd->frequency <= 1)
		imd->condition = SHORT_CIRCUIT;

	else if ((imd->frequency >= 9) && (imd->frequency <= 11))
	{
		imd->condition = NORMAL;
	}
	else if ((imd->frequency >= 19) && (imd->frequency <= 21))
	{
		imd->condition = IMD_UNDERVOLTAGE;
	}
	else if ((imd->frequency >= 29) && (imd->frequency <= 31))
	{
		imd->condition = SPEED_START;

		if ((imd->dutyCycle > 4.9) && (imd->dutyCycle < 10.1))
			imd->status = SST_GOOD;

		else if ((imd->dutyCycle > 89.9) && (imd->dutyCycle < 95.1))
			imd->status = SST_BAD;

		else
			imd->status = INVALID_DC;
	}
	else if ((imd->frequency >= 39) && (imd->frequency <= 41))
	{
		imd->condition = DEVICE_ERROR;
	}
	else if ((imd->frequency >= 49) && (imd->frequency <= 51))
	{
		imd->condition = FAULT_EARTH_CONNECTION;
	}
	else
	{
		imd->condition = UNDEFINED_FREQ;
	}


	if ((imd->condition == NORMAL) || (imd->condition == IMD_UNDERVOLTAGE))
	{
		if ((imd->dutyCycle > 5) && (imd->dutyCycle < 95))
		{
			imd->insulation_kOhm = 90 * 1200 / (imd->dutyCycle - 5) - 1200;
			imd->status = DC_OK;
		}
		else
			imd->status = INVALID_DC;
	}
	else if ((imd->condition == DEVICE_ERROR) || (imd->condition == FAULT_EARTH_CONNECTION))
	{
		if ((imd->dutyCycle > 47.4) && (imd->dutyCycle < 52.6))
			imd->status = DEVICE_FAULT;
		else
			imd->status = INVALID_DC;
	}
}



/* Check for AMS Errors if they can happen and update the respective error flags */
void update_BMS_Errors(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, Ivt *ivt, Accu_info* accuInfo)
{
	/* BMS Info updating */
	update_BMS_info_struct(master, slave_array, bmsInfo);

	/* Check for what AMS Errors actually happen */
	if (bmsInfo->max_voltage_cell.value < OV_THRESHOLD)
		no_overvoltage_time = xTaskGetTickCount();

	if (bmsInfo->min_voltage_cell.value > UV_THRESHOLD)
		no_undervoltage_time = xTaskGetTickCount();

	if (bmsInfo->ov_flag_cell == 0)
		no_overvoltage_flag_time = xTaskGetTickCount();

	if (bmsInfo->uv_flag_cell == 0)
		no_undervoltage_flag_time = xTaskGetTickCount();

	if (ivt->current < CHARGE_CURR_THRESHOLD)
		no_overcurrent_charge_time = xTaskGetTickCount();

	if (ivt->current > DISCHARGE_CURR_THRESHOLD)
		no_overcurrent_discharge_time = xTaskGetTickCount();

	if (bmsInfo->max_temp_cell.value < OT_THRESHOLD)
		no_overtemperature_time = xTaskGetTickCount();

	if (bmsInfo->min_temp_cell.value > UT_THRESHOLD)
		no_undertemperature_time = xTaskGetTickCount();

	if (ivt->voltage_vs > 300)
		no_isabelle_voltage_error_time = xTaskGetTickCount();


	if (xTaskGetTickCount() - no_overvoltage_time > VOLTAGE_TIMEOUT_MS)
		bmsInfo->state = OVERVOLTAGE;
	else if (bmsInfo->state == OVERVOLTAGE)
			bmsInfo->state = BMS_OK;

	if (xTaskGetTickCount() - no_undervoltage_time > VOLTAGE_TIMEOUT_MS)
		bmsInfo->state = UNDERVOLTAGE;
	else if (bmsInfo->state == UNDERVOLTAGE)
			bmsInfo->state = BMS_OK;

	if (xTaskGetTickCount() - no_overvoltage_flag_time > VOLTAGE_TIMEOUT_MS)
		bmsInfo->state = OVERVOLTAGE;
	else if (bmsInfo->state == OVERVOLTAGE)
			bmsInfo->state = BMS_OK;

	if (xTaskGetTickCount() - no_undervoltage_flag_time > VOLTAGE_TIMEOUT_MS)
		bmsInfo->state = UNDERVOLTAGE;
	else if (bmsInfo->state == UNDERVOLTAGE)
		bmsInfo->state = BMS_OK;

	if (xTaskGetTickCount() - no_overtemperature_time > TEMP_TIMEOUT_MS)
		bmsInfo->state = OVERTEMP;
	else if (bmsInfo->state == OVERTEMP)
			bmsInfo->state = BMS_OK;

	if (xTaskGetTickCount() - no_undertemperature_time > TEMP_TIMEOUT_MS)
		bmsInfo->state = UNDERTEMP;
	else if (bmsInfo->state == UNDERTEMP)
			bmsInfo->state = BMS_OK;

	if (xTaskGetTickCount() - no_overcurrent_charge_time > CURRENT_TIMEOUT_MS)
		bmsInfo->state = OVERCURRENT_CHARGE;
	else if (bmsInfo->state == OVERCURRENT_CHARGE)
			bmsInfo->state = BMS_OK;

	if (xTaskGetTickCount() - no_overcurrent_discharge_time > CURRENT_TIMEOUT_MS)
		bmsInfo->state = OVERCURRENT_DISCHARGE;
	else if (bmsInfo->state == OVERCURRENT_DISCHARGE)
			bmsInfo->state = BMS_OK;


	if (bmsInfo->slave_error_id != 0)
		bmsInfo->state = SLAVE_ERROR;
	else if (bmsInfo->state == SLAVE_ERROR)
		bmsInfo->state = BMS_OK;

	if (bmsInfo->comm_error_id != 0)
		bmsInfo->state = COMMUNICATION_ERROR;
	else if (bmsInfo->state == COMMUNICATION_ERROR)
		bmsInfo->state = BMS_OK;


	if (xTaskGetTickCount() - ivt->IVT_I_Time_Previous > IVT_TIMEOUT_MS)
		bmsInfo->state = ISABELLE_DEAD;
	else if (bmsInfo->state == ISABELLE_DEAD)
		bmsInfo->state = BMS_OK;

	if ((xTaskGetTickCount() - no_isabelle_voltage_error_time > VOLTAGE_TIMEOUT_MS) && (accuInfo->AIR_M_State == 1) && (accuInfo->AIR_P_State == 1))
		bmsInfo->state = ISABELLE_NO_VOLTAGE;
	else if (bmsInfo->state == ISABELLE_NO_VOLTAGE)
		bmsInfo->state = BMS_OK;

	if (bmsInfo->state != BMS_OK)
		bmsInfo->last_error = bmsInfo->state;
}



/* Update TSAC Errors in case of PreCharge failure or */
void update_TSAC_Errors(EventBits_t event_bits, Accu_info* accuInfo)
{
	//uint8_t ts_active      = (event_bits & 0x01) >> 0;
	uint8_t air_m_supp     = (event_bits & 0x02) >> 1;
	uint8_t air_m_state    = (event_bits & 0x04) >> 2;
//	uint8_t air_p_supp     = (event_bits & 0x08) >> 3;
//	uint8_t air_p_state    = (event_bits & 0x10) >> 4;
	uint8_t over_60V	   = (event_bits & 0x20) >> 5;
	//uint8_t pc_done		   = (event_bits & 0x40) >> 6;
	uint8_t pc_relay_error = (event_bits & 0x80) >> 7;

	if(pc_relay_error)
	{
		accuInfo->precharge_failed = 1;
		accuInfo->state 	 = PC_RELAY_IMPLAUSIBILITY;
		accuInfo->last_error = PC_RELAY_IMPLAUSIBILITY;
	}

	if(air_m_supp && !air_m_state)
	{
		accuInfo->state		 = AIR_M_IMPLAUSIBILITY;
		accuInfo->last_error = AIR_M_IMPLAUSIBILITY;
	}

//	if(air_p_supp && !air_p_state)
//	{
//		accuInfo->state		 = AIR_P_IMPLAUSIBILITY;
//		accuInfo->last_error = AIR_P_IMPLAUSIBILITY;
//	}

	if(air_m_supp && air_m_state && (!over_60V && !pc_relay_error))
	{
		accuInfo->precharge_failed = 1;
		accuInfo->state		 = PC_CIRCUIT_ERROR;
		accuInfo->last_error = PC_CIRCUIT_ERROR;
	}

	if(accuInfo->state != TSAC_OK)
		xTaskNotify(ACCU_Error_Handle, accuInfo->state, eSetValueWithOverwrite);
}

/* Indicate AMS Error */
void ams_fault(MCU* master, Accu_info* accuInfo)
{
	/* Along with AIR+ error open AMS_Fault */
	HAL_GPIO_WritePin(master->AMS_OK, master->AMS_OKPin, GPIO_PIN_RESET);
	accuInfo->ams_error = 1;
}


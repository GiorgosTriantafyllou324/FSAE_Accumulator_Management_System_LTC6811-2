#include <JsonUsb.h>



void USB_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	char initial_message[200] = "USB_Connected!!!\n";
	CDC_Transmit_FS((uint8_t*)initial_message, strlen(initial_message));
}


// 7msec
USBD_StatusTypeDef voltages_json(LTC6811 *slave_array)
{
	USBD_StatusTypeDef usb_result;

	char  *string 			= NULL;
	cJSON *VoltagesJson 	= NULL;

	cJSON *VoltageValueJson = NULL;

	float buffer[144];
	int i = 0 ;
	for (uint8_t slave = 0; slave < SLAVES_NUM; slave++){
		for(uint8_t cell = 0; cell < CELLS_NUM; cell++){

			buffer[i]  	  = slave_array[slave].voltage[cell];
			i++;
		}
	}

	VoltagesJson = cJSON_CreateObject();
	if (VoltagesJson == NULL)
		goto end;
	VoltageValueJson = cJSON_CreateFloatArray(buffer,144);
	if (VoltageValueJson == NULL)
		goto end;
	cJSON_AddItemToObject(VoltagesJson, "Voltages", VoltageValueJson);

	string = cJSON_Print(VoltagesJson);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(VoltagesJson);

	return usb_result;
}

USBD_StatusTypeDef balancing_json(LTC6811 *slave_array)
{
	USBD_StatusTypeDef usb_result;

	char  *string 			= NULL;
	cJSON *BalancesJson 	= NULL;

	cJSON *BalanceStateJson = NULL;

	int buffer[144];
	int i = 0 ;
	for (uint8_t slave = 0; slave < SLAVES_NUM; slave++){
		for(uint8_t cell = 0; cell < CELLS_NUM; cell++){

			buffer[i]  	  = slave_array[slave].dcc[cell];
			i++;

		}
	}
	BalancesJson = cJSON_CreateObject();
	if (BalancesJson == NULL)
		goto end;
	BalanceStateJson = cJSON_CreateIntArray(buffer,144);
	if (BalanceStateJson == NULL)
		goto end;
	cJSON_AddItemToObject(BalancesJson, "Balancing", BalanceStateJson);

	string = cJSON_Print(BalancesJson);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else
	{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(BalancesJson);

	return usb_result;
}


//3msec
USBD_StatusTypeDef temperatures_json(LTC6811 *slave_array)
{
	USBD_StatusTypeDef usb_result;

	char  *string 		    	= NULL;
	cJSON *TemperaturesJson 	= NULL;
	cJSON *TemperatureValueJson = NULL;

	/* Buffer contains 144 temperature positions for the 144 cells, but only some are measured from NTCs
	 * If one is not measured, the value in the array's cell is 0xFF                                    */
	float buffer[144];
	uint8_t i = 0;
	for (uint8_t slave = 0; slave < SLAVES_NUM; slave++){
		for(uint8_t ntc = 0; ntc < NTCS_NUM; ntc++)
		{
			if (ntc_to_cell_position[slave][ntc] == 0xFF)
			{
				buffer[i] = 0xFF;
				//i++;
				continue;
			}

			while (i < ntc_to_cell_position[slave][ntc] - 1)
			{
				buffer[i] = 0xFF;  // No NTC measuring that specific cell
				i++;
			}
			buffer[i] = slave_array[slave].temp[ntc];
			i++;

		}
	}

	TemperaturesJson = cJSON_CreateObject();
	if (TemperaturesJson == NULL)
		goto end;
	TemperatureValueJson = cJSON_CreateFloatArray(buffer,144);
	if (TemperatureValueJson == NULL)
		goto end;
	cJSON_AddItemToObject(TemperaturesJson, "Temperatures", TemperatureValueJson);

	string = cJSON_Print(TemperaturesJson);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(TemperaturesJson);

	return usb_result;
}

//1msec
USBD_StatusTypeDef humidities_json(LTC6811 *slave_array)
{
	USBD_StatusTypeDef usb_result;

	char  *string 			= NULL;
	cJSON *VoltagesJson 	= NULL;

	cJSON *VoltageValueJson = NULL;

	float buffer[SLAVES_NUM];
	int i = 0;
	for (uint8_t slave = 0; slave < SLAVES_NUM; slave++)
	{
		buffer[i] = slave_array[slave].humidity;
		i++;
	}

	VoltagesJson = cJSON_CreateObject();
	if (VoltagesJson == NULL)
		goto end;
	VoltageValueJson = cJSON_CreateFloatArray(buffer, SLAVES_NUM);
	if (VoltageValueJson == NULL)
		goto end;
	cJSON_AddItemToObject(VoltagesJson, "Humidities", VoltageValueJson);

	string = cJSON_Print(VoltagesJson);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(VoltagesJson);

	return usb_result;
}



USBD_StatusTypeDef PECerrors_json(LTC6811 *slave_array)
{
	USBD_StatusTypeDef usb_result;

	char  *string 			= NULL;
	cJSON *VoltagesJson 	= NULL;

	cJSON *VoltageValueJson = NULL;

	float buffer[SLAVES_NUM];
	int i = 0;
	for (uint8_t slave = 0; slave < SLAVES_NUM; slave++)
	{
		buffer[i] = slave_array[slave].pec_errors;
		i++;
	}

	VoltagesJson = cJSON_CreateObject();
	if (VoltagesJson == NULL)
		goto end;
	VoltageValueJson = cJSON_CreateFloatArray(buffer, SLAVES_NUM);
	if (VoltageValueJson == NULL)
		goto end;
	cJSON_AddItemToObject(VoltagesJson, "PEC_Errors", VoltageValueJson);

	string = cJSON_Print(VoltagesJson);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(VoltagesJson);

	return usb_result;
}



USBD_StatusTypeDef Accu_json(Accu_info *accuInfo)
{
	USBD_StatusTypeDef usb_result;

	char *string = NULL;
	char AccuNumber[20];
	cJSON *AccuInfoJson = NULL;

	cJSON *AccuValueJson=NULL;
	cJSON *jsonMessage = cJSON_CreateObject();
	if (jsonMessage == NULL)
		goto end;

	AccuInfoJson = cJSON_CreateObject();
	if (AccuInfoJson == NULL)
		goto end;

	char AccuValue[10];

	// AMS_ERROR
	sprintf(AccuValue, "%u", accuInfo->ams_error);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "Ams_Error");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// IMD_ERROR
	sprintf(AccuValue, "%u", accuInfo->imd_error);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "Imd_Error");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// AIR_P_Supp
	sprintf(AccuValue, "%u", accuInfo->AIR_P_Supp);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "AIR_P_Supp");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// AIR_M_Supp
	sprintf(AccuValue, "%u", accuInfo->AIR_M_Supp);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "AIR_M_Supp");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// AIR_P_State
	sprintf(AccuValue, "%u", accuInfo->AIR_P_State);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "AIR_P_State");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// AIR_M_State
	sprintf(AccuValue, "%u", accuInfo->AIR_M_State);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "AIR_M_State");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// over60_dclink
	sprintf(AccuValue, "%u", accuInfo->over60V_dclink);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "over60_dclink");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// dc_dc_temp
	sprintf(AccuValue, "%0.4f", accuInfo->dc_dc_temp);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "dc_dc_temp");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// HVroom_humidity
	sprintf(AccuValue, "%u", accuInfo->HVroom_humidity);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "HVroom_humidity");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// precharge_voltage
	sprintf(AccuValue, "%0.4f", accuInfo->precharge_voltage);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "precharge_voltage");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	// AIR_P_State_Int
	sprintf(AccuValue, "%u", accuInfo->AIR_P_State_Int);
	AccuValueJson = cJSON_CreateString(AccuValue);
	if (AccuInfoJson == NULL)
		goto end;

	sprintf(AccuNumber, "AIR_P_State_Int");
	cJSON_AddItemToObject(AccuInfoJson, AccuNumber, AccuValueJson);
	memset(AccuValue, 0, sizeof(AccuValue));

	cJSON_AddItemToObject(jsonMessage, "AccumulatorInfo", AccuInfoJson);
	string = cJSON_Print(jsonMessage);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(jsonMessage);

	return usb_result;
}


// Isabelle
USBD_StatusTypeDef Ivt_json(Ivt *ivt)
{
	USBD_StatusTypeDef usb_result;

	char   IvtNumber[20];
	char  *string 		= NULL;
	cJSON *IvtInfoJson 	= NULL;

	cJSON *IvtValueJson = NULL;
	cJSON *jsonMessage = cJSON_CreateObject();
	if (jsonMessage == NULL)
		goto end;

	IvtInfoJson = cJSON_CreateObject();
	if (IvtInfoJson == NULL)
		goto end;

	char IvtValue[10];

	// Vehicle side voltage
	sprintf(IvtValue, "%0.1f ", ivt->voltage_vs);
	IvtValueJson = cJSON_CreateString(IvtValue);
	if (IvtInfoJson == NULL)
		goto end;

	sprintf(IvtNumber, "V_Side_Voltage");
	cJSON_AddItemToObject(IvtInfoJson, IvtNumber, IvtValueJson);
	memset(IvtValue, 0, sizeof(IvtValue));

	// Total accumulator current
	sprintf(IvtValue, "%0.2f", ivt->current);
	IvtValueJson = cJSON_CreateString(IvtValue);
	if (IvtInfoJson == NULL)
		goto end;

	sprintf(IvtNumber, "Current");
	cJSON_AddItemToObject(IvtInfoJson, IvtNumber, IvtValueJson);
	memset(IvtValue, 0, sizeof(IvtValue));

	// Ampere hours consumed
	sprintf(IvtValue, "%0.3f", ivt->Ah_consumed);
	IvtValueJson = cJSON_CreateString(IvtValue);
	if (IvtInfoJson == NULL)
		goto end;

	sprintf(IvtNumber, "Ah_consumed");
	cJSON_AddItemToObject(IvtInfoJson, IvtNumber, IvtValueJson);
	memset(IvtValue, 0, sizeof(IvtValue));

	// Total TSAC energy consumed since the last LVMS power cycling
	sprintf(IvtValue, "%u", ivt->Wh_consumed);
	IvtValueJson = cJSON_CreateString(IvtValue);
	if (IvtInfoJson == NULL)
		goto end;

	sprintf(IvtNumber, "Energy Consumed");
	cJSON_AddItemToObject(IvtInfoJson, IvtNumber, IvtValueJson);
	memset(IvtValue, 0, sizeof(IvtValue));

	cJSON_AddItemToObject(jsonMessage, "Isabelle Info", IvtInfoJson);
	string = cJSON_Print(jsonMessage);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(jsonMessage);

	return usb_result;
}

// Elcon charger
USBD_StatusTypeDef Elcon_json(Elcon *elcon)
{
	USBD_StatusTypeDef usb_result;

	char   ElconNumber[20];
	char  *string 			= NULL;
	cJSON *ElconInfoJson 	= NULL;
	cJSON *ElconValueJson   = NULL;
	cJSON *jsonMessage = cJSON_CreateObject();
	if (jsonMessage == NULL)
		goto end;

	ElconInfoJson = cJSON_CreateObject();
	if (ElconInfoJson == NULL)
		goto end;

	cJSON_AddItemToObject(jsonMessage, "Elcon Info", ElconInfoJson);

	char ElconValue[10];

	// Target voltage set from BMS
	sprintf(ElconValue, "%0.1f ", elcon->target_voltage);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Target_Voltage");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// Actual output Voltage
	sprintf(ElconValue, "%0.1f", elcon->output_voltage);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Output_Voltage");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// Target Current set from BMS
	sprintf(ElconValue, "%0.1f", elcon->target_current);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Target_Current");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// Actual output Current
	sprintf(ElconValue, "%0.1f", elcon->output_current);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Output_Current");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// Elcon connect status
	sprintf(ElconValue, "%u", elcon->connected);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Elcon_connected");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// AC input status
	sprintf(ElconValue, "%u", elcon->ac_input_error);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Elcon_AC_input_OK");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// CAN BUS error indicator
	sprintf(ElconValue, "%u", elcon->can_error);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "CANBUS_Error");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// Elcon target charge state
	sprintf(ElconValue, "%u", elcon->N_target_charge_state);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Target_charge_state");  // Inverted Logic!!
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// Elcon actual charge state
	sprintf(ElconValue, "%u", elcon->N_charge_state);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Elcon_charge_status");  // Inverted Logic!!
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));

	// Elcon over temperature flag
	sprintf(ElconValue, "%u", elcon->charger_over_temp);
	ElconValueJson = cJSON_CreateString(ElconValue);
	if (ElconInfoJson == NULL)
		goto end;

	sprintf(ElconNumber, "Elcon_overtemp");
	cJSON_AddItemToObject(ElconInfoJson, ElconNumber, ElconValueJson);
	memset(ElconValue, 0, sizeof(ElconValue));


	string = cJSON_Print(jsonMessage);
	if (string == NULL)
	{
		cJSON_free(string);
	}
	else{
		usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
		cJSON_free(string);
	}

	end:
	cJSON_Delete(jsonMessage);

	return usb_result;
}

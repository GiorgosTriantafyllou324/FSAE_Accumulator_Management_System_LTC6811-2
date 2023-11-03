/*
 * SD Card Functions, using FreeRTOS
 */

#include <SD_Card.h>
#include <usb_device.h>
#include <usbd_cdc_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LTC6811    *sd_slave_array;
Accu_info  *sd_accuInfo;
BMS_info   *sd_bmsInfo;
Ivt 	   *sd_ivt;
Elcon      *sd_elcon;
Imd        *sd_imd;
P23_status *sd_p23_status;

// Tasos Variables
FILINFO fileInfo;
int maxNumber = -1;
int filenumber = 420;
char line[100];



FRESULT SD_Card_init(SD_Card* sdCard, LTC6811* slave_array, Accu_info* accuInfo, BMS_info* bmsInfo, Ivt* ivt, Elcon* elcon, Imd* imd, P23_status* p23status)
{
	sd_slave_array = slave_array;
	sd_accuInfo    = accuInfo;
	sd_ivt         = ivt;
	sd_elcon       = elcon;
	sd_imd         = imd;
	sd_p23_status  = p23status;
	sd_bmsInfo     = bmsInfo;

	sdCard->read_flag      = 0;
	sdCard->unsaved_writes = 0;
	sdCard->message_num    = 1;

	sdCard->fresult = f_mount(&(sdCard->fs), "/", 1);
	if (sdCard->fresult != FR_OK)
	{
		sdCard->mounted = false;
		return sdCard->fresult;
	}

	sdCard->fresult = f_stat("number.csv", &fileInfo);
	if (sdCard->fresult == FR_OK) {
		sdCard->fresult = f_open(&(sdCard->file), "number.csv", FA_READ | FA_WRITE);
		if (sdCard->fresult != FR_OK)
			return sdCard->fresult;
		if (f_gets(line, sizeof(line), &(sdCard->file)) != NULL) {
			// Rewind the file pointer to the beginning
			f_lseek(&(sdCard->file), 0);
			if (sscanf(line, "%d", &filenumber) == 1) {
				filenumber++;
			} else {
				filenumber = 1;
			}
			snprintf(line, sizeof(line), "%d\n", filenumber);
			sdCard->fresult = f_puts(line, &(sdCard->file));
		}
		sdCard->fresult = f_close(&(sdCard->file));
	}
	else{
		filenumber=0;
		sdCard->fresult = f_open(&(sdCard->file), "number.csv", FA_OPEN_APPEND | FA_WRITE);
		snprintf(line, sizeof(line), "%d\n", filenumber);
		sdCard->fresult = f_puts(line, &(sdCard->file));
		sdCard->fresult = f_close(&(sdCard->file));
	}
	char allFileName[20]; // Adjust the size based on your filename requirements
	snprintf(allFileName, sizeof(allFileName), "all%d.csv", filenumber);
	sdCard->fresult = f_open(&(sdCard->file), allFileName, FA_OPEN_APPEND | FA_WRITE);
	if (sdCard->fresult != FR_OK)
		return sdCard->fresult;
	sdCard->mounted = true;



	char header[600];

	sprintf(header, "Message_num, HAL_GetTick()");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	for(uint8_t cell = 0; cell < 144; cell++){
		sprintf(header, ", V%d", cell);
		sdCard->fresult = f_puts(header, &(sdCard->file));
	}

	//BMS Temperatures
	sprintf(header, " , T0");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	for(uint8_t ntc = 1; ntc < 80; ntc++){
		sprintf(header, ", T%d", ntc);
		sdCard->fresult = f_puts(header, &(sdCard->file));
	}

	//BMS Humidities
	sprintf(header, " , H0");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	for(uint8_t rh = 1; rh < 16; rh++){
		sprintf(header, ", H%d", rh);
		sdCard->fresult = f_puts(header, &(sdCard->file));
	}

	//BMS PEC Errors
	sprintf(header, " ,PEC0");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	for(uint8_t id = 1; id < 16; id++){
		sprintf(header, ", PEC%d", id);
		sdCard->fresult = f_puts(header, &(sdCard->file));
	}

	// AccuInfo
	sprintf(header, " ,ams_error, imd_error, AIR_P_Supp, AIR_M_Supp, AIR_P_State,  AIR_M_State, over60_dclink, dc_dc_temp, HVroom_humidity, HVroom_temperature, "
			        "precharge_voltage, AIR_P_State_Int, precharge_actual_state, precharge_relay_error, air_stuck, ts_active, precharge_done, "
			        "precharge_failed");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// IVT
	sprintf(header, " ,current, voltage_vs, Ah_consumed, Wh_consumed");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// Elcon
	sprintf(header, " ,elcon_connected, target_voltage, output_voltage, target_current, output_current, "
			        "target_charge_state, charge_state, hw_fail, charger_over_temp, ac_input_ok, charger_can_error");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// IMD
	sprintf(header, " ,IMD_condition, IMD_status, insulation_kOhm");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// BMS Info
	sprintf(header, " ,min_SoC, max_SoC, accu_voltage");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// P23 Status
	sprintf(header, " ,actual_torque, requested_torque, motor_rpm, bspd_status, ts_off, rtd_done, vcu_alive, "
					"accu_alive, hall_fl, hall_fr, hall_rl, hall_rr, "
			        "power_limiter, apps1, apps2, Brake_pressure_front, Brake_pressure_rear, tsal_impl_air_m,"
			        "tsal_impl_air_p, tsal_impl_acccu, tsal_impl_pc_state, vcu_flags, vcu_sd_flags, inv_enabled,"
			        "inv_curr_lim_reached, tdk1_current, tdk2_current, lv_max_cell_voltage, lv_min_cell_voltage \n");
	sdCard->fresult = f_puts(header, &(sdCard->file));

	/* Auto Save SD Card for Headers */
	sdCard->fresult = f_sync(&(sdCard->file));

	return FR_OK;
}



void SD_card_write(SD_Card* sdCard)
{
	char header[500];

	sprintf(header, "%" PRIu32 ", ", sdCard->message_num);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	sprintf(header, "%" PRIu32 ", ", xTaskGetTickCount());
	sdCard->fresult = f_puts(header, &(sdCard->file));

	//BMS Voltages
	for (uint8_t slave = 0; slave < 16; slave++){
		for(uint8_t cell = 0; cell < 9; cell++){
			sprintf(header, "%0.4f, ", sd_slave_array[slave].voltage[cell]);
			sdCard->fresult = f_puts(header, &(sdCard->file));
		}
	}

	//BMS Temperatures
	for (uint8_t slave = 0; slave < 16; slave++){
		for(uint8_t ntc = 0; ntc < 5; ntc++){
			sprintf(header, "%0.4f, ", sd_slave_array[slave].temp[ntc]);
			sdCard->fresult = f_puts(header, &(sdCard->file));
		}
	}

	//BMS Humidities
	for (uint8_t rh = 0; rh < 16; rh++)
	{
		sprintf(header, "%0.4f, ", sd_slave_array[rh].humidity);
		sdCard->fresult = f_puts(header, &(sdCard->file));
	}

	//BMS PEC Errors
	for (uint8_t id = 0; id < SLAVES_NUM; id++)
	{
		sprintf(header, "%" PRIu16 ", ", sd_slave_array[id].pec_errors);
		sdCard->fresult = f_puts(header, &(sdCard->file));
	}

	// ACCU Info
	sprintf(header, "%u, %u, %u, %u, %u, %u, %u, %0.2f, %u, %u, %0.2f, %u, %u, %u, %u, %u, %u, %u, ", sd_accuInfo->ams_error,
			         sd_accuInfo->imd_error, sd_accuInfo->AIR_P_Supp, sd_accuInfo->AIR_M_Supp, sd_accuInfo->AIR_P_State, sd_accuInfo->AIR_M_State,
					 sd_accuInfo->over60V_dclink, sd_accuInfo->dc_dc_temp, sd_accuInfo->HVroom_humidity, sd_accuInfo->HVroom_temperature,
					 sd_accuInfo->precharge_voltage, sd_accuInfo->AIR_P_State_Int, sd_accuInfo->precharge_actual_state, sd_accuInfo->precharge_relay_error, sd_accuInfo->air_stuck, sd_accuInfo->ts_active, sd_accuInfo->precharge_done, sd_accuInfo->precharge_failed);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	//IVT
	sprintf(header, "%0.2f, %0.2f, %0.4f, ", sd_ivt->current, sd_ivt->voltage_vs, sd_ivt->Ah_consumed);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	sprintf(header, "%d, ", sd_ivt->Wh_consumed);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// ELCON
	sprintf(header,"%u, %0.1f, %0.1f, %0.2f, %0.2f, %u, %u, %u, %u, %u, %u, ",
							sd_elcon->connected,
							sd_elcon->target_voltage, sd_elcon->output_voltage,
							sd_elcon->target_current, sd_elcon->output_current,
							sd_elcon->N_target_charge_state, sd_elcon->N_charge_state,
							sd_elcon->hw_fail, sd_elcon->charger_over_temp,
			                sd_elcon->ac_input_error, sd_elcon->can_error);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// IMD
	sprintf(header,"%u, %u, ", sd_imd->condition, sd_imd->status);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	sprintf(header, "%" PRIu16 ", ", sd_imd->insulation_kOhm);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// BMS Info
	sprintf(header,"%u, %u, %0.3f", sd_bmsInfo->min_SoC, sd_bmsInfo->max_SoC, sd_bmsInfo->accu_voltage);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	sprintf(header, "%" PRIu16 ", ", sd_imd->insulation_kOhm);
	sdCard->fresult = f_puts(header, &(sdCard->file));

	// P23 status
	sprintf(header, "%d, %d, %d, %u, %u, %u, %u, %u, %d, %d, %d, %d, %u, %u, %u, %u, %u, %u , %u, %u, %u, %u, %u, %u, %u, %0.4f, %0.4f, %0.4f, %0.4f \n ",
							sd_p23_status->actual_torque,   sd_p23_status->requested_torque,
							sd_p23_status->motor_rpm,       sd_p23_status->bspd_status,
							sd_p23_status->ts_off,          sd_p23_status->rtd_done,
							sd_p23_status->vcu_alive,       sd_p23_status->accu_alive,
							sd_p23_status->hall_fl,			sd_p23_status->hall_fr,
							sd_p23_status->hall_rl,			sd_p23_status->hall_rr,
							sd_p23_status->power_limiter,
							sd_p23_status->apps1,           sd_p23_status->apps2,
							sd_p23_status->brf,             sd_p23_status->brr,
							sd_p23_status->tsal_impl_air_m, sd_p23_status->tsal_impl_air_p,
							sd_p23_status->tsal_impl_accu,  sd_p23_status->tsal_impl_pc_state,
							sd_p23_status->vcu_flags,       sd_p23_status->vcu_sd_flags,
							sd_p23_status->inv_enabled,     sd_p23_status->inv_curr_lim_reached,
							sd_p23_status->tdk1_current,    sd_p23_status->tdk2_current,
							sd_p23_status->lv_max_cell_voltage, sd_p23_status->lv_min_cell_voltage);

	sdCard->fresult = f_puts(header, &(sdCard->file));

	sdCard->unsaved_writes++;
	sdCard->message_num++;
	if (sdCard->unsaved_writes >= AUTOSAVE_CYCLES)
	{
		sdCard->fresult = f_sync(&(sdCard->file));
		sdCard->unsaved_writes = 0;
	}

}


void SD_card_read(SD_Card* sdCard)
{
	sdCard->fresult = f_close(&sdCard->file);  /* Saves the data stored in the SD so far and closes */
	sdCard->fresult = f_mount(NULL, "", 1);    /* Unmounts the SD to initialize it again */


	sdCard->fresult = f_mount(&(sdCard->fs), "/", 1);
	if (sdCard->fresult != FR_OK)
	{
		sdCard->mounted = false;
	}
	sdCard->fresult = f_open(&(sdCard->file), "all.csv", FA_READ);
	if (sdCard->fresult != FR_OK)
	{
		sdCard->mounted = false;
	}


	char     buffer[200];
	UINT     bytesRead = 0;
	uint32_t read_counter = 1;
	do
	{
		sdCard->fresult = f_read(&sdCard->file, buffer, sizeof(buffer), &bytesRead);
		sdCard->fresult = f_lseek(&sdCard->file, sizeof(buffer) * read_counter);
		read_counter++;

		CDC_Transmit_FS ((uint8_t*)buffer, sizeof(buffer));
		osDelay(5);      /* Without delay, USB messages are lost */
	}
	while (bytesRead == sizeof(buffer));

	sdCard->fresult = f_close(&sdCard->file);
	sdCard->fresult = f_mount(NULL, "", 1);
	sdCard->fresult = SD_Card_init(sdCard, sd_slave_array, sd_accuInfo, sd_bmsInfo, sd_ivt,sd_elcon, sd_imd, sd_p23_status);
}


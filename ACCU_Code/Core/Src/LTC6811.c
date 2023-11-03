#include <LTC6811.h>
#include "task.h"

uint32_t test;

BMSstatus_t BMS_Init(MCU *master, LTC6811 *slave_array, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *CS_BMS,  GPIO_TypeDef *AMS_OK,
			      uint16_t CS_pin, uint64_t AMS_OK_pin, BMS_info *bmsInfo, uint8_t refon, uint8_t adcopt)
{
    BMSstatus_t result;

	init_PEC15_Table();
	bms_info_struct_init(bmsInfo);
	mcu_struct_init(master, spiHandle, CS_BMS, AMS_OK, CS_pin , AMS_OK_pin);
	slave_array_init(slave_array);

	if (!ntc_rh_init(slave_array, NTC_LUT, NTC_voltage, V_REF, NTC_CONST_RESISTOR_KOHM, NTC_LUT_LENGTH))
		return NTC_LUT_ERROR;

	uint8_t gpio[5] = {1, 1, 1, 1, 1};                  // No pull-down resistors are activated in the GPIOs
	uint8_t no_dcc[9]  = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 	// No cell is discharging

	write_cfgr(master, slave_array, BC, 0, gpio, refon, adcopt, UV_THRESHOLD, OV_THRESHOLD, no_dcc, 0);  // DYNATH ALVANIA

	find_ids_in_bus(master, slave_array);
	for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
		if (master->ids_in_bus[id] == false)
			return COMM_ERROR;


	if (DIAGNOSE_ENABLED)
	{
		for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
		{
			write_cfgr(master, slave_array, BC, 0, gpio, refon, adcopt, UV_THRESHOLD, OV_THRESHOLD, no_dcc, 0);  // DYNATH ALVANIA

			result = diagnose(master, slave_array, bmsInfo, id, 3);
			if (result != OK)
				return result;
		}
	}

	/* Set the UV/OV flag thresholds */
	write_cfgr(master, slave_array, BC, 0, gpio, refon, adcopt, UV_THRESHOLD, OV_THRESHOLD, no_dcc, 0);  // DYNATH ALVANIA
	result = safe_write_cfgr(master, slave_array, BC, 0, gpio, refon, adcopt, UV_THRESHOLD, OV_THRESHOLD, no_dcc, 0, 3);
	if (result != OK)
		return result;

	return OK;
}


void bms_info_struct_init(BMS_info *bms)
{
	bms->mode = STDBY;

	bms->uv_threshold = UV_THRESHOLD;
	bms->ov_threshold = OV_THRESHOLD;

	bms->ut_threshold = UT_THRESHOLD;
	bms->ot_threshold = OT_THRESHOLD;

	// CHANGE VALUES
	bms->target_charge_current = 0;
	bms->charge_flag = 0;

	bms->max_humidity.value = 0;
	bms->max_humidity.pos   = 0;

	bms->state = BMS_OK;
	bms->last_error = BMS_OK;
	bms->BalInfo.state = off;
	bms->BalInfo.balancing_enabled = 0;
}



void mcu_struct_init(MCU *master, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *CS_BMS,  GPIO_TypeDef *AMS_OK,
					uint16_t CS_pin, uint64_t AMS_OK_pin)
{
	master->spiHandle 	= spiHandle;
	master->csBMS		= CS_BMS;
	master->AMS_OK		= AMS_OK;
	master->csBMSPin	= CS_pin;
	master->AMS_OKPin	= AMS_OK_pin;

	for(uint8_t i = 0; i < MAX_ATTEMPTS; ++i){
		master->read_status[i] = 0;
	}

	for(uint8_t i = 0; i < 6; ++i){
		master->write_buf[i] = 0;
	}

	for(uint8_t i = 0; i < 16; ++i){
		master->ids_in_bus[i] = false;
	}
	master->command_buf = 0;

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);
}


void slave_array_init(LTC6811 *slave_array)
{
	for (uint8_t id = 0; id < SLAVES_NUM; ++id)
		slave_struct_init(&slave_array[id]);
}

/* Initializes a single LTC6811 struct */
void slave_struct_init(LTC6811 *slave)
{
	/* Debugging Variables */

	for (uint8_t i = 0; i < 6; ++i)
		slave->tx_buf[i] = 0;

	for (uint8_t i = 0; i < 2; ++i)
		slave->pec_status[i] = 0;

	slave->pec_errors  		 = 0;
	slave->voltage_deviation = 0.0;
	slave->diagnose_state    = NO_DIAGNOSE_TEST;

	slave->last_valid_msg = xTaskGetTickCount();

	/* REGISTERS */
	for(uint8_t i = 0; i < 5; ++i) {
		slave->gpio_pulldown[i] = 1;
		slave->gpio_level[i]    = 0;
	}

	slave->refon 	= 0;
	slave->dten 	= 1;
	slave->adcopt   = 0;
	slave->min_v 	= 0.0;
	slave->max_v 	= 0.0;
	slave->dcto     = 0;

	for(uint8_t i = 0; i < CELLS_NUM; ++i) {
		slave->dcc[i] 	    = 1;
		slave->voltage[i]   = 0.0;
		slave->uv_flag[i]   = 1;
		slave->ov_flag[i]   = 1;
		slave->sctrl[i]	    = 0;
		slave->pwm[i]	    = 0;
		slave->open_wire[i] = true;
	}
	slave->open_wire[CELLS_NUM] = true;

	for(uint8_t i = 0; i < NTCS_NUM; ++i) {
		slave->gpio_voltage[i] = 0.0;
		slave->temp[i] = 0.0;
	}

	slave->vref2    = 0.0;
	slave->humidity = 0.0;
	slave->sc 	    = 0.0;
	slave->itmp     = 0.0;
	slave->va 	    = 0.0;
	slave->vd 	    = 0.0;

	slave->muxfail = 1;
	slave->thsd    = 1;

}


/* ------------------------------------- PEC --------------------------------------------------- */
uint16_t pec15Table[256];
uint16_t CRC15_POLY = 0x4599;
/* ------------- PRIVATE FUNCTIONS --------------- */
void init_PEC15_Table(){
	uint16_t remainder;
	for (int i = 0; i < 256; i++){
		remainder = i << 7;
		for (int bit = 8; bit > 0; --bit){
			if (remainder & 0x4000){
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC15_POLY);
			}
			else{
				remainder = ((remainder << 1));
			}
		}
		pec15Table[i] = remainder & 0xFFFF;
	}
}

uint16_t pec15(const uint8_t *data , const uint8_t len){
	uint16_t remainder,address;
	remainder = 16; //PEC seed
	for (uint8_t i = 0; i < len; i++){
		address = ((remainder >> 7) ^ data[i]) & 0xFF;	// calculate PEC table address
		remainder = (remainder << 8 ) ^ pec15Table[address];
	}
	return (remainder * 2);	// The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}


/*****************************************************************************
*  @Description	  Updates the ids_in_bus array with true for every id present
*  				  and false for every id not found
******************************************************************************/
void find_ids_in_bus(MCU *master, LTC6811 *slave_array)
{
	for (uint8_t id = 0; id < 16; ++id) {
		if (read(master, slave_array, id, RDCFGA, 3) == OK)
			master->ids_in_bus[id] = true;
		else
			master->ids_in_bus[id] = false;
	}
}


/*****************************************************************************
*  @Description	  Updates all fields of the BMS_info struct except the state.
*                 The state is updated from update_BMS_errors because it takes
*                 into account the timeouts
******************************************************************************/
void update_BMS_info_struct(MCU* master, LTC6811 *slave_array, BMS_info *bmsInfo)
{
	/* Updates voltage parameters */
	float accu_voltage = 0;

	for (uint8_t id = 0; id < SLAVES_NUM; ++id)
		for (uint8_t cell = 0; cell < CELLS_NUM; ++cell)
			accu_voltage += slave_array[id].voltage[cell];


	bmsInfo->accu_voltage = accu_voltage;
	bmsInfo->avg_cell_voltage = accu_voltage / (SLAVES_NUM * CELLS_NUM);

	find_min_max_v(bmsInfo, slave_array);

	/* Updates the OV/UV flags on bmsInfo struct */
	update_uv_ov_flags(bmsInfo, slave_array);

	/* Updates temperature parameters */
	float cell_temp_sum = 0;
	uint8_t total_ntcs_num = 0;
	for (uint8_t id = 0; id < SLAVES_NUM; ++id)
		for (uint8_t ntc = 0; ntc < NTCS_NUM; ++ntc)
		{
			if (ntc_to_cell_position[id][ntc] == 0xFF)  // Guarantees that the GPIO is taken from an NTC and not an RH sensor
				continue;

			cell_temp_sum += slave_array[id].temp[ntc];
			total_ntcs_num++;
		}
	bmsInfo->avg_cell_temp = cell_temp_sum / total_ntcs_num;

	find_min_max_temp(bmsInfo, slave_array);


	/* Updates humidity parameter */
	find_max_humidity(bmsInfo, slave_array);


	/* Checks if there are hardware errors (malfunctioning slave / open wires) */
	bool slave_error_present = false;
	float delta = 0.005;
	for (ID_t id = ID_0; id < SLAVES_NUM; id++)
	{
		if ((slave_array[id].diagnose_state != DIAGNOSE_OK) && (slave_array[id].diagnose_state != NO_DIAGNOSE_TEST))
		{
			bmsInfo->slave_error_id = id + 1;   // slave_error_num takes values from 1 to SLAVES_NUM
			slave_error_present = true;
			break;
		}

		/* Checks if the parameters stored in the configuration register of each slave are correct */
		if (slave_array[id].min_v < UV_THRESHOLD - delta)
		{
			bmsInfo->slave_error_id = id + 1;   // slave_error_num takes values from 1 to SLAVES_NUM
			slave_error_present = true;
		}
		if (slave_array[id].max_v > OV_THRESHOLD + delta)
		{
			bmsInfo->slave_error_id = id + 1;   // slave_error_num takes values from 1 to SLAVES_NUM
			slave_error_present = true;
		}
	}

	if (!slave_error_present)
		bmsInfo->slave_error_id = 0;   // No slave error in the bus


	/* Checks if there are communication errors (dead slave) */
	if (update_comm_errors(master, slave_array, bmsInfo) != OK)  // If COMM errors exist
	{
		for (ID_t id = 0; id < SLAVES_NUM; ++id)
		{
			if (master->no_comm_time[id] > COMM_TIMEOUT_MS)
			{
				bmsInfo->comm_error_id = id + 1;   // comm_error_num takes values from 1 to SLAVES_NUM
				break;
			}
		}
	}
	else
		bmsInfo->comm_error_id = 0;	  // Clear Communication error

}


BMSstatus_t update_comm_errors(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo)
{
	bool comm_error = false;

	for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
	{
		master->no_comm_time[id] = xTaskGetTickCount() - slave_array[id].last_valid_msg;
		if (master->no_comm_time[id] > COMM_TIMEOUT_MS)
		{
			comm_error = true;
			//ams_fault(bmsInfo, master, slave_array);
		}
	}

	if (comm_error)
		return COMM_ERROR;

	return OK;
}


/*--------------------------------- BALANCING FUNCTIONS -------------------------------------------------------*/


/*****************************************************************************
*  @Description	  Finds which cells have to start - stop balancing and starts -
*  				  stops the discharging
******************************************************************************/
BMSstatus_t update_balancing_cells(BMS_info *bms, MCU *master, LTC6811 *slave_array, const uint8_t attempts)
{

	/* OverTemperature Handling */
	// UNNECESSARY SINCE update_balancing_cells IS CALLED EVERY 10s
	if (bms->max_temp_cell.value > bms->ot_threshold)
	{
		bms->charge_flag = false;
		stop_balancing(bms, master, slave_array, attempts);
	}

	if (bms->state == OVERTEMP)
	{
		if (bms->max_temp_cell.value < bms->ot_threshold - 5)
			bms->state = BMS_OK;
		return OK;
	}

	/* Final stage of balancing with no charging at higher voltages */
	else if (bms->max_voltage_cell.value >= HIGH_V)
	{
		bms->charge_flag = false;
		bms->target_charge_current = 0;

		if (bms->min_voltage_cell.value >= HIGH_V)
			return finalize_balancing(bms, master, slave_array, attempts);

		else if (bms->min_voltage_cell.value >= MID_V)
			return balance_to_min(bms, master, slave_array, 0.005, attempts);

		else
			return balance_to_min(bms, master, slave_array, 0.010, attempts);
	}

	/* Middle stage of balancing with a low current charging current */
	else if (bms->max_voltage_cell.value >= MID_V)
	{
//		bms->charge_flag = true;
//		bms->target_charge_current = MAX_CHARGE_CURRENT / 2;

		if (bms->min_voltage_cell.value >= LOW_V)
			return balance_to_min(bms, master, slave_array, 0.003, attempts);
		else
			return balance_to_min(bms, master, slave_array, 0.010, attempts);
	}

	/* High charge current and balancing to min_cell simultaneously */
	else if (bms->max_voltage_cell.value >= LOW_V)
	{
//		bms->charge_flag = true;
//		bms->target_charge_current = MAX_CHARGE_CURRENT;

		if (bms->min_voltage_cell.value >= LOW_V)    // TESTED
			return balance_to_min(bms, master, slave_array, 0.005, attempts);

		else   // TESTED
			return balance_to_min(bms, master, slave_array, 0.010, attempts);

	}

	/* Stage where max cell is */
	else {  // 3.30 > b->max_cell_v >= 3.00
//		bms->target_charge_current = MAX_CHARGE_CURRENT;
//		bms->charge_flag = true;

		if (bms->BalInfo.state == on)
		{
			return stop_balancing(bms, master, slave_array, attempts);
		}
		return OK;
	}
}


/*****************************************************************************
*  @Description	  Takes over the balancing process if all cells are above ~4.19V
******************************************************************************/
BMSstatus_t finalize_balancing(BMS_info *bms, MCU *master, LTC6811 *slave_array, const uint8_t attempts)
{
	float deviation = bms->max_voltage_cell.value - bms->min_voltage_cell.value;

	if ((bms->max_voltage_cell.value < bms->ov_threshold) && (deviation < 0.001)) {
//		bms->target_charge_current = MAX_CHARGE_CURRENT / 8;
//		bms->charge_flag = true;
		if (bms->BalInfo.state == on) {
			return stop_balancing(bms, master, slave_array, attempts);
		}
		return OK;
	}

	else if (deviation < 0.003) {
		bms->charge_flag = false;
		return balance_to_min(bms, master, slave_array, 0.001, attempts);
	}

	else if (deviation < 0.005) {
		bms->charge_flag = 0;
		return balance_to_min(bms, master, slave_array, 0.003, attempts);
	}

	else {  // deviation >= 0.005
		bms->charge_flag = 0;
		return balance_to_min(bms, master, slave_array, 0.003, attempts);
	}
}

/*****************************************************************************
*  @Description	  Decides the cells that have to be balanced and discharges them
******************************************************************************/
BMSstatus_t balance_to_min(BMS_info *bms, MCU *master, LTC6811 *slave_array, const float deviation, const uint8_t attempts)
{
	for (uint8_t id = 0; id < SLAVES_NUM; ++id)
	{
		for (uint8_t pos = 0; pos < CELLS_NUM; ++pos)
		{
			if (slave_array[id].voltage[pos] - bms->min_voltage_cell.value > deviation)
				bms->BalInfo.unbalanced_cells[id][pos] = true;
			else
		 		bms->BalInfo.unbalanced_cells[id][pos] = false;
		}
	}

	return balance_cells(bms, master, slave_array, attempts);
}


/*****************************************************************************
*  @Description	  Takes the cells that have to discharge and creates the
*  			      discharge command
******************************************************************************/
BMSstatus_t balance_cells(BMS_info *bms, MCU *master, LTC6811 *slave_array, const uint8_t attempts)
{
	bms->BalInfo.state = on;

	uint8_t result;
	bool success = true;

	for (uint8_t id = 0; id < SLAVES_NUM; ++id) {
		uint8_t dcc[CELLS_NUM];
		for (uint8_t pos = 0; pos < CELLS_NUM; ++pos) {
			if (bms->BalInfo.unbalanced_cells[id][pos] == true)
				dcc[pos] = 1;
			else
				dcc[pos] = 0;
		}
		result = discharge(master, slave_array, id, dcc, attempts);
		if (result != OK)
			success = false;
	}
	if (!success)
		return SAFE_WRITE_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Finds the minimum and maximum cell voltage and position out
*  				  of the whole Accumulator Container
******************************************************************************/
// louzoun oi fisses twn connection sta kelia 64 & 82 (cell_0 dld) - tash 3.37 anti gia 3.78V
void find_min_max_v(BMS_info *bms, LTC6811 *slave_array)
{
	bms->max_voltage_cell.value = slave_array[0].voltage[0];
	bms->max_voltage_cell.pos   = 1;
	bms->min_voltage_cell.value = slave_array[0].voltage[0];
	bms->min_voltage_cell.pos   = 1;

	for (uint8_t id =  0; id < SLAVES_NUM; ++id)
	{
		for (uint8_t pos = 0; pos < CELLS_NUM; ++pos)
		{

			if (slave_array[id].voltage[pos] > bms->max_voltage_cell.value)
			{
				bms->max_voltage_cell.value = slave_array[id].voltage[pos];
				bms->max_voltage_cell.pos   = id * 9 + pos + 1;
			}

			if (slave_array[id].voltage[pos] < bms->min_voltage_cell.value)
			{
				bms->min_voltage_cell.value = slave_array[id].voltage[pos];
				bms->min_voltage_cell.pos   = id * 9 + pos + 1;
			}
		}
	}
}



void update_uv_ov_flags(BMS_info *bmsInfo, LTC6811 *slave_array)
{
	bmsInfo->ov_flag_cell = 0;
	bmsInfo->uv_flag_cell = 0;

	for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
	{
		for (uint8_t cell = 0; cell < CELLS_NUM; ++cell)
		{
			if (slave_array[id].uv_flag[cell] == 1)
			{
				bmsInfo->uv_flag_cell = 1;
				break;
			}
			if (slave_array[id].ov_flag[cell] == 1)
			{
				bmsInfo->ov_flag_cell = 1;
				break;
			}
		}
	}
}



/*****************************************************************************
*  @Description	  Stops the discharge of all the cells in the ACCU container.
*                 If voltage_measurement == true, the dcc[] arrays in the
*                 slave_array struct are not updated
******************************************************************************/
BMSstatus_t stop_balancing(BMS_info *bms, MCU *master, LTC6811 *slave_array, const uint8_t attempts)
{
	uint8_t result;
	bool success = true;

	uint8_t dcc[CELLS_NUM];
	for (uint8_t cell = 0; cell < CELLS_NUM; ++cell)
		dcc[cell] = 0;

	for (uint8_t id = 0; id < SLAVES_NUM; ++id) {
		result = discharge(master, slave_array, id, dcc, attempts);
		if (result != OK)
			success = false;
	}

	if (!success)
		return SAFE_WRITE_ERROR;

	bms->BalInfo.state = off;

	return OK;
}

/*****************************************************************************
*  @Description	  Starts discharging all cells for bal_time_minutes and sees
*                 if their voltage after the balancing has dropped. Not to be
*                 used in the final code
******************************************************************************/
BMSstatus_t test_balancing(MCU* master, BMS_info* bmsInfo, LTC6811* slave_array, uint8_t bal_time_minutes, float voltage_difference[16][9])
{
	BMSstatus_t result;
	float initial_voltages[SLAVES_NUM][CELLS_NUM];

	update_voltages_and_wait(master, slave_array, bmsInfo, BC, 0xFF, MD_0);
	read_voltage_registers_all(master, slave_array, bmsInfo, 3);

	for (uint8_t id = 0; id < SLAVES_NUM; ++id)
		for (uint8_t cell = 0; cell < CELLS_NUM; ++cell)
		{
			initial_voltages[id][cell] = slave_array[id].voltage[cell];
			bmsInfo->BalInfo.unbalanced_cells[id][cell] = true;
		}

	result = balance_cells(bmsInfo, master, slave_array, 3);  // discharges all cells in the ACCU
	if (result != OK)
		return result;

	uint8_t pwm_array[CELLS_NUM];
	for (uint8_t i = 0; i < CELLS_NUM; ++i)
		pwm_array[i] = 0x0F;

	uint8_t current_tick = HAL_GetTick();
	while (HAL_GetTick() - current_tick < bal_time_minutes * 60 * 1000)
	{
		// keeps the slaves awake to keep balancing
		write_pwm(master, BC, 0xFF, pwm_array);

		test++;
		osDelay(500);
	}

	result = stop_balancing(bmsInfo, master, slave_array, 3);
	if (result != OK)
		return result;

	osDelay(10);

	update_voltages_and_wait(master, slave_array, bmsInfo, BC, 0xFF, MD_0);
	read_voltage_registers_all(master, slave_array, bmsInfo, 3);

	for (uint8_t id = ID_0; id < SLAVES_NUM; ++id)
	{
		for (uint8_t cell = 0; cell < CELLS_NUM; ++cell)
			voltage_difference[id][cell] = slave_array[id].voltage[cell] - initial_voltages[id][cell];
	}

	return OK;
}

/*****************************************************************************
*  @Description	  Sends the discharge command keeping all other parameters in
*  				  the register unaltered
******************************************************************************/
BMSstatus_t discharge(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t *dcc, const uint8_t attempts)
{
	for(uint8_t i = 0; i < CELLS_NUM; i++)
		slave_array[id].dcc[i] = dcc[i];

	return safe_write_cfgr(master, slave_array, 1, id, slave_array[id].gpio_pulldown,
						   slave_array[id].refon, slave_array[id].adcopt, UV_THRESHOLD,
						   OV_THRESHOLD, dcc, slave_array[id].dcto, attempts);
}

/********************* START OF DIAGNOSTIC FUNCTIONS ************************/


/*****************************************************************************
*  @Description	  Checks both ADCs in an LTC6811 by overlapping measurements
*  				  of cell 7
******************************************************************************/
BMSstatus_t check_adcs(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts)
{
	update_and_wait(master, NO_BC, id, ADOL(MD_3, 0), 220);

	if (read(master, slave_array, id, RDCVC, attempts) != OK)
		return READ_ERROR;

	slave_array[id].adc_deviation = slave_array[id].voltage[6] - slave_array[id].voltage[5];

	if (slave_array[id].adc_deviation >= 0.002)		// ADC Error over 2mV
		return ADC_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Tests the internal digital filters and memory of the chip
*  				  for all possible modes
******************************************************************************/
BMSstatus_t self_test_all_modes(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts)
{
	for (uint8_t md = 0; md < 4; ++md)
		for (uint8_t st = 1; st < 3; ++st)
			if (self_test(master, slave_array, id, md, st, attempts) != OK)
				return SELF_TEST_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Tests the internal digital filters and memory of the chip
*  				  for a specific mode
******************************************************************************/
BMSstatus_t self_test(MCU *master, LTC6811 *slave_array, const ID_t id, const Mode_t md, const uint8_t st, const uint8_t attempts)
{
	uint16_t memory[3]; 	/* Each memory cell has 2 bytes in the register */

	uint8_t adcopt = slave_array[id].adcopt;

	uint16_t self_test_array[4][2][2] = { { { 0x9555, 0x9555 }, { 0x6AAA, 0x6AAA } },
			                              { { 0x9565, 0x9553 }, { 0x6A9A, 0x6AAC } },
									      { { 0x9555, 0x9555 }, { 0x6AAA, 0x6AAA } },
									      { { 0x9555, 0x9555 }, { 0x6AAA, 0x6AAA } } };

	update_and_wait(master, 1, id, CVST(md, st), 220);

	read(master, slave_array, id, RDCVA, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];
	memory[1] = (slave_array[id].tx_buf[3] << 8) | slave_array[id].tx_buf[2];
	memory[2] = (slave_array[id].tx_buf[5] << 8) | slave_array[id].tx_buf[4];

	for (uint8_t i = 0; i < 3; ++i){
		if (memory[i] != self_test_array[md][st - 1][adcopt])
		    return SELF_TEST_ERROR;
	}

	read(master, slave_array, id, RDCVB, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];
	memory[1] = (slave_array[id].tx_buf[3] << 8) | slave_array[id].tx_buf[2];
	memory[2] = (slave_array[id].tx_buf[5] << 8) | slave_array[id].tx_buf[4];

	for (uint8_t i = 0; i < 3; ++i){
		if (memory[i] != self_test_array[md][st - 1][adcopt])
		    return SELF_TEST_ERROR;
	}

	read(master, slave_array, id, RDCVC, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];
	memory[1] = (slave_array[id].tx_buf[3] << 8) | slave_array[id].tx_buf[2];
	memory[2] = (slave_array[id].tx_buf[5] << 8) | slave_array[id].tx_buf[4];

	for (uint8_t i = 0; i < 3; ++i){
		if (memory[i] != self_test_array[md][st - 1][adcopt])
		    return SELF_TEST_ERROR;
	}

	read(master, slave_array, id, RDCVD, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];
	memory[1] = (slave_array[id].tx_buf[3] << 8) | slave_array[id].tx_buf[2];
	memory[2] = (slave_array[id].tx_buf[5] << 8) | slave_array[id].tx_buf[4];

	for (uint8_t i = 0; i < 3; ++i){
		if (memory[i] != self_test_array[md][st - 1][adcopt])
		    return SELF_TEST_ERROR;
	}



	update_and_wait(master, 1, id, AXST(md, st), 220);

	read(master, slave_array, id, RDAUXA, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];
	memory[1] = (slave_array[id].tx_buf[3] << 8) | slave_array[id].tx_buf[2];
	memory[2] = (slave_array[id].tx_buf[5] << 8) | slave_array[id].tx_buf[4];

	for (uint8_t i = 0; i < 3; ++i){
		if (memory[i] != self_test_array[md][st - 1][adcopt])
		    return SELF_TEST_ERROR;
	}

	read(master, slave_array, id, RDAUXB, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];
	memory[1] = (slave_array[id].tx_buf[3] << 8) | slave_array[id].tx_buf[2];
	memory[2] = (slave_array[id].tx_buf[5] << 8) | slave_array[id].tx_buf[4];

	for (uint8_t i = 0; i < 3; ++i) {
		if (memory[i] != self_test_array[md][st - 1][adcopt])
		    return SELF_TEST_ERROR;
	}


	update_and_wait(master, 1, id, STATST(md, st), 220);

	read(master, slave_array, id, RDSTATA, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];
	memory[1] = (slave_array[id].tx_buf[3] << 8) | slave_array[id].tx_buf[2];
	memory[2] = (slave_array[id].tx_buf[5] << 8) | slave_array[id].tx_buf[4];

	for (uint8_t i = 0; i < 3; ++i){
		if (memory[i] != self_test_array[md][st - 1][adcopt])
			return SELF_TEST_ERROR;
	}

	read(master, slave_array, id, RDSTATB, attempts);
	memory[0] = (slave_array[id].tx_buf[1] << 8) | slave_array[id].tx_buf[0];

	if (memory[0] != self_test_array[md][st - 1][adcopt])
		return SELF_TEST_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Clears all registers (0xFF) and reads back to confirm that
*  				  they have been cleared
******************************************************************************/
BMSstatus_t safe_clear(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t attempts)
{
	update_and_wait(master, nbc, id, CLRCELL, 50);

	update_and_wait(master, nbc, id, CLRAUX, 50);

	update_and_wait(master, nbc, id, CLRSCTRL, 50);

	update_and_wait(master, nbc, id, CLRSTAT, 50);

	if (nbc == NO_BC) { // Non-Broadcast command
		if (validate_clear(master, slave_array, id, attempts) != OK)
			return SAFE_CLEAR_ERROR;

	} else
		for (uint8_t id = 0; id < SLAVES_NUM; ++id)
			if (validate_clear(master, slave_array, id, attempts) != OK)
				return SAFE_CLEAR_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Confirms that the chip's registers have been cleared
******************************************************************************/
BMSstatus_t validate_clear(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts)
{
	/* Safe Clear cell voltage registers */
	read(master, slave_array, id, RDCVA, attempts);
	for (uint8_t i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	read(master, slave_array, id, RDCVB, attempts);
	for (uint8_t i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	read(master, slave_array, id, RDCVC, attempts);
	for (uint8_t i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	read(master, slave_array, id, RDCVD, attempts);
	for (uint8_t i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	/* Safe Clear auxiliary registers */
	read(master, slave_array, id, RDAUXA, attempts);
	for (uint8_t i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	read(master, slave_array, id, RDAUXB, attempts);
	for (uint8_t i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	/* Safe Clear status registers */
	read(master, slave_array, id, RDSTATA, attempts);
	for (int i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	read(master, slave_array, id, RDSTATB, attempts);
	for (uint8_t i = 0; i < 5; ++i)
		if (slave_array[id].tx_buf[i] != 0xFF)
			return SAFE_CLEAR_ERROR;

	if ((slave_array[id].tx_buf[5] & 0b11) != 0b11)
		return SAFE_CLEAR_ERROR;

	/* Safe Clear S-control registers */
	read(master, slave_array, id, RDSCTRL, attempts);
	for (int i = 0; i < 6; ++i)
		if (slave_array[id].tx_buf[i] != 0x00)
			return SAFE_CLEAR_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Checks the functionality of the multiplexer and sees if there
*  				  is a thermal shutdown
******************************************************************************/
BMSstatus_t check_mux_and_thsd(MCU *master, LTC6811 *slave_array, const ID_t id, uint8_t attempts)
{
	update_and_wait(master, NO_BC, id, DIAGN, 50);
	if(read(master, slave_array, id, RDSTATB, attempts) != OK)
		return READ_ERROR;

	if (slave_array[id].muxfail != 0)
		return MUX_ERROR;

	if (slave_array[id].thsd != 0)
		return THSD_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Checks if the V_ref2, V_digital, V_analog are within the
*  				  specification limits
******************************************************************************/
BMSstatus_t check_reference_voltages(MCU *master, LTC6811 *slave_array, const ID_t id, uint8_t attempts)
{
	update_and_wait(master, NO_BC, id, ADAX(0, 0b110), 220);

	if(read(master, slave_array, id, RDAUXB, attempts) != OK)
		return READ_ERROR;

	if ((slave_array[id].vref2 > 3.005) || (slave_array[id].vref2 < 2.995))
		return VREF2_ERROR;

	update_and_wait(master, NO_BC, id, ADSTAT(2, 0), 150);

	if(read(master, slave_array, id, RDSTATA, attempts) != OK)
		return READ_ERROR;

	if(read(master, slave_array, id, RDSTATB, attempts) != OK)
		return READ_ERROR;

	if (slave_array[id].itmp > 100)
		return ITMP_ERROR;

	if ((slave_array[id].va < 4.5) || (slave_array[id].va > 5.5))
		return VA_ERROR;

	if ((slave_array[id].vd < 2.7) || (slave_array[id].vd > 3.6))
		return VD_ERROR;

	return OK;
}

/*****************************************************************************
*  @Description	  Checks if there is an open circuit or cut wire in the
*  				  voltage measurements
******************************************************************************/
BMSstatus_t check_open_wire(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, const ID_t id, const Mode_t md, const uint8_t attempts)
{
	uint8_t repeats = 200;        /* Defines how many times the ADOW command is run */
	float pu_voltage[CELLS_NUM];

	 /* pup -> 1 */
	for (uint8_t i = 0; i < repeats; ++i)
		update(master, 1, id, ADOW(md, 0, 0, 1));


    if(read_voltage_registers(master, slave_array, bmsInfo, id , attempts) != OK)
        return READ_ERROR;

    for(uint8_t cell = 0; cell < CELLS_NUM; ++cell) {
        pu_voltage[cell] = slave_array[id].voltage[cell];
    }

    /* pup -> 0 */
    for (uint8_t i = 0; i < repeats; ++i)
    	update(master, 1, id, ADOW(md, 0, 0, 0));

    if(read_voltage_registers(master, slave_array, bmsInfo, id , attempts) != OK)
        return READ_ERROR;

    float voltage_diff[CELLS_NUM];

    for(uint8_t cell = 2; cell <= CELLS_NUM; ++cell){

    	voltage_diff[cell] = pu_voltage[cell - 1] - slave_array[id].voltage[cell - 1];

        if(voltage_diff[cell] < -0.4)
        	slave_array[id].open_wire[cell - 1] = true;
        else
        	slave_array[id].open_wire[cell - 1] = false;

    }

    if(pu_voltage[0] == 0)
        slave_array[id].open_wire[0] = true;
    else
        slave_array[id].open_wire[0] = false;


    if(slave_array[id].voltage[CELLS_NUM - 1] == 0)
        slave_array[id].open_wire[CELLS_NUM] = true;
    else
        slave_array[id].open_wire[CELLS_NUM] = false;


    for(uint8_t cell = 0; cell <= CELLS_NUM; ++cell)
		if(slave_array[id].open_wire[cell] == true)
			return ADOW_ERROR;

    return OK;
}

/*****************************************************************************
*  @Description	  Checks if the sum of all the cells' voltages measured
*                 individually is close to the voltage measurement of the
*                 whole stack measured extra
******************************************************************************/
BMSstatus_t validate_voltage_measurements(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, const ID_t id)
{
	BMSstatus_t result;

	result = update_voltages(master, slave_array, bmsInfo, NO_BC, id, MD_0);
	if (result != OK)
		return result;

	while (poll_status(master, NO_BC, id) != OK)
		osDelay(5);

	result = read_voltage_registers(master, slave_array, bmsInfo, id, 3);  // Reads voltages
	if (result != OK)
		return result;

	result = read(master, slave_array, id, RDSTATA, 3);           // Reads SC
	if (result != OK)
		return result;

	float cells_sum = 0.0;
	for (uint8_t cell = 0; cell < CELLS_NUM; ++cell)
		cells_sum += slave_array[id].voltage[cell];

	slave_array[id].voltage_deviation = cells_sum - slave_array[id].sc;
	if ((slave_array[id].voltage_deviation > 0.07) || (slave_array[id].voltage_deviation < -0.07))
		return VOLTAGE_DEV_ERROR;

	return OK;
}


/*******************************************************************************
*  @Description	  Combines all above functions to provide a full diagnostic test
********************************************************************************/
BMSstatus_t diagnose(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, const ID_t id, uint8_t attempts)
{
	BMSstatus_t result;

	result = safe_clear(master, slave_array, NO_BC, id, attempts);
	if (result != OK)
	{
		slave_array[id].diagnose_state  = SAFE_CLEAR_ERROR;
		return result;
	}

	result = check_adcs(master, slave_array, id, attempts);
	if (result != OK)
	{
		slave_array[id].diagnose_state  = ADC_ERROR;
		return result;
	}

	result = self_test_all_modes(master, slave_array, id, attempts);
	if (result != OK)
	{
		slave_array[id].diagnose_state  = SELF_TEST_FAILED;
		return result;
	}

	result = check_mux_and_thsd(master, slave_array, id, attempts);
	if (result != OK)
	{
		slave_array[id].diagnose_state  = MUX_THSD_FAILED;
		return result;
	}

	result = check_reference_voltages(master, slave_array, id, attempts);
	if (result != OK)
	{
		slave_array[id].diagnose_state  = REF_VOLTAGE_FAILED;
		return result;
	}

	result = check_open_wire(master, slave_array, bmsInfo, id, MD_2, attempts);
	if (result != OK)
	{
		slave_array[id].diagnose_state  = OPEN_WIRE_FAILED;
		return result;
	}

	result = validate_voltage_measurements(master, slave_array, bmsInfo, id);
	if (result != OK)
	{
		slave_array[id].diagnose_state  = VOLT_VALIDATION_FAILED;
		return result;
	}

	slave_array[id].diagnose_state = DIAGNOSE_OK;
	return OK;
}

/*********************** END OF DIAGNOSTIC FUNCTIONS **************************/

/* reads SC as well */
BMSstatus_t update_voltages(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo, Broadcast_t nbc, ID_t id, Mode_t md)
{
	if (bmsInfo->BalInfo.state == on)
	{
		BMSstatus_t result = stop_balancing(bmsInfo, master, slave_array, 3);
		bmsInfo->BalInfo.state = measuring;
		osDelay(10);
		if (result != OK)
			return result;
	}

	update(master, nbc, id, ADCVSC(md, 0));

	return OK;
}


BMSstatus_t update_voltages_and_wait(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo, Broadcast_t nbc, ID_t id, Mode_t md)
{
	if (bmsInfo->BalInfo.state == on)
	{
		BMSstatus_t result = stop_balancing(bmsInfo, master, slave_array, 3);
		bmsInfo->BalInfo.state = measuring;
		if (result != OK)
			return result;
	}

	HAL_Delay(10);

	return update_and_wait(master, nbc, id, ADCVSC(md, 0), 220);
}

/*****************************************************************************
*  @Description	  Reads the voltage registers of a slave
******************************************************************************/
BMSstatus_t read_voltage_registers(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, const ID_t id, const uint8_t attempts)
{
	/* After cells have stopped balancing to measure their voltage, this part resumes the discharging */
	if (bmsInfo->BalInfo.state == measuring)
	{
		BMSstatus_t result = balance_cells(bmsInfo, master, slave_array, attempts);
		if (result != OK)
			return result;

		bmsInfo->BalInfo.state = on;
	}

	if(read(master, slave_array, id, RDCVA, attempts) != OK)
		return READ_ERROR;

	if(read(master, slave_array, id, RDCVB, attempts) != OK)
		return READ_ERROR;

	if(read(master, slave_array, id, RDCVC, attempts) != OK)
		return READ_ERROR;

	if(read(master, slave_array, id, RDCVD, attempts) != OK)
		return READ_ERROR;

	return OK;
}


BMSstatus_t read_voltage_registers_all(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, uint8_t attempts)
{
	BMSstatus_t result;

	for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
	{
		result = read_voltage_registers(master, slave_array, bmsInfo, id, attempts);
		if (result != OK)
			return result;
	}

	return OK;
}


/*****************************************************************************
*  @Description	  Updates the voltage registers of a slave and reads them.
*  				  If balancing, it stops discharging the cells before measuring
*  				  their voltages and then starts again the discharge process
******************************************************************************/
BMSstatus_t get_voltages(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, const Broadcast_t nbc, const ID_t id, const Mode_t md, const uint8_t attempts)
{
	BMSstatus_t result = update_voltages_and_wait(master, slave_array, bmsInfo, nbc, id, md);
	if (result != OK)
		return result;

	ID_t start_id, end_id;
	if (nbc == NO_BC)
	{
		start_id = id;
		end_id = id + 1;
	}
	else
	{
		start_id = ID_0;
		end_id = SLAVES_NUM;
	}

	for (ID_t Id = start_id; Id < end_id; ++Id)
	{
		result = read(master, slave_array, Id, RDSTATA, 3);
		if (result != OK)
			return result;

		result = read_voltage_registers(master, slave_array, bmsInfo, Id, attempts);
		if (result != OK)
			return result;
	}

	return OK;
}


/*****************************************************************************
*  @Description	  Updates the voltage flags registers and reads the flags
*  				  (undervoltage / overvoltage according to write_cfgr)
******************************************************************************/
BMSstatus_t get_voltage_flags(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, Broadcast_t nbc, const ID_t id, const Mode_t md, const uint8_t attempts)
{
	BMSstatus_t result = update_voltages_and_wait(master, slave_array, bmsInfo, nbc, id, md);
	if (result != OK)
		return result;

	if (nbc == NO_BC)
		return read_voltage_flags(master, slave_array, bmsInfo, id, attempts);

	return read_voltage_flags_all(master, slave_array, bmsInfo, attempts);
}


BMSstatus_t read_voltage_flags(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo, ID_t id, uint8_t attempts)
{
	/* After cells have stopped balancing to measure their voltage, this part resumes the discharging */
	if (bmsInfo->BalInfo.state == measuring)
	{
		BMSstatus_t result = balance_cells(bmsInfo, master, slave_array, attempts);
		if (result != OK)
			return result;

		bmsInfo->BalInfo.state = on;
	}

	if(read(master, slave_array, id, RDSTATB, attempts) != OK)
		return READ_ERROR;

	return OK;
}


BMSstatus_t read_voltage_flags_all(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo, uint8_t attempts)
{
	for (ID_t id = ID_0; id < SLAVES_NUM; ++id)
	{
		BMSstatus_t result = read_voltage_flags(master, slave_array, bmsInfo, id, attempts);
		if (result != OK)
			return result;
	}

	return OK;
}


/* SAFE WRITE FUNCTIONS */

/*****************************************************************************
*  @Description	  Writes to the Configuration Register Group and reads back
*  				  from it to confirm successful write
******************************************************************************/
BMSstatus_t safe_write_cfgr(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *gpio,
		                const uint8_t refon, const uint8_t adcopt, const float uv_limit, const float ov_limit,
						const uint8_t *dcc, const uint8_t dcto, const uint8_t attempts)
{

	write_cfgr(master, slave_array, nbc, id, gpio, refon, adcopt, uv_limit, ov_limit, dcc, dcto);

	ID_t start_id;
	ID_t end_id;

	// The ID to be read will only be the requested ID from the function arguments
	if (nbc == NO_BC)
	{
		start_id = id;
		end_id = start_id + 1;
	}

	// The IDs to be read will be all the IDs present on the bus
	else
	{
		start_id = ID_0;
		end_id = SLAVES_NUM;
	}

	for (ID_t counter_id = start_id; counter_id < end_id; ++counter_id)
	{
		uint8_t read_status = read(master, slave_array, counter_id, RDCFGA, attempts);
		if (read_status != OK)
			return read_status;

		// checks if bytes 1 to 5 are the same
		for (uint8_t i = 1; i < 6; ++i)
			if (master->write_buf[i] != slave_array[id].tx_buf[i])
				return SAFE_WRITE_ERROR;

		// checks if REFON and ADCOPT are the same
		if ((master->write_buf[0] & 0b101) != (slave_array[id].tx_buf[0] & 0b101))
			return SAFE_WRITE_ERROR;
	}

	return OK;
}

/*****************************************************************************
*  @Description	  Writes to the PWM Register Group and reads back
*  				  from it to confirm successful write
******************************************************************************/
BMSstatus_t safe_write_pwm(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *pwm, const uint8_t attempts)
{
	write_pwm(master, nbc, id, pwm);

	ID_t start_id;
	ID_t end_id;

	// The ID to be read will only be the requested ID from the function arguments
	if (nbc == NO_BC)
	{
		start_id = id;
		end_id = start_id + 1;
	}

	// The IDs to be read will be all the IDs present on the bus
	else
	{
		start_id = ID_0;
		end_id = SLAVES_NUM;
	}

	for (ID_t counter_id = start_id; counter_id < end_id; ++counter_id)
	{
		uint8_t read_status = read(master, slave_array, id, RDPWM, attempts);
		if (read_status != OK)
			return read_status;

		for (uint8_t i = 0; i < 6; ++i) {
			if (master->write_buf[i] != slave_array[id].tx_buf[i])
				return SAFE_WRITE_ERROR;
		}
	}

	return OK;
}

/*****************************************************************************
*  @Description	  Writes to the S Control Register Group and reads back
*  				  from it to confirm successful write
******************************************************************************/
BMSstatus_t safe_write_sctrl(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *sctrl, const uint8_t attempts)
{
	write_sctrl(master, nbc, id, sctrl);

	ID_t start_id;
	ID_t end_id;

	// The ID to be read will only be the requested ID from the function arguments
	if (nbc == NO_BC)
	{
		start_id = id;
		end_id = start_id + 1;
	}

	// The IDs to be read will be all the IDs present on the bus
	else
	{
		start_id = ID_0;
		end_id = SLAVES_NUM;
	}

	for (ID_t counter_id = start_id; counter_id < end_id; ++counter_id)
	{
		uint8_t read_status = read(master, slave_array, id, RDSCTRL, attempts);
		if (read_status != OK)
			return read_status;

		for (uint8_t i = 0; i < 6; ++i) {
			if (master->write_buf[i] != slave_array[id].tx_buf[i])
				return SAFE_WRITE_ERROR;
		}
	}

	return OK;
}


/* WRITE FUNCTIONS */

/*****************************************************************************
*  @Description	  Writes to the Configuration Register Group
******************************************************************************/
void write_cfgr(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *gpio_pd,
		           const uint8_t refon, const uint8_t adcopt, const float uv_limit, const float ov_limit,
				   const uint8_t *dcc, const uint8_t dcto)
{
	uint16_t vuv = uv_limit / 16 / ADC_RES - 1;
	uint16_t vov = ov_limit / 16 / ADC_RES;

	if (nbc == NO_BC)  // Non-Broadcast command
		for (uint8_t i = 0; i < 5; ++i)
			slave_array[id].gpio_pulldown[i] = gpio_pd[i];
	else
		for (uint8_t j = 0; j < SLAVES_NUM; ++j)
			for (uint8_t i = 0; i < 5; ++i)
				slave_array[j].gpio_pulldown[i] = gpio_pd[i];

	/* gpio_pulldown:    0: GPIOx Pull-Down ON, 1: GPIOx Pull-Down OFF */
	master->write_buf[0] =  0x00 | (gpio_pd[4] << 7) | (gpio_pd[3] << 6) | (gpio_pd[2] << 5) | (gpio_pd[1] << 4) | (gpio_pd[0] << 3) | (refon << 2) | adcopt;
	master->write_buf[1] =  vuv;
	master->write_buf[2] = (vov << 4) | (vuv >> 8);
	master->write_buf[3] = (vov >> 4);
	master->write_buf[4] = 0x00 | (dcc[6] << 7) | (dcc[5] << 6) | (dcc[4] << 4) | (dcc[3] << 3) | (dcc[2] << 2) | (dcc[1] << 1) | dcc[0];
	master->write_buf[5] = 0x00 | (dcto << 4)   | (dcc[8] << 1) | dcc[7];

	write_data(master, nbc, id, WRCFGA);
}

/*****************************************************************************
*  @Description	  Writes to the S Control Register Group
******************************************************************************/
void write_sctrl(MCU *master, const Broadcast_t nbc, const ID_t id, const uint8_t *sctrl)
{
	master->write_buf[0] = sctrl[0]  | (sctrl[1] << 4);
	master->write_buf[1] = sctrl[2]  | (sctrl[3] << 4);
	master->write_buf[2] = 0x00      |  sctrl[4];
	master->write_buf[3] = sctrl[5]  | (sctrl[6] << 4);
	master->write_buf[4] = sctrl[7]  | (sctrl[8] << 4);
	master->write_buf[5] = 0x00;

	write_data(master, nbc, id, WRSCTRL);
}

/*****************************************************************************
*  @Description	  Writes to the PWM Register Group
******************************************************************************/
void write_pwm(MCU *master, const Broadcast_t nbc, const ID_t id, const uint8_t *pwm)
{
	master->write_buf[0] = pwm[0]  | (pwm[1] << 4);
	master->write_buf[1] = pwm[2]  | (pwm[3] << 4);
	master->write_buf[2] = 0x00    |  pwm[4];
	master->write_buf[3] = pwm[5]  | (pwm[6] << 4);
	master->write_buf[4] = pwm[7]  | (pwm[8] << 4);
	master->write_buf[5] = 0x00;

	write_data(master, nbc, id, WRPWM);
}

/*****************************************************************************
*  @Description	  Updates the requested register values and waits until the
*  				  ADC has finished the conversion
******************************************************************************/
BMSstatus_t update_and_wait(MCU *master, const Broadcast_t nbc, const ID_t id, const uint16_t address, const uint16_t timeout)
{
	update(master, nbc, id, address);
	return poll(master, nbc, id, timeout);
}



/* ADDRESSES WITH ARGUMENTS */

/*****************************************************************************
*  @Description   Creates the command for the ADC conversion of voltages
******************************************************************************/
uint16_t ADCV(const uint8_t MD,const uint8_t DCP,const uint8_t CH) {
	return 0b01001100000 | (MD << 7) | (DCP << 4) | CH;
}

/*****************************************************************************
*  @Description   Creates the command that checks for open wires
******************************************************************************/
uint16_t ADOW(const uint8_t MD, const uint8_t DCP,const uint8_t CH, const uint8_t PUP) {
	return 0b01000101000 | (MD << 7) | (PUP << 6) | (DCP << 4) | CH ;
}

/*****************************************************************************
*  @Description   Creates the command for the self test of the voltages
******************************************************************************/
uint16_t CVST(const uint8_t MD, const uint8_t ST) {
	return 0b01000000111 | (MD << 7) | (ST << 5);
}

/*****************************************************************************
*  @Description   Creates the command for the ADC test
******************************************************************************/
uint16_t ADOL(const uint8_t MD, const uint8_t DCP) {
	return 0b01000000001 | (MD << 7) | (DCP << 4);
}

/******************************************************************************
*  @Description   Creates the command for the ADC conversion of GPIOs and Vref2
*******************************************************************************/
uint16_t ADAX(const uint8_t MD, const uint8_t CHG) {
	return 0b10001100000 | (MD << 7) | CHG;
}

/******************************************************************************
*  @Description   Creates the command ADAX with digital redundancy
*******************************************************************************/
uint16_t ADAXD(const uint8_t MD, const uint8_t CHG) {
	return 0b10000000000 | (MD << 7) | CHG;
}

/******************************************************************************
*  @Description   Creates the command for the self test of GPIOs
*******************************************************************************/
uint16_t AXST(const uint8_t MD, const uint8_t ST) {
	return 0b10000000111 | (MD << 7) | (ST << 5);
}

/******************************************************************************
*  @Description   Creates the command for the ADC conversion of status registers
*******************************************************************************/
uint16_t ADSTAT(const uint8_t MD, const uint8_t CHST) {
	return 0b10001101000 | (MD << 7) | CHST;
}

/******************************************************************************
*  @Description   Creates the command for the ADSTAT with digital redundancy
*******************************************************************************/
uint16_t ADSTATD(const uint8_t MD, const uint8_t CHST) {
	return 0b10000001000 | (MD << 7) | CHST;
}

/******************************************************************************
*  @Description   Creates the command for the self test of the status registers
*******************************************************************************/
uint16_t STATST(const uint8_t MD, const uint8_t ST) {
	return 0b10000001111 | (MD << 7) | (ST << 5);
}

/******************************************************************************
*  @Description   Creates the command for the ADC conversion of voltages &
*  				  GPIO 1,2
*******************************************************************************/
uint16_t ADCVAX(const uint8_t MD, const uint8_t DCP) {
	return 0b10001101111 | (MD << 7) | (DCP << 4);
}

/*****************************************************************************
*  @Description   Creates the command for the ADC conversion of voltages &
*  				  the total voltage of the stack (sum of all cells - SC)
******************************************************************************/
uint16_t ADCVSC(const uint8_t MD, const uint8_t DCP) {
	return 0b10001100111 | (MD << 7) | (DCP << 4);
}


/* READ FUNCTIONS */

/*****************************************************************************
*  @Description   Creates a read command of a specific register,
*  				  tramsits it and processes the data - updates the slave struct.
*  				  If the PEC (CRC received) is wrong, it retransmits the
*  				  command for attempt times and if it fails, returns error
******************************************************************************/
BMSstatus_t read(MCU *master, LTC6811 *slave_array, const ID_t id, const uint16_t address, const uint8_t attempts)
{
	for (uint8_t counter = 0; counter < attempts; ++counter) {

		// read_data gets a reference of the specific slave
		master->read_status[counter] = read_data(master, &slave_array[id], id, address);
		if(master->read_status[counter] == OK)
			return OK;
	}
	return READ_ERROR;
}



/* PRIVATE FUNCTION
 * Read and process read data from registers */
/*****************************************************************************
*  @Description   Gets the data received from a specific register of the chip
*  				  and updates the respective struct (slave_array[id])
******************************************************************************/
void process_read_data(LTC6811 *slave, const uint16_t address)
{
	switch (address){

		case RDCFGA:
			slave->adcopt 		=  slave->tx_buf[0]       & 0x01;		/* ADC option bit */
			slave->dten  		= (slave->tx_buf[0] >> 1) & 0x01;		/* Discharge timer enable */
			slave->refon 		= (slave->tx_buf[0] >> 2) & 0x01;		/* Set this bit for REFUP state */

			slave->gpio_level[0]  	= (slave->tx_buf[0] >> 3) & 0x01;	/* GPIO pull-downs */
			slave->gpio_level[1]  	= (slave->tx_buf[0] >> 4) & 0x01;
			slave->gpio_level[2]  	= (slave->tx_buf[0] >> 5) & 0x01;
			slave->gpio_level[3]  	= (slave->tx_buf[0] >> 6) & 0x01;
			slave->gpio_level[4]  	= (slave->tx_buf[0] >> 7) & 0x01;

			// The OV, UV thresholds have a resolution of 1.6mV
			uint16_t min_v		= (slave->tx_buf[1] | (slave->tx_buf[2] << 8)) & 0x0FFF;
			slave->min_v        = (min_v + 1) * 16 * ADC_RES;
			uint16_t max_v      = (slave->tx_buf[2] >> 4) | (slave->tx_buf[3] << 4);
			slave->max_v		=  max_v * 16 * ADC_RES;

			slave->dcc[0]		=  slave->tx_buf[4]       & 0x01;		/* Discharge C1  - CELL1 */
			slave->dcc[1]		= (slave->tx_buf[4] >> 1) & 0x01;		/* Discharge C2  - CELL2 */
			slave->dcc[2]		= (slave->tx_buf[4] >> 2) & 0x01;		/* Discharge C3  - CELL3 */
			slave->dcc[3]		= (slave->tx_buf[4] >> 3) & 0x01;		/* Discharge C4  - CELL4 */
			slave->dcc[4]		= (slave->tx_buf[4] >> 4) & 0x01;		/* Discharge C5  - CELL5 */
			slave->dcc[5]		= (slave->tx_buf[4] >> 6) & 0x01;		/* Discharge C7  - CELL6 */
			slave->dcc[6]		= (slave->tx_buf[4] >> 7) & 0x01;		/* Discharge C8  - CELL7 */
			slave->dcc[7]		=  slave->tx_buf[5]       & 0x01;		/* Discharge C9  - CELL8 */
			slave->dcc[8]		= (slave->tx_buf[5] >> 1) & 0x01;		/* Discharge C10 - CELL9 */

			slave->dcto			= (slave->tx_buf[5] >> 4) & 0x0F;
			break;

		case RDCVA:
			slave->voltage[0] = (slave->tx_buf[0] | (slave->tx_buf[1] << 8)) * ADC_RES;		/* C0-1 ---> Cell_1 */
			slave->voltage[1] = (slave->tx_buf[2] | (slave->tx_buf[3] << 8)) * ADC_RES;		/* C1-2 ---> Cell_2 */
			slave->voltage[2] = (slave->tx_buf[4] | (slave->tx_buf[5] << 8)) * ADC_RES;		/* C2-3 ---> Cell_3 */
			break;

		case RDCVB:
			slave->voltage[3] = (slave->tx_buf[0] | (slave->tx_buf[1] << 8)) * ADC_RES;		/* C3-4 ---> Cell_4 */
			slave->voltage[4] = (slave->tx_buf[2] | (slave->tx_buf[3] << 8)) * ADC_RES;		/* C4-5 ---> Cell_5 */
			break;

		case RDCVC:
			slave->voltage[5] = (slave->tx_buf[0] | (slave->tx_buf[1] << 8)) * ADC_RES;		/* C6-7 ---> Cell_6 */
			slave->voltage[6] = (slave->tx_buf[2] | (slave->tx_buf[3] << 8)) * ADC_RES;		/* C7-8 ---> Cell_7 */
			slave->voltage[7] = (slave->tx_buf[4] | (slave->tx_buf[5] << 8)) * ADC_RES;		/* C8-9 ---> Cell_8 */
			break;

		case RDCVD:
			slave->voltage[8] = (slave->tx_buf[0] | (slave->tx_buf[1] << 8)) * ADC_RES;		/* C9-10 ---> Cell_9 */
			break;

		case RDAUXA:
			for (uint8_t i = 0; (i < 3) && (i < NTCS_NUM); ++i)
				slave->gpio_voltage[i] = (slave->tx_buf[2 * i] | (slave->tx_buf[2 * i + 1] << 8)) * ADC_RES;

			// slave->gpio_voltage[0] = (slave->tx_buf[0] | (slave->tx_buf[1] << 8)) * ADC_RES;		/* GPIO 1 */
			// slave->gpio_voltage[1] = (slave->tx_buf[2] | (slave->tx_buf[3] << 8)) * ADC_RES;		/* GPIO 2 */
			// slave->gpio_voltage[2] = (slave->tx_buf[4] | (slave->tx_buf[5] << 8)) * ADC_RES;		/* GPIO 3 */
			break;

		case RDAUXB:
			for (uint8_t i = 3; (i < 5) && (i < NTCS_NUM); ++i)
				slave->gpio_voltage[i] = (slave->tx_buf[2 * (i - 3)] | (slave->tx_buf[2 * (i - 3) + 1] << 8)) * ADC_RES;

			// slave->gpio_voltage[3] 	= (slave->tx_buf[0] | (slave->tx_buf[1] << 8)) * ADC_RES;		/* GPIO 4 */
			// slave->gpio_voltage[4] 	= (slave->tx_buf[2] | (slave->tx_buf[3] << 8)) * ADC_RES;		/* GPIO 5 */
			slave->vref2	= (slave->tx_buf[4] | (slave->tx_buf[5] << 8)) * ADC_RES;		/* V_REF2 */
			break;

		case RDSTATA:
			slave->sc 	= (slave->tx_buf[0] | slave->tx_buf[1] << 8) * ADC_RES * 20;					/* Sum of all cells */
			slave->itmp	= (slave->tx_buf[2] | slave->tx_buf[3] << 8) * ADC_RES / 0.0075 - 273;	/* Internal Temperature */
			slave->va	= (slave->tx_buf[4] | slave->tx_buf[5] << 8) * ADC_RES;						/* Analog Supply voltage */
			break;

		case RDSTATB:
			slave->vd 	= (slave->tx_buf[0] | slave->tx_buf[1] << 8) * ADC_RES;		/* Digital Supply voltage */

			slave->uv_flag[0] =  slave->tx_buf[2]       & 0x01;		/* Under-voltage cell 1: C1UV  */
			slave->uv_flag[1] = (slave->tx_buf[2] >> 2) & 0x01;		/* Under-voltage cell 2: C2UV  */
			slave->uv_flag[2] = (slave->tx_buf[2] >> 4) & 0x01;		/* Under-voltage cell 3: C3UV  */
			slave->uv_flag[3] = (slave->tx_buf[2] >> 6) & 0x01;		/* Under-voltage cell 4: C4UV  */
			slave->uv_flag[4] =  slave->tx_buf[3]       & 0x01;		/* Under-voltage cell 5: C5UV  */
			slave->uv_flag[5] = (slave->tx_buf[3] >> 4) & 0x01;		/* Under-voltage cell 6: C7UV  */
			slave->uv_flag[6] = (slave->tx_buf[3] >> 6) & 0x01;		/* Under-voltage cell 7: C8UV  */
			slave->uv_flag[7] =  slave->tx_buf[4]       & 0x01;		/* Under-voltage cell 8: C9UV  */
			slave->uv_flag[8] = (slave->tx_buf[4] >> 2) & 0x01;		/* Under-voltage cell 9: C10UV */

			slave->ov_flag[0] = (slave->tx_buf[2] >> 1) & 0x01;		/* Over-voltage cell 1: C1OV  */
			slave->ov_flag[1] = (slave->tx_buf[2] >> 3) & 0x01;		/* Over-voltage cell 2: C2OV  */
			slave->ov_flag[2] = (slave->tx_buf[2] >> 5) & 0x01;		/* Over-voltage cell 3: C3OV  */
			slave->ov_flag[3] = (slave->tx_buf[2] >> 7) & 0x01;		/* Over-voltage cell 4: C4OV  */
			slave->ov_flag[4] = (slave->tx_buf[3] >> 1) & 0x01;		/* Over-voltage cell 5: C5OV  */
			slave->ov_flag[5] = (slave->tx_buf[3] >> 5) & 0x01;		/* Over-voltage cell 6: C7OV  */
			slave->ov_flag[6] = (slave->tx_buf[3] >> 7) & 0x01;		/* Over-voltage cell 7: C8OV  */
			slave->ov_flag[7] = (slave->tx_buf[4] >> 1) & 0x01;		/* Over-voltage cell 8: C9OV  */
			slave->ov_flag[8] = (slave->tx_buf[4] >> 3) & 0x01;		/* Over-voltage cell 9: C10OV */

			slave->muxfail 	  = (slave->tx_buf[5] >> 1) & 0x01;		/* Multiplexer fail */
			slave->thsd		  =  slave->tx_buf[5] & 0x01;			/* Thermal-Shutdown */
			break;

		case RDSCTRL:
			slave->sctrl[0] = slave->tx_buf[0]  & 0x0F;
			slave->sctrl[1] = (slave->tx_buf[0] & 0xF0) >> 4;
			slave->sctrl[2] = slave->tx_buf[1] & 0x0F;
			slave->sctrl[3] = (slave->tx_buf[1] & 0xF0) >> 4;
			slave->sctrl[4] = slave->tx_buf[2] & 0x0F;
			slave->sctrl[5] = slave->tx_buf[3] & 0x0F;
			slave->sctrl[6] = (slave->tx_buf[3] & 0xF0) >> 4;
			slave->sctrl[7] = slave->tx_buf[4] & 0x0F;
			slave->sctrl[8] = (slave->tx_buf[4] & 0xF0) >> 4;
			break;

		case RDPWM:
			slave->pwm[0] = slave->tx_buf[0] & 0x0F;
			slave->pwm[1] = (slave->tx_buf[0] & 0xF0) >> 4;
			slave->pwm[2] = slave->tx_buf[1] & 0x0F;
			slave->pwm[3] = (slave->tx_buf[1] & 0xF0) >> 4;
			slave->pwm[4] = slave->tx_buf[2] & 0x0F;
			slave->pwm[5] = slave->tx_buf[3] & 0x0F;
			slave->pwm[6] = (slave->tx_buf[3] & 0xF0) >> 4;
			slave->pwm[7] = slave->tx_buf[4] & 0x0F;
			slave->pwm[8] = (slave->tx_buf[4] & 0xF0) >> 4;
			break;
	}
}


/*****************************************************************************
*  @Description   Gets as input the characteristics of the voltage divider
*  				  and the Resistance - Temperature curve of the NTCs used and
*  				  creates the Voltage - Temperature curve. Called only once
******************************************************************************/
bool ntc_rh_init(LTC6811 *slave_array, const float *ntc_resistance, float *ntc_voltage, const float vref, const uint32_t resistor_kOhm, const uint16_t length)
{
	for (uint16_t i = 0; i < length; ++i)
		ntc_voltage[i] = vref * ntc_resistance[i] / (ntc_resistance[i] + resistor_kOhm);

	bool init_ok = true;

	/* The following checks if the ntc_to_cell_position[][] array has been written correctly by the programmer */
	for (uint8_t id = 0; id < SLAVES_NUM; ++id)
	{
		for (uint8_t ntc = 0; ntc < NTCS_NUM - 1; ++ntc)
			if ((ntc_to_cell_position[id][ntc] < 1) || (ntc_to_cell_position[id][ntc] > 144))
				init_ok = false;

		if (ntc_to_cell_position[id][NTCS_NUM - 1] != 255)
		{
			slave_array[id].humidity = 0xFF; // Since humidity is a percentage from 0 to 100, the value 0xFF = 255 shows that there is no RH sensor on the slave
			if ((ntc_to_cell_position[id][NTCS_NUM - 1] < 1) || (ntc_to_cell_position[id][NTCS_NUM - 1] > 144))
					init_ok = false;
		}
	}
	return init_ok;
}


/*****************************************************************************
*  @Description   Uses the binary search algorithm to match the given NTC
*  				  voltage to the corresponding temperature - first called in
*  				  lookup_value
******************************************************************************/
uint16_t find_position(const float* lookup_table, const uint16_t start, const uint16_t end, const float value)
{
	if (value < lookup_table[end])
	    return end;

	if (value > lookup_table[start])
	    return start;

	uint16_t mid = start + (end - start) / 2;

	if (mid == start) {
	    if (lookup_table[start] - value < value - lookup_table[end])
	        return start;
	    else
	        return end;
	}

	if (value == lookup_table[mid])
	    return mid;

	if (value > lookup_table[mid])
	    return find_position(lookup_table, start, mid, value);

	// value < lookup_table[mid]
	return find_position(lookup_table, mid, end, value);
}


/*****************************************************************************
*  @Description   Finds the correct temperature for the given NTC voltage
******************************************************************************/
uint16_t lookup_value(const float *lookup_table, const uint16_t length, const float value) {
	return find_position(lookup_table, 0, length - 1, value);
}



void update_temperatures(MCU* master, Broadcast_t nbc, ID_t id, Mode_t md)
{
	update(master, nbc, id, ADAX(md, 0));
}


BMSstatus_t update_temperatures_and_wait(MCU* master, Broadcast_t nbc, ID_t id, Mode_t md)
{
	return update_and_wait(master, nbc, id, ADAX(md, 0), 220);
}


BMSstatus_t read_temperature_registers(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts)
{

	if (read(master, slave_array, id, RDAUXA, attempts) != OK)
		return READ_ERROR;
	if (read(master, slave_array, id, RDAUXB, attempts) != OK)
		return READ_ERROR;

	uint16_t position;
	for(uint8_t i = 0; i < NTCS_NUM; i++)
	{
		/* If no NTC is placed in this slot */
		if (ntc_to_cell_position[id][i] == 0xFF)
		{
			slave_array[id].temp[i] = 255; // Temperature can not reach 255, so this value indicates that in this position a RH sensor is connected
			read_humidity(slave_array, id);
		}
		else
		{
			position = lookup_value(NTC_voltage, NTC_LUT_LENGTH, slave_array[id].gpio_voltage[i]);
			slave_array[id].temp[i] = 1.0 * position / 2;
		}

	}

	return OK;
}


/*****************************************************************************
*  @Description   Sends the command for voltage conversion of the NTCs, reads
*  				  the result and converts it to temperature
******************************************************************************/
BMSstatus_t get_temperatures(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const float *ntc_voltage, const ID_t id, const Mode_t md, const uint8_t attempts)
{
	update_temperatures(master, nbc, id, md);
	poll(master, nbc, id, 220);

	return read_temperature_registers(master, slave_array, id, attempts);
}


/*****************************************************************************
*  @Description   Reads the temperatures that are already in the structs and
*  				  finds the minimum and maximum as well as their cell position
******************************************************************************/
void find_min_max_temp(BMS_info *bms, LTC6811 *slave_array)
{
	bms->max_temp_cell.value  = slave_array[0].temp[0];
	bms->max_temp_cell.pos    = ntc_to_cell_position[0][0];
	bms->min_temp_cell.value  = slave_array[0].temp[0];
	bms->min_temp_cell.pos    = ntc_to_cell_position[0][0];

	for (uint8_t id =  0; id < SLAVES_NUM; ++id)
	{
		for (uint8_t ntc = 0; ntc < NTCS_NUM; ++ntc)
		{

			if ((slave_array[id].temp[ntc] > bms->max_temp_cell.value) && (slave_array[id].temp[ntc] != 0xFF)) // second condition guarantees that this measurement belongs to an NTC and not a RH sensor
			{
				bms->max_temp_cell.value = slave_array[id].temp[ntc];
				bms->max_temp_cell.pos   = ntc_to_cell_position[id][ntc];
			}

			if ((slave_array[id].temp[ntc] < bms->min_temp_cell.value) && (slave_array[id].temp[ntc] != 0xFF)) // second condition guarantees that this measurement belongs to an NTC and not a RH sensor
			{
				bms->min_temp_cell.value = slave_array[id].temp[ntc];
				bms->min_temp_cell.pos   = ntc_to_cell_position[id][ntc];
				if (bms->min_temp_cell.value == 0)
					osDelay(1);
			}
		}
	}
}



/*****************************************************************************
*  @Description   Reads the voltage from the slave's struct and converts it to
*  				  relative humidity percentage
*  				  Humidity sensor used: 10142048-31
******************************************************************************/
void read_humidity(LTC6811 *slave_array, const ID_t id)
{
	float humidity = -12.5 + 125 * slave_array[id].gpio_voltage[RH_GPIO_NUM] / slave_array[id].va;
	if (humidity > 100)
		humidity = 100;

	slave_array[id].humidity = humidity;
}



/*****************************************************************************
*  @Description   Reads the structs in slave_array and finds the greatest
*  				  humidity and its position
******************************************************************************/
void find_max_humidity(BMS_info *bmsInfo, LTC6811 *slave_array)
{
	bmsInfo->max_humidity.value = 0;
	bmsInfo->max_humidity.pos   = 0;

	for (uint8_t id = 0; id < SLAVES_NUM; ++id)
	{
		if ((slave_array[id].humidity != 255) && (slave_array[id].humidity > bmsInfo->max_humidity.value)) // First condition guarantees that there is a RH sensor on that slave
		{
			bmsInfo->max_humidity.value = slave_array[id].humidity;
			bmsInfo->max_humidity.pos   = id + 1;
		}
	}
}



void update_status(MCU* master, Broadcast_t nbc, ID_t id, Mode_t md)
{
	update(master, nbc, id, ADSTAT(md, 0));
}

BMSstatus_t read_status_registers(MCU* master, LTC6811* slave_array, ID_t id, uint8_t attempts)
{
	if (read(master, slave_array, id, RDSTATA, attempts) != OK)
		return READ_ERROR;
	if (read(master, slave_array, id, RDSTATB, attempts) != OK)
		return READ_ERROR;

	return OK;
}



BMSstatus_t read_cfgr_register(MCU* master, LTC6811* slave_array, ID_t id, uint8_t attempts)
{
	if (read(master, slave_array, id, RDCFGA, attempts) != OK)
		return READ_ERROR;

	return OK;
}



// USED FOR DEBUGGING
BMSstatus_t measure_all(MCU *master, LTC6811 *slave_array, BMS_info *bms, const float *ntc_voltage, const ID_t id) {

	if(get_voltages(master, slave_array, bms, id, MD_2, 0, 1) != OK)
		return READ_ERROR;

	get_temperatures(master, slave_array, NO_BC, ntc_voltage, id, MD_2, 1);

	//update_and_wait(master, NO_BC, id, ADSTAT(2, 0), 150);		// read status to check for OV's during charging
	//read(master, slave_array, id, RDSTATA, 1);
	//read(master, slave_array, id, RDSTATB, 1);

	return OK;

}




/************************* LOW LEVEL FUCTIONS ***********************/


/******************************************************************************
*  @Description   Wakes up the chip. Transitions the core from Sleep to Standby
*  				  The chip needs 400us to properly wake up. If no valid command
*  				  has been received for 1.8 to 2.2 seconds, it goes back to
*  				  sleep
*******************************************************************************/
void wake_up(const MCU *master) {

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_RESET);
	//for (unsigned int i = 0; i < 1000; ++i);
	uint32_t tick0 = xTaskGetTickCount();
	while(xTaskGetTickCount() - tick0 < 10);
	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);
}


/*****************************************************************************
*  @Description   Transitions the isoSPI port from Idle to Ready. The chip
*  				  needs 10us to transition properly. If there is no
*  				  activity (either valid or invalid command) on the isoSPI bus
*  			      for 6.7ms, it goes back to idle
******************************************************************************/
void isospi_ready(const MCU *master) {

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_RESET);

	//osDelay(2);
	uint16_t j = 0;
	for (uint16_t i = 0; i < 1000; ++i)
		j++;

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 1000; ++i)
		j++;
}

/* HAL Functions */

/*****************************************************************************
*  @Description   Creates the read command, transmits it and processes the
*  				  received bytes. If the read fails once, it returns Error
******************************************************************************/
// slave is a pointer to an LTC6811 struct
BMSstatus_t read_data(MCU *master, LTC6811 *slave, const ID_t id, const uint16_t address){

    uint16_t command = address | (id << 11) | (0x0001 << 15);
	uint8_t  cmd0  	 = command >> 8;
	uint8_t  cmd1  	 = command;
 	uint8_t  cmd[2]  = {cmd0, cmd1};

 	master->command_buf = command;

	uint16_t cmd_pec = pec15(cmd, 2);
	uint8_t cmd_pec0 = cmd_pec >> 8;
	uint8_t cmd_pec1 = cmd_pec;

	uint8_t write_msg[4 + 8] = {cmd0, cmd1, cmd_pec0, cmd_pec1};
	for (uint8_t i = 4; i < 12; ++i)
		write_msg[i] = 0;
	uint8_t read_msg[4 + 8];

	// isoSPI must be in READY state
	isospi_ready(master);

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive(master->spiHandle, write_msg, read_msg, sizeof(write_msg), TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);
		return TX_ERROR;
	}
	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);

	for (uint8_t i = 4; i < 10; ++i)
		slave->tx_buf[i - 4] = read_msg[i];

	uint16_t data_pec = pec15(slave->tx_buf, 6);

	/* Check if the LTC6811 sent wrong PEC */
    slave->pec_status[0] = data_pec;
    slave->pec_status[1] = (read_msg[10] << 8) | read_msg[11];
	if (slave->pec_status[0] != slave->pec_status[1]){
		slave->pec_errors++;
		return READ_ERROR;
	}
	process_read_data(slave, address);
	slave->last_valid_msg = xTaskGetTickCount();     // If the code gets here, the slave has communicated successfully

	/* No error in communicating with LTC6811 */
	return OK;
}



/*****************************************************************************
*  @Description   Transmits the update command (given in address argumant)
*  				  which starts the ADC conversions of the requested quantity
******************************************************************************/
BMSstatus_t update(MCU *master, const Broadcast_t nbc, const ID_t id, const uint16_t address){

	uint16_t command;
	if (nbc == 1)      // Non-Broadcast command
		command = 0x8000 | (id << 11) | address;
	else
		command = address;

	master->command_buf = command;

	uint8_t cmd0 = command >> 8;
	uint8_t cmd1 = command;
	uint8_t cmd[2] = {cmd0, cmd1};

	uint16_t cmd_pec = pec15(cmd, 2);
	uint8_t cmd_pec0 = cmd_pec >> 8;
	uint8_t cmd_pec1 = cmd_pec;
	uint8_t monitor_msg[4] = {cmd0, cmd1, cmd_pec0, cmd_pec1};


	// isoSPI must be in READY state
	isospi_ready(master);

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(master->spiHandle, monitor_msg, sizeof(monitor_msg), TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);
		return TX_ERROR;
	}

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);

	return OK;
}


/* PRIVATE FUNCTIONS */

/*****************************************************************************
*  @Description   Gets 6 bytes from write_X functions and writes it to the
*  				  respective register. Does not check if data has been
*  				  successfully written to the register
******************************************************************************/
BMSstatus_t write_data(MCU *master, const Broadcast_t nbc, const ID_t id, const uint16_t address) {

	uint16_t command;
	if (nbc == NO_BC)   // Non-Broadcast command
		command = 0x8000 | (id << 11) | address;

	else
		command = 0x0000 | address;

	master->command_buf = command;

	uint8_t cmd0 = command >> 8;
	uint8_t cmd1 = command;
	uint8_t cmd[2] = {cmd0, cmd1};

	uint16_t cmd_pec  = pec15(cmd, 2);
	uint8_t  cmd_pec0 = cmd_pec >> 8;
	uint8_t  cmd_pec1 = cmd_pec;

	uint16_t data_pec  = pec15(master->write_buf, 6);
	uint8_t  data_pec0 = data_pec >> 8;
	uint8_t  data_pec1 = data_pec;

	uint8_t write_msg[12] = {cmd0, cmd1, cmd_pec0, cmd_pec1};
	for (uint8_t i = 0; i < 6; ++i)
		write_msg[i + 4] = master->write_buf[i];
	write_msg[10] = data_pec0;
	write_msg[11] = data_pec1;


	// isoSPI must be in READY state
	isospi_ready(master);

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(master->spiHandle, write_msg, sizeof(write_msg), TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);
		return TX_ERROR;
	}

	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);
	return OK;

}


/*****************************************************************************
*  @Description	  Waits until the ongoing ADC has finished the conversion
*                 and returns error if it exceeds the defined timeout
******************************************************************************/
BMSstatus_t poll(MCU *master, const Broadcast_t nbc, const ID_t id, const uint16_t timeout)
{
	uint32_t start_time = HAL_GetTick();

	while (poll_status(master, nbc, id) != OK)
	{
		if (HAL_GetTick() - start_time > timeout)
			return TIMEOUT_ERROR;
	}
	return OK;
}


/*****************************************************************************
*  @Description   Transmits a command to detect if the ADC conversion has
*  				  ended. If after the command and while the CS is Low, the MISO
*  				  line remains low, the conversion has not ended
******************************************************************************/
BMSstatus_t poll_status(MCU *master, const Broadcast_t nbc, const ID_t id){
	uint16_t command;
	if (nbc == 1)              // Non-Broadcast command
		command = 0x8000 | (id << 11) | PLADC;
	else
		command = PLADC;

	master->command_buf = command;

	uint8_t cmd0 = command >> 8;
	uint8_t cmd1 = command;
	uint8_t cmd[2] = {cmd0, cmd1};

	uint16_t cmd_pec = pec15(cmd, 2);
	uint8_t cmd_pec0 = cmd_pec >> 8;
	uint8_t cmd_pec1 = cmd_pec;
	uint8_t poll_msg[5] = {cmd0, cmd1, cmd_pec0, cmd_pec1, 0};
	uint8_t read_msg[5] = {0, 0, 0, 0, 0};

	// isoSPI must be in READY state
	isospi_ready(master);
	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(master->spiHandle, poll_msg, read_msg, sizeof(poll_msg), TIMEOUT) != HAL_OK) {
		HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);
		return TX_ERROR;
	}
	HAL_GPIO_WritePin(master->csBMS, master->csBMSPin, GPIO_PIN_SET);

	if (read_msg[4] != 0xFF)
		return ADC_BUSY;

	return OK;
}


#ifndef LTC6811_H_
#define LTC6811_H_

#include "math.h"
#include <stdint.h>
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

#include "cmsis_os.h"

#include <LTC6811-board.h>  // Includes details of the specific PCB that the code runs

#define ADC_RES 		 0.0001f
#define MAX_ADC_ERROR    0.001f

#define TIMEOUT			   50


/* Constant register address */
typedef enum
{
	WRCFGA 	  = 0x001,  // 00000000001
	RDCFGA	  =	0x002,  // 00000000010
	RDCVA	  =	0x004,  // 00000000100
	RDCVB	  =	0x006,  // 00000000110
	RDCVC	  =	0x008,  // 00000001000
	RDCVD	  =	0x00A,  // 00000001010
	RDAUXA	  =	0x00C,  // 00000001100
	RDAUXB	  =	0x00E,  // 00000001110
	RDSTATA	  =	0x010,  // 00000010000
	RDSTATB	  =	0x012,  // 00000010010
	WRSCTRL	  =	0x014,  // 00000010100
	RDSCTRL	  =	0x016,  // 00000010110
	WRPWM	  =	0x020,  // 00000100000
	RDPWM	  =	0x022,  // 00000100010
	STSCTRL	  =	0x019,  // 00000011001
	CLRSCTRL  =	0x018,  // 00000011000
	CLRCELL	  =	0x711,  // 11100010001
	CLRAUX	  =	0x712,  // 11100010010
	CLRSTAT	  =	0x713,  // 11100010011
	PLADC	  =	0x714,  // 11100010100
	DIAGN	  =	0x715,  // 11100010101

}LTC6811Command_t;



/* Status of the executed commands */
typedef enum
{
	OK 				 = 0,  /* Command executed without errors */
	TX_ERROR			,  /* Pops in case of HAL_SPI unable to send frame command */
	READ_ERROR		    ,  /* Read error occurs if received PEC is wrong */
	SAFE_WRITE_ERROR    ,  /* Safe Write error occurs if the register's content is not what has just been written */
	SAFE_CLEAR_ERROR    ,  /* Safe Clear error occurs if the register's content has not been cleared */
	SELF_TEST_ERROR     ,  /* Self Test error occurs if the IC's internal filters or memory are not working properly */
	ADC_BUSY            ,  /* Indicated the ADC has not finished the conversion */
	ADC_ERROR           ,  /* ADC error occurs if the ADC has an unacceptable tolerance */
	TIMEOUT_ERROR       ,  /* Returned from poll() if ADC conversion is not done in predefined time */
	ADOW_ERROR		    ,  /* ADOW error occurs if an open wire is detected */
	MUX_ERROR		    ,  /* MUX error occurs if the multiplexer is not working as expected */
	THSD_ERROR		    ,  /* THSD error occurs if a thermal shutdown has occured (internal temperature > 150 C) */
	VREF2_ERROR		    ,  /* Vref_2 error occurs if the reference voltage 2 is out of the specified limits */
	ITMP_ERROR		    ,  /* ITMP error occurs if the chips measured temperature is above 100 C */
	VA_ERROR		    ,  /* Va error occurs if the reference analog voltage is out of the specified limits */
	VD_ERROR		    ,  /* Vd error occurs if the reference digital voltage (used in the ADCs) is out of the specified limits */
	NTC_LUT_ERROR       ,  /* Occurs only in ntc_rh_init() if the ntc_to_cell_position[][] array has wrong syntax */
	VOLTAGE_DEV_ERROR   ,  /* Occurs when SC differs more than a threshold from the sum of the cells measured individually */
	COMM_ERROR          ,  /* Returned when a slave / slaves has lost communication with the master */

}BMSstatus_t;



/* All the IDs of the slaves on the bus */
typedef enum
{
	ID_0  = 0,
	ID_1,
	ID_2,
	ID_3,
	ID_4,
	ID_5,
	ID_6,
	ID_7,
	ID_8,
	ID_9,
	ID_10,
	ID_11,
	ID_12,
	ID_13,
	ID_14,
	ID_15,

}ID_t;



/* Dictates whether the command will be broadcasted to all slaves */
typedef enum
{
	BC    = 0,
	NO_BC = 1,

}Broadcast_t;


/* ADC sampling frequency modes */
typedef enum
{
	MD_0 = 0,	// 422Hz : Default	 : 12-13msec
	MD_1 = 1,	// 27kHz : Fast mode : 1msec
	MD_2 = 2,	// 7kHz  : Normal    : 3-4msec
	MD_3 = 3,	// 26HZ  : Filtered  : 201msec

}Mode_t;



/*
* ------------------------ Begin of LTC6811 structures -----------------------
*/

/* "Private" struct used for BMS_info struct */
typedef struct obj
{
	float value;
	uint8_t pos;

}obj;

typedef struct BMS_info
{
	BMS_Mode_t mode;

	obj min_voltage_cell;
	obj max_voltage_cell;

	uint8_t uv_flag_cell;   // 1 : at least one cell has triggered the undervoltage flag from the LTC6811
	uint8_t ov_flag_cell;   // 1 : at least one cell has triggered the overvoltage flag from the LTC6811

	float uv_threshold;
	float ov_threshold;

	uint8_t ut_threshold;
	uint8_t ot_threshold;

	bool  charge_flag;
	uint16_t target_charge_current;

	obj max_humidity;

	enum      // BMS States, if bms_state != OK, ShutDown should open
	{
		BMS_OK 			   = 0,
		OVERVOLTAGE 		  ,
		UNDERVOLTAGE          ,
		OVERCURRENT_CHARGE    ,
		OVERCURRENT_DISCHARGE ,
		OVERTEMP       		  ,
		UNDERTEMP			  ,
		ISABELLE_DEAD		  , /* Current sensor not responding */
		COMMUNICATION_ERROR   ,	/* Lost communication with a slave */
		SLAVE_ERROR			  , /* ADOW error, Diagnostics Error */
		HUMIDITY_ERROR		  ,
		ISABELLE_NO_VOLTAGE   ,

	} state,
	  last_error;


	float 	 accu_voltage;
	float    avg_cell_voltage;
	float    avg_cell_temp;
	obj 	 min_temp_cell;
	obj  	 max_temp_cell;

	uint8_t  comm_error_id;       /* 1 -> SLAVES_NUM,   if comm_error_id == 0, there is no error */
	uint8_t  slave_error_id;      /* 1 -> SLAVES_NUM,  if slave_error_id == 0, there is no error */

	/* Struct including info about the balancing process */
	struct
	{
		uint8_t balancing_enabled;
		bool    unbalanced_cells[SLAVES_NUM][CELLS_NUM];

		enum
		{
			off    = 0,
			on        ,
			measuring ,   // cells are temporarily stopped from discharging to measure voltages
		} state;

	}BalInfo;

	uint8_t min_SoC, max_SoC, min_initial_SoC, max_initial_SoC;       /* State of Charge - Energy, percentage % */
	float   min_total_Ah, max_total_Ah;

} BMS_info;


typedef struct MCU
{
	SPI_HandleTypeDef   *spiHandle;            		  /* SPI characteristics and Fault_Masks */
    GPIO_TypeDef        *csBMS;               		  /* CS of LTC6820 control */
    GPIO_TypeDef        *AMS_OK;            		  /* AMS_OK GPIO pin */
    uint16_t            csBMSPin;
    uint16_t            AMS_OKPin;

    uint8_t             read_status[MAX_ATTEMPTS];    /* BMS controller - Error logs */
    uint8_t             write_buf[6];                 /* (dEbugMode)Temporary buffer to save "probably" written data */
    uint16_t			command_buf;  				  /* Stores the read-write-update commands sent by the master */

	bool     ids_in_bus[SLAVES_NUM];                  /* Finds which IDs (slaves) are connected to the isoSPI bus */
	uint16_t no_comm_time[SLAVES_NUM];                /* After update_comm_errors() is called, it stores how long a slave has not responded in ms */

} MCU;


typedef struct LTC6811
{
	/* Debugging Variables */
	uint8_t  tx_buf[6];                 	/* (dEbugMode)Temporary save the data that are read */
	uint16_t pec_status[2];             	/* Contains {expected PEC, received PEC}            */
	uint16_t pec_errors;

	/* PCB - Hardware errors */
	bool open_wire[CELLS_NUM + 1];    /* Stores the result of the ADOW command */
	float voltage_deviation;          /* Stores the voltage difference of SC minus the sum of the cells measured individually */
	float adc_deviation;              /* Stores the deviation of the 2 ADCs in the chip */

	enum      // Diagnose states: To verify which diagnose function failed at the specific slave ID
	{
		DIAGNOSE_OK     	= 0,
		SAFE_CLEAR_FAILED 	   ,
		ADC_FAILED			   ,
		SELF_TEST_FAILED	   ,
		MUX_THSD_FAILED		   ,
		REF_VOLTAGE_FAILED	   ,
		OPEN_WIRE_FAILED	   ,
		VOLT_VALIDATION_FAILED ,
		NO_DIAGNOSE_TEST       ,

	}diagnose_state;

	/* COMM error information */
	uint32_t last_valid_msg;                /* dictates the time when the last valid message (correct PEC) was received from this slave by the MCU */

	/* REGISTERS */
	/* Configuration Register Group */
	uint8_t gpio_pulldown[5];				/* 0: GPIOx Pull-Down ON, 1: GPIOx Pull-Down OFF */
	uint8_t gpio_level[5];                  /* 0: GPIOx is LOW, 1: GPIOx is HIGH */
	uint8_t refon;       					/* Transitions to REFUP state */
	uint8_t dten;            				/* Enables the discharge timer */
	uint8_t adcopt;      					/* ADC mode setting */
	float   min_v;         					/* UV voltage setting */
	float   max_v;         					/* OV voltage setting */
	uint8_t dcc[CELLS_NUM];  				/* Turns ON cell balancing */
	uint8_t dcto;            				/* Discharge timer */

	/* Cell Voltage Register Groups A-D */
	float voltage[CELLS_NUM];

	/* Auxiliary Register Groups A-B */
	float gpio_voltage[NTCS_NUM];    		/* Voltage in GPIOs from NTCs */
	float vref2;             				/* NTCs Reference Voltage */
	float temp[NTCS_NUM];
	float humidity;

	/* Status Register Group A */
	float sc;                				/* Sum of all cells */
	float itmp;              				/* Internal Temperature */
	float va;
	/* Analog Power Supply: 4,5 -> 5,5 V */
	/* Status Register Group B */
	float   vd;                				/* Digital Power Supply: 2.7 -> 3.6 V */
	uint8_t uv_flag[CELLS_NUM];
	uint8_t ov_flag[CELLS_NUM];
	uint8_t muxfail;         				/* Multiplexer self test. 0: OK, 1: ERROR */
	uint8_t thsd;            				/* Thermal-Shutdown. 0: OK, 1: ERROR */

	/* S Control Register Group */
	uint8_t sctrl[CELLS_NUM];        		/* S pin control (used for active balancing) */

	/* PWM Register Group */
	uint8_t pwm[CELLS_NUM];        			/* PWM control (used for passive balancing) */

} LTC6811;

/*
* ------------------------ End of LTC6811 structures -----------------------
*/


extern ID_t test_id;

/* STRUCT INITIALIZATION FUNCTIONS */
BMSstatus_t BMS_Init(MCU *master, LTC6811 *slave_array, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *CS_BMS,  GPIO_TypeDef *AMS_OK,
						uint16_t CS_pin, uint64_t AMS_OK_pin, BMS_info *bms, uint8_t refon, uint8_t adcopt);
void bms_info_struct_init(BMS_info *bmsInfo);
void mcu_struct_init(MCU *master, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *CS_BMS,  GPIO_TypeDef *AMS_OK, uint16_t CS_pin, uint64_t AMS_OK_pin);
void slave_struct_init(LTC6811 *slave);
void slave_array_init(LTC6811 *slave_array);

/* GENERAL FUNCTIONS */
void find_ids_in_bus(MCU *master, LTC6811 *slave_array);
void update_BMS_info_struct(MCU* master, LTC6811 *slave_array, BMS_info* bmsInfo);

/* DEBUGGING FUNCTIONS */
BMSstatus_t measure_all(MCU *master, LTC6811 *slave_array, BMS_info* bms, const float *ntc_voltage, const ID_t id);

/* COMMUNICATION FUNCTIONS */
BMSstatus_t update_comm_errors(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo);

/* CHARGING - BALANCING FUNCTIONS */
BMSstatus_t update_balancing_cells(BMS_info *bmsInfo,  MCU *master, LTC6811 *slave_array, const uint8_t attempts);
BMSstatus_t finalize_balancing(BMS_info *bmsInfo,  MCU *master, LTC6811 *slave_array, const uint8_t attempts);
BMSstatus_t balance_to_min(BMS_info *bmsInfo,  MCU *master, LTC6811 *slave_array, const float deviation, const uint8_t attempts);
BMSstatus_t balance_cells(BMS_info *bms, MCU *master, LTC6811 *slave_array, const uint8_t attempts);
BMSstatus_t stop_balancing(BMS_info *bms, MCU *master, LTC6811 *slave_array, const uint8_t attempts);
BMSstatus_t test_balancing(MCU* master, BMS_info* bmsInfo, LTC6811* slave_array, uint8_t bal_time_minutes, float voltage_difference[16][9]);
BMSstatus_t discharge(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t *dcc, const uint8_t attempts);

/* GET VOLTAGE FUNCTIONS */
BMSstatus_t update_voltages(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo, Broadcast_t nbc, ID_t id, Mode_t md);
BMSstatus_t update_voltages_and_wait(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo, Broadcast_t nbc, ID_t id, Mode_t md);
BMSstatus_t read_voltage_registers(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, ID_t id, uint8_t attempts);
BMSstatus_t read_voltage_registers_all(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, uint8_t attempts);
BMSstatus_t get_voltages(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, const Broadcast_t nbc, const ID_t id, const Mode_t md, const uint8_t attempts);
BMSstatus_t get_voltage_flags(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, Broadcast_t nbc, const ID_t id, const Mode_t md, const uint8_t attempts);
BMSstatus_t read_voltage_flags(MCU* master, LTC6811* slave_array, BMS_info* bmsInfo, ID_t id, uint8_t attempts);
BMSstatus_t read_voltage_flags_all(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, const uint8_t attempts);

void find_min_max_v(BMS_info *bmsInfo, LTC6811 *slave_array);
void update_uv_ov_flags(BMS_info *bmsInfo, LTC6811 *slave_array);

/* GET TEMPERATURE - HUMIDITY FUNCTIONS */
uint16_t lookup_value(const float *lookup_table, const uint16_t length, const float value);
uint16_t find_position(const float* lookup_table, const uint16_t start, const uint16_t end, const float value);
bool ntc_rh_init(LTC6811 *slave_array, const float *ntc_resistance, float *ntc_voltage, const float vref, const uint32_t resistor_kOhm, const uint16_t length);
void update_temperatures(MCU* master, Broadcast_t nbc, ID_t id, Mode_t md);
BMSstatus_t update_temperatures_and_wait(MCU* master, Broadcast_t nbc, ID_t id, Mode_t md);
BMSstatus_t read_temperature_registers(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts);
BMSstatus_t get_temperatures(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const float *ntc_voltage, const ID_t id, const Mode_t md, const uint8_t attempts);
void find_min_max_temp(BMS_info *bmsInfo, LTC6811 *slave_array);
void read_humidity(LTC6811 *slave_array, const ID_t id);
void find_max_humidity(BMS_info *bmsInfo, LTC6811 *slave_array);

/* GET STATUS VALUES FUNCTIONS */
void update_status(MCU* master, Broadcast_t nbc, ID_t id, Mode_t md);
BMSstatus_t read_status_registers(MCU* master, LTC6811* slave_array, ID_t id, uint8_t attempts);
BMSstatus_t read_cfgr_register(MCU* master, LTC6811* slave_array, ID_t id, uint8_t attempts);

/* DIAGNOSTIC FUNCTIONS */
BMSstatus_t check_adcs(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts);
BMSstatus_t self_test_all_modes(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts);
BMSstatus_t self_test(MCU *master, LTC6811 *slave_array, const ID_t id, const Mode_t md, const uint8_t st, const uint8_t attempts);
BMSstatus_t safe_clear(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const uint8_t id, const uint8_t attempts);
BMSstatus_t validate_clear(MCU *master, LTC6811 *slave_array, const ID_t id, const uint8_t attempts);
BMSstatus_t check_mux_and_thsd(MCU *master, LTC6811 *slave_array, const ID_t id, uint8_t attempts);
BMSstatus_t check_reference_voltages(MCU *master, LTC6811 *slave_array, const ID_t id, uint8_t attempts);
BMSstatus_t check_open_wire(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, const ID_t id, const Mode_t md, const uint8_t attempts);
BMSstatus_t validate_voltage_measurements(MCU *master, LTC6811 *slave_array, BMS_info *bmsInfo, const ID_t id);
BMSstatus_t diagnose(MCU *master, LTC6811 *slave_array, BMS_info* bmsInfo, const ID_t id, uint8_t attempts);       /* Combines all of the above functions */

/* SAFE WRITE FUNCTIONS */
BMSstatus_t safe_write_cfgr(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *gpio,
		                const uint8_t refon, const uint8_t adcopt, const float uv_limit, const float ov_limit,
						const uint8_t *dcc, const uint8_t dcto, const uint8_t attempts);
BMSstatus_t safe_write_pwm(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *pwm, const uint8_t attempts);
BMSstatus_t safe_write_sctrl(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *sctrl, const uint8_t attempts);

/* WRITE FUNCTIONS */
void write_cfgr(MCU *master, LTC6811 *slave_array, const Broadcast_t nbc, const ID_t id, const uint8_t *gpio_pd,
		            const uint8_t refon, const uint8_t adcopt, const float uv_limit, const float ov_limit,
				    const uint8_t *dcc, const uint8_t dcto);
void write_sctrl(MCU *BMS, const Broadcast_t nbc, const ID_t id, const uint8_t *sctrl);
void write_pwm(MCU *BMS, const Broadcast_t nbc, const ID_t id, const uint8_t *pwm);

/* READ FUNCTIONS */
BMSstatus_t read(MCU *master, LTC6811 *slave_array, const ID_t id, const uint16_t address, const uint8_t attempts);
void process_read_data(LTC6811 *slave, const uint16_t address);

/* UPDATE FUNCTIONS */
uint16_t ADCV(const uint8_t MD,const uint8_t DCP,const uint8_t CH);
uint16_t ADOW(const uint8_t MD,const uint8_t DCP,const uint8_t CH,const uint8_t PUP);
uint16_t CVST(const uint8_t MD,const uint8_t ST);
uint16_t ADOL(const uint8_t MD,const uint8_t DCP);
uint16_t ADAX(const uint8_t MD,const uint8_t CHG);
uint16_t ADAXD(const uint8_t MD,const uint8_t CHG);
uint16_t AXST(const uint8_t MD,const uint8_t ST);
uint16_t ADSTAT(const uint8_t MD,const uint8_t CHST);
uint16_t ADSTATD(const uint8_t MD,const uint8_t CHST);
uint16_t STATST(const uint8_t MD,const uint8_t ST);
uint16_t ADCVAX(const uint8_t MD,const uint8_t DCP);
uint16_t ADCVSC(const uint8_t MD,const uint8_t DCP);

/* HAL FUNCTIONS */
void wake_up(const MCU *master);
void isospi_ready(const MCU *master);
BMSstatus_t read_data(MCU *master, LTC6811 *slave, const ID_t id, const uint16_t address);
BMSstatus_t update(MCU *BMS, const Broadcast_t nbc, const ID_t id, const uint16_t address);
BMSstatus_t update_and_wait(MCU *master, const Broadcast_t nbc, const ID_t id, const uint16_t address, const uint16_t timeout);
BMSstatus_t write_data(MCU *master, const Broadcast_t nbc, const ID_t id, const uint16_t address);
BMSstatus_t poll(MCU *master, const Broadcast_t nbc, const ID_t id, const uint16_t timeout);
BMSstatus_t poll_status(MCU *master, const Broadcast_t nbc, const ID_t id);

/* PEC CALCULATION FUNCTIONS */
void init_PEC15_Table();
uint16_t pec15(const uint8_t *data , const uint8_t len);


#endif /* LTC6811_H_ */

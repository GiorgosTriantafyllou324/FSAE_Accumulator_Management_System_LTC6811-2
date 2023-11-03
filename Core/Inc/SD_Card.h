/*
------------------ CONFIGURATION OF SD CARD ON STM32: ----------------------------

*In .ioc --> Middleware --> FATFS --> user-defined (check)
*Keep SD card SPI speed around 2.25 Mbps
*Copy the files fatfs_sd.c and fatfs_sd.h in the respective src and inc folders
*Open the fatfs_sd.c file and edit the following
 extern SPI_HandleTypeDef 	hspi1;
 #define HSPI_SDCARD		&hspi1
 #define	SD_CS_PORT		GPIOA
 #define SD_CS_PIN			GPIO_PIN_4
*In STM32f1xx_it.c define:
 volatile uint8_t FatFsCnt = 0;
 volatile uint8_t Timer1, Timer2;

 void SDTimer_Handler(void)
 {
   if(Timer1 > 0)
     Timer1--;

   if(Timer2 > 0)
     Timer2--;
 }

 void SysTick_Handler(void)
 {
   // USER CODE BEGIN SysTick_IRQn 0
	  FatFsCnt++;
	  if(FatFsCnt >= 10) {
	    FatFsCnt = 0;
	    SDTimer_Handler(); }

  // USER CODE END SysTick_IRQn 0
  HAL_IncTick();
  // USER CODE BEGIN SysTick_IRQn 1

  // USER CODE END SysTick_IRQn 1
 }

----------------------------- END OF CONFIGURATION -------------------------------
*/



#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_
#include <stdio.h>
#include "fatfs_sd.h"
#include "fatfs.h"
#include "diskio.h"
#include <LTC6811.h>
#include <ACCU_CAN_functions.h>
#include <inttypes.h>

#define AUTOSAVE_CYCLES 50 // number of writes to the SD card before f_sync() is called

/* ----- SD Variables ------ */
typedef struct SD_Card
{
	FATFS   fs;  	    // System's basic object
	FIL     file;
	FRESULT fresult;  	// stores the result of the last SD card operation
	UINT br, bw;  		// File read/write count

	uint16_t unsaved_writes;  // calls f_sync when unsaved_writes == AUTOSAVE_CYCLES
	uint32_t message_num;     // counts the number of messages written
	uint8_t  read_flag;       // Determines if the user wants to read the content of the disk
	/* capacity related */
	FATFS   *pfs;
	DWORD    fre_clust;
	uint32_t total, free_space;

	bool connected; // Reads the SD_Detection pin to see if it is connected
	bool mounted;   // true only if the SD Card has been initialized properly

	uint16_t time_interval; // Stores the time interval after which a new message is written

}SD_Card;

FRESULT SD_Card_init(SD_Card* sdCard, LTC6811* slave_array, Accu_info* accuInfo, BMS_info* bmsInfo, Ivt* ivt, Elcon* elcon, Imd* imd, P23_status *p23status);
void SD_card_write(SD_Card* sdCard);
void SD_card_read(SD_Card* sdCard);


#endif /* INC_SD_CARD_H_ */

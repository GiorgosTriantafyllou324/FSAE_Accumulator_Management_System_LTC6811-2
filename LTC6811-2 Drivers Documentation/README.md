# LTC6811-2 Drivers Documentation
* The architecture of the BMS is a master-slave configuration where the master (STM32F4 MCU) communicates via iso-SPI with 16 slave PCBS (LTC6811-2) connected in parallel. The schematic of the slave PCBs can be found in this folder. Each sensing IC measures 9 cell voltages and has 5 analog inputs for NTC measurements. On the PCB, it can be defined if the 5th measurement will be temperature or humidity measurement (NTC or RH sensor).

The LTC6811-2 libraries can be found in ACCU_Code\Core\Inc & ACCU_Code\Core\Src

LTC6811-board.h / LTC6811-board.c:
    Include the very basic characteristics of the Slave Board that the LTC6811 is part of. New designs require a change in these files

LTC6811.h / LTC6811.c:
    Include all the functions used to interface the sensing IC and most functions to process the data. Can be applied in any project using the STM32 HAL Libraries. 

* In this folder there can also be found diagrams indicating the connection of all the functions in the driver. Each arrow in the diagram points from the callee to the caller.

The functions are called from the ACCU_Code\Core\Src\freertos.c file: 
- The task responsible for the communication of the master with the 16 slaves is the BMS_Communication_Task
- The task responsible for the detection of possible errors from the existing data is the BMS_Main_Task
- The task responsible for the charging process is the Charge_Task. ELCON charger has been utilized
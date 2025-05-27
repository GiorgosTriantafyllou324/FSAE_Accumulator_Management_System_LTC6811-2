# LTC6811-2 Drivers Documentation

The architecture of the BMS is a master-slave configuration where the master (STM32F4 MCU) communicates via iso-SPI with 16 slave PCBS (LTC6811-2) connected in parallel. The schematic of the slave PCBs can be found in this folder. Each sensing IC measures 9 cell voltages and has 5 analog inputs for NTC measurements. On the PCB, it can be defined if the 5th input will be a temperature or humidity measurement (NTC or RH sensor).

The LTC6811-2 libraries can be found in ```ACCU_Code/Core/Inc``` & ```ACCU_Code/Core/Src```

* ```LTC6811-board.h``` / ```LTC6811-board.c```:
    Include the very basic characteristics of the Slave Board that the LTC6811 is part of. New designs require a change in these files

* ```LTC6811.h``` / ```LTC6811.c```:
    Include all the functions used to interface the sensing IC and most functions to process the data. Can be applied in any project that is using the STM32 HAL Libraries. 

Diagrams indicating the connection of all the functions in the driver can also be found in this folder. **Each arrow in the diagram points from the callee to the caller**.

The functions are called from the ```ACCU_Code/Core/Src/freertos.c``` file which is the main file including all the tasks running (Real Time Operating System is used for the scheduling):
- The task responsible for the communication of the master with the 16 slaves is the *BMS_Communication_Task*
- The task responsible for the detection of possible errors from the existing data is the *BMS_Main_Task*
- The task responsible for the charging process is the *Charge_Task*. ELCON is the charger utilized. The system manages automatically both the charging and the cell balancing process.

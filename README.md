# FSAE_Accumulator_Management_System
This project is created to accomodate the needs of a 600V Battery Pack management and communication system for an electric race car participating in the FSAE International Competition.

Its core component is the **Battery Management System (BMS)** that ensures safe charging and discharging of the 288 cells, but other features pertaining the Precharge Process for the inverter DC Link Capacitor, 
data logging to an SD card, communication with the rest of the car through CAN bus and data transmission to a PC through USB.

The code is written for the STM32 MCU family - specifically used STM32F4

FreeRTOS is utilized, so the main file is  ```ACCU_Code\Core\Src\freertos.c```

All these implemented services can be found inside the following paths:
- ACCU_Code\Core\Inc
- ACCU_Code\Core\Src

The aforementioned custom libraries include:

* **LTC6811-2_BMS_Sensing_IC Custom Drivers: Further documentation in the LTC6811-2_Drivers_Documentation folder**

* Charge Process and Current Measurement of the ACCUmulator: ```ACCU_base_lib.h``` / ```ACCU_base_lib.c``` libraries

* USB communication with external laptop using the Json format: ```JsonUSB.h``` / ```JsonUSB.c``` libraries send data to a laptop for a GUI

* SD-Card data storage using the FATFS Drivers: ```SD_Card.h``` / ```SD_Card.c``` libraries store all the data of the accumulator and the car to an SD card on the board

* Communication of the ACCUmulator with the CAN bus protocol: ```ACCU_CAN_functions.h``` / ```ACCU_CAN_functions.c``` libraries manage the CAN bus communication 

* Precharge function for an FSAE car: Can be found in the ```freertos.c``` file
      
* Various other functionalities ...


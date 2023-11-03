/*
 * JsonUsb.h
 *
 *  Created on: Jan 8, 2023
 *      Author: avlac
 */

#ifndef INC_JSONUSB_H_
#define INC_JSONUSB_H_

#include <stdio.h>
#include <LTC6811.h>
#include <ACCU_CAN_functions.h>
#include "usbd_cdc_if.h"
#include "cJSON.h"

void USB_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

USBD_StatusTypeDef voltages_json(LTC6811 *slave_array);
USBD_StatusTypeDef balancing_json(LTC6811 *slave_array);
USBD_StatusTypeDef temperatures_json(LTC6811 *slave_array);
USBD_StatusTypeDef humidities_json(LTC6811 *slave_array);
USBD_StatusTypeDef PECerrors_json(LTC6811 *slave_array);
USBD_StatusTypeDef Accu_json(Accu_info *accuInfo);
USBD_StatusTypeDef Ivt_json(Ivt *ivt);
USBD_StatusTypeDef Elcon_json(Elcon *elcon);


#endif /* INC_JSONUSB_H_ */

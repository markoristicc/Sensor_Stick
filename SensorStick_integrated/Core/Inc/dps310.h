/*
 * DPS310.h
 *
 *  Created on: Jan 26, 2023
 *      Author: danny
 */

#ifndef INC_DPS310_H_
#define INC_DPS310_H_

#include "stm32l4xx_hal.h"
#include "sens_packet.h"

void DPS310_Start (void);

float DPS310_GetPress (sens_pkt_flt* pkt);

#endif /* INC_DPS310_H_ */

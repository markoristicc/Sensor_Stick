/*
 * DPS310.h
 *
 *  Created on: Jan 26, 2023
 *      Author: danny
 */

#ifndef INC_DPS310_H_
#define INC_DPS310_H_

#include "stm32l4xx_hal.h"

void DPS310_Start (void);

void DPS310_GetPress (void);

#endif /* INC_DPS310_H_ */

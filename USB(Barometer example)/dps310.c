/*
 * DPS310.c
 *
 *  Created on: Jan 26, 2023
 *      Author: danny
 */

#include "stm32l4xx_hal.h"
#include "math.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;
#define DPS310_I2C &hi2c2

#define DPS310_ADDR 0x77 << 1

static const int k = 253952;

//maybe these vars could be put in a barometer object struct?
static int val =0;
uint8_t p3=0;
int val2=0;
float trawsc=0;
float prawsc=0;
float pcomp=0;
int iter = 0;
float initial=0;
float altitude = 0;
float dif = 0;
int dc00=0;
int dc10=0;
int dc01=0;
int dc11=0;
int dc20=0;
int dc21=0;
int dc30=0;

void uprintf(char *str)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
}

void getTwosComplement(uint32_t *raw, uint8_t length)
{
	if (*raw & ((uint32_t)1 << (length - 1)))
	{
		*raw -= (uint32_t)1 << length;
	}
}


void read_callibration_data (void)
{
	//READ FROM reg 0x10 to 0x21
	uint8_t buffer[18]={0};
	HAL_I2C_Mem_Read(&hi2c2, DPS310_ADDR, 0x10, 1, buffer, 18, 1000);
	uint32_t c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
	getTwosComplement(&c00, 20);
	dc00 = (int) c00;

	uint32_t c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
	getTwosComplement(&c10, 20);
	dc10 = (int) c10;

	uint32_t c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
	getTwosComplement(&c01, 16);
	dc01 = (int) c01;

	uint32_t c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
	getTwosComplement(&c11, 16);
	dc11 = (int) c11;


	uint32_t c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
	getTwosComplement(&c20, 16);
	dc20 = (int) c20;


	uint32_t c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
	getTwosComplement(&c21, 16);
	dc21 = (int) c21;


	uint32_t c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
	getTwosComplement(&c30, 16);
	dc30 = (int) c30;

	//WRITE TO REG 0x06
	uint8_t prscfg[1];
	prscfg[0] = 0x04;
	HAL_I2C_Mem_Write(&hi2c2, 0x77 << 1, 0x06, 1, prscfg, 1, 1000);


	//WRITE TO REG 0x07
	uint8_t tmpcfg[1];
	tmpcfg[0] = 0x04;
	HAL_I2C_Mem_Write(&hi2c2, 0x77 << 1, 0x07, 1, tmpcfg, 1, 1000);


	//WRITE TO REG 0x09
	uint8_t cfgreg[1];
	cfgreg[0] = 0x3C;
	HAL_I2C_Mem_Write(&hi2c2, 0x77 << 1, 0x09, 1, cfgreg, 1, 1000);

	//WRITE 0x07 TO REG 0x08
	uint8_t meascfg[1];
	meascfg[0] = 0x07;
	HAL_I2C_Mem_Write(&hi2c2, 0x77 << 1, 0x08, 1, meascfg, 1, 1000);

	//Read REG 0x0A(interrupt status)
	uint8_t rupts[1]={0};
	HAL_I2C_Mem_Read(&hi2c2, 0x77 << 1, 0x0A, 1, rupts, 1, 1000);
}

float DPS310_GetPress (void)
{
	int val2 = 0;

	//READ REG 0x03 TO REG 0x05
	uint8_t tmp[3]={0};
	HAL_I2C_Mem_Read(&hi2c2, 0x77 << 1, 0x03, 1, tmp, 3, 1000);
	uint32_t tmpraw =((tmp[0] << 16) |(tmp[1] << 8) | (tmp[2]));
	getTwosComplement(&tmpraw, 24);
	int traw = (int) tmpraw;
	val=traw;

	//READ REG 0x00 TO REG 0x02
	uint8_t prs[3]={0};
	HAL_I2C_Mem_Read(&hi2c2, 0x77 << 1, 0x00, 1, prs, 3, 1000);
	uint32_t prsraw = prs[0] << 16 | prs[1] << 8 | prs[2];
	getTwosComplement(&prsraw, 24);
	int praw = (int) prsraw;
	val2=praw;

	//CALC PRESSURE
	prawsc = (float) val2/k;
	trawsc = (float) val/k;
	pcomp = dc00+prawsc*(dc10+prawsc*(dc20+prawsc*dc30))+trawsc*dc01+trawsc*prawsc*(dc11+prawsc*dc21);
	pcomp = pcomp/100;
	altitude = (float) 44330 * (1-pow(pcomp/1015,1/5.255));
	if (iter <= 1)
	{
		initial = altitude;
	}
	else
	{
		dif = altitude - initial;
	}
	iter++;

	//Read REG 0x0A(interrupt status)
	uint8_t rupts[1]={0};
	HAL_I2C_Mem_Read(&hi2c2, 0x77 << 1, 0x0A, 1, rupts, 1, 1000);

	return dif;
}

void DPS310_Start (void)
{
	read_callibration_data();
}

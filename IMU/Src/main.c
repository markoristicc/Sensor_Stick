/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bmi08x.h"
#include "common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*********************************************************************/
/* global variables */
/*********************************************************************/
unsigned char data_sync_int = false;
unsigned char accel_data_ready = false;
unsigned char gyro_data_ready = false;

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! bmi08x int config */
struct bmi08x_int_cfg int_config;

/*Data Sync configuration object*/
struct bmi08x_data_sync_cfg sync_cfg;

/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;

/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;

/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;

/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/*!
 * @brief    This internal API is used to initialize the bmi08x sensor
 */
static int8_t init_bmi08x_sync(void);
static int8_t init_bmi08x_get_data(void);

/*!
 * @brief    BMI08x data sync. interrupt callback
 */
void bmi08x_data_sync_int();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
}
/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08x_get_data(void) {
	int8_t rslt;

	rslt = bmi08a_init(&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_init", rslt);

	if (rslt == BMI08X_OK) {
		rslt = bmi08g_init(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_init", rslt);
	}

//    if (rslt == BMI08X_OK)
//    {
//        printf("Uploading config file !\n");
//        rslt = bmi08a_load_config_file(&bmi08xdev);
//        bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);
//    }

	if (rslt == BMI08X_OK) {
		bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;

		if (bmi08xdev.variant == BMI085_VARIANT) {
			bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
		} else if (bmi08xdev.variant == BMI088_VARIANT) {
			bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
		}

		bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
		bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

		rslt = bmi08a_set_power_mode(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

		rslt = bmi08a_set_meas_conf(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);

		bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
		bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
		bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
		bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

		rslt = bmi08g_set_power_mode(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);

		rslt = bmi08g_set_meas_conf(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_meas_conf", rslt);
	}

	return rslt;

}

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor with default.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08x_sync(void) {
	int8_t rslt;

	/* Initialize bmi08a */
	rslt = bmi08a_init(&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_init", rslt);

	/* Initialize bmi08g */
	rslt = bmi08g_init(&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08g_init", rslt);

	if (rslt == BMI08X_OK) {
		printf("BMI08x initialization success!\n");
		printf("Accel chip ID - 0x%x\n", bmi08xdev.accel_chip_id);
		printf("Gyro chip ID - 0x%x\n", bmi08xdev.gyro_chip_id);

		/* Reset the accelerometer */
		rslt = bmi08a_soft_reset(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_soft_reset", rslt);

		/* Read/write length */
		bmi08xdev.read_write_len = 32;

		printf("Uploading BMI08X data synchronization feature config !\n");
		rslt = bmi08a_load_config_file(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);

		/* Set accel power mode */
		bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
		rslt = bmi08a_set_power_mode(&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

		if (rslt == BMI08X_OK) {
			bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
			rslt = bmi08g_set_power_mode(&bmi08xdev);
			bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);
		}

		if ((bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_ACTIVE)
				&& (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_NORMAL)) {
			/* API uploads the bmi08x config file onto the device */
			if (rslt == BMI08X_OK) {
				/* Assign accel range setting */
				if (bmi08xdev.variant == BMI085_VARIANT) {
					bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
				} else if (bmi08xdev.variant == BMI088_VARIANT) {
					bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
				}

				/* Assign gyro range setting */
				bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;

				/* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
				sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_400HZ;

				rslt = bmi08a_configure_data_synchronization(sync_cfg,
						&bmi08xdev);
				bmi08x_error_codes_print_result(
						"bmi08a_configure_data_synchronization", rslt);
			}

			if (rslt == BMI08X_OK) {
				printf("BMI08x data synchronization feature configured !\n\n");
			} else {
				printf(
						"BMI08x data synchronization feature configuration failure!\n\n");
			}
		}
	}

	return rslt;
}
/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t enable_bmi08x_interrupt() {
	int8_t rslt;
	uint8_t data = 0;

	/* Set accel interrupt pin configuration */
	accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
	accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
	accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	/* Enable accel data ready interrupt channel */
	rslt = bmi08a_set_int_config(
			(const struct bmi08x_accel_int_channel_cfg*) &accel_int_config,
			&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

	if (rslt == BMI08X_OK) {
		/* Set gyro interrupt pin configuration */
		gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
		gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
		gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

		/* Enable gyro data ready interrupt channel */
		rslt = bmi08g_set_int_config(
				(const struct bmi08x_gyro_int_channel_cfg*) &gyro_int_config,
				&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);

		rslt = bmi08g_get_regs(BMI08X_REG_GYRO_INT3_INT4_IO_MAP, &data, 1,
				&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_get_regs", rslt);
	}

	return rslt;
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t disable_bmi08x_interrupt() {
	int8_t rslt;

	/* Set accel interrupt pin configuration */
	accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
	accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
	accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

	/* Disable accel data ready interrupt channel */
	rslt = bmi08a_set_int_config(
			(const struct bmi08x_accel_int_channel_cfg*) &accel_int_config,
			&bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

	if (rslt == BMI08X_OK) {
		/* Set gyro interrupt pin configuration */
		gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
		gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
		gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

		/* Disable gyro data ready interrupt channel */
		rslt = bmi08g_set_int_config(
				(const struct bmi08x_gyro_int_channel_cfg*) &gyro_int_config,
				&bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
	}

	return rslt;
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @return int8_t
 *
 */
static int8_t enable_bmi08x_data_synchronization_interrupt() {
	int8_t rslt = BMI08X_OK;

	/* Set accel interrupt pin configuration */
	/* Configure host data ready interrupt */
	int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_2;
	int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin =
	BMI08X_ENABLE;

	/* Configure Accel syncronization input interrupt pin */
	int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_1;
	int_config.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
	int_config.accel_int_config_2.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin =
	BMI08X_ENABLE;

	/* Set gyro interrupt pin configuration */
	int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_4;
	int_config.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;

	int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_3;
	int_config.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin =
	BMI08X_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode =
	BMI08X_INT_MODE_PUSH_PULL;

	/* Enable synchronization interrupt pin */
	rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config", rslt);

	return rslt;
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return int8_t
 *
 */
static int8_t disable_bmi08x_data_synchronization_interrupt() {
	int8_t rslt;

	/*turn off the sync feature*/
	sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;

	rslt = bmi08a_configure_data_synchronization(sync_cfg, &bmi08xdev);
	bmi08x_error_codes_print_result("bmi08a_configure_data_synchronization",
			rslt);

	/* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
	/* configure synchronization interrupt pins */
	if (rslt == BMI08X_OK) {
		/* Set accel interrupt pin configuration */
		/* Configure host data ready interrupt */
#if defined(MCU_APP20)
	        int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
	    #elif defined(MCU_APP30)
	        int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_2;
	    #endif
		int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
		int_config.accel_int_config_1.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;
		int_config.accel_int_config_1.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.accel_int_config_1.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;

		/* Configure Accel synchronization input interrupt pin */
#if defined(MCU_APP20)
	        int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
	    #elif defined(MCU_APP30)
	        int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_1;
	    #endif
		int_config.accel_int_config_2.int_type = BMI08X_ACCEL_INT_SYNC_DATA_RDY;
		int_config.accel_int_config_2.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;
		int_config.accel_int_config_2.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.accel_int_config_2.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;

		/* Set gyro interrupt pin configuration */
#if defined(MCU_APP20)
	        int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
	    #elif defined(MCU_APP30)
	        int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_4;
	    #endif
		int_config.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
		int_config.gyro_int_config_1.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_1.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;
		int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;

#if defined(MCU_APP20)
	        int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
	    #elif defined(MCU_APP30)
	        int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_3;
	    #endif
		int_config.gyro_int_config_2.int_type = BMI08X_GYRO_INT_DATA_RDY;
		int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin =
		BMI08X_DISABLE;
		int_config.gyro_int_config_2.int_pin_cfg.lvl =
		BMI08X_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_2.int_pin_cfg.output_mode =
		BMI08X_INT_MODE_PUSH_PULL;

		/* Disable synchronization interrupt pin */
		rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config",
				rslt);
	}

	return rslt;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	int8_t rslt;
	float x = 0.0, y = 0.0, z = 0.0;

	bmi085_handle_t bmi085_handle = {
			.spi_handle = &hspi1,
			.nssg_port = GPIOC,
			.nssa_port = GPIOB,
			.nssa_pin = BMI085a_NSS_Pin,
			.nssg_pin = BMI085g_NSS_Pin,
			.ps_pin = BMI085_PS_Pin,
			.ps_port = GPIOC,
			.timer_ptr= &htim2 };
	/* Interface given as parameter
	 *           For I2C : BMI08X_I2C_INTF
	 *           For SPI : BMI08X_SPI_INTF
	 * Sensor variant given as parameter
	 *          For BMI085 : BMI085_VARIANT
	 *          For BMI088 : BMI088_VARIANT
	 */
//	HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);

	init_handle(&bmi085_handle);

	rslt = bmi08x_interface_init(&bmi08xdev, BMI08X_SPI_INTF, BMI085_VARIANT);
	bmi08x_error_codes_print_result("bmi08x_interface_init", rslt);

	/* Initialize the sensors */
	init_bmi08x_get_data();

	/* Enable data ready interrupts */
//	enable_bmi08x_data_synchronization_interrupt();
//	uint32_t start_time = HAL_GetTick();
	enable_bmi08x_interrupt();

	printf("Accel data range : 16G for BMI085 and 24G for BMI088\n");
	printf("Gyro data range : 250 dps for BMI085 and BMI088\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* Code for Data Sync, inquiring if necessary! */
//		if (data_sync_int == true) {
//			data_sync_int = false;
//
//			rslt = bmi08a_get_synchronized_data(&bmi08x_accel, &bmi08x_gyro,
//					&bmi08xdev);
//			bmi08x_error_codes_print_result("bmi08a_get_synchronized_data",
//					rslt);
//
//			printf(
//					"\nACCEL  Acc_Raw_X : %d    Acc_Raw_Y : %d    Acc_Raw_Z : %d   ;    GYRO  Gyr_Raw_X : %d   Gyr_Raw_Y : %d   Gyr_Raw_Z : %d   Timestamp : %lu\n",
//					bmi08x_accel.x, bmi08x_accel.y, bmi08x_accel.z,
//					bmi08x_gyro.x, bmi08x_gyro.y, bmi08x_gyro.z,
//					HAL_GetTick() - start_time);
//
//			if (bmi08xdev.variant == BMI085_VARIANT) {
//				/* Converting lsb to meter per second squared for 16 bit accelerometer at 16G range. */
//				x = lsb_to_mps2(bmi08x_accel.x, 16, 16);
//				y = lsb_to_mps2(bmi08x_accel.y, 16, 16);
//				z = lsb_to_mps2(bmi08x_accel.z, 16, 16);
//			} else if (bmi08xdev.variant == BMI088_VARIANT) {
//				/* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
//				x = lsb_to_mps2(bmi08x_accel.x, 24, 16);
//				y = lsb_to_mps2(bmi08x_accel.y, 24, 16);
//				z = lsb_to_mps2(bmi08x_accel.z, 24, 16);
//			}
//
//			/* Print the data in m/s2. */
//			printf(
//					"Acc_ms2_X = %4.2f   Acc_ms2_Y = %4.2f   Acc_ms2_Z = %4.2f  ",
//					x, y, z);
//
//			/* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
//			x = lsb_to_dps(bmi08x_gyro.x, 250, 16);
//			y = lsb_to_dps(bmi08x_gyro.y, 250, 16);
//			z = lsb_to_dps(bmi08x_gyro.z, 250, 16);
//
//			/* Print the data in dps. */
//			printf(
//					"\t  Gyr_DPS_X = %4.2f   Gyr_DPS_Y = %4.2f   Gyr_DPS_Z = %4.2f\n",
//					x, y, z);

//		}
		if(accel_data_ready == true && (bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_ACTIVE)){
			accel_data_ready = false;

			rslt = bmi08a_get_data(&bmi08x_accel, &bmi08xdev);
			bmi08x_error_codes_print_result("bmi08a_get_data", rslt);

			/* Converting lsb to meter per second squared for 16 bit accelerometer at 16G range. */
			x = lsb_to_mps2(bmi08x_accel.x, 16, 16);
			y = lsb_to_mps2(bmi08x_accel.y, 16, 16);
			z = lsb_to_mps2(bmi08x_accel.z, 16, 16);
			/* Print the data in m/s2. */
			printf(
					"\t  Acc_ms2_X = %4.2f,  Acc_ms2_Y = %4.2f,  Acc_ms2_Z = %4.2f\n\r",
					x, y, z);
		}
//		if(gyro_data_ready == true && (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_NORMAL)){
//			gyro_data_ready = false;
//
//			rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08xdev);
//									bmi08x_error_codes_print_result("bmi08g_get_data",
//											rslt);
//
//			/* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
//			x = lsb_to_dps(bmi08x_gyro.x, 250, 16);
//			y = lsb_to_dps(bmi08x_gyro.y, 250, 16);
//			z = lsb_to_dps(bmi08x_gyro.z, 250, 16);
//
//			/* Print the data in dps. */
//			printf(
//					"\t  Gyr_DPS_X = %4.2f  , Gyr_DPS_Y = %4.2f  , Gyr_DPS_Z = %4.2f\n\r",
//					x, y, z);
//		}
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BMI085g_NSS_Pin|BMI085_PS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMI085a_NSS_GPIO_Port, BMI085a_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BMI085_Accel_DR_INT_Pin */
  GPIO_InitStruct.Pin = BMI085_Accel_DR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMI085_Accel_DR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMI085_Gyro_DR_INT_Pin */
  GPIO_InitStruct.Pin = BMI085_Gyro_DR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMI085_Gyro_DR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin BMI085a_NSS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|BMI085a_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI085g_NSS_Pin BMI085_PS_Pin */
  GPIO_InitStruct.Pin = BMI085g_NSS_Pin|BMI085_PS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* BMI08x data sync. interrupt callback */
void EXTI4_IRQHandler()
{
//    data_sync_int = true;
	gyro_data_ready = true;

	HAL_GPIO_EXTI_IRQHandler(BMI085_Gyro_DR_INT_Pin);
}
void EXTI0_IRQHandler()
{
//    data_sync_int = true;
	accel_data_ready = true;

	HAL_GPIO_EXTI_IRQHandler(BMI085_Accel_DR_INT_Pin);
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    float half_scale = ((1 << bit_width) / 2.0f);

    gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);

    return gravity;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI08X_GYRO_RANGE_2000_DPS)) * (val);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
__disable_irq();
while (1) {
}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

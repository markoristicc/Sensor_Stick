/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "usbd_cdc_if.h"

#include "sens_packet.h" //sens packet in float form

#include "iis2mdc_reg.h" //magnetometer
#include "bmi08x_defs.h"
#include "bmi08x.h" 	// IMU
#include "dps310.h" // barometer
#include <GNSS.h> //GPS

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union {  ///magnetometer
	int16_t i16bit[3];
} axis3bit16_t;


typedef struct {
	uint8_t dev_id; //dev_id =0b1000101
	uint8_t stale_bits; // 0bXX[imu_acc][imu_gyr][mag][airspeed][bar][gps]
	uint32_t timestamp;
	uint16_t imu_acc_x;
	uint16_t imu_acc_y;
	uint16_t imu_acc_z;
	uint16_t imu_gyr_x;
	uint16_t imu_gyr_y;
	uint16_t imu_gyr_z;
	uint16_t mag_x;
	uint16_t mag_y;
	uint16_t mag_z;
//	axis3bit16_t imu_acc;
//	axis3bit16_t imu_gyr;
//	axis3bit16_t mag;
	uint16_t airspeed;
	uint32_t bar_press;
	uint32_t bar_temp;
	uint32_t gps_lat;
	uint32_t gps_long;
} sens_pkt;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Magnetometer ----------------------------------*/

#define MAG_BUS hi2c1

#define BOOT_TIME       20 //ms

#define SELF_TEST_SAMPLES 50

#define ST_MIN_POS    15.0f
#define ST_MAX_POS    500.0f
/* END of Magnetometer ----------------------------------*/

/* Barometer ----------------------------------*/

#define BAR_BUS hi2c2

/* END of Barometer ----------------------------------*/

#define DEBUG_MODE false

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)  //IMU

#define BMI08X_READ_WRITE_LEN  UINT8_C(46)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID      UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID      UINT16_C(0x66)

#define DPS_TO_RPS		(0.01745329251f)

#define IMU_ACC_NEW 0b1<<5
#define IMU_GYR_NEW 0b1<<4
#define MAG_NEW 0b1<<3
#define AIR_NEW 0b1<<2
#define BAR_NEW 0b1<<1
#define GPS_NEW 0b1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

unsigned char imu_read = false;
unsigned char mag_read = false;
unsigned char dp_read = false;
unsigned char bar_read = false;
unsigned char gps_read = false;

unsigned char init_done = false;
//packet header init

sens_pkt_flt gpkt; //needs to be global so timer callback can access

//used to trigger a gps read at 1Hz
int gps_counter = 0;

/*IMU ---global variables-----------------------------------------*/

volatile uint8_t rx_rdy = 1;
volatile uint8_t tx_rdy = 1;

volatile uint8_t accel_rdy = 0;
volatile uint8_t gyro_rdy = 0;

uint8_t * current_reg_data_ptr;
uint32_t current__len = 0;
uint16_t dev_addr;

uint8_t error_status = 0;
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

///*! Variable that holds the I2C device address or SPI chip selection for accel */
uint16_t acc_dev_add;
//
///*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint16_t gyro_dev_add;


/* END of IMU ----------------------------------------------------------------*/

/* Magnetometer -------------------------------------------------------------*/
static axis3bit16_t test_data_raw_magnetic[SELF_TEST_SAMPLES];
static float test_magnetic_mG[SELF_TEST_SAMPLES][3];
static uint8_t whoamI, rst, mag_drdy;
static uint8_t tx_buffer[1000];

static int16_t data_raw_temperature;
static int16_t data_raw_magnetic[3];
static float temperature_degC;
static float magnetic_mG[3];

/* End of Magnetometer -------------------------------------------------------------*/


/* GPS ----------------------------------------------------------*/
uint8_t Rxdata[750];
char Txdata[750];
char GPS_Payyload[100];
uint8_t Flag = 0;
static int Msgindex;
char *ptr;

float time, Latitude, Longitude;
int Hours, Min, Sec;

/*  End of GPS -------------------------------------------------------------*/


/* DP Sensor ---------------------------------------------------*/
static const uint8_t DP_ADDR = 0x28<<1; // Use 8-bit address

uint8_t* buf;

/* End of DP Sensor --------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* IMU -----------------------------------------------------------------------*/

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08X_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08X_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us    : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void bmi08x_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bma      : Structure instance of bmi08x_dev
 *  @param[in] intf     : Interface selection parameter
 *                          For I2C : BMI08X_I2C_INTF
 *                          For SPI : BMI08X_SPI_INTF
 *  @param[in] variant  : Sensor variant parameter
 *                          For BMI085 : BMI085_VARIANT
 *                          For BMI088 : BMI088_VARIANT
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bma, uint8_t intf,
		uint8_t variant);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt);

//void bmi08x_coines_deinit(void);

/*********************************************************************/
/* static function declarations */
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
 * @brief    This internal API is used to initialize the bmi08x sensor with default
 */
static int8_t init_bmi08x(void);

/* END of IMU ----------------------------------------------------------------*/

/*  Magnetometer -------------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
/*  End of Magnetometer ------------------------------------------------------*/


/*  GPS ------------------------------------------------------------------*/

void get_location(sens_pkt_flt* pkt);
void Format_data(float Time, float Lat, float Long);

/* End of GPS ---------------------------------------------------------------*/

/* DP Sensor ---------------------------------------------------*/

void DPMeasurement(sens_pkt_flt* pkt);
/* End of DP Sensor --------------------------------------------*/

static int packet_transmit(sens_pkt_flt* pkt);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* IMU ----------------------------------------------------also gnuc stuff also dp sensor-------------------*/

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
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
static int8_t init_bmi08x(void) {
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
		bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_125_DPS;
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


/* END of IMU ----------------------------------------------------------------*/


/*  Magnetometer -------------------------------------------------------------*/
static inline float ABSF(float _x){
	return (_x < 0.0f) ? -(_x) : _x;
}

static int iis2mdc_flush_samples(stmdev_ctx_t *dev_ctx){
	uint8_t reg;
	axis3bit16_t dummy;
	int samples = 0;

	iis2mdc_mag_data_ready_get(dev_ctx, &reg);

	if(reg){
		iis2mdc_magnetic_raw_get(dev_ctx, dummy.i16bit);
		samples++;
	}

	return samples;
}

static int test_self_test_iis2mdc(stmdev_ctx_t *dev_ctx)
{
  uint8_t reg;
  float media[3] = { 0.0f, 0.0f, 0.0f };
  float mediast[3] = { 0.0f, 0.0f, 0.0f };
  uint8_t match[3] = { 0, 0, 0 };
  uint8_t j = 0;
  uint16_t i = 0;
  uint8_t k = 0;
  uint8_t axis;
  int result = 0;
  /* Restore default configuration */
  iis2mdc_reset_set(dev_ctx, PROPERTY_ENABLE);

  do {
    iis2mdc_reset_get(dev_ctx, &rst);
  } while (rst);

  iis2mdc_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
  /* Set / Reset sensor mode */
  iis2mdc_set_rst_mode_set(dev_ctx, IIS2MDC_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation */
  iis2mdc_offset_temp_comp_set(dev_ctx, PROPERTY_ENABLE);
  /* Set device in continuous mode */
  iis2mdc_operating_mode_set(dev_ctx, IIS2MDC_CONTINUOUS_MODE);
  /* Set Output Data Rate to 100 Hz */
  iis2mdc_data_rate_set(dev_ctx, IIS2MDC_ODR_100Hz);
  /* Power up and wait for 20 ms for stable output */
  platform_delay(20);
  /* Flush old samples */
  iis2mdc_flush_samples(dev_ctx);

  do {
    iis2mdc_mag_data_ready_get(dev_ctx, &reg);

    if (reg) {
      /* Read magnetic field data */
      memset(test_data_raw_magnetic[i].i16bit, 0x00, 3 * sizeof(int16_t));
      iis2mdc_magnetic_raw_get(dev_ctx, test_data_raw_magnetic[i].i16bit);

      for (axis = 0; axis < 3; axis++)
        test_magnetic_mG[i][axis] =
          iis2mdc_from_lsb_to_mgauss(test_data_raw_magnetic[i].i16bit[axis]);

      i++;
    }
  } while (i < SELF_TEST_SAMPLES);

  for (k = 0; k < 3; k++) {
    for (j = 0; j < SELF_TEST_SAMPLES; j++) {
      media[k] += test_magnetic_mG[j][k];
    }

    media[k] = (media[k] / j);
  }

  /* Enable self test mode */
  iis2mdc_self_test_set(dev_ctx, PROPERTY_ENABLE);
  platform_delay(60);
  i = 0;
  /* Flush old samples */
  iis2mdc_flush_samples(dev_ctx);

  do {
    iis2mdc_mag_data_ready_get(dev_ctx, &reg);

    if (reg) {
      /* Read accelerometer data */
      memset(test_data_raw_magnetic[i].i16bit, 0x00, 3 * sizeof(int16_t));
      iis2mdc_magnetic_raw_get(dev_ctx, test_data_raw_magnetic[i].i16bit);

      for (axis = 0; axis < 3; axis++)
        test_magnetic_mG[i][axis] =
          iis2mdc_from_lsb_to_mgauss(test_data_raw_magnetic[i].i16bit[axis]);

      i++;
    }
  } while (i < SELF_TEST_SAMPLES);

  for (k = 0; k < 3; k++) {
    for (j = 0; j < SELF_TEST_SAMPLES; j++) {
      mediast[k] += test_magnetic_mG[j][k];
    }

    mediast[k] = (mediast[k] / j);
  }

  /* Check for all axis self test value range */
  for (k = 0; k < 3; k++) {
    if ((ABSF(mediast[k] - media[k]) >= ST_MIN_POS) &&
        (ABSF(mediast[k] - media[k]) <= ST_MAX_POS)) {
      match[k] = 1;
      result += 1;
    }

    sprintf((char *)tx_buffer, "%d: |%f| <= |%f| <= |%f| %s\r\n", k, ST_MIN_POS, ABSF(mediast[k] - media[k]), ST_MAX_POS, match[k] == 1 ? "PASSED" : "FAILED");
    tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
  }

  /* Disable self test mode */
  iis2mdc_operating_mode_set(dev_ctx, IIS2MDC_POWER_DOWN);
  iis2mdc_self_test_set(dev_ctx, PROPERTY_DISABLE);
  return result == 3 ? 0 : -1;
}

static int mag_init_routine(stmdev_ctx_t *dev_ctx){
		platform_delay(BOOT_TIME);

	  /* Check device id */

	  iis2mdc_device_id_get(dev_ctx, &whoamI);
	  sprintf((char*) tx_buffer, "whoamI: %d\r\n", whoamI);
	  tx_com(tx_buffer, strlen((char const *)tx_buffer));

	  if (whoamI != IIS2MDC_ID) {
		  sprintf((char*) tx_buffer, "whoamI failed\r\n");
		  tx_com(tx_buffer, strlen((char const *)tx_buffer));
	  }

	  test_self_test_iis2mdc(dev_ctx); ///Magnetometer


	  do { //reset sensor state
		  iis2mdc_reset_get(dev_ctx, &rst);
	  } while (rst);

		/* Enable Block Data Update */
		iis2mdc_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
		/* Set Output Data Rate */
		iis2mdc_data_rate_set(dev_ctx, IIS2MDC_ODR_10Hz); //can change this as needed
		/* Set / Reset sensor mode */
		iis2mdc_set_rst_mode_set(dev_ctx, IIS2MDC_SENS_OFF_CANC_EVERY_ODR);
		/* Enable temperature compensation */
		iis2mdc_offset_temp_comp_set(dev_ctx, PROPERTY_ENABLE);
		/* Set device in continuous mode */
		iis2mdc_operating_mode_set(dev_ctx, IIS2MDC_CONTINUOUS_MODE);

	return 0;

}
/*  End of Magnetometer -------------------------------------------------------------*/



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

  gpkt.dev_id = 0b1000101;
  gpkt.stale_bits = 0;

  //packet data init
  gpkt.timestamp = 0;
  //  for(int i = 0; i < 3; i++){
  //	  pkt.imu_acc.i16bit[i] = 0;
  //	  pkt.imu_gyr.i16bit[i] = 0;
  //	  pkt.mag.i16bit[i] = 0;
  //  }
  gpkt.imu_acc_x = 0;
  gpkt.imu_acc_y = 0;
  gpkt.imu_acc_z = 0;
  gpkt.imu_gyr_x = 0;
  gpkt.imu_gyr_y = 0;
  gpkt.imu_gyr_z = 0;
  gpkt.mag_x = 0;
  gpkt.mag_y = 0;
  gpkt.mag_z = 0;
  gpkt.airspeed = 0;
  gpkt.differential_pressure = 0;
  gpkt.bar_press = 0;
  gpkt.bar_temp = 0;
  gpkt.gps_lat = 0;
  gpkt.gps_long = 0;

  GNSS_StateHandle GNSS_Handle;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //mag device handlers
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &MAG_BUS;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */



  /* IMU main variables ----------------------------------------------------*/
  	int8_t rslt;
  	float x = 0.0, y = 0.0, z = 0.0;
  	uint8_t imu_status = 0;

	struct bmi08x_err_reg error;
		/* Interface given as parameter
		 *           For I2C : BMI08X_I2C_INTF
		 *           For SPI : BMI08X_SPI_INTF
		 * Sensor variant given as parameter
		 *          For BMI085 : BMI085_VARIANT
		 *          For BMI088 : BMI088_VARIANT
		 */

	HAL_TIM_Base_Start(&htim2);
	rslt = bmi08x_interface_init(&bmi08xdev, BMI08X_SPI_INTF, BMI088_VARIANT);
	bmi08x_error_codes_print_result("bmi08x_interface_init", rslt);


	if (rslt == BMI08X_OK) {

		rslt = init_bmi08x();
		bmi08x_error_codes_print_result("init_bmi08x", rslt);
		printf("Initialized :3 \r\n");

//		/* Enable data ready interrupts */
		if (rslt == BMI08X_OK) {
			rslt = enable_bmi08x_interrupt();
			bmi08x_error_codes_print_result("enable_bmi08x_interrupt", rslt);
		}

		if (rslt == BMI08X_OK) {
			printf(
					"ODR : %d  BW : %d   RANGE : %d \r\n",
					bmi08xdev.accel_cfg.odr, bmi08xdev.accel_cfg.bw, bmi08xdev.accel_cfg.range);
			rslt = bmi08g_get_meas_conf(&bmi08xdev);
			bmi08x_error_codes_print_result("bmi08g_get_meas_conf", rslt);
			printf(
					"ODR : %d  BW : %d   RANGE : %d \r\n",
					bmi08xdev.accel_cfg.odr, bmi08xdev.accel_cfg.bw, bmi08xdev.accel_cfg.range);
		}
	}
  /* END OF IMU ----------------------------------------------------*/

  /*  Magnetometer -------------------------------------------------------*/

	mag_init_routine(&dev_ctx);

  /*  End of Magnetometer -------------------------------------------------------------*/

  /*  Barometer -------------------------------------------------------*/

  DPS310_Start();

  /*  End of Barometer -------------------------------------------------------------*/

  /* GPS -------------------------------------------------------------*/

  GNSS_Init(&GNSS_Handle, &huart3);
  GNSS_LoadConfig(&GNSS_Handle);
  HAL_Delay(250);
  GNSS_SetMode(&GNSS_Handle, 5); //Airborne 1g mode
  GNSS_GetUniqID(&GNSS_Handle);
  GNSS_ParseBuffer(&GNSS_Handle);
	printf("Unique ID: %04X %04X %04X %04X %04X %04X \n\r",
			GNSS_Handle.uniqueID[0], GNSS_Handle.uniqueID[1],
			GNSS_Handle.uniqueID[2], GNSS_Handle.uniqueID[3],
			GNSS_Handle.uniqueID[4], GNSS_Handle.uniqueID[5]);


  /* End of GPS -----------------------------------------------------------*/


  /*  DP Sensor -------------------------------------------------------*/


  /*  End of DP Sensor -------------------------------------------------------*/

  init_done = true;
  HAL_TIM_Base_Start_IT(&htim15);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	gpkt.timestamp = HAL_GetTick();
	//printf("Time: %d ms \r\n", (int) HAL_GetTick());

	/* IMU while loop -------------------needs to be functionalized-----------------------------------------*/

	if (accel_rdy) {
		rslt = bmi08a_get_data(&bmi08x_accel, &bmi08xdev);
		bmi08x_error_codes_print_result("bmi08a_get_data", rslt);
		accel_rdy = 0;
		if(DEBUG_MODE){
			printf(
					"ACCEL Acc_Raw_X : %d  Acc_Raw_Y : %d   Acc_Raw_Z : %d ",
					 bmi08x_accel.x, bmi08x_accel.y,
					bmi08x_accel.z);
		}
		gpkt.stale_bits = gpkt.stale_bits | IMU_ACC_NEW;
//		pkt.imu_acc_x= bmi08x_accel.x;
//		pkt.imu_acc_y = bmi08x_accel.y;
//		pkt.imu_acc_z = bmi08x_accel.z;


		if (bmi08xdev.variant == BMI085_VARIANT) {
			/* Converting lsb to meter per second squared for 16 bit accelerometer at 16G range. */
			x = lsb_to_mps2(bmi08x_accel.x, 16, 16);
			y = lsb_to_mps2(bmi08x_accel.y, 16, 16);
			z = lsb_to_mps2(bmi08x_accel.z, 16, 16);
		} else if (bmi08xdev.variant == BMI088_VARIANT) {
			/* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
			x = lsb_to_mps2(bmi08x_accel.x, 24, 16);
			y = lsb_to_mps2(bmi08x_accel.y, 24, 16);
			z = lsb_to_mps2(bmi08x_accel.z, 24, 16);
		}

		if(DEBUG_MODE){
		/* Print the data in m/s2. */
			printf(
					"\t  Acc_ms2_X = %4.2f,  Acc_ms2_Y = %4.2f,  Acc_ms2_Z = %4.2f\n\r",
					x, y, z);
		}
		gpkt.imu_acc_x= x;
		gpkt.imu_acc_y = y;
		gpkt.imu_acc_z = z;
	}

	if (gyro_rdy) {
		rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08xdev);
		bmi08x_error_codes_print_result("bmi08g_get_data",
				rslt);
		gyro_rdy = 0;
		gpkt.stale_bits = gpkt.stale_bits | IMU_GYR_NEW;
//		pkt.imu_gyr_x = bmi08x_gyro.x;
//		pkt.imu_gyr_y = bmi08x_gyro.y;
//		pkt.imu_gyr_z = bmi08x_gyro.z;

		if(DEBUG_MODE){
			printf(
					"GYRO  Gyr_Raw_X : %d   Gyr_Raw_Y : %d   Gyr_Raw_Z : %d   ",
					 bmi08x_gyro.x, bmi08x_gyro.y,
					bmi08x_gyro.z);
		}
		/* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
		x = lsb_to_dps(bmi08x_gyro.x, 250, 16) * DPS_TO_RPS;
		y = lsb_to_dps(bmi08x_gyro.y, 250, 16) * DPS_TO_RPS;
		z = lsb_to_dps(bmi08x_gyro.z, 250, 16) * DPS_TO_RPS;

		if(DEBUG_MODE){
		/* Print the data in rps. */
			printf("\t  Gyr_DPS_X = %4.2f  , Gyr_DPS_Y = %4.2f  , Gyr_DPS_Z = %4.2f\n\r",
					x, y, z);
		}
		gpkt.imu_gyr_x = x;
		gpkt.imu_gyr_y = y;
		gpkt.imu_gyr_z = z;
	}

	/* IMU while loop -------------------------------------------------------------------------*/

	/*  Magnetometer -------------------------------------------------------------*/
	  /* Read output only if new value is available */
	iis2mdc_mag_data_ready_get(&dev_ctx, &mag_drdy);

	if (mag_drdy) {
		/* Read magnetic field data */
		memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
		iis2mdc_magnetic_raw_get(&dev_ctx, data_raw_magnetic);
		magnetic_mG[0] = iis2mdc_from_lsb_to_mgauss( data_raw_magnetic[0]);
		magnetic_mG[1] = iis2mdc_from_lsb_to_mgauss( data_raw_magnetic[1]);
		magnetic_mG[2] = iis2mdc_from_lsb_to_mgauss( data_raw_magnetic[2]);

		gpkt.stale_bits = gpkt.stale_bits | MAG_NEW;
//		pkt.mag_x = data_raw_magnetic[0]; //not sure if this is gonna work how i want it to but maybe
//		pkt.mag_y = data_raw_magnetic[1];
//		pkt.mag_z = data_raw_magnetic[2];
		gpkt.mag_x = magnetic_mG[0] * 0.0000001; //convert mGauss to Tesla by multiplyng by 10^-7
		gpkt.mag_y = magnetic_mG[1] * 0.0000001;
		gpkt.mag_z = magnetic_mG[2] * 0.0000001;
		if(DEBUG_MODE){
			sprintf((char *)tx_buffer,
					"Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
					magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
			tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
		}
		/* Read temperature data */
		memset( &data_raw_temperature, 0x00, sizeof(int16_t) );
		iis2mdc_temperature_raw_get( &dev_ctx, &data_raw_temperature );
		temperature_degC = iis2mdc_from_lsb_to_celsius (
							 data_raw_temperature );
		if(DEBUG_MODE){
			sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f\r\n",
					temperature_degC );
			tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
		}
	} else {
		gpkt.stale_bits = gpkt.stale_bits & ~MAG_NEW;
	}


	/*  End of Magnetometer -------------------------------------------------------------*/

	/*  GPS -------------------------------------------------------------*/

	if(gps_read == true){
		gps_read = false;

		gpkt.stale_bits = gpkt.stale_bits | GPS_NEW;

		//GNSS_GetPVTData(&GNSS_Handle);
		GNSS_GetPOSLLHData(&GNSS_Handle);
		GNSS_ParseBuffer(&GNSS_Handle);

		gpkt.gps_lat = GNSS_Handle.fLat;
		gpkt.gps_long = GNSS_Handle.fLon;

		if(DEBUG_MODE){
			printf("Lat: %f\r\n", gpkt.gps_lat);
			printf("Long: %f\r\n", gpkt.gps_long);
		}
	}

	/*  End of GPS -------------------------------------------------------------*/

	/*  Differential Pressure -------------------------------------------------------------*/
	if(dp_read){
		DPMeasurement(&gpkt); ///Dp sensor
		dp_read = false;
	}
	/*  End of Differential Pressure -------------------------------------------------------------*/
	/*  Barometer -------------------------------------------------------------*/

	DPS310_GetPress(&gpkt); ///barometer

	/*  End of Barometer -------------------------------------------------------------*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 59999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 29;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GYRO_CS_Pin|ACCEL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : AIRSPEED_DRDY_Pin */
  GPIO_InitStruct.Pin = AIRSPEED_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AIRSPEED_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GYRO_CS_Pin ACCEL_CS_Pin */
  GPIO_InitStruct.Pin = GYRO_CS_Pin|ACCEL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

static int packet_transmit(sens_pkt_flt* pkt){

	CDC_Transmit_FS((uint8_t *)pkt, sizeof(sens_pkt_flt));
	//do this commented out bit if you want the packet in a string for some reason
//	char packet[4+12+12+12+4+16+16+1]; //+1 for null terminator since no header
//	sprintf(packet, "%02x%02x", header->dev_id, header->stale_bits);
//	//these are all int16_t and not uint16_t and i think it dont like negative numbers
//	sprintf(packet + strlen(packet), "%04hx%04hx%04hx", data->imu_acc.i16bit[0], data->imu_acc.i16bit[1], data->imu_acc.i16bit[2]);
//	sprintf(packet + strlen(packet), "%04hx%04hx%04hx", data->imu_gyr.i16bit[0], data->imu_gyr.i16bit[1], data->imu_gyr.i16bit[2]);
//	sprintf(packet + strlen(packet), "%04hx%04hx%04hx", data->mag.i16bit[0], data->mag.i16bit[1], data->mag.i16bit[2]);
//
//	//these are unsigned
//	sprintf(packet + strlen(packet), "%04x", data->airspeed);
//	//not sure yet how to convert u32 to hex
//	sprintf(packet + strlen(packet), "%08lx%08lx", data->bar_press, data->bar_temp);
//	sprintf(packet + strlen(packet), "%08lx%08lx", data->gps_lat, data->gps_long);
//

	return 0;
}

/* IMU -----------------------------------------------*/
/* BMI08x data sync. interrupt callback */

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi) { // Message received .. Do Something ...
	rx_rdy = 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi) { // Message received .. Do Something ...
	tx_rdy = 1;
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi) { // Message received .. Do Something ...
	if(hspi->State == HAL_SPI_STATE_READY){
		tx_rdy = 1;
		rx_rdy = 1;
	}
}

//could maybe trigger a gps read every 100 of these? AKA 1Hz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	gpkt.stale_bits = packet_transmit(&gpkt);

	gps_counter++;
	if(gps_counter > 99){
		gps_counter = 0;
		//trigger gps read here
		gps_read = true; ///GPS
	}
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

// These functions are the only ones that really need to be implemented
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {

	uint16_t dev_addr = *(uint16_t*) intf_ptr;

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&hspi1, (uint8_t*) &reg_addr, 1, 100) != HAL_OK) {
		return 1;
	}
	if (HAL_SPI_Receive(&hspi1, reg_data, len, 100) != HAL_OK) {
		return 1;
	}

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_SET);
	bmi08x_delay_us(100, 0);
	return 0;
}

BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {

	uint16_t dev_addr = *(uint16_t*) intf_ptr;

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 100) != HAL_OK) {
		return 1;
	}
	if (HAL_SPI_Transmit(&hspi1, reg_data, len, 50) != HAL_OK) {
		return 1;
	}

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_SET);

	bmi08x_delay_us(100, 0);
	return 0;
}

void bmi08x_delay_us(uint32_t period, void *intf_ptr) {
//    coines_delay_usec(period);
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < period * 10)
		;  // wait for the counter to reach the us input in the parameter
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf,
		uint8_t variant) {
	int8_t rslt = BMI08X_OK;
//    struct coines_board_info board_info;

	if (bmi08x != NULL) {
//        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB);
//        if (result < COINES_SUCCESS)
//        {
//            printf(
//                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
//                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
//            exit(result);
//        }
//
//        result = coines_get_board_info(&board_info);
		/* Bus configuration : SPI */
		// BMI085a_NSS_Pin|BMI085g_NSS_Pin|BMI085_PS_Pin
		if (intf == BMI08X_SPI_INTF) {
//            printf("SPI Interface \n");

			/* To initialize the user SPI function */
			bmi08x->intf = BMI08X_SPI_INTF;
			bmi08x->read = bmi08x_spi_read;
			bmi08x->write = bmi08x_spi_write;

			/* SPI chip select pin for Accel (CSB1_A) */
			acc_dev_add = ACCEL_CS_Pin;

			/* SPI chip select pin for Gyro (CSB2_G) */
			gyro_dev_add = GYRO_CS_Pin;

		}

		/* Selection of bmi085 or bmi088 sensor variant */
		bmi08x->variant = variant;

		/* Assign accel device address to accel interface pointer */
		bmi08x->intf_ptr_accel = &acc_dev_add;

		/* Assign gyro device address to gyro interface pointer */
		bmi08x->intf_ptr_gyro = &gyro_dev_add;

		/* Configure delay in microseconds */
		bmi08x->delay_us = bmi08x_delay_us;

		/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
		bmi08x->read_write_len = 32;


	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt) {
	if (rslt != BMI08X_OK) {
		printf("%s\t", api_name);
		if (rslt == BMI08X_E_NULL_PTR) {
			printf("Error [%d] : Null pointer\r\n", rslt);
		} else if (rslt == BMI08X_E_COM_FAIL) {
			printf("Error [%d] : Communication failure\r\n", rslt);
		} else if (rslt == BMI08X_E_DEV_NOT_FOUND) {
			printf("Error [%d] : Device not found\r\n", rslt);
		} else if (rslt == BMI08X_E_OUT_OF_RANGE) {
			printf("Error [%d] : Out of Range\r\n", rslt);
		} else if (rslt == BMI08X_E_INVALID_INPUT) {
			printf("Error [%d] : Invalid input\r\n", rslt);
		} else if (rslt == BMI08X_E_CONFIG_STREAM_ERROR) {
			printf("Error [%d] : Config stream error\r\n", rslt);
		} else if (rslt == BMI08X_E_RD_WR_LENGTH_INVALID) {
			printf("Error [%d] : Invalid Read write length\r\n", rslt);
		} else if (rslt == BMI08X_E_INVALID_CONFIG) {
			printf("Error [%d] : Invalid config\r\n", rslt);
		} else if (rslt == BMI08X_E_FEATURE_NOT_SUPPORTED) {
			printf("Error [%d] : Feature not supported\r\n", rslt);
		} else if (rslt == BMI08X_W_FIFO_EMPTY) {
			printf("Warning [%d] : FIFO empty\r\n", rslt);
		} else {
			printf("Error [%d] : Unknown error code\r\n", rslt);
		}
	}

}

/* END of IMU ---------------------------------------*/



/* Magnetometer ---------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, IIS2MDC_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, IIS2MDC_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);

}

static void platform_delay(uint32_t ms){
	HAL_Delay(ms);
}
/*End of Magnetometer ------------------------------------------------*/



/* GPS -------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	Flag = 1;
}

//needs to be changed to modify packet
void get_location(sens_pkt_flt* pkt){

	uint8_t dummy[1] = {0xFF};

	HAL_UART_Transmit(&huart3,dummy,1,HAL_MAX_DELAY);

	if(Flag == 1){
		Msgindex=0;
		strcpy(Txdata, (char*)(Rxdata));
		ptr=strstr(Txdata, "GPRMC");
		if(*ptr == 'G'){
			while(1){
				GPS_Payyload[Msgindex]= *ptr;
				Msgindex++;
				*ptr=*(ptr+Msgindex);
				if(*ptr == '\n'){
					GPS_Payyload[Msgindex]='\0';
					break;
				}
			}
			sscanf(GPS_Payyload, "GPRMC, %f,A,%f,N,%f", &time, &Latitude, &Longitude);
			printf("GPS New Data: \r\n");
			Format_data(time, Latitude, Longitude);
			HAL_Delay(1);
			Flag=0;
		}
	} else {
		printf("GPS Stale Data: \r\n");
		Format_data(time, Latitude, Longitude);
	}
}

void Format_data(float Time, float Lat, float Long){
	char Data[100];
	Hours=(int)Time/1000;
	Min=(int)(Time - (Hours*10000))/100;
	Sec=(int)(Time-((Hours*10000)+(Min*100)));
	sprintf(Data, "\r\n Lat=%f, Long=%f",Latitude,Longitude);
	HAL_UART_Transmit(&huart1,(uint8_t*)Data,strlen(Data),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n\n",3,HAL_MAX_DELAY);
}

/* END of GPS -----------------------------------*/

/* DP Sensor ---------------------------*/
void DPMeasurement(sens_pkt_flt* pkt){
	HAL_StatusTypeDef ret;
	for(int i = 0; i < 4; i++){
		buf[i] = 0;
	}
	ret = HAL_I2C_Master_Receive_DMA(&hi2c3, DP_ADDR, buf, 4);
	int16_t val = (((int16_t)(buf[0]) << 8) & 0x3F00) | buf[1]; //getting value from sensor
	int16_t temp = (((int16_t)(buf[2]) << 3) & 0x7F8) | buf[3]>>5;
	if(ret != HAL_OK){
		perror( "how did i get here");
	}
	val = val - 0x2000;
	val = abs(val);

	pkt->stale_bits = pkt->stale_bits | AIR_NEW;
//	pkt->airspeed = val;

	float diffP = (float)(val * 20)/6554;
	float vel = (diffP * 2)/1.225;
	vel = sqrt(vel);
	if(DEBUG_MODE){
		printf("%f \r\n", vel);
		printf("%i \r\n", temp);
	}
	pkt->airspeed = vel;
	pkt->differential_pressure = diffP;
}

/* End DP Sensor -----------------------------------------*/



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
	//initiate read on GPIO 3 rising edge
	case GPIO_PIN_3:
		dp_read = true;
		break;
	case GPIO_PIN_13:
		dp_read = true;
		break;
	case GPIO_PIN_0:
		accel_rdy = 1;
		break;
	case GPIO_PIN_2:
		gyro_rdy = 1;
		break;
	}
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
  while (1)
  {
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

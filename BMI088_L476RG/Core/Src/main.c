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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "bmi08x_defs.h"
#include "bmi08x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/******************************************************************************/
/*!                       Macro definitions                                   */

#define BMI08X_READ_WRITE_LEN  UINT8_C(46)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID      UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID      UINT16_C(0x66)


/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

#define DPS_TO_RPS		(0.01745329251f)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width) {
	float gravity;

	float half_scale = ((1 << bit_width) / 2.0f);

	gravity = (float) ((GRAVITY_EARTH * val * g_range) / half_scale);

	return gravity;
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	int8_t rslt;

	uint8_t times_to_read = 0;
	float x = 0.0, y = 0.0, z = 0.0;
	uint8_t status = 0;

//	struct bmi08x_sensor_data bmi08x_accel_pos;
//	struct bmi08x_sensor_data bmi08x_accel_neg;

	struct bmi08x_err_reg error;
	/* Interface given as parameter
	 *           For I2C : BMI08X_I2C_INTF
	 *           For SPI : BMI08X_SPI_INTF
	 * Sensor variant given as parameter
	 *          For BMI085 : BMI085_VARIANT
	 *          For BMI088 : BMI088_VARIANT
	 */

	HAL_TIM_Base_Start(&htim2);

	printf("Hello World! \r\n");
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
		while (1) {

			if (accel_rdy) {
				rslt = bmi08a_get_data(&bmi08x_accel, &bmi08xdev);
				bmi08x_error_codes_print_result("bmi08a_get_data", rslt);
				accel_rdy = 0;
				printf(
						"ACCEL Acc_Raw_X : %d  Acc_Raw_Y : %d   Acc_Raw_Z : %d ",
						 bmi08x_accel.x, bmi08x_accel.y,
						bmi08x_accel.z);

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

				/* Print the data in m/s2. */
				printf(
						"\t  Acc_ms2_X = %4.2f,  Acc_ms2_Y = %4.2f,  Acc_ms2_Z = %4.2f\n\r",
						x, y, z);


			}
			if (gyro_rdy) {
				rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08xdev);
				bmi08x_error_codes_print_result("bmi08g_get_data",
						rslt);
				gyro_rdy = 0;
				printf(
						"GYRO  Gyr_Raw_X : %d   Gyr_Raw_Y : %d   Gyr_Raw_Z : %d   ",
						 bmi08x_gyro.x, bmi08x_gyro.y,
						bmi08x_gyro.z);

				/* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
				x = lsb_to_dps(bmi08x_gyro.x, 250, 16) * DPS_TO_RPS;
				y = lsb_to_dps(bmi08x_gyro.y, 250, 16) * DPS_TO_RPS;
				z = lsb_to_dps(bmi08x_gyro.z, 250, 16) * DPS_TO_RPS;

				/* Print the data in rps. */
				printf("\t  Gyr_DPS_X = %4.2f  , Gyr_DPS_Y = %4.2f  , Gyr_DPS_Z = %4.2f\n\r",
						x, y, z);

			}

		}
	}

	return 0;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BMI085a_NSS_Pin|BMI085g_NSS_Pin|BMI085_PS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : INT_A_Pin INT_G_Pin */
  GPIO_InitStruct.Pin = INT_A_Pin|INT_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI085a_NSS_Pin BMI085g_NSS_Pin BMI085_PS_Pin */
  GPIO_InitStruct.Pin = BMI085a_NSS_Pin|BMI085g_NSS_Pin|BMI085_PS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

// These functions are the only ones that really need to be implemented
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {

	dev_addr = *(uint16_t*) intf_ptr;
//	uint16_t tx = (reg_addr) << 8;

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_RESET);
	while(!(tx_rdy && rx_rdy));
	tx_rdy = 0;
//	rx_rdy = 0;

	if (HAL_SPI_Transmit_DMA(&hspi2, &reg_addr, sizeof(reg_addr)) != HAL_OK) {
		return 1;
	}


//	tx_rdy = 0;
	while(!(tx_rdy));
	rx_rdy = 0;
	if (HAL_SPI_Receive_DMA(&hspi2, reg_data, len) != HAL_OK) {
			return 1;
	}
//
//	tx_rdy = 0;
	while(!(rx_rdy));
	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_SET);
//	bmi08x_delay_us(200, 0);
	return 0;
}

BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data,
		uint32_t len, void *intf_ptr) {

	dev_addr = *(uint16_t*) intf_ptr;

	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_RESET);

	while(!(tx_rdy && rx_rdy) );
	tx_rdy = 0;
//	rx_rdy = 0;

	if (HAL_SPI_Transmit_DMA(&hspi2, &reg_addr, sizeof(reg_addr)) != HAL_OK) {
		return 1;
	}

//	rx_rdy = 0;
//	tx_rdy = 0;
	while(!tx_rdy);
	tx_rdy = 0;
	if (HAL_SPI_Transmit_DMA(&hspi2,  reg_data, len) != HAL_OK) {
			return 1;
	}
//
//	tx_rdy = 0;
	while(!tx_rdy);
	HAL_GPIO_WritePin(GPIOC, dev_addr, GPIO_PIN_SET);
	return 0;
}

/*!
 * Set up with a timer
 */
void bmi08x_delay_us(uint32_t period, void *intf_ptr) {
//    coines_delay_usec(period);
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < period)
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
			acc_dev_add = BMI085a_NSS_Pin;

			/* SPI chip select pin for Gyro (CSB2_G) */
			gyro_dev_add = BMI085g_NSS_Pin;

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT_A_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	accel_rdy = 1; // Toggle The Output (LED) Pin
    }
    if(GPIO_Pin == INT_G_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
	{
		gyro_rdy = 1; // Toggle The Output (LED) Pin
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

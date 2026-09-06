/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  Calls target system initialization then loop in main.
  ******************************************************************************
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/

#include "mx_usart1.h"
#include <stdio.h>
#include <string.h>

#include "iis3dwb_reg.h"

int _write(int file, char *ptr, int len)
{
    hal_uart_handle_t *huart1 = mx_usart1_uart_gethandle();

    if (huart1 != NULL)
    {
        HAL_UART_Transmit(huart1, ptr, len, 1000);
    }

    return len;
}

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        10 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);


/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  /** System Init: this code placed in targets folder initializes your system.
    * It calls the initialization (and sets the initial configuration) of the peripherals.
    * You can use STM32CubeMX to generate and call this code or not in this project.
    * It also contains the HAL initialization and the initial clock configuration.
    */
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }
  else
  {
    /*
      * You can start your application code here
      */

	  printf("HELLO\n");
	  HAL_GPIO_WritePin(CS_PORT, CS_PIN, HAL_GPIO_PIN_SET);

	  stmdev_ctx_t dev_ctx;

	  /* Initialize mems driver interface */
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.mdelay = platform_delay;
	  dev_ctx.handle = mx_spi1_gethandle();

	  /* Init test platform */
//	  platform_init();

	  /* Wait sensor boot time */
	  platform_delay(BOOT_TIME);

	  /* Check device ID */
	  iis3dwb_device_id_get(&dev_ctx, &whoamI);
      printf("IIS3DWB_ID=0x%x,id=0x%x\n", IIS3DWB_ID, whoamI);
	  if (whoamI != IIS3DWB_ID)
	    while (1);


	  /* Restore default configuration */
	  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);

	  do {
	    iis3dwb_reset_get(&dev_ctx, &rst);
	  } while (rst);

	  /* Enable Block Data Update */
	  iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	  /* Set Output Data Rate */
	  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);

	  /* Set full scale */
	  iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_2g);

	  /* Configure filtering chain(No aux interface)
	   * Accelerometer low pass filter path
	   */
	  iis3dwb_xl_filt_path_on_out_set(&dev_ctx, IIS3DWB_LP_ODR_DIV_100);

	  while (1) {
	    uint8_t reg;

	    /* Read output only if new xl value is available */
	    iis3dwb_xl_flag_data_ready_get(&dev_ctx, &reg);

	    if (reg) {
	      /* Read acceleration field data */
	      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	      iis3dwb_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
	      acceleration_mg[0] =
	        iis3dwb_from_fs2g_to_mg(data_raw_acceleration[0]);
	      acceleration_mg[1] =
	        iis3dwb_from_fs2g_to_mg(data_raw_acceleration[1]);
	      acceleration_mg[2] =
	        iis3dwb_from_fs2g_to_mg(data_raw_acceleration[2]);
	      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
	              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	    }

	    iis3dwb_temp_flag_data_ready_get(&dev_ctx, &reg);

	    if (reg) {
	      /* Read temperature data */
	      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	      iis3dwb_temperature_raw_get(&dev_ctx, &data_raw_temperature);
	      temperature_degC = iis3dwb_from_lsb_to_celsius(data_raw_temperature);
	      printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
	    }
	  }
  }
} /* end main */




/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument used to select the SPI handler
 * @param  reg       register to write
 * @param  bufp      pointer to data to write
 * @param  len       number of consecutive registers to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
  hal_status_t status;
  uint8_t tx[33] = {0};
  uint8_t rx[33] = {0};

  if ((handle == NULL) || (bufp == NULL) ||
      (len == 0U) || (len > 32U))
  {
    return -1;
  }

  /* SPI write command: RW bit = 0 */
  tx[0] = reg & 0x7FU;
  memcpy(&tx[1], bufp, len);

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, HAL_GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive((hal_spi_handle_t *)handle,
                                   tx,
                                   rx,
                                   (uint32_t)len + 1U,
                                   1000U);
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, HAL_GPIO_PIN_SET);

  return (status == HAL_OK) ? 0 : -1;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument used to select the SPI handler
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that stores the data read
 * @param  len       number of consecutive registers to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len)
{
  hal_status_t status;
  uint8_t tx[33] = {0};
  uint8_t rx[33] = {0};

  if ((handle == NULL) || (bufp == NULL) ||
      (len == 0U) || (len > 32U))
  {
    return -1;
  }

  /* SPI read command: RW bit = 1 */
  tx[0] = reg | 0x80U;

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, HAL_GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive((hal_spi_handle_t *)handle,
                                   tx,
                                   rx,
                                   (uint32_t)len + 1U,
                                   1000U);
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, HAL_GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    return -1;
  }

  memcpy(bufp, &rx[1], len);
  return 0;
}

/*
 * @brief  Send data through USART1
 *
 * @param  tx_buffer pointer to transmit buffer
 * @param  len       number of bytes to transmit
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  hal_uart_handle_t *huart1 = mx_usart1_uart_gethandle();

  if ((huart1 != NULL) && (tx_buffer != NULL) && (len > 0U))
  {
    HAL_UART_Transmit(huart1, tx_buffer, len, 1000U);
  }
}

/*
 * @brief  Platform specific delay
 *
 * @param  ms delay in milliseconds
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

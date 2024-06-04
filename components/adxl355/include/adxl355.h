#ifndef __ADXL355_H
#define __ADXL355_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <esp_timer.h>

//***************************************************************************//
//***************************** GLOBAL VARIABLES ****************************//
//***************************************************************************//
// spi accelerometer interface
/**
 * @brief Sample rates in Hertz (Hz)
 *
 */
typedef enum
{
    F_31_5HZ = 0x00, /**< Speed at 31.5 samples per second*/
    F_125HZ          /**< Speed at 125 samples per second */
} frequency_t;

/**
 * @brief Scale configuration in g
 *
 */
typedef enum
{
    SCALE_2G = 0x00, /**< ±2g Scale */
    SCALE_4G,        /**< ±4g Scale */
    SCALE_8G,        /**< ±8g Scale */
} scale_t;

// handle for a device on a SPI bus
extern spi_device_handle_t adxl355_accel_handle;

extern int64_t accel_fifo_length_time0, accel_fifo_length_time1,
    accel_fifo_length_time;

#define DEBUG 0

// defining pins to use SPI bus
#define SPI_ADXL355_HOST SPI2_HOST
#define SPI_ADXL355_GPIO_MOSI 11
#define SPI_ADXL355_GPIO_MISO 13
#define SPI_ADXL355_GPIO_CS 10
#define SPI_ADXL355_GPIO_SCLK 12

// SPI Freq
// SPI clock speed ranges from 100 kHz to 10 MHz
#define _SPI_CLOCK 2 * 1000 * 1000

#define _INSERT_DELAY 0xFF // expressed in _INIT_CONF_OPTIONS as s *10e-2 (cs)

#define FREQ_CONF_INDEX 4  // index of freq config in _INIT_CONF_OPTIONS
#define SCALE_CONF_INDEX 5 // index of scale config in _INIT_CONF_OPTIONS

//***************************************************************************//
//************ ADXL355 REGISTERS TO CONFIG/READ-FROM AND OPTIONS ************//
//***************************************************************************//
// registers addresses data type -> uint8_t
// Filter settings register
// specify parameters for the internal high-pass
// and low-pass filters.
#define ADXL355_REG_FILTER_SETTINGS 0x28
// Filter settings register options
// ODR and low-pass filter corner
#define ADXL355_ODR_125HZ 0x05  // 0000 0101 - 125 Hz and 31.25 Hz
#define ADXL355_ODR_31_5HZ 0x07 // 0000 0111 - 31.25 Hz and 7.813 Hz

// Range register and Interrupt polarity
#define ADXL355_REG_RANGE 0x2C
// Range register options
// #define ADXL355_INT_POLARITY_HIGH 0x40 // INT1 and INT2 are active high
#define ADXL355_HIGH_SPEED_MODE 0x80 // 1000 0000 - bit 7 (1 = high speed mode)
#define ADXL355_SCALE_2G 0x01        // range ±2 g
#define ADXL355_SCALE_4G 0x02        // range ±4 g
#define ADXL355_SCALE_8G 0x03        // range ±8 g

// Power Control Register
#define ADXL355_REG_POWER_CTL 0x2D
// Power Control register options
#define ADXL355_DISABLE_TEMP 0x02 // 0000 0010 - bit 1 (1 = disable temperature)
#define ADXL355_MEASURE_OFF 0x01  // 0000 0001 - bit 0 (1 = standby mode)
#define ADXL355_MEASURE_ON 0x00   // 0000 0000 - bit 0 (0 = measurement mode)

// Reset register
#define ADXL355_REG_RESET 0x2F
// Reset options
#define ADXL355_DO_RESET 0x52 // to resets the device

// FIFO_SAMPLES register
// This register indicates the number of valid data samples
// present in the FIFO buffer. This number ranges from 0 to 96.
#define ADXL355_REG_FIFO_ENTRIES 0x05

// FIFO data register
// Read this register to access data stored in the FIFO
#define ADXL355_REG_FIFO_DATA 0x11

// Axis data registers *************************************
// these registers contain the every axis acceleration data.
// Data is left justified and formatted as twos complement.
#define ADXL355_REG_XDATA3 0x08
#define ADXL355_REG_XDATA2 0x09
#define ADXL355_REG_XDATA1 0x0A
#define ADXL355_REG_YDATA3 0x0B
#define ADXL355_REG_YDATA2 0x0C
#define ADXL355_REG_YDATA1 0x0D
#define ADXL355_REG_ZDATA3 0x0E
#define ADXL355_REG_ZDATA2 0x0F
#define ADXL355_REG_ZDATA1 0x10
// Axis data registers *************************************

// Interrupt pin (INTx) function map register
#define ADXL355_REG_INT_MAP 0X2A
// INT_MAP register options
#define ADXL355_FIFO_FULL_INT1 0X02 // FIFO_FULL interrupt enable on INT1
//***************************************************************************//
//************ ADXL355 REGISTERS TO CONFIG/READ-FROM AND OPTIONS ************//
//***************************************************************************//

//***************************************************************************//
//*********** ADXL355 INITIALIZING REGISTERS AND OPTIONS SEQUENCE ***********//
//***************************************************************************//
// start configuration
#define _INIT_CONF_REGISTERS                                                    \
    {                                                                           \
        ADXL355_REG_RESET, _INSERT_DELAY, ADXL355_REG_POWER_CTL, _INSERT_DELAY, \
            ADXL355_REG_FILTER_SETTINGS, ADXL355_REG_RANGE,                     \
    }

#define _INIT_CONF_OPTIONS                                                    \
    {                                                                         \
        ADXL355_DO_RESET, 10, ADXL355_DISABLE_TEMP | ADXL355_MEASURE_OFF, 10, \
            ADXL355_ODR_125HZ, ADXL355_HIGH_SPEED_MODE | ADXL355_SCALE_2G,    \
    }

// stop accel configuration
#define _END_CONF_REGISTERS                  \
    {                                        \
        ADXL355_REG_POWER_CTL, _INSERT_DELAY \
    }
#define _END_CONF_OPTIONS                              \
    {                                                  \
        ADXL355_DISABLE_TEMP | ADXL355_MEASURE_OFF, 10 \
    }

// accel measurement mode on configuration
#define _ON_MEASUREMENT_REGISTERS            \
    {                                        \
        ADXL355_REG_POWER_CTL, _INSERT_DELAY \
    }
#define _ON_MEASUREMENT_OPTIONS                       \
    {                                                 \
        ADXL355_DISABLE_TEMP | ADXL355_MEASURE_ON, 10 \
    }
//***************************************************************************//
//******************** ADXL355 INIT REGISTERS AND OPTIONS *******************//
//***************************************************************************//

//***************************************************************************//
//***************************** GLOBAL VARIABLES ****************************//
//***************************************************************************//

//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+//
//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+//
//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//
//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//
//***************************** FUNCTIONS SECTION ***************************//
//********************************** BELOW **********************************//
//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//
//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//
//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+//
//+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+//

esp_err_t init_spi_adxl355(void);

esp_err_t _spi_transaction(spi_device_handle_t *spi_dev_handle,
                           uint8_t register_addr, uint8_t read_write,
                           uint8_t *buffer_out, uint16_t len_out,
                           uint8_t *buffer_in, uint16_t len_in);

esp_err_t _read_register(spi_device_handle_t *spi_dev_handle,
                         uint8_t register_addr, uint8_t *buf, uint16_t len);

esp_err_t accel_fifo_length(spi_device_handle_t *spi_dev, uint16_t *length);

esp_err_t adxl355_config_start(spi_device_handle_t *spi_dev, frequency_t freq,
                               scale_t scale);

esp_err_t adxl355_measure_off(spi_device_handle_t *spi_dev);

esp_err_t adxl355_measure_on(spi_device_handle_t *spi_dev);

void accel_debug_check(spi_device_handle_t *spi_dev);

esp_err_t accel_read_fifo(spi_device_handle_t *spi_dev, uint8_t *ptr,
                          uint16_t length);

#endif /* __ADXL355_H */
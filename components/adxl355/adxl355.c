#include "adxl355.h"
#include <stdio.h>
#include <string.h>

//***************************************************************************//
//***************************** GLOBAL VARIABLES ****************************//
//***************************************************************************//
static const char *TAG = "SHM SENSOR NODE -> ADXL355";
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

////////////////////////////////////////////////////////////////////////////////
//************************** Initialize SPI ADXL355 **************************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t init_spi_adxl355(void) {
  // configuration structure for a SPI bus
  spi_bus_config_t adxl355_spi_bus_config = {
      .mosi_io_num = SPI_ADXL355_GPIO_MOSI,
      .miso_io_num = SPI_ADXL355_GPIO_MISO,
      .sclk_io_num = SPI_ADXL355_GPIO_SCLK,
      .quadwp_io_num = -1,  // not used
      .quadhd_io_num = -1,  // not used
      .max_transfer_sz = 32 // maximum transfer size, in bytes
  };
  // initialize a SPI bus
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_ADXL355_HOST, &adxl355_spi_bus_config,
                                     SPI_DMA_CH_AUTO));

  // configuration structure for a SPI slave device
  // that is connected to one of the SPI buses (ADXL355)
  spi_device_interface_config_t adxl355_spi_dev_iface_conf = {
      .command_bits = 8,
      .address_bits = 0,
      .clock_speed_hz = _SPI_CLOCK,
      .mode = 0,
      .spics_io_num = SPI_ADXL355_GPIO_CS,
      .queue_size = 7};
  ESP_LOGI(TAG, "Initializing SPI ADXL355 Accelerometer");
  // allocate a device on a SPI bus
  ESP_ERROR_CHECK(spi_bus_add_device(
      SPI_ADXL355_HOST, &adxl355_spi_dev_iface_conf, &adxl355_accel_handle));

  return ESP_OK;
}
////////////////////////////////////////////////////////////////////////////////
//************************** Initialize SPI ADXL355 **************************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//*********************** Own SPI transaction function ***********************//
////////////////////////////////////////////////////////////////////////////////
/**
 * The function '_spi_transaction' performs a SPI transaction with a specified
 * SPI device handle, register address, read_write flag, output buffer,
 * output buffer length, input buffer, and input buffer length.
 *
 * @param spi_dev_handle spi_device_handle_t pointer, which is a handle to the
 * SPI device.
 * @param register_addr The register´s address to write to or read from.
 * @param read_write A flag indicating whether the transaction is a write
 * operation or a read operation. If 'read_write' is 1 (non-zero), it indicates
 * a write operation. If 'read_write' is 0, it indicates a read operation.
 * @param buffer_out A pointer to the buffer containing the data to be written
 * to the SPI device.
 * @param len_out The length of the data to be written to the SPI device in
 * bytes.
 * @param buffer_in A pointer to the buffer where the received data will be
 * stored.
 * @param len_in The length of the input buffer in bytes.
 *
 * @return an 'esp_err_t' value, which is the error code indicating the success
 * or failure of the SPI transaction.
 */
esp_err_t _spi_transaction(spi_device_handle_t *spi_dev_handle,
                           uint8_t register_addr, uint8_t read_write,
                           uint8_t *buffer_out, uint16_t len_out,
                           uint8_t *buffer_in, uint16_t len_in) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // initialize to zero the transaction

  // t.flags = write ? SPI_TRANS_USE_TXDATA : SPI_TRANS_USE_RXDATA;

  // register_addr << 1: shifts all the bits of register_addr to the left 1
  // position
  // A6 A5 A4 A3 A2 A1 R/W' last bit: 1 if R(read)   ***
  //                                  0 if W (write) ***
  t.cmd = register_addr << 1 | (read_write ? 0x00 : 0x01);
  t.addr = NULL;

  // TX
  t.tx_buffer = read_write ? buffer_out : NULL;
  // attribute .length is in bits, so multiply by 8 the
  // 'len_out' or 'len_in' received as parameters
  t.length = len_out >= len_in ? len_out * 8 : len_in * 8;

  // RX
  t.rx_buffer = read_write ? NULL : buffer_in;
  t.rxlength = t.length;

  // ESP_LOGI(TAG, "Debugging _spi_transaction before
  // spi_device_polling_transmit...");
  ret = spi_device_polling_transmit(*spi_dev_handle, &t);
  // ESP_LOGI(TAG, "Debugging _spi_transaction after
  // spi_device_polling_transmit...");

  return ret;
}
////////////////////////////////////////////////////////////////////////////////
//*********************** Own SPI transaction function ***********************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//************************* Reading ADXL355 Register *************************//
////////////////////////////////////////////////////////////////////////////////
/**
 * The function '_read_register' reads data from a specified register using SPI
 * communication.
 *
 * @param spi_dev_handle spi_device_handle_t pointer, which is a handle to the
 * SPI device.
 * @param register_addr address of the register that you want to read from.
 * @param buf pointer to a buffer where the data read from the register will be
 * stored. The length of the buffer is specified by the "len" parameter.
 * @param len length of the buffer 'buf' in bytes. It specifies the number of
 * bytes to read from the register specified by 'register_addr'.
 *
 * @return an 'esp_err_t' value.
 */
esp_err_t _read_register(spi_device_handle_t *spi_dev_handle,
                         uint8_t register_addr, uint8_t *buf, uint16_t len) {
  esp_err_t ret;

  // Occupy the SPI bus for a device to do continuous transactions.
  // Transactions to all other devices will be put off until
  // ``spi_device_release_bus`` is called. The function will wait until all the
  // existing transactions have been sent.
  spi_device_acquire_bus(*spi_dev_handle, portMAX_DELAY);

  // reset, clock auto, accelerometer duty cycled
  ret = _spi_transaction(spi_dev_handle, register_addr, 0, NULL, 0, buf, len);
  if (ret != ESP_OK) {
    ESP_LOGE("SPI_TRANSACTION <_read_register>", "read register ERROR");
    return ret;
  }

  spi_device_release_bus(*spi_dev_handle);
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
//************************* Reading ADXL355 Register *************************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//*********************** Reading ADXL355 FIFO Length ************************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t accel_fifo_length(spi_device_handle_t *spi_dev, uint16_t *length) {
  accel_fifo_length_time0 = esp_timer_get_time();

  esp_err_t ret;
  uint8_t tmp;

  spi_device_acquire_bus(*spi_dev, portMAX_DELAY);
  // ADXL355_REG_FIFO_ENTRIES indicates the number of valid data samples present
  // in the FIFO buffer. This number ranges from 0 to 96.
  ret =
      _spi_transaction(spi_dev, ADXL355_REG_FIFO_ENTRIES, 0, NULL, 0, &tmp, 1);

  // Number of data samples stored in the FIFO (x, y, z axis)
  *length = tmp;

  // number of full xyz sets (e.g.: 96 values means 96/3 xyz sets of
  // samples)
  int numRegs = *length / 3;

  // There are 96 21-bit locations in the FIFO. Each location
  // contains 20 bits of data and a marker bit for the x-axis data.
  // each location ocupies 3 bytes
  // total amount of bytes = (data samples in FIFO) * 3
  *length *= 3;

  ESP_LOGI("ADXL355 FIFO Length",
           "There are %d available samples (set of x, y, z accel values) in "
           "FIFO (%d bytes)",
           numRegs, (*length));

  spi_device_release_bus(*spi_dev);

  accel_fifo_length_time1 = esp_timer_get_time();
  accel_fifo_length_time = accel_fifo_length_time1 - accel_fifo_length_time0;

  return ret;
}
////////////////////////////////////////////////////////////////////////////////
//*********************** Reading ADXL355 FIFO Length ************************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//********************** Starting ADXL355 Accelerometer **********************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t adxl355_config_start(spi_device_handle_t *spi_dev, frequency_t freq,
                               scale_t scale) {
  uint8_t raddr[] = _INIT_CONF_REGISTERS;
  uint8_t ropts[] = _INIT_CONF_OPTIONS;
  uint8_t buf, i;
  esp_err_t ret;

  switch (freq) {
  case (F_125HZ):
    ropts[FREQ_CONF_INDEX] = ADXL355_ODR_125HZ;
    break;
  case (F_31_5HZ):
    ropts[FREQ_CONF_INDEX] = ADXL355_ODR_31_5HZ;
    break;
  default:
    return ESP_FAIL;
  }

  switch (scale) {
  case (SCALE_8G):
    ropts[SCALE_CONF_INDEX] = ADXL355_HIGH_SPEED_MODE | ADXL355_SCALE_8G;
    break;
  case (SCALE_4G):
    ropts[SCALE_CONF_INDEX] = ADXL355_HIGH_SPEED_MODE | ADXL355_SCALE_4G;
    break;
  case (SCALE_2G):
    ropts[SCALE_CONF_INDEX] = ADXL355_HIGH_SPEED_MODE | ADXL355_SCALE_2G;
  }

  for (i = 0; i < (uint8_t)sizeof(ropts); i++) {
    // ESP_LOGI(TAG, "Debugging <%d>...", i);
    if (raddr[i] == _INSERT_DELAY) {
      // ESP_LOGI(TAG, "Debugging if _INSERT_DELAY <%d>...", i);
      //  vTaskDelay(pdMS_TO_TICKS(ropts[i] * 10)));
      vTaskDelay((ropts[i] * 10) / portTICK_PERIOD_MS);
    } else {
      // ESP_LOGI(TAG, "Debugging if not _INSERT_DELAY <%d>...", i);
      buf = ropts[i];
      // ESP_LOGI(TAG, "Debugging if not _INSERT_DELAY, before _spi_transaction
      // <%d>...", i);
      ret = _spi_transaction(spi_dev, raddr[i], 1, &buf, 1, NULL, 0);
      // ESP_LOGI(TAG, "Debugging if not _INSERT_DELAY, after _spi_transaction
      // <%d>...", i);
      if (ret != ESP_OK) {
        // %2x outputs the value in hex format: at least 2 digits, padding with
        // 0 if less than 2 digits
        ESP_LOGE("SPI_TRANSACTION <accel_start>",
                 "Sending config to register 0x%2x -> ERROR", raddr[i]);
        return ret;
      }
      // else ESP_LOGI("IAM20680_START","Register %2x <- %2x OK", raddr[i],
      // buf);
    }
  }

  ESP_LOGI(TAG, "ADXL355 with Frequency and Scale CONFIGURED CORRECTLY!!!");

  return ESP_OK;
}
////////////////////////////////////////////////////////////////////////////////
//********************** Starting ADXL355 Accelerometer **********************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//********************** Stopping ADXL355 Accelerometer **********************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t adxl355_measure_off(spi_device_handle_t *spi_dev) {
  uint8_t raddr[] = _END_CONF_REGISTERS;
  uint8_t ropts[] = _END_CONF_OPTIONS;
  uint8_t buf, i;
  esp_err_t ret = ESP_OK;

  for (i = 0; i < (uint8_t)sizeof(ropts); i++) {
    if (raddr[i] == _INSERT_DELAY)
      vTaskDelay((ropts[i] * 10) / portTICK_PERIOD_MS);
    else {
      buf = ropts[i];
      ret = _spi_transaction(spi_dev, raddr[i], 1, &buf, 1, NULL, 0);
      if (ret != ESP_OK) {
        // %2x outputs the value in hex format: at least 2 digits, padding with
        // 0 if less than 2 digits
        ESP_LOGE("SPI_TRANSACTION <accel_stop>",
                 "Sending config to register 0x%2x -> ERROR", raddr[i]);
        break;
      }
    }
  }
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
//********************** Stopping ADXL355 Accelerometer **********************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//***************** Measurement Mode ON ADXL355 Accelerometer ****************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t adxl355_measure_on(spi_device_handle_t *spi_dev) {
  uint8_t raddr[] = _ON_MEASUREMENT_REGISTERS;
  uint8_t ropts[] = _ON_MEASUREMENT_OPTIONS;
  uint8_t buf, i;
  esp_err_t ret = ESP_OK;

  for (i = 0; i < (uint8_t)sizeof(ropts); i++) {
    if (raddr[i] == _INSERT_DELAY)
      vTaskDelay((ropts[i] * 10) / portTICK_PERIOD_MS);
    else {
      buf = ropts[i];
      ret = _spi_transaction(spi_dev, raddr[i], 1, &buf, 1, NULL, 0);
      if (ret != ESP_OK) {
        // %2x outputs the value in hex format: at least 2 digits, padding with
        // 0 if less than 2 digits
        ESP_LOGE("SPI_TRANSACTION <accel_measure_on>",
                 "Sending config to register 0x%2x -> ERROR", raddr[i]);
        break;
      }
    }
  }

  return ret;
}
////////////////////////////////////////////////////////////////////////////////
//***************** Measurement Mode ON ADXL355 Accelerometer ****************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//************** Debugging ADXL355 (Checking registers values) ***************//
////////////////////////////////////////////////////////////////////////////////
void accel_debug_check(spi_device_handle_t *spi_dev) {
  uint8_t raddr[] = _INIT_CONF_REGISTERS;
  uint8_t ropts[] = _INIT_CONF_OPTIONS;

  uint8_t buf, i;

  for (i = 0; i < (uint8_t)sizeof(ropts); i++) {
    if (raddr[i] != _INSERT_DELAY) {
      _read_register(spi_dev, raddr[i], &buf, 1);
      // shows the actual value of the given register and
      // the initial configurated option
      ESP_LOGI("ADXL355 DEBUG CHECK", "Register %2x: %2x (init. conf: %2x)",
               raddr[i], buf, ropts[i]);
    }
  }
  return;
}
////////////////////////////////////////////////////////////////////////////////
//************** Debugging ADXL355 (Checking registers values) ***************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//*********************** Reading ADXL355 FIFO Values ************************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t accel_read_fifo(spi_device_handle_t *spi_dev, uint8_t *ptr,
                          uint16_t length) {
  // uint8_t *i;
  esp_err_t ret;
  uint8_t nRegs, aux[96 * 3];
  uint8_t v[3];

  spi_device_acquire_bus(*spi_dev, portMAX_DELAY);

  // reading ADXL355_REG_FIFO_DATA register, to access data stored in the FIFO.
  ret = _spi_transaction(spi_dev, ADXL355_REG_FIFO_DATA, 0, NULL, 0,
                         (uint8_t *)aux, length);

  spi_device_release_bus(*spi_dev);

  nRegs = length / 3; // number of samples in the fifo, each has 3 bytes
  // uint32_t numero = 0;

  // data treatment
  // fifo locations
  // -------------------------------------------
  // |X19|X18|X17|X16|X15|X14|X13|X12| -> aux[0]
  // |X11|X10|X09|X08|X07|X06|X05|X04| -> aux[1]
  // |X03|X02|X01|X00|   |   | 0 | 1 | -> aux[2]
  // -------------------------------------------
  // |Y19|Y18|Y17|Y16|Y15|Y14|Y13|Y12| -> aux[3]
  // |Y11|Y10|Y09|Y08|Y07|Y06|Y05|Y04| -> aux[4]
  // |Y03|Y02|Y01|Y00|   |   | 0 | 0 | -> aux[5]
  // -------------------------------------------
  // |Z19|Z18|Z17|Z16|Z15|Z14|Z13|Z12| -> aux[6]
  // |Z11|Z10|Z09|Z08|Z07|Z06|Z05|Z04| -> aux[7]
  // |Z03|Z02|Z01|Z00|   |   | 0 | 0 | -> aux[8]
  // -------------------------------------------
  // twos complement, most significant bit -> negative 1
  //                                          positive 0
  // v[0] = |0-1|0-1|0-1|0-1|X19|X18|X17|X16|
  // v[1] = |X15|X14|X13|X12|X11|X10|X09|X08|
  // v[2] = |X07|X06|X05|X04|X03|X02|X01|X00|
  for (uint16_t i = 0; i < nRegs; i++) {
    v[0] = (0x0F & (aux[(i * 3)] >> 4));
    // if number in a fifo location is negative (most-significant-bit is 1)
    // fill with 1 at beggining
    if ((aux[(i * 3)] & 0x80) > 0)
      v[0] |= 0xF0;
    v[1] = (0xF0 & (aux[(i * 3)] << 4)) | (0x0F & (aux[(i * 3) + 1] >> 4));
    v[2] = (0xF0 & (aux[(i * 3) + 1] << 4)) | (0x0F & (aux[(i * 3) + 2] >> 4));

    // ESP_LOGI(TAG, "DEBUGGING accel_read_fifo");

    // copy v to RingBuffer pointer
    // memcpy(ptr + (i * 3), v, 3);

#if DEBUG
    if (i == 0) {
      int32_t valor;
      double valoracc;
      double sensitivity = 256000.0;
      double scaleFactor = 3.9e-6;
      double g = 9.8;

      valor = v[0] << 16 | v[1] << 8 | v[2];
      if ((v[0] & 0x80) > 0)
        valor |= 0xff << 24;

      valoracc = (double)valor / sensitivity * g;

      if (i % 3 == 0) {
        printf("DRIVER_ADXL355\tMuestra %2d => x = %+02.2f m/s2, ", (i / 3) + 1,
               valoracc);
      }
      if (i % 3 == 1) {
        printf("y = %+02.2f m/s2, ", valoracc);
      }
      if (i % 3 == 2) {
        printf("z = %+02.2f m/s2\n", valoracc);
      }
    }
#endif
  } // end for (uint16_t i = 0; i < nRegs; i++)

  return ret;
}
////////////////////////////////////////////////////////////////////////////////
//*********************** Reading ADXL355 FIFO Values ************************//
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//************************************D´A************************************//
///////////////////////////////////////////////////////////////////////////////
// *****************************************************************************
// ** Project name:               SHM SENSOR NODE
// ** Created by:                 Andy Duarte Taño
// ** Created:                    25/03/2024
// ** Last modified:              22/04/2024
// ** Software:                   C/C++, ESP-IDF Framework, VS Code
// ** Hardware:                   ESP32-S3-DevKit-C1
//                                Reyax RYLR998 LoRa Module
//                                ADXL355
// ** Contact:                    andyduarte0192@gmail.com
//                                andyduarte0192@ugr.es

// ** Copyright (c) 2024, Andy Duarte Taño. All rights reserved.

// ** This code is free to use for any purpose as long as the author is cited.

// ** Code description:
// *****************************************************************************
///////////////////////////////////////////////////////////////////////////////
//************************************D´A************************************//
///////////////////////////////////////////////////////////////////////////////

// Users can call heap_caps_get_free_size(MALLOC_CAP_8BIT) to get the free size
// of all DRAM heaps.

///////////////////////////////////////////////////////////////////////////////
//************************************+ +************************************//
///////////////////////////////////////////////////////////////////////////////

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <driver/spi_common.h>
#include <esp_timer.h>
// #include <inttypes.h>
#include "adxl355.h"
#include "esp_spiffs.h"
#include "numerical_systems_conv.h"
#include "rylr998.h"
#include <esp_random.h>
#include <float.h> // required for DBL_EPSILON
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define NULL_END ((char){'\0'})

// note: you should check if offset==sizeof(buf) after use
#define strcpyALL(buf, offset, ...)                                            \
  do {                                                                         \
    char *bp =                                                                 \
        (char *)(buf + offset); /*so we can add to the end of a string*/       \
    const char *s, *a[] = {__VA_ARGS__, NULL}, **ss = a;                       \
    while ((s = *ss++))                                                        \
      while ((*s) && (++offset < (int)sizeof(buf)))                            \
        *bp++ = *s++;                                                          \
    if (offset != sizeof(buf))                                                 \
      *bp = 0;                                                                 \
  } while (0)

//***************************************************************************//
//************************************+ +************************************//
//***************************************************************************//

//***************************************************************************//
//**************************** TX Timer section *****************************//
//***************************************************************************//
// A count of the number of times the timer expires.
int8_t timer_expires_counter = 0;
const int16_t RETRANSMISSION_TIMEOUT = 1000;
const int timerID = 1;
const int8_t RETRANSMISSION_COUNT = 5;
//***************************************************************************//
//**************************** TX Timer section *****************************//
//***************************************************************************//

//***************************************************************************//
//********************************** FLAGS **********************************//
//***************************************************************************//
char *rcv_ack =
    "Y"; // to know if we received the ACK of the message previously transmitted
char *is_data_sent_ok = "N";
char *is_sending_data = "N";
char *is_ctrl_sent_ok =
    "N"; // to know if we alredy sent the 'control' message that indicates the
         // RX how many bits represent each sample for each axis (*_bits_tx)
         // because this will be our first message to send ALWAYS!!!
char *is_sending_ctrl = "Y"; // to know when we are sending the 'control'
                             // message, so we just send ONE

char *is_rylr998_module_init = "N";

char *retransmission_msg_defined = "N"; // to know if we already allocated
                                        // space for the retransmission message
char *need_retransmit =
    "N"; // variable to control if we need to retransmit data
char *need_retransmit_ctrl =
    "N"; // to know if we nned retransmit the 'ctrl' message
char *need_retransmit_data =
    "N"; // to know if we nned retransmit the 'data' message
//***************************************************************************//
//********************************** FLAGS **********************************//
//***************************************************************************//

//***************************************************************************//
//***************************** HANDLE VARIABLES ****************************//
//***************************************************************************//
QueueHandle_t uart_queue;

// Handles for the tasks
TaskHandle_t check_ack_task_handle = NULL, transmit_data_task_handle = NULL,
             xyz_data_reading_task_handle = NULL,
             compressing_samples_task_handle = NULL;

esp_timer_handle_t xyz_data_reading_tmr_hndl;

spi_device_handle_t adxl355_accel_handle;

TimerHandle_t resendTimer; // global variable to work with the timer
//***************************************************************************//
//***************************** HANDLE VARIABLES ****************************//
//***************************************************************************//

//***************************************************************************//
//***************************** GLOBAL VARIABLES ****************************//
//***************************************************************************//
// testing parameters
frequency_t test_freq = F_125HZ;
scale_t test_scale = SCALE_8G;

// max and min values of each axis set of samples
double min_x_value = 0; // min x initializing
double max_x_value = 0; // max x initializing
double min_y_value = 0; // min y initializing
double max_y_value = 0; // max y initializing
double min_z_value = 0; // min z initializing
double max_z_value = 0; // max z initializing

// to keep tracking of the sample we are extracting from
// *_samples_compressed_bin that we are copying to total_bits_tx_after_pad0
int d_a = 0;

// needed bits for each axis to transmit their respective samples
uint8_t x_bits_tx, y_bits_tx, z_bits_tx;

Lora_Data_t Lora_data;

uint16_t MSG_COUNTER_TX = 0; // counter to set the message transaction ID

#define MAXCHAR 4096

#define CR 10
#define N 1024     // number of samples
#define p (N / CR) // number of compressed samples
int8_t **sensing_mtrx;
int *rnd_index_arr;              // to store 'p' random values -> 0~N-1
uint16_t xyz_data_reg_count = 0; // to keep tracking the times timer expires

// all next will store all samples taken (N rows and 3 columns)
// 3 columns for each acceleration data
// XDATA3-XDATA2-XDATA1
// YDATA3-YDATA2-YDATA1
// ZDATA3-ZDATA2-ZDATA1
uint8_t **x_samples; // x measurements
uint8_t **y_samples; // y measurements
uint8_t **z_samples; // z measurements

// all next will have all samples re-arranged,
// and decoded (double type)
double *x_samples_double; // x measurements
double *y_samples_double; // y measurements
double *z_samples_double; // z measurements

// all next will have compressed measurement vector
double *x_samples_compressed; // x measurements
double *y_samples_compressed; // y measurements
double *z_samples_compressed; // z measurements

// all next will have the difference between consecutive compressed samples
// double *x_samples_compressed_dif; // x measurements
// double *y_samples_compressed_dif; // y measurements
// double *z_samples_compressed_dif; // z measurements

// array of pointers for the binary representation of the position of a
// sample in the new interval [0 ~ (max + |min|)]
char *x_samples_compressed_bin[p];
char *y_samples_compressed_bin[p];
char *z_samples_compressed_bin[p];

uint16_t XYZ_DATA_READ_PERIOD_MS = 8;

#define TASK_MEMORY 1024 * 2

int64_t temp_time0, temp_time1;

#define LED_PIN GPIO_NUM_1

//***************************************************************************//
//***************************** GLOBAL VARIABLES ****************************//
//***************************************************************************//

static const char *TAG = "!!!SHM SENSOR NODE!!!";

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

///////////////////////////////////////////////////////////////////////////////
//******************* Callback function when timer expires ******************//
///////////////////////////////////////////////////////////////////////////////
void vTimerCallbackRYLR998(TimerHandle_t pxTimer) {
  // Define a callback function that will be used by multiple timer instances.
  // The callback function counts the number of times the timer expires,
  // and stop the timer once the timer has expired 5 times.

  ESP_LOGW(TAG, "Timer timed-out");

  // Optionally do something if the pxTimer parameter is NULL.
  // configASSERT(pxTimer);

  // Increment the number of times that pxTimer has expired.
  timer_expires_counter += 1;

  // if timer expired 'RETRANSMISSION_COUNT' times, then stop it
  if (timer_expires_counter == RETRANSMISSION_COUNT) {
    // Do not use a block time if calling a timer API function from a
    // timer callback function, as doing so could cause a deadlock!
    xTimerStop(pxTimer, 0);
    ESP_LOGW(TAG, "TX never received ACK message from RX");

    // reset the counter
    timer_expires_counter = 0;

    ESP_LOGW(TAG, "Re-starting transmit_data_task to retransmit the data");

    // rcv_ack = "Y"; // added, I need to remove this, it's only for testing

    need_retransmit = "Y"; // changed
    if ((strncmp((const char *)is_sending_ctrl, "Y", 1) == 0)) {
      need_retransmit_ctrl = "Y";
      need_retransmit_data = "N";

      // is_data_sent_ok = "+OK"; // added, I need to remove this, it's only for
      //  testing
    }
    if ((strncmp((const char *)is_sending_data, "Y", 1) == 0)) {
      need_retransmit_ctrl = "N";
      need_retransmit_data = "Y";

      // is_data_sent_ok = "+OK"; // added, I need to remove this, it's only for
      //  testing
    }

    ESP_LOGW(TAG, "!!!Retransmission needed!!!\n");
    vTaskDelay(pdMS_TO_TICKS(DELAY));
    xTaskNotifyGive(transmit_data_task_handle);
  }
}
///////////////////////////////////////////////////////////////////////////////
//******************* Callback function when timer expires ******************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//****************** Setting timer for ACK message timeout ******************//
///////////////////////////////////////////////////////////////////////////////
esp_err_t set_timer_rylr998() {
  // Create then start the timer. Starting the timer before the scheduler
  // has been started means the timers will start running immediately that
  // the scheduler starts.

  ESP_LOGI(TAG, "RYLR998 Timer init configuration");
  resendTimer = xTimerCreate(
      "RETRANSMISSION TIMEOUT TIMER", // Just a text name, not used by the
                                      // kernel.
      (pdMS_TO_TICKS(
          RETRANSMISSION_TIMEOUT)), // Timer period in ticks. The function
                                    // converts milisec to ticks
      pdTRUE, // The timers will auto-reload themselves when they expire.
      (void *)timerID,      // Assign the timer a unique id equal
      vTimerCallbackRYLR998 // Each timer calls the same callback when it
                            // expires.
  );

  if (resendTimer == NULL) {
    // The timer was not created.
    ESP_LOGE(TAG, "***RYLR998 Timer was not created***");
  } else {
    // Start the timer.  No block time is specified, and even if one was
    // it would be ignored because the scheduler has not yet been
    // started.
    if (xTimerStart(resendTimer, 0) != pdPASS) {
      // The timer could not be set into the Active state.
      ESP_LOGE(TAG,
               "***RYLR998 Timer could not be set into the Active state***");

      // I can get rid if this if, because we start the timer and the line below
      // stops it, IT´S REDUNDANT
    }
  }

  xTimerStop(resendTimer, 0);
  ESP_LOGI(TAG, "RYLR998 Timer configured and stopped !!!COMPLETED!!!");

  return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////
//****************** Setting timer for ACK message timeout ******************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//**************** CALCULATE NUMBER OF BITS OF A GIVEN NUMBER ***************//
///////////////////////////////////////////////////////////////////////////////
// num >>= 1; its equivalent to divide num by 2 and assign the result to num
// again. Its an efficient way to divide an integer number by 2 using bit
// operations
uint8_t needed_bits(uint32_t num) {
  uint8_t n_bits = 0;
  while (num > 0) {
    num >>= 1; // Desplazar el número hacia la derecha
    n_bits++;  // Incrementar la cantidad de bits
  }
  return n_bits;
}
///////////////////////////////////////////////////////////////////////////////
//**************** CALCULATE NUMBER OF BITS OF A GIVEN NUMBER ***************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//**************************** ACCEL RESOLUTION *****************************//
///////////////////////////////////////////////////////////////////////////////
double accel_res(scale_t scale) {
  // calculating the resolution accoirding to accel selected scale
  double resolution = 0;
  switch (scale) {
  case SCALE_2G:
    resolution = (2.048 * 2) / pow(2, 20);
    break;
  case SCALE_4G:
    resolution = (4.096 * 2) / pow(2, 20);
    break;
  case SCALE_8G:
    resolution = (8.192 * 2) / pow(2, 20);
    break;
  default:
    break;
  }

  return resolution;
}
///////////////////////////////////////////////////////////////////////////////
//**************************** ACCEL RESOLUTION *****************************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//****************** ROUND UP TO A NUMBER OF DECIMAL PLACES *****************//
///////////////////////////////////////////////////////////////////////////////
double round_to_decimal(double num, scale_t scale) {
  int decimal_places = 0;
  switch (scale) {
  case SCALE_2G:
    decimal_places = 11;
    break;
  case SCALE_4G:
    decimal_places = 10;
    break;
  case SCALE_8G:
    decimal_places = 9;
    break;
  default:
    break;
  }
  double multiplier = pow(10, decimal_places);
  return round(num * multiplier) / multiplier;
}
///////////////////////////////////////////////////////////////////////////////
//****************** ROUND UP TO A NUMBER OF DECIMAL PLACES *****************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//************************* ENCODE MIN AXIS SAMPLES *************************//
//************************** FROM DOUBLE TO 32-BITS *************************//
///////////////////////////////////////////////////////////////////////////////
/* uint32_t encode_min_axis_samples_32bits(scale_t scale, double
value_to_encode) { uint32_t encoded_value = 0;

  int32_t divission_value = 0;

  // calculating the resolution according to accel selected scale
  double resolution = accel_res(scale);

  divission_value = value_to_encode / resolution;

  // |0-1|0-1|0-1|0-1|0-1|0-1|0-1|0-1|
  // |0-1|0-1|0-1|0-1|X19|X18|X17|X16|
  // |X15|X14|X13|X12|X11|X10|X09|X08|
  // |X07|X06|X05|X04|X03|X02|X01|X00|
  // I will keep X00 ~ X18 to convert it to de value
  // and X19 will indicate the sign (+ or -)

  // shifting 19 places to right the 'value_to_decode' to get the sign
  static uint8_t sign;
  sign = 0x01 & (value_to_decode >> 19);

  // out of 32 bits, we ignore the first 13 bits, so only the remaining will
  // have the real value to convert, and adjust it according to the sign in bit
  // X19
  static int32_t real_value;
  real_value = 0x7FFFF & (value_to_decode);

  // if sign == 1, it means 'value_to_decode' is negative
  // and substracting 2^(n_bits - 1) to the 'real_value', we get the
  // corresponding negative value
  if (sign) {
    real_value = real_value - pow(2, 19);
  }

  decoded_value = resolution * real_value;

  return encoded_value;
} */
///////////////////////////////////////////////////////////////////////////////
//************************* ENCODE MIN AXIS SAMPLES *************************//
//************************** FROM DOUBLE TO 32-BITS *************************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//****************************** DECODE SAMPLES *****************************//
//************************** FROM 20-BITS TO DOUBLE *************************//
///////////////////////////////////////////////////////////////////////////////
double decode_20bits_sample(scale_t scale, uint32_t value_to_decode) {
  double decoded_value = 0;

  // calculating the resolution accoirding to accel selected scale
  double resolution = accel_res(scale);

  // |0-1|0-1|0-1|0-1|0-1|0-1|0-1|0-1|
  // |0-1|0-1|0-1|0-1|X19|X18|X17|X16|
  // |X15|X14|X13|X12|X11|X10|X09|X08|
  // |X07|X06|X05|X04|X03|X02|X01|X00|
  // I will keep X00 ~ X18 to convert it to real_value
  // and X19 will indicate the sign (+ or -)

  // shifting 19 places to right the 'value_to_decode' to get the sign
  static uint8_t sign;
  sign = 0x01 & (value_to_decode >> 19);

  // out of 32 bits, we ignore the first 13 bits, so only the remaining will
  // have the real value to convert (not signed), and adjust it according to the
  // sign in bit X19
  static int32_t real_value;
  real_value = 0x7FFFF & (value_to_decode);

  // if sign == 1, it means 'value_to_decode' is negative
  // and substracting 2^(n_bits - 1) to the 'real_value', we get the
  // corresponding negative value
  if (sign) {
    real_value = real_value - pow(2, 19);
  }

  decoded_value = resolution * real_value;

  return decoded_value;
}
///////////////////////////////////////////////////////////////////////////////
//****************************** DECODE SAMPLES *****************************//
//************************** FROM 20-BITS TO DOUBLE *************************//
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//***************************** RE-ARRANGED DATA *****************************//
//*********** COMBINE EACH 8-BITS REGISTER INTO ONE 24-BITS NUMBER ***********//
////////////////////////////////////////////////////////////////////////////////
uint32_t re_arrange_accel_data(uint8_t data_0, uint8_t data_1, uint8_t data_2) {
  //  data registers
  //  ----------------------------------------------------
  //  |X19|X18|X17|X16|X15|X14|X13|X12| -> data_0
  //  |X11|X10|X09|X08|X07|X06|X05|X04| -> data_1
  //  |X03|X02|X01|X00|   |   | 0 | 1 | -> data_2
  //  ----------------------------------------------------
  //  |Y19|Y18|Y17|Y16|Y15|Y14|Y13|Y12| -> data_0
  //  |Y11|Y10|Y09|Y08|Y07|Y06|Y05|Y04| -> data_1
  //  |Y03|Y02|Y01|Y00|   |   | 0 | 0 | -> data_2
  //  ----------------------------------------------------
  //  |Z19|Z18|Z17|Z16|Z15|Z14|Z13|Z12| -> data_0
  //  |Z11|Z10|Z09|Z08|Z07|Z06|Z05|Z04| -> data_1
  //  |Z03|Z02|Z01|Z00|   |   | 0 | 0 | -> data_2
  //  ----------------------------------------------------
  //  twos complement, most significant bit -> negative 1
  //                                           positive 0
  //
  //  v[0] = |0-1|0-1|0-1|0-1|X19|X18|X17|X16|
  //  v[1] = |X15|X14|X13|X12|X11|X10|X09|X08|
  //  v[2] = |X07|X06|X05|X04|X03|X02|X01|X00|
  //
  //  tmp_24_bits_num = |0000|-|X19-X16|-|X15-X08|-|X07-X00|
  //  ----------------------------------------------------

  uint32_t tmp_24_bits_num = 0;
  uint8_t v[3];

  v[0] = (0x0F & (data_0 >> 4));
  // if first bit of 'data_0' is 1, it means is negative
  // fill with 1 at beggining
  if ((data_0 & 0x80) > 0) {
    v[0] = v[0] | 0xF0;
    tmp_24_bits_num = tmp_24_bits_num | 0xFF000000;
  }
  v[1] = (0xF0 & (data_0 << 4)) | (0x0F & (data_1 >> 4));
  v[2] = (0xF0 & (data_1 << 4)) | (0x0F & (data_2 >> 4));

  tmp_24_bits_num = (v[0] << 16) | (v[1] << 8) | v[2];

  return tmp_24_bits_num;
}
////////////////////////////////////////////////////////////////////////////////
//***************************** RE-ARRANGED DATA *****************************//
//*********** COMBINE EACH 8-BITS REGISTER INTO ONE 24-BITS NUMBER ***********//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//***************************** INITIALIZE SPIFFS ****************************//
//*************************** SPI FLASH FILE SYSTEM **************************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t init_file_system() {
  esp_vfs_spiffs_conf_t sile_sys_config = {.base_path = "/storage",
                                           .partition_label = NULL,
                                           .max_files = 2,
                                           .format_if_mount_failed = true};

  esp_err_t result = esp_vfs_spiffs_register(&sile_sys_config);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(result));
    return ESP_FAIL;
  }

  // calculte info of our partition to see the used memory
  // and how much we have left
  size_t total_partition_size = 0, used_partition_size = 0;
  result = esp_spiffs_info(sile_sys_config.partition_label,
                           &total_partition_size, &used_partition_size);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get partition info (%s)", esp_err_to_name(result));
  } else {
    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total_partition_size,
             used_partition_size);
  }

  return ESP_OK;
}
////////////////////////////////////////////////////////////////////////////////
//***************************** INITIALIZE SPIFFS ****************************//
//*************************** SPI FLASH FILE SYSTEM **************************//
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
int compare(const void *a, const void *b) { return (*(int *)a - *(int *)b); }
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//************************ BUILD RANDOM SAMPLES ARRAY ************************//
//************************** NEW USING esp_random ****************************//
////////////////////////////////////////////////////////////////////////////////
esp_err_t p_rnd_samples(int *perm_array, const int n, const int m) {
  // auxiliar array to track which numbers have been generated already
  // calloc sets allocated memory to zero
  bool *generated_numbers = calloc(n, sizeof(bool));
  if (generated_numbers == NULL) {
    ESP_LOGE(TAG, "NOT ENOUGH HEAP");
    ESP_LOGE(TAG, "Failed to allocate *generated_numbers");
    return ESP_FAIL;
  }

  // generate 'p' unique random numbers
  for (int i = 0; i < m; i++) {
    int rnd_numb;
    do {
      rnd_numb = esp_random() % n; // generate random number between 0 and N-1
    } while (generated_numbers[rnd_numb]); // check if generated number
                                           // have been generated before

    // store the unique random number
    perm_array[i] = rnd_numb;
    generated_numbers[rnd_numb] = true; // mark the number as generated
  }

  // ascending sort the random numbers
  qsort(perm_array, m, sizeof(int), compare);

  free(generated_numbers);

  return ESP_OK;
}
////////////////////////////////////////////////////////////////////////////////
//************************ BUILD RANDOM SAMPLES ARRAY ************************//
//************************** NEW USING esp_random ****************************//
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//************************** Initialization of LED  *************************//
///////////////////////////////////////////////////////////////////////////////
void init_led(void) {
  ESP_ERROR_CHECK(gpio_reset_pin(LED_PIN));
  ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));

  gpio_set_level(LED_PIN, 0);

  ESP_LOGI(TAG, "Init LED !!!COMPLETED!!!");
}
///////////////////////////////////////////////////////////////////////////////
//************************** Initialization of LED  *************************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//*************************** INITIALIZE 2D ARRAYS **************************//
//******************** ALL SAMPLES & COMPRESSED SAMPLES *********************//
///////////////////////////////////////////////////////////////////////////////
esp_err_t init_2d_arrays() {
  // Note that arr[i][j] is same as *(*(arr+i)+j)
  /* int o, count = 0;
  for (i = 0; i < r; i++)
    for (j = 0; j < c; j++)
      arr[i][j] = ++count; */ // OR *(*(arr+i)+j) = ++count

  /* for (i = 0; i < r; i++)
    for (j = 0; j < c; j++)
      printf("%d ", arr[i][j]); */

  /* Code for further processing and free the
     dynamically allocated memory */

  //************************************************************************//
  // 2D arrays to hold ALL original samples
  x_samples = (uint8_t **)malloc(N * sizeof(uint8_t *));
  if (x_samples == NULL)
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate **x_samples");
  y_samples = (uint8_t **)malloc(N * sizeof(uint8_t *));
  if (y_samples == NULL)
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate **y_samples");
  z_samples = (uint8_t **)malloc(N * sizeof(uint8_t *));
  if (z_samples == NULL)
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate **z_samples");

  for (int i = 0; i < N; i++) {
    x_samples[i] = (uint8_t *)malloc(3 * sizeof(uint8_t));
    if (x_samples[i] == NULL)
      ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *x_samples[%d]", i);
    y_samples[i] = (uint8_t *)malloc(3 * sizeof(uint8_t));
    if (y_samples[i] == NULL)
      ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *y_samples[%d]", i);
    z_samples[i] = (uint8_t *)malloc(3 * sizeof(uint8_t));
    if (z_samples[i] == NULL)
      ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *z_samples[%d]", i);
  }

  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After *_samples allocation");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");
  // 2D arrays to hold ALL samples
  //************************************************************************//

  //************************************************************************//
  // 2D arrays to hold the DECODED original samples
  x_samples_double = (double *)malloc(N * sizeof(double));
  if (x_samples_double == NULL) {
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *x_samples_double");
    return ESP_FAIL;
  }

  y_samples_double = (double *)malloc(N * sizeof(double));
  if (y_samples_double == NULL) {
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *y_samples_double");
    return ESP_FAIL;
  }

  z_samples_double = (double *)malloc(N * sizeof(double));
  if (z_samples_double == NULL) {
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *z_samples_double");
    return ESP_FAIL;
  }

  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After *_samples_double allocation");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");
  // 2D arrays to hold the DECODED original samples
  //************************************************************************//

  //************************************************************************//
  // 2D arrays to hold COMPRESSED samples
  x_samples_compressed = (double *)malloc(p * sizeof(double));
  if (x_samples_compressed == NULL) {
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *x_samples_compressed");
    return ESP_FAIL;
  }

  y_samples_compressed = (double *)malloc(p * sizeof(double));
  if (y_samples_compressed == NULL) {
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *y_samples_compressed");
    return ESP_FAIL;
  }

  z_samples_compressed = (double *)malloc(p * sizeof(double));
  if (z_samples_compressed == NULL) {
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *z_samples_compressed");
    return ESP_FAIL;
  }

  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After *_samples_compressed allocation");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");
  // 2D array to hold CMPRESSED samples
  //************************************************************************//

  //************************************************************************//
  // 2D arrays to hold SENSING MATRIX
  sensing_mtrx = (int8_t **)malloc(p * sizeof(int8_t *));
  if (sensing_mtrx == NULL)
    ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate **sensing_mtrx");

  for (int i = 0; i < p; i++) {
    sensing_mtrx[i] = (int8_t *)malloc(N * sizeof(int8_t));
    if (sensing_mtrx[i] == NULL)
      ESP_LOGE(TAG, "*NOT ENOUGH HEAP* Failed to allocate *sensing_mtrx[%d]",
               i);
  }

  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After sensing_mtrx allocation");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");
  // 2D arrays to hold SENSING MATRIX
  //************************************************************************//

  return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////
//*************************** INITIALIZE 2D ARRAYS **************************//
//******************** ALL SAMPLES & COMPRESSED SAMPLES *********************//
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//************************* Compressing Samples Task *************************//
////////////////////////////////////////////////////////////////////////////////
void compressing_samples_task(void *pvParameters) {
  double resolution = accel_res(test_scale);
  //
  uint32_t tmp_arranged_sample = 0;
  while (1) {
    // Block indefinitely (without a timeout, so no need to check the function's
    // return value) to wait for a notification. Here the RTOS task notification
    // is being used as a binary semaphore, so the notification value is cleared
    // to zero on exit. NOTE! Real applications should not block indefinitely,
    // but instead time out occasionally in order to handle error conditions
    // that may prevent the interrupt from sending any more notifications.
    ulTaskNotifyTake(pdTRUE,         // Clear the notification value on exit
                     portMAX_DELAY); // Block indefinitely

    //*************************** COMPRESSION STAGE **************************//
    // step 1: re-arrange and decode samples ***********************************
    //
    for (uint16_t i = 0; i < N; i++) {
      temp_time0 += esp_timer_get_time();
      //
      // tmp_arranged_sample = |0000|-|X19-X16|-|X15-X08|-|X07-X00|
      tmp_arranged_sample = re_arrange_accel_data(
          x_samples[i][0], x_samples[i][1], x_samples[i][2]);
      x_samples_double[i] = round_to_decimal(
          decode_20bits_sample(test_scale, tmp_arranged_sample), test_scale);
      // x_samples_double[i] = resolution * tmp_arranged_sample;
      temp_time1 += esp_timer_get_time();
      //
      // tmp_arranged_sample = |0000|-|Y19-Y16|-|Y15-Y08|-|Y07-Y00|
      tmp_arranged_sample = re_arrange_accel_data(
          y_samples[i][0], y_samples[i][1], y_samples[i][2]);
      y_samples_double[i] = round_to_decimal(
          decode_20bits_sample(test_scale, tmp_arranged_sample), test_scale);
      // y_samples_double[i] = resolution * tmp_arranged_sample;
      //
      //   tmp_arranged_sample = |0000|-|Z19-Z16|-|Z15-Z08|-|Z07-Z00|
      tmp_arranged_sample = re_arrange_accel_data(
          z_samples[i][0], z_samples[i][1], z_samples[i][2]);
      z_samples_double[i] = round_to_decimal(
          decode_20bits_sample(test_scale, tmp_arranged_sample), test_scale);
      // z_samples_double[i] = resolution * tmp_arranged_sample;
      //
    } // end for (uint16_t i = 0; i < N; i++)
    //
    ESP_LOGI(TAG, "Taken time to decode a sample: <%lld us>",
             temp_time1 / N - temp_time0 / N);
    //
    ESP_LOGW(TAG, "compressing_samples_task DEBUGGING <COMPRESSION STAGE STEP "
                  "1 FINISHED>");
    // step 1: re-arrange and decode samples ***********************************

    // step 2: read antipodal matrix to build sensing matrix *******************
    FILE *antipodal_mtrx_file =
        fopen("/storage/antipodal_sensing_matrix.csv", "r");
    if (antipodal_mtrx_file == NULL) {
      ESP_LOGE(TAG, "Failed to open file for reading");
      abort();
    }
    //
    char buffer[MAXCHAR];
    char *record, *line;
    int row = 0;
    int column = 0;
    //
    for (int i = 0; i < p; i++) {
      // 'fgets' returns 'NULL' if there´s nothing else to read from the file
      while ((line = fgets(buffer, sizeof(buffer), antipodal_mtrx_file)) !=
             NULL) {
        // compare each rnd_index_arr[i] with the corresponding readed line in
        // the file
        if (row != rnd_index_arr[i]) {
          row++;
          continue;
        } else {
          record = strtok(line, ",");
          while (record != NULL) {
            // printf("%s ", record); // here you can put the record into the
            //  array as per your requirement.
            sensing_mtrx[i][column++] = atoi(record);
            record = strtok(NULL, ",");
          }
          ++row;
          column = 0;
        }
        break;
      }
    }
    // Close the file
    fclose(antipodal_mtrx_file);
    //
    ESP_LOGW(TAG, "compressing_samples_task DEBUGGING <COMPRESSION STAGE STEP "
                  "2 FINISHED>");
    // step 2: read antipodal matrix to build sensing matrix *******************

    // step 3: multiply 'sensing_mtrx' by each '*_samples_arranged' array ******
    double tmp_sum_x, tmp_sum_y, tmp_sum_z;
    for (int i = 0; i < p; i++) {
      // vTaskDelay(pdMS_TO_TICKS(10));
      // ESP_LOGI(TAG, "compressing_samples_task DEBUGGING");
      tmp_sum_x = 0;
      tmp_sum_y = 0;
      tmp_sum_z = 0;
      for (int j = 0; j < N; j++) {
        // vTaskDelay(pdMS_TO_TICKS(10));
        tmp_sum_x += (sensing_mtrx[i][j] * x_samples_double[j]);
        //
        tmp_sum_y += (sensing_mtrx[i][j] * y_samples_double[j]);
        //
        tmp_sum_z += (sensing_mtrx[i][j] * z_samples_double[j]);
      }
      x_samples_compressed[i] = round_to_decimal(tmp_sum_x, test_scale);
      y_samples_compressed[i] = round_to_decimal(tmp_sum_y, test_scale);
      z_samples_compressed[i] = round_to_decimal(tmp_sum_z, test_scale);
    }
    //
    ESP_LOGW(TAG, "compressing_samples_task DEBUGGING <COMPRESSION STAGE STEP "
                  "3 FINISHED>");
    // step 3: multiply 'sensing_mtrx' by each '*_samples_arranged' array ******
    //*************************** COMPRESSION STAGE **************************//

    //*********************** SAMPLES RE-DIMENSIONING ***********************//
    // perform a redimension of the samples to transmit,
    // so they occupy less bits
    // setp 1: determine max and min values of each axis set of samples *******
    min_x_value = x_samples_compressed[0]; // min x initializing
    max_x_value = x_samples_compressed[0]; // max x initializing
    min_y_value = y_samples_compressed[0]; // min y initializing
    max_y_value = y_samples_compressed[0]; // max y initializing
    min_z_value = z_samples_compressed[0]; // min z initializing
    max_z_value = z_samples_compressed[0]; // max z initializing
    //
    for (int i = 1; i < p; i++) {
      if (x_samples_compressed[i] < min_x_value)
        min_x_value = x_samples_compressed[i];
      if (x_samples_compressed[i] > max_x_value)
        max_x_value = x_samples_compressed[i];
      //
      if (y_samples_compressed[i] < min_y_value)
        min_y_value = y_samples_compressed[i];
      if (y_samples_compressed[i] > max_y_value)
        max_y_value = y_samples_compressed[i];
      //
      if (z_samples_compressed[i] < min_z_value)
        min_z_value = z_samples_compressed[i];
      if (z_samples_compressed[i] > max_z_value)
        max_z_value = z_samples_compressed[i];
    }
    //
    ESP_LOGW(TAG, "Values 'max' and 'min' for each axis:");
    ESP_LOGW(TAG, "double_min_x_value = %.15lf", min_x_value);
    ESP_LOGW(TAG, "double_max_x_value = %.15lf", max_x_value);
    ESP_LOGW(TAG, "double_min_y_value = %.15lf", min_y_value);
    ESP_LOGW(TAG, "double_max_y_value = %.15lf", max_y_value);
    ESP_LOGW(TAG, "double_min_z_value = %.15lf", min_z_value);
    ESP_LOGW(TAG, "double_max_z_value = %.15lf", max_z_value);
    //
    ESP_LOGW(TAG,
             "compressing_samples_task DEBUGGING <SAMPLES RE-DIMENSIONING STEP "
             "1 FINISHED>");
    vTaskDelay(pdMS_TO_TICKS(10));
    // setp 1: determine max and min values of each axis set of samples *******

    // step 2: determine how many possible values are multiples of ************
    // 'range/2^n-bits'(which is the resolution of the accelerometer) can fit *
    // inside those min and max ***********************************************
    //
    uint32_t x_possible_values =
        ((fabs(min_x_value)) + (max_x_value)) / (resolution);
    uint32_t y_possible_values =
        ((fabs(min_y_value)) + (max_y_value)) / (resolution);
    uint32_t z_possible_values =
        ((fabs(min_z_value)) + (max_z_value)) / (resolution);
    //
    ESP_LOGW(TAG,
             "Possible values multiples of resolution (%.12lf):", resolution);
    ESP_LOGW(TAG,
             "x_possible_values = %lu\t\ty_possible_values = "
             "%lu\t\tz_possible_values = %lu",
             x_possible_values, y_possible_values, z_possible_values);
    //
    ESP_LOGW(TAG,
             "compressing_samples_task DEBUGGING <SAMPLES RE-DIMENSIONING STEP "
             "2 FINISHED>");
    vTaskDelay(pdMS_TO_TICKS(10));
    // step 2: determine how many possible values are multiples of ************
    // 'range/2^n-bits'(which is the resolution of the accelerometer) can fit *
    // inside those min and max ***********************************************

    // step 3: determine how many bits are necessary to represent *************
    // the quantity of those possible values multiples of the resolution, *****
    // between those max and min **********************************************
    x_bits_tx = needed_bits(x_possible_values);
    y_bits_tx = needed_bits(y_possible_values);
    z_bits_tx = needed_bits(z_possible_values);
    //
    ESP_LOGW(TAG,
             "Needed bits to represent the quantity of those possible "
             "values multiples of the resolution (%.12lf):",
             resolution);
    ESP_LOGW(TAG, "x_bits_tx = %u\t\ty_bits_tx = %u\t\tz_bits_tx = %u",
             x_bits_tx, y_bits_tx, z_bits_tx);
    //
    ESP_LOGW(TAG,
             "compressing_samples_task DEBUGGING <SAMPLES RE-DIMENSIONING STEP "
             "3 FINISHED>");
    vTaskDelay(pdMS_TO_TICKS(10));
    // step 3: determine how many bits are necessary to represent *************
    // the quantity of those possible values multiples of the resolution, *****
    // between those max and min **********************************************

    // step 4: determine the new binary value of each sample inside ************
    // the new interval ********************************************************
    // from (min ~ max) we moved it to (0 ~ max + |min|) ***********************
    //  sample + |min|
    // -------------------------------------------------------------------------
    // range / 2^n-bits
    // this will result in an unsigned-int that represents the position of that
    // sample value in the new interval ****************************************
    // -------------------------------------------------------------------------
    // to store temporarily that position of the sample value in the new
    // interval [0 ~ (max + |min|)]
    double x_sample_compressed_temp_dec;
    double y_sample_compressed_temp_dec;
    double z_sample_compressed_temp_dec;
    //
    // to store temporarily the binary of the sample value in the new interval
    // [0 ~ (max + |min|)]
    char *x_sample_compressed_temp_bin =
        malloc((x_bits_tx + 1) * sizeof(*x_sample_compressed_temp_bin));
    if (x_sample_compressed_temp_bin == NULL) {
      ESP_LOGE(
          TAG,
          "*NOT ENOUGH HEAP* Failed to allocate *x_sample_compressed_temp_bin");
    }
    char *y_sample_compressed_temp_bin =
        malloc((y_bits_tx + 1) * sizeof(*y_sample_compressed_temp_bin));
    if (y_sample_compressed_temp_bin == NULL) {
      ESP_LOGE(
          TAG,
          "*NOT ENOUGH HEAP* Failed to allocate *y_sample_compressed_temp_bin");
    }
    char *z_sample_compressed_temp_bin =
        malloc((z_bits_tx + 1) * sizeof(*z_sample_compressed_temp_bin));
    if (z_sample_compressed_temp_bin == NULL) {
      ESP_LOGE(
          TAG,
          "*NOT ENOUGH HEAP* Failed to allocate *z_sample_compressed_temp_bin");
    }
    //
    ESP_LOGW(TAG, "*********************************************************");
    ESP_LOGW(TAG, "After *_sample_compressed_temp_bin allocation");
    ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
    ESP_LOGW(TAG, "*********************************************************");
    vTaskDelay(pdMS_TO_TICKS(10));
    //
    // Allocate rows (+1 for the ending character \0)
    for (int i = 0; i < p; ++i) {
      x_samples_compressed_bin[i] =
          (char *)malloc((x_bits_tx + 1) * sizeof(char));
      if (x_samples_compressed_bin[i] == NULL) {
        ESP_LOGE(TAG,
                 "*NOT ENOUGH HEAP* Failed to allocate "
                 "x_samples_compressed_bin[%d]",
                 i);
      }
      //
      y_samples_compressed_bin[i] =
          (char *)malloc((y_bits_tx + 1) * sizeof(char));
      if (y_samples_compressed_bin[i] == NULL) {
        ESP_LOGE(TAG,
                 "*NOT ENOUGH HEAP* Failed to allocate "
                 "y_samples_compressed_bin[%d]",
                 i);
      }
      //
      z_samples_compressed_bin[i] =
          (char *)malloc((z_bits_tx + 1) * sizeof(char));
      if (z_samples_compressed_bin[i] == NULL) {
        ESP_LOGE(TAG,
                 "*NOT ENOUGH HEAP* Failed to allocate "
                 "z_samples_compressed_bin[%d]",
                 i);
      }
    }
    //
    ESP_LOGW(TAG, "*********************************************************");
    ESP_LOGW(TAG, "After *_samples_compressed_bin allocation");
    ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
    ESP_LOGW(TAG, "*********************************************************");
    vTaskDelay(pdMS_TO_TICKS(10));
    //
    for (int i = 0; i < p; i++) {
      // x_sample_compressed_temp_dec is the position of x_samples_compressed[i]
      // in the new interval [0 ~ (max + |min|)]
      x_sample_compressed_temp_dec =
          ((fabs(min_x_value)) + (x_samples_compressed[i])) / (resolution);
      // x_sample_compressed_temp_bin is the binary representation
      DecimalToBinary(x_sample_compressed_temp_dec, x_bits_tx,
                      x_sample_compressed_temp_bin);
      strcpy(x_samples_compressed_bin[i], x_sample_compressed_temp_bin);
      ESP_LOGW(TAG, "X-AXIS [%d] -> %.15lf : %.15lf : %.1lf : %s", i,
               x_samples_compressed[i],
               ((fabs(min_x_value)) + (x_samples_compressed[i])),
               x_sample_compressed_temp_dec, x_samples_compressed_bin[i]);
      //
      // y_sample_compressed_temp_dec is the position of y_samples_compressed[i]
      // in the new interval [0 ~ (max + |min|)]
      y_sample_compressed_temp_dec =
          ((fabs(min_y_value)) + (y_samples_compressed[i])) / (resolution);
      // y_sample_compressed_temp_bin is the binary representation
      DecimalToBinary(y_sample_compressed_temp_dec, y_bits_tx,
                      y_sample_compressed_temp_bin);
      strcpy(y_samples_compressed_bin[i], y_sample_compressed_temp_bin);
      ESP_LOGW(TAG, "Y-AXIS [%d] -> %.15lf : %.15lf : %.1lf : %s", i,
               y_samples_compressed[i],
               ((fabs(min_y_value)) + (y_samples_compressed[i])),
               y_sample_compressed_temp_dec, y_samples_compressed_bin[i]);
      //
      // z_sample_compressed_temp_dec is the position of z_samples_compressed[i]
      // in the new interval [0 ~ (max + |min|)]
      z_sample_compressed_temp_dec =
          ((fabs(min_z_value)) + (z_samples_compressed[i])) / (resolution);
      // z_sample_compressed_temp_bin is the binary representation
      DecimalToBinary(z_sample_compressed_temp_dec, z_bits_tx,
                      z_sample_compressed_temp_bin);
      strcpy(z_samples_compressed_bin[i], z_sample_compressed_temp_bin);
      ESP_LOGW(TAG, "Z-AXIS [%d] -> %.15lf : %.15lf : %.1lf : %s", i,
               z_samples_compressed[i],
               ((fabs(min_z_value)) + (z_samples_compressed[i])),
               z_sample_compressed_temp_dec, z_samples_compressed_bin[i]);

      // x_samples_representation[i] = ((fabs(double_min_x_value)) +
      // (double_x_samples[i])) / ((RANGE) / (RES));
      // y_samples_representation[i] = ((fabs(double_min_y_value)) +
      // (double_y_samples[i])) / ((RANGE) / (RES));
      // z_samples_representation[i] = ((fabs(double_min_z_value)) +
      // (double_z_samples[i])) / ((RANGE) / (RES));
    }

    ESP_LOGW(TAG,
             "compressing_samples_task DEBUGGING <SAMPLES RE-DIMENSIONING STEP "
             "4 FINISHED>\n");
    vTaskDelay(pdMS_TO_TICKS(10));
    // step 4: determine the new binary value of every samples ****************
    // set foreach axis *******************************************************
    //*********************** SAMPLES RE-DIMENSIONING ***********************//

    // CHECK ALL MALLOC CALLS TO FREE MEMORY
    // ********************************************************************** //
    // FREEING ALLOCATED MEMORY  ******************************************** //
    for (int i = 0; i < p; i++) {
      free(x_samples[i]);
      free(y_samples[i]);
      free(z_samples[i]);

      free(sensing_mtrx[i]);
    }
    free(x_samples);
    free(y_samples);
    free(z_samples);

    free(x_samples_double);
    free(y_samples_double);
    free(z_samples_double);

    free(x_samples_compressed);
    free(y_samples_compressed);
    free(z_samples_compressed);

    free(x_sample_compressed_temp_bin);
    free(y_sample_compressed_temp_bin);
    free(z_sample_compressed_temp_bin);

    free(rnd_index_arr);
    free(sensing_mtrx);

    /* free(x_samples_compressed_dif);
    free(y_samples_compressed_dif);
    free(z_samples_compressed_dif); */
    // FREEING ALLOCATED MEMORY  ******************************************** //
    // ********************************************************************** //

    ESP_LOGI(TAG, "***!!!COMPRESSED "
                  "FINISHED!!!***********************************************"
                  "***********************************************");

    ESP_LOGE(TAG, "*********************************************************");
    ESP_LOGW(TAG, "After COMPRESSED FINISHED");
    ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
    ESP_LOGE(TAG,
             "*********************************************************\n");
    //
    ESP_LOGI(TAG, "Waiting 2s before transmitting data...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskNotifyGive(transmit_data_task_handle);
    //
  } // while (1)
} // compressing_samples_task
////////////////////////////////////////////////////////////////////////////////
//************************* Compressing Samples Task *************************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//******************** Periodic timer callback function **********************//
////////////////////////////////////////////////////////////////////////////////
void xyz_data_reading_tmr_callback(void *arg) {
  xTaskNotifyGive(xyz_data_reading_task_handle);
}
////////////////////////////////////////////////////////////////////////////////
//******************** Periodic timer callback function **********************//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//************************** XYZ Data Reading Task ***************************//
////////////////////////////////////////////////////////////////////////////////
void xyz_data_reading_task(void *pvParameters) {
  uint8_t tmp_buf;
  while (1) {
    // Block indefinitely (without a timeout, so no need to check the function's
    // return value) to wait for a notification. Here the RTOS task notification
    // is being used as a binary semaphore, so the notification value is cleared
    // to zero on exit. NOTE! Real applications should not block indefinitely,
    // but instead time out occasionally in order to handle error conditions
    // that may prevent the interrupt from sending any more notifications.
    ulTaskNotifyTake(pdTRUE,         // Clear the notification value on exit
                     portMAX_DELAY); // Block indefinitely

    ///// REAL CODE ******************************************************//
    if (xyz_data_reg_count == N) {
      xyz_data_reg_count = 0;
      esp_timer_stop(xyz_data_reading_tmr_hndl);

      gpio_set_level(LED_PIN, 0);
      ESP_LOGI(TAG, "Turning OFF Measurement Mode ADXL355");
      ESP_ERROR_CHECK(adxl355_measure_off(&adxl355_accel_handle));

      xTaskNotifyGive(compressing_samples_task_handle);
    } else {
      ///// reading x, y, z sample values /////*******************************
      // ESP_LOGI(TAG, "Reading xyz data registers");
      // x axis measurement
      // XDATA3-XDATA2-XDATA1
      _read_register(&adxl355_accel_handle, ADXL355_REG_XDATA3, &tmp_buf, 1);
      x_samples[xyz_data_reg_count][0] = tmp_buf; // storing
      _read_register(&adxl355_accel_handle, ADXL355_REG_XDATA2, &tmp_buf, 1);
      x_samples[xyz_data_reg_count][1] = tmp_buf; // storing
      _read_register(&adxl355_accel_handle, ADXL355_REG_XDATA1, &tmp_buf, 1);
      x_samples[xyz_data_reg_count][2] = tmp_buf; // storing
      //
      // y axis measurement
      // YDATA3-YDATA2-YDATA1
      _read_register(&adxl355_accel_handle, ADXL355_REG_YDATA3, &tmp_buf, 1);
      y_samples[xyz_data_reg_count][0] = tmp_buf; // storing
      _read_register(&adxl355_accel_handle, ADXL355_REG_YDATA2, &tmp_buf, 1);
      y_samples[xyz_data_reg_count][1] = tmp_buf; // storing
      _read_register(&adxl355_accel_handle, ADXL355_REG_YDATA1, &tmp_buf, 1);
      y_samples[xyz_data_reg_count][2] = tmp_buf; // storing
      //
      // z axis measurement
      // ZDATA3-ZDATA2-ZDATA1
      _read_register(&adxl355_accel_handle, ADXL355_REG_ZDATA3, &tmp_buf, 1);
      z_samples[xyz_data_reg_count][0] = tmp_buf; // storing
      _read_register(&adxl355_accel_handle, ADXL355_REG_ZDATA2, &tmp_buf, 1);
      z_samples[xyz_data_reg_count][1] = tmp_buf; // storing
      _read_register(&adxl355_accel_handle, ADXL355_REG_ZDATA1, &tmp_buf, 1);
      z_samples[xyz_data_reg_count][2] = tmp_buf; // storing
      ///// reading x, y, z sample values /////*******************************

      xyz_data_reg_count++;
      // da++;
    } // if (xyz_data_reg_count == N)
    ///// REAL CODE ******************************************************//

    ///// TESTING 200MS READ FIFO *****************************************//
    /* if (xyz_data_reg_count == 50) {
      xyz_data_reg_count = 0;
      esp_timer_stop(xyz_data_reading_tmr_hndl);
      ESP_LOGI(TAG, "***!!!APP FINISHED!!!***");
      ESP_LOGI(TAG, "<da> = %u", da);

      // freeing up allocated memory *******************************************
      for (int i = 0; i < p; i++) {
        free(x_samples[i]);
        free(y_samples[i]);
        free(z_samples[i]);
      }
      free(x_samples);
      free(y_samples);
      free(z_samples);

      free(perm);
      // freeing up allocated memory *******************************************
    } else {
      ESP_ERROR_CHECK(accel_fifo_length(&adxl355_accel_handle, &flength));
      ESP_LOGI(TAG, "Fifo length: %d", flength);

      ESP_ERROR_CHECK(accel_read_fifo(&adxl355_accel_handle,
                                      (uint8_t *)(ptr_fifo_read), flength));

      xyz_data_reg_count++;
    } */
    ///// TESTING 200MS READ FIFO *****************************************//

  } // while (1)
} // xyz_data_reading_task
////////////////////////////////////////////////////////////////////////////////
//************************** XYZ Data Reading Task ***************************//
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//********** Transmit data only if ACK received correctly by the RX *********//
///////////////////////////////////////////////////////////////////////////////
void transmit_data_task(void *pvParameters) {
  ///// header for the data to send
  char *header_hex_of_data_to_send = NULL;
  //
  ///// data to send
  char *data_to_send_hex = NULL;
  //
  ///// full message to send (header + data)
  char *full_message_hex = NULL;
  //
  ///// full message to retransmit if needed (header + data)
  char *retransmission_message_hex = NULL;
  //
  // temporal vlaue to store each 8-bit segment
  char *tmp_segment_bin = NULL;
  //
  // temporal variable to store the conversion of
  // decimal tmp_segment_dec to hexadecimal (in DATA BLOCKING)
  char *tmp_segment_hex = NULL;
  while (1) {
    // Block to wait for permission to transmit
    // Block indefinitely (without a timeout, so no need to check the function's
    // return value) to wait for a notification. Here the RTOS task notification
    // is being used as a binary semaphore, so the notification value is cleared
    // to zero on exit. NOTE! Real applications should not block indefinitely,
    // but instead time out occasionally in order to handle error conditions
    // that may prevent the interrupt from sending any more notifications.
    ulTaskNotifyTake(pdTRUE,         // Clear the notification value on exit
                     portMAX_DELAY); // Block indefinitely

    // Print out remaining task stack memory (words)
    ESP_LOGW(TAG, "Bytes free in 'transmit_data_task' stack: <%zu>",
             uxTaskGetStackHighWaterMark(NULL));

    //
    /* ESP_LOGE(
        TAG,
        "******************* <DEBUGGING FLAGS STATE> ********************");
    ESP_LOGW(TAG, "rcv_ack: <%s>", rcv_ack);
    ESP_LOGW(TAG, "is_data_sent_ok: <%s>", is_data_sent_ok);
    ESP_LOGW(TAG, "is_sending_data: <%s>", is_sending_data);
    ESP_LOGW(TAG, "is_ctrl_sent_ok: <%s>", is_ctrl_sent_ok);
    ESP_LOGW(TAG, "is_sending_ctrl: <%s>", is_sending_ctrl);
    ESP_LOGW(TAG, "is_rylr998_module_init: <%s>", is_rylr998_module_init);
    ESP_LOGW(TAG, "retransmission_msg_defined: <%s>",
             retransmission_msg_defined);
    ESP_LOGW(TAG, "need_retransmit: <%s>", need_retransmit);
    ESP_LOGW(TAG, "need_retransmit_ctrl: <%s>", need_retransmit_ctrl);
    ESP_LOGW(TAG, "need_retransmit_data: <%s>", need_retransmit_data);
    ESP_LOGE(
        TAG,
        "******************* <DEBUGGING FLAGS STATE> ********************\n");
  */
    //

    // ESP_LOGI(TAG, "transmit_data_task...");
    //  ESP_LOGI(TAG, "MSG_COUNTER_TX = <%d>...\n", MSG_COUNTER_TX);

    // we only send while ack is received or if it´s necessary to retransmit,
    // while we received "+OK" when we sent the message
    while ((MSG_COUNTER_TX <= MAX_TRANSACTION_ID) &&
           ((strncmp((const char *)rcv_ack, "Y", 1) == 0) ||
            (strncmp((const char *)need_retransmit, "Y", 1) == 0))) {
      //
      ///// RETRANSMISSION *****************************************************
      ///// RETRANSMISSION *****************************************************
      ///// RETRANSMISSION *****************************************************
      // if retransmission needed
      if (strncmp((const char *)need_retransmit, "Y", 1) == 0) {
        // if what we were transmitting was the 'ctrl' message
        ///// VISUALIZE /////
        if (strncmp((const char *)need_retransmit_ctrl, "Y", 1) == 0) {
          ESP_LOGE(TAG,
                   "******************* <RETRANSMISSION> ********************");
          ESP_LOGW(TAG, "SENDING 'CTRL' MESSAGE - MSG_COUNTER_TX <%d>",
                   MSG_COUNTER_TX);
          ESP_LOGW(TAG, "retransmission_message_hex (ctrl): <%s>",
                   retransmission_message_hex);
          ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>",
                   xPortGetFreeHeapSize());
          ESP_LOGE(
              TAG,
              "******************* <RETRANSMISSION> ********************\n");
          //
          need_retransmit = "N";
          need_retransmit_ctrl = "N";
          //
          ///// VISUALIZE /////
          //
          gpio_set_level(LED_PIN, 1);
          lora_send(LORA_RX_ADDRESS, retransmission_message_hex); // sending
          is_ctrl_sent_ok = "N";
          is_sending_ctrl = "Y";
          // free(full_message_hex);
          vTaskDelay(pdMS_TO_TICKS(200));
          gpio_set_level(LED_PIN, 0);
          vTaskDelay(pdMS_TO_TICKS(200));
          //
          break;
        } // (strncmp((const char *)is_sending_ctrl, "Y", 1) == 0)

        if (strncmp((const char *)need_retransmit_data, "Y", 1) == 0) {
          ESP_LOGE(TAG,
                   "******************* <RETRANSMISSION> ********************");
          ESP_LOGW(TAG, "SENDING 'DATA' MESSAGE - MSG_COUNTER_TX <%d>",
                   MSG_COUNTER_TX);
          ESP_LOGW(TAG, "retransmission_message_hex (dta): <%s>",
                   retransmission_message_hex);
          ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>",
                   xPortGetFreeHeapSize());
          ESP_LOGE(
              TAG,
              "******************* <RETRANSMISSION> ********************\n");
          //
          need_retransmit = "N";
          need_retransmit_data = "N";
          //
          ///// VISUALIZE /////
          //
          gpio_set_level(LED_PIN, 1);
          lora_send(LORA_RX_ADDRESS, retransmission_message_hex); // sending
          is_data_sent_ok = "N";
          is_sending_data = "Y";
          // free(full_message_hex);
          vTaskDelay(pdMS_TO_TICKS(200));
          gpio_set_level(LED_PIN, 0);
          vTaskDelay(pdMS_TO_TICKS(200));
          //
          break;
        } // (strncmp((const char *)is_sending_ctrl, "Y", 1) == 0)
      }   // if retransmission needed
      ///// RETRANSMISSION *****************************************************
      ///// RETRANSMISSION *****************************************************
      ///// RETRANSMISSION *****************************************************
      //
      ///// CTRL ***************************************************************
      ///// CTRL ***************************************************************
      ///// CTRL ***************************************************************
      ///// FIRST -> need to tell the RX how many bits represent each sample for
      /// each axis (*_bits_tx)
      // x_bits_tx -> 0 ~ 255 (8 bits) (I supose 256 max number of bits)
      // y_bits_tx -> 0 ~ 255 (8 bits) (I supose 256 max number of bits)
      // z_bits_tx -> 0 ~ 255 (8 bits) (I supose 256 max number of bits)
      //
      // this info will be sent as 'ctrl' (01) type message
      // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      // |0 1 2 3 4 5 6 7|0 1 2 3 4 5 6 7|       <<HEADER>>
      // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      // |0 1|       Transaction ID      |
      // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      //
      //     x_bits_tx       y_bits_tx       z_bits_tx
      // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      // |0 1 2 3 4 5 6 7|0 1 2 3 4 5 6 7|0 1 2 3 4 5 6 7|     <<PAYLOAD...>>
      // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      //
      // |    32 bits    |    32 bits    |    32 bits    |
      // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      // |  min_x_value  |  min_y_value  |  min_z_value  |     <<...PAYLOAD>>
      // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      //
      // if is_ctrl_sent_ok == "N" it means we haven't send
      // the 'ctrl' message yet
      if (strncmp((const char *)is_ctrl_sent_ok, "N", 1) == 0) {
        //
        header_hex_of_data_to_send = add_header_hex(ctrl, MSG_COUNTER_TX);
        //
        // preparing hexadecimal format of data_to_send_hex
        // it will have:
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |      8 bits      |     8 bits       |     8 bits       |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |    x_bits_tx     |    y_bits_tx     |    z_bits_tx     |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |data_to_send_hex_0|data_to_send_hex_1|data_to_send_hex_2|
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |       0xFF       |       0xFF       |       0xFF       |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // ..........................................................
        //
        // ..........................................................
        // |     32 bits      |     32 bits      |     32 bits      |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |   min_x_value    |   min_y_value    |   min_z_value    |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |data_to_send_hex_3|data_to_send_hex_4|data_to_send_hex_5|
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |    0xFFFFFFFF    |    0xFFFFFFFF    |    0xFFFFFFFF    |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        //
        // 8 + 8 + 8 + 32 + 32 + 32 = 120 bits
        // 120 bits in hexadecimal it's like:
        // 0 x FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
        // ********** 30 hexadecimal characters **********
        //
        data_to_send_hex = malloc((30 + 1) * sizeof(*data_to_send_hex));
        if (data_to_send_hex == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(
              TAG,
              "Failed to allocate *data_to_send_hex in transmit_data_task");
        }
        //
        char *data_to_send_hex_0 =
            malloc((2 + 1) * sizeof(*data_to_send_hex_0));
        if (data_to_send_hex_0 == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(
              TAG,
              "Failed to allocate *data_to_send_hex_0 in transmit_data_task");
        }
        char *data_to_send_hex_1 =
            malloc((2 + 1) * sizeof(*data_to_send_hex_1));
        if (data_to_send_hex_1 == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(
              TAG,
              "Failed to allocate *data_to_send_hex_1 in transmit_data_task");
        }
        char *data_to_send_hex_2 =
            malloc((2 + 1) * sizeof(*data_to_send_hex_2));
        if (data_to_send_hex_2 == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(
              TAG,
              "Failed to allocate *data_to_send_hex_2 in transmit_data_task");
        }
        //
        DecimalToHexadecimal(x_bits_tx, data_to_send_hex_0);
        DecimalToHexadecimal(y_bits_tx, data_to_send_hex_1);
        DecimalToHexadecimal(z_bits_tx, data_to_send_hex_2);
        //
        //
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |     32 bits      |     32 bits      |     32 bits      |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |   min_x_value    |   min_y_value    |   min_z_value    |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |data_to_send_hex_3|data_to_send_hex_4|data_to_send_hex_5|
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |    0xFFFFFFFF    |    0xFFFFFFFF    |    0xFFFFFFFF    |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        char *data_to_send_hex_3 =
            malloc((8 + 1) * sizeof(*data_to_send_hex_3));
        if (data_to_send_hex_3 == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(
              TAG,
              "Failed to allocate *data_to_send_hex_3 in transmit_data_task");
        }
        char *data_to_send_hex_4 =
            malloc((8 + 1) * sizeof(*data_to_send_hex_4));
        if (data_to_send_hex_4 == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(
              TAG,
              "Failed to allocate *data_to_send_hex_4 in transmit_data_task");
        }
        char *data_to_send_hex_5 =
            malloc((8 + 1) * sizeof(*data_to_send_hex_5));
        if (data_to_send_hex_5 == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(
              TAG,
              "Failed to allocate *data_to_send_hex_5 in transmit_data_task");
        }
        //
        // to store temporarily the subdivisions of data_to_send_hex_3,
        // data_to_send_hex_4 and data_to_send_hex_5 into 8-bit words so we can
        // use DecimalToHexadecimal function to get the hexadecimal
        uint8_t tmp_8bits_section;
        char *tmp_8bits_section_hex =
            malloc((2 + 1) * sizeof(*tmp_8bits_section_hex));
        if (tmp_8bits_section_hex == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(TAG, "Failed to allocate *tmp_8bits_section_hex in "
                        "transmit_data_task");
        }
        //
        double resolution = accel_res(test_scale);
        int32_t min_x_value_int32 = min_x_value / resolution;
        int32_t min_y_value_int32 = min_y_value / resolution;
        int32_t min_z_value_int32 = min_z_value / resolution;
        ESP_LOGW(TAG, "!!!DEBUGGING!!! min_x_value_int32: %ld",
                 min_x_value_int32);
        ESP_LOGW(TAG, "!!!DEBUGGING!!! min_y_value_int32: %ld",
                 min_y_value_int32);
        ESP_LOGW(TAG, "!!!DEBUGGING!!! min_z_value_int32: %ld",
                 min_z_value_int32);
        uint8_t dayi = 3;
        //
        // the next 'for' loop composes the message like the following format
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |     32 bits      |     32 bits      |     32 bits      |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |   min_x_value    |   min_y_value    |   min_z_value    |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |data_to_send_hex_3|data_to_send_hex_4|data_to_send_hex_5|
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |  0 x FF FF FF FF |  0 x FF FF FF FF |  0 x FF FF FF FF |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        for (size_t i = 0; i < 8; i += 2) {
          // x_min_value
          tmp_8bits_section = ((min_x_value_int32) >> (dayi * 8)) & 0xFF;
          DecimalToHexadecimal(tmp_8bits_section, tmp_8bits_section_hex);
          tmp_8bits_section_hex[2] = NULL_END; // this line can be removed
          strcpy(data_to_send_hex_3 + i, tmp_8bits_section_hex);
          ESP_LOGW(TAG, "!!!DEBUGGING!!! data_to_send_hex_3[%zu]: %s", i,
                   data_to_send_hex_3 + i);
          // y_min_value
          tmp_8bits_section = ((min_y_value_int32) >> (dayi * 8)) & 0xFF;
          DecimalToHexadecimal(tmp_8bits_section, tmp_8bits_section_hex);
          tmp_8bits_section_hex[2] = NULL_END; // this line can be removed
          strcpy(data_to_send_hex_4 + i, tmp_8bits_section_hex);
          ESP_LOGW(TAG, "!!!DEBUGGING!!! data_to_send_hex_4[%zu]: %s", i,
                   data_to_send_hex_4 + i);
          // z_min_value
          tmp_8bits_section = ((min_z_value_int32) >> (dayi * 8)) & 0xFF;
          DecimalToHexadecimal(tmp_8bits_section, tmp_8bits_section_hex);
          tmp_8bits_section_hex[2] = NULL_END; // this line can be removed
          strcpy(data_to_send_hex_5 + i, tmp_8bits_section_hex);
          ESP_LOGW(TAG, "!!!DEBUGGING!!! data_to_send_hex_5[%zu]: %s", i,
                   data_to_send_hex_5 + i);
          //
          dayi--;
        }
        //
        // |                    data_to_send_hex                    |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |data_to_send_hex_0|data_to_send_hex_1|data_to_send_hex_2|
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        // |data_to_send_hex_3|data_to_send_hex_4|data_to_send_hex_5|
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
        strcpy(data_to_send_hex, data_to_send_hex_0);
        strcpy(data_to_send_hex + 2, data_to_send_hex_1);
        strcpy(data_to_send_hex + 4, data_to_send_hex_2);
        strcpy(data_to_send_hex + 6, data_to_send_hex_3);
        strcpy(data_to_send_hex + 14, data_to_send_hex_4);
        strcpy(data_to_send_hex + 22, data_to_send_hex_5);
        //
        data_to_send_hex[30] = NULL_END;
        //
        //
        full_message_hex = malloc((strlen(header_hex_of_data_to_send) +
                                   strlen(data_to_send_hex) + 1) *
                                  sizeof(*full_message_hex));
        if (full_message_hex == NULL) {
          ESP_LOGE(TAG, "NOT ENOUGH HEAP");
          ESP_LOGE(TAG, "Failed to allocate *full_message_hex in task "
                        "transmit_data_task");
        }
        //
        // if we haven't allocated space for 'retransmission_msg_defined' yet,
        // we allocate, so we don't need to allocate over and over again
        if (strncmp((const char *)retransmission_msg_defined, "N", 1) == 0) {
          // composing retransmission message to send (if needed)
          // retransmission_message_hex will be the current full_message_hex
          retransmission_message_hex =
              malloc((strlen(header_hex_of_data_to_send) +
                      strlen(data_to_send_hex) + 1) *
                     sizeof(*retransmission_message_hex));
          if (retransmission_message_hex == NULL) {
            ESP_LOGE(TAG, "NOT ENOUGH HEAP");
            ESP_LOGE(TAG,
                     "Failed to allocate *retransmission_message_hex in task "
                     "transmit_data_task");
          }
          retransmission_msg_defined = "Y";
        }
        //
        AppendString(header_hex_of_data_to_send, data_to_send_hex,
                     full_message_hex);
        free(header_hex_of_data_to_send);
        free(data_to_send_hex);
        //
        // copying 'full_message_hex' to 'retransmission_message_hex' to send
        // 'retransmission_message_hex' when retransmission needed
        strcpy(retransmission_message_hex, full_message_hex);
        //
        //
        ///// VISUALIZE /////
        ESP_LOGE(TAG,
                 "******************** <TRANSMISSION> *********************");
        ESP_LOGW(TAG, "SENDING 'CTRL' MESSAGE - MSG_COUNTER_TX <%d>",
                 MSG_COUNTER_TX);
        ESP_LOGW(TAG, "full_message_hex (ctrl): <%s>", full_message_hex);
        ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>",
                 xPortGetFreeHeapSize());
        ESP_LOGE(TAG,
                 "******************** <TRANSMISSION> *********************\n");
        ///// VISUALIZE /////
        //
        gpio_set_level(LED_PIN, 1);
        lora_send(LORA_RX_ADDRESS, full_message_hex); // sending
        is_ctrl_sent_ok = "N";
        is_sending_ctrl = "Y";
        free(full_message_hex);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        //
        break;
      } // (strncmp((const char *)is_ctrl_sent_ok, "N", 1) == 0)
      ///// CTRL ***************************************************************
      ///// CTRL ***************************************************************
      ///// CTRL ***************************************************************

      ///// DATA BLOCKING ******************************************************
      ///// DATA BLOCKING ******************************************************
      ///// DATA BLOCKING ******************************************************
      // if (is_data_sent_ok == "N") AND (is_ctrl_sent_ok == "+OK") it means
      // we already sent the needed 'ctrl' message and we can start
      // sending our block of data...
      //
      // making sure 'd_a' it's not superior to 'p'
      if (d_a < p) {
        if ((strncmp((const char *)is_data_sent_ok, "+OK", 3) == 0) &&
            (strncmp((const char *)is_ctrl_sent_ok, "+OK", 3) == 0)) {
          //
          ///// compising the header
          header_hex_of_data_to_send = add_header_hex(data, MSG_COUNTER_TX);
          //
          ///// composing the block of data to send (in binary)
          // (x_bits_tx + y_bits_tx + z_bits_tx) indicates how many bits
          // we need to send a 'xyz triplet'
          uint8_t xyz_bits_tx = x_bits_tx + y_bits_tx + z_bits_tx;
          //
          // max number of 'xyz triplets' we can send in ONE message
          uint8_t max_xyx_triplets_to_send =
              rylr998_payload_max_bits / xyz_bits_tx;
          //
          // |                    total_bits_tx_after_pad0                    |
          // +----+-------------+-------------+-------------+---+-------------+
          //  pad | xyz_bits_tx | xyz_bits_tx | xyz_bits_tx |...| xyz_bits_tx |
          // +----+-------------+-------------+-------------+---+-------------+
          // 0...0|  x - y - z  |  x - y - z  |  x - y - z  |...|  x - y - z  |
          // +----+-------------+-------------+-------------+---+-------------+
          // the above format, must be multiple of 8,
          // so we pad with '0'
          // so we make:
          uint16_t total_bits_tx_after_pad0 =
              (((xyz_bits_tx * max_xyx_triplets_to_send) / 8) + 1) * 8;
          //
          // total zeros added for padding so the message to send
          // is multiple of 8
          uint16_t padding_zeros = total_bits_tx_after_pad0 -
                                   (xyz_bits_tx * max_xyx_triplets_to_send);
          // allocate memory for the message to send, char type,
          // each character is one bit
          char *tmp_data_to_send_bin = malloc((total_bits_tx_after_pad0 + 1) *
                                              sizeof(*tmp_data_to_send_bin));
          if (tmp_data_to_send_bin != NULL) {
            ///// reset 'tmp_data_to_send_bin' to '0' **************************
            memset(tmp_data_to_send_bin, '0', total_bits_tx_after_pad0);
            tmp_data_to_send_bin[total_bits_tx_after_pad0] = NULL_END;
            ///// reset 'tmp_data_to_send_bin' to '0' **************************
          } else {
            ESP_LOGE(TAG, "NOT ENOUGH HEAP");
            ESP_LOGE(TAG, "Failed to allocate *tmp_data_to_send_bin in "
                          "transmit_data_task");
          }
          //
          // filling up 'tmp_data_to_send_bin' with the xyz-triplets from:
          //                                   -> x_samples_compressed_bin
          //                                   -> y_samples_compressed_bin
          //                                   -> z_samples_compressed_bin
          // the first (total_bits_tx_after_pad0 - xyz_bits_tx) bits will be
          // the padded '0'
          // we start iterating from the next position after the padded zeros
          // which is:
          // 'total_bits_tx_after_pad0' - 'xyz_bits_tx *
          // max_xyx_triplets_to_send' so we start from here:
          //
          // |                      tmp_data_to_send_bin                      |
          // +----+-------------+-------------+-------------+---+-------------+
          //  pad | xyz_bits_tx | xyz_bits_tx | xyz_bits_tx |...| xyz_bits_tx |
          // +----+-------------+-------------+-------------+---+-------------+
          // 0...0|  x - y - z  |  x - y - z  |  x - y - z  |...|  x - y - z  |
          // +----+-------------+-------------+-------------+---+-------------+
          //      ^                                                           ^
          // from | . . . . . . . . . . . . . . . . . . . . . . . . . . . to  |
          // here | . . . . . . . . . . . . . . . . . . . . . . . . . . .here |
          for (int i = padding_zeros; i < total_bits_tx_after_pad0;
               (i += xyz_bits_tx)) {
            strcpy(tmp_data_to_send_bin + i,
                   x_samples_compressed_bin[d_a]); // x data
            //
            strcpy(tmp_data_to_send_bin + (i + x_bits_tx),
                   y_samples_compressed_bin[d_a]); // y data
            //
            strcpy(tmp_data_to_send_bin + (i + x_bits_tx + y_bits_tx),
                   z_samples_compressed_bin[d_a]); // z data
            //
            d_a++;
            if (d_a == p) {
              break; // break from 'for' loop
            }
          }
          tmp_data_to_send_bin[total_bits_tx_after_pad0] =
              NULL_END; // ensure null-termination
          //
          ///// composing the block of data to send (in hexadecimal)
          // we are sending 'total_bits_tx_after_pad0' bits
          // it means a total of (total_bits_tx_after_pad0 / 8) 8-bit words
          // each of those 8-bit words will be converted to hexadecimal, which
          // means it will occupy 2 hexadecimal characters
          // so we multiply the number (total_bits_tx_after_pad0 / 8) by 2 to
          // get how many hexadecimal characters we need to compose the
          // 'data_to_send_hex' message
          uint16_t data_to_send_hex_characters =
              ((total_bits_tx_after_pad0 / 8) * 2) + 1;
          data_to_send_hex =
              malloc((data_to_send_hex_characters) * sizeof(*data_to_send_hex));
          if (data_to_send_hex == NULL) {
            ESP_LOGE(TAG, "NOT ENOUGH HEAP");
            ESP_LOGE(
                TAG,
                "Failed to allocate *data_to_send_hex in transmit_data_task");
          }
          //
          // segment size -> 8-bit words -> so 8 BITS
          const size_t segment_size = 8;
          // calculate number of 8-bit words needed
          const size_t num_segments = total_bits_tx_after_pad0 / segment_size;
          //
          // temporal vlaue to store each 8-bit segment
          tmp_segment_bin =
              malloc((segment_size + 1) * sizeof(*tmp_segment_bin));
          if (tmp_segment_bin == NULL) {
            ESP_LOGE(TAG, "NOT ENOUGH HEAP");
            ESP_LOGE(TAG, "Failed to allocate *tmp_segment_bin in "
                          "transmit_data_task");
          }
          //
          // temporal variable to store the conversion of
          // decimal tmp_segment_dec to hexadecimal
          tmp_segment_hex = malloc((2 + 1) * sizeof(*tmp_segment_hex));
          if (tmp_segment_hex == NULL) {
            ESP_LOGE(TAG, "NOT ENOUGH HEAP");
            ESP_LOGE(TAG, "Failed to allocate *tmp_segment_hex in "
                          "transmit_data_task");
          }
          //
          // ITERATE OVER THE SEGMENTS *****************************************
          for (size_t i = 0; i < num_segments; i++) {
            // copy next 8-character segment in the temporal variable
            strncpy(tmp_segment_bin, tmp_data_to_send_bin + i * segment_size,
                    segment_size);
            tmp_segment_bin[segment_size] = NULL_END; // null-end character
            //
            // getting the corresponding decimal value of the current
            // 8-character segment
            int tmp_segment_dec = BinaryToDecimal(tmp_segment_bin);
            //
            DecimalToHexadecimal(tmp_segment_dec, tmp_segment_hex);
            //
            // append every 'tmp_segment_hex' to 'data_to_send_hex'
            strcpy(data_to_send_hex + (i * 2), tmp_segment_hex);
          }
          data_to_send_hex[data_to_send_hex_characters] =
              NULL_END; // ensure null-termination
          //
          // free the allocated space for 'tmp_segment_hex' 'cause we don´t need
          // to use it anymore
          free(tmp_segment_hex);
          //
          free(tmp_segment_bin);
          //
          free(tmp_data_to_send_bin);
          //
          // composing full message to send:
          // full_message_hex = data_to_send_header_hex + data_to_send_hex
          full_message_hex = malloc((strlen(header_hex_of_data_to_send) +
                                     strlen(data_to_send_hex) + 1) *
                                    sizeof(*full_message_hex));
          if (full_message_hex == NULL) {
            ESP_LOGE(TAG, "NOT ENOUGH HEAP");
            ESP_LOGE(TAG, "Failed to allocate *full_message_hex in task "
                          "transmit_data_task");
          }
          //
          // if we haven't allocated space for 'retransmission_msg_defined' yet,
          // we allocate it, so we don't need to allocate over and over again
          if (strncmp((const char *)retransmission_msg_defined, "N", 1) == 0) {
            // composing retransmission message to send (if needed)
            // retransmission_message_hex will be the current full_message_hex
            retransmission_message_hex =
                malloc((strlen(header_hex_of_data_to_send) +
                        strlen(data_to_send_hex) + 1) *
                       sizeof(*retransmission_message_hex));
            if (retransmission_message_hex == NULL) {
              ESP_LOGE(TAG, "NOT ENOUGH HEAP");
              ESP_LOGE(TAG,
                       "Failed to allocate *retransmission_message_hex in task "
                       "transmit_data_task");
            }
            retransmission_msg_defined = "Y";
          }
          //
          AppendString(header_hex_of_data_to_send, data_to_send_hex,
                       full_message_hex);
          free(header_hex_of_data_to_send);
          free(data_to_send_hex);
          //
          // copying 'full_message_hex' to 'retransmission_message_hex' to send
          // 'retransmission_message_hex' when retransmission needed
          strcpy(retransmission_message_hex, full_message_hex);
          //
          ///// VISUALIZE /////
          ESP_LOGE(TAG,
                   "******************** <TRANSMISSION> *********************");
          ESP_LOGW(TAG, "SENDING 'BLOCKING' MESSAGE - MSG_COUNTER_TX <%d>",
                   MSG_COUNTER_TX);
          ESP_LOGW(TAG, "full_message_hex (blocking): <%s>", full_message_hex);
          ESP_LOGW(TAG, "!!!DEBUGGING!!! d_a: <%d>", d_a);
          ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>",
                   xPortGetFreeHeapSize());
          ESP_LOGE(
              TAG,
              "******************** <TRANSMISSION> *********************\n");
          ///// VISUALIZE /////
          //
          gpio_set_level(LED_PIN, 1);
          lora_send(LORA_RX_ADDRESS, full_message_hex); // sending
          is_data_sent_ok = "N";
          is_sending_data = "Y";
          free(full_message_hex);
          vTaskDelay(pdMS_TO_TICKS(200));
          gpio_set_level(LED_PIN, 0);
          vTaskDelay(pdMS_TO_TICKS(200));
        } // if ((strncmp((const char *)is_data_sent_ok, "+OK", 3) == 0) &&
        //  (strncmp((const char *)is_ctrl_sent_ok, "+OK", 3) == 0))
        break;
      } else { // if (d_a < p)
        // freeing up allocated memory **********************
        for (int i = 0; i < p; i++) {
          free(x_samples_compressed_bin[i]);
          free(y_samples_compressed_bin[i]);
          free(z_samples_compressed_bin[i]);
        }
        free(retransmission_message_hex);
        //
        retransmission_msg_defined = "N";
        // freeing up allocated memory **********************
        ESP_LOGE(TAG,
                 "******************** <APP FINISHED> *********************");
        ESP_LOGW(TAG, "!!!DEBUGGING!!! d_a: <%d>", d_a);
        ESP_LOGE(TAG,
                 "******************** <APP FINISHED> *********************");

        d_a = 0;
        break;
      } // esle if (d_a < p)
      ///// DATA BLOCKING ******************************************************
      ///// DATA BLOCKING ******************************************************
      ///// DATA BLOCKING ******************************************************

      /* ///// FOR TESTING PURPOSES, WE ARE SENDING A COUNTER AS DATA
      *************
      // converting the data_to_send from decimal to HEX
      //
      //// this is to know the number of digits the decimal number
      ///// will have when converted to hexadecimal
      int hex_digits = 0;
      uint16_t temp1 = MSG_COUNTER_TX;
      while (temp1 > 0) {
        temp1 /= 16;
        hex_digits++;
      }
      // the digits must be always multiple of 2
      // so it will be like 0x0F 0xFF 0x0FFF 0xFFFF and so on
      if (hex_digits % 2) {
        hex_digits++;
      }
      ///// this is to know the number of digits the decimal number
      ///// will have when converted to hexadecimal
      //
      // in this case, 'data_to_send_hex' is only 4 hexadecimal digits because
      // what we are sending is 'MSG_COUNTER_TX', which is 'uint16_t'
      char *data_to_send_hex_MSB =
          malloc((2 + 1) * sizeof(*data_to_send_hex_MSB));
      if (data_to_send_hex_MSB == NULL) {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *data_to_send_hex_MSB in function "
                      "transmit_data_task");
      }
      //
      char *data_to_send_hex_LSB =
          malloc((2 + 1) * sizeof(*data_to_send_hex_LSB));
      if (data_to_send_hex_LSB == NULL) {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *data_to_send_hex_LSB in function "
                      "transmit_data_task");
      }
      //
      data_to_send_hex = malloc((4 + 1) * sizeof(*data_to_send_hex));
      if (data_to_send_hex == NULL) {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *data_to_send_hex in task "
                      "transmit_data_task");
      }
      //
      // separate 'data_to_send' into 1-byte (8 bits) words
      uint8_t data_to_send_LSB = MSG_COUNTER_TX & 0xFF;
      uint8_t data_to_send_MSB = (MSG_COUNTER_TX >> 8) & 0xFF;
      // separate 'data_to_send' into 1-byte (8 bits) words
      //
      DecimalToHexadecimal(data_to_send_LSB, data_to_send_hex_LSB);
      DecimalToHexadecimal(data_to_send_MSB, data_to_send_hex_MSB);
      //
      AppendString(data_to_send_hex_MSB, data_to_send_hex_LSB,
                   data_to_send_hex);
      //
      free(data_to_send_hex_MSB);
      free(data_to_send_hex_LSB);
      ///// FOR TESTING PURPOSES, WE ARE SENDING A COUNTER AS DATA *************
    */

    } // while ((MSG_COUNTER_TX <= MAX_TRANSACTION_ID) && ((strncmp((const char
      // *)rcv_ack, "Y", 1) == 0) || (strncmp((const char
      // *)need_retransmit, "Y", 1) == 0)) && (strncmp((const char
      // *)is_data_sent_ok, "+OK", 3) == 0))
  }   // while (1)
} // transmit_data_task
///////////////////////////////////////////////////////////////////////////////
//********** Transmit data only if ACK received correctly by the RX *********//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//******* Check if last data message was received correctly by the RX *******//
///////////////////////////////////////////////////////////////////////////////
void check_ack_task(void *pvParameters) {
  while (1) {
    // Block to wait for data received (+RCV=) and check if ACK
    // Block indefinitely (without a timeout, so no need to check the function's
    // return value) to wait for a notification. Here the RTOS task notification
    // is being used as a binary semaphore, so the notification value is cleared
    // to zero on exit. NOTE! Real applications should not block indefinitely,
    // but instead time out occasionally in order to handle error conditions
    // that may prevent the interrupt from sending any more notifications.
    ulTaskNotifyTake(pdTRUE,         // Clear the notification value on exit
                     portMAX_DELAY); // Block indefinitely

    // Print out remaining task stack memory (words)
    ESP_LOGW(TAG, "Bytes free in 'check_ack_task' stack: <%zu>",
             uxTaskGetStackHighWaterMark(NULL));

    ///// extract the header from the incoming data /////
    char *header_hex = malloc((4 + 1) * sizeof(*header_hex));
    if (header_hex == NULL) {
      ESP_LOGE(TAG, "NOT ENOUGH HEAP");
      ESP_LOGE(TAG, "Failed to allocate *header_hex in task check_ack_task");
    }
    GetSubString(Lora_data.Data, 0, 4, header_hex);
    ///// extract the header from the incoming data /////

    ///// converting the extracted header to binary /////
    uint16_t header_dec = HexadecimalToDecimal(header_hex);

    // ***********************************************************************
    // 0xC000 -> 1100 0000 0000 0000 (first 2 bits are '11' which means 'ack')
    // so we compare the first 2 bits in the extracted header
    // and the last 14 bits compare to MSG_COUNTER_TX
    if (((header_dec & 0xC000) == 0xC000) &&
        ((header_dec & 0x3FFF) == MSG_COUNTER_TX)) {
      // reset the counter
      timer_expires_counter = 0;
      xTimerStop(resendTimer, 0);

      // set the flag to know we already received the 'ack' message
      rcv_ack = "Y";
      //
      ESP_LOGI(TAG,
               "'ACK' message corresponding to transaction ID <%u> RECEIVED",
               MSG_COUNTER_TX);

      if ((strncmp((const char *)is_sending_ctrl, "Y", 1) == 0)) {
        is_data_sent_ok = "+OK";

        // we reset the flag 'retransmission_msg_defined' becassue we
        // already sent the 'ctrl' message, and we are gonna send the block
        // of data after, so the 'retransmission_message_hex' will have
        // different length
        retransmission_msg_defined = "N";

        is_sending_ctrl = "N";
      }

      // set the flags to not retransmit
      need_retransmit = "N";
      need_retransmit_ctrl = "N";
      need_retransmit_data = "N";

      MSG_COUNTER_TX++;
      //
      ///// VISUALIZE /////
      ESP_LOGE(TAG,
               "******************** FREE HEAP MEMORY ********************");
      ESP_LOGW(TAG, "<%lu> BYTES", xPortGetFreeHeapSize());
      ESP_LOGE(TAG,
               "******************** FREE HEAP MEMORY ********************");
      ///// VISUALIZE /////
      //
      // Send a notification to transmit_data_task()
      // bringing it out of the 'Blocked' state
      vTaskDelay(pdMS_TO_TICKS(DELAY / 10));
      xTaskNotifyGive(transmit_data_task_handle);
    }
    //
    // ***********************************************************************
  }
}
///////////////////////////////////////////////////////////////////////////////
//******* Check if last data message was received correctly by the RX *******//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//**************** Task to receeive the data in the UART port ***************//
///////////////////////////////////////////////////////////////////////////////
void uart_task(void *pvParameters) {
  uart_event_t event;

  // data received in uart port which is what we receive from LoRa
  uint8_t *incoming_uart_data = (uint8_t *)malloc(BUF_SIZE);

  while (1) {
    // waiting for UART event
    if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {

      /* ESP_LOGE(TAG,
               "******************** <MEMORY CHECKING> *********************");
      ESP_LOGI(TAG, "Heap integrety %i",
               heap_caps_check_integrity_all(true));              // Added
      ESP_LOGI(TAG, "FreeHeapSize: %lu", xPortGetFreeHeapSize()); // Added
      ESP_LOGI(TAG, "Bytes free in stack: %zu",
               uxTaskGetStackHighWaterMark(NULL)); // Added
      ESP_LOGE(
          TAG,
          "******************** <MEMORY CHECKING> *********************\n"); */

      // ESP_LOGW(TAG, "!!!DEBUGGING!!! ENTERING 'xQueueReceive' in
      // 'uart_task'");

      bzero(incoming_uart_data, BUF_SIZE);

      // Print out remaining task stack memory (words)
      ESP_LOGW(TAG, "Bytes free in 'uart_task' stack: <%zu>",
               uxTaskGetStackHighWaterMark(NULL));
      switch (event.type) {
        // Event of UART receving data
        /*We'd better handler data event fast, there would be much more data
        events than other types of events. If we take too much time on data
        event, the queue might be full.*/
      case UART_DATA:
        uart_read_bytes(UART_NUM, incoming_uart_data, event.size,
                        pdMS_TO_TICKS(100));

        ESP_ERROR_CHECK(uart_flush(UART_NUM));

        ESP_LOGI(TAG, "Data received from LoRa module: %s", incoming_uart_data);

        // if the module answers +OK, we check if we were sending:
        // >> 'data' message type
        // or
        // >> 'ctrl' message type
        // so we wait for the 'ack' message type
        //
        if ((strncmp((const char *)incoming_uart_data, "+OK", 3) == 0) &&
            (strncmp((const char *)is_rylr998_module_init, "Y", 1) == 0)) {
          //
          // are we sending 'data' type message???
          if (strncmp((const char *)is_sending_data, "Y", 1) == 0) {
            is_data_sent_ok = "+OK";

            // is_sending_data = "N";
          }
          //
          // are we sending 'ctrl' type message???
          if (strncmp((const char *)is_sending_ctrl, "Y", 1) == 0) {
            is_ctrl_sent_ok = "+OK";

            // is_sending_ctrl = "N";
          }
          //
          // we wait for 'ack'
          rcv_ack = "N";
          //
          ///// start timer /////
          xTimerStart(resendTimer, 0);
          ///// start timer /////
        }
        ///// if the module answers +OK and we are sending data /////

        ///// if the module is receiving data, we proccess it /////
        // +RCV=22,length,data,RSSI,SNR
        // extract the components of the received message
        if ((strncmp((const char *)incoming_uart_data, "+RCV=", 5) == 0) &&
            (strncmp((const char *)is_rylr998_module_init, "Y", 1) == 0)) {
          char *token = strtok((char *)incoming_uart_data, "=");
          // loop through the string to extract all other tokens
          uint8_t count_token = 0;
          while (token != NULL) {
            token = strtok(NULL, ",");
            count_token++;
            switch (count_token) {
            case 1:
              Lora_data.Address = token;
              break;
            case 2:
              Lora_data.DataLength = token;
              break;
            case 3:
              Lora_data.Data = token;
              break;
            case 4:
              Lora_data.SignalStrength = token;
              break;
            case 5:
              Lora_data.SignalNoise = token;
              // Send a notification to check_ack_task(), bringing it out of the
              // Blocked state
              vTaskDelay(pdMS_TO_TICKS(DELAY / 50));
              xTaskNotifyGive(check_ack_task_handle);
              break;
            default:
              break;
            }
          }
        }
        ///// if the module is receiving data, we proccess it /////
        break;

      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control
        // for your application. The ISR has already reset the rx FIFO, As an
        // example, we directly flush the rx buffer here in order to read more
        // data.
        uart_flush_input(UART_NUM);
        xQueueReset(uart_queue);
        break;

      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider increasing your buffer
        // size As an example, we directly flush the rx buffer here in order to
        // read more data.
        uart_flush_input(UART_NUM);
        xQueueReset(uart_queue);
        break;

      // Others
      default:
        ESP_LOGI(TAG, "UART event type: %d", event.type);
        break;
      }
    }
  }
  free(incoming_uart_data);
  incoming_uart_data = NULL;
  vTaskDelete(NULL);
}
///////////////////////////////////////////////////////////////////////////////
//**************** Task to receeive the data in the UART port ***************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//*************************** UART Initialization ***************************//
///////////////////////////////////////////////////////////////////////////////
void init_uart(void) {
  uart_config_t uart_config = {
      .baud_rate = RYLR998_UART_BAUD_RATE,
      .data_bits = RYLR998_UART_DATA_BITS,
      .parity = RYLR998_UART_PARITY,
      .stop_bits = RYLR998_UART_STOP_BITS,
      .flow_ctrl = RYLR998_UART_FLOW_CTRL,
      .source_clk = RYLR998_UART_SOURCE_CLK,
  };

  // Configure UART parameters
  // uart_param_config(uart_port_t uart_num, const uart_config_t
  // *uart_config);
  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

  // Set UART pins
  // uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num,
  // int rts_io_num, int cts_io_num);
  // When calling 'uart_set_pin', instead of GPIO number, `UART_PIN_NO_CHANGE`
  // can be provided to keep the currently allocated pin.
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, RYLR998_UART_TX_GPIO_NUM,
                               RYLR998_UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));

  // Install UART driver
  ESP_ERROR_CHECK(
      uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 20, &uart_queue, 0));

  ESP_LOGI(TAG, "UART Interface Configuration !!!COMPLETED!!!");
}
///////////////////////////////////////////////////////////////////////////////
//*************************** UART Initialization ***************************//
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//******************************** Main Task *********************************//
////////////////////////////////////////////////////////////////////////////////
void app_main(void) {
  init_led();
  ESP_LOGI(TAG, "Waiting 50 ms");
  vTaskDelay(pdMS_TO_TICKS(50));

  ESP_LOGI(TAG, "Initializing file system...");
  ESP_ERROR_CHECK(init_file_system());
  ESP_LOGI(TAG, "Waiting 50 ms");
  vTaskDelay(pdMS_TO_TICKS(50));

  init_uart();
  ESP_LOGI(TAG, "Waiting 50 ms");
  vTaskDelay(pdMS_TO_TICKS(50));
  xTaskCreate(uart_task, "uart_task", TASK_MEMORY * 2, NULL, 12, NULL);
  ESP_LOGI(TAG, "Task 'uart_task' !!!CREATED!!!");
  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After 'uart_task' created");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");

  //************************************************************************//
  ///// tasks creation /////
  xTaskCreatePinnedToCore(xyz_data_reading_task, "xyz_data_reading_task",
                          TASK_MEMORY * 2, NULL, 10,
                          &xyz_data_reading_task_handle, tskNO_AFFINITY);
  ESP_LOGI(TAG, "Task 'xyz_data_reading_task' !!!CREATED!!!");
  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After 'xyz_data_reading_task' created");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");

  xTaskCreatePinnedToCore(compressing_samples_task, "compressing_samples_task",
                          TASK_MEMORY * 4, NULL, 10,
                          &compressing_samples_task_handle, tskNO_AFFINITY);
  ESP_LOGI(TAG, "Task 'compressing_samples_task' !!!CREATED!!!");
  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After 'compressing_samples_task' created");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");

  xTaskCreatePinnedToCore(check_ack_task, "check_ack_task", TASK_MEMORY * 2,
                          NULL, 10, &check_ack_task_handle, tskNO_AFFINITY);
  ESP_LOGI(TAG, "Task 'check_ack_task' !!!CREATED!!!");
  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After 'check_ack_task' created");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");

  xTaskCreatePinnedToCore(transmit_data_task, "transmit_data_task",
                          TASK_MEMORY * 2, NULL, 10, &transmit_data_task_handle,
                          tskNO_AFFINITY);
  ESP_LOGI(TAG, "Task 'transmit_data_task' !!!CREATED!!!");
  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After 'transmit_data_task' created");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");
  ///// tasks creation /////
  //************************************************************************//

  //************************************************************************//
  // building random indices array (this needs to go in the sensing task)
  // ESP_LOGI(TAG, "DEBUGGING  building random samples array perm");
  rnd_index_arr = (int *)malloc(p * sizeof(rnd_index_arr));

  ESP_LOGW(TAG, "*********************************************************");
  ESP_LOGW(TAG, "After rnd_index_arr allocation");
  ESP_LOGW(TAG, "Free heap memmory (bytes): <%lu>", xPortGetFreeHeapSize());
  ESP_LOGW(TAG, "*********************************************************");

  if (rnd_index_arr == NULL) {
    ESP_LOGE(TAG, "NOT ENOUGH HEAP");
    ESP_LOGE(TAG, "Failed to allocate *perm");
  } else {
    ESP_ERROR_CHECK(p_rnd_samples(rnd_index_arr, N, p));
  }
  //  building random indices array (this needs to go in the sensing task)
  //************************************************************************//

  init_rylr998_module();
  ESP_LOGI(TAG, "Waiting 50 ms");
  vTaskDelay(pdMS_TO_TICKS(50));

  ESP_ERROR_CHECK(set_timer_rylr998());

  ESP_ERROR_CHECK(init_2d_arrays());

  ESP_ERROR_CHECK(init_spi_adxl355());
  ESP_LOGI(TAG, "SPI ADXL355 Accelerometer INITIALIZED!!!");

  ESP_LOGI(TAG, "Stopping accelerometer, Measure OFF");
  ESP_ERROR_CHECK(adxl355_measure_off(&adxl355_accel_handle));
  ESP_LOGI(TAG, "Accelerometer STOPPED (OFF)!!!");

  ESP_LOGI(TAG, "Waiting 2s");
  vTaskDelay(pdMS_TO_TICKS(2000));

  ///// esp periodic timer /////
  // Periodic Timer to tick
  const esp_timer_create_args_t xyz_data_reading_tmr_args = {
      .callback = &xyz_data_reading_tmr_callback, .name = "XYZ Data Reg Timer"};
  ESP_ERROR_CHECK(
      esp_timer_create(&xyz_data_reading_tmr_args, &xyz_data_reading_tmr_hndl));
  ///// esp periodic timer /////

  ESP_LOGI(TAG, "Configuring ADXL355 with Frequency and Scale");
  ESP_ERROR_CHECK(
      adxl355_config_start(&adxl355_accel_handle, test_freq, test_scale));
  ESP_LOGI(TAG, "Waiting 100ms");
  vTaskDelay(pdMS_TO_TICKS(100));

  // running the periodic time
  ESP_LOGI(TAG, "Starting periodic timer");
  ESP_ERROR_CHECK(esp_timer_start_periodic(xyz_data_reading_tmr_hndl,
                                           XYZ_DATA_READ_PERIOD_MS * 1000));
  gpio_set_level(LED_PIN, 1);

  ESP_LOGI(TAG, "Turning ON Measurement Mode ADXL355");
  ESP_ERROR_CHECK(adxl355_measure_on(&adxl355_accel_handle));
  // ESP_LOGI(TAG, "Waiting 100ms");
  // vTaskDelay(pdMS_TO_TICKS(100));
}
////////////////////////////////////////////////////////////////////////////////
//******************************** Main Task *********************************//
////////////////////////////////////////////////////////////////////////////////
#include "rylr998.h"
#include "numerical_systems_conv.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "SHM SENSOR NODE -> RYLR998";

//***************************************************************************//
//********************* LoRa Data Message Header Format *********************//
//***************************** (designed by me) ****************************//
//***************************************************************************//
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |     byte 1    |    byte 2     |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// |0 1 2 3 4 5 6 7|0 1 2 3 4 5 6 7|
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// | T |       Transaction ID      |
// +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

// T -> message type (reset, control, data, ack)
// Transaction ID -> counter of the message to send
//***************************************************************************//
//********************* LoRa Data Message Header Format *********************//
//***************************** (designed by me) ****************************//
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

///////////////////////////////////////////////////////////////////////////////
//********************** RYLR998 Module Initialization **********************//
///////////////////////////////////////////////////////////////////////////////
void init_rylr998_module(void)
{
    ESP_LOGW(TAG, "!!!DEBUGGING!!! ENTERING 'init_rylr998_module'...");
    // ESP_LOGI(TAG, "Sending Factory Command to LoRa Module...\n");
    // char *factory_command = "AT+FACTORY\r\n";
    //  int err_uart_factory_command =
    // uart_write_bytes(UART_NUM, (const char *)factory_command,
    // strlen(factory_command));
    //  Returns
    //  (-1) Parameter error
    //  OTHERS(>= 0) The number of bytes pushed to the TX FIFO
    /* if (err_uart_factory_command != -1) {
      ESP_LOGI(TAG, "Factory Command -> !!!SENT SUCCESSFULLY!!!");
    } else {
      ESP_LOGE(TAG, "Factory Command -> ***SENDING FAILED***");
    } */
    // vTaskDelay(pdMS_TO_TICKS(DELAY));

    char *data_config = "";

    // 0
    ESP_LOGI(TAG, "Sending <AT> Command to LoRa Module...\n");
    char *test_command = "AT\r\n";
    uart_write_bytes(UART_NUM, (const char *)test_command, strlen(test_command));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // 0
    ESP_LOGI(TAG, "Sending Reset Command to LoRa Module...\n");
    char *reset_command = "AT+RESET\r\n";
    uart_write_bytes(UART_NUM, (const char *)reset_command,
                     strlen(reset_command));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // 1
    ESP_LOGI(TAG, "Sending Address Config Command to LoRa Module...\n");
    char *param_add = "AT+ADDRESS=" LORA_ADDRESS "\r\n";
    uart_write_bytes(UART_NUM, (const char *)param_add, strlen(param_add));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data_config = "AT+ADDRESS?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // 2
    ESP_LOGI(TAG, "Sending Network ID Config Command to LoRa Module...\n");
    char *param_net_id = "AT+NETWORKID=" LORA_NETWORK_ID "\r\n";
    uart_write_bytes(UART_NUM, (const char *)param_net_id, strlen(param_net_id));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data_config = "AT+NETWORKID?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // 3
    ESP_LOGI(TAG, "Sending Band Config Command to LoRa Module...\n");
    char *param_band = "AT+BAND=" LORA_BAND "\r\n";
    uart_write_bytes(UART_NUM, (const char *)param_band, strlen(param_band));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data_config = "AT+BAND?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // 4
    char param_set[30];
    snprintf(param_set, 25, "AT+PARAMETER=%d,%d,%d,%d\r\n", LORA_SPREADING_FACTOR,
             LORA_BANDWIDTH, LORA_CODING_RATE, LORA_PREAMBLE);
    // char *param_set = "AT+PARAMETER=" LORA_PARAMETERES "\r\n";
    ESP_LOGI(TAG, "Sending <%s> Config Command to LoRa Module...\n", param_set);
    uart_write_bytes(UART_NUM, (const char *)param_set, strlen(param_set));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data_config = "AT+PARAMETER?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    /* ESP_LOGI(TAG, "Sending IPR Baud Rate Command to LoRa Module...\n");
    char *ipr_command = "AT+IPR=" LORA_BAUD_RATE "\r\n";
    uart_write_bytes(UART_NUM, (const char *)ipr_command, strlen(ipr_command));

    data_config = "AT+IPR?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY)); */

    // 5
    ESP_LOGI(TAG, "Sending Domain Password Config Command to LoRa Module...\n");
    char *param_pass = "AT+CPIN=" LORA_DOMAIN_PASS "\r\n";
    uart_write_bytes(UART_NUM, (const char *)param_pass, strlen(param_pass));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    data_config = "AT+CPIN?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // 6
    /* ESP_LOGI(TAG, "Sending RF Output Power Command to LoRa Module...\n");
    char *rf_output_power = "AT+CRFOP=" LORA_RF_OUTPUT_POWER "\r\n";
    uart_write_bytes(UART_NUM, (const char *)rf_output_power,
                     strlen(rf_output_power));
    vTaskDelay(pdMS_TO_TICKS(DELAY)); */
    data_config = "AT+CRFOP?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // 7
    ESP_LOGI(TAG, "Sending Mode Command to LoRa Module...\n");
    char *mode_command = "AT+MODE=" LORA_MODE_SLEEP "\r\n";
    uart_write_bytes(UART_NUM, (const char *)mode_command,
                     strlen(mode_command));

    data_config = "AT+MODE?\r\n";
    uart_write_bytes(UART_NUM, (const char *)data_config, strlen(data_config));
    vTaskDelay(pdMS_TO_TICKS(DELAY));

    // DONE
    ESP_LOGI(TAG, "RYLR998 Module Initialized !!!COMPLETED!!!\n");
    is_rylr998_module_init = true;
    ESP_LOGW(TAG, "!!!DEBUGGING!!! LEAVING 'init_rylr998_module'...");
}
///////////////////////////////////////////////////////////////////////////////
//********************** RYLR998 Module Initialization **********************//
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//********************** Send data through LoRa module ***********************//
////////////////////////////////////////////////////////////////////////////////
void lora_send(int address, const char *data, gpio_num_t gpio_pin)
{
    char command_data[300] = "";
    char command[] = "AT+SEND=";
    char separator[] = ",";
    char end_of_line[] = "\r\n";
    // "AT+SEND=2,11,000|255|000"
    snprintf(command_data, 299, "%s%d%s%d%s%s%s", command, address, separator,
             strlen(data), separator, data, end_of_line);
    ESP_LOGI(TAG, "Sending to LoRa module command_data: <%s>", command_data);

    int bytes_pushed_to_tx_fifo;

    // ESP_LOGW(TAG, "!!!DEBUGGING!!! Before 'uart_write_bytes' in 'lora_send'
    // function");
    gpio_set_level(gpio_pin, 1);
    bytes_pushed_to_tx_fifo = uart_write_bytes(
        UART_NUM, (const char *)command_data, strlen(command_data));
    ESP_LOGW(TAG, "!!!DEBUGGING!!! bytes_pushed_to_tx_fifo= <%d>",
             bytes_pushed_to_tx_fifo);
    // ESP_LOGW(TAG, "!!!DEBUGGING!!! After 'uart_write_bytes' in 'lora_send'
    // function\n");

    // vTaskDelay(pdMS_TO_TICKS(DELAY * 2));
}
////////////////////////////////////////////////////////////////////////////////
//********************** Send data through LoRa module ***********************//
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//************ Add header in HEX to the data message to transmit ************//
///////////////////////////////////////////////////////////////////////////////
char *add_header_hex(msg_type message_type, const uint16_t transactionID)
{
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // |     byte 1    |    byte 2     |
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // |0 1 2 3 4 5 6 7|0 1 2 3 4 5 6 7|
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // | T |       Transaction ID      |
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

    /* 2 bits: message type (T)
       14 bits: transaction ID
       header length: 16 bits -> 2 bytes (0xFFFF)*/

    // when declaring a variable, make sure to initialize the variable,
    // to avoid warnings
    uint16_t msg_type_tmp = 0;
    switch (message_type)
    {
    case rst:
        msg_type_tmp = 0; // 0000 0000 0000 0000
        break;
    case ctrl:
        msg_type_tmp = 16384; // 0100 0000 0000 0000
        break;
    case data:
        msg_type_tmp = 32768; // 1000 0000 0000 0000
        break;
    case ack:
        msg_type_tmp = 49152; // 1100 0000 0000 0000
        break;
    default:
        break;
    }

    uint16_t header_decimal;
    header_decimal = msg_type_tmp | transactionID;

    // allocating memory for the header hexadecimal string
    // ALWAYS 4 digits: 0xAAAA
    char *header_hex_MSB = malloc((2 + 1) * sizeof(*header_hex_MSB));
    if (header_hex_MSB == NULL)
    {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *header_hex_MSB in function "
                      "add_header_hex");
    }

    char *header_hex_LSB = malloc((2 + 1) * sizeof(*header_hex_LSB));
    if (header_hex_LSB == NULL)
    {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *header_hex_LSB in function "
                      "add_header_hex");
    }

    char *header_hex = malloc((4 + 1) * sizeof(*header_hex));
    if (header_hex == NULL)
    {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *header_hex in function "
                      "add_header_hex");
    }

    // separate 'header_decimal' into 1-byte (8 bits) words
    uint8_t header_decimal_LSB = header_decimal & 0xFF;
    uint8_t header_decimal_MSB = (header_decimal >> 8) & 0xFF;
    // separate 'header_decimal' into 1-byte (8 bits) words

    DecimalToHexadecimal(header_decimal_LSB, header_hex_LSB);
    DecimalToHexadecimal(header_decimal_MSB, header_hex_MSB);

    AppendString(header_hex_MSB, header_hex_LSB, header_hex);

    free(header_hex_MSB);
    free(header_hex_LSB);

    return header_hex;
}
///////////////////////////////////////////////////////////////////////////////
//************ Add header in HEX to the data message to transmit ************//
///////////////////////////////////////////////////////////////////////////////

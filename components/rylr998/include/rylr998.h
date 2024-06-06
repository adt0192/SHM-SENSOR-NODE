#ifndef RYLR_998_H
#define RYLR_998_H

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// #define SND_MSG_PIN GPIO_NUM_2

extern bool is_rylr998_module_init; // to know if we already configured the
                                    // LoRa module so if we recieve "+OK" we
                                    // can discriminate if we are sending data
                                    // or still configuring the mpdule

//***************************************************************************//
//*************************** RYLR998 UART CONFIG ***************************//
//***************************************************************************//
#define RYLR998_UART_BAUD_RATE 115200
#define RYLR998_UART_DATA_BITS UART_DATA_8_BITS         // uart_types.h
#define RYLR998_UART_PARITY UART_PARITY_DISABLE         // uart_types.h
#define RYLR998_UART_STOP_BITS UART_STOP_BITS_1         // uart_types.h
#define RYLR998_UART_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE // uart_types.h
#define RYLR998_UART_SOURCE_CLK UART_SCLK_APB           // uart_types.h->clk_tree_defs.h
#define RYLR998_UART_TX_GPIO_NUM GPIO_NUM_17
#define RYLR998_UART_RX_GPIO_NUM GPIO_NUM_18
#define UART_NUM UART_NUM_1 // UART port
//***************************************************************************//
//*************************** RYLR998 UART CONFIG ***************************//
//***************************************************************************//

//***************************************************************************//
//************************ RYLR998 CONFIG PARAMETERS ************************//
//***************************************************************************//
#define LORA_BAND "868000000" // Frequency Band
#define LORA_NETWORK_ID "18"  // ID of LoRa Network
#define LORA_ADDRESS "2"      // Address of this module
#define LORA_RX_ADDRESS 22    // Address of receiver module
#define LORA_MODE_TX_RX "0"   // 0：Transceiver mode
#define LORA_MODE_SLEEP "1"   // 1：Sleep mode.
#define LORA_BAUD_RATE "115200"
#define LORA_SPREADING_FACTOR 8 // 5 to 11
#define LORA_BANDWIDTH 7        // 7: 125 KHz 8: 250 KHz 9: 500 KHz
#define LORA_CODING_RATE 1      // 1 to 4
#define LORA_PREAMBLE \
    12 // When NETWORKID=18, The value can be configured to 4~24. Other NETWORKID
       // can only be configured to 12.
// #define LORA_PARAMETERES "7,7,1,12" // AT+PARAMETER=<Spreading Factor>,
// <Bandwidth>,<Coding Rate>, <Programmed Preamble>
#define LORA_DOMAIN_PASS "44794C61"
#define LORA_RF_OUTPUT_POWER "22"
//***************************************************************************//
//************************ RYLR998 CONFIG PARAMETERS ************************//
//***************************************************************************//

//***************************************************************************//
//*************************** RYLR998 PAYLOAD INFO **************************//
//***************************************************************************//
// RYLR998 -> 240 bytes MAX of payload (ASCII format)
//         -> this means 240 characters (each character is 8-bits long)
//
// >> the header will have 16 bits -> 0xFFFF (4 characters, so 4 bytes)
// >> so there are 236 bytes left (236 ASCII characters)
//    -> which is 236 hexadecimal characters (e.g. 0xFFFF...FF)
//    -> every 2 hex-characters we are encoding <8-bits word>
//       so we have 236 / 2 = ((118 8-bit words))
//       118 * 8 = 944
//    -> so we have (944 BITS) of total space to send ONE MESSAGE WITH RYLR998
static const uint16_t rylr998_payload_max_bits = 944;
//***************************************************************************//
//*************************** RYLR998 PAYLOAD INFO **************************//
//***************************************************************************//

//***************************************************************************//
//*********************** LoRa received message format **********************//
//***************************************************************************//
typedef struct LoraData
{
    char *Address;
    char *DataLength;
    char *Data;
    char *SignalStrength;
    char *SignalNoise;
} Lora_Data_t;
//***************************************************************************//
//*********************** LoRa received message format **********************//
//***************************************************************************//

//***************************************************************************//
//********************* Messages types to send in header ********************//
//***************************************************************************//
typedef enum
{
    rst,  // 00 binary
    ctrl, // 01 binary
    data, // 10 binary
    ack   // 11 binary
} msg_type;
//***************************************************************************//
//********************* Messages types to send in header ********************//
//***************************************************************************//

//***************************************************************************//
//***************************** GLOBAL VARIABLES ****************************//
//***************************************************************************//
#define MAX_TRANSACTION_ID 16383 // 14 bits

#define LORA_PKT_TOA = 500 // Time on Air 500ms
#define BUF_SIZE 1024
#define DELAY 500

// static volatile uint16_t val;
//***************************************************************************//
//***************************** GLOBAL VARIABLES ****************************//
//***************************************************************************//

void init_rylr998_module(void);

void lora_send(int address, const char *data, gpio_num_t gpio_pin);

char *add_header_hex(msg_type message_type, const uint16_t transactionID);

#endif /* RYLR_998_H */
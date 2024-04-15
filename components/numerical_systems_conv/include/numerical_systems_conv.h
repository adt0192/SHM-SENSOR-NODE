#ifndef __NUMERICAL_SYS_CONV_H
#define __NUMERICAL_SYS_CONV_H

#include "esp_err.h"
#include "esp_log.h"

esp_err_t GetSubString(char *str, int index, int count, char *subStr);

esp_err_t AppendString(const char *str1, const char *str2, char *str0);

esp_err_t CharToString(char c, char *str);

esp_err_t InsertString(char *str, int index, char *subStr, char *str_concat);

int BinaryToDecimal(char *bin);

esp_err_t DecimalToBinary(uint32_t decimal, uint32_t bits, char *binaryString);

esp_err_t DecimalToHexadecimal(int decimal, char *hexString);

uint16_t HexadecimalToDecimal(char *hex);

#endif /* __NUMERICAL_SYS_CONV_H */
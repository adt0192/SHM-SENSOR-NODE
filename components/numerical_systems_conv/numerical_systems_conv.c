#include "numerical_systems_conv.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // required for memcpy()

static const char *TAG = "SHM SENSOR NODE -> NUMB SYSTEM CONVERSION";

///////////////////////////////////////////////////////////////////////////////
//***************************** Common Functions ****************************//
///////////////////////////////////////////////////////////////////////////////
esp_err_t GetSubString(char *str, int index, int count, char *subStr)
{
    int strLen = strlen(str);
    int lastIndex = index + count;

    if (index >= 0 && lastIndex > strLen)
    {
        subStr = "";
        return ESP_OK;
    }

    for (int i = 0; i < count; i++)
    {
        subStr[i] = str[index + i];
    }

    subStr[count] = '\0';

    return ESP_OK;
}

esp_err_t AppendString(const char *str1, const char *str2, char *str0)
{
    int str1Len = strlen(str1);
    int str2Len = strlen(str2);
    int str0Len = str1Len + str2Len + 1;

    for (int i = 0; i < str1Len; i++)
        str0[i] = str1[i];

    for (int i = 0; i < str2Len; i++)
        str0[(str1Len + i)] = str2[i];

    str0[str0Len - 1] = '\0';

    return ESP_OK;
}

esp_err_t CharToString(char c, char *str)
{
    str[0] = c;
    str[1] = '\0';

    return ESP_OK;
}

esp_err_t InsertString(char *str, int index, char *subStr, char *str_concat)
{
    char *s0 = malloc((strlen(str) - index + 1) * sizeof(*s0));
    if (s0 == NULL)
    {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *s0 in function InsertString");
        return ESP_FAIL;
    }
    GetSubString(str, index, strlen(str) - index, s0);

    char *s1 = malloc((index + 1) * sizeof(*s1));
    if (s1 == NULL)
    {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *s1 in function InsertString");
        return ESP_FAIL;
    }
    GetSubString(str, 0, index, s1);

    char *s2 = malloc((strlen(s1) + strlen(subStr) + 1) * sizeof(*s2));
    if (s2 == NULL)
    {
        ESP_LOGE(TAG, "NOT ENOUGH HEAP");
        ESP_LOGE(TAG, "Failed to allocate *s2 in function InsertString");
        return ESP_FAIL;
    }
    AppendString(s1, subStr, s2);

    AppendString(s2, s0, str_concat);

    free(s0);
    free(s1);
    free(s2);

    return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////
//***************************** Common Functions ****************************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//**************************** Binary To Decimal ****************************//
///////////////////////////////////////////////////////////////////////////////
int BinaryToDecimal(char *bin)
{
    int binLength = strlen(bin);
    double dec = 0;

    for (int i = 0; i < binLength; ++i)
    {
        dec += (bin[i] - 48) * pow(2, ((binLength - i) - 1));
    }

    return (int)dec;
}
///////////////////////////////////////////////////////////////////////////////
//**************************** Binary To Decimal ****************************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//**************************** Decimal To Binary ****************************//
///////////////////////////////////////////////////////////////////////////////
esp_err_t DecimalToBinary(uint32_t decimal, uint32_t bits, char *binaryString)
{
    for (int i = bits - 1; i >= 0; i--)
    {
        binaryString[i] = (decimal % 2) + '0';
        decimal /= 2;
    }

    // adding NULL character to the end of the string
    binaryString[bits] = '\0';

    return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////
//**************************** Decimal To Binary ****************************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//***************************** Decimal To HEX ******************************//
///////////////////////////////////////////////////////////////////////////////
esp_err_t DecimalToHexadecimal(int decimal, char *hexString)
{
    int i;
    for (i = 2 - 1; i >= 0; i--)
    {
        int remainder = decimal % 16;
        if (remainder < 10)
        {
            hexString[i] = remainder + '0';
        }
        else
        {
            hexString[i] = remainder - 10 + 'A';
        }
        decimal /= 16;
    }

    // adding NULL character to the end of the string
    hexString[2] = '\0';

    return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////
//***************************** Decimal To HEX ******************************//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//***************************** HEX To Decimal ******************************//
///////////////////////////////////////////////////////////////////////////////
uint16_t HexadecimalToDecimal(char *hex)
{
    int hexLength = strlen(hex);
    double dec = 0;

    for (int i = 0; i < hexLength; ++i)
    {
        char b = hex[i];

        if (b >= 48 && b <= 57)
            b -= 48;
        else if (b >= 65 && b <= 70)
            b -= 55;

        dec += b * pow(16, ((hexLength - i) - 1));
    }

    return (uint16_t)dec;
}
///////////////////////////////////////////////////////////////////////////////
//***************************** HEX To Decimal ******************************//
///////////////////////////////////////////////////////////////////////////////
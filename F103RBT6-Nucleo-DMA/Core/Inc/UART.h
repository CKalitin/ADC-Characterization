#include "main.h"

// Note that UART_HandleTypeDef and ADC_HandleTypeDef are parameters so that we can change if we use UART1, UART2, etc. or ADC1, ADCx, etc.
void Continue_On_UART_Receive(UART_HandleTypeDef);
void Send_ADC_Values_Over_UART(UART_HandleTypeDef, int, int, int);

// Variable declarations (extern = declaration, definition in .c file)
extern char tx_buff[100];
extern char rx_buff[100];

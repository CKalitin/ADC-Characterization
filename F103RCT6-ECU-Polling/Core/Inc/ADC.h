#include "main.h"

// Note that UART_HandleTypeDef and ADC_HandleTypeDef are parameters so that we can change if we use UART1, UART2, etc. or ADC1, ADCx, etc.
void Get_Averaged_ADC_Values(ADC_HandleTypeDef, int, int, int*, int*, int*);

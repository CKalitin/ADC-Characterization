#include "stm32f1xx_hal.h"
#include <stdint.h>

#define ADC_BUFFER_LENGTH 50

// Variable declarations (extern = declaration, definition in .c file)
extern uint16_t adcBuffer[ADC_BUFFER_LENGTH];
extern uint16_t adcAveragedValue;

// Note that UART_HandleTypeDef and ADC_HandleTypeDef are parameters so that we can change if we use UART1, UART2, etc. or ADC1, ADCx, etc.
void Get_Averaged_ADC_Values(int*, int*, int*);

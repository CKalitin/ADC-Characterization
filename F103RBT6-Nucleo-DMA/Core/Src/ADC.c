#include "ADC.h"

uint16_t adcBuffer[ADC_BUFFER_LENGTH];
uint16_t adcAveragedValue;

/**
  * @brief Take numSamples ADC values, average them, then return both raw and error adjusted values
  * @param adcValuesAveraged: Pointer to the averaged ADC value
  * @param adcValuesAdjusted: Pointer to the averaged ADC value, adjusted for error
  * @retval None
  */
void Get_Averaged_ADC_Values(int* adcValuesAveraged, int* adcValuesADCAdjusted, int* adcValuesCurrentAdjusted){
  *adcValuesAveraged = adcAveragedValue; // Set the pointer output

  // ADC Error Polynomial: -75.8 + 0.0222x
  // Current Sensor ADC Error Polynomial: 72 - 0.0325x + 19.6 - 8.82E-3x = 91.6 - 0.04132x

  int adcError = -75.8 + 0.0222 * (*adcValuesAveraged); // This is a predetermined error polynomial
  int currentSensorADCError = 91.6 - 0.04132 * (*adcValuesAveraged); // This is the predetermined erroy polynomial for the current sensor

  *adcValuesADCAdjusted = *adcValuesAveraged - adcError; // Set the other pointer output
  *adcValuesCurrentAdjusted = *adcValuesAveraged - adcError - currentSensorADCError; // Set the other pointer output
}

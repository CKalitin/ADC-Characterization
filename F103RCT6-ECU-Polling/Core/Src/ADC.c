#include "ADC.h"

/**
  * @brief Take numSamples ADC values, average them, then return both raw and error adjusted values
  * @param hadc: ADC_HandleTypeDef object
  * @param numSamples: Number of ADC values to average
  * @param msPerObv: Milliseconds between each ADC observation
  * @param adcValuesAveraged: Pointer to the averaged ADC value
  * @param adcValuesAdjusted: Pointer to the averaged ADC value, adjusted for error
  * @retval None
  */
void Get_Averaged_ADC_Values(ADC_HandleTypeDef hadc, int numSamples, int msPerObv, int* adcValuesAveraged, int* adcValuesAdcAdjusted, int* adcValuesCurrentAdjusted){
  int adcValuesSum = 0;

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1);

  // Get numSamples ADC values, each 1ms apart
  for (int i = 0; i < numSamples; i++){
    adcValuesSum += HAL_ADC_GetValue(&hadc);
    HAL_Delay(msPerObv);
  }

  *adcValuesAveraged = adcValuesSum / numSamples; // Set pointer output

  // ADC Error Polynomial: -75.8 + 0.0222x
  // Current Sensor ADC Error Polynomial: 72 - 0.0325x + 19.6 - 8.82E-3x = 91.6 - 0.04132x

  int adcError = -65.4 + 0.0104 * (*adcValuesAveraged); // This is a predetermined error polynomial
  int currentSensorADCError = 91.6 - 0.04132 * (*adcValuesAveraged); // This is the predetermined erroy polynomial for the current sensor

  *adcValuesAdcAdjusted = *adcValuesAveraged - adcError; // Set the other pointer output
  *adcValuesCurrentAdjusted = *adcValuesAveraged - adcError - currentSensorADCError; // Set the other pointer output
}

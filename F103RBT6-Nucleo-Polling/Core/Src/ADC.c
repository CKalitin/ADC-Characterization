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
void Get_Averaged_ADC_Values(ADC_HandleTypeDef hadc, int numSamples, int msPerObv, int* adcValuesAveraged, int* adcValuesAdjusted){
  int adcValuesSum = 0;

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1);

  // Get numSamples ADC values, each 1ms apart
  for (int i = 0; i < numSamples; i++){
    int out = HAL_ADC_GetValue(&hadc);
    adcValuesSum += out;
    HAL_Delay(msPerObv);
  }

  int adcError = -72.5 + 0.0133*(adcValuesSum / numSamples); // This is a predetermined error polynomial

  // Set pointer outputs
  *adcValuesAveraged = adcValuesSum / numSamples;
  *adcValuesAdjusted = *adcValuesAveraged - adcError;
}

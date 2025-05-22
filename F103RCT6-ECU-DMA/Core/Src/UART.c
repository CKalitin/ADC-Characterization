#include "UART.h"

// Varaible definitions
char tx_buff[100] = {0};
char rx_buff[100] = {0};

/**
  * @brief Wait for 's' character to be received over UART to continue the program
  * @param uart: UART_HandleTypeDef object
  * @retval None
  */
void Continue_On_UART_Receive(UART_HandleTypeDef uart){
  // The 's' character here is arbitrary
  // We just pause the program until we see 's\n' received over uart
  memset(rx_buff, 0, sizeof(rx_buff));
  while (rx_buff[0] != 's'){
    memset(rx_buff, 0, sizeof(rx_buff));
    HAL_UART_Receive(&uart, (uint8_t*)rx_buff, sizeof(rx_buff), 1000); // HAL_UART_Receive waits until '\n' to continue the program
    HAL_Delay(1); // Wait a ms because why not
  }
}

/**
  * @brief Send ADC values over UART to the Python script
  * @param uart: UART_HandleTypeDef object
  * @retval None
  */
void Send_ADC_Values_Over_UART(UART_HandleTypeDef uart, int adcValuesAveraged, int adcValuesADCAdjusted, int adcValuesCurrentAdjusted){
  char new_buff[50] = {0};
  memset(new_buff, 0, sizeof(new_buff));
  memset(tx_buff, 0, sizeof(tx_buff));
  sprintf(new_buff, "%d, %d, %d\n\r", adcValuesAveraged, adcValuesADCAdjusted, adcValuesCurrentAdjusted);
  HAL_UART_Transmit(&uart, (uint8_t*)new_buff, sizeof(new_buff), 1000);
}

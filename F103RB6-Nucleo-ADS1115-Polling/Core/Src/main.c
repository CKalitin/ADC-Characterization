/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "UART.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static uint8_t ADS1115_ADDR = 0b10010000; // I2C address of the ADS1115, left aligned (as I2C HAL expects, 7 bit address with R/W bit set by the HAL)
static uint8_t ADS1115_REG_CONVERSION = 0x00; // Register address for ADC conversion output
static uint8_t ADS1115_REG_CONFIG = 0x01; // Register address for configuration
static uint8_t ADS1115_REG_LO_THRESH = 0x02; // Register address for low threshold
static uint8_t ADS1115_REG_HI_THRESH = 0x03; // Register address for high threshold

uint16_t ads1115_config = 0b0000001011100011; // 15: OS, 14:12: MUX, 11:9: PGA, 8: MODE, 7:5: DR, 4: COMP_MODE, 3: COMP_POL, 2: COMP_LAT, 1:0: COMP_QUE
uint16_t ads1115_low_thresh = 32768; // 0V
uint16_t ads1115_high_thresh = 58024; // 3V

// To use these masks, do ads1115_config | ads1115_config_a0_gnd
// Note that the default ads1115_config is set to 000, which is A0 relative to A1
uint16_t ads1115_config_a0_gnd = 0b0100000000000000; // Mask to select reading voltage between A0 and GND
uint16_t ads1115_config_a1_gnd = 0b0101000000000000; // Mask for A1 reading relative to GND
uint16_t ads1115_config_a2_gnd = 0b0110000000000000; // Mask for A2 reading relative to GND
uint16_t ads1115_config_a3_gnd = 0b0111000000000000; // Mask for A0 reading relative to A1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static void ADS1115_Init(void);
static void ADS1115_Thresh_Init(void);
static void ADS1115_I2C_Reset(void);
static HAL_StatusTypeDef ADS1115_Select_Register(uint8_t* reg);
static HAL_StatusTypeDef ADS1115_Write(uint8_t* reg, uint16_t* data);
static int16_t ADS1115_Read(void);

static HAL_StatusTypeDef I2C_Transmit(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout);
static HAL_StatusTypeDef I2C_Receive(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_Delay(100);
  ADS1115_I2C_Reset(); // Init is called inside here
  HAL_Delay(100);

  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, ADS1115_ADDR, 1, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // This function waits until 's' is received on UART to continue
    // This way, from the Python script we can command the STM32 chip operate only when we tell it to
    //Continue_On_UART_Receive(huart2);

	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // Toggle the LED high when we're collecting or sending values

    // Get 100 samples the slow way
    // At 860 SPS, 2ms delay should be enough to not get repeat values
    int32_t ads_data_sum = 0;
    for (int i = 0; i < 100; i++) {
      ads_data_sum += ADS1115_Read(); // Read the ADC value
      HAL_Delay(2);
    }

    int32_t ads_voltage_averaged = ads_data_sum / 100 * 8.192 * 1000000 / 65536; // uV bc. *1e6 to get full precision in an integer

    Send_ADC_Values_Over_UART(huart2, ads_voltage_averaged);

	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ALRT_Pin */
  GPIO_InitStruct.Pin = ALRT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALRT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void ADS1115_Init(void) {
  ADS1115_Write(&ADS1115_REG_CONFIG, &ads1115_config);
  ADS1115_Select_Register(&ADS1115_REG_CONVERSION); // Select conversion register for reading from
}

static void ADS1115_Thresh_Init(void) {
  ADS1115_Write(&ADS1115_REG_LO_THRESH, &ads1115_low_thresh);
  ADS1115_Write(&ADS1115_REG_HI_THRESH, &ads1115_high_thresh);
}

// See: https://community.st.com/t5/stm32-mcus-products/i2c-hal-busy/td-p/620525
static void ADS1115_I2C_Reset(void){
  // Attempt to write config
  HAL_StatusTypeDef status = ADS1115_Write(&ADS1115_REG_CONFIG, &ads1115_config);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (status != HAL_OK){
    HAL_I2C_DeInit(&hi2c1); // De-initialize the I2C peripheral before settings pins as GPIOs
    HAL_Delay(10);  // Brief delay to ensure peripheral resets

    // Init the SCL GPIO as an open-drain GPIO output
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Send 9 pulses on SCL and keep SDA high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // SDA high
    for (int i = 0; i < 9; i++){
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_Delay(20);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
      HAL_Delay(20);
    }

    // Generate a STOP condition to release the bus
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);  // SDA low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);    // SCL high
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);    // SDA high
    HAL_Delay(10);  // Stabilize bus
    MX_I2C1_Init();

    status = HAL_I2C_IsDeviceReady(&hi2c1, ADS1115_ADDR, 1, HAL_MAX_DELAY);

    // print the status to UART
    char msg[50];
    sprintf(msg, "ADS1115 Reset Status: %d\r\n", status);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    if (status == HAL_OK) {
      ADS1115_Init();
    } else{
      ADS1115_I2C_Reset(); // Try again, ad infinitum
      uint32_t error = HAL_I2C_GetError(&hi2c1);
      sprintf(msg, "I2C Error Code: %lu\r\n", error);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
  }
}

static HAL_StatusTypeDef ADS1115_Select_Register(uint8_t* reg) {
  uint8_t buf[1] = {*reg}; // Buffer to send, first byte is the register address
  return I2C_Transmit(&hi2c1, ADS1115_ADDR, buf, sizeof(buf), HAL_MAX_DELAY);
}

static HAL_StatusTypeDef ADS1115_Write(uint8_t* reg, uint16_t* data) {
  // Write this better later, this data length this is stupid what if the msb is actually 0x00 and you want to send that
  uint8_t data_byte_msb = (uint8_t)((*data >> 8) & 0xFF); // First byte of data to send, most significant byte
  uint8_t data_byte_lsb = (uint8_t)(*data & 0xFF); // Least significant byte, with 0xFF mask to clear the upper 8 bits, probably not needed
  uint8_t buf[3] = {*reg, data_byte_msb, data_byte_lsb}; // Buffer to send, first byte is the register address, second and third are the data bytes
  uint8_t buf_len = data_byte_msb == 0x00 ? 2 : 3; // If msb if empty, only send register and lsb byte
  return I2C_Transmit(&hi2c1, ADS1115_ADDR, buf, buf_len, HAL_MAX_DELAY);
}

// This reads from whatever register was last selected. Note that both Select_Register and Write functions will select the register to either read or write to
static int16_t ADS1115_Read(void) {
  uint8_t buf[2];
  I2C_Receive(&hi2c1, ADS1115_ADDR, buf, sizeof(buf), HAL_MAX_DELAY);
  int16_t ads_data = (buf[0] << 8) | buf[1]; // Combine the two bytes into a single 16-bit value
  int32_t ads_voltage = ads_data * 8.192 * 1000000 / 65536; // Output in uV to get full precision, going to need to dynamically change the full scale range of the ADC and this variable
  //char msg[50];
  //snprintf(msg, sizeof(msg), "ADC Value: %ld\r\n", ads_voltage);
  //HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);  
  return ads_data;
}

static HAL_StatusTypeDef I2C_Transmit(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, Timeout);
  if (status != HAL_OK)
  {
    char msg[50];
    snprintf(msg, sizeof(msg), "I2C Transmit Error: %d\r\n", status);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  }
  return status;
}

static HAL_StatusTypeDef I2C_Receive(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
  HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size, Timeout);
  if (status != HAL_OK)
  {
    char msg[50];
    snprintf(msg, sizeof(msg), "I2C Receive Error: %d\r\n", status);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  }
  return status;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

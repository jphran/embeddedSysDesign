/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t readADC(void);
void ledPotIndicator(uint16_t voltageVal);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable gpioc clk
	//set up leds on board
	GPIO_InitTypeDef LEDinitStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP, 
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
		
	HAL_GPIO_Init(GPIOC, &LEDinitStr); //enable gpio LED pins	
	// turn off led's
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //red
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //blue
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //orange
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);	// green
								
	//will be using ADC_IN10 which is PC0															
	//set up ADC in pin
	GPIO_InitTypeDef ADCinitStr = {GPIO_PIN_0,
															GPIO_MODE_ANALOG, 
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
		
	HAL_GPIO_Init(GPIOC, &ADCinitStr); //enable gpio LED pins	
	
	
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; //enable ADC clock
	

	//adc calibration
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
	 ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= (1 << 31); //start self-calibration, pg 257
	//wait until calibration complete, pg 230
	while (ADC1->CR & (1 << 31)); //do nothing
	
	//enable adc
	ADC1->ISR &= ~(1 << 0); //clear ADRDY bit, pg 231
	ADC1->CR |= (1 << 0); //enable adc, pg 258
	//wait until adc ready, pg 255
	while (!(ADC1->ISR & (1 << 0)))
	{
		ADC1->CR |= (1 << 0); //enable adc, pg 258
	}
	
	//config adc
	ADC1->CFGR1 |= (1 << 4); //set data resolution to 8 bit, periphrefman.pdf pg 261
	ADC1->CFGR1 |= (1 << 13); //set to continuous mode, periphrefman.pdf pg 260
  ADC1->CFGR1 &= ~((1 << 10) | (1 << 11)); //disable hardware trigs, pg260
	ADC1->CHSELR |= (1 << 10); //enable ADC_IN10 conversion, pg 265
	
	//start adc
	ADC1->CR |= (1 << 2); //start conversion, pg 258
															
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t voltageVal = 0;

	
  while (1)
  {
		ledPotIndicator(readADC());
	}
  /* USER CODE END 3 */

}

//reads and returns ADC conversion
uint16_t readADC(void)
{
	//wait until end of conversion, pg 255
	while (!(ADC1->ISR & (1 << 2)))
	{
		//do nothing
	}
	return ADC1->DR;
}

void ledPotIndicator(uint16_t voltageVal)
{
	uint8_t redOnAt = 10;
	uint8_t greenOnAt = 256/2;
	uint8_t blueOnAt = 256/4 * 3;
	uint8_t orangeOnAt = 255;
	
	if (voltageVal >= redOnAt)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); //red
	}
	else {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);} //red
	
	if (voltageVal >= greenOnAt)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);	// green
	}
	else {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);}	// green
	
	if (voltageVal >= blueOnAt) 
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //blue
	}
	else {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);} //blue
	
	if (voltageVal >= orangeOnAt)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //orange
	}
	else {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);} //orange
}





























/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

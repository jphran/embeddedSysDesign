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
//blocking method. opens i2c transaction for 1 byte, returns 1 if success
int writeI2C(uint8_t addy, uint8_t msg);
//blocking method. reads i2c transaction for 1 byte, returns msg if success
uint8_t readI2C(uint8_t addy);

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
		//enable clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable gpioc clk
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable gpiob clk
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //enable i2c2 
	
	
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
	
	//enable pins
	GPIO_InitTypeDef initStr = {GPIO_PIN_11 | GPIO_PIN_13,
															GPIO_MODE_AF_OD, //set gpio to alternate function, open drain mode
															GPIO_SPEED_FREQ_LOW,
															GPIO_PULLUP};
		
	HAL_GPIO_Init(GPIOB, &initStr); //enable gpio PC11/13 pin
															
	GPIO_InitTypeDef initStr1 = {GPIO_PIN_14,
															GPIO_MODE_OUTPUT_PP, //set gpio to alternate function, open drain mode
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
		
	HAL_GPIO_Init(GPIOB, &initStr1); //enable gpio PB14 pin
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //set pin HIGH
															
	GPIO_InitTypeDef initStr2 = {GPIO_PIN_0,
															GPIO_MODE_OUTPUT_PP, //set gpio to alternate function, open drain mode
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
		
	HAL_GPIO_Init(GPIOC, &initStr2); //enable gpio PC0 pin
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); //set pin HIGH
															
															
	//enable af1 on PB11 and enable af5 on pb13
	GPIOB->AFR[1] |= (1 << 12) | (1 << 20) | (1 << 22);
	
	//configure i2c2														
	I2C2->TIMINGR |= (1 << 28); //PRESC = 1;
	I2C2->TIMINGR |= (0x13 << 0); //SCLL = 0x13;
	I2C2->TIMINGR |= (0xF << 8); //SCLH = 0xF;
	I2C2->TIMINGR |= (0x2 << 16); //SDADEL = 0x2;	
	I2C2->TIMINGR |= (0x4 << 20); //SCLDEL = 0x4;
	I2C2->CR1 |= (1 << 0); //peripheral enabled
															
	/* Clear the NBYTES and SADD bit fields
	* The NBYTES field begins at bit 16, the SADD at bit 0
	*/
	//I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	/* Set NBYTES = 42 and SADD = 0x14
	* Can use hex or decimal values directly as bitmasks.
	* Remember that for 7-bit addresses, the lowest SADD bit
	* is not used and the mask must be shifted by one.
	*/
	//I2C2->CR2 |= (42 << 16) | (0x14 << 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	const uint8_t gyroAddress = 0x6B;
	const uint8_t gyroWhoAmI = 0x0F; //lab5.pdf pg 10
	const uint8_t expectedVal = 0xD4;
	
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
//		//configure transaction params
//		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear NBYTES and SADD
//		I2C2->CR2 |= (gyroAddress << 1); //set SADD (slave address)
//		I2C2->CR2 &= ~(1 << 10); //set write direction
//		I2C2->CR2 |= (1 << 16); //set 1 byte to transfer
//		I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675
//		
//		//wait unitl TXIS flags are set, periphrefman.pdf pg. 680
//		while(!(I2C2->ISR & (1 << 1))){ 
//			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//			if(I2C2->ISR & (1 << 4)){ //if NACKF set
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//throw err
//			}
//		}
//		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//		

//		
//		//send who am i register address
//		I2C2->TXDR = gyroWhoAmI;
//		
//		//wait till transfer complete flag is set
//		while(!(I2C2->ISR & (1 << 6))){
//			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//			//do nothing
//		}
//		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		
		if(!writeI2C(gyroAddress, gyroWhoAmI)){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//throw err
		}
		uint8_t val = readI2C(gyroAddress);
		
//		//config transaction params
//		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear NBYTES and SADD
//		I2C2->CR2 |= (gyroAddress << 1); //set SADD (slave address)
//		I2C2->CR2 |= (1 << 10); //set read direction
//		I2C2->CR2 |= (1 << 16); //set 1 byte to transfer
//		I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675
//		
//		//wait unitl RXNE flags are set, periphrefman.pdf pg. 680
//		while(!(I2C2->ISR & ((1 << 2) | (1 << 4)))){ 
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//			if(I2C2->ISR & (1<<4)){ //if NACKF set
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//throw err
//			}
//		}
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//		
//		//wait till transfer complete flag is set
//		while(!(I2C2->ISR & (1 << 6))){
//			//do nothing
//		}
		
		//check for proper value
		if(val & expectedVal){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //throw all good
		}
		else{ //throw received but no good
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		}
		
		
		
		HAL_Delay(500);
  }
  /* USER CODE END 3 */

}


int writeI2C(uint8_t addy, uint8_t msg){
	//configure transaction params
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear NBYTES and SADD
	I2C2->CR2 |= (addy << 1); //set SADD (slave address)
	I2C2->CR2 &= ~(1 << 10); //set write direction
	I2C2->CR2 |= (1 << 16); //set 1 byte to transfer
	I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675
	
	//wait unitl TXIS flags are set, periphrefman.pdf pg. 680
	while(!(I2C2->ISR & (1 << 1))){ 
		if(I2C2->ISR & (1 << 4)){ //if NACKF set
			return 0;
		}
	}
	
	//send who am i register address
	I2C2->TXDR = msg;
	
	//wait till transfer complete flag is set
	while(!(I2C2->ISR & (1 << 6))){
		//do nothing
	}
	return 1;
}


uint8_t readI2C(uint8_t addy){
	//config transaction params
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear NBYTES and SADD
	I2C2->CR2 |= (addy << 1); //set SADD (slave address)
	I2C2->CR2 |= (1 << 10); //set read direction
	I2C2->CR2 |= (1 << 16); //set 1 byte to transfer
	I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675
	
	//wait unitl RXNE flags are set, periphrefman.pdf pg. 680
	while(!(I2C2->ISR & ((1 << 2) | (1 << 4)))){ 
		if(I2C2->ISR & (1<<4)){ //if NACKF set
			return NULL;	//throw err
		}
	}
	
	//wait till transfer complete flag is set
	while(!(I2C2->ISR & (1 << 6))){
		//do nothing
	}
	
	I2C2->CR2 |= (1 << 14); //set stop bit
	
	return I2C2->RXDR;
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

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
#include "stm32f072xb.h"                  // Device header
//#include <limits.h>

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

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {
	
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
		
	/* This example uses HAL library calls to control
	the GPIOC peripheral. You'll be redoing this code
	with hardware register access. */
		
	//__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  //line 470, 7820
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Set up a configuration struct to pass to the initialization function
	/*
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9, 
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high
	*/
	// GP Out mode
	GPIOC->MODER |= (1 << 12); //line 6543, set MODER6(13,12) to 0b01
	GPIOC->MODER |= (1 << 14); //line 6548, set MODER7(15,14) to 0b01
	// push-pull type
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7)); // set (6,7) to 0b00
	// set pin speed low
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14)); //pg 157, set OSPEEDR7(15,14) & OSPEEDR6(13,12)  to 0bx0
	// set no pull down 
	GPIOC->PUPDR &= ~((1<< 15) | (1 << 14) | (1 << 13) | (1 << 12)); //pg 157, set PUPDR6/7 to 0b00
	// init pin logic
	GPIOC->ODR |= (1 << 6); //pg 158, set to 0b1
	GPIOC->ODR &= ~(1 << 7); // set to 0b0
	
	//set up push button
	GPIOA->MODER &= ~(1 << 0); //set (1,0) to 0b00
	//GPIOA->OTYPER &= ~(1 << 0); //set (0) to 0b0
	GPIOA->OSPEEDR &= ~(1 << 0); //set (0) to 0bx0
	GPIOA->PUPDR &= ~(1 << 0); //set (0) to 0b0
	GPIOA->PUPDR |= (1 << 1); //set (1) to 0b1
	uint32_t debounceVar;
	
	while (1) {
		debounceVar <<= 1;
		debounceVar |= (GPIOA->IDR & GPIO_IDR_0);
		
		if(debounceVar == 0x7FFFFFFF){
			if((GPIOC->ODR & GPIO_ODR_7)){
				GPIOC->ODR |= (1 << 6); //pg 158, set to 0b1
				GPIOC->ODR &= ~(1 << 7); // set to 0b0
			}
			else{
				GPIOC->ODR |= (1 << 7); //pg 158, set to 0b1
				GPIOC->ODR &= ~(1 << 6); // set to 0b0
			}
		}
		
		HAL_Delay(2);
		/*
		HAL_Delay(200); // Delay 200ms
			
		// Toggle the output state of both PC8 and PC9
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
		GPIOC->ODR |= (1 << 7); //pg 158, set to 0b1
		GPIOC->ODR &= ~(1 << 6); // set to 0b0
		HAL_Delay(200); // Delay 200ms
		GPIOC->ODR |= (1 << 6); //pg 158, set to 0b1
		GPIOC->ODR &= ~(1 << 7); // set to 0b0
		*/
	}
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

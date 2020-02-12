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
#include <math.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
void TIM2_IRQHandler(void){ //defined in ../Application/startup_stm32f072xb.s
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	TIM2->SR &= ~(1 << 0); //reset interrupt pending flag
}


int main(void)
{
	const uint32_t BASE_FREQ = 8000000; //8 Mhz
  const uint32_t PSC_VAL2 = 7999; // yields 1kHz clk
	const uint32_t DES_FREQ2 = 4; // Hz
	const uint32_t ARR_VAL2 = BASE_FREQ/(PSC_VAL2+1)/DES_FREQ2;
	const uint32_t PSC_VAL3 = 79; // yields 100kHz clk
	const uint32_t DES_FREQ3 = 800; // Hz
	const uint32_t ARR_VAL3 = BASE_FREQ/(PSC_VAL3+1)/DES_FREQ3;
	
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	//enable GPIOC clock
	__HAL_RCC_GPIOC_CLK_ENABLE(); //enable gpioc clock in rcc

	// set up leds
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	GPIO_InitTypeDef initStr1 = {GPIO_PIN_6 | GPIO_PIN_7,
															GPIO_MODE_AF_PP, //set gpio to alternate function mode
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
		
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC		
	HAL_GPIO_Init(GPIOC, &initStr1);															
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Start PC9 high
															
	//set up timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable the TIM2 clock in the advanced performance bus register
	TIM2->PSC = PSC_VAL2; //this divides the base clk freq by 8000 (yields a 1kHz clock)
	TIM2->ARR = ARR_VAL2; //produces a UEV every 4Hz (BASE_FREQ/PSC/DES_FREQ = 250)
	TIM2->DIER |= (1 << 0); //enable update event (periph pg. 442)
	TIM2->CR1 |= (1 << 0); //enable timer (periph pg. 436)
	NVIC_EnableIRQ(TIM2_IRQn); //enable timer 2 interupt vector in the NVIC (defined in stm32f072xb.h)
	
  //set up timer 3
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->PSC = PSC_VAL3; //this divides the base clk freq by 80 (yields a 100kHz clock)
	TIM3->ARR = ARR_VAL3; //produces a UEV at 800Hz (BASE_FREQ/PSC/DES_FREQ = 125)
	TIM3->DIER |= (1 << 0); //enable update event (periph pg. 442)
	TIM3->CCMR1 &= ~((1 << 0) | (1 << 1) | (1 << 8) | (1 << 9));	//set ch1 and ch2 to output mode (periph pg 445)	
	TIM3->CCMR1 |= (1 << 4) | (1 << 5) | (1 << 6);	//this set ch1 to pwm mode 2	
	TIM3->CCMR1 &= ~(1 << 12);	
	TIM3->CCMR1 |= (1 << 13) | (1 << 14);	//this and one above set ch2 to pwm mode 1	
	TIM3->CCMR1 |= (1 << 3); //enable output compare 1 preload (periph pg 446)
  TIM3->CCMR1 |= (1 << 11); //enable output compare 2 preload (periph pg 445)
	TIM3->CCER |= TIM_CCER_CC1E; //enable capture/compare 1 output
  TIM3->CCER |= TIM_CCER_CC2E; //enable capture/compare 2 output (def stem32f072xb.h)
  TIM3->CCR1 = (uint16_t)(ARR_VAL3/2);
	TIM3->CCR2 = (uint16_t)(ARR_VAL3/10);
	GPIOC->AFR[0] = GPIO_AFRL_AFRL0; 
	GPIOC->AFR[1] = GPIO_AFRH_AFRH0; //set alternate function pin to PC6 to TIM3_CH1
	TIM3->CR1 |= (1 << 0); //enable clock 

  while (1)
  {
		//do nothing
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

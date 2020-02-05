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

void SystemClock_Config(void);
void Error_Handler(void);
//writing the EXTI interrupt handler
void EXTI0_1_IRQHandler(void){
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	EXTI->PR |= EXTI_PR_PR0; //reset interrupt pending flag
	
	//delay for excersize 2
	int i = 0;
	while(++i < 2000000){
		//do nothing 
	}
	i = 0;
	
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
}


int main(void)
{
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	
	//GPIO setup
	GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);																											
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Start PC8 high
	
	//user button setup
	GPIO_InitTypeDef initButStr	= {GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN};									
	HAL_GPIO_Init(GPIOA, &initButStr);
	
	//connect PA0 to EXTI0
	EXTI->IMR |= EXTI_IMR_IM0; //unmask interrupt request
	EXTI->RTSR |= EXTI_RTSR_RT0; //enable rising trigger for interrupt
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable SYSCFG clock
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; //set multiplexer to connect PA0 and EXTICR1
	
	//enable and set priority of EXTI interrupt (line 0 to priority 1)
	IRQn_Type inter = EXTI0_1_IRQn;
	NVIC_EnableIRQ(inter);
	NVIC_SetPriority(inter, 1);
	//NVIC_SetPriority(inter, 3);
	
	//change systick interrupt number
	//NVIC_SetPriority(SysTick_IRQn, 2);
	

	
	
  while (1)
  {
		HAL_Delay(500);
		// Toggle the output state of both PC8 and PC9
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
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

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

const unsigned int desBaudRate = 115200;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

// transmit a single char over USART
void USART_sendChar(char toSend);
//transmit a string over USART
void USART_sendString(char * toSend);
//receive a char over USART
char USART_readChar(void);
//set up USART interrupt handler
void USART3_4_IRQHandler(void);
//checks second char in user cmd
void pinCmd(char cmd);
//checks first char in user cmd
uint16_t whichLED(char led);
//clears user cmd
void clearCmd(void);
//globals used in USART3 irq handler
char cmd[2] = {NULL, NULL};
uint16_t newCmd = 0;
uint16_t userLED = NULL;

														
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	//set up USART3 on PC4(TX) and PC5(RX)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable gpioc clk
	GPIO_InitTypeDef initStr = {GPIO_PIN_4 | GPIO_PIN_5,
															GPIO_MODE_AF_PP, //set gpio to alternate function mode
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
		
	HAL_GPIO_Init(GPIOC, &initStr); //enable gpio PC4/5 pins
	GPIOC->AFR[0] |= (1 << 16) | (1 << 20); // switch PC4/5 to AF1
							
	//set up USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; //enable usart3 clk
	USART3->BRR = (int) (HAL_RCC_GetHCLKFreq()/desBaudRate); //set BRR to 416/417 which yields an err of <1% on 115200 baud
	USART3->CR1 |= (1 << 2) | (1 << 3); //enable transmit and receive (periphrefmanual.pdf, pg 734)
	USART3->CR1 |= (1 << 5); //enable receive register not empty interrupt
	USART3->CR1 |= (1 << 0); //enable USART3
															
	char toSend = 'j';
	char toSendArr[] = "hello";
															
	//set up leds on board
	GPIO_InitTypeDef LEDinitStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP, 
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
		
	HAL_GPIO_Init(GPIOC, &LEDinitStr); //enable gpio LED pins	
	// turn off led's
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);															
	
	char received = NULL;
	char errMsg[] = "Err: no matching LED color to '";

	NVIC_EnableIRQ(USART3_4_IRQn); //enables the USART3 global ineterrupt
	NVIC_SetPriority(USART3_4_IRQn, 1); //set high priority
	
	USART_sendString("CMD: "); //send prompt							
	
  while (1)
  {
		USART_sendString(toSendArr);
		HAL_Delay(100);
		
		
		received = USART_readChar();
		
		switch(received)
		{
			case 'r': 
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
				break;
			case 'b':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
				break;
			case 'o':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
				break;
			case 'g':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
				break;
			default: 
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				USART_sendString(errMsg);
				USART_sendString(&received);
				USART_sendString("'\n");
		}
		
		
		
//		while(newCmd != 1); //do nothing
		if(newCmd){
			userLED = whichLED(cmd[0]);
			pinCmd(cmd[1]);
			USART_sendString(cmd);
			USART_sendString(" executed.\n");
			clearCmd();
			USART_sendString("CMD: "); //send prompt
		}
  }
}









void USART_sendChar(char toSend){
	while(!(USART3->ISR & USART_ISR_TXE)){ //wait till transmit register is empty
		//do nothing
	}
	
	USART3->TDR = toSend; //send data to transmit register
}

void USART_sendString(char *toSend){ //TODO: figure out why 'for(int i = 0; i < sizeof(toSend)/sizeof(char); i++) doesnt' work
	unsigned int i = 0;
	while(toSend[i] != '\0'){
		USART_sendChar(toSend[i++]);
	}
}

char USART_readChar(void){
	while(!(USART3->ISR & USART_ISR_RXNE)){ //wait for read register to fill
		//do nothing
	}
	
	return USART3->RDR;
}

void USART3_4_IRQHandler(void){
	while(!(USART3->ISR & USART_ISR_RXNE)){ //wait for read register to fill
		//do nothing
	}
	if(cmd[0]){
		cmd[1] = USART3->RDR;
		newCmd = 1;
	}
	else{cmd[0] = USART3->RDR;}
}

uint16_t whichLED(char led){
	if(led == 'r'){
		return GPIO_PIN_6;
	}
	else if(led == 'b'){
		return GPIO_PIN_7;
	}
	else if(led == 'o'){
		return GPIO_PIN_8;
	}
	else if(led == 'g'){
		return GPIO_PIN_9;
	}
	else{USART_sendString("Err: improper pin name\n"); clearCmd(); return NULL;}//TODO: throw err	}
}

void pinCmd(char cmd){
	if(cmd == '0'){
		HAL_GPIO_WritePin(GPIOC, userLED, GPIO_PIN_RESET);
	}
	else if(cmd == '1'){
		HAL_GPIO_WritePin(GPIOC, userLED, GPIO_PIN_SET);
	}
	else if(cmd == '2'){
		HAL_GPIO_TogglePin(GPIOC, userLED);
	}
	else{USART_sendString("Err: improper cmd\n"); clearCmd();}//todo: throw err}
}

void clearCmd(void){
	cmd[0] = NULL;
	cmd[1] = NULL;
	newCmd = 0;
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

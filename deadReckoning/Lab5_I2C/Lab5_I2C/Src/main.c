
// Modified by: Justin Francis, 4/27/20

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "MadgwickAHRS.h"
#include "ins_types.h"
#include "MPU9250.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

//Macros
#define redOn HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET) //red
#define blueOn HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET) //blue
#define orangeOn HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET) //orange
#define greenOn HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)	// green
#define redOff HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET) //red
#define blueOff HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET) //blue
#define orangeOff HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET) //orange
#define greenOff HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)	// green

//for Madgwick Filter
#define sampleFreq 77.0f
#define betaDef 0.1f

struct Pose 
{
	float roll, pitch, yaw;
} pose;

//FN protos
void SystemClock_Config(void);
void Error_Handler(void);
//*******************USART***********************
const unsigned int desBaudRate = 115200;
// transmit a single char over USART
void USART_sendChar(char toSend);
//transmit a string over USART
void USART_sendString(char * toSend);
//receive a char over USART
char USART_readChar(void);
//set up USART interrupt handler
void USART3_4_IRQHandler(void);


int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();
	
	initTickCounter();
	
	//enable clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable gpioc clk
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable gpiob clk

	//set up USART3 on PC4(TX) and PC5(RX)
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

	initI2C();

	uint16_t count = 0;
	struct Orientation ont;
	float magneticBearing;
	char magString[sizeof(float)];
	
	while (1)
	{

		updateIMU(&ont, &magneticBearing);
		
		
		count += 1;
		if(count >= 100)
		{
			char buffer[ONT_FMT_LEN];	
			ont_format(&ont, buffer);
			USART_sendString(buffer);
//			
			USART_sendString("\n\n\r***Bearing***\n\r");
			sprintf(magString, "%f", magneticBearing);
			USART_sendString(magString);
			USART_sendString("\n\r");

			count = 0;
		} 
		
		
		HAL_Delay(1);

	}

}


//***************************FN*******************************************
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

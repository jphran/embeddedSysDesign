
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
// #define M_PI 3.14159
// #define radToDeg(rad) (rad * 180.0 / M_PI)
// #define degToRad(deg) (deg * M_PI / 180.0)

// //filters
// #define COMP_FILT_GYRO_GAIN 0.9
// #define COMP_FILT_ACC_GAIN (1 - COMP_FILT_GYRO_GAIN)
//for Madgwick Filter
#define sampleFreq 77.0f
#define betaDef 0.1f
//globals (yes, I know these are bad)
// unsigned int g_dt = 0;

//Typedefs
// union u{
//     int16_t i;
//     char rawData[sizeof(int16_t)];
// } rawData_to_int;

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
//********************I2C**************************
//blocking method. reads i2c transaction for n bytes
// int readI2C(uint8_t slaveAddy, uint8_t regAddy, unsigned int numBytes, signed char *returnMsg);
// //blocking method that writes to a slave, writes n bytes, first byte in msg (msg[0]) is register address
// void writeI2C(uint8_t slaveAddy, uint8_t numBytes, uint8_t *msg);
// //for timing
// void TIM2_IRQHandler(void){ //defined in ../Application/startup_stm32f072xb.s
// 	g_dt += 1;
// 	TIM2->SR &= ~(1 << 0); //reset interrupt pending flag
// }

// void initTickCounter()
// {
// 	//for timing
// 	const uint32_t BASE_FREQ = 8e6; //8 Mhz
//   const uint32_t PSC_VAL2 = 79; // yields 100kHz clk
// 	const uint32_t DES_FREQ2 = 1e5; // Hz
// 	const uint32_t ARR_VAL2 = BASE_FREQ/(PSC_VAL2+1)/DES_FREQ2;
	
// 	//set up timer 2
// 	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable the TIM2 clock in the advanced performance bus register
// 	TIM2->PSC = PSC_VAL2; //this divides the base clk freq by 8000 (yields a 1kHz clock)
// 	TIM2->ARR = ARR_VAL2; //produces a UEV every 4Hz (BASE_FREQ/PSC/DES_FREQ = 250)
// 	TIM2->DIER |= (1 << 0); //enable update event (periph pg. 442)
// 	TIM2->CR1 |= (1 << 0); //enable timer (periph pg. 436)
// 	NVIC_EnableIRQ(TIM2_IRQn); //enable timer 2 interupt vector in the NVIC (defined in stm32f072xb.h)
// //	NVIC_SetPriority(TIM2_IRQn, 1); //set high priority
// }

// void initI2C()
// {
// 	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //enable i2c2 

// 	//enable sda (11) and scl (13)
// 	GPIO_InitTypeDef i2cInitStr = {GPIO_PIN_11 | GPIO_PIN_13,
// 															GPIO_MODE_AF_OD, //set gpio to alternate function, open drain mode
// 															GPIO_SPEED_FREQ_LOW,
// 															GPIO_PULLUP};
		
// 	HAL_GPIO_Init(GPIOB, &i2cInitStr); //enable gpio PC11/13 pin
																																
// 	//enable af1 on PB11 and enable af5 on pb13
// 	GPIOB->AFR[1] |= (1 << 12) | (1 << 20) | (1 << 22);

// 	//configure i2c2 to 400 kHz (for MPU 9250)														
// 	I2C2->TIMINGR &= ~(0xFU << 28); //PRESC = 0;
// 	I2C2->TIMINGR |= (0x9 << 0); //SCLL = 0x9;
// 	I2C2->TIMINGR |= (0x3 << 8); //SCLH = 0x3;
// 	I2C2->TIMINGR |= (0x1 << 16); //SDADEL = 0x1;	
// 	I2C2->TIMINGR |= (0x3 << 20); //SCLDEL = 0x3;
// 	I2C2->CR1 |= (1 << 0); //peripheral enabled
// }

// //returns true if correct chips are enslaved
// uint8_t initIMU()
// {
// 	//For use with the MPU 9250
// 	const uint8_t mMPU_ADDR = 0x68U; //AD0 is pulled low
// 	const uint8_t mMPU_WHO_AM_I_REG = 0x75U;
// 	const uint8_t mMPU_WHO_AM_I_VAL = 0x71U;
// 	const uint8_t mMPU_CONFIG_REG = 0x1AU;
// 	const uint8_t mMPU_INT_PIN_CONFIG_REG = 0x37U;
// 	const uint8_t mMPU_INT_PIN_CONFIG_SET = 0x02U;
// 	const uint8_t mMPU_GYRO_CONFIG_REG = 0x1BU; //given (X, Y, Z) in big-endian
// 	const uint8_t mMPU_GYRO_CONFIG_SET = 0x00U; //set gyro to +- 250 [dps]
// 	const uint8_t mMPU_ACCEL_CONFIG_REG = 0x1CU; 
// 	const uint8_t mMPU_ACCEL_CONFIG_SET = 0x00; //set accel to +- 2 [g]
// 	const uint8_t mMAG_ADDR = 0x0CU; 
// 	const uint8_t mMAG_WAI_REG = 0x00U;
// 	const uint8_t mMAG_WAI_VAL = 0x48U;
// 	const uint8_t mMAG_CTRL_REG = 0x0AU;
// 	const uint8_t mMAG_CTRL_SET = 0x16U; //or 0x12U, TODO: figure out what mode 1 vs 2 is

	
// 	//init mpu
// 	uint8_t configArr[] = {mMPU_INT_PIN_CONFIG_REG, mMPU_INT_PIN_CONFIG_SET, mMPU_GYRO_CONFIG_REG, mMPU_GYRO_CONFIG_SET, mMPU_ACCEL_CONFIG_REG, mMPU_ACCEL_CONFIG_SET};
// 	writeI2C(mMPU_ADDR, 6, configArr);
		
// 	//init magnetometer
// 	uint8_t magConfigArr[] = {mMAG_CTRL_REG, mMAG_CTRL_SET};
// 	writeI2C(mMAG_ADDR, 2, magConfigArr);
	
// 	//check comms are up with the correct slave
// 	uint8_t isCommsUp;
// 	int8_t whoAmIReturn;
	
// 	readI2C(mMAG_ADDR, mMAG_WAI_REG, 1, &whoAmIReturn);

// 	if(whoAmIReturn & mMAG_WAI_VAL)
// 	{
// 		isCommsUp = 1;
// 	}
// 	else
// 	{
// 		return 0;
// 	}
	
// 	readI2C(mMPU_ADDR, mMPU_WHO_AM_I_REG, 1, &whoAmIReturn);

// 	if(whoAmIReturn & mMPU_WHO_AM_I_VAL)
// 	{
// 		isCommsUp = 1;
// 	}
// 	else
// 	{
// 		return 0;
// 	}
	
// 	return isCommsUp;
// }

// void updateIMU(struct Orientation* ont, float* magneticBearing)
// {
// 	static const uint8_t mMPU_ADDR = 0x68U; //AD0 is pulled low
// 	static const uint8_t mMPU_GYRO_OUT_REG = 0x43U; 
// 	static const uint8_t mMPU_ACCEL_OUT_REG = 0x3BU; //registers 59-64, given (X, Y, Z) in big-endian
// 	static const uint8_t mMAG_ADDR = 0x0CU; 
// 	static const uint8_t mMAG_ST1_REG = 0x02U; //used to check if data ready
// 	static const uint8_t mMAG_OUT_REG = 0x03U; //given (X, Y, Z) in two's complement little-endian
// 	static const float MAGNETIC_DECLINATION_SLC = 12.5; //deg east

// 	signed char accelOutRaw[6];
// 	float accelOut[3];
// 	signed char gyroOutRaw[6];
// 	float gyroOut[3];
	// signed char magOutRaw[6];
	// float magOut[3];
	// float accelYawAng;
	// float gyroYawAng;
	
	// //read data
	// readI2C(mMPU_ADDR, mMPU_ACCEL_OUT_REG, 6, accelOutRaw);
	// readI2C(mMPU_ADDR, mMPU_GYRO_OUT_REG, 6, gyroOutRaw);

	// int8_t isReady;

	// do
	// {
	// 	readI2C(mMAG_ADDR, mMAG_ST1_REG, 1, &isReady);
	// }
	// while(!(isReady & 0x01));

	// readI2C(mMAG_ADDR, mMAG_OUT_REG, 7, magOutRaw);

	// //assemble data
	// for (int i = 0; i < 3; i++)
	// {
	// 	//accel (big-endian)
	// 	rawData_to_int.rawData[1] = accelOutRaw[i*2];
	// 	rawData_to_int.rawData[0] = accelOutRaw[(i*2)+1];
	// 	accelOut[i] = rawData_to_int.i * 4.0 / pow(2, 16); //4.0 => +-2.0g's (setup in accel config), 2^16 (resolution)
	// 	//gyro (big-endian)
	// 	rawData_to_int.rawData[1] = gyroOutRaw[i*2];
// 		rawData_to_int.rawData[0] = gyroOutRaw[(i*2)+1];
// 		gyroOut[i] = rawData_to_int.i * 500.0 / pow(2, 16); //500.0 => +250.0 dps (setup in gyro config), 2^16 (resolution)
// 		//mag (little-endian)
// 		rawData_to_int.rawData[0] = magOutRaw[i*2];
// 		rawData_to_int.rawData[1] = magOutRaw[(i*2)+1];
// 		magOut[i] = rawData_to_int.i * (4912.0 * 2) / pow(2,16); //4912.0*2 => +- 4912 uT (setup in mag config), 2^16 (resolution)
// 	}

// 	//convert data to the good stuff (bearings and position)
// 	*magneticBearing = radToDeg(atan2(magOut[1], magOut[0])) - MAGNETIC_DECLINATION_SLC;
// 	accelYawAng = radToDeg(asin(accelOut[2] / sqrt((accelOut[0] * accelOut[0]) + (accelOut[1] * accelOut[1]) + (accelOut[2] * accelOut[2]))));
// 	gyroYawAng += radToDeg(degToRad(gyroOut[2])) * (g_dt * 1e-5); 
// 	ont->pos.v.x += accelOut[0] * 9.81 * g_dt * 1e-5;
// 	ont->pos.v.y += accelOut[1] * 9.81 * g_dt * 1e-5;
// 	ont->pos.o.x += ont->pos.v.x * g_dt * 1e-5;
// 	ont->pos.o.y += ont->pos.v.y * g_dt * 1e-5;
// 	g_dt = 0;

// 	//complimentary filter
// 	ont->rot.o.z = gyroYawAng * COMP_FILT_GYRO_GAIN + accelYawAng * COMP_FILT_ACC_GAIN;
	
// 	//fill remaining orientation members
// 	ont->pos.a.x = accelOut[0];
// 	ont->pos.a.y = accelOut[1];
// 	ont->pos.a.z = accelOut[2];
// 	ont->rot.v.x = gyroOut[0];
// 	ont->rot.v.y = gyroOut[1];
// 	ont->rot.v.z = gyroOut[2];
// }


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
//	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //enable i2c2 

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
//		blueOn;
//		//read data
//		readI2C(mMPU_ADDR, mMPU_ACCEL_OUT_REG, 6, accelOutRaw);
//		readI2C(mMPU_ADDR, mMPU_GYRO_OUT_REG, 6, gyroOutRaw);
//		
//		int8_t isReady;

//		do
//		{
//			readI2C(mMAG_ADDR, mMAG_ST1_REG, 1, &isReady);
//		}
//		while(!(isReady & 0x01));
//	
//		readI2C(mMAG_ADDR, mMAG_OUT_REG, 7, magOutRaw);

//		//assemble data
//		for (int i = 0; i < 3; i++)
//		{
//			//accel (big-endian)
//			rawData_to_int.rawData[1] = accelOutRaw[i*2];
//			rawData_to_int.rawData[0] = accelOutRaw[(i*2)+1];
//			accelOut[i] = rawData_to_int.i * 4.0 / pow(2, 16); //4.0 => +-2.0g's (setup in accel config), 2^16 (resolution)
//			//gyro (big-endian)
//			rawData_to_int.rawData[1] = gyroOutRaw[i*2];
//			rawData_to_int.rawData[0] = gyroOutRaw[(i*2)+1];
//			gyroOut[i] = rawData_to_int.i * 500.0 / pow(2, 16); //500.0 => +250.0 dps (setup in gyro config), 2^16 (resolution)
//			//mag (little-endian)
//			rawData_to_int.rawData[0] = magOutRaw[i*2];
//			rawData_to_int.rawData[1] = magOutRaw[(i*2)+1];
//			magOut[i] = rawData_to_int.i * (4912.0 * 2) / pow(2,16); //4912.0*2 => +- 4912 uT (setup in mag config), 2^16 (resolution)
//		}
//		
//		//convert data to the good stuff (bearings and position)
//		magneticBearingFromNorth = radToDeg(atan2(magOut[1], magOut[0]));
//		accelYawAng = radToDeg(asin(accelOut[2] / sqrt((accelOut[0] * accelOut[0]) + (accelOut[1] * accelOut[1]) + (accelOut[2] * accelOut[2]))));
//		gyroYawAng += radToDeg(degToRad(gyroOut[2])) * (g_dt * 1e-5); 
//		sysVelX += accelOut[0] * 9.81 * g_dt * 1e-5;
//		sysVelY += accelOut[1] * 9.81 * g_dt * 1e-5;
//		sysPosX += sysVelX * g_dt * 1e-5;
//		sysPosY += sysVelY * g_dt * 1e-5;
//		g_dt = 0;

//		//complimentary filter
//		sysYawAng = gyroYawAng * COMP_FILT_GYRO_GAIN + accelYawAng * COMP_FILT_ACC_GAIN;
		
		//madgwick filter
//		MadgwickAHRSupdate(gyroOut[0], gyroOut[1], gyroOut[2], accelOut[0], accelOut[1], accelOut[2], magOut[0], magOut[1], magOut[2]);
//		//convert to euler angles
//		// roll (x-axis rotation)
//    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
//    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
//    pose.roll = atan2(sinr_cosp, cosr_cosp);

//    // pitch (y-axis rotation)
//    double sinp = 2 * (q0 * q2 - q3 * q1);
//    if (fabs(sinp) >= 1)
//        pose.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//    else
//        pose.pitch = asin(sinp);

//    // yaw (z-axis rotation)
//    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
//    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
//    pose.yaw = atan2(siny_cosp, cosy_cosp);

		updateIMU(&ont, &magneticBearing);
		
		
		count += 1;
		if(count >= 100)
		{
			char buffer[ONT_FMT_LEN];	
			ont_format(&ont, buffer);
			USART_sendString(buffer);
//			for (int i = 0; i < 3; i++)
//			{
//				sprintf(accelString, "%f",accelOut[i]);
//				USART_sendString(accelString);
//				USART_sendString("\n\r");
//			}
//			USART_sendString("\n\n\r***Position***\n\r");
//			sprintf(accelString, "%f", sysPosX);
//			USART_sendString(accelString);
//			USART_sendString("\n\r");
//			sprintf(accelString, "%f", sysPosY);
//			USART_sendString(accelString);
//			USART_sendString("\n\r");
//			
//			USART_sendString("\n\n\r***Yaw***\n\r");
//			sprintf(accelString, "%f", sysYawAng);
//			USART_sendString(accelString);
//			USART_sendString("\n\r");
//			
			USART_sendString("\n\n\r***Bearing***\n\r");
			sprintf(magString, "%f", magneticBearing);
			USART_sendString(magString);
			USART_sendString("\n\r");

			count = 0;
		} 
		
		
		HAL_Delay(1);
		
//		blueOff;
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


// void writeI2C(uint8_t slaveAddy, uint8_t numBytes, uint8_t* msg){
// 	//configure transaction params
// 	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear CR2 nbytes and sadd registers
// 	// I2C2->CR2 = 0; //clear CR2 register
// 	I2C2->CR2 |= (slaveAddy << 1); //set SADD (slave address)
// 	I2C2->CR2 &= ~(1 << 10); //set write direction
// 	I2C2->CR2 |= (numBytes << 16); //set  byte to transfer
// 	I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675

// 	for(int i = 0; i < numBytes; ++i){
// 		//wait unitl TXIS flags are set, periphrefman.pdf pg. 680
// 		while(!(I2C2->ISR & (1 << 1))){ 
// 			if(I2C2->ISR & (1 << 4)){ //if NACKF set
// 				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//throw err, red led on
// 			}

// 		}
// 		//send slave msg
// 		I2C2->TXDR = msg[i];
// 	}
// 	//wait till transfer complete flag is set
// 	while(!(I2C2->ISR & (1 << 6))){
// 		//do nothing
// 	}
// }


// int readI2C(uint8_t slaveAddy, uint8_t regAddy, unsigned int numBytes, signed char* returnMsg){
// 	////////////////write slave address ////////////////////////////
// 	writeI2C(slaveAddy, 1, &regAddy);

// 	//configure transaction params	
// 	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear CR2 nbytes and sadd registers
// 	//I2C2->CR2 = 0; //clear entire register NBYTES and SADD
// 	I2C2->CR2 |= (slaveAddy << 1); //set SADD (slave address)
// 	I2C2->CR2 |= (1 << 10); //set read direction
// 	I2C2->CR2 |= (numBytes << 16); //set bytes to transfer
// 	I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675
		
// 	//////////////////read slave msg////////////////////////////
// 	for(int i = 0; i < numBytes; ++i){
// 		//wait unitl RXNE flags are set, periphrefman.pdf pg. 680
// 		while(!(I2C2->ISR & ((1 << 2) | (1 << 4)))){ 
// 			if(I2C2->ISR & (1<<4)){ //if NACKF set
// 				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//throw err, red LED on
// 				return NULL;	//throw err
// 			}
// 		}
// 		returnMsg[i] = I2C2->RXDR;
// 	}
// 	//wait till transfer complete flag is set
// 	while(!(I2C2->ISR & (1 << 6))){
// 		//do nothing
// 	}
// 	I2C2->CR2 |= (1 << 14); //set stop bit
// 	return 1;
// }





























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

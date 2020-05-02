#include <MPU9250.h>

//********************TIMING**************************
//for timing
void TIM2_IRQHandler(void){ //defined in ../Application/startup_stm32f072xb.s
	g_dt += 1;
	TIM2->SR &= ~(1 << 0); //reset interrupt pending flag
}

void initTickCounter()
{
    //for timing
    const uint32_t BASE_FREQ = 8e6; //8 Mhz
    const uint32_t PSC_VAL2 = 79; // yields 100kHz clk
    const uint32_t DES_FREQ2 = 1e5; // Hz
    const uint32_t ARR_VAL2 = BASE_FREQ/(PSC_VAL2+1)/DES_FREQ2;

    //set up timer 2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable the TIM2 clock in the advanced performance bus register
    TIM2->PSC = PSC_VAL2; //this divides the base clk freq by 8000 (yields a 1kHz clock)
    TIM2->ARR = ARR_VAL2; //produces a UEV every 4Hz (BASE_FREQ/PSC/DES_FREQ = 250)
    TIM2->DIER |= (1 << 0); //enable update event (periph pg. 442)
    TIM2->CR1 |= (1 << 0); //enable timer (periph pg. 436)
    NVIC_EnableIRQ(TIM2_IRQn); //enable timer 2 interupt vector in the NVIC (defined in stm32f072xb.h)
//	NVIC_SetPriority(TIM2_IRQn, 1); //set high priority
}

//********************I2C**************************
void initI2C()
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //enable i2c2 

	//enable sda (11) and scl (13)
	GPIO_InitTypeDef i2cInitStr = {GPIO_PIN_11 | GPIO_PIN_13,
															GPIO_MODE_AF_OD, //set gpio to alternate function, open drain mode
															GPIO_SPEED_FREQ_LOW,
															GPIO_PULLUP};
		
	HAL_GPIO_Init(GPIOB, &i2cInitStr); //enable gpio PC11/13 pin
																																
	//enable af1 on PB11 and enable af5 on pb13
	GPIOB->AFR[1] |= (1 << 12) | (1 << 20) | (1 << 22);

	//configure i2c2 to 400 kHz (for MPU 9250)														
	I2C2->TIMINGR &= ~(0xFU << 28); //PRESC = 0;
	I2C2->TIMINGR |= (0x9 << 0); //SCLL = 0x9;
	I2C2->TIMINGR |= (0x3 << 8); //SCLH = 0x3;
	I2C2->TIMINGR |= (0x1 << 16); //SDADEL = 0x1;	
	I2C2->TIMINGR |= (0x3 << 20); //SCLDEL = 0x3;
	I2C2->CR1 |= (1 << 0); //peripheral enabled
}

void writeI2C(uint8_t slaveAddy, uint8_t numBytes, uint8_t* msg){
	//configure transaction params
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear CR2 nbytes and sadd registers
	// I2C2->CR2 = 0; //clear CR2 register
	I2C2->CR2 |= (slaveAddy << 1); //set SADD (slave address)
	I2C2->CR2 &= ~(1 << 10); //set write direction
	I2C2->CR2 |= (numBytes << 16); //set  byte to transfer
	I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675

	for(int i = 0; i < numBytes; ++i){
		//wait unitl TXIS flags are set, periphrefman.pdf pg. 680
		while(!(I2C2->ISR & (1 << 1))){ 
			if(I2C2->ISR & (1 << 4)){ //if NACKF set
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//throw err, red led on
			}

		}
		//send slave msg
		I2C2->TXDR = msg[i];
	}
	//wait till transfer complete flag is set
	while(!(I2C2->ISR & (1 << 6))){
		//do nothing
	}
}

int readI2C(uint8_t slaveAddy, uint8_t regAddy, unsigned int numBytes, signed char* returnMsg){
	////////////////write slave address ////////////////////////////
	writeI2C(slaveAddy, 1, &regAddy);

	//configure transaction params	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //clear CR2 nbytes and sadd registers
	//I2C2->CR2 = 0; //clear entire register NBYTES and SADD
	I2C2->CR2 |= (slaveAddy << 1); //set SADD (slave address)
	I2C2->CR2 |= (1 << 10); //set read direction
	I2C2->CR2 |= (numBytes << 16); //set bytes to transfer
	I2C2->CR2 |= (1 << 13); //set start bit, periprefman.pdf pg 675
		
	//////////////////read slave msg////////////////////////////
	for(int i = 0; i < numBytes; ++i){
		//wait unitl RXNE flags are set, periphrefman.pdf pg. 680
		while(!(I2C2->ISR & ((1 << 2) | (1 << 4)))){ 
			if(I2C2->ISR & (1<<4)){ //if NACKF set
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//throw err, red LED on
				return NULL;	//throw err
			}
		}
		returnMsg[i] = I2C2->RXDR;
	}
	//wait till transfer complete flag is set
	while(!(I2C2->ISR & (1 << 6))){
		//do nothing
	}
	I2C2->CR2 |= (1 << 14); //set stop bit
	return 1;
}

//********************IMU**************************
//returns true if correct chips are enslaved
uint8_t initIMU()
{
	//init mpu
	uint8_t configArr[] = {mMPU_INT_PIN_CONFIG_REG, mMPU_INT_PIN_CONFIG_SET, mMPU_GYRO_CONFIG_REG, mMPU_GYRO_CONFIG_SET, mMPU_ACCEL_CONFIG_REG, mMPU_ACCEL_CONFIG_SET};
	writeI2C(mMPU_ADDR, 6, configArr);
		
	//init magnetometer
	uint8_t magConfigArr[] = {mMAG_CTRL_REG, mMAG_CTRL_SET};
	writeI2C(mMAG_ADDR, 2, magConfigArr);
	
	//check comms are up with the correct slave
	uint8_t isCommsUp;
	int8_t whoAmIReturn;
	
	readI2C(mMAG_ADDR, mMAG_WAI_REG, 1, &whoAmIReturn);

	if(whoAmIReturn & mMAG_WAI_VAL)
	{
		isCommsUp = 1;
	}
	else
	{
		return 0;
	}
	
	readI2C(mMPU_ADDR, mMPU_WHO_AM_I_REG, 1, &whoAmIReturn);

	if(whoAmIReturn & mMPU_WHO_AM_I_VAL)
	{
		isCommsUp = 1;
	}
	else
	{
		return 0;
	}
	
	return isCommsUp;
}

void updateIMU(struct Orientation* ont, float* magneticBearing)
{
	signed char accelOutRaw[6];
	float accelOut[3];
	signed char gyroOutRaw[6];
	float gyroOut[3];
	signed char magOutRaw[6];
	float magOut[3];
	float accelYawAng;
	float gyroYawAng;
	
	//read data
	readI2C(mMPU_ADDR, mMPU_ACCEL_OUT_REG, 6, accelOutRaw);
	readI2C(mMPU_ADDR, mMPU_GYRO_OUT_REG, 6, gyroOutRaw);

	int8_t isReady;

	do
	{
		readI2C(mMAG_ADDR, mMAG_ST1_REG, 1, &isReady);
	}
	while(!(isReady & 0x01));

	readI2C(mMAG_ADDR, mMAG_OUT_REG, 7, magOutRaw);

	//assemble data
	for (int i = 0; i < 3; i++)
	{
		//accel (big-endian)
		rawData_to_int.rawData[1] = accelOutRaw[i*2];
		rawData_to_int.rawData[0] = accelOutRaw[(i*2)+1];
		accelOut[i] = rawData_to_int.i * 4.0 / pow(2, 16); //4.0 => +-2.0g's (setup in accel config), 2^16 (resolution)
		//gyro (big-endian)
		rawData_to_int.rawData[1] = gyroOutRaw[i*2];
		rawData_to_int.rawData[0] = gyroOutRaw[(i*2)+1];
		gyroOut[i] = rawData_to_int.i * 500.0 / pow(2, 16); //500.0 => +250.0 dps (setup in gyro config), 2^16 (resolution)
		//mag (little-endian)
		rawData_to_int.rawData[0] = magOutRaw[i*2];
		rawData_to_int.rawData[1] = magOutRaw[(i*2)+1];
		magOut[i] = rawData_to_int.i * (4912.0 * 2) / pow(2,16); //4912.0*2 => +- 4912 uT (setup in mag config), 2^16 (resolution)
	}

	//convert data to the good stuff (bearings and position)
	*magneticBearing = radToDeg(atan2(magOut[1], magOut[0])) - MAGNETIC_DECLINATION_SLC;
	accelYawAng = radToDeg(asin(accelOut[2] / sqrt((accelOut[0] * accelOut[0]) + (accelOut[1] * accelOut[1]) + (accelOut[2] * accelOut[2]))));
	gyroYawAng += radToDeg(degToRad(gyroOut[2])) * (g_dt * 1e-5); 
	ont->pos.v.x += accelOut[0] * 9.81 * g_dt * 1e-5;
	ont->pos.v.y += accelOut[1] * 9.81 * g_dt * 1e-5;
	ont->pos.o.x += ont->pos.v.x * g_dt * 1e-5;
	ont->pos.o.y += ont->pos.v.y * g_dt * 1e-5;
	g_dt = 0;

	//complimentary filter
	ont->rot.o.z = gyroYawAng * COMP_FILT_GYRO_GAIN + accelYawAng * COMP_FILT_ACC_GAIN;
	
	//fill remaining orientation members
	ont->pos.a.x = accelOut[0];
	ont->pos.a.y = accelOut[1];
	ont->pos.a.z = accelOut[2];
	ont->rot.v.x = gyroOut[0];
	ont->rot.v.y = gyroOut[1];
	ont->rot.v.z = gyroOut[2];
}

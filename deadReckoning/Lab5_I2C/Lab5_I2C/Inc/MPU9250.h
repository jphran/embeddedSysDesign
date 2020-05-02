#ifndef MPU9250_H
#define MPU9250_H

    #include <ins_types.h>

    //filters
    #define COMP_FILT_GYRO_GAIN 0.9
    #define COMP_FILT_ACC_GAIN (1 - COMP_FILT_GYRO_GAIN)
    #define M_PI 3.14159
    #define radToDeg(rad) (rad * 180.0 / M_PI)
    #define degToRad(deg) (deg * M_PI / 180.0)

    //For use with the MPU 9250
    static const uint8_t mMPU_ADDR = 0x68U; //AD0 is pulled low
    static const uint8_t mMPU_WHO_AM_I_REG = 0x75U;
    static const uint8_t mMPU_WHO_AM_I_VAL = 0x71U;
    static const uint8_t mMPU_CONFIG_REG = 0x1AU;
    static const uint8_t mMPU_INT_PIN_CONFIG_REG = 0x37U;
    static const uint8_t mMPU_INT_PIN_CONFIG_SET = 0x02U;
    static const uint8_t mMPU_GYRO_CONFIG_REG = 0x1BU; //given (X, Y, Z) in big-endian
    static const uint8_t mMPU_GYRO_CONFIG_SET = 0x00U; //set gyro to +- 250 [dps]
    static const uint8_t mMPU_ACCEL_CONFIG_REG = 0x1CU; 
    static const uint8_t mMPU_ACCEL_CONFIG_SET = 0x00; //set accel to +- 2 [g]
    static const uint8_t mMAG_ADDR = 0x0CU; 
    static const uint8_t mMAG_WAI_REG = 0x00U;
    static const uint8_t mMAG_WAI_VAL = 0x48U;
    static const uint8_t mMAG_CTRL_REG = 0x0AU;
    static const uint8_t mMAG_CTRL_SET = 0x16U; //or 0x12U, TODO: figure out what mode 1 vs 2 is
    static const uint8_t mMPU_GYRO_OUT_REG = 0x43U; 
    static const uint8_t mMPU_ACCEL_OUT_REG = 0x3BU; //registers 59-64, given (X, Y, Z) in big-endian
    static const uint8_t mMAG_ST1_REG = 0x02U; //used to check if data ready
    static const uint8_t mMAG_OUT_REG = 0x03U; //given (X, Y, Z) in two's complement little-endian
    static const float MAGNETIC_DECLINATION_SLC = 12.5; //deg east
    static int8_t g_dt = 0;

    static union u{
        int16_t i;
        char rawData[sizeof(int16_t)];
    } rawData_to_int;

    //blocking method. reads i2c transaction for n bytes
    int readI2C(uint8_t slaveAddy, uint8_t regAddy, unsigned int numBytes, signed char *returnMsg);
    //blocking method that writes to a slave, writes n bytes, first byte in msg (msg[0]) is register address
    void writeI2C(uint8_t slaveAddy, uint8_t numBytes, uint8_t *msg);
    //for timing
    extern void TIM2_IRQHandler(void);

    void initTickCounter(void);

    void initI2C(void);

    //returns true if correct chips are enslaved
    uint8_t initIMU(void);

    void updateIMU(struct Orientation* ont, float* magneticBearing);

#endif


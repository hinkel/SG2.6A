#include "board.h"
#include "mw.h"

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 hardware
#define MPU6050_ADDRESS         0x68

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6050_SMPLRT_DIV      0       //8000Hz

//#define MPU6050_DLPF_CFG   0// 256 Hz//#define MPU6050_DLPF_CFG   1 // 188 HZ//#define MPU6050_DLPF_CFG   2//  98 HZ
#define MPU6050_DLPF_CFG   3        //  42 Hz

#define MPU6000ES_REV_C4        0x14
#define MPU6000ES_REV_C5        0x15
#define MPU6000ES_REV_D6        0x16
#define MPU6000ES_REV_D7        0x17
#define MPU6000ES_REV_D8        0x18
#define MPU6000_REV_C4          0x54
#define MPU6000_REV_C5          0x55
#define MPU6000_REV_D6          0x56
#define MPU6000_REV_D7          0x57
#define MPU6000_REV_D8          0x58
#define MPU6000_REV_D9          0x59

static void mpu6050AccInit(void);
static void mpu6050AccRead(int16_t *accData);
static void mpu6050AccAlign(int16_t *accData);
static void mpu6050GyroInit(void);
static void mpu6050GyroRead(int16_t *gyroData);
static void mpu6050GyroAlign(int16_t *gyroData);
static void mpu6050TempRead(float *tempData);

//static uint8_t mpuAccelHalf = 0;

bool mpu6050Detect(sensor_t * acc, sensor_t * gyro)
{
    bool ack;
    uint8_t sig;//, rev;
//    uint8_t tmp[6];

    delay(35);                  // datasheet page 13 says 30ms. other stuff could have been running meanwhile. but we'll be safe

    ack = i2cRead(MPU6050_ADDRESS, MPU_RA_WHO_AM_I, 1, &sig);
    if (!ack)
        return false;

    // So like, MPU6xxx has a "WHO_AM_I" register, that is used to verify the identity of the device.
    // The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C address.
    // The least significant bit of the MPU-60X0’s I2C address is determined by the value of the AD0 pin. (we know that already).
    // But here's the best part: The value of the AD0 pin is not reflected in this register.
    if (sig != (MPU6050_ADDRESS & 0x7e))
        return false;

/*
    // determine product ID and accel revision
    i2cRead(MPU6050_ADDRESS, MPU_RA_XA_OFFS_H, 6, tmp);
    rev = ((tmp[5] & 0x01) << 2) | ((tmp[3] & 0x01) << 1) | (tmp[1] & 0x01);
    if (rev)
    {                                                              // Congrats, these parts are better.
        if (rev == 1)
        {
            mpuAccelHalf = 1;
        }
        else if (rev == 2)
        {
            mpuAccelHalf = 0;
        }
        else
        {
            failureMode(5);
        }
    }
    else
    {
        i2cRead(MPU6050_ADDRESS, MPU_RA_PRODUCT_ID, 1, &sig);
        rev = sig & 0x0F;
        if (!rev)
        {
            failureMode(5);
        }
        else if (rev == 4)
        {
            mpuAccelHalf = 1;
        }
        else
        {
            mpuAccelHalf = 0;
        }
    }
*/

    acc->init         = mpu6050AccInit;
    acc->read         = mpu6050AccRead;
    acc->align        = mpu6050AccAlign;
    gyro->init        = mpu6050GyroInit;
    gyro->read        = mpu6050GyroRead;
    gyro->align       = mpu6050GyroAlign;
    gyro->temperature = mpu6050TempRead;
    return true;
}

static void mpu6050AccInit(void) {
// ACC Init stuff. Moved into gyro init because the reset would screw up accel config.
}

static void mpu6050GyroInit(void)
{
    uint8_t DLPFCFG;
    gpio_config_t gpio;

    // MPU_INT output on rev4/5 hardware (PB13, PC13)
    gpio.pin = Pin_13;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    if (hse_value == 8000000) gpioInit(GPIOB, &gpio);
    else if (hse_value == 12000000) gpioInit(GPIOC, &gpio);

    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(5);
    i2cWrite(MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    /*  DLPF_CFG 0: ACC 260Hz // GYRO 256Hz
    	  DLPF_CFG 1: ACC 184Hz // GYRO 188Hz
    	  DLPF_CFG 2: ACC 94Hz  // GYRO  98Hz
    	  DLPF_CFG 3: ACC 44Hz  // GYRO  42Hz
    	  DLPF_CFG 4: ACC 21Hz  // GYRO  20Hz
    	  DLPF_CFG 5: ACC 10Hz  // GYRO  10Hz
    	  DLPF_CFG 6: ACC  5Hz  // GYRO   5Hz*/
    switch (cfg.gy_lpf)
    {
    case 256:
        DLPFCFG = 0;
        break;
    case 188:
        DLPFCFG = 1;
        break;
    case 98:
        DLPFCFG = 2;
        break;
    default:
    case 42:
        DLPFCFG = 3;
        break;
    case 20:
        DLPFCFG = 4;
        break;
    case 10:
        DLPFCFG = 5;
        break;
    case 5:
        DLPFCFG = 6;
        break;
    }
    i2cWrite(MPU6050_ADDRESS, MPU_RA_CONFIG, DLPFCFG);       // CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, 0x18);     // GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
//    i2cWrite(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, 2 << 3);// Accel scale 8g (4096 LSB/g)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, 1 << 3);  // Accel scale 4g (8192 LSB/g)    
}

static void mpu6050AccRead(int16_t *accData)
{
    uint8_t buf[6];
    i2cRead(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

static void mpu6050GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    i2cRead(MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);         // Changed to full resolution here
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

static void mpu6050TempRead(float *tempData)
{
    uint8_t buf[2];
    int16_t temp;
    i2cRead(MPU6050_ADDRESS, MPU_RA_TEMP_OUT_H, 2, buf);
    temp =(int16_t)((buf[0] << 8) | buf[1]);
    *tempData = 36.53f + ((float)temp / 340.0f);             // That is what the invense doc says. Here Arduinopage: *tempData = ((float)temp + 12412.0f) / 340.0f;
}

static void mpu6050AccAlign(int16_t *accData)
{
    int16_t temp[2];
    temp[0]    =  accData[0];
    temp[1]    =  accData[1];
    accData[0] =  temp[1];                                   // official direction is RPY
    accData[1] = -temp[0];
    accData[2] =  accData[2];
}

static void mpu6050GyroAlign(int16_t *gyroData)
{
    gyroData[0] =  gyroData[0];                              // official direction is RPY
    gyroData[1] =  gyroData[1];
    gyroData[2] = -gyroData[2];
}

void MPU6050ReadAllShit(int16_t *accData, float *tempData, int16_t *gyroData)
{
//  0x3B acc  6 bytes
//  0x41 temp 2 bytes
//  0x43 gyro 6 bytes
    uint8_t buf[14];
    int16_t temp;
    i2cRead(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 14, buf);
    accData[0]  = (int16_t)((buf[0]  << 8) | buf[1]);
    accData[1]  = (int16_t)((buf[2]  << 8) | buf[3]);
    accData[2]  = (int16_t)((buf[4]  << 8) | buf[5]);
    temp        = (int16_t)((buf[6]  << 8) | buf[7]);
    *tempData   = 36.53f + ((float)temp / 340.0f);           // That is what the invense doc says. Here Arduinopage: *tempData = ((float)temp + 12412.0f) / 340.0f;
    gyroData[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    gyroData[1] = (int16_t)((buf[10] << 8) | buf[11]);
    gyroData[2] = (int16_t)((buf[12] << 8) | buf[13]);
}

//Degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53

#include "board.h"
#include "mw.h"

#ifdef debugmode
float gyrostddev[3];
#endif

bool   calibratingA = false;                         // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
bool   calibratingG = false;                         // So no Gyrocal when feature pass
float  MainDptCut;
bool   MpuSpecial;
extern uint16_t InflightcalibratingA;
extern int16_t  AccInflightCalibrationArmed;
extern uint16_t AccInflightCalibrationMeasurementDone;
extern uint16_t AccInflightCalibrationSavetoEEProm;
extern uint16_t AccInflightCalibrationActive;
extern uint16_t batteryWarningVoltage;
extern uint8_t  batteryCellCount;
float  gyroZero[3];                                  // Populated upon initialization
float  magCal[3];                                    // Gain for each axis, populated at sensor init

sensor_t acc;                                        // acc access functions
sensor_t gyro;                                       // gyro access functions
baro_t   baro;                                       // barometer access functions
uint8_t  accHardware = ACC_DEFAULT;                  // which accel chip is used/detected

static void Mag_Calibration(void);
static void sphere_fit_least_squares(const float x[], const float y[], const float z[], uint16_t size, float *sphere);
static void ACC_getRawRot(void);
static void Gyro_getRawRot(void);

void sensorsAutodetect(void)                         // AfroFlight32 i2c sensors
{
    int16_t deg, min;
    drv_adxl345_config_t acc_params;
    uint8_t sig          = 0;
    bool    ack          = false;
    bool    haveMpu6k    = false;
    bool    havel3g4200d = false;

    GyroScale = ((32767.0f / 16.4f) * M_PI) / ((32767.0f / 4.0f) * 180.0f * 1000000.0f); // Mwii Scale for MPU6050 & 3050
    if (mpu6050Detect(&acc, &gyro))                  // Autodetect gyro hardware. We have MPU3050 or MPU6050.
    {
        haveMpu6k = true;                            // this filled up  acc.* struct with init values
    }
    else if (l3g4200dDetect(&gyro))
    {
        havel3g4200d = true;
        GyroScale = ((32767.0f / 14.2857f) * M_PI) / ((32767.0f / 4.0f) * 180.0f * 1000000.0f);
    }
    else if (!mpu3050Detect(&gyro))
    {
        failureMode(3);                              // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
    }

retry:                                               // Accelerometer. Fuck it. Let user break shit.
    switch (cfg.acc_hdw)
    {
    case 0:                                          // autodetect
    case 1:                                          // ADXL345
        acc_params.useFifo = false;
        acc_params.dataRate = 800;                   // unused currently
        if (adxl345Detect(&acc_params, &acc))
            accHardware = ACC_ADXL345;
        if (cfg.acc_hdw == ACC_ADXL345)
            break;
        ;                                            // fallthrough
    case 2:                                          // MPU6050
        if (haveMpu6k)
        {
            mpu6050Detect(&acc, &gyro);              // yes, i'm rerunning it again.  re-fill acc struct
            accHardware = ACC_MPU6050;
            if (cfg.acc_hdw == ACC_MPU6050)
                break;
        }
        ; // fallthrough
    case 3:                                          // MMA8452
        if (mma8452Detect(&acc))
        {
            accHardware = ACC_MMA8452;
            if (cfg.acc_hdw == ACC_MMA8452)
                break;
        }
    }

    if (accHardware == ACC_DEFAULT)                  // Found anything? Check if user fucked up or ACC is really missing.
    {
        if (cfg.acc_hdw > ACC_DEFAULT)
        {
            cfg.acc_hdw = ACC_DEFAULT;               // Nothing was found and we have a forced sensor type. Stupid user probably chose a sensor that isn't present.
            goto retry;
        }
        else
        {
            sensorsClear(SENSOR_ACC);                // We're really screwed
        }
    }

    if (sensors(SENSOR_ACC)) acc.init();             // This is a dummy call right now    

    if (feature(FEATURE_PASS))                       // Stop here we need just ACC for Vibrationmonitoring
    {
        sensorsClear(SENSOR_BARO);
        sensorsClear(SENSOR_MAG);
        return;
    }

    if (haveMpu6k && accHardware == ACC_MPU6050) MpuSpecial = true;
    else  MpuSpecial = false;
    
#ifdef BARO                                          // Crashpilot Skip Baro & Mag on feature pass
    delay(600);                                      // Let things settle
    ack = i2cRead(0x77, 0x77, 1, &sig);              // Check Baroadr.(MS & BMP) BMP will say hello here, MS not
    if ( ack) ack = bmp085Detect(&baro);             // Are we really dealing with BMP?
    if (!ack) ack = ms5611Detect(&baro);             // No, Check for MS Baro
    if (!ack) sensorsClear(SENSOR_BARO);             // Nothing successful, Nothing on 0x77, or baro defect, or some other device
    GroundAltInitialized = false;                    // Now time to init things
    if(cfg.esc_nfly) ESCnoFlyThrottle = constrain(cfg.esc_nfly, cfg.esc_min, cfg.esc_max); // Set the ESC PWM signal threshold for not flyable RPM
    else ESCnoFlyThrottle = cfg.esc_min + (((cfg.esc_max - cfg.esc_min) * 5) / 100); // If not configured, take 5% above esc_min
#endif

    gyro.init();                                     // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.

    if (havel3g4200d)
    {
        l3g4200dConfig(cfg.gy_lpf);
    }
    else
    {
        if (!haveMpu6k)
            mpu3050Config(cfg.gy_lpf);
    }

#ifdef MAG
    if (!hmc5883lDetect()) sensorsClear(SENSOR_MAG);
    deg = cfg.mag_dec / 100;                 // calculate magnetic declination
    min = cfg.mag_dec % 100;
    magneticDeclination = ((float)deg + ((float)min / 60.0f)); // heading is in decimaldeg units NO 0.1 deg shit here
#endif

    // Initialize Cut off frequencies for mainpid D and gps D (later)
    // Examples for _filter: "1 / ( 2 * PI * f_cut )"
    // 10 Hz filter = 15.9155e-3 
    // 15 Hz filter = 10.6103e-3
    // 20 Hz filter =  7.9577e-3
    // 25 Hz filter =  6.3662e-3
    // 30 Hz filter =  5.3052e-3
    MainDptCut = 0.5f / (M_PI * (float)cfg.maincuthz);
}

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 4095) * cfg.vbatscale;
}

void batteryInit(void)
{
    uint32_t i;
    uint32_t voltage = 0;

    // average up some voltage readings
    for (i = 0; i < 32; i++)
    {
        voltage += adcGetChannel(ADC_BATTERY);
        delay(10);
    }

    voltage = batteryAdcToVoltage((uint16_t)(voltage / 32));

    // autodetect cell count, going from 2S..6S
    for (i = 2; i < 6; i++)
    {
        if (voltage < i * cfg.vbatmaxcellvoltage)
            break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * cfg.vbatmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

// ALIGN_GYRO = 0,
// ALIGN_ACCEL = 1,
// ALIGN_MAG = 2
void alignSensors(uint8_t type, int16_t *data)
{
    uint8_t i;
    int16_t tmp[3];

    tmp[0] = data[0];                                             // make a copy :(
    tmp[1] = data[1];
    tmp[2] = data[2];
    for (i = 0; i < 3; i++)
    {
        int8_t axis = cfg.align[type][i];
        if (axis > 0)data[axis - 1] = tmp[i];
        else data[-axis - 1] = -tmp[i];
    }
}

static void Acc_1KHz_Calib_SumUp(float *x, float *y, float *z, uint16_t count)
{
    uint16_t i;
    *x = *y = *z = 0;
    for (i = 0; i < count; i++)
    {
        delay(1);
        ACC_getRawRot();
        *x += accADC[0];
        *y += accADC[1];
        *z += accADC[2];
    }
}

#define ACC1Gcount   1000
#define ACCmaxcount   500
#define ACCdiscardcnt 100
#define ACCavgcount    20
static void Acc_Calibrate(void)                                   // Total Samples = ACCmaxcount * ACCavgcount + ACC1Gcount + ACCdiscardcnt
{
    float    x[ACCmaxcount], y[ACCmaxcount], z[ACCmaxcount];
    float    TempX, TempY, TempZ;
    uint16_t i;
#ifdef debugmode  
    stdev_t  var[3];
#endif
    Acc_1KHz_Calib_SumUp(&TempX, &TempY, &TempZ, ACCdiscardcnt);  // Discard the first 100 Values
    Acc_1KHz_Calib_SumUp(&TempX, &TempY, &TempZ, ACC1Gcount);     // Summ up 1000 values to get the sensor 1G
    cfg.sens_1G = sqrtf(TempX * TempX + TempY * TempY + TempZ * TempZ) / ACC1Gcount;

#ifdef debugmode
    for (i = 0; i < 3; i++) devClear(&var[i]);
#endif

    for (i = 0; i < ACCmaxcount; i++)
    {
        Acc_1KHz_Calib_SumUp(&TempX, &TempY, &TempZ, ACCavgcount);// Average 20 ACC Values here
        x[i] = TempX / ACCavgcount;
        y[i] = TempY / ACCavgcount;
        z[i] = TempZ / ACCavgcount - cfg.sens_1G;
#ifdef debugmode
        devPush(&var[0], x[i]);
        devPush(&var[1], y[i]);
        devPush(&var[2], z[i]);
#endif
    }
    sphere_fit_least_squares(x, y, z, ACCmaxcount, cfg.accZero);
    cfg.angleTrim[ROLL] = cfg.angleTrim[PITCH] = 0;
#ifdef debugmode
    cfg.accstddev[0] = devStandardDeviation(&var[0]);
    cfg.accstddev[1] = devStandardDeviation(&var[1]);
    cfg.accstddev[2] = devStandardDeviation(&var[2]);
#endif
    writeParams(1);
    systemReset(false);
}

static void ACC_Common(void)
{
    uint8_t axis;

    if (calibratingA) Acc_Calibrate();                            // Dead end (Freeze, Calibrate, Save, Reset)

    if (feature(FEATURE_INFLIGHT_ACC_CAL))                        // THAT IS ATAVISTIC AND MUST BE REDONE
    {
        static float   b[3];
        static float   accZero_saved[3];                          //   = { 0, 0, 0 };
        static int16_t angleTrim_saved[2];                        // = { 0, 0 };

        if (InflightcalibratingA == 50)                           // Saving old zeropoints before measurement
        {
            for (axis = 0; axis < 2; axis++)
            {
                accZero_saved[axis]   = cfg.accZero[axis];
                angleTrim_saved[axis] = cfg.angleTrim[axis];
            }
            accZero_saved[YAW] = cfg.accZero[YAW];
        }
        if (InflightcalibratingA > 0)
        {
            for (axis = 0; axis < 3; axis++)
            {
                if (InflightcalibratingA == 50) b[axis] = cfg.accZero[axis] = 0;
                b[axis] += accADC[axis];
            }
            if (InflightcalibratingA == 1)
            {
                AccInflightCalibrationActive          = 0;
                AccInflightCalibrationMeasurementDone = 1;
                toggleBeep                            = 2;        // buzzer for indicating the end of calibration

                for (axis = 0; axis < 2; axis++)
                {
                    cfg.accZero[axis]   = accZero_saved[axis];
                    cfg.angleTrim[axis] = angleTrim_saved[axis];
                }
                cfg.accZero[YAW] = accZero_saved[YAW];              
            }
            InflightcalibratingA--;
        }
        if (AccInflightCalibrationSavetoEEProm == 1)              // the copter is landed, disarmed and the combo has been done again
        {
            AccInflightCalibrationSavetoEEProm = 0;
            for (axis = 0; axis < 3; axis++) b[axis] /= 50;
            cfg.sens_1G         = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
            for (axis = 0; axis < 3; axis++) cfg.accZero[axis] = b[axis];
            cfg.accZero[YAW]   -= cfg.sens_1G;
            cfg.angleTrim[ROLL] = cfg.angleTrim[PITCH] = 0;
            writeParams(1);                                       // write accZero in EEPROM
            systemReset(false);
        }
    }
    for (axis = 0; axis < 3; axis++) accADC[axis] = ((accADC[axis] - cfg.accZero[axis]) * acc_1G) / cfg.sens_1G; // Scale to mwii shit 512
}

#define Gyromaxcount   500
#define Gyrodiscardcnt 100
#define Gyroavgcount     3
static void Gyro_Calibrate(void)                                  // Total Samples = Gyromaxcount * Gyroavgcount + Gyrodiscardcnt
{
    float    xyz[3][Gyromaxcount];
    float    Temp[3];                                             // Code is shorter with Temp variable
    uint16_t i;
    uint8_t  k, l, breakout = 0;
    stdev_t  var[3];

    for (i = 0; i < Gyrodiscardcnt; i++)                          // Discard first 100 reads
    {
        delay(1);
        Gyro_getRawRot();
    }
  
    do                                                            // Shaky Hands Loop
    {
        for (k = 0; k < 3; k++) devClear(&var[k]);
        breakout++;
        for (i = 0; i < Gyromaxcount; i++)
        {
            for (k = 0; k < 3; k++) Temp[k] = 0;
            for (k = 0; k < Gyroavgcount; k++)
            {
                delay(1);
                Gyro_getRawRot();
                for (l = 0; l < 3; l++) Temp[l] += gyroADC[l];
            }
            for (k = 0; k < 3; k++)
            {
                xyz[k][i] = Temp[k] / Gyroavgcount;
                devPush(&var[k], xyz[k][i]);
            }
        }
        Temp[0] = 0;
        for (k = 0; k < 3; k++)
        {
            Temp[0] += devStandardDeviation(&var[k]);
#ifdef debugmode
            gyrostddev[k] = devStandardDeviation(&var[k]);
#endif
        }
    }
    while (Temp[0] > cfg.gy_stdev && breakout < 14);              // Breakout prevents endlessloop after 21 sec
    sphere_fit_least_squares(xyz[0], xyz[1], xyz[2], Gyromaxcount, gyroZero);
    calibratingG = false;
}

static void GYRO_Common(void)
{
    int8_t axis;
    static int16_t previousGyroADC[3] = { 0, 0, 0 };

    if (calibratingG) Gyro_Calibrate();

    for (axis = 0; axis < 3; axis++)
    {
        gyroADC[axis]  = (gyroADC[axis] - gyroZero[axis]) * 0.25f;
        gyroADC[axis]  = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800); // gyroADC[axis]  = constrain(gyroADC[axis], previousGyroADC[axis] - 3200, previousGyroADC[axis] + 3200);//anti gyro glitch, limit the variation between two consecutive readings
        gyroData[axis] = gyroADC[axis];   // Feed gyroData here as well
        previousGyroADC[axis] = gyroADC[axis];
    }
}

void GETMPU6050(void)
{
    int16_t accADC16[3];  
    int16_t gyroADC16[3];
    uint8_t i;
    MPU6050ReadAllShit(accADC16, &telemTemperature1, gyroADC16);
    if (cfg.align[ALIGN_ACCEL][0]) alignSensors(ALIGN_ACCEL, accADC16); else acc.align(accADC16);
    if (cfg.align[ALIGN_GYRO][0])  alignSensors(ALIGN_GYRO, gyroADC16); else gyro.align(gyroADC16);  
    for (i = 0; i < 3; i++)
    {
        accADC[i]  = accADC16[i];
        gyroADC[i] = gyroADC16[i];
    }
    ACC_Common();
    GYRO_Common();
}

static void ACC_getRawRot(void)
{
    int16_t accADC16[3];
    acc.read(accADC16);
    if (cfg.align[ALIGN_ACCEL][0]) alignSensors(ALIGN_ACCEL, accADC16); else acc.align(accADC16);
    accADC[0] = accADC16[0];
    accADC[1] = accADC16[1];
    accADC[2] = accADC16[2];
}

static void Gyro_getRawRot(void)
{
    int16_t gyroADC16[3];
    gyro.read(gyroADC16);     // range: +/- 8192; +/- 2000 deg/sec
    if (cfg.align[ALIGN_GYRO][0]) alignSensors(ALIGN_GYRO, gyroADC16); else gyro.align(gyroADC16);
    gyroADC[0] = gyroADC16[0];
    gyroADC[1] = gyroADC16[1];
    gyroADC[2] = gyroADC16[2];  
}

void ACC_getADC(void)
{
    ACC_getRawRot();
    ACC_Common();
}

void Gyro_getADC(void)
{
    Gyro_getRawRot();
    GYRO_Common();
}

#ifdef BARO
void Baro_update(void)                                            // Note Pressure is now global for telemetry 1hPa = 1mBar
{
    static float    BaroSpikeTab[5];
    static uint32_t LastBaroTime, LastGeneraltime;
    static uint16_t TimeTable[10], baroDeadline = 0;
    static uint8_t  state = 0, idx = 0, SkipCnt = 0;
    float           extmp;
    bool            rdy;
    uint8_t         sortidx, maxsortidx, i;
    uint32_t        TimeNowMicros, TimeSum = 0;

    newbaroalt = false;                                           // Reset Newbarovalue since it's iterative and not interrupt driven it's OK
    if (micros() - LastGeneraltime < baroDeadline) return;        // Make it rollover friendly
    switch (state)
    {
    case 0:
        baro.start_ut();                                          // Temperature Conversion start
        LastGeneraltime = micros();
        baroDeadline    = baro.ut_delay - 1;
        SkipCnt = 0;                                              // Reset Skipcounter, reduces 27ms delay to average 20ms delay for ms baro (37Hz to 50Hz)
        state++;
        break;
    case 1:
        baro.get_ut();                                            // Readout Temp fall through case
        state++;
    case 2:
        baro.start_up();                                          // Pressure Conversion start
        LastGeneraltime = micros();
        baroDeadline    = baro.up_delay - 1;
        state++;
        break;
    case 3:
        baro.get_up();                                            // Readout Pressure
        baroDeadline    = 0;                                      // Don't use delay between read. Cycletime is enough. Before: TimeNowMicros + baro.repeat_delay - 1;
        ActualPressure  = baro.calculate();
        TimeNowMicros   = micros();                               // Do timestuff
        TimeTable[idx]  = TimeNowMicros - LastBaroTime;           // Filter Timejitter here
        LastBaroTime    = TimeNowMicros;
        idx++;
        if (idx == 10) idx = 0;
        for (i = 0; i < 10; i++) TimeSum += TimeTable[i];
        BaroDeltaTime   = (float)TimeSum * 0.1f;                  // BaroDeltaTime = TimeNowMicros - LastBaroTime;
        BaroSpikeTab[4] = ActualPressure;                         // feed both ends of array with new data
        BaroSpikeTab[0] = ActualPressure;                         // feed both ends of array with new data
        rdy             = false;
        maxsortidx      = 4;
        while(!rdy)
        {
            rdy = true;
            for (sortidx = 0; sortidx < maxsortidx; sortidx++)
            {
                extmp = BaroSpikeTab[sortidx];
                if (extmp > BaroSpikeTab[sortidx+1])
                {
                    BaroSpikeTab[sortidx]   = BaroSpikeTab[sortidx+1];
                    BaroSpikeTab[sortidx+1] = extmp;
                    rdy = false;
                }
            }
            maxsortidx --;
        }
        if(GroundAltInitialized) ActualPressure = BaroSpikeTab[2];// Do Groundalt with raw values to prevent runup from old sikefilter value
        BaroAlt = (1.0f - pow(ActualPressure / 101325.0f, 0.190295f)) * 4433000.0f; // Centimeter. Actual mwii/bf does it the apm way, no doubt it's faster but better? I stick to the slower, more precise method.
        SkipCnt++ ;
        if (SkipCnt == 2 || baro.baro_type == 1) state = 0;       // Read new Temp every 2nd run gives us little more speed without loosing resolution. However it worsens BMP - so not done there // baro_type: 1 = BMP 2 = MS
        else state = 2;
        newbaroalt = true;
        break;
    }
}
#endif

#ifdef SONAR
void Sonar_init(void)                                                                 // 0 = PWM56, 1 = RC78, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4 = MBRC78
{
    uint8_t utmp8;
    if (Snr_init())                                                                   // If snr hardware-init was successful
    {
        // Check for user configuration error.
        if (cfg.snr_min == cfg.snr_max)                                               // General min & max Check
        {
            cfg.snr_min = 50;                                                         // Use some safe values then
            cfg.snr_max = 100;
        }
        if (cfg.snr_min > cfg.snr_max)                                                // OMG User mixed up min & max
        {
            utmp8       = cfg.snr_max;                                                // Swap values 8 bit is sufficient snr_min (5...200)
            cfg.snr_max = cfg.snr_min;
            cfg.snr_min = utmp8;
        }
        switch(cfg.snr_type)                                                          // Sensor specific min & max check
        {
        case 0:                                                                       // Check for HC-SR04
        case 1:
        case 2:
            cfg.snr_max = min(cfg.snr_max, 400);                                      // Adjust snr_max for HC-SR04 // 10 is lower limit in cli already!
            break;
        case 3:                                                                       // Check for Maxbotics
        case 4:
            cfg.snr_min = max(cfg.snr_min,  30);                                      // Adjust snr_min for Maxbotics // "700" is upper limit in cli already!
            break;
        }
        sensorsSet(SENSOR_SONAR);                                                     // Signalize Sonar available (esp with I2C Sonar), or PWM Pins initialized
        sonarAlt    = -1;                                                             // Initialize with errorvalue
        SonarStatus =  0;
    }
}

// THIS WILL BE CALLED, ONLY WHEN GroundAlt is Initialized !!
#define SonarErrorLimit 5                                                             // We will bridge 5 consecutive faulty reads, HC-SR04 = 300ms Maxbotix = 500ms
void Sonar_update(void)
{
    static  int16_t  LastGoodSonarAlt = -1;                                           // Initialize with errorvalue
    static  uint32_t AcceptTimer = 0;
    static  uint8_t  Errorcnt = 0;                                                    // This is compared to SonarErrorLimit
    bool    newdata = false;
    int16_t LastSonarAlt = sonarAlt;                                                  // Save Last Alt here for comparison
    uint8_t tilt;
  
    switch (cfg.snr_type)                                                             // 0 = PWM56, 1 = RC78, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4 = MBRC78
    {
    case 0:                                                                           // sonar_pwm56 HC-SR04
    case 1:                                                                           // sonar_rc78  HC-SR04
        newdata = hcsr04_get_distancePWM(&sonarAlt);                                  // Look for HC-SR04 Sonar Data
        break;
    case 2:
        newdata = DaddyW_get_i2c_distance(&sonarAlt);                                 // Ask DaddyW I2C Sonar for Data
        break;
    case 3:                                                                           // sonar_pwm56 Maxbotics
    case 4:                                                                           // sonar_rc78  Maxbotics
        newdata = MaxBotix_get_distancePWM(&sonarAlt);                                // Look for Maxbotics Sonar Data
        break;
    }

    if (newdata)                                                                      // 100 ms with Maxbotix, 60ms with HC-SR04
    {
        tilt = 100 - constrain(TiltValue * 100.0f, 0, 100.0f);                        // We don't care for upsidedownstuff, because althold is disabled than anyway
        if (cfg.snr_dbg) { debug[1] = tilt; debug[2] = sonarAlt; }                    // Give raw tilt & sonar in debugmode
        if (sonarAlt >= cfg.snr_min && sonarAlt <= cfg.snr_max && tilt < cfg.snr_tilt)
        {
            LastGoodSonarAlt = sonarAlt;
            Errorcnt = SonarBreach = 0;                                                          // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
        }
        else
        {                                                                             // So sonarvalues are not sane here
            Errorcnt = min(Errorcnt + 1, SonarErrorLimit);                            // Increase Errorcount within upper limit            
            if (tilt < cfg.snr_tilt && sonarAlt != -1)                                // Determine Limit breach type independent of tilt
            {
                if (sonarAlt <= cfg.snr_min) SonarBreach = 1;                         // We breached lower limit
                if (sonarAlt >= cfg.snr_max) SonarBreach = 2;                         // We breached upper limit
            }
            else SonarBreach = 0;                                                     // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
            if (Errorcnt != SonarErrorLimit) sonarAlt = LastGoodSonarAlt;             // Bridge error with last value, when it's -1 we take care later
            else sonarAlt = -1;
        }

        if (LastSonarAlt != -1 && sonarAlt != -1 && cfg.snr_diff && abs(sonarAlt - LastSonarAlt) > cfg.snr_diff) // Too much Difference between reads?
            sonarAlt = -1;
        
        if (sonarAlt < 0)                                                             // Handle error here separately
        {
            LastGoodSonarAlt = -1;
            SonarStatus = 0;
        }
        else
        {
            switch(SonarStatus)
            {
            case 0:
                SonarStatus++;                                                        // Definition of "SonarStatus" 0 = no contact, 1 = Made first contact, 2 = Steady contact
                AcceptTimer = currentTimeMS + 700;                                    // Set 700 ms timeout before signalizing "steady contact" this exceeds our "bridging" from above
                break;
            case 1:
                if (currentTimeMS >= AcceptTimer) SonarStatus++;                      // 2 = Steady contact // imu/getEstimatedAltitude will be happy to know
            default:
                break;
            }
        }
    }
    if (cfg.snr_dbg) debug[0] = sonarAlt;                                             // Display Sonaralt like seen by althold
}
#endif

#ifdef MAG
/* TC notes about mag orientation
 Default mag orientation is -2, -3, 1 or no way? Is THIS finally the proper orientation?? (by GrootWitBaas)
 magADC[ROLL] = rawADC[2];   // X
 magADC[PITCH] = -rawADC[0]; // Y
 magADC[YAW] = -rawADC[1];   // Z
*/

// Rearranged & Extended by Crashpilot
void Mag_init(void)                                               // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
{
    LD1_ON();
    hmc5883lInit(magCal);                                         // Crashpilot: Calculate Gains / Scale
    LD1_OFF();
}

static void Mag_getRawADC(void)                                   // Read aligned
{
    int16_t magADC[3];
    uint8_t i;
    hmc5883lRead(magADC);                                         // Does that now: alignSensors(ALIGN_MAG, magADC);
    for (i = 0; i < 3; i++) magADCfloat[i] = (float)magADC[i];    // Put into floats
}

void Mag_getADC(void)
{
    static uint32_t Lasttime = 0, Deadline = 0;
    uint8_t         i;
    uint32_t        TimeNow = micros();
    if (TimeNow - Lasttime < Deadline) return;                    // Make it rollover friendly that (int_32) stuff actually didn't work on baro
    Lasttime = TimeNow;
    Deadline = 14000;                                             // Do 71Hz in normal operation this is below 75 hz
    if (f.CALIBRATE_MAG)                                          // Lets calibrate
    {
        Mag_Calibration();                                        // Calibrates and saves whatever result
        f.CALIBRATE_MAG = false;                                  // Dont do this again...      
    }
    else                                                          // Normal stuff here. We apply scale & bias if calib is done
    {
        Mag_getRawADC();                                          // Read mag sensor with orientation correction do nothing more for now
        for (i = 0; i < 3; i++)
        {
            magADCfloat[i]   = magADCfloat[i] * magCal[i];        // Adjust Mag readout by GAIN/SCALE
            magADCfloat[i]  -= cfg.magZero[i];                    // AND by BIAS
        }
    }
    return;                                                       // Signalize something was done with 1
}

static void Mag_Calibration(void)                                 // Called from XHz loop normally....
{
#define MAGmaxcount      500                                      // Take 500 samples at 10Hz rate i.e 50Sec
#define MAGerror       10000
#define MAGdiscardcnt     50
    float    xyz[3][MAGmaxcount], Temp[3];
    uint16_t ValIDX;
    uint8_t  i, k;
    uint8_t  gathercnt = cfg.mag_time * 5;

    LD0_OFF();                                                    // Green LED OFF
    LD1_ON();                                                     // Red LED ON

    for (i = 0; i < MAGdiscardcnt; i++)                           // Discard first reads at 50Hz Rate
    {
        delay(20);
        Mag_getRawADC();
    }

    for (ValIDX = 0; ValIDX < MAGmaxcount; ValIDX++)              // Gather up Mag Data. Freeze FC. Adjust Mag readout JUST by GAIN/SCALE
    {
        for (i = 0; i < 3; i++) Temp[i] = 0;                      // Clear accumulator
        for (i = 0; i < gathercnt; i++)                           // Get some!
        {
            delay(24);                                            // Do 42 Hz // Math 24ms * 500 * 5 = 60000ms = 1 Minute
            Mag_getRawADC();                                      // Read mag sensor with correct orientation
            for (k = 0; k < 3; k++) Temp[k] += magADCfloat[k];
        }
        for (i = 0; i < 3; i++) xyz[i][ValIDX] = (Temp[i] / (float)gathercnt) * magCal[i];
        LED0_TOGGLE;
        LED1_TOGGLE;
    }

    sphere_fit_least_squares(xyz[0], xyz[1], xyz[2], MAGmaxcount, cfg.magZero);
    cfg.mag_calibrated = 1;
    for (i = 0; i < 3; i++)
    {
        if (fabs(cfg.magZero[i]) > MAGerror)
        {
            cfg.mag_calibrated = 0;                               // Supress GPS functions & Guicrazymag
            cfg.magZero[i] = 0;          
        }
    }
    cfg.mag_motorcompusable = 0;                                  // Future: After mag calibration a new motorcompensation is needed
    writeParams(1);                                               // Calibration done, save whatever result
}

/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *           Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Crashpilot note: Parameter slightly changed. Size to float.
 * Docu is here: https://pixhawk.ethz.ch/px4/docs/calibration__routines_8cpp.html
 ****************************************************************************/
 #define sflsdelta 0.0f
 #define maxiterations 100
void sphere_fit_least_squares(const float x[], const float y[], const float z[], uint16_t size, float *sphere)
{
    uint16_t i, n;
    float x_sumplain = 0.0f, x_sumsq = 0.0f, x_sumcube = 0.0f, y_sumplain = 0.0f, y_sumsq = 0.0f;
    float y_sumcube = 0.0f, z_sumplain = 0.0f, z_sumsq = 0.0f, z_sumcube = 0.0f, xy_sum = 0.0f;
    float xz_sum = 0.0f, yz_sum = 0.0f, x2y_sum = 0.0f, x2z_sum = 0.0f, y2x_sum = 0.0f, y2z_sum = 0.0f;
    float z2x_sum = 0.0f, z2y_sum = 0.0f, x2 = 0.0f, y2 = 0.0f, z2 = 0.0f, x_sum = 0.0f, x_sum2 = 0.0f;
    float x_sum3 = 0.0f, y_sum = 0.0f, y_sum2 = 0.0f, y_sum3 = 0.0f, z_sum = 0.0f, z_sum2 = 0.0f, z_sum3 = 0.0f;
    float XY = 0.0f, XZ = 0.0f, YZ = 0.0f, X2Y = 0.0f, X2Z = 0.0f, Y2X = 0.0f, Y2Z = 0.0f, Z2X = 0.0f, Z2Y = 0.0f;
    float F0 = 0.0f, F1 = 0.0f, F2 = 0.0f, F3 = 0.0f, F4 = 0.0f, A = 0.0f, B = 0.0f, C = 0.0f, A2 = 0.0f, B2 = 0.0f;
    float C2 = 0.0f, QS = 0.0f, QB = 0.0f, Rsq = 0.0f, Q0 = 0.0f, Q1 = 0.0f, Q2 = 0.0f, aA = 0.0f, aB = 0.0f, aC = 0.0f;
    float nA = 0.0f, nB = 0.0f, nC = 0.0f, dA = 0.0f, dB = 0.0f, dC = 0.0f;
    float fltsize = (float)size;
  
    for (i = 0; i < size; i++)
    {
        x2 = x[i] * x[i];
        y2 = y[i] * y[i];
        z2 = z[i] * z[i];
        x_sumplain += x[i];
        x_sumsq += x2;
        x_sumcube += x2 * x[i];
        y_sumplain += y[i];
        y_sumsq += y2;
        y_sumcube += y2 * y[i];
        z_sumplain += z[i];
        z_sumsq += z2;
        z_sumcube += z2 * z[i];
        xy_sum += x[i] * y[i];
        xz_sum += x[i] * z[i];
        yz_sum += y[i] * z[i];
        x2y_sum += x2 * y[i];
        x2z_sum += x2 * z[i];
        y2x_sum += y2 * x[i];
        y2z_sum += y2 * z[i];
        z2x_sum += z2 * x[i];
        z2y_sum += z2 * y[i];
    }

    //Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
    //
    //    P is a structure that has been computed with the data earlier.
    //    P.npoints is the number of elements; the length of X,Y,Z are identical.
    //    P's members are logically named.
    //
    //    X[n] is the x component of point n
    //    Y[n] is the y component of point n
    //    Z[n] is the z component of point n
    //
    //    A is the x coordiante of the sphere
    //    B is the y coordiante of the sphere
    //    C is the z coordiante of the sphere
    //    Rsq is the radius squared of the sphere.
    //
    //This method should converge; maybe 5-100 iterations or more.
    //
    x_sum  = x_sumplain / fltsize;    //sum( X[n] )
    x_sum2 = x_sumsq    / fltsize;    //sum( X[n]^2 )
    x_sum3 = x_sumcube  / fltsize;    //sum( X[n]^3 )
    y_sum  = y_sumplain / fltsize;    //sum( Y[n] )
    y_sum2 = y_sumsq    / fltsize;    //sum( Y[n]^2 )
    y_sum3 = y_sumcube  / fltsize;    //sum( Y[n]^3 )
    z_sum  = z_sumplain / fltsize;    //sum( Z[n] )
    z_sum2 = z_sumsq    / fltsize;    //sum( Z[n]^2 )
    z_sum3 = z_sumcube  / fltsize;    //sum( Z[n]^3 )
    XY     = xy_sum     / fltsize;    //sum( X[n] * Y[n] )
    XZ     = xz_sum     / fltsize;    //sum( X[n] * Z[n] )
    YZ     = yz_sum     / fltsize;    //sum( Y[n] * Z[n] )
    X2Y    = x2y_sum    / fltsize;    //sum( X[n]^2 * Y[n] )
    X2Z    = x2z_sum    / fltsize;    //sum( X[n]^2 * Z[n] )
    Y2X    = y2x_sum    / fltsize;    //sum( Y[n]^2 * X[n] )
    Y2Z    = y2z_sum    / fltsize;    //sum( Y[n]^2 * Z[n] )
    Z2X    = z2x_sum    / fltsize;    //sum( Z[n]^2 * X[n] )
    Z2Y    = z2y_sum    / fltsize;    //sum( Z[n]^2 * Y[n] )

    //Reduction of multiplications
     F0 = x_sum2 + y_sum2 + z_sum2;
     F1 =  0.5f * F0;
     F2 = -8.0f * (x_sum3 + Y2X + Z2X);
     F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
     F4 = -8.0f * (X2Z + Y2Z + z_sum3);

    //Set initial conditions:
     A = x_sum;
     B = y_sum;
     C = z_sum;

    //First iteration computation:
     A2 = A * A;
     B2 = B * B;
     C2 = C * C;
     QS = A2 + B2 + C2;
     QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);

    //Set initial conditions:
     Rsq = F0 + QB + QS;

    //First iteration computation:
     Q0 = 0.5f * (QS - Rsq);
     Q1 = F1 + Q0;
     Q2 = 8.0f * (QS - Rsq + QB + F0);

    //Iterate N times, ignore stop condition.
    n = 0;
    while (n < maxiterations)
    {
        n++;
        //Compute denominator:
        aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
        aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
        aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
        aA = (aA == 0.0f) ? 1.0f : aA;
        aB = (aB == 0.0f) ? 1.0f : aB;
        aC = (aC == 0.0f) ? 1.0f : aC;

        //Compute next iteration
        nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
        nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
        nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);

        //Check for stop condition
        dA = (nA - A);
        dB = (nB - B);
        dC = (nC - C);

        if ((dA * dA + dB * dB + dC * dC) <= sflsdelta) break;

        //Compute next iteration's values
        A = nA;
        B = nB;
        C = nC;
        A2 = A * A;
        B2 = B * B;
        C2 = C * C;
        QS = A2 + B2 + C2;
        QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
        Rsq = F0 + QB + QS;
        Q0 = 0.5f * (QS - Rsq);
        Q1 = F1 + Q0;
        Q2 = 8.0f * (QS - Rsq + QB + F0);
    }

    sphere[0] = A;
    sphere[1] = B;
    sphere[2] = C;
}
#endif

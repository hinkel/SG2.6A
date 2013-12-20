#include "board.h"
#include "mw.h"

extern uint8_t useServo;
extern rcReadRawDataPtr rcReadRawFunc;
extern uint16_t pwmReadRawRC(uint8_t chan);

static void _putc(void *p, char c)
{
    uartWrite(c);
}

int main(void)
{
    uint8_t i;
    drv_pwm_config_t pwm_params;
    drv_adc_config_t adc_params;

    for (i = 0; i < MAX_RC_CHANNELS; i++) rcData[i] = rcDataSAVE[i] = 1502;

#if 0
    // PC12, PA15 using this to write asm for bootloader :)
    RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO; // GPIOA/C+AFIO only
    AFIO->MAPR &= 0xF0FFFFFF;
    AFIO->MAPR = 0x02000000;
    GPIOA->CRH = 0x34444444;               // PIN 15 Output 50MHz
    GPIOA->BRR = 0x8000;                   // set low 15
    GPIOC->CRH = 0x44434444;               // PIN 12 Output 50MHz
    GPIOC->BRR = 0x1000;                   // set low 12
#endif

#if 0
    // using this to write asm for bootloader :)
    RCC->APB2ENR |= RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO; // GPIOB + AFIO
    AFIO->MAPR &= 0xF0FFFFFF;
    AFIO->MAPR = 0x02000000;
    GPIOB->BRR = 0x18;                     // set low 4 & 3
    GPIOB->CRL = 0x44433444;               // PIN 4 & 3 Output 50MHz
#endif

    systemInit();
    init_printf(NULL, _putc);

    checkFirstTime(false);
    readEEPROM();

    // configure power ADC
    if (cfg.power_adc_channel > 0 && (cfg.power_adc_channel == 1 || cfg.power_adc_channel == 9))
        adc_params.powerAdcChannel = cfg.power_adc_channel;
    else
    {
        adc_params.powerAdcChannel = 0;
        cfg.power_adc_channel = 0;
    }
    adcInit(&adc_params);
    serialInit(cfg.serial_baudrate);
    sensorsSet(SENSOR_ACC | SENSOR_BARO | SENSOR_MAG); // Will be cleared if not available
    sensorsClear(SENSOR_SONAR);            // Will be set on init

    NumberOfMotors = mixerInit();          // this will set useServo var depending on mixer type

    // when using airplane/wing mixer, servo/motor outputs are remapped
    if (cfg.mixerConfiguration == MULTITYPE_AIRPLANE || cfg.mixerConfiguration == MULTITYPE_FLYING_WING) pwm_params.airplane = true;
    else pwm_params.airplane = false;
    pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SPEKTRUM) || feature(FEATURE_GRAUPNERSUMH); // spektrum/Graupner Sumh support uses UART too
    pwm_params.usePPM  = feature(FEATURE_PPM);
    
    if (feature(FEATURE_SPEKTRUM) || feature(FEATURE_GRAUPNERSUMH))  // disable RC PWM inputs if using spektrum/Graupner Sumh
    {
        cfg.devorssi = 0;
        pwm_params.enablePWMInput = false;
    }
    else pwm_params.enablePWMInput = true;

    pwm_params.useServos    = useServo;
    pwm_params.extraServos  = (cfg.gbl_flg & GIMBAL_FORWARDAUX) || (feature(FEATURE_LED) && cfg.LED_Type == 1);
    pwm_params.motorPwmRate = cfg.esc_pwm;
    pwm_params.servoPwmRate = cfg.srv_pwm;

    pwm_params.useRC5   = false;
    pwm_params.useRC6   = false;
    pwm_params.useRC78  = false;
    pwm_params.usePWM56 = false;

#ifdef SONAR
    if (feature(FEATURE_SONAR))               // Set Sonar PWM Channels depending on Rc configuration. I2C Sonar needs no further attention here
    {
        switch(cfg.snr_type)                  // 0 = PWM56, 1 = RC78, 2 = I2C (DaddyWalross), 3 = MBPWM56, 4 = MBRC78
        {
        case 0:
        case 3:
            if (NumberOfMotors < 5 && !feature(FEATURE_SERVO_TILT)) pwm_params.usePWM56 = true; // Only do this with 4 Motors or less
            break;
        case 1:
        case 4:
            if (feature(FEATURE_PPM)) pwm_params.useRC78  = true; // ToDo: We should check for max motornumbers as well here
        default:
            break;
        }
    }
#endif

    if (feature(FEATURE_PPM) && feature(FEATURE_LED) && (cfg.LED_Type == 2))
    {
        if (cfg.LED_Pinout == 0) pwm_params.useRC5 = true;
        if (cfg.LED_Pinout == 1) pwm_params.useRC6 = true;
    }

    switch (cfg.power_adc_channel)
    {
    case 1:
        pwm_params.adcChannel = PWM2;
        break;
    case 9:
        pwm_params.adcChannel = PWM8;
        break;
    default:
        pwm_params.adcChannel = 0;
        break;
    }

    pwmInit(&pwm_params);

    rcReadRawFunc = pwmReadRawRC;          // configure PWM/CPPM read function. spektrum will override that

    if (feature(FEATURE_SPEKTRUM))
    {
        spektrumInit();
        rcReadRawFunc = spektrumReadRawRC;
    }
	  else if (feature(FEATURE_GRAUPNERSUMH))
         {
             graupnersumhInit();
             rcReadRawFunc = graupnersumhReadRawRC;
         }
         else if (feature(FEATURE_GPS) && !feature(FEATURE_PASS)) // spektrum and GPS are exclusive Optional GPS - available in both PPM and PWM input mode. In PWM input, reduces number of available channels by 2.
                  gpsInit(cfg.gps_baudrate);
    
    if (!feature(FEATURE_PASS))
    {
#ifdef SONAR
        if (feature(FEATURE_SONAR)) Sonar_init();        // Initialize Sonars here depending on Rc configuration.
#endif
        if (feature(FEATURE_PPM) && feature(FEATURE_LED) && (cfg.LED_Type == 2)) ledToggleInit();

        LD1_ON();                          // Crashpilot LED1_ON;
        LD0_OFF();                         // Crashpilot LED0_OFF;
        for (i = 0; i < 10; i++)
        {
            LED1_TOGGLE;
            LED0_TOGGLE;
            delay(25);
            BEEP_ON;
            delay(25);
            BEEP_OFF;
        }
        LD0_OFF();                         // Crashpilot LED0_OFF;
        LD1_OFF();                         // Crashpilot LED1_OFF;

        sensorsAutodetect();               // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
        imuInit();                         // Mag is initialized inside imuInit

        if (feature(FEATURE_VBAT)) batteryInit(); // Check battery type/voltage

// Initialize some other variables here before the "real thing"
        
        previousTime = micros();
        if (cfg.mixerConfiguration == MULTITYPE_GIMBAL) calibratingA = true; // Don't know if that is still a good idea
        calibratingG = true;
        f.SMALL_ANGLES_25 = 1;
        if(cfg.stat_clear) ClearStats();
        newpidimax = (float)cfg.newpidimax * 8192.0f;
        baseflight_mavlink_init();         // Always precalculate some Mavlink stuff, maybe needed
        SonarLandWanted = cfg.snr_land;    // Variable may be overwritten by failsave
        while (1) loop();                  // Do Harakiri        
    }
    else                                   // We want feature pass, do the minimal program
    {
        sensorsAutodetect();
        imuInit();
        writeAllMotors(cfg.esc_moff);      // All Motors off
        LD0_ON();
        LD1_OFF();
        while(1) pass();                   // Do feature pass
    }
}

void HardFault_Handler(void)
{
    writeAllMotors(cfg.esc_moff);	         // fall out of the sky
    while (1);
}

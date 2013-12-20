#include "board.h"
#include "mw.h"

// Oct 2013 Crashpilot 1000
// Based on the Timecop / Mwii Port
flags_t  f;
int16_t  debug[4];
uint8_t  toggleBeep    = 0;
uint32_t currentTime   = 0;
uint32_t currentTimeMS = 0;
uint32_t previousTime  = 0;
uint16_t cycleTime     = 0;                                          // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint8_t  vbat;                                                       // battery voltage in 0.1V steps
float    telemTemperature1;                                          // gyro sensor temperature
uint16_t failsafeCnt   = 0;
float    TiltValue;                                                  // 1.0 is horizontal 0 is vertical, minus is upsidedown
int16_t  rcData[MAX_RC_CHANNELS];                                    // RC RAWDATA int16_t rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // interval [1000;2000]
int16_t  rcDataSAVE [MAX_RC_CHANNELS];
int16_t  rcCommand[4];                                               // interval [esc min;esc max] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t  rssi;                                                       // 0 - 255 = 0%-100%
int16_t  lookupPitchRollRC[PITCH_LOOKUP_LENGTH];                     // lookup table for expo & RC rate PITCH+ROLL
int16_t  lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];                   // lookup table for expo & mid THROTTLE
rcReadRawDataPtr rcReadRawFunc = NULL;                               // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)
uint8_t  rcOptions[CHECKBOXITEMS];
float    axisPID[3];
float    newpidimax;
static   float dynP8[3], dynD8[3];

// **********************
// IMU
// **********************
float    headFreeModeHold;
float    heading;
float    magHold;
float    magneticDeclination;
float    GyroScale;

// **********************
// SONAR / BARO / Autoland / Autostart / Failsave
// **********************
uint8_t  SonarStatus = 0;                                            // 0 = no contact, 1 = made contact, 2 = steady contact
uint8_t  SonarBreach = 0;                                            // 0 = Breach unknown, 1 = breached lower limit, 2 = breached upper limit (not used)
uint8_t  SonarLandWanted   = 0;                                      // This is done to virtualize cfg.snr_land because failsafe can disable it and it could be saved in the worst case
float    ActualPressure;
float    GroundAlt;
#ifdef barotempdebug
float    BaroActualTemp;
#endif
int16_t  ESCnoFlyThrottle;
static   uint8_t GoodRCcnt;                                          // Number of good consecutive Signals before arming
static   int16_t FSBaroThrottle;
static   bool CopterFlying = false;

// **********************
// GPS
// **********************
int32_t  Real_GPS_coord[2];                                          // RAW GPS Coords
int32_t  GPS_home[2];
int32_t  GPS_WP[2];                                                  // Currently used WP
uint8_t  GPS_numSat;
uint16_t GPS_distanceToHome;                                         // distance to home point in meters
int16_t  GPS_directionToHome;                                        // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;                                    // altitude in m and speed in 0.1m/s
uint8_t  GPS_update = 0;                                             // it's a binary toogle to distinct a GPS position update
float    GPS_angle[2] = { 0, 0 };                                    // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;                                      // degrees * 10
uint8_t  GPS_Present = 0;                                            // Checksum from Gps serial
uint8_t  GPS_Enable = 0;
float    nav[2];
int16_t  maxbank10;                                                  // Maximum GPS Tiltangle in degree * 10
int16_t  maxbank100;                                                 // Maximum GPS Tiltangle in degree * 100
int8_t   nav_mode = NAV_MODE_NONE;                                   // Navigation mode
int8_t   wp_status = WP_STATUS_NONE;                                 // Waypoint status
int8_t   ph_status;
int32_t  WP_Target_Alt;
int16_t  WP_Desired_Climbrate;                                       // Climbrate in cm/s
bool     WP_Fastcorner;                                              // Dont decrease Speed at Target
float    sin_yaw_y;
float    cos_yaw_x;
static   uint8_t  PHminSat;

// *************************
// Automatic ACC Calibration
// *************************
uint16_t InflightcalibratingA = 0;
int16_t  AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;

// **********************
// Battery monitoring
// **********************
uint8_t  batteryCellCount = 3;                                       // cell count
uint16_t batteryWarningVoltage;                                      // annoying buzzer after this one, battery ready to be dead

// **********************
// Motor monitoring
// **********************
uint8_t  motorpercent[MAX_MONITORED_MOTORS]; // Is set in board.h to 8
uint16_t motorabspwm[MAX_MONITORED_MOTORS];
uint8_t  NumberOfMotors;                                             // Store number of Motors used, initialized in main in mixerinit

// **********************
// MWCRGB
// **********************
uint32_t LED_Value = 1500;
uint8_t  LED_Value_Delay = 8;

// **********************
// EEPROM
// **********************
uint32_t ScheduleEEPROMwriteMS = 0;

static int16_t RCDeadband(int16_t rcvalue, uint8_t rcdead);
static void GetAuxChannels(void);
static void DoThrcmmd_DynPid(void);
static void DoRcHeadfree(void);
static bool DeadPilot(void);
static int16_t DoMotorStats(bool JustDoRcThrStat);
static void DoLEDandBUZZER(void);
static void DisArmCopter(void);
static void BlinkGPSSats(void);
static void MWCRGBMONO_LED(uint8_t buzz);
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);
static void ChkFailSafe(void);
static void DoKillswitch(void);
static void GetClimbrateTorcDataTHROTTLE(int16_t cr);

void pass(void)                                                      // Crashpilot Feature pass
{
    static uint32_t blinktime, rctimer;
    static uint8_t  lastpassmotor;
    uint32_t timetmp;
    computeIMU();
    timetmp = micros();
    if (spektrumFrameComplete() || graupnersumhFrameComplete()) computeRC(); // Generates no rcData yet, but rcDataSAVE
    if ((int32_t)(timetmp - rctimer) >= 0)                           // 50Hz
    {
        rctimer = timetmp + 20000;
        if (!feature(FEATURE_SPEKTRUM) && !feature(FEATURE_GRAUPNERSUMH)) computeRC();
        GetActualRCdataOutRCDataSave();                              // Now we have new rcData to deal and MESS with
        if (failsafeCnt > 2)
        {
            rcData[THROTTLE] = cfg.esc_moff;                         // Motor off
            writeAllMotors(rcData[THROTTLE]);                        // Set all motors to zero just to be sure if user is messing in cli without saving
        }
        if (lastpassmotor != cfg.pass_mot) writeAllMotors(cfg.esc_moff); // Motonumber was changed in cli without saving
        lastpassmotor = cfg.pass_mot;
        if (!cfg.pass_mot) writeAllMotors(rcData[THROTTLE]);        // All Motors?
        else pwmWriteMotor(cfg.pass_mot - 1, rcData[THROTTLE]);     // Specific Motor?
        failsafeCnt++;
    }
    f.ARMED = 0;                                                     // Always set this as a dummy so serial com accepts "#" and "R"
    serialCom(false);
    if ((int32_t)(timetmp - blinktime) >= 0)
    {
        blinktime = timetmp + 150000;
        LED1_TOGGLE;
        LED0_TOGGLE;
    }
}

void loop(void)
{
    static uint8_t  rcDelayCommand;                                  // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint32_t RTLGeneralTimer, AltRCTimer0, LastLoopTime, rcTime = 0, BaroAutoTimer;
    static uint32_t LastSerialTimeMS;
    float           error, AngleRateTmp, RateError, delta, RCfactor;
    float           PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0;
    static float    errorAngle = 0, lastGyro[3] = { 0, 0, 0 }, lastDTerm[3] = { 0, 0, 0 };
    static float    errorGyroI[3] = { 0, 0, 0 }, errorAngleI[2] = { 0, 0 };
    float           prop;
    static uint8_t  ThrFstTimeCenter, AutolandState, AutostartState, HoverThrcnt, RTLstate;
    static int8_t   Althightchange;
    static uint16_t HoverThrottle;
    static int16_t  BaroLandThrlimiter, SnrLandThrlimiter, initialThrottleHold, LastAltThrottle;
    static int16_t  DistanceToHomeMetersOnRTLstart;
    static int16_t  AutostartTargetHight, AutostartFilterAlt, AutostartFilterVario, AutostartClimbrate;
    static stdev_t  variovariance;
    float           tmp0flt, dT, MwiiTimescale;
    int16_t         tmp0, thrdiff;
    uint8_t         axis, i;    
    
    // this will return false if spektrum is disabled. shrug.
    if (spektrumFrameComplete() || graupnersumhFrameComplete()) computeRC(); // Generates no rcData yet, but rcDataSAVE
    if ((currentTime - rcTime) >= 20000)                             // 50Hz
    {
        rcTime = currentTime;
        if (!feature(FEATURE_SPEKTRUM) && !feature(FEATURE_GRAUPNERSUMH)) computeRC();
        GetActualRCdataOutRCDataSave();                              // Now we have new rcData to deal and MESS with

        if ((rcData[THROTTLE] < cfg.rc_min) && !f.BARO_MODE)         // if ((rcData[THROTTLE] < cfg.rc_min) && !AutolandState)
        {
            for (i = 0; i < 2; i++) errorGyroI[i] = errorAngleI[i] = 0;
            errorGyroI[YAW] = 0;
            rcDelayCommand++;
            if (rcData[YAW] < cfg.rc_min && rcData[PITCH] < cfg.rc_min && !f.ARMED)
            {
                if (rcDelayCommand == 20)
                {
                    calibratingG = true;
                    if (feature(FEATURE_GPS)) GPS_reset_home_position();
                }
            }
            else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (!f.ARMED && rcData[YAW] < cfg.rc_min && rcData[PITCH] > cfg.rc_max && rcData[ROLL] > cfg.rc_max))
            {
                if (rcDelayCommand == 20)
                {
                    if (AccInflightCalibrationMeasurementDone)       // trigger saving into eeprom after landing
                    {
                        AccInflightCalibrationMeasurementDone = 0;
                        AccInflightCalibrationSavetoEEProm = 1;
                    }
                    else
                    {
                        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
                        if (AccInflightCalibrationArmed) toggleBeep = 2;
                        else toggleBeep = 3;
                    }
                }
            }
            else if (cfg.activate[BOXARM])
            {
                if (rcOptions[BOXARM] && f.OK_TO_ARM)
                {
                    if (feature(FEATURE_FAILSAFE))                   // Crashpilot1000
                    {
                        if (GoodRCcnt > (5 * cfg.fs_delay)) f.ARMED = 1;
                        else  f.ARMED = 0;
                    }
                    else f.ARMED = 1;
                    headFreeModeHold = heading;
                }
                else if (f.ARMED) f.ARMED = 0;
                rcDelayCommand = 0;
            }
            else if ((rcData[YAW] < cfg.rc_min || (cfg.rc_rllrm && rcData[ROLL] < cfg.rc_min)) && f.ARMED)
            {
                if (rcDelayCommand == 20)
                    f.ARMED = 0;                                     // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
            }
            else if ((rcData[YAW] > cfg.rc_max || (rcData[ROLL] > cfg.rc_max && cfg.rc_rllrm)) && rcData[PITCH] < cfg.rc_max && !f.ARMED && !calibratingG && f.ACC_CALIBRATED)
            {
                if (rcDelayCommand == 20)
                {
                    if (feature(FEATURE_FAILSAFE))                   // Crashpilot1000
                    {
                        if (GoodRCcnt > (5 * cfg.fs_delay)) f.ARMED = 1;
                        else  f.ARMED = 0;
                    }
                    else f.ARMED = 1;
                    headFreeModeHold = heading;
                }
            }
            else
                rcDelayCommand = 0;
        }
        else if (rcData[THROTTLE] > cfg.rc_max && !f.ARMED)
        {
            if (rcData[YAW] < cfg.rc_min && rcData[PITCH] < cfg.rc_min) // throttle=max, yaw=left, pitch=min
            {
                if (rcDelayCommand == 20)
                    calibratingA = true;
                rcDelayCommand++;
            }
            else if (rcData[YAW] > cfg.rc_max && rcData[PITCH] < cfg.rc_min) // throttle=max, yaw=right, pitch=min
            {
                if (rcDelayCommand == 20)
                    f.CALIBRATE_MAG = 1;                             // MAG calibration request
                rcDelayCommand++;
            }
            else if (rcData[PITCH] > cfg.rc_max)
            {
                cfg.angleTrim[PITCH] += 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else if (rcData[PITCH] < cfg.rc_min)
            {
                cfg.angleTrim[PITCH] -= 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else if (rcData[ROLL] > cfg.rc_max)
            {
                cfg.angleTrim[ROLL] += 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else if (rcData[ROLL] < cfg.rc_min)
            {
                cfg.angleTrim[ROLL] -= 2;
                writeParams(1);
#ifdef LEDRING
                if (feature(FEATURE_LED) && (cfg.LED_Type == 3)) ledringBlink();
#endif
            }
            else rcDelayCommand = 0;
        }

        if (feature(FEATURE_INFLIGHT_ACC_CAL))
        {
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > cfg.rc_min && !rcOptions[BOXARM])     // Copter is airborne and you are turning it off via boxarm : start measurement
            {
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = 0;
            }
            if (rcOptions[BOXPASSTHRU])                              // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
            {
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone) InflightcalibratingA = 50;
            }
            else if (AccInflightCalibrationMeasurementDone && !f.ARMED)
            {
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
            }
        }

        GetAuxChannels();
        DoThrcmmd_DynPid();                                          // Populates rcCommand[THROTTLE] from rcData[THROTTLE] and does empiric pid attenuation
        
        if (feature(FEATURE_LCD) && !f.ARMED)
        {
            if (rcData[THROTTLE] < cfg.rc_min && rcData[YAW] > cfg.rc_max && rcData[PITCH] > cfg.rc_max) serialOSD();
            else if (OLED_Type > 0) OLED_Status();
        }

        if (!rcOptions[BOXARM]) f.OK_TO_ARM = 1;               // Moved it here

        if (sensors(SENSOR_BARO) && !GroundAltInitialized) DisArmCopter(); // Keep Copter disarmed until baro init is done
          
/////// GPS INS TESTCODE
//				int16_t knob = constrain(rcData[AUX3]-1000,0,1000);
//				cfg.gps_ins_vel = (500 + (float)knob * 0.5f)/1000.0f;
//				debug[0] = cfg.gps_ins_vel*1000;
/////// GPS INS TESTCODE
/////// GPS INS TESTCODE
//				int16_t knob = constrain(rcData[AUX3]-1000,0,1000);
//				cfg.gps_ins_pos = (200 + (float)knob * 0.8f)/1000.0f;
//				debug[0] = cfg.gps_ins_pos*1000;
/////// GPS INS TESTCODE

        PHminSat = cfg.gps_ph_minsat;                                // Don't forget to set PH Minsats here!!

        if (feature(FEATURE_FAILSAFE))
        {
            ChkFailSafe();                                           // Only check Failsafe if copter is armed
        }
        else
        {
            failsafeCnt = 0;
            FSBaroThrottle = 0;                                      // Set to invalid
        }

//      SPECIAL RTL Crashpilot
//      Full RTL with Althold+Hightcheck+Autoland+Disarm
        if (rcOptions[BOXGPSHOME] || rcOptions[BOXGPSHOLD])          // Switch to Angle & MAG mode when GPS is on anyway
        {
            rcOptions[BOXHORIZON] = 0;
            rcOptions[BOXANGLE]   = 1;
            rcOptions[BOXMAG]     = 1;
        }

        if (AutolandState || AutostartState)                         // Switch to Angle mode when AutoBarofunctions anyway
        {
            rcOptions[BOXHORIZON] = 0;
            rcOptions[BOXANGLE]   = 1;
        }

#define RTLsettleTime 2000                                           // 2 sec
        if (sensors(SENSOR_GPS) && sensors(SENSOR_BARO) && f.GPS_FIX_HOME && rcOptions[BOXGPSHOME])
        {
            rcOptions[BOXBARO]     = 1;                              // Baro On
            rcOptions[BOXMAG]      = 1;                              // MAG ON Idea: Louis
            rcOptions[BOXGPSHOLD]  = 1;                              // GPS hold
            rcOptions[BOXPASSTHRU] = 0;                              // Passthru off
            rcOptions[BOXHEADFREE] = 0;                              // HeadFree off
            rcOptions[BOXGPSHOME]  = 0;                              // RTL OFF
            rcData[THROTTLE] = cfg.rc_mid;                           // Put throttlestick to middle: Althold
            PHminSat = 5;                                            // Sloppy PH is sufficient
            if (!RTLstate) RTLstate = 1;                             // Start RTL Sequence if it isn't already running
            if (GPS_numSat < 5) RTLstate = 0;                        // Error!
            if (cfg.rtl_mnd && RTLstate == 1 && GPS_distanceToHome < cfg.rtl_mnd)
                RTLstate = 0;                                        // Dont Do RTL if too close and RTL not already running
            
            switch (RTLstate)
            {
            case 0:                                                  // Error!! Do landing
                rcData[THROTTLE] = cfg.rc_min - 10;                  // Put throttlestick to lowest-10
                break;
            case 1:                                                  // prepare timer
                RTLGeneralTimer = currentTimeMS + RTLsettleTime;
                RTLstate++;
                break;
            case 2:                                                  // Hover certain time and wait for solid PH
                if (currentTimeMS > RTLGeneralTimer && ph_status == PH_STATUS_DONE) RTLstate++;
                break;
            case 3:                                                  // Check hight and climb if neccessary
                if (cfg.rtl_mnh)
                {
                    if (EstAlt < ((uint16_t)cfg.rtl_mnh * 100)) GetClimbrateTorcDataTHROTTLE((int16_t)cfg.rtl_cr);
                    else RTLstate++;
                }
                else RTLstate++;                                     // For safety, skip if turned off
                break;
            case 4:                                                  // Wait for Tailstuff before RTL
                if (cfg.nav_controls_heading)                        // Tail control
                {
                    if (cfg.nav_tail_first) magHold = wrap_180((float)(GPS_directionToHome - 180));
                    else magHold = GPS_directionToHome;
                    if(abs(wrap_180(heading - magHold)) < 6.0f) RTLstate++;// Turns true, when in range of +-6 degrees
                }
                else RTLstate++;
                break;
            case 5:                                                  // Prepare RTL
                DistanceToHomeMetersOnRTLstart = GPS_distanceToHome; // Set actual distance to Home in meters
                rcOptions[BOXGPSHOLD] = 0;                           // GPS hold OFF
                rcOptions[BOXGPSHOME] = 1;                           // Engage RTL
                RTLstate++;
                break;
            case 6:                                                  // OMG Do the f** RTL now
                rcOptions[BOXGPSHOLD] = 0;                           // GPS hold OFF
                rcOptions[BOXGPSHOME] = 1;                           // RTL
                tmp0 = (int16_t)GPS_distanceToHome - DistanceToHomeMetersOnRTLstart; // tmp0 contains flyawayvalue
                if ((cfg.gps_rtl_flyaway && tmp0 > (int16_t)cfg.gps_rtl_flyaway) ||
                   (wp_status == WP_STATUS_DONE && ph_status == PH_STATUS_DONE)) RTLstate++;
                break;
            case 7:                                                  // Do Autoland
                rcData[THROTTLE] = cfg.rc_min - 10;                  // Put throttlestick to lowest-10
                break;                                               // Repeat forever because Autoland will disarm the thing
            }
        }
        else RTLstate = 0;                                           // No BOXGPSHOME request? Reset Variable
//      SPECIAL RTL Crashpilot END

        if (rcOptions[BOXANGLE] && sensors(SENSOR_ACC))
        {
            if (!f.ANGLE_MODE)
            {
                errorAngleI[ROLL] = errorAngleI[PITCH] = 0;
                f.ANGLE_MODE = 1;
            }
        }
        else f.ANGLE_MODE = 0;

        if (rcOptions[BOXHORIZON] && sensors(SENSOR_ACC))
        {
            if (!f.HORIZON_MODE)
            {
                errorAngleI[ROLL] = errorAngleI[PITCH] = 0;
                f.HORIZON_MODE = 1;
            }
        }
        else f.HORIZON_MODE = 0;

        if ((GPS_numSat < 5 || !f.GPS_FIX) &&(f.ANGLE_MODE || f.HORIZON_MODE)) LD1_ON();
        else LD1_OFF();

        if (rcOptions[BOXGPSAUTO])
        {
        } else f.GPS_AUTO_MODE = 0;

#ifdef BARO
        if (sensors(SENSOR_BARO))
        {
            if (!f.ARMED) {f.BARO_MODE = 0; LastAltThrottle = 0;}    // Reset Baro stuff while not armed

            if (rcOptions[BOXBARO] && GroundAltInitialized)
            {
                if (!f.BARO_MODE)                                    // Initialize Baromode here if it isn't already
                {
                    AltRCTimer0 = 0; Althightchange = 0; AutolandState = 0; AutostartState = 0; ThrFstTimeCenter = 0;
                    AltHold = EstAlt;
                    if (FSBaroThrottle)
                    {                                                // Use Baro failsafethrottle here
                        LastAltThrottle     = FSBaroThrottle;
                        initialThrottleHold = FSBaroThrottle;
                    }
                    else
                    {
                        LastAltThrottle     = rcCommand[THROTTLE];
                        initialThrottleHold = rcCommand[THROTTLE];
                    }
                    f.BARO_MODE = 1;                                 // Finally set baromode to initialized
                }
                else
                {                                                    // Baromode initialized check for Autolanding/starting
                    if(CopterFlying)                                 // Are we somehow airborne?
                    {
                        if(AutostartState)
                        {
                            if ((abs(rcData[THROTTLE] - cfg.rc_mid) > cfg.rc_dbah))// Autostartus interruptus
                            {
                                AutostartState = 0; ThrFstTimeCenter = 0; Althightchange = 0; AltRCTimer0 = 0;
                                AltHold             = EstAlt;
                                initialThrottleHold = LastAltThrottle;
                            }
                        }
                        else
                        {
                            if (rcData[THROTTLE] < cfg.rc_min && !AutolandState) AutolandState = 1; // Start Autoland
                            if (rcData[THROTTLE] > cfg.rc_min && AutolandState)// Autolandus interruptus on Userinput reset some stuff
                            {
                                AutolandState = 0; ThrFstTimeCenter = 0; Althightchange = 0; AltRCTimer0 = 0;
                                AltHold             = EstAlt;
                                initialThrottleHold = LastAltThrottle;
                            }
                        }
                    }
                    else
                    {
                        if (cfg.as_trgt && !AutostartState && ThrFstTimeCenter) AutostartState = 1; // Start Autostart
                    }
                }
            } else {f.BARO_MODE   = 0; AutolandState = 0; AutostartState = 0;} // No Baroswitch, no Autoland/start
        } else {f.BARO_MODE   = 0; AutolandState = 0; AutostartState = 0;} // No Baro, no Autoland/start
#endif

#ifdef MAG
        if (sensors(SENSOR_MAG) &&  cfg.mag_calibrated)
        {
            if (rcOptions[BOXMAG])
            {
                if (!f.MAG_MODE)
                {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            }
            else f.MAG_MODE = 0;

            if (rcOptions[BOXHEADFREE])
            {
                if (!f.HEADFREE_MODE) f.HEADFREE_MODE = 1;
            }
            else f.HEADFREE_MODE = 0;

            if (rcOptions[BOXHEADADJ]) headFreeModeHold = heading;   // acquire new heading
        }
#endif

        if (sensors(SENSOR_GPS))
        {

            if (f.GPS_FIX && GPS_numSat >= 5)
            {

                if (rcOptions[BOXGPSHOME] && f.GPS_FIX_HOME)         // Crashpilot RTH is possible with 5 Sats for emergency and if homepos is set!
                {
                    if (!f.GPS_HOME_MODE)
                    {
                        f.GPS_HOME_MODE = 1;
                        nav_mode = NAV_MODE_RTL;                     // Set nav_mode before so GPS_set_next_wp can init it.
                        GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                    }
                }
                else f.GPS_HOME_MODE = 0;

                if (rcOptions[BOXGPSHOLD] && GPS_numSat >= PHminSat) // Crashpilot Only do poshold with specified Satnr or more
                {
                    if (!f.GPS_HOLD_MODE)
                    {
                        f.GPS_HOLD_MODE = 1;
                        nav_mode = NAV_MODE_POSHOLD;
                        GPS_set_next_wp(&Real_GPS_coord[LAT], &Real_GPS_coord[LON]);
                    }
                }
                else f.GPS_HOLD_MODE = 0;

            }
            else
            {
                f.GPS_HOME_MODE = f.GPS_HOLD_MODE = 0;
                nav_mode = NAV_MODE_NONE;
            }

        }                                                            // END of sensors SENSOR_GPS

        else

        {
            f.GPS_HOME_MODE = f.GPS_HOLD_MODE = 0;
            nav_mode = NAV_MODE_NONE;
        }

        if (rcOptions[BOXPASSTHRU]) f.PASSTHRU_MODE = 1;
        else f.PASSTHRU_MODE = 0;

        if (cfg.mixerConfiguration == MULTITYPE_FLYING_WING || cfg.mixerConfiguration == MULTITYPE_AIRPLANE) f.HEADFREE_MODE = 0;

        if (DoingGPS() && cfg.rc_dbgps)                              // Do some additional deadband for GPS, if needed
        {
            rcCommand[PITCH] = RCDeadband(rcCommand[PITCH], cfg.rc_dbgps);
            rcCommand[ROLL]  = RCDeadband(rcCommand[ROLL],  cfg.rc_dbgps);
        }

        if (f.HEADFREE_MODE) DoRcHeadfree();                         // Rotates Rc commands according mag and homeheading in headfreemode
        if (cfg.rc_killt)    DoKillswitch();                         // AT THE VERY END DO SOME KILLSWITCHSTUFF, IF WANTED

// *********** END OF 50Hz RC LOOP ***********
    }
    else
    {
        DoLEDandBUZZER();                                            // Do that, if not doing RC stuff
    }

#ifdef SONAR
    if (sensors(SENSOR_SONAR) && GroundAltInitialized) Sonar_update();// Update "SonarStatus". Do it here because HC-SR04 will produce delay
#endif

#ifdef MAG
    if (sensors(SENSOR_MAG)) Mag_getADC();
#endif

    currentTimeMS = millis();                                        // Feed values if needed in outer timeloop
    currentTime   = micros();
    if (currentTime - LastLoopTime < cfg.looptime)                   // Waste away some time?
    {
        serialCom(false);                                            // Yes! Do serial then, no single step
        LastSerialTimeMS = currentTimeMS;
        return;                                                      // Return to main "while" endless loop
    }
    LastLoopTime = currentTime;

    computeIMU();                                                    // looptime Timeloop starts here on predefined basis
    currentTimeMS = millis();
    currentTime   = micros();		                                     // Crashpilot moved it here
    cycleTime     = currentTime - previousTime;
    previousTime  = currentTime;

#ifdef BARO
    if (sensors(SENSOR_BARO))                                        // The normal stuff to keep it simple
    {
        Baro_update();
        getEstimatedAltitude();                                      // Combine with sonar if possible
    }
  
#define HoverTimeBeforeLand     2000                                 // Wait 2 sec in the air for VirtualThrottle to catch up
    if (sensors(SENSOR_BARO) && f.BARO_MODE && f.ARMED)              // GroundAltInitialized must not be checked but armed, in case of dumb user -> see above
    {
        getAltitudePID();                                            // !Do not forget to calculate the Baropids...

        if(AutostartState)
        {
            AutostartFilterAlt   = ((AutostartFilterAlt   << 1) + AutostartFilterAlt   + (int16_t)EstAlt) >> 2;
            AutostartFilterVario = ((AutostartFilterVario << 1) + AutostartFilterVario + (int16_t)vario)  >> 2;
            devPush(&variovariance, (float)AutostartFilterVario);
        }
        switch(AutostartState)
        {
        case 0:
            break;
        case 1:                                                      // Initialize Autostart with relative targethight
            AutostartTargetHight = (int16_t)EstAlt + (int16_t)cfg.as_trgt * 100;
            AutostartFilterAlt   = (int16_t)EstAlt;
            AutostartFilterVario = (int16_t)vario;
            AutostartClimbrate   = (int16_t)cfg.as_lnchr << 1;
            initialThrottleHold  = ESCnoFlyThrottle;                 // Set higher baselinethrottle than esc min.
            devClear(&variovariance);
            devPush(&variovariance, vario);
            GetClimbrateTorcDataTHROTTLE(AutostartClimbrate);
            AutostartState++;
            break;
        case 2:                                                      // Wait for liftoff. It is assumed when the std dev of the vario exceeds a limit or if the climbrate is beyond 50cm/s
            if (AutostartFilterVario < 50 || ((uint8_t)devStandardDeviation(&variovariance) < cfg.as_stdev))
            {
                GetClimbrateTorcDataTHROTTLE(AutostartClimbrate);    // No, liftoff increase virtual targethight (like a rubberband to pull the copter off the ground)
            }
            else                                                     // Liftoff! Force alhold to reset virtual targethight
            {
                rcData[THROTTLE]   = cfg.rc_mid;                     // We have a liftoff, force althold (reset internal altholdtarget)
                AltRCTimer0        = 0;                              // vario = 0;
                AutostartClimbrate = (int16_t)cfg.as_clmbr;
                AutostartState++;
            }
            break;
        case 3:                                                      // Now climb with desired rate to targethight.
            tmp0 = (int16_t)((float)AutostartFilterAlt + (float)AutostartFilterVario * cfg.bar_lag);// Actual predicted hight
            AutostartClimbrate = constrain((abs(AutostartTargetHight - tmp0) / 3), 10, (int16_t)cfg.as_clmbr);// Slow down when getting closer to target
            if (AutostartFilterAlt < AutostartTargetHight)
            {
                GetClimbrateTorcDataTHROTTLE(AutostartClimbrate);    // Climb
            }
            else
            {
                rcData[THROTTLE] = cfg.rc_mid;                       // Hover
                AltRCTimer0 = 0; AutostartState = 0;                 // vario = 0;
            }
            break;
        }

        switch (AutolandState)
        {
        case 0:                                                      // No Autoland Do nothing
            SnrLandThrlimiter    = BaroLandThrlimiter = 0;           // Reset it here! BaroAutoTimer = 0;
            break;
        
        case 1:                                                      // Start Althold
            rcData[THROTTLE]     = cfg.rc_mid;                       // Put throttlestick to middle
            BaroAutoTimer        = currentTimeMS + HoverTimeBeforeLand;// prepare timer
            HoverThrcnt          = 1;                                // Initialize Hoverthrottlestuff here
            HoverThrottle        = DoMotorStats(true);               // Try to get average here from flight. Returns 0 if not possible.
            if (HoverThrottle < LastAltThrottle) HoverThrottle = LastAltThrottle; // Take the bigger one as base
            AutolandState++;
            break;
        
        case 2:                                                      // We hover here and gather the Hoverthrottle
            rcData[THROTTLE]     = cfg.rc_mid;                       // Put throttlestick to middle: Hover some time to gather Hoverthr
            HoverThrottle       += LastAltThrottle;
            HoverThrcnt ++;
            if (HoverThrcnt == 20)
            {
              HoverThrottle      = HoverThrottle / 20;               // Average of 20 Values
              if (currentTimeMS > BaroAutoTimer) AutolandState++;
              else HoverThrcnt   = 1;
            }
            break;
            
        case 3:                                                      // Start descent initialize Variables
            if (cfg.al_debounce)                                     // Set BaroLandThrlimiter now, if wanted
            {
                tmp0 = (int16_t)HoverThrottle - cfg.esc_min;         // tmp0 contains absolute absolute hoverthrottle
                if (tmp0 > 0) BaroLandThrlimiter = HoverThrottle + ((float)tmp0 * (float)cfg.al_debounce * 0.01f);// Check here to be on the safer side. Don't set BaroLandThrlimiter if something is wrong
            }
            GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_barolr);
            if (sensors(SENSOR_SONAR) && SonarStatus == 2) GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_snrlr);// Set al_snrlr on steady sonar contact
            BaroAutoTimer = 0;
            AutolandState++;
            break;

        case 4:                                                      // Keep descending and check for landing
            GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_barolr);
            if (sensors(SENSOR_SONAR))                               // Adjust Landing
            {
                if (SonarStatus == 2) GetClimbrateTorcDataTHROTTLE(-(int16_t)cfg.al_snrlr);// SolidSonarContact use maybe different Landrate
                if (SonarLandWanted && SonarBreach == 1 && !SnrLandThrlimiter)             // Sonarlanding if SonarBreach = 1 (Proximity breach) fix the upper throttlevalue
                    SnrLandThrlimiter = cfg.esc_max;                                       // Set maximal thr as upper limit, will be adjusted below
            }
            if (LastAltThrottle <= ESCnoFlyThrottle && !BaroAutoTimer)// ESCnoFlyThrottle is set upon Baro initialization in sensors/sensorsAutodetect
            {
                BaroAutoTimer = currentTimeMS;
                if (SnrLandThrlimiter) BaroAutoTimer += (uint32_t)cfg.al_tosnr;// Aided Sonar landing is wanted and limiter set, do perhaps shorter timeout then
                else BaroAutoTimer += (uint32_t)cfg.al_tobaro;                 // No Sonar limiter, do the normal timeout
            }
            if (LastAltThrottle > ESCnoFlyThrottle) BaroAutoTimer = 0;         // Reset Timer
            if ((BaroAutoTimer && currentTimeMS > BaroAutoTimer) || (TiltValue < 0)) AutolandState++; // Proceed to disarm if timeup or copter upside down
            break;
            
        case 5:                                                      // Shut down Copter forever....
            DisArmCopter();
            break;
        }
        
        thrdiff = rcData[THROTTLE] - cfg.rc_mid;
        tmp0    = abs(thrdiff);
        
        if (tmp0 < cfg.rc_dbah && !ThrFstTimeCenter) ThrFstTimeCenter = 1;
        if (currentTimeMS >= AltRCTimer0)                                                                         // X Hz Loop
        {
            AltRCTimer0 = currentTimeMS + 100;
            if (ThrFstTimeCenter && tmp0 > cfg.rc_dbah)
            {
                initialThrottleHold = initialThrottleHold + (BaroP / 100);							                          // Adjust Baselinethr by 1% of BaroP
                if (LastAltThrottle < cfg.esc_max && thrdiff > 0)
                {
                    Althightchange = 1;
                    AltHold += (float)(thrdiff - cfg.rc_dbah) * 0.125f;
                }
                if (LastAltThrottle > cfg.esc_min && thrdiff < 0)
                {
                    Althightchange = -1;
                    if (!AutolandState) AltHold += (float)(thrdiff + cfg.rc_dbah) * 0.125f * cfg.bar_dscl;        // Descent with less rate in manual mode
                    else AltHold += (float)(thrdiff + cfg.rc_dbah) * 0.125f;
                }
            }
            else                                                                                                  // Stick is to center here
            {
                if (ThrFstTimeCenter && Althightchange)                                                           // Are we coming from a hight change?
								{
								    AltHold = EstAlt + vario * cfg.bar_lag;                                                       // We are coming from a hightchange project stoppingpoint
                    initialThrottleHold = LastAltThrottle;                                                        // This is for starting in althold otherwise the initialthr would be idle throttle
								}
                Althightchange = 0;
            }
        }                                                                                                         // End of X Hz Loop

        if (AutolandState || AutostartState) BaroD = 0;                                                           // Don't do Throttle angle correction when autolanding/starting
        if (AutostartState == 2) BaroI = BaroI >> 1;                                                              // Reduce Variobrake on Autostart during liftoffphase
        rcCommand[THROTTLE] = constrain(initialThrottleHold + BaroP + BaroD - BaroI, cfg.esc_min, cfg.esc_max);

        if (AutolandState)                                                                                        // We are Autolanding and 
        {
            if (SnrLandThrlimiter)                                                                                // Check sonarlimiter first
            {                                                                                                     // Sonar has given proximity alert and sonar land support is wanted
                if (LastAltThrottle < SnrLandThrlimiter) SnrLandThrlimiter = LastAltThrottle;                     // Adjust limiter here
                rcCommand[THROTTLE] = min(rcCommand[THROTTLE], SnrLandThrlimiter);
            }
            else if (BaroLandThrlimiter) rcCommand[THROTTLE] = min(rcCommand[THROTTLE], BaroLandThrlimiter);      // Only do Barolimiter on landing, if we have no Sonarlimiter
        }
        
        
        LastAltThrottle = rcCommand[THROTTLE];
    }

// Baro STATS LOGGING
        if (sensors(SENSOR_BARO) && f.ARMED)
        {
            tmp0 = (int16_t)((int32_t)EstAlt / 100);
            if (tmp0 > cfg.MaxAltMeter) cfg.MaxAltMeter = tmp0;
            if (tmp0 < cfg.MinAltMeter) cfg.MinAltMeter = tmp0;
        }
// Baro STATS LOGGING

#endif

#ifdef MAG
    if (sensors(SENSOR_MAG))
    {
        float magP;
        if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE)
        {
            if (DoingGPS()) magP = (float)cfg.gps_yaw;               // Do slower mag P in gps mode
            else magP = (float)cfg.P8[PIDMAG];
            if (f.SMALL_ANGLES_25) rcCommand[YAW] -= wrap_180(heading - magHold) * magP / 30;// 18 deg
        }
        else magHold = heading;
    }
#endif

    if (sensors(SENSOR_GPS) && sensors(SENSOR_MAG))                  // Only do GPS stuff if the Mag is available
    {
        GPS_alltime();                                               // Do INS GPS stuff here
        
        if (!DoingGPS() || !f.GPS_FIX_HOME || !cfg.mag_calibrated)   // Only do further GPS stuff, if needed and possible
        {
            GPS_reset_nav();
            nav_mode  = NAV_MODE_NONE;
            ph_status = PH_STATUS_NONE;
            wp_status = WP_STATUS_NONE;
        }
        else                                                         // proceed here if mag calibrated and the rest is fine as well
        {
            GPS_angle[ROLL]  = constrain((nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) * 0.1f,  -maxbank10, maxbank10);
            GPS_angle[PITCH] = constrain((nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) * 0.1f,  -maxbank10, maxbank10);
        }
    } else GPS_angle[0] = GPS_angle[1] = 0;

    tmp0flt       = (float)cycleTime;
    MwiiTimescale = tmp0flt * 3.3333333e-4f;
    dT            = tmp0flt * 0.000001f;
    RCfactor      = dT / (MainDptCut + dT);                                     // used for pt2 element
    prop = (float)min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),500);

    for (axis = 0; axis < 3; axis++)
    {
        switch (cfg.mainpidctrl)
        {
        case 0:                                                                 // 0 = OriginalMwiiPid pimped by me
        {
            if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < YAW)                 // MODE relying on ACC 50 degrees max inclination
            {
                errorAngle  = constrain(2.0f * (float)rcCommand[axis] + GPS_angle[axis], -500.0f, +500.0f) - angle[axis] + (float)cfg.angleTrim[axis]; //  Removed INFO BRM errorAngle = errorAngle * (float)cycleTime / BasePIDtime; // Crashpilot: Include Cylcletime take 3ms as basis. More deltaT more error
                PTermACC    = errorAngle * (float)cfg.P8[PIDLEVEL] * 0.01f;
                tmp0flt     = (float)cfg.D8[PIDLEVEL] * 5.0f;
                PTermACC    = constrain(PTermACC, -tmp0flt, +tmp0flt);
                errorAngle *= MwiiTimescale;
                errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000.0f, +10000.0f);
                ITermACC    = (errorAngleI[axis] * (float)cfg.I8[PIDLEVEL]) / 4096.0f;
            }

            if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == YAW)                 // MODE relying on GYRO or YAW axis
            {
                error  = (float)rcCommand[axis] * 80.0f / (float)cfg.P8[axis];  // Removed INFO BRM error  = error * (float)cycleTime / BasePIDtime;     // Crashpilot: Include Cylcletime take 3ms as basis. More deltaT more error
                error -= gyroData[axis];
                PTermGYRO = (float)rcCommand[axis];
                error *= MwiiTimescale;
                errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000.0f, +16000.0f);
                if (abs(gyroData[axis]) > 640.0f) errorGyroI[axis] = 0;
                ITermGYRO = errorGyroI[axis] * (float)cfg.I8[axis] * 0.000125f;
            }

            if (f.HORIZON_MODE && axis < YAW)
            {
                PTerm = (PTermACC * (500.0f - prop) + PTermGYRO * prop) * 0.002f;
                ITerm = (ITermACC * (500.0f - prop) + ITermGYRO * prop) * 0.002f;
            }
            else
            {
                if (f.ANGLE_MODE && axis < YAW)
                {
                    PTerm = PTermACC;
                    ITerm = ITermACC;
                }
                else
                {
                    PTerm = PTermGYRO;
                    ITerm = ITermGYRO;
                }
            }
            PTerm          -= (gyroData[axis] * dynP8[axis] * 0.0125f);
            delta           = (gyroData[axis] - lastGyro[axis]) / MwiiTimescale;
            lastGyro[axis]  = gyroData[axis];
            lastDTerm[axis] = lastDTerm[axis] + RCfactor * (delta - lastDTerm[axis]);
            axisPID[axis]   = PTerm + ITerm - (lastDTerm[axis] * dynD8[axis] * 0.09375f);
            break;
        }

// Alternative Controller by alex.khoroshko http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=30#p37465
        case 1:                                                                 // 1 = New mwii controller (float pimped + pt1element)
        {
            if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < YAW)                 // MODE relying on ACC
            {
                errorAngle = constrain(2.0f * (float)rcCommand[axis] + GPS_angle[axis], -500.0f, +500.0f) - angle[axis] + (float)cfg.angleTrim[axis];
            }
            if (axis == YAW)
            {
                AngleRateTmp = ((float)(cfg.yawRate + 27) * (float)rcCommand[axis]) * 0.03125f; // AngleRateTmp = (((int32_t)(cfg.yawRate + 27) * rcCommand[2]) >> 5);
            }
            else
            {
                if (!f.ANGLE_MODE)                                              //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                {
                    AngleRateTmp = ((float)(cfg.rollPitchRate + 27) * (float)rcCommand[axis]) * 0.0625f; // AngleRateTmp = ((int32_t) (cfg.rollPitchRate + 27) * rcCommand[axis]) >> 4;
                    if (f.HORIZON_MODE)
                    {
                        AngleRateTmp += ((float)cfg.I8[PIDLEVEL] * errorAngle) * 0.0390625f; // increased by x10 //0.00390625f AngleRateTmp += (errorAngle * (float)cfg.I8[PIDLEVEL]) >> 8;
                    }
                }
                else
                {
                    AngleRateTmp = ((float)cfg.P8[PIDLEVEL] * errorAngle) * 0.0223214286f; // AngleRateTmp = (errorAngle * (float)cfg.P8[PIDLEVEL]) >> 4; * LevelPprescale;
                }
            }
            RateError         = AngleRateTmp - gyroData[axis];
            PTerm             = ((float)cfg.P8[axis] * RateError) * 0.0078125f;
            errorGyroI[axis] += ((float)cfg.I8[axis] * RateError  * (float)cycleTime) / 2048.0f;
            errorGyroI[axis]  = constrain(errorGyroI[axis], -newpidimax, newpidimax);
            ITerm             = errorGyroI[axis] / 8192.0f;
            delta             = (RateError - lastGyro[axis]) * (16383.75f / (float)cycleTime); // lastGyro should be called "lastError"
            lastGyro[axis]    = RateError;
            lastDTerm[axis]   = lastDTerm[axis] + RCfactor * (delta - lastDTerm[axis]);
            axisPID[axis]     = PTerm + ITerm + (((float)cfg.D8[axis] * lastDTerm[axis]) * 0.01171875f);
            break;
        }
        }                                                                           // End of Switch
    }

    if (f.ARMED)
    {
        if (rcCommand[THROTTLE] > ESCnoFlyThrottle) CopterFlying = true;
        if (cfg.rc_flpsp && sensors(SENSOR_ACC) && TiltValue < 0 && !f.ANGLE_MODE) rcCommand[THROTTLE] = cfg.esc_min; // Do idlethrottle when upside down
    }
    else
    {
        CopterFlying = false;
// SCHEDULE EEPROM WRITES HERE
// 1. Because all changes within that timeout are collected and then written at once without bothering the eeprom too much
// 2. While flying you can save so many Data as you want in cfg.x and when you land and disarm they are actually written
        if (ScheduleEEPROMwriteMS && currentTimeMS >= ScheduleEEPROMwriteMS)
        {
            writeParams(0);                                          // Write, don't blink
            ScheduleEEPROMwriteMS = 0;                               // Reset Timer do this only once
        }
    }

    mixTable();
    writeServos();
    writeMotors();
    DoMotorStats(false);                                             // False means no hoverthrottlegeneration for failsafe

    if (currentTimeMS - LastSerialTimeMS >= 10)                      // If Serial wasn't possible during timewaste (user set looptime too small), do it now and ensure 100Hz
    {
        serialCom(true);                                             // Do singlestep
        LastSerialTimeMS = currentTimeMS;
    }
}

// END OF MAINLOOP
static int16_t DoMotorStats(bool JustDoRcThrStat)
{
    static uint32_t motortotal[MAX_MONITORED_MOTORS], DatasetCnt, LastMotorStatTimeMS;
    static uint8_t  maxmotnr, motcasestate = 0;
    uint32_t Onepercent, Sum;
    uint8_t i;

    if (JustDoRcThrStat)
    {
        if (motcasestate != 1 || !maxmotnr) return 0;                // Error! No data / no motor
        Sum = 0;
        for (i = 0; i < maxmotnr; i++) Sum += ((motortotal[i] / DatasetCnt) + cfg.esc_min);
        return Sum / maxmotnr;
    }

    if (currentTimeMS - LastMotorStatTimeMS >= 100)                  // Do Motorstats at 10 Hz rate
    {
        LastMotorStatTimeMS = currentTimeMS;
        switch(motcasestate)
        {
        case 0:                                                      // Bootuprun / Reset state
            for (i = 0; i < MAX_MONITORED_MOTORS; i++) motortotal[i] = 0; // Clear sum
            DatasetCnt = 0;
            maxmotnr = min(NumberOfMotors, MAX_MONITORED_MOTORS);        
            if (f.ARMED) motcasestate++;                             // Wait for Arming
            break;
        case 1:                                                      // Collect data here
            if (f.ARMED)
            {
                for (i = 0; i < maxmotnr; i++) motortotal[i] += motor[i] - cfg.esc_min;
                DatasetCnt++; 
            } else motcasestate++;                                   // Disarmed! Go on to calculate
            break;
        case 2:                                                      // Calculate percentage and reset statemachine
            Onepercent = 0;
            for (i = 0; i < maxmotnr; i++)
            {
                Onepercent += motortotal[i] / 100;                   // Overflow very unlikely but /100 saves us anyway. Precision is not relevant.
                motorabspwm[i] = (motortotal[i] / DatasetCnt) + cfg.esc_min;
            }
            for (i = 0; i < maxmotnr; i++) motorpercent[i] = motortotal[i] / Onepercent;
            motcasestate = 0;
            break;
        }
    }
    return 0;                                                        // Not necessary
}

// SOME RC FUNCTIONS START
// Can not invoke Autolanding!
// cr is in cm/s Climbrate of "0" does althold        
static void GetClimbrateTorcDataTHROTTLE(int16_t cr)
{
    float tmp;
    if (!cr)
    {
        rcData[THROTTLE] = cfg.rc_mid;
        return;
    }
    tmp = (float)cr * 0.8f;
    if (cr > 0) rcData[THROTTLE] = tmp + cfg.rc_mid + cfg.rc_dbah;
    else rcData[THROTTLE] = tmp + cfg.rc_mid - cfg.rc_dbah;
    rcData[THROTTLE] = constrain(rcData[THROTTLE], cfg.rc_min + 1, cfg.rc_max);
}

static int16_t RCDeadband(int16_t rcvalue, uint8_t rcdead)           // Actually needed for additional GPS deadband
{
    if (abs(rcvalue) < rcdead) rcvalue = 0;
    else if (rcvalue > 0) rcvalue = rcvalue - (int16_t)rcdead;
    else rcvalue = rcvalue + (int16_t)rcdead;
    return rcvalue;
}

uint16_t pwmReadRawRC(uint8_t chan)
{
    uint16_t data;
    if (chan > 7) data = pwmRead(chan);
    else data = pwmRead(cfg.rcmap[chan]);
    if (data < 750 || data > 2250) data = cfg.rc_mid;
    return data;
}

void computeRC(void)                                                 // Just harvest RC Data
{
    static int16_t rcData4Values[MAX_RC_CHANNELS][4], rcDataMean[MAX_RC_CHANNELS];
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;

    rc4ValuesIndex++;
    for (chan = 0; chan < cfg.rc_auxch + 4; chan++)
    {
        rcData4Values[chan][rc4ValuesIndex % 4] = rcReadRawFunc(chan);
        rcDataMean[chan] = 0;
        for (a = 0; a < 4; a++) rcDataMean[chan] += rcData4Values[chan][a];
        rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
        if (rcDataMean[chan] < rcDataSAVE[chan] - 3) rcDataSAVE[chan] = rcDataMean[chan] + 2;
        if (rcDataMean[chan] > rcDataSAVE[chan] + 3) rcDataSAVE[chan] = rcDataMean[chan] - 2;
    }
}

void GetActualRCdataOutRCDataSave(void)                              // Make RC Data available
{
    uint8_t i;
    for (i = 0; i < MAX_RC_CHANNELS; i++) rcData[i] = rcDataSAVE[i];
    if (cfg.devorssi) rssi = ((constrain(rcData[cfg.rc_auxch + 3], 900, 1923) - 900) >> 2); // [0-255] Based on the Specs send in by BAMFAX
}
     
static void GetAuxChannels(void)
{
    uint32_t    auxState = 0, auxStateTMP;
    uint8_t i;

    for (i = 0; i < cfg.rc_auxch; i++)                           // for (i = 0; i < 4; i++)
    {
        auxStateTMP = rcData[AUX1 + i];
        auxState   |= (auxStateTMP < 1300) << (3 * i) |
                      (1300 < auxStateTMP && auxStateTMP < 1700) << (3 * i + 1) |
                      (auxStateTMP > 1700) << (3 * i + 2);
    }
    for (i = 0; i < CHECKBOXITEMS; i++) rcOptions[i] = (auxState & cfg.activate[i]) > 0;
}

static void DoThrcmmd_DynPid(void)
{
#define BREAKPOINT 1500
    uint8_t  axis;
    uint32_t prop1, prop2 = 100;
    uint32_t tmp, tmp2; 

    if (rcData[THROTTLE] >= BREAKPOINT)                              // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    {
        if (rcData[THROTTLE] < 2000)
        {
            prop2 -= (uint16_t) cfg.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
        }
        else
        {
            prop2 -= cfg.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++)
    {
        tmp   = min(abs(rcData[axis] - cfg.rc_mid), 500);
        prop1 = 100;
        if (axis != 2)                                               // ROLL & PITCH
        {
            if (cfg.rc_db)
            {
                if (tmp > cfg.rc_db) tmp -= cfg.rc_db;
                else tmp = 0;
            }
            tmp2 = min(tmp / 100, PITCH_LOOKUP_LENGTH - 1);          // Prevent out of bounds
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 -= (uint16_t) cfg.rollPitchRate * tmp / 500;
            prop1 = (uint16_t) prop1 * prop2 / 100;
        }
        else                                                         // YAW
        {
            if (cfg.rc_dbyw)
            {
                if (tmp > cfg.rc_dbyw) tmp -= cfg.rc_dbyw;
                else tmp = 0;
            }
            rcCommand[axis] = tmp;
            prop1 -= (uint16_t)cfg.yawRate * tmp / 500;
        }
        dynP8[axis] = (float)cfg.P8[axis] * prop1 / 100;          // dynI8[axis] = (uint16_t) cfg.I8[axis] * prop1 / 100;
        dynD8[axis] = (float)cfg.D8[axis] * prop1 / 100;
        if (rcData[axis] < cfg.rc_mid) rcCommand[axis] = -rcCommand[axis];
    }
    tmp = constrain(rcData[THROTTLE], cfg.rc_min, 2000);
    tmp = (uint32_t) (tmp - cfg.rc_min) * 1000 / (2000 - cfg.rc_min);// [MINCHCK;2000] -> [0;1000]
    tmp2 = min(tmp / 100, THROTTLE_LOOKUP_LENGTH - 1);               // Prevent out of bounds
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [esc_min;esc_max]
}

static void DoRcHeadfree(void)
{
    int16_t rcCommand_PITCH;
    float   cosDiff, sinDiff, radDiff;
    radDiff = wrap_180(heading - headFreeModeHold) * RADX;        // Degree to RAD
    cosDiff = cosf(radDiff);
    sinDiff = sinf(radDiff);
    rcCommand_PITCH  = (float)rcCommand[PITCH] * cosDiff + (float)rcCommand[ROLL]  * sinDiff;
    rcCommand[ROLL]  = (float)rcCommand[ROLL]  * cosDiff - (float)rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
}

static bool DeadPilot(void)
{
    static int16_t  lastcheksum = 0;
    static uint32_t deadtimer = 0;
    bool            output;
    int16_t         cheksum;
    uint8_t         i;

    if (!f.ARMED || !cfg.fs_ddplt)
    {
        deadtimer = 0;
        return false;
    }
    cheksum = 0;
    for (i = 0; i < 4; i++) cheksum = cheksum + rcData[i];
    if (cheksum != lastcheksum)
    {
        deadtimer = 0;
        output = false;
    }
    else if(deadtimer == 0) deadtimer = currentTimeMS + (uint32_t)cfg.fs_ddplt * 1000;
    lastcheksum = cheksum;
    if (currentTimeMS > deadtimer && deadtimer != 0) output = true;
    return output;
}

// If Copter is armed by box and a cfg.rc_killt (in ms) is defined
// Don't enable Killswitch when Notarmed or Rc Signal disturbed, dont do killswitch if FEATURE_INFLIGHT_ACC_CAL is wanted
static void DoKillswitch(void)
{
    static uint32_t Killtimer = 0;
    if (cfg.activate[BOXARM] && !rcOptions[BOXARM] && !Killtimer) Killtimer = currentTimeMS + (uint32_t)cfg.rc_killt;
    if (!f.ARMED || rcOptions[BOXARM] || failsafeCnt > 1 || feature(FEATURE_INFLIGHT_ACC_CAL)) Killtimer = 0;
    if (Killtimer && currentTimeMS >= Killtimer) DisArmCopter(); // Kill Copter
}  

// Failsafe routine Modified & Moved here by Crashpilot
// 4 Cases:
// 1: No Sensors -> Do the old stuff
// 2: GPS -> Do RTL and do the old stuff
// 3: Baro -> Do Baro Autoland
// 4: BARO & GPS -> Do full feature RTL
// Logic Do FS if FScount is too high, or pilotdead (no stickinput for x seconds)
// If FS is already running (fstimer !=0) but the goodcount is too low (maybe single good spike) keep FS running
static void ChkFailSafe(void)
{
    static uint32_t Failsafetimer;
    uint8_t i;
  
    if (CopterFlying && ((failsafeCnt > (5 * cfg.fs_delay)) || DeadPilot() || (Failsafetimer && GoodRCcnt < (5 * cfg.fs_delay))))
    {
        if (!Failsafetimer) Failsafetimer = currentTimeMS + 100 * (uint32_t)(cfg.fs_delay + cfg.fs_ofdel);
        for (i = 0; i < 3; i++)                              // Center Roll Pitch Yaw here
        {
            rcData[i]    = cfg.rc_mid;                       // rcData contains raw rc values
            rcCommand[i] = 0;                                // rcCommand contains RPY -500 0 + 500 and 1000-2000 for throttle
        }
        rcOptions[BOXPASSTHRU] = 0;                          // Passthru OFF
        rcOptions[BOXHEADFREE] = 0;                          // HeadFree OFF
        rcOptions[BOXHORIZON]  = 0;                          // Horizon OFF
        rcOptions[BOXANGLE]    = 1;                          // Angle ON
        rcOptions[BOXMAG]      = 1;                          // MAG ON
        rcOptions[BOXBARO]     = 1;                          // Baro ON
        PHminSat               = 5;                          // Sloppy PH is sufficient

        if (cfg.fs_nosnr) SonarLandWanted = 0;               // This can disable the aided sonarlanding, that could cause problems on trees
        if (TiltValue < 0) DisArmCopter();                   // Copter is upside down in anglemode -> disarm the poor bustard

        i = 0;                                               // i = (sensors(SENSOR_GPS) << 1) | sensors(SENSOR_BARO);// Bitshifting of a boolean returnvalue is not good?
        if (sensors(SENSOR_GPS))
        {
            i = 2;
            if (f.GPS_FIX_HOME && !cfg.fs_jstph)
            {
                rcOptions[BOXGPSHOME] = 1;                   // Do RTL+Autoland+MotorOFF
                rcOptions[BOXGPSHOLD] = 0;                   // No explicit PH
            }
            else                                             // OMG we have no Homepos - just do Autoland, turn on PH just in case...
            {
                rcOptions[BOXGPSHOME] = 0;
                rcOptions[BOXGPSHOLD] = 1;                   // Pos Hold ON
                rcData[THROTTLE]      = cfg.rc_min - 10;     // Do Autoland turns off motors
            }
        }
#ifdef BARO
        if (sensors(SENSOR_BARO))
        {
            i++;
            if (!FSBaroThrottle)                             // Throttlechannel not valid any more, find new baselinethrottle for althold
            {
                FSBaroThrottle = DoMotorStats(true);         // Try to get FSBaro ESC Throttle from statistics
                if (FSBaroThrottle < ESCnoFlyThrottle) FSBaroThrottle = max(cfg.fs_rcthr, ESCnoFlyThrottle); // Throttle too low, take predefined throttle then
            }
        }
#endif
        switch(i)                                            // Do Failsafe according to sensorset
        {
        case 0:                                              // No Sensors just go down
        case 2:                                              // Just GPS and no baro
            rcCommand[THROTTLE] = rcData[THROTTLE] = cfg.fs_rcthr; // set rcCommand[THROTTLE] & rcData[THROTTLE](avoid disarm) to a predef. value, we have no baro...who builds such copters today?
            if (currentTimeMS > Failsafetimer) DisArmCopter();// Turn OFF motors Time in 0.1sec
            break;
        case 1:                                              // Just Baro. Do Autoland
            rcData[THROTTLE] = cfg.rc_min - 10;              // Do Autoland turns off motors
        case 3:                                              // GPS & Baro. Do RTH & Autoland. If Homepos not set just Autoland & try PH
            break;
        }
    }
    else
    {
        Failsafetimer  = 0;
        FSBaroThrottle = 0;                                  // Set to invalid
    }

    if (!failsafeCnt) GoodRCcnt = min(GoodRCcnt + 1, 250);   // Increase goodcount while failcount is zero
    else GoodRCcnt = 0;
    failsafeCnt++;                                           // reset to 0 by pwm / spektrum driver on Signal
}
// SOME RC FUNCTIONS END

// SOME LED FUNCTIONS START
static void DoLEDandBUZZER(void)
{
    static uint32_t calibratedAccTime;
    static uint8_t  buzzerFreq, vbatTimer = 0, ind = 0;
    static uint16_t vbatRawArray[8];  
    uint16_t        vbatRaw = 0;
    uint8_t         i;

    if (feature(FEATURE_VBAT))
    {
        if (!(++vbatTimer % VBATFREQ))
        {
            vbatRawArray[(ind++) % 8] = adcGetChannel(ADC_BATTERY);
            for (i = 0; i < 8; i++) vbatRaw += vbatRawArray[i];
            vbat = batteryAdcToVoltage(vbatRaw / 8);
        }
        if ((vbat > batteryWarningVoltage) || (vbat < cfg.vbatmincellvoltage)) buzzerFreq = 0;  // VBAT ok, buzzer off
        else buzzerFreq = 4;                                         // low battery
    }

    buzzer(buzzerFreq);                                              // external buzzer routine that handles buzzer events globally now

    if ((calibratingA && sensors(SENSOR_ACC)) || calibratingG)
    {
        LED0_TOGGLE;                                                 // Calibration phasis
    }
    else
    {
        if (f.ACC_CALIBRATED) LD0_OFF();                             // Crashpilot LED0_OFF;
        if (f.ARMED) LD0_ON();                                       // Crashpilot LED0_ON;
      
        switch(cfg.tele_prot)                                        // 0 (Dfault)=Keep Multiwii @CurrentUSB Baud, 1=Frsky @9600Baud, 2=Mavlink @CurrentUSB Baud, 3=Mavlink @57KBaud (like stock minimOSD wants it)
        {
        case 1:
            initFRSKYTelemetry(f.ARMED);                             // This will switch to/from 9600 or 115200 baud depending on state.
            break;
        case 3:                                                      // 3 = Mavlink @57KBaud
            initMinimOSDTelemetry(f.ARMED);
        default:
            break;
        }
    }

    if (feature(FEATURE_LED))
    {
        MWCRGBMONO_LED(buzzerFreq);
#ifdef LEDRING
        if (cfg.LED_Type == 3)
        {
            static uint32_t LEDTime;
            if (currentTimeMS >= LEDTime)
            {
                LEDTime = currentTimeMS + 50;
                ledringState();
            }
        }
#endif
    }

    if (currentTimeMS >= calibratedAccTime)
    {
        if (!f.SMALL_ANGLES_25)
        {
            f.ACC_CALIBRATED = 0;                                    // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTimeMS + 500;
        }
        else f.ACC_CALIBRATED = 1;
    }

    if (sensors(SENSOR_GPS)) BlinkGPSSats();
}                                                                    // END

// Some misc stuff goes here, LED etc.
// Purpose: Make mw.c a little bit more readable
// LED Inverter
void LD0_OFF(void)
{
    if (cfg.LED_invert)
    {
        LED0_ON;
    }
    else LED0_OFF;
}
void LD0_ON(void)
{
    if (cfg.LED_invert)
    {
        LED0_OFF;
    }
    else LED0_ON;
}
void LD1_OFF(void)
{
    if (cfg.LED_invert)
    {
        LED1_ON;
    }
    else LED1_OFF;
}
void LD1_ON(void)
{
    if (cfg.LED_invert)
    {
        LED1_OFF;
    }
    else LED1_ON;
}

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;
    for (r = 0; r < repeat; r++)
    {
        for (i = 0; i < num; i++)
        {
            LED0_TOGGLE;                                             // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

static void MWCRGBMONO_LED(uint8_t buzz)
{
    if (cfg.LED_Type == 1)  	                                   // MWCRGB
    {
        if (failsafeCnt > 5) LED_Value = 2100;
        else if (buzz > 3) LED_Value = 2200;                     // Changed http://fpv-treff.de/viewtopic.php?f=18&t=1368&start=1660#p33083
        else
        {
            if ((cfg.LED_Armed == 1) || f.ARMED)
            {
                if (rcData[cfg.LED_ControlChannel - 1] < 1300) LED_Value = cfg.LED_Pattern1;
                else if ((rcData[cfg.LED_ControlChannel - 1] >= 1300) && (rcData[cfg.LED_ControlChannel - 1] <= 1700)) LED_Value = cfg.LED_Pattern2;
                else LED_Value = cfg.LED_Pattern3;
            }
            else LED_Value = 1000;
        }
    }
    if (cfg.LED_Type == 2)                                       // MONO_LED
    {
        if (failsafeCnt > 5)                                      // Update the leds every frame, must also be called if leds are disabled now
        {
            LED_Value = 2100;
            LED_Value_Delay = 10;
        }
        else if (buzz > 3)
        {
            LED_Value = 2200;
            LED_Value_Delay = 10;
        }
        else
        {
            if ((cfg.LED_Armed == 1) || f.ARMED)
            {
                if (rcData[cfg.LED_ControlChannel - 1] < 1300)
                {
                    LED_Value = cfg.LED_Pattern1;
                    LED_Value_Delay = cfg.LED_Toggle_Delay1;
                }
                else if ((rcData[cfg.LED_ControlChannel - 1] >= 1300) && (rcData[cfg.LED_ControlChannel - 1] <= 1700))
                {
                    LED_Value = cfg.LED_Pattern2;
                    LED_Value_Delay = cfg.LED_Toggle_Delay2;
                }
                else
                {
                    LED_Value = cfg.LED_Pattern3;
                    LED_Value_Delay = cfg.LED_Toggle_Delay3;
                }
            }
            else
            {
                LED_Value = 1000;
                LED_Value_Delay=10;
            }
        }
        ledToggleUpdate(true);
    }
    else ledToggleUpdate(false);
}

static void BlinkGPSSats(void)
{
    static uint32_t Timebase;
    static uint8_t  cnt, blinkcase, blinkblock;

    if (GPS_numSat < 5 || !f.GPS_FIX) blinkcase = 0;
    if (f.GPS_FIX && GPS_numSat >= 5 && !blinkcase)
    {
        blinkcase = 1;
        blinkblock = GPS_numSat - 4;                             // Change suggested by Hinkel So 5 Sats will blink one time
        Timebase = 0;
    }
    if (currentTimeMS >= Timebase)
    {
        Timebase = currentTimeMS + 10;                           // Timebase 10 ms
        if (blinkcase) LD1_OFF();                                // LED1_OFF;
        switch(blinkcase)
        {
        case 0:
            break;
        case 1:
            cnt = 30;                                            // 300 ms on
            blinkcase++;
            break;
        case 2:
            LED1_TOGGLE;
            cnt--;
            if (!cnt) blinkcase++;
            break;
        case 3:
            cnt = 70;                                            // 700 ms off
            blinkcase++;
            break;
        case 4:
            cnt--;
            if (!cnt) blinkcase++;
            break;
        case 5:
            blinkblock--;
            if (!blinkblock) blinkcase++;
            else blinkcase = 1;
            break;
        case 6:
            cnt = 200;                                           // 2.0s Break before next flickerblock
            blinkcase++;
            break;
        case 7:
            cnt--;
            if (!cnt) blinkcase = 0;
            break;
        }
    }
}
// SOME LED FUNCTIONS END

// SOME OTHER FUNCTIONS START
static void DisArmCopter(void)
{
    f.ARMED        = 0;
    f.OK_TO_ARM    = 0;
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1)
    {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    }
    else
    {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

static float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}
// SOME OTHER FUNCTIONS END

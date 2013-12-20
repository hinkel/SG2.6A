#include "board.h"
#include "mw.h"

static uint8_t numberMotor = 0;
uint8_t useServo = 0;
int16_t motor[MAX_MOTORS];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

static motorMixer_t currentMixer[MAX_MOTORS];

static const motorMixer_t mixerTri[] =
{
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] =
{
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

static const motorMixer_t mixerQuadX[] =
{
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

static const motorMixer_t mixerBi[] =
{
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};

static const motorMixer_t mixerY6[] =
{
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

static const motorMixer_t mixerHex6P[] =
{
    { 1.0f, -1.0f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -1.0f, -0.866025f, -1.0f },     // FRONT_R
    { 1.0f,  1.0f,  0.866025f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f, -0.866025f,  1.0f },     // FRONT
    { 1.0f,  0.0f,  0.866025f, -1.0f },     // REAR
};

static const motorMixer_t mixerY4[] =
{
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

static const motorMixer_t mixerHex6X[] =
{
    { 1.0f, -0.866025f,  1.0f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  1.0f, -1.0f },     // REAR_L
    { 1.0f,  0.866025f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f, -0.866025f,  0.0f, -1.0f },     // RIGHT
    { 1.0f,  0.866025f,  0.0f,  1.0f },     // LEFT
};

static const motorMixer_t mixerOctoX8[] =
{
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] =
{
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] =
{
    { 1.0f,  1.0f, -0.5f,  1.0f },          // MIDFRONT_L
    { 1.0f, -0.5f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  0.5f,  1.0f },          // MIDREAR_R
    { 1.0f,  0.5f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  0.5f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -0.5f, -1.0f },          // MIDFRONT_R
    { 1.0f, -0.5f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  0.5f, -1.0f },          // MIDREAR_L
};

static const motorMixer_t mixerVtail4[] =
{
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -0.0f },          // FRONT_L
};

// Keep this synced with MultiType struct in mw.h!
const mixer_t mixers[] =
{
    //    Mo Se Mixtable
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
    { 2, 1, mixerBi },             // MULTITYPE_BI
    { 0, 1, NULL },                // * MULTITYPE_GIMBAL
    { 6, 0, mixerY6 },             // MULTITYPE_Y6
    { 6, 0, mixerHex6P },          // MULTITYPE_HEX6
    { 1, 1, NULL },                // * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MULTITYPE_OCTOFLATX
    { 1, 1, NULL },                // * MULTITYPE_AIRPLANE
    { 0, 1, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, 1, NULL },                // * MULTITYPE_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // MULTITYPE_VTAIL4
    { 0, 0, NULL },                // MULTITYPE_CUSTOM
};

uint8_t mixerInit(void)
{
    uint8_t i;
    useServo = mixers[cfg.mixerConfiguration].useServo;    // enable servos for mixes that require them. note, this shifts motor counts.
    if (feature(FEATURE_SERVO_TILT)) useServo = 1;         // if we want camstab/trig, that also enables servos, even if mixer doesn't

    if (cfg.mixerConfiguration == MULTITYPE_CUSTOM)
    {
        for (i = 0; i < MAX_MOTORS; i++)                   // load custom mixer into currentMixer
        {
            if (cfg.customMixer[i].throttle == 0.0f) break;// check if done
            currentMixer[i] = cfg.customMixer[i];
            numberMotor++;
        }
    }
    else
    {
        numberMotor = mixers[cfg.mixerConfiguration].numberMotor;
        // copy motor-based mixers
        if (mixers[cfg.mixerConfiguration].motor)
        {
            for (i = 0; i < numberMotor; i++) currentMixer[i] = mixers[cfg.mixerConfiguration].motor[i];
        }
    }
    return numberMotor;
}

void mixerLoadMix(int index)
{
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_MOTORS; i++)
        cfg.customMixer[i].throttle = 0.0f;

    // do we have anything here to begin with?
    if (mixers[index].motor != NULL)
    {
        for (i = 0; i < mixers[index].numberMotor; i++)
            cfg.customMixer[i] = mixers[index].motor[i];
    }
}

void writeServos(void)
{
    static uint32_t yawarmdelaytimer = 0;
    if (!useServo) return;
    switch (cfg.mixerConfiguration)
    {
    case MULTITYPE_BI:
        pwmWriteServo(0, servo[4]);
        pwmWriteServo(1, servo[5]);
        break;

    case MULTITYPE_TRI:
        if (!cfg.tri_ydel)
        {
            pwmWriteServo(0, servo[5]);                               // like always
        }
        else
        {
            if (f.ARMED)
            {
                if (!yawarmdelaytimer) yawarmdelaytimer = currentTimeMS + (uint32_t)cfg.tri_ydel;
                if (currentTimeMS >= yawarmdelaytimer) pwmWriteServo(0, servo[5]);  // like always
                else pwmWriteServo(0, cfg.tri_ymid);                                // Give middlesignal to yaw servo when disarmed
            }
            else
            {
                yawarmdelaytimer = 0;
                pwmWriteServo(0, cfg.tri_ymid);                                    // Give middlesignal to yaw servo when disarmed
            }
        }
        break;

    case MULTITYPE_AIRPLANE:
        break;
        
    case MULTITYPE_FLYING_WING:
            pwmWriteServo(0, servo[3]);
            pwmWriteServo(1, servo[4]);
        break;
        
    case MULTITYPE_GIMBAL:
        pwmWriteServo(0, servo[0]);
        pwmWriteServo(1, servo[1]);
        break;
        
    default:
        // Two servos for SERVO_TILT, if enabled
        if (feature(FEATURE_SERVO_TILT))
        {
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
        }
        break;
    }
}

// extern uint8_t cliMode;
void writeMotors(void)
{
    uint8_t i;
    for (i = 0; i < numberMotor; i++) pwmWriteMotor(i, motor[i]);
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;
    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}

static void airplaneMixer(void)
{
#if 0
    uint16_t servomid[8];
    int16_t flaperons[2] = { 0, 0 };

    for (i = 0; i < 8; i++) servomid[i] = 1500 + cfg.servotrim[i]; // servo center is 1500?

    if (!f.ARMED) motor[0] = cfg.esc_moff; // Kill throttle when disarmed
    else motor[0] = rcData[THROTTLE];

    if (cfg.flaperons) { }

    if (cfg.flaps)
    {
        int16_t flap = 1500 - constrain(rcData[cfg.flaps], cfg.servoendpoint_low[2], cfg.servoendpoint_high[2]);
        static int16_t slowFlaps = flap;

        if (cfg.flapspeed)
        {
            if (slowFlaps < flap)
            {
                slowFlaps += cfg.flapspeed;
            }
            else if (slowFlaps > flap)
            {
                slowFlaps -= cfg.flapspeed;
            }
        }
        else
        {
            slowFlaps = flap;
        }
        servo[2] = servomid[2] + (slowFlaps * cfg.servoreverse[2]);
    }

    if (f.PASSTHRU_MODE)   // Direct passthru from RX
    {
        servo[3] = servomid[3] + ((rcCommand[ROLL] + flapperons[0]) * cfg.servoreverse[3]);     //   Wing 1
        servo[4] = servomid[4] + ((rcCommand[ROLL] + flapperons[1]) * cfg.servoreverse[4]);     //   Wing 2
        servo[5] = servomid[5] + (rcCommand[YAW] * cfg.servoreverse[5]);                        //   Rudder
        servo[6] = servomid[6] + (rcCommand[PITCH] * cfg.servoreverse[6]);                      //   Elevator
    }
    else     // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
    {
        servo[3] = (servomid[3] + (((int16_t)axisPID[ROLL] + flapperons[0]) * cfg.servoreverse[3]));     //   Wing 1
        servo[4] = (servomid[4] + (((int16_t)axisPID[ROLL] + flapperons[1]) * cfg.servoreverse[4]));     //   Wing 2
        servo[5] = (servomid[5] + ((int16_t)axisPID[YAW] * cfg.servoreverse[5]));                        //   Rudder
        servo[6] = (servomid[6] + ((int16_t)axisPID[PITCH] * cfg.servoreverse[6]));                      //   Elevator
    }
#endif
}

void mixTable(void)
{
    int16_t  maxMotor;
    uint32_t i;

    // prevent "yaw jump" during yaw correction
    if (numberMotor > 3) axisPID[YAW] = constrain(axisPID[YAW], -100.0f - (float)abs(rcCommand[YAW]), +100.0f + (float)abs(rcCommand[YAW]));

    // motors for non-servo mixes
    if (numberMotor > 1)
        for (i = 0; i < numberMotor; i++)
            motor[i] = (float)rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + cfg.tri_ydir * axisPID[YAW] * currentMixer[i].yaw;

    // airplane / servo mixes
    switch (cfg.mixerConfiguration)
    {
    case MULTITYPE_BI:
        servo[4] = constrain(1500 + (cfg.tri_ydir * axisPID[YAW]) + axisPID[PITCH], 1020, 2000);   //LEFT
        servo[5] = constrain(1500 + (cfg.tri_ydir * axisPID[YAW]) - axisPID[PITCH], 1020, 2000);   //RIGHT
        break;

    case MULTITYPE_TRI:
        servo[5] = constrain(cfg.tri_ymid + cfg.tri_ydir * axisPID[YAW], cfg.tri_ymin, cfg.tri_ymax); //REAR
        break;

    case MULTITYPE_GIMBAL:
        servo[0] = constrain(cfg.gbl_pmd + (float)cfg.gbl_pgn * angle[PITCH] * 0.0625f + rcCommand[PITCH], cfg.gbl_pmn, cfg.gbl_pmx);
        servo[1] = constrain(cfg.gbl_rmd + (float)cfg.gbl_rgn * angle[ROLL]  * 0.0625f + rcCommand[ROLL] , cfg.gbl_rmn, cfg.gbl_rmx);
        break;

    case MULTITYPE_AIRPLANE:
        airplaneMixer();
        break;

    case MULTITYPE_FLYING_WING:
        motor[0] = rcCommand[THROTTLE];
        if (f.PASSTHRU_MODE)
        {
            // do not use sensors for correction, simple 2 channel mixing
            servo[0]  = cfg.pitch_direction_l * (rcData[PITCH] - cfg.rc_mid) + cfg.roll_direction_l * (rcData[ROLL] - cfg.rc_mid);
            servo[1]  = cfg.pitch_direction_r * (rcData[PITCH] - cfg.rc_mid) + cfg.roll_direction_r * (rcData[ROLL] - cfg.rc_mid);
        }
        else
        {
            // use sensors to correct (gyro only or gyro + acc)
            servo[0]  = cfg.pitch_direction_l * axisPID[PITCH] + cfg.roll_direction_l * axisPID[ROLL];
            servo[1]  = cfg.pitch_direction_r * axisPID[PITCH] + cfg.roll_direction_r * axisPID[ROLL];
        }
        servo[0] = constrain(servo[0] + cfg.wing_left_mid, cfg.wing_left_min, cfg.wing_left_max);
        servo[1] = constrain(servo[1] + cfg.wing_right_mid, cfg.wing_right_min, cfg.wing_right_max);
        break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT))
    {
        uint16_t aux[2] = { 0, 0 };
        if ((cfg.gbl_flg & GIMBAL_NORMAL) || (cfg.gbl_flg & GIMBAL_TILTONLY)) aux[0] = rcData[AUX3] - cfg.rc_mid;
        if (!(cfg.gbl_flg & GIMBAL_DISABLEAUX34)) aux[1] = rcData[AUX4] - cfg.rc_mid;
        servo[0] = cfg.gbl_pmd + aux[0];
        servo[1] = cfg.gbl_rmd  + aux[1];

        if (rcOptions[BOXCAMSTAB])
        {
            if (cfg.gbl_flg & GIMBAL_MIXTILT)
            {
                servo[0] -= ((float)(-cfg.gbl_pgn) * angle[PITCH] * 0.0625f) - (float)cfg.gbl_rgn * angle[ROLL] * 0.0625f;
                servo[1] += ((float)(-cfg.gbl_pgn) * angle[PITCH] * 0.0625f) + (float)cfg.gbl_rgn * angle[ROLL] * 0.0625f;
            }
            else
            {
                servo[0] += (float)cfg.gbl_pgn * angle[PITCH] * 0.0625f;
                servo[1] += (float)cfg.gbl_rgn * angle[ROLL]  * 0.0625f;
            }
        }

        servo[0] = constrain(servo[0], cfg.gbl_pmn, cfg.gbl_pmx);
        servo[1] = constrain(servo[1], cfg.gbl_rmn, cfg.gbl_rmx);
    }

    if (cfg.gbl_flg & GIMBAL_FORWARDAUX)
    {
        int offset = 0;
        if (feature(FEATURE_SERVO_TILT))
            offset = 2;
        for (i = 0; i < 4; i++)
            pwmWriteServo(i + offset, rcData[AUX1 + i]);
    }

    if (feature(FEATURE_LED) && (cfg.LED_Type == 1))
    {
        if (feature(FEATURE_SERVO_TILT))
            pwmWriteServo(2, LED_Value);
        else
            pwmWriteServo(0, LED_Value);
    }

    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++) if (motor[i] > maxMotor) maxMotor = motor[i];
    for (i = 0; i < numberMotor; i++)
    {
        if (maxMotor > cfg.esc_max) motor[i] -= maxMotor - cfg.esc_max;    // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] = constrain(motor[i], cfg.esc_min, cfg.esc_max);
        if ((rcData[THROTTLE]) < cfg.rc_min)
        {
            if (!feature(FEATURE_MOTOR_STOP)) motor[i] = cfg.esc_min;
            else motor[i] = cfg.esc_moff;
        }
        if (!f.ARMED) motor[i] = cfg.esc_moff;
    }
}

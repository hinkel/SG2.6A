#include "board.h"
#include "mw.h"

//#define Earthradius           637100078.5d                         // Average Earthradius in cm (from 637813700cm ellipsoid to a sphere)
//#define MagicEarthNumber       1.1119494f                          // Based on average Earthradius pi/180 * Earthradius / 10^7
//#define MagicEarthNumber       1.1113175f                            // used by apm "INERTIALNAV_LATLON_TO_CM"
#define MagicEarthNumber 1.113195f                                   // LOL! The "new" apm number
//#define MagicEarthNumber         1.11163345f                         // OWN Earth number does correct projection!!

// They are defined in mw.h
// #define LAT  0
// #define LON  1
// #define GPS_Y 0
// #define GPS_X 1

typedef struct PID_PARAM_
{
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

typedef struct PID_
{
    float integrator;          // integrator value "I"
    float last_error;          // D
    float delta1;              // D pt2 element
} PID;

// NAV & PH PID Variables
static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;
static PID_PARAM navPID_PARAM;

static PID       posholdPID[2];
static PID       poshold_ratePID[2];
static PID       navPID[2];

// INS & Core Variables
static int16_t   maxbankbrake100;         // Maximum GPS Brake Tiltangle < maxbank100
static float     Real_GPS_speed[2] = { 0, 0 }; // Is the earthframespeed measured by GPS Coord Difference
static float     MIX_speed[2]      = { 0, 0 }; // That is a 1:1 Mix of Acc speed and GPS Earthframespeed
static int32_t   Last_Real_GPS_coord[2];
uint32_t         TimestampNewGPSdata;     // Crashpilot in micros

// Earth / Location constants
static float     OneCmTo[2];              // Moves one cm in Gps coords
static float     CosLatScaleLon;          // this is used to offset the shrinking longitude as we go towards the poles
static float     GPSRAWtoRAD;

static float     get_P(float error, struct PID_PARAM_* pid);
static float     get_I(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);
static float     get_D(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);
static int32_t   wrap_36000(int32_t angle);

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps&acc position data
// This is another important part of the gps ins
void GPS_calc_velocity(void)                                                    // actual_speed[GPS_Y] y_GPS_speed positve = Up (NORTH) // actual_speed[GPS_X] x_GPS_speed positve = Right (EAST)
{
    static uint32_t LastTimestampNewGPSdata;
    static bool     INSusable;
    float           gpsHz, tmp0;
    uint32_t        RealGPSDeltaTime;
    uint8_t         i;

    GPS_calc_longitude_scaling(false);                                          // Init CosLatScaleLon if not already done to avoid div by zero etc..
    RealGPSDeltaTime = TimestampNewGPSdata - LastTimestampNewGPSdata;           // RealGPSDeltaTime in ms! NOT us!
    LastTimestampNewGPSdata = TimestampNewGPSdata;
    if (RealGPSDeltaTime != 0)                                                  // New GPS Data?
    {
        INSusable = false;                                                      // Set INS to ununsable in advance we will see later
        if (RealGPSDeltaTime < 400)                                             // In Time? 2,5Hz-XXHz
        {
            INSusable = true;                                                   // INS is alive
            gpsHz = 1000.0f/(float)RealGPSDeltaTime;                            // Set GPS Hz, try to filter below
            if (RealGPSDeltaTime >  10 && RealGPSDeltaTime <  30) gpsHz = 50.0f;// 50Hz Data  20ms filter out timejitter
            if (RealGPSDeltaTime >  40 && RealGPSDeltaTime <  60) gpsHz = 20.0f;// 20Hz Data  50ms filter out timejitter
            if (RealGPSDeltaTime >  80 && RealGPSDeltaTime < 120) gpsHz = 10.0f;// 10Hz Data 100ms filter out timejitter
            if (RealGPSDeltaTime > 180 && RealGPSDeltaTime < 220) gpsHz = 5.0f; //  5Hz Data 200ms
            if (RealGPSDeltaTime > 230 && RealGPSDeltaTime < 270) gpsHz = 4.0f; //  4Hz Data 250ms
            tmp0 = MagicEarthNumber * gpsHz;
            Real_GPS_speed[LON] = (float)(Real_GPS_coord[LON] - Last_Real_GPS_coord[LON]) * tmp0 * CosLatScaleLon ; // cm/s
            Real_GPS_speed[LAT] = (float)(Real_GPS_coord[LAT] - Last_Real_GPS_coord[LAT]) * tmp0;                   // cm/s

            for (i = 0; i < 2; i++)
            {
                Last_Real_GPS_coord[i] = Real_GPS_coord[i];
                ACC_speed[i]           = (ACC_speed[i] * cfg.gps_ins_vel) + (Real_GPS_speed[i] * (1.0f - cfg.gps_ins_vel)); // CF: GPS Correction
            }
        }
    }                                                                           // End of X Hz Loop

    if ((millis() - TimestampNewGPSdata) > 500) INSusable = false;              // INS is NOT OK, too long (500ms) no correction

    if (INSusable)
    {
        for (i = 0; i < 2; i++) MIX_speed[i] = (ACC_speed[i] + Real_GPS_speed[i]) * 0.5f;
    }
    else
    {
        GPS_reset_nav();                                                        // Ins is fucked, reset stuff
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm Get bearing from pos1 to pos2, returns an 1deg = 100 precision
// Now with more correct BEARING calclation according to this: http://www.movable-type.co.uk/scripts/latlong.html
// var coslat2 = cos(lat2);
// var y = sin(dLon) * coslat2;
// var x = cos(lat1) * sin(lat2) - sin(lat1) * coslat2 * cos(dLon);
// var brng = atan2(y, x).toDeg(); -180  0 +180
//
void GPS_distance_cm_bearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, uint32_t *dist, int32_t *bearing)
{
    float dLatRAW, dLonRAW, x, y, lat1RAD, lat2RAD, Coslat2RAD;
    if (*lat2 && *lat1 && *lon2 && *lon1)                                       // Crashpilot Errorcheck
    {
        GPS_calc_longitude_scaling(false);
        dLatRAW    = (float)(*lat2 - *lat1);                                    // difference of latitude in 1/10 000 000 degrees
        dLonRAW    = (float)(*lon2 - *lon1);      
        x          = dLonRAW * CosLatScaleLon;
        *dist      = sqrtf(dLatRAW * dLatRAW + x * x) * MagicEarthNumber;       // dist in cm
        dLonRAW   *= GPSRAWtoRAD;                                               // dLatRAW = dLatRAW * GPSRAWtoRAD; // 10^7 DEGdelta to Rad
        lat1RAD    = *lat1 * GPSRAWtoRAD;
        lat2RAD    = *lat2 * GPSRAWtoRAD;
        Coslat2RAD = cosf(lat2RAD);
        y          = sinf(dLonRAW) * Coslat2RAD;
        x          = cosf(lat1RAD) * sinf(lat2RAD) - sinf(lat1RAD) * Coslat2RAD * cos(dLonRAW);
        *bearing   = constrain((int32_t)(atan2f(y, x) * RADtoDEG100), -18000, 18000);
        if (*bearing < 0) *bearing += 36000;
    }
    else                                                                        // Error!
    {
        *dist = 0;
        *bearing = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates. Crashpilot distance error in CM now!
void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng)
{
    GPS_calc_longitude_scaling(false);                          // Just in case scaling isn't done
    if (*target_lng && *target_lat && *gps_lng && *gps_lat)
    {
        LocError[LON] = (float)(*target_lng - *gps_lng) * MagicEarthNumber * CosLatScaleLon; // X Error in cm not lon!
        LocError[LAT] = (float)(*target_lat - *gps_lat) * MagicEarthNumber;                  // Y Error in cm not lat!
    }
    else
    {
        LocError[LON] = 0;
        LocError[LAT] = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Crashpilot NEW PH Logic
// #define GPS_X 1 // #define GPS_Y 0
// #define LON   1 // #define LAT   0
// VelEast;   // 1 // VelNorth;  // 0
// 0 is NICK part  // 1 is ROLL part
// actual_speed[GPS_Y] = VelNorth;

void GPS_calc_posholdCrashpilot(bool overspeed)
{
    uint8_t axis;
    float   rate_error, tilt, tmp0, tmp1, expo;
    float   maxbank100new;

    expo = (float)cfg.gps_ph_expo * 0.01f;   
    for (axis = 0; axis < 2; axis++)
    {
        maxbank100new = (float)maxbank100;
        if (overspeed)
        {
            tmp1 = abs(ACC_speed[axis]);
            if (tmp1 == 0) tmp1 = 1.0f;
            tmp0 = (float)cfg.gps_ph_settlespeed / tmp1;
            tmp1 = (float)cfg.gps_ph_minbrakepercent * 0.01f;                   // this is the minimal break percentage
            tmp0 = constrain(sqrt(tmp0), tmp1, 1.0f);                           // 1 - 100%
            maxbank100new = (float)maxbankbrake100 * tmp0;
        }

        rate_error  = get_P(LocError[axis],                                 &posholdPID_PARAM);      // Do absolute position error here, that means transform positionerror to a velocityerror
        rate_error += get_I(LocError[axis], &dTnav, &posholdPID[axis],      &posholdPID_PARAM);
        rate_error  = constrain(rate_error - ACC_speed[axis], -1000, 1000);                          // +- 10m/s; // Calculate velocity Error for actual axis
        tilt        = get_P(rate_error,                                     &poshold_ratePID_PARAM);
        tilt       += get_D(rate_error,     &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM); // Constrained to half max P
        nav[axis]   = constrain(tilt, -maxbank100new, maxbank100new);

        if (!overspeed && cfg.gps_ph_expo)                                      // Do expo if set and currently not braking
        {
            tmp1      = nav[axis] / maxbank100new;                              // Put in range of -1 +1
            tmp0      = tmp1 * (1.0f - expo) + tmp1 * tmp1 * tmp1 * expo;       // Do expo here
            nav[axis] = tmp0 * maxbank100new;                                   // Put back in original range
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
void GPS_calc_nav_rate(uint16_t max_speed)
{
    float   trig[2], rate_error;
    float   temp, tiltcomp, trgtspeed;
    uint8_t axis;
    int32_t crosstrack_error;
    if ((abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) && cfg.nav_ctrkgain != 0)// If we are too far off or too close we don't do track following
    {
        temp = (float)(target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * (float)wp_distance * cfg.nav_ctrkgain;  // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    }
    else nav_bearing = target_bearing;

    temp = (float)(9000l - nav_bearing) * RADX100;                              // nav_bearing and maybe crosstrack
    trig[GPS_X] = cosf(temp);
    trig[GPS_Y] = sinf(temp);
    for (axis = 0; axis < 2; axis++)
    {
        trgtspeed  = (trig[axis] * (float)max_speed);                           // Target speed
        rate_error = trgtspeed - MIX_speed[axis];                               // Since my INS Stuff is shit, reduce ACC influence to 50% anyway better than leadfilter
        rate_error = constrain(rate_error, -1000, 1000);
        nav[axis]  = get_P(rate_error,                        &navPID_PARAM) +  // P + I + D
                     get_I(rate_error, &dTnav, &navPID[axis], &navPID_PARAM) +
                     get_D(rate_error, &dTnav, &navPID[axis], &navPID_PARAM);
        if (cfg.nav_tiltcomp)                                                   // Do the apm 2.9.1 magic tiltcompensation
        {
            tiltcomp = trgtspeed * trgtspeed * ((float)cfg.nav_tiltcomp * 0.0001f);
            if(trgtspeed < 0) tiltcomp = -tiltcomp;
        }
        else tiltcomp = 0;
        nav[axis]  = nav[axis] + tiltcomp;                                      // Constrain is done at the end of all GPS stuff
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

uint16_t GPS_calc_desired_speed(void)
{
    uint16_t max = cfg.nav_speed_max;
    if (!WP_Fastcorner) max = (uint16_t)min((uint32_t)max, wp_distance / (uint32_t)cfg.nav_approachdiv); //nav_approachdiv = 2-10
    else max = min(max, wp_distance);
    if (max > waypoint_speed_gov)
    {
        waypoint_speed_gov += (100.0f * dTnav);                                 // increase speed
        max = waypoint_speed_gov;
    }
    max = constrain(max, (uint16_t)cfg.nav_speed_min, cfg.nav_speed_max);       // Put output in desired range
    return max;
}

////////////////////////////////////////////////////////////////////////////////////
// ***   Utilities   ***
////////////////////////////////////////////////////////////////////////////////////
void GPS_set_pids(void)                                                         // Is done after EEPROM read
{
    float imax = (float)cfg.gps_maxangle * 50;                                  // Set Imax to half maximal Tiltangle (*100)
    posholdPID_PARAM.kP        = (float)cfg.P8[PIDPOS]  /   100.0f;             // Original Scale
    posholdPID_PARAM.kI        = (float)cfg.I8[PIDPOS]  / 10000.0f;             // Now used scaled down by /10
    cfg.D8[PIDPOS]             = 0;
    posholdPID_PARAM.Imax      = imax;

    poshold_ratePID_PARAM.kP   = (float)cfg.P8[PIDPOSR] /     5.0f;             // Need more P
    poshold_ratePID_PARAM.kD   = (float)cfg.D8[PIDPOSR] /  1000.0f;
    cfg.I8[PIDPOSR]            = 0;
    poshold_ratePID_PARAM.Imax = imax;

    navPID_PARAM.kP            = (float)cfg.P8[PIDNAVR] /    10.0f;
    navPID_PARAM.kI            = (float)cfg.I8[PIDNAVR] /  1000.0f;
    navPID_PARAM.kD            = (float)cfg.D8[PIDNAVR] /  1000.0f;
    navPID_PARAM.Imax          = imax;
    
    GPSRAWtoRAD                = 0.0000001f * M_PI / 180.0f;
    OneCmTo[LAT]               = 1.0f / MagicEarthNumber;                       // Moves North one cm
    maxbank10                  = (int16_t)cfg.gps_maxangle * 10;                // Initialize some values here
    maxbank100                 = (int16_t)cfg.gps_maxangle * 100;
    maxbankbrake100             = (int16_t)cfg.gps_ph_brakemaxangle * 100;
    if (maxbankbrake100 > maxbank100) maxbankbrake100 = maxbank100;
}

static float get_P(float error, struct PID_PARAM_ *pid)
{
    return error * pid->kP;
}

static float get_I(float error, float *dt, struct PID_ *pid, struct PID_PARAM_ *pid_param)
{
    float time = *dt;
    pid->integrator += (error * pid_param->kI * time);
    pid->integrator  = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

static float get_D(float error, float *dt, struct PID_ *pid, struct PID_PARAM_ *pid_param)
{
    static float GPSDptCut;
    static bool ini = false;
    float DeltaRAW;
    if(!ini)
    {
        GPSDptCut  = 0.5f / (M_PI * (float)cfg.gpscuthz);
        ini = true;
    }
    DeltaRAW        = ((error - pid->last_error) / *dt);
    pid->last_error = error;
    pid->delta1     = pid->delta1 + (*dt / (GPSDptCut + *dt)) * (DeltaRAW - pid->delta1);
    return pid->delta1 * pid_param->kD;
}

static void reset_PID(struct PID_* pid)
{
    pid->integrator = 0;
    pid->last_error = 0;
    pid->delta1     = 0;
}

void GPS_reset_home_position(void)
{
    if (f.GPS_FIX && GPS_numSat >= 5)
    {
        GPS_calc_longitude_scaling(true);
        GPS_home[LAT] = Real_GPS_coord[LAT];
        GPS_home[LON] = Real_GPS_coord[LON];
        nav_takeoff_heading = heading;                                          // save takeoff heading
        f.GPS_FIX_HOME = 1;
    }
}

void GPS_reset_nav(void)                                                        // reset navigation (stop the navigation processor, and clear nav)
{
    uint8_t i;
    for (i = 0; i < 2; i++)
    {
        Last_Real_GPS_coord[i] = Real_GPS_coord[i];
        ACC_speed[i] = MIX_speed[i] = Real_GPS_speed[i] = 0;
        LocError[i]  = 0;
        GPS_angle[i] = 0;
        nav[i]       = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }
    waypoint_speed_gov = (float)cfg.nav_speed_min;
    WP_Fastcorner = false;
}

bool DoingGPS(void)
{
    if (f.GPS_HOLD_MODE || f.GPS_HOME_MODE) return true;
    else return false;
}

bool check_missed_wp(void)
{
    float temp;
    temp = (float)(target_bearing - original_target_bearing);
    temp = wrap_18000(temp);
    return (abs(temp) > 10000);                                                 // we passed the waypoint by 100 degrees
}

void GPS_calc_longitude_scaling(bool force)
{
    static bool ini = false;  
    float rads;
    if (force) ini = false;                                                     // Force Re-ini of scalingfactors
    if (ini) return;
    ini  = true;
    rads = fabs((float)Real_GPS_coord[LAT] * GPSRAWtoRAD);
    CosLatScaleLon = cosf(rads);                                                // can only be 0 at 90 degree, perhaps at the poles?
    if (!CosLatScaleLon) CosLatScaleLon = 0.001745328f;                         // Avoid divzero (value is cos of 89.9 Degree)
    OneCmTo[LON] = OneCmTo[LAT] / CosLatScaleLon;                               // Moves EAST  one cm // OneCmTo[LAT] calculated on startup
}

float wrap_180(float value)                                                     // Wrap to -180 0 +180 Degree
{
    while (value >   180.0f) value -= 360.0f;
    while (value <= -180.0f) value += 360.0f;
    return value;
}

float wrap_18000(float value)
{
    while (value >   18000.0f) value -= 36000.0f;
    while (value <= -18000.0f) value += 36000.0f;
    return value;
}

static int32_t wrap_36000(int32_t angle)
{
    while (angle <      0) angle += 36000;
    while (angle >= 36000) angle -= 36000;
    return angle;
}

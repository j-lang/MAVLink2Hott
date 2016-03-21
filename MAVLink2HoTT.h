#define byte uint8_t

/* Flightmode definitions */
#define MOTORS_ARMED  128
#define STAB 0
#define ACRO 1
#define ALTH 2
#define AUTO 3
#define GUID 4
#define LOIT 5
#define RETL 6
#define CIRC 7
#define LAND 9
#define OFLO 10
#define DRIF 11
#define SPOR 13
#define FLIP 14
#define ATUN 15
#define POSI 16
#define BREA 17
#define THRO 18
/* Plane */
#define MANU 19
#define TRAI 20
#define FBWA 21
#define FBWB 22
#define CRUI 23

#define OFFSET_HEIGHT 500   /* OffSet for HoTT protocol, 500 = 0 in HoTT display */
#define OFFSET_VARIO  30000 /* OffSet for HoTT protocol, 30000 = 0 in HoTT display */
#define START                   1
#define MSG_RATE            10              // Hertz

// ******************************************
// Message #0  HEARTHBEAT 
uint8_t    apm_type = 0;
uint8_t    apm_autopilot = 0;
uint8_t    apm_base_mode = 0;
uint8_t    apm_mav_mode = 0;
uint32_t   apm_custom_mode = 0;
uint8_t    apm_system_status = 0;
uint8_t    apm_mavlink_version = 0;

// Message # 1  SYS_STATUS 
uint16_t apm_temp = 0;                  // pressure sensor temp
float    apm_abs = 0;                   // absolute pressure *0.01 as float




// Message #24  GPS_RAW_INT 
uint8_t    apm_fixtype = 3;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    apm_sat_visible = 0;           // numbers of visible satelites
int32_t    apm_latitude = 0;              // 585522540;
int32_t    apm_longitude = 0;            // 162344467;
int32_t    apm_gps_altitude = 0;        // 1000 = 1m
uint16_t   apm_gps_vel = 0;

// Message #24  ATTITUDE 
float  apm_yaw = 0;                     // relative heading from DCM - it is actually a float but HoTT needs int8

// Message # 29  SCALED_PRESSURE 
uint16_t  apm_voltage_battery = 0;    // 1000 = 1V
int16_t    apm_current_battery = 0;    //  10 = 1A


// Message #74 VFR_HUD 
int32_t    apm_airspeed = 0;
uint32_t  apm_groundspeed = 0;
uint32_t  apm_heading = 0;
uint16_t  apm_throttle = 0;

// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    apm_bar_altitude = 0;    // 100 = 1m
int32_t    apm_climb_rate=0;        // 100= 1m/s

// Message #27 RAW IMU 
int32_t   apm_accX = 0;
int32_t   apm_accY = 0;
int32_t   apm_accZ = 0;

int32_t   apm_accX_old = 0;
int32_t   apm_accY_old = 0;
int32_t   apm_accZ_old = 0;

int16_t   apm_ceiling = 0;               // altitude ceiling from AQ = Max Altitude in HoTT

static uint8_t  apm_mav_type;
static uint8_t  apm_nav_mode = 0;               // Navigation mode

static int8_t   pos_NS;
static int8_t   pos_EW;
static int8_t   NSEW;  
static float    home_distance_calc = 0;        // distance to home //
static float    bearing;
static bool     m2h_got_home = 0;              // tels if home position is set
static int16_t  degMin;
static int16_t  deg;
static int32_t  deg_sec;
static float    sec;                           // Time functions: in HoTT the running time
static int8_t   min;                           // in Spektrum the actual GPS aquired time
static int8_t   hour;
static float    lat;
static float    lon;
 
static uint8_t mavlinkHB_char = 0;             // MavLink HeartBeat character
static uint8_t free_char1;
static uint8_t free_char2;
static uint8_t free_char3;
static uint8_t msg_number;  

// Flightmode 
byte flMode;   
byte isArmed = 0;
byte isArmedOld = 0;

// Timer-events
static uint8_t counter = 0;                        /* General counter */
unsigned long tEvent = 0;
static uint8_t eventActive = 0;
static uint8_t ioCounter = 0;  
unsigned long tNow = 0;


static bool     displayVersionDone = 0;

/* Signaling event definitions */
enum signalingEventTypes {
    SIG_EVENT_NONE = 0,
    SIG_EVENT_DISARMED_NOGPS,
    SIG_EVENT_DISARMED_GPS,
    SIG_EVENT_ARMED,
    SIG_EVENT_FLYING,
    SIG_EVENT_HFREE,
    SIG_EVENT_ALTHOLD,
    SIG_EVENT_POSHOLD,
    SIG_EVENT_MISSION,
    SIG_EVENT_DVH,
    SIG_EVENT_LOWBATT,
    SIG_EVENT_RADIOLOSS,
    SIG_EVENT_RADIOLOSS2,
    SIG_EVENT_INIT,
    SIG_EVENT_LAND,
};


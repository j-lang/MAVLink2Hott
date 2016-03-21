#ifndef HoTTv4_h
#define HoTTv4_h

/** ###### HoTT module specifications ###### */
#define HOTTV4_GENERAL_AIR_MODULE 0x8D            // General Air Module ID
#define HOTTV4_GENERAL_AIR_SENSOR_TEXT_ID 0xD0    // General Air Text ID
#define HOTTV4_ELECTRICAL_AIR_MODULE 0x8E         // Electric Air Module ID
#define HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID 0xE0 // Electric Air Text ID
#define HOTTV4_GPS_MODULE 0x8A                    // GPS Module ID
#define HOTTV4_GPS_SENSOR_ID 0xA0                 // GPS Text ID
#define HOTTV4_VARIO_MODULE 0x89                  // Vario Sensor Module ID
#define HOTTV4_VARIO_SENSOR_ID 0x90               // Vario Text ID

#ifdef HOTTFAST  // Faster protocol (>oct 2013) for BOTH!! transmitter & receiver
  #define HOTTV4_TX_DELAY     500   // time in us between 2 bytes in a binpacket 
  #define HOTTV4_SENDDELAY    5000  // HoTT line idle-protocol
#endif

#ifdef HOTTNORM  // Normal protcol (<oct 2013) for either transmitter or receiver
  #define HOTTV4_TX_DELAY     1500  // time in us between 2 bytes in a binpacket
  #define HOTTV4_SENDDELAY    4500  // HoTT line idle-protocol                 
#endif

/** ###### VARIO Text ###### */
#define VARIO_ASCIIS 21
// Buffer for the available 21 ASCII + \0 chars
char text[VARIO_ASCIIS+1];

/* place status messages in FLASH memory instead of RAM */
const prog_char flModeStrings [48][20] PROGMEM = {
//01234567891234567890
 "Stabilized         ",  //0  flightMode
 "Acro               ",  //1
 "Altitude Hold      ",  //2
 "Auto               ",  //3
 "Guided             ",  //4
 "Loiter             ",  //5
 "Return to Launch   ",  //6
 "Circle             ",  //7
 "                   ",  //8
 "Land               ",  //9
 "                   ",  //10
 "Drift              ",  //11
 "                   ",  //12
 "Sport              ",  //13
 "Flip               ",  //14
 "Auto Tune          ",  //15
 "Position Hold      ",  //16
 "Brake              ",  //17
 "Throw              ",  //18
 "Manual             ",  //19  Plane
 "Training           ",  //20
 "Fly by Wire A      ",  //21
 "Fly by Wire B      ",  //22
 "Cruise             ",  //23
 "Stabilized      Dis",  //0  flightMode
 "Acro            Dis",  //1
 "Altitude Hold   Dis",  //2
 "Auto            Dis",  //3
 "Guided          Dis",  //4
 "Loiter          Dis",  //5
 "ReturnToLaungch Dis",  //6
 "Circle          Dis",  //7
 "                   ",  //8
 "Land            Dis",  //9
 "                   ",  //10
 "Drift           Dis",  //11
 "                   ",  //12
 "Sport           Dis",  //13
 "Flip            Dis",  //14
 "Auto Tune       Dis",  //15
 "Position Hold   Dis",  //16
 "Brake           Dis",  //17
 "Throw           Dis",  //18
 "Manual          Dis",  //19  Plane
 "Training        Dis",  //20
 "Fly by Wire A   Dis",  //21
 "Fly by Wire B   Dis",  //22
 "Cruise          Dis",  //23
};
 
 
 
/* place flightMode strings table in PROGMEM flash */
const char *flight_modes[] = {
 flModeStrings[0],  flModeStrings[1],  flModeStrings[2],  flModeStrings[3],  flModeStrings[4],  flModeStrings[5],
 flModeStrings[6],  flModeStrings[7],  flModeStrings[8],  flModeStrings[9],  flModeStrings[10], flModeStrings[11],
 flModeStrings[12], flModeStrings[13], flModeStrings[14], flModeStrings[15], flModeStrings[16], flModeStrings[17],
 flModeStrings[18], flModeStrings[19], flModeStrings[20], flModeStrings[21], flModeStrings[22], flModeStrings[23], 
 flModeStrings[24], flModeStrings[25], flModeStrings[26], flModeStrings[27], flModeStrings[28], flModeStrings[29],
 flModeStrings[30], flModeStrings[31], flModeStrings[32], flModeStrings[33], flModeStrings[34], flModeStrings[35],
 flModeStrings[36], flModeStrings[37], flModeStrings[38], flModeStrings[39], flModeStrings[40], flModeStrings[41],
 flModeStrings[42], flModeStrings[43], flModeStrings[44], flModeStrings[45], flModeStrings[46], flModeStrings[47],
 flModeStrings[48]
 };

const uint8_t kHoTTv4BinaryPacketSize = 45; 
static uint8_t outBuffer[45];  // if using textmode too, increase to 173

static enum _hottV4_state {
    IDLE,
    BINARY,
    TEXT,
    SENDING,
  } hottV4_state = IDLE;


typedef enum {
  HoTTv4NotificationErrorCalibration     = 0x01,
  HoTTv4NotificationErrorReceiver        = 0x02,
  HoTTv4NotificationErrorDataBus         = 0x03,
  HoTTv4NotificationErrorNavigation      = 0x04,
  HoTTv4NotificationErrorError           = 0x05,
  HoTTv4NotificationErrorCompass         = 0x06,
  HoTTv4NotificationErrorSensor          = 0x07,
  HoTTv4NotificationErrorGPS             = 0x08,
  HoTTv4NotificationErrorMotor           = 0x09,
  
  HoTTv4NotificationMaxTemperature       = 0x0A,
  HoTTv4NotificationAltitudeReached      = 0x0B,
  HoTTv4NotificationWaypointReached      = 0x0C,
  HoTTv4NotificationNextWaypoint         = 0x0D,
  HoTTv4NotificationLanding              = 0x0E,
  HoTTv4NotificationGPSFix               = 0x0F,
  HoTTv4NotificationUndervoltage         = 0x10,
  HoTTv4NotificationGPSHold              = 0x11,
  HoTTv4NotificationGPSHome              = 0x12,
  HoTTv4NotificationGPSOff               = 0x13,
  HoTTv4NotificationBeep                 = 0x14,
  HoTTv4NotificationMicrocopter          = 0x15,
  HoTTv4NotificationCapacity             = 0x16,
  HoTTv4NotificationCareFreeOff          = 0x17,
  HoTTv4NotificationCalibrating          = 0x18,
  HoTTv4NotificationMaxRange             = 0x19,
  HoTTv4NotificationMaxAltitude          = 0x1A,
  
  HoTTv4Notification20Meter              = 0x25,
  HoTTv4NotificationMicrocopterOff       = 0x26,
  HoTTv4NotificationAltitudeOn           = 0x27,
  HoTTv4NotificationAltitudeOff          = 0x28,
  HoTTv4Notification100Meter             = 0x29,
  HoTTv4NotificationCareFreeOn           = 0x2E,
  HoTTv4NotificationDown                 = 0x2F,
  HoTTv4NotificationUp                   = 0x30,
  HoTTv4NotificationHold                 = 0x31,
  HoTTv4NotificationGPSOn                = 0x32,
  HoTTv4NotificationFollowing            = 0x33,
  HoTTv4NotificationStarting             = 0x34,
  HoTTv4NotificationReceiver             = 0x35,
} HoTTv4Notification;

/* * * * * * * * * * * * * 
 * EAM Message structure *
 * * * * * * * * * * * * */
 struct {
  uint8_t startByte;
  uint8_t eam_sensor_id;
  uint8_t alarmTone;       /* Alarm */
  uint8_t sensor_text_id;
  uint16_t alarmInverse;

  uint8_t cell1L;         /* Low Voltage Cell 1-7 in 2mV steps */
  uint8_t cell2L;
  uint8_t cell3L;
  uint8_t cell4L;
  uint8_t cell5L;
  uint8_t cell6L;
  uint8_t cell7L;
  uint8_t cell1H;         /* High Voltage Cell 1-7 in 2mV steps */
  uint8_t cell2H;  
  uint8_t cell3H;  
  uint8_t cell4H;
  uint8_t cell5H;
  uint8_t cell6H;
  uint8_t cell7H;
  uint16_t battery1;     /* Battery 1 LSB/MSB in 100mv steps; 50 == 5V */
  uint16_t battery2;     /* Battery 2 LSB/MSB in 100mv steps; 50 == 5V */
  uint8_t temp1;         /* Temp 1; Offset of 20. 20 == 0C */
  uint8_t temp2;         /* Temp 2; Offset of 20. 20 == 0C */
  uint16_t height;       /* Height. Offset -500. 500 == 0 */
  uint16_t current;      /* 1 = 0.1A */
  uint16_t driveVoltage; /* Main battery */ 
  uint16_t capacity;     /* mAh */
  uint16_t climbm2s;     /* climb rate in 0.01m/s; 120 == 0 */
  uint8_t climbm3s;      /* climb rate in m/3s; 120 == 0 WATCH the 255 (uint8) as max*/
  uint16_t rpm;          /* RPM. 10er steps; 300 == 3000rpm */
  uint8_t minutes;
  uint8_t seconds;
  uint16_t speed;        /* Air speed in km.h */    // moet dit niet een uint8_t zijn!!!!

  uint8_t endByte;
  uint8_t chksum;
} HoTTV4ElectricAirModule;

/* * * * * * * * * * * * * *
 * Text Message structure  *
 * * * * * * * * * * * * * */
struct {
  uint8_t startByte;
  uint8_t sensorTextID;
  uint8_t alarm;
  uint8_t text[8*21];
  uint8_t endByte;
  uint8_t chksum;
} HoTTv4ElectricalAirTextModule;

struct {
  uint8_t startByte;
  uint8_t sensorTextID;
  uint8_t alarm;
  uint8_t text[3*21];
  uint8_t endByte;
  uint8_t chksum;
} HoTTv4GPSTextModule;

/* * * * * * * * * * * * * 
 * GAM Message structure *
 * * * * * * * * * * * * */
struct {
  uint8_t startByte;
  uint8_t gam_sensor_id;
  uint8_t alarmTone;     /* Alarm */
  uint8_t sensor_text_id;
  uint16_t alarmInverse;

  uint8_t cell1;         /* Voltage Cell 1-7 in 2mV steps */
  uint8_t cell2;
  uint8_t cell3;
  uint8_t cell4;
  uint8_t cell5;
  uint8_t cell6;
  uint16_t battery1;     /* Battery 1 LSB/MSB in 100mv steps; 50 == 5V */
  uint16_t battery2;     /* Battery 2 LSB/MSB in 100mv steps; 50 == 5V */
  uint8_t temp1;         /* Temp 1; Offset of 20. 20 == 0C */
  uint8_t temp2;         /* Temp 2; Offset of 20. 20 == 0C */
  uint8_t fuel_procent;  /* Fuel capacity in %. Values 0--100 graphical display ranges: 0-25% 50% 75% 100% */
  uint16_t fuel_ml;      /* Fuel in ml scale. Full = 65535! */
  uint16_t rpm;          /* RPM. 10er steps; 300 == 3000rpm */
  uint16_t height;       /* altitude in meters. offset of 500, 500 = 0m */
  uint16_t climbm2s;     /* climb rate in 0.01m/s; 120 == 0 */
  uint8_t climbm3s;      /* climb rate in m/3s; 120 == 0 */
  uint16_t current;      /* 1 = 0.1A */
  uint16_t driveVoltage; /* Main battery */ 
  uint16_t capacity;     /* mAh */
  uint16_t speed;        /* Air speed in km.h */
  uint8_t min_cell_volt; /* minimum cell voltage in 2mV steps. 124 = 2,48V */
  uint8_t min_cell_volt_num; /* #number of the cell with the lowest voltage */
  uint16_t rpm2;         /* RPM2. 10er steps; 300 == 3000rpm */  
  uint8_t general_error_number;    /* Voice error == 12. TODO: more docu */
  uint8_t pressure;      /* Pressure up to 16bar. 0,1bar scale. 20 = 2bar*/

  uint8_t version;
  uint8_t endByte;
  uint8_t chksum;
} HoTTV4GeneralAirModule;


/* * * * * * * * * * * * * 
 * GPS Message structure *
 * * * * * * * * * * * * */
struct {
  uint8_t startByte;
  uint8_t gps_sensor_id;
  uint8_t alarmTone;     /* Alarm */
  uint8_t sensor_text_id;
  uint16_t alarmInverse;

  uint8_t flight_direction; /* flight direction in 2 degrees/step (1 = 2degrees); */
  uint16_t gps_speed;    /* km/h*/
  int8_t pos_NS;         /* north = 0, south = 1 */
  int16_t pos_NS_dm;     /* degree minutes ie N48°39 988 */
  int16_t pos_NS_sec;    /* position seconds */
  int8_t pos_EW;         /* east = 0, west = 1 */
  int16_t pos_EW_dm;     /* degree minutes ie E4°39 988 */
  int16_t pos_EW_sec;    /* position seconds */
  uint16_t home_distance;/* distance to home */  
  uint16_t height;       /* altitude in meters. offset of 500, 500 = 0m */
  uint16_t climbm2s;     /* climb rate in 0.01m/s; 120= 0 */
  uint8_t climbm3s;      /* climb rate in m/3s; 120 = 0 */
  uint8_t gps_satelites; /* sat count */
  uint8_t free_char3;    /* where does this char appear ???*/
  uint8_t home_direction;/* direction from starting point to Model position (2 degree steps) */
  uint8_t angle_roll;    /* angle roll in 2 degree steps */
  uint8_t angle_pitch;   /* angle pitch in 2 degree steps */  
  uint8_t angle_heading; /* angle heading in 2 degree steps */  
  uint8_t gps_time_h;    /* UTC time hours */
  uint8_t gps_time_m;    /* UTC time minutes */
  uint8_t gps_time_s;    /* UTC time seconds */
  uint8_t gps_time_sss;  /* UTC time miliseconds */  
  uint16_t msl_height;   /* mean see level altitude in meters.*/
  uint8_t vibration;     /* vibration level in % ???? */
  uint8_t free_char1;    /* appears right to home distance */
  uint8_t free_char2;    /* appears right to home distance */
  uint8_t gps_fix_char;  /* GPS ASCII D=DGPS 2=2D 3=3D -=No Fix */  

  uint8_t version;       
  uint8_t endByte;
  uint8_t chksum;
} HoTTV4GPSModule;

/* * * * * * * * * * * * * 
 * Vario Message structure *
 * * * * * * * * * * * * */
struct {
  uint8_t startByte;
  uint8_t vario_sensor_id;
  uint8_t alarmTone;     /* Alarm */
  uint8_t sensor_text_id;
  uint8_t alarmInverse;

  uint16_t height;       /* altitude in meters. offset of 500, 500 = 0m */
  uint16_t height_max;   /* altitude in meters. offset of 500, 500 = 0m */
  uint16_t height_min;   /* altitude in meters. offset of 500, 500 = 0m */
  uint16_t climbm2s;     /* climb rate in 0.01m/s; 120= 0 */
  uint16_t Vclimbm3s;    /* climb rate in 0.01m/3s; 30000= 0 */
  uint16_t Vclimbm10s;   /* climb rate in 0.01m/10s; 30000 = 0 */
  uint8_t text_msg[VARIO_ASCIIS];  // text index, should be 21 characters
  uint8_t free_char1;    /* appears right to home distance */
  uint8_t free_char2;    /* appears right to home distance */
  uint8_t gps_fix_char;  /* GPS ASCII D=DGPS 2=2D 3=3D -=No Fix */  
  uint8_t flight_direction; /*angle heading in 2 degree steps */  

  uint8_t version;       
  uint8_t endByte;
  uint8_t chksum;
} HoTTV4VarioModule;




#endif



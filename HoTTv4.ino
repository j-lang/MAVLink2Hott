#ifdef HoTT
#include "HoTTv4.h"

/* Common setup method for HoTTv4 */
void hottV4Setup() {
  hottV4Serial.begin(19200);
  hottV4EnableReceiverMode();
}

/* Enables RX and disables TX */
static inline void hottV4EnableReceiverMode() {
#if defined (BOARD_MICRO)              /* the arduino MICRO has changed a lot on the hardware, port 9 (PB05) is working */  
  DDRB &= ~(1 << 5);
  PORTB |= (1 << 5);
#else
//  DDRD &= ~(1 << hTX);                 /* direct portmanipulation, remark: PORTD is for digital port 0-7 only */
//  PORTD |= (1 << hTX);
  DDRB &= ~(1 << 1);                    /* Special CAN/Mav/Led distroboard.. ToDo: clean up */
  PORTB |= (1 << 1);
#endif
}

/* Enables TX and disables RX  */
static inline void hottV4EnableTransmitterMode() {
#if defined (BOARD_MICRO)              /* the arduino MICRO has changed a lot on the hardware, port 9 (PB05) is working */    
  DDRB |= (1 << 5);
#else
//  DDRD |= (1 << hTX);
  DDRB |= (1 << 1);                    /* Special CAN/Mav/Led distroboard.. ToDo: clean up */
#endif  
}

/* Writes out given byte to HoTT serial interface. */
static void hottV4SerialWrite(uint8_t c) {
  hottV4Serial.write(c);
}

void hottV4SendTelemetry() {
 /**
 * Read the HoTTv4 request from the transmitter and send HoTTv4 capable data frames according to the
 * requested module.
 */

  if ((hottV4Serial.available() > 1) && (hottV4_state == IDLE)) {
    uint8_t c = hottV4Serial.read();
      switch (c) {
        case HOTTV4_ELECTRICAL_AIR_MODULE:
          hottV4_state = BINARY;
          delayMicroseconds(HOTTV4_SENDDELAY); // protocol specific wait time before sending
          hottV4SendEAM();
          hottV4_state = IDLE;
          break;
        case HOTTV4_GENERAL_AIR_MODULE:
          hottV4_state = BINARY;
          delayMicroseconds(HOTTV4_SENDDELAY); // protocol specific wait time before sending
          hottV4SendGAM();
          hottV4_state = IDLE;
          break;
        case HOTTV4_GPS_MODULE:
          hottV4_state = BINARY;
          delayMicroseconds(HOTTV4_SENDDELAY); // protocol specific wait time before sending
          hottV4SendGPS();
          hottV4_state = IDLE;
          break;
        case HOTTV4_VARIO_MODULE:
          hottV4_state = BINARY;
          delayMicroseconds(HOTTV4_SENDDELAY); // protocol specific wait time before sending
          hottV4SendVario();
          hottV4_state = IDLE;
          break;
        default:
          hottV4_state = IDLE;
      } // end switch 
   }
}
/* Protocol decoding and frame transmitting from here on.. */
 
/**
 * Expects an array of at least size bytes. All bytes till size will be transmitted
 * to the HoTT capable receiver. Last byte will always be treated as checksum and is
 * calculated on the fly.
 */
static void hottV4SendData(uint8_t *data, uint8_t size) {
  hottV4Serial.flush();
  hottV4_state = SENDING;
  if (hottV4Serial.available() == 0) {
    hottV4EnableTransmitterMode();
 
    uint16_t crc = 0;

    for (uint8_t i = 0; i < (size - 1); i++) {
      crc += data[i];     
      hottV4SerialWrite(data[i]);
      delayMicroseconds(HOTTV4_TX_DELAY);     // Protocol specific delay between each transmitted byte
    }
    
    // Write package checksum
    hottV4SerialWrite(crc & 0xFF);
  }
  hottV4EnableReceiverMode();  
  hottV4_state = IDLE;                      // Transmit done, ready to receive
}

/**
 * Sends HoTTv4 capable EAM (Electric Air Module) telemetry frame.
 */
static void hottV4SendEAM() {
  /** Minimum data set for EAM */
  HoTTV4ElectricAirModule.startByte      = 0x7C;
  HoTTV4ElectricAirModule.eam_sensor_id  = HOTTV4_ELECTRICAL_AIR_MODULE;
  HoTTV4ElectricAirModule.sensor_text_id = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTV4ElectricAirModule.endByte        = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4ElectricAirModule.alarmTone    = 0x0;
  HoTTV4ElectricAirModule.alarmInverse = 0x0;
  //(int &)
  HoTTV4ElectricAirModule.battery1     = apm_voltage_battery;           /* bat1 gauge */
  HoTTV4ElectricAirModule.driveVoltage = apm_voltage_battery;           /* main batt icon */
  HoTTV4ElectricAirModule.temp1        = 20 + (apm_temp/1E2);  /* pressure sensor temp from AutoQuad */
  HoTTV4ElectricAirModule.temp2        = 20;                   /* nothing yet */

 #ifdef curSens  
  HoTTV4ElectricAirModule.current      = Value_mA / 100;       /* 1 = 0.1A, this arrives in mA */
  HoTTV4ElectricAirModule.capacity     = current_total /10; 
 #endif
 
  HoTTV4ElectricAirModule.height       = OFFSET_HEIGHT + (apm_gps_altitude/1E3);
 // functions.ino // HoTTV4ElectricAirModule.climbm2s     = 120 + (alt_diff_1s/1E1);
 // HoTTV4ElectricAirModule.climbm3s     = 120 + (alt_diff_3s/1E3);

 // functions.ino // HoTTV4ElectricAirModule.minutes      = minutes;
 // functions.ino // HoTTV4ElectricAirModule.seconds      = seconds;

  /* low voltage alarm: battery voltage / number of cells not lower then LOW_BATT value per cell. note: battery voltage is x10 hense the LOW_BATT * 10 */
  if ( (apm_voltage_battery / apm_cell_count) <= (LOW_BATT * 10) ) {    /* can also be settable on fixed by using: HoTTModuleSettings.alarmDriveVoltage */
    HoTTV4ElectricAirModule.alarmTone  = HoTTv4NotificationUndervoltage;  
    HoTTV4ElectricAirModule.alarmInverse |= 0x80;            /* Invert Voltage display to indicate alarm */
  } 
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Sends HoTTv4 capable GAM (General Air Module) telemetry frame.
 */
static void hottV4SendGAM() {
  /** Minimum data set for GAM */
  HoTTV4GeneralAirModule.startByte      = 0x7C;
  HoTTV4GeneralAirModule.gam_sensor_id  = HOTTV4_GENERAL_AIR_MODULE;
  HoTTV4GeneralAirModule.sensor_text_id = HOTTV4_GENERAL_AIR_SENSOR_TEXT_ID;
  HoTTV4GeneralAirModule.endByte        = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4GeneralAirModule.alarmTone    = 0x0;
  HoTTV4GeneralAirModule.alarmInverse = 0x0;

  HoTTV4GeneralAirModule.battery1     = apm_voltage_battery;
  HoTTV4GeneralAirModule.driveVoltage = apm_voltage_battery;
  HoTTV4GeneralAirModule.temp1        = 20 + (apm_temp/1E2);      /* pressure sensor temp from AutoQuad */
  HoTTV4GeneralAirModule.temp2        = 20;                       /* nothing yet */
  HoTTV4GeneralAirModule.min_cell_volt_num = apm_cell_count;       /* nothing yet, for the moment the number of detected cells in the battery */

 #ifdef curSens  // Already send by Electric Air module, enable this if for any reason the EAM is disabled 
  //HoTTV4GeneralAirModule.current = Value_mA / 100;     /* 1 = 0.1A, this arrives in mA */
  //HoTTV4GeneralAirModule.capacity = current_total /10;
 #endif
  
  HoTTV4GeneralAirModule.height       = OFFSET_HEIGHT + ( apm_gps_altitude/1E3);
// functions.ino //   HoTTV4GeneralAirModule.climbm2s     = 120 + (alt_diff_1s/1E1); 
// functions.ino //   HoTTV4GeneralAirModule.climbm3s     = 120 + (alt_diff_3s/1E3);
// functions.ino //  HoTTV4GeneralAirModule.fuel_procent = apm_fuel_procent;
  
  HoTTV4GeneralAirModule.pressure     = apm_abs/1E2;

  /* low voltage alarm: battery voltage / number of cells not lower then LOW_BATT value per cell. note: battery voltage is x10 hense the LOW_BATT * 10 */
  if ( (apm_voltage_battery / apm_cell_count) <= (LOW_BATT * 10) ) {    /* can also be settable on fixed by using: HoTTModuleSettings.alarmDriveVoltage */
    HoTTV4GeneralAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4GeneralAirModule.alarmInverse |= 0x80;              /* Invert Voltage display to indicate alarm */
  } 
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy GAM data to output buffer
  memcpy(&outBuffer, &HoTTV4GeneralAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Sends HoTTv4 capable GPS telemetry frame.
 */
static void hottV4SendGPS() {
  /** Minimum data set for GPS */
  HoTTV4GPSModule.startByte      = 0x7C;
  HoTTV4GPSModule.gps_sensor_id  = HOTTV4_GPS_MODULE;
  HoTTV4GPSModule.sensor_text_id = HOTTV4_GPS_SENSOR_ID;
  HoTTV4GPSModule.endByte        = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4GPSModule.alarmTone            = 0x0;
  HoTTV4GPSModule.alarmInverse         = 0x0;

  HoTTV4GPSModule.flight_direction     = ((uint16_t)(apm_yaw*100)/100) >>1; /* in 2* steps HoTTModule.flight_direction, arrives as float *100/100 to loose the floating point and convert to int */
  HoTTV4GPSModule.height               = OFFSET_HEIGHT + (apm_gps_altitude/1E3);
  HoTTV4GPSModule.msl_height           = msl_height;
  // functions.ino //HoTTV4GPSModule.climbm2s             = 120 + (alt_diff_1s/1E1); 
 // functions.ino // HoTTV4GPSModule.climbm3s             = 120 + (alt_diff_3s/1E3);
  
  HoTTV4GPSModule.gps_fix_char         = 48+apm_fixtype;          /* ASCII 48=0 fixtype can be 0,2(d) or 3(d) */
  HoTTV4GPSModule.gps_satelites        = apm_sat_visible;

  convertLat (apm_latitude);                                      /* coordinates arrives from mavlink *1e7, convert in dec min & dec sec */
  HoTTV4GPSModule.pos_NS               = pos_NS;                   /* calculated by convertLat / convertLon */
  HoTTV4GPSModule.pos_NS_dm            = degMin;
  HoTTV4GPSModule.pos_NS_sec           = deg_sec;

  convertLon (apm_longitude);
  HoTTV4GPSModule.pos_EW               = pos_EW;
  HoTTV4GPSModule.pos_EW_dm            = degMin;
  HoTTV4GPSModule.pos_EW_sec           = deg_sec;

  
 // functions.ino // HoTTV4GPSModule.home_distance        = home_distance_calc;
  HoTTV4GPSModule.gps_speed            = apm_gps_vel * 3.6;        /* gps speed in m/s, convert to km/h */

  HoTTV4GPSModule.angle_roll           = 0;                        /* ?? where on the screens ?? */
  HoTTV4GPSModule.angle_heading        = 0;                        /* ?? where on the screens ?? */
// functions.ino  HoTTV4GPSModule.home_direction       = ((uint16_t)(bearing*100)/100) >>1;           /* bearing angle from MAV position to home position in 2 deg steps */
  HoTTV4GPSModule.vibration            = 0;                        /* ?? where on the screens ?? */  
  
  HoTTV4GPSModule.free_char1           = free_char1;
  HoTTV4GPSModule.free_char2           = free_char2;
  
  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy GPS data to output buffer
  memcpy(&outBuffer, &HoTTV4GPSModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Sends HoTTv4 capable GPS telemetry frame.
 */
static void hottV4SendVario() {
  /** Minimum data set for Vario */
  
  HoTTV4VarioModule.startByte          = 0x7C;
  HoTTV4VarioModule.vario_sensor_id    = HOTTV4_VARIO_MODULE;
  HoTTV4VarioModule.sensor_text_id     = HOTTV4_VARIO_SENSOR_ID;
  HoTTV4VarioModule.version            = 0xFF;
  HoTTV4VarioModule.endByte            = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4VarioModule.alarmTone          = 0x0;
  HoTTV4VarioModule.alarmInverse       = 0x0;

/* test data 
  alt_diff_1s = 3200;  // is 3.2m/s 
  alt_diff_3s = 6500;  // is 6.5m/3s
  alt_diff_10s = 14000; // is 14 m/10s
*/

// functions.ino  HoTTV4VarioModule.climbm2s           = OFFSET_VARIO + (alt_diff_1s/1E1);           /* in cm/s */
// functions.ino  HoTTV4VarioModule.Vclimbm3s          = OFFSET_VARIO + (alt_diff_3s/1E1);           /* in cm/3s */
// functions.ino  HoTTV4VarioModule.Vclimbm10s         = OFFSET_VARIO + (alt_diff_10s/1E1);          /* in cm/10s */
  HoTTV4VarioModule.height             = OFFSET_HEIGHT + ( apm_gps_altitude/1E3);
  HoTTV4VarioModule.height_min         = OFFSET_HEIGHT;
// functions.ino //  HoTTV4VarioModule.height_max         = OFFSET_HEIGHT + height_max;                 /* either Ceiling or maximum reached altitude (GPS Altitude) */
  if ((apm_ceiling) && (apm_fixtype == 3) && ( apm_gps_altitude/1E3 > apm_ceiling)){
      HoTTV4VarioModule.alarmInverse |= 0x80;                                        /* Invert Altitude display to indicate ceiling reached */
  }
  HoTTV4VarioModule.free_char1         = mavlinkHB_char;                             /* Mavlink HeartBeat char next to Alt */

  HoTTV4VarioModule.flight_direction   =  ((uint16_t)(apm_yaw*100)/100) >>1;         /* in 2* steps HoTTModule.flight_direction, arrives as float *100/100 to loose the floating point and convert to int */

  /* Text message on bottom line in GPS, Vario, General Air module screen */
  strcpy_P(text, (char*) flight_modes[flMode]);                                    /*retrieve correct flightMode text from mapping table and PROGMEM space */
  /* First, break down the message (max 21 char) and place the seperate characters into the index */
  uint8_t offset = (VARIO_ASCIIS - strlen(text)) / 2;
  for(uint8_t index = 0; (index + offset) < VARIO_ASCIIS; index++) {
    if (text[index] != 0x0) {
      HoTTV4VarioModule.text_msg[index+offset] = text[index];
    } else {
      break;
    }
  }  
 
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy Vario data to output buffer
  memcpy(&outBuffer, &HoTTV4VarioModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}  








#endif

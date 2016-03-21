
/*
APM2.5 Mavlink to Graupner HoTT interface using Arduino/Genuino mini/micro or compatible board

******************************************************
Cut board on the backside to separate Vin from VUSB

Connection on Arduino:
HoTT Telemetry T --> 5
HoTT Telemetry T --> 9 Arduino micro 
HoTT Telemetry - --> GND



APM Telemetry DF13-6  Pin 1 --> VCC
APM Telemetry DF13-6  Pin 2 --> RX
APM Telemetry DF13-6  Pin 6 --> GND

This is the data we send to HoTT, you can change this to have your own
set of data
******************************************************
Data transmitted to FrSky Taranis:
Cell           ( Voltage of Cell=Cells/4. [V] This is my LiPo pack 4S ) 
Cells         ( Voltage from LiPo [V] )
A2             ( Analog voltage from input A0 on Teensy 3.1 )
Alt             ( Altitude from baro.  [m] )
GAlt          ( Altitude from GPS   [m])
HDG         ( Compass heading  [deg])
Rpm         ( Throttle when ARMED [%] )
AccX         ( AccX m/s ? )
AccY         ( AccY m/s ? )
AccZ         ( AccZ m/s ? )
VSpd        ( Vertical speed [m/s] )
Speed      ( Ground speed from GPS,  [km/h] )
T1            ( GPS status = apm_sat_visible*10) + apm_fixtype )
T2            ( ARMED=1, DISARMED=0 )
Vfas          ( same as Cells )
Longitud    
Latitud
Dist          ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position

******************************************************

*/
/* Communication defines */
#define HEARTBEAT         // Option : HeartBeat signal
#define HoTT              // Option: graupner HoTT port, cannot be used with SerialDebug at the same time except on a Pro MICRO!
#define HOTTFAST          // Option: HOTTFAST or HOTTNORM, FAST if both the transmitter & receiver are upgraded with FW > oct 2013
#define SERDB             // Option: Output debug mavlink information to SoftwareSerial, cannot be used with HoTT output at the same time except on a Pro MICRO!



/* try to determine board and MCU version in compiler settings */
#if defined(__AVR_ATmega328P__)
  #undef BOARD_MICRO
  #ifdef HoTT
    #undef SERDB
  #endif
#elif defined(__AVR_ATmega32U4__)
  #define BOARD_MICRO
#endif

#include <GCS_MAVLink.h>
#include <HoTTv4.h>
#include <SoftwareSerial.h>
#include "MAVLink2HoTT.h"

#if defined (BOARD_MICRO)              /* the arduino MICRO can use the USB connector for the serial monitor */
  #define debugSerial         Serial
  #define _MavLinkSerial      Serial1 
#else
  #define _MavLinkSerial      Serial
#endif    




// ******************************************
// These are special for FrSky
int32_t   adc2 = 0;               // 100 = 1.0V
int32_t     vfas = 0;                // 100 = 1,0V
int32_t     gps_status = 0;     // (apm_sat_visible * 10) + apm_fixtype
                                             // ex. 83 = 8 sattelites visible, 3D lock 
uint8_t   apm_cell_count = 0;

// ******************************************
uint8_t     MavLink_Connected;
uint8_t     buf[MAVLINK_MAX_PACKET_LEN];

uint16_t  hb_count;
int16_t msl_height = 0;

#define LOW_BATT   3.35      /* low battery per cell for HoTT display */
#define LOW_BATT_2 3.2       /* low battery per cell for LED warning */
#define packCapacity      5000          // default mAh capacity

unsigned long MavLink_Connected_timer;
unsigned long hb_timer;
unsigned long acc_timer;

int led = 13;

mavlink_message_t msg;

#ifdef BOARD_MICRO
  #define hbLed 17                       /* micro does not have a port 13 led, but the RX led is tied to 17 */
  #define MAVCOM MAVLINK_COMM_1
#endif
#ifndef BOARD_MICRO
  #define MAVCOM MAVLINK_COMM_0
#endif
  
static uint8_t hRX=6;                    /* software serial port for HoTT OR! Debug */
static uint8_t hTX=5;                    /* if using the JDrones board use 6 & 5 */


#ifdef SERDB
  #if defined (BOARD_MICRO)              /* the arduino MICRO can use the USB connector for the serial monitor */
    #define DPL Serial.println
    #define DPN Serial.print
  #else
    SoftwareSerial dbSerial(hRX,hTX);    /* (rx port,tx port) */
    #define DPL dbSerial.println 
    #define DPN dbSerial.print
  #endif    
#endif

#ifdef HoTT
  #if defined (BOARD_MICRO)              /* the arduino MICRO has changed a lot on the hardware, port 9 (PB05) is working */
    SoftwareSerial hottV4Serial(9,9);    /* (rx_tx port,rx_tx port). HoTT uses 1 wire for both RX/TX . */
  #else
    SoftwareSerial hottV4Serial(hTX,hTX);/* (rx_tx port,rx_tx port). HoTT uses 1 wire for both RX/TX . */
  #endif
#endif  

// ******************************************
void setup()  {
  

  _MavLinkSerial.begin(57600);
#ifdef SERDB 
  debugSerial.begin(57600);
#endif

  MavLink_Connected = 0;
  MavLink_Connected_timer=millis();
  hb_timer = millis();
  acc_timer=millis();
  hb_count = 0;

  
  pinMode(led,OUTPUT);
  pinMode(12,OUTPUT);
  
  pinMode(14,INPUT);
  analogReference(DEFAULT);
  
  #ifdef HoTT
    hottV4Setup();
  #endif  
  
}


// ******************************************
void loop()  {
  uint16_t len;
  
    if ((millis()-tEvent) >= 100) {  // non interrupt based 10Hz timer
      timerEvent();
      tEvent= millis(); // reset timer
    } 
 
    if(millis()-hb_timer > 1500) {
        hb_timer=millis();
        if(!MavLink_Connected) {    // Start requesting data streams from MavLink
            digitalWrite(led,HIGH);
             mavlink_msg_request_data_stream_pack(0xFF,0xBE,&msg,1,1,MAV_DATA_STREAM_EXTENDED_STATUS, MSG_RATE, START);
             len = mavlink_msg_to_send_buffer(buf, &msg);
             _MavLinkSerial.write(buf,len);
            delay(10);
           mavlink_msg_request_data_stream_pack(0xFF,0xBE,&msg,1,1,MAV_DATA_STREAM_EXTRA2, MSG_RATE, START);
           len = mavlink_msg_to_send_buffer(buf, &msg);
           _MavLinkSerial.write(buf,len);
             delay(10);
           mavlink_msg_request_data_stream_pack(0xFF,0xBE,&msg,1,1,MAV_DATA_STREAM_RAW_SENSORS, MSG_RATE, START);
           len = mavlink_msg_to_send_buffer(buf, &msg);
           _MavLinkSerial.write(buf,len);
          digitalWrite(led,LOW);
            }
        }
 
  if((millis() - MavLink_Connected_timer) > 1500)  {   // if no HEARTBEAT from APM  in 1.5s then we are not connected
      MavLink_Connected=0;
      hb_count = 0;
      } 
  
  _MavLink_receive();                   // Check MavLink communication

  #ifdef HoTT  
    hottV4SendTelemetry();
  #endif  // HoTT  
  //FrSkySPort_Process();               // Check FrSky S.Port communication
   
  adc2 =analogRead(0)/4;               // Read ananog value from A0 (Pin 14). ( Will be A2 value on FrSky LCD)
  
  if((millis() - acc_timer) > 1000) {    // Reset timer for AccX, AccY and AccZ
    apm_accX_old=apm_accX;
    apm_accY_old=apm_accY;
    apm_accZ_old=apm_accZ;
    acc_timer=millis();
    //debugSerial.println(apm_base_mode);
    }
  
} // end loop


void _MavLink_receive() { 
  mavlink_message_t msg;
  mavlink_status_t status;

	while(_MavLinkSerial.available()) 
                { 
		uint8_t c = _MavLinkSerial.read();
                if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
                      {
		       switch(msg.msgid)
			    {
			     case MAVLINK_MSG_ID_HEARTBEAT:  // 0
                                apm_mav_type    = mavlink_msg_heartbeat_get_type(&msg);
                                apm_base_mode   = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
                                apm_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                                apm_mav_mode = apm_base_mode || apm_custom_mode;
                                MavLink_Connected_timer=millis(); 
                                if(!MavLink_Connected); {
                                    hb_count++;   
                                    if((hb_count++) > 10) {        // If  received > 10 heartbeats from MavLink then we are connected
                                        MavLink_Connected=1;
                                        hb_count=0;
                                        digitalWrite(led,HIGH);      // LED will be ON when connected to MavLink, else it will slowly blink
                                        }
                                     }
                                 if(isBit(mavlink_msg_heartbeat_get_base_mode(&msg),MOTORS_ARMED)) {
                                    if(isArmedOld == 0) {
                                        isArmedOld = 1;
                                    }    
                                    isArmed = 1;  
                                  } else {
                                    isArmed = 0;
                                    isArmedOld = 0;
                                  }            
                                  CheckFlightMode();    
                                 break;
                              case MAVLINK_MSG_ID_SYS_STATUS :   // 1
                                  apm_voltage_battery = Get_Volt_Average(mavlink_msg_sys_status_get_voltage_battery(&msg));  // 1 = 1mV
                                  apm_current_battery = Get_Current_Average(mavlink_msg_sys_status_get_current_battery(&msg));     // 1=10mA

                                  if(apm_voltage_battery > 21000) apm_cell_count = 6;
                                  else if (apm_voltage_battery > 16800 && apm_cell_count != 6) apm_cell_count = 5;
                                  else if(apm_voltage_battery > 12600 && apm_cell_count != 5) apm_cell_count = 4;
                                  else if(apm_voltage_battery > 8400 && apm_cell_count != 4) apm_cell_count = 3;
                                  else if(apm_voltage_battery > 4200 && apm_cell_count != 3) apm_cell_count = 2;
                                  else apm_cell_count = 0;
                                  break;
                              case MAVLINK_MSG_ID_SCALED_PRESSURE:
                                   apm_abs = mavlink_msg_scaled_pressure_get_press_abs(&msg);                  // absolute presure          
                                   apm_temp = mavlink_msg_scaled_pressure_get_temperature(&msg);               // internal pressure sensor temp
                                  break;
                              case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
                                 apm_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
                                 apm_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
                                 gps_status = (apm_sat_visible*10) + apm_fixtype; 
                                 if(apm_fixtype == 3)  {
                                     apm_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
                                     apm_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
                                     apm_gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);    // 1m =1000
                                     apm_gps_vel = (mavlink_msg_gps_raw_int_get_vel(&msg) / 100.0f);            // speed in m/s
                                     }
                                 break;
                              case MAVLINK_MSG_ID_ATTITUDE:
                                 apm_yaw = RAD_TO_DEG *(mavlink_msg_attitude_get_yaw(&msg));
                                 break;
                              case MAVLINK_MSG_ID_RAW_IMU:   // 27
                                 apm_accX = mavlink_msg_raw_imu_get_xacc(&msg) / 10;                // 
                                 apm_accY = mavlink_msg_raw_imu_get_yacc(&msg) / 10;
                                 apm_accZ = mavlink_msg_raw_imu_get_zacc(&msg) / 10;
                                 break;      
                              case MAVLINK_MSG_ID_VFR_HUD:   //  74
                                  apm_airspeed = 0;
                                  apm_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);      // 100 = 1m/s
                                  apm_heading = mavlink_msg_vfr_hud_get_heading(&msg);     // 100 = 100 deg
                                  apm_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);        //  100 = 100%
                                  apm_bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg) * 100;        //  m
                                  apm_climb_rate=mavlink_msg_vfr_hud_get_climb(&msg) * 100;        //  m/s
                                  break; 
                              case MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA:
                                  apm_ceiling = mavlink_msg_safety_allowed_area_get_p1z(&msg);            // maximum allowed altitude if set
                                  break;
                              default:
				  break;
			    }

		      }
                }
}




/* This file contains conversion & calculation functions for values used for the HoTT protocol */

unsigned long bootTime = millis();                 /* timer since start of program */


void timerEvent() {  /* this event is called @ 10Hz */
  eventActive=1;
  ioCounter ++ ;     /* update counter to use in several loops */
  if (ioCounter > 9) ioCounter =0;

/* 10Hz calls */
//  checkCurrent();   /* calculate current (if defined) */
  
  eventActive=0;

/* end 10Hz calls */
if (ioCounter == 1){
/* 1Hz routines between this line */
  eventActive=1;
  CheckFlightMode(); 
  //?checkBattery();
  //?updateFlightTimer();
  //?checkAltitude();
  //?checkHomeDistance();
  //?varioCalc();
  #ifdef SERDB
    printDebugInfo();
  #endif  
/* and this line */
eventActive=0;
}

/* check if Home coordinates are set every 200 cycles = 20 sec*/
#ifndef useMavHome
  counter ++;
  if ((counter > 200)&& (!m2h_got_home)) {
      //? checkHomeSet();
      counter =0 ;
  }
#endif

/* Display version on display for 5 sec*/
  if (!displayVersionDone){
    flMode = SIG_EVENT_INIT;
    if (millis() - bootTime >5000) {
      displayVersionDone = 1;
      tNow = millis();       // reset the timer
    }
  }
} /* end timerEvent() */


uint16_t Volt_AverageBuffer[10]; 
uint16_t Current_AverageBuffer[10]; 
  
//returns the average of Voltage for the 10 last values  
uint32_t Get_Volt_Average(uint16_t value)  {
      uint8_t i;
      uint32_t sum=0;
      
      for(i=9;i>0;i--)  {
          Volt_AverageBuffer[i]=Volt_AverageBuffer[i-1];
          sum+=Volt_AverageBuffer[i];
          }
      Volt_AverageBuffer[0]=value;    
      return (sum+=value)/10;
  }
  
  //returns the average of Current for the 10 last values  
uint32_t Get_Current_Average(uint16_t value)  {
      uint8_t i;
      uint32_t sum=0;
      
      for(i=9;i>0;i--)  {
          Current_AverageBuffer[i]=Current_AverageBuffer[i-1];
          sum+=Current_AverageBuffer[i];
          }
      Current_AverageBuffer[0]=value;    
      return (sum+=value)/10;
  }
  
  
// Checking if BIT is active in PARAM, return true if it is, false if not
byte isBit(byte param, byte bitfield) {
 if((param & bitfield) == bitfield) return 1;
  else return 0;  
}  



void CheckFlightMode() {
     if(apm_mav_type == MAV_TYPE_QUADROTOR || apm_mav_type == MAV_TYPE_HELICOPTER || apm_mav_type == MAV_TYPE_COAXIAL || apm_mav_type == MAV_TYPE_HEXAROTOR 
     || apm_mav_type == MAV_TYPE_OCTOROTOR || apm_mav_type == MAV_TYPE_TRICOPTER) { // ArduCopter MultiRotor or ArduCopter Heli
       if(apm_custom_mode == 0 ) flMode = STAB;   // Stabilize
       if(apm_custom_mode == 1 ) flMode = ACRO;   // Acrobatic
       if(apm_custom_mode == 2 ) flMode = ALTH;   // Alt Hold
       if(apm_custom_mode == 3 ) flMode = AUTO;   // Auto
       if(apm_custom_mode == 4 ) flMode = GUID;   // Guided
       if(apm_custom_mode == 5 ) flMode = LOIT;   // Loiter
       if(apm_custom_mode == 6 ) flMode = RETL;   // Return to Launch
       if(apm_custom_mode == 7 ) flMode = CIRC;   // Circle
       if(apm_custom_mode == 9 ) flMode = LAND;   // Land
       if(apm_custom_mode == 10) flMode = OFLO;   // OF_Loiter
       if(apm_custom_mode == 11) flMode = DRIF;   // Drift
       if(apm_custom_mode == 13) flMode = SPOR;   // Sport
       if(apm_custom_mode == 14) flMode = FLIP;   // Flip
       if(apm_custom_mode == 15) flMode = ATUN;   // Auto Tune
       if(apm_custom_mode == 16) flMode = POSI;   // Position Hold
       if(apm_custom_mode == 17) flMode = BREA;   // Break
       if(apm_custom_mode == 18) flMode = THRO;   // Thrower
     } else if(apm_mav_type == MAV_TYPE_FIXED_WING) { // ArduPlane
       if(apm_custom_mode == 0 ) flMode = MANU;   // Manual
       if(apm_custom_mode == 1 ) flMode = CIRC;   // Circle
       if(apm_custom_mode == 2 ) flMode = STAB;   // Stabilize
       if(apm_custom_mode == 3 ) flMode = TRAI;   // Stabilize
       if(apm_custom_mode == 4 ) flMode = ACRO;   // Acrobatic
       if(apm_custom_mode == 5 ) flMode = FBWA;   // FLY_BY_WIRE_A
       if(apm_custom_mode == 6 ) flMode = FBWB;   // FLY_BY_WIRE_B
       if(apm_custom_mode == 7 ) flMode = CRUI;   // Cruise
       if(apm_custom_mode == 8 ) flMode = ATUN;   // Auto Tune
       if(apm_custom_mode == 11) flMode = RETL;   // Return to Launch
       if(apm_custom_mode == 10) flMode = AUTO;   // AUTO
       if(apm_custom_mode == 12) flMode = LOIT;   // Loiter
       if(apm_custom_mode == 15) flMode = GUID;   // GUIDED
     }
     if (isArmed ==0){
       flMode = flMode + 24;   // Select disarmed text
     }
}
/* Position functions 
 * Mavlink transmits the coordinates as coordinate * 1E7
 * NOTE: HoTT displays the coordinates as degrees - minutes and seconds 
 */
void convertLat (int32_t in_coords){
  if(in_coords >= 0) {
    pos_NS = 0;
  } else {
    pos_NS = 1;
  }
  lat = (in_coords) / 10000000.0f;
  deg = int (lat) ;        // 1st part: convert to int so only the part before the floating point remains as degree
  sec = (lat - deg); 
  min = int (sec * 60);    // 2nd part, take the int from the remaining part multiplied by 60 to convert to min
  deg_sec = abs(int(((sec * 60) - min) * 10000));  //HoTT does not allow the negative sign in case of the Southern hemisphere
  degMin = abs(int(deg * 100 + min));
} /* end convertLat */

void convertLon (int32_t in_coords){
    if(in_coords >= 0) {
    pos_EW = 0;
  } else {
    pos_EW = 1;
  }
  lon = (in_coords) / 10000000.0f;
  deg = int (lon) ;        // 1st part: convert to int so only the part before the floating point remains as degree
  sec = (lon - deg); 
  min = int (sec * 60);    // 2nd part, take the int from the remaining part multiplied by 60 to convert to min
  deg_sec = abs(int(((sec * 60) - min) * 10000));  //HoTT does not allow the negative sign in case of the Southern hemisphere
  degMin = abs(int(deg * 100 + min));
} /* end convertLon */

/* Debug Output */
#ifdef SERDB
void printDebugInfo(){
  DPN(" apm_mav_type; ");
  DPN(apm_system_status);  
  DPN(" apm_system_status; ");
  DPN(apm_mav_type);  
  DPN(" apm_fixtype; ");
  DPN(apm_fixtype);
  DPN(" apm_sat_visible; ");
  DPN(apm_sat_visible);
  DPN(" apm_latitude; ");
  DPN(apm_latitude);
  DPN(" apm_longitude; ");
  DPN(apm_longitude);
  DPN(" gps_status; ");
  DPN(gps_status);
  DPN(" apm_voltage_battery; ");
  DPN(apm_voltage_battery);
  DPN(" apm_current_battery; ");
  DPN(apm_current_battery);
    DPL();
  DPN(" apm_base_mode; ");
  DPN(apm_base_mode);
  DPN(" apm_custom_mode; ");
  DPN(apm_custom_mode);
  DPN(" flMode; ");
  DPN(flMode);
 

  DPL();
/*
    DPN (F("SysID"));
    DPN ("\t");
    DPN (F("CompID"));
    DPN ("\t");
    DPN (F("MavMode"));
    DPN ("\t");
    DPN (F("NavMode"));
    DPN ("\t");
    DPN (F("SysStat"));
    DPN ("\t");
    DPN (F("flMode"));
    DPN ("\t");
    DPN (F("FIX"));
    DPN ("\t");
    DPN (F("Volt"));
    DPN ("\t");    
    DPN (F("Armed"));
    DPL ();
    DPN (aq_mav_system);
    DPN ("\t");
    DPN (aq_mav_component);
    DPN ("\t");
    DPN (m2h_mode);
    DPN ("\t");
    DPN (m2h_nav_mode);
    DPN ("\t");
    DPN (m2h_sysstat);
    DPN ("\t");
    DPN (flMode);
    DPN ("\t");
    DPN (m2h_fix_type);
    DPN ("\t");    
    DPN (m2h_vbat_A/1E1);
    DPN ("\t");
    DPN (isArmed);
    DPL ();

    DPN (F("AQ Lat"));
    DPN ("\t");
    DPN (F("AQ Lon"));
    DPN ("\t");
    DPN (F("Alt"));
    DPN ("\t");
    DPN (F("V m/s"));
    DPN ("\t");
    DPN (F("Hom Lat"));
    DPN ("\t");    
    DPN (F("Hom Lon"));
    DPN ("\t");    
    DPN (F("Dist2home"));
    DPN ("\t");        
    DPN (F("Head"));
    DPN ("\t");
    DPN (F("Temp"));
    DPN ("\t");
    DPN (F("Thrott"));
    
    DPL ();
    DPN (double(m2h_gps_lat/1E7) );
    DPN ("\t");
    DPN (double(m2h_gps_lon/1E7) );
    DPN ("\t");
    DPN (m2h_gps_alt/1E3);
    DPN ("\t");
    DPN (alt_diff_1s/1E3);
    DPN ("\t");
    DPN (m2h_gps_lat_home/1E7);
    DPN ("\t");
    DPN (m2h_gps_lon_home/1E7);    
    DPN ("\t");
    DPN (home_distance_calc);
    DPN ("\t\t");
    DPN( ((uint16_t)(m2h_yaw*100)/100) );    
    DPN ("\t");
    DPN (m2h_temp/1E2);
    DPN ("\t");
    DPN (throttle-1024);
    DPL ();    
    */
}
#endif
/* End Debug Output */  

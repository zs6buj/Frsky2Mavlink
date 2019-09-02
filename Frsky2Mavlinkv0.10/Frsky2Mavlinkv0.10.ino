     
/*

    ZS6BUJ's Frsky to Mavlink

     Eric Stockenstrom - First code January 2018
     

This application reads Frsky passthrough serial telemetry sent from a Pixhawk or APM flight 
controller through Frsky Taranis equipment, emerging from the backbay SPort, converts it back
to Mavlink and sends it out again on Bluetooth.


 Changelog:
 2019-09-02 Adapt for mavlink 2 libraries
 */

  
#include <cstring>
#include <mavlink_types.h>
#include <common/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

boolean FT = true;
int iLth=0;
int pLth;
byte chr = 0x00;
const int packetSize = 70; 
byte pBuff[packetSize]; 
short crc=0;  
boolean crc_bad; 

//************* Pin Assignments
// Serial1 Frsky telemetry in     RX = A3   (TX = A2 not used)
// Serial to monitor print        RX = A10   TX = A9
// Serial2 for Mav BT out         RX = B11   TX = B10

int StatusLed = 6;  
int BoardLed = PC13;
int ledState = LOW; 
unsigned long ledMillis = 0;

boolean serGood = false;
boolean latlon_800_flag=false;     // 0800  
boolean lat_800_flag=false;        // 0800
boolean lon_800_flag=false;        // 0800  
boolean ST_5000_flag = false;      // 5000  On demand
boolean AP_5001_flag = false;      // 5001  
boolean GPS_5002_flag = false;     // 5002  
boolean Bat_5003_flag = false;     // 5003  
boolean Home_5004_flag = false;    // 5004  
boolean velyaw_5005_flag = false;  // 5005  
boolean AT_5006_flag = false;      // 5006  
boolean Param_5007_flag = false;   // 5007  3 x each at init
boolean Param_50071_flag = false;  // 5007 id1 
boolean Param_50072_flag = false;  // 5007 id2 
boolean Param_50073_flag = false;  // 5007 id3 
boolean Param_50074_flag = false;  // 5007 id4
boolean Param_50075_flag = false;  // 5007 id5
boolean Param_5008_flag = false;   // 5008 

//FrSky Variables
short ms2bits;
uint32_t fr_payload;

// FrSky Passthrough Variables
// 0x800 GPS
uint32_t fr_latlong;
float fr_lat = 0;
float fr_lon = 0;

// 0x5000 Text Msg
uint32_t fr_textmsg;
char ct[5];
char p_ct[5];
int ct_dups=0;
uint8_t fr_severity;
char fr_text[80];
boolean eot=false;

// 0x5001 AP Status
uint8_t fr_flight_mode;
uint8_t fr_simple;

uint8_t fr_land_complete;
uint8_t fr_armed;
uint8_t fr_bat_fs;
uint8_t fr_ekf_fs;

// 0x5002 GPS Status
uint8_t fr_numsats;
uint8_t fr_gpsStatus;
uint8_t fr_hdop;
uint8_t fr_vdop;
uint32_t fr_gps_alt;
float   fr_fgps_alt;
uint8_t neg;

//0x5003 Batt
float fr_bat_volts;
float fr_bat_amps;
uint16_t fr_bat_mAh;

// 0x5004 Home
uint16_t fr_home_dist;
float    fr_fhome_dist;
uint32_t fr_home_angle;
int32_t fr_home_alt;
float   fr_fhome_alt;
//float fr_home_angle;
short fr_pwr;

// 0x5005 Velocity and yaw
uint32_t fr_velyaw;
float fr_yaw;
float fr_vx;
float fr_vy;

// 0x5006 Attitude and range
float fr_roll;
float fr_pitch;
float fr_range;

// 0x5007 Parameters  - Sent 3x each at init
uint8_t fr_param_id ;
uint32_t fr_param_val;
uint32_t fr_frame_type;
uint32_t fr_fs_bat_volts;
uint32_t fr_fs_bat_mAh;
uint32_t fr_bat1_capacity;
uint32_t fr_bat2_capacity;

//0x5008 Batt
float fr_bat2_volts;
float fr_bat2_amps;
uint16_t fr_bat2_mAh;

// Mavlink timers
uint32_t HBmillis = 0;  // Heartbeat #00
uint8_t HBperiod;
// Mavlink message frequency
const float HB_Hz = 1.5;  

// Mavlink Header
uint8_t    system_id;
uint8_t    component_id;
uint8_t    mvType;

mavlink_message_t msg;
uint16_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

// HEARTBEAT  #0
uint8_t   mvAutopilotType;
uint8_t   mvSystemMode;
uint32_t  mvCustomMode;
uint8_t   mvSystemState;

// SYS_STATUS #1
uint32_t onboard_control_sensors_present=1;
uint32_t onboard_control_sensors_enabled=1;
uint32_t onboard_control_sensors_health=1;
uint16_t load;
uint16_t voltage_battery;
int16_t current_battery;
int8_t battery_remaining;
uint16_t drop_rate_comm=0;
uint16_t errors_comm=0;
uint16_t errors_count1=0;
uint16_t errors_count2=0;
uint16_t errors_count3=0;
uint16_t errors_count4=0;

// GPS_RAW_INT #24
uint64_t utime_usec;
uint8_t fix_type;
//int32_t lat;
//int32_t lon;
//int32_t altmsl;
uint16_t eph;
uint16_t epv;
uint16_t vel;
uint16_t cog;  //course over ground
uint8_t satellites_visible;
  
// ATTITUDE (#30)
uint32_t time_boot_ms;
float roll;
float pitch;
//float yaw;
float rollspeed;
float pitchspeed;
float yawspeed;

// GLOBAL_POSITION_INT #33
int32_t lat;
int32_t lon;
int32_t altmsl;
int32_t altrel;
int16_t vx;
int16_t vy;
int16_t vz;
uint16_t yaw;

// VFR_HUD ( #74 )
float airspeed;
float groundspeed; 
float alt; 
float climb; 
int16_t heading; 
uint16_t throttle; 

// STATUSTEXT #253 

uint8_t st_severity;
char st_text[50];

//***************************************************
void setup()
{
  SetupBluetooth();           // HC-06 module called zs6buj on Mav out serial2
  
  Serial1.begin(57600);       // Frsky Telemetry input      - TX2 & RX2, Pins A2  & A3
  Serial2.begin(57600);       // Mavlink Telemetry output   - TX3 & RX3, Pins B10 & B11
  Serial.begin(115200);       // Flash & Print Output       - TX1 & RX1, Pins A9  & A10
  
  delay(500);
  pinMode(StatusLed , OUTPUT ); 
  pinMode(BoardLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BoardLed, HIGH);  // Logic is reversed! Initialse off

  // Initialise the Mavlink header
  system_id = 20;                           // ID 20 for this aircraft
  component_id = MAV_COMP_ID_AUTOPILOT1;    // The component sending the message is autopilot
  
  mvType = MAV_TYPE_QUADROTOR;         
  mvAutopilotType = MAV_AUTOPILOT_GENERIC;
  mvSystemMode = MAV_MODE_PREFLIGHT;
  mvCustomMode = 0;
  mvSystemState = MAV_STATE_STANDBY;   // System ready for flight

  // Initialise Mav message frequencies
  HBmillis = millis();
  HBperiod = 1000 / HB_Hz;
     
  Serial.println("Starting up......");
  
}
//***************************************************
//***************************************************
void loop()  {

  DecodeFrSky();   
  DecodeMavlink(); // If you expect anything inbound from GCS
  EncodeMavlink();
  SendHeartbeat();  // To GCS
 
 // delay(10);
    
}  
//***************************************************
//***************************************************
void EncodeMavlink() {     
  
  // Heartbeat #00
  SendHeartbeat();   // To GCS

  // sys_status message #1
  if (Bat_5003_flag) { 
    voltage_battery = fr_bat_volts * 100;  // mV for Mavlink, decivolts for Frsky
    current_battery = fr_bat_amps;         //  in 10*milliamperes (1 = 10 milliampere) for Mavlink = deciamps for Frsky
    battery_remaining = fr_bat_mAh;         // mAh for Mavlink
                   
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, onboard_control_sensors_present, onboard_control_sensors_enabled, 
      onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, drop_rate_comm, errors_comm, 
      errors_count1, errors_count2, errors_count3, errors_count4);
   len = mavlink_msg_to_send_buffer(buf, &msg);
   Serial2.write(buf, len); 
   Bat_5003_flag=false; 
   
   Serial.print(" Mavlink #1: Bat volts=");
   Serial.print(voltage_battery);
   Serial.print(" Bat amps=");
   Serial.print(current_battery);        
   Serial.print(" Bat remain=");
   Serial.println(battery_remaining);
        
  }
  
  // GPS_RAW_INT #24
  if (GPS_5002_flag && velyaw_5005_flag) {  
    fix_type = fr_gpsStatus;
    altmsl = fr_gps_alt;
    eph = fr_hdop;
    epv = fr_vdop;
    cog = fr_yaw * 100;  //  direction of movement) in degrees * 100 - yaw will have to do for now
    satellites_visible = fr_numsats;

    mavlink_msg_gps_raw_int_pack(system_id, component_id,  &msg,
        utime_usec, fix_type, lat, lon, altmsl, eph, epv, vel, cog, satellites_visible, 0, 0, 0, 0, 5 );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len); 
  //  GPS_5002_flag=false;       // Done lower at #74
  //  velyaw_5005_flag=false;    // Done lower at #74
/*
    Serial.print(" Mavlink #24: fix_type=");
    Serial.print(fix_type);
    Serial.print(" altmsl=");
    Serial.print(altmsl);        
    Serial.print(" eph=");
    Serial.print(eph);
    Serial.print(" epv=");
    Serial.print(epv);    
    Serial.print(" cog=");
    Serial.print(cog);    // Course over ground - GPS direction of travel - black line
    Serial.print(" sats vis=");
    Serial.println(satellites_visible);       
*/
  }

 // ATTITUDE #30   Max Hz
 if (AT_5006_flag) { 
   roll = fr_roll/180*PI;       // Degrees to radians for Mavlink
   pitch = fr_pitch/180*PI; 
                   
   mavlink_msg_attitude_pack(system_id, component_id, &msg, time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
   len = mavlink_msg_to_send_buffer(buf, &msg);
   Serial2.write(buf, len); 
   /*
   Serial.print(" Mavlink #30: roll=");
   Serial.print(roll, 2);
   Serial.print("rad   pitch=");
   Serial.print(pitch, 2); 
   Serial.println("rad");       
     */
   AT_5006_flag=false;
 }
  
  // Global Position Int #33                    lat = fr_lat * 1E7;  // expressed as degrees * 1E7
  
  if (latlon_800_flag && GPS_5002_flag && Home_5004_flag && velyaw_5005_flag) {  
    lat = fr_lat * 1E7;
    lon = fr_lon * 1E7;
    altmsl = fr_gps_alt;
    altrel = fr_fhome_alt;   // expressed as * 1000 (millimeters) can be negative
    vx = fr_vx;              // expressed as m/s * 100
    vy = fr_vy;              // expressed as m/s * 100
  //  yaw = fr_yaw / 500;      // compass heading in degrees * 100, 0.0..359.99 degrees
    yaw = fr_yaw;      // compass heading in degrees * 100, 0.0..359.99 degrees
    
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg, 
      time_boot_ms, lat, lon, altmsl, altrel, vx, vy, vz, yaw);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len); 

    // Expire the flags
    latlon_800_flag=false;
  //  GPS_5002_flag=false;       // Done lower at #74
    Home_5004_flag=false;
  //  velyaw_5005_flag=false;    // Done lower at #74
   
    Serial.print(" Mavlink #33: lat=");
    Serial.print(lat);
    Serial.print(" lon=");
    Serial.print(lon);        
    Serial.print(" altmsl=");
    Serial.print(altmsl);
    Serial.print(" altrel=");
    Serial.print(altrel);    
    Serial.print(" vx=");
    Serial.print(vx);
    Serial.print(" vy=");
    Serial.print(vy);    
    Serial.print(" yaw(hdg)=");
    Serial.println(yaw);       

  }
  /*
  // VFR_HUD ( #74 )
  if (velyaw_5005_flag && GPS_5002_flag) {
    airspeed = fr_vx * 0.1;         // Current airspeed in m/s - Frsky decimeters/s
    groundspeed = fr_vx * 0.1;      // Current ground speed in m/s - Frsky decimeters/s
    alt = fr_fgps_alt;              // Current altitude (MSL) in meters
    climb = fr_vy * 0.1;            // Current climb rate in meters/second - Frsky decimeters/s
    heading = fr_yaw;               // Current heading in degrees, in compass units (0..360, 0=north)
    throttle = 0;                   // Current throttle setting in integer percent, 0 to 100 - Don't have it :(

    mavlink_msg_vfr_hud_pack(system_id, component_id, &msg,
                   airspeed, groundspeed, heading, throttle, alt, climb);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len); 

    Serial.print(" Mavlink #74: airspeed=");
    Serial.print(airspeed, 1);
    Serial.print(" groundspeed=");
    Serial.print(groundspeed, 1);        
    Serial.print(" alt (msl)=");
    Serial.print(alt, 1);
    Serial.print(" climb=");
    Serial.print(climb, 1);    
    Serial.print(" heading=");
    Serial.print(heading);
    Serial.print(" throttle=");
    Serial.println(throttle);    
             
    GPS_5002_flag=false;
    velyaw_5005_flag=false; 
  }
  */
  // STATUSTEXT ( #253 )
  if (ST_5000_flag){ 
    st_severity =  fr_severity;
    strcpy(st_text, fr_text);   // 0-48

    mavlink_msg_statustext_pack(system_id, component_id, &msg, st_severity, st_text);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);  

    Serial.print(" Mavlink #253: Severity=");
    Serial.print(st_severity);
    Serial.print(" Msg : ");  
    Serial.println(st_text);

   fr_text[0]=0;
   ST_5000_flag=false;                   
  }
}
//***************************************************
void SendHeartbeat() {     // To GCS
  /*
   Serial.print("Heartbeat 1    millis=");
   Serial.print(millis());
   Serial.print("   HBmillis=");
   Serial.println(HBmillis);
   */
  if (millis() > (HBmillis + HBperiod)){  
    // Serial.println("Send Heartbeat");
    // Pack the message
    mavlink_msg_heartbeat_pack(system_id,component_id, &msg, mvType, mvAutopilotType, mvSystemMode, mvCustomMode, mvSystemState);
    // Copy the message to the send buffer
    len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message
    Serial2.write(buf, len);
    HBmillis=millis();                              
  }
}

//***************************************************
void DecodeFrSky() {
   if (FT) {                  // Sync to the first start/stop character 0x7E
    chr = NextChar();
    while (!(chr==0x7E)) {
      chr = NextChar();
     }
    FT=false;
  }
  // Candidate found 
  
  pBuff[0]=chr;            // Start-Stop character 0x7E
  pBuff[1]=NextChar();     // Sensor-ID
  
  chr=NextChar();                 // Start-Stop or Data-Frame Header
  
  if (chr==0x10) {                // If data frame header
    pBuff[2]=chr;
    boolean goodPacket=ParseFrsky();
    if (goodPacket) ProcessFrsky();
  //  DisplayTheBuffer(10); 
    chr=NextChar();   //  Should be the next Start-Stop  
    }
 // else DisplayTheBuffer(2); 
 
  if (!(chr==0x7E)) FT=true;  //  If next char is not start-stop then the frame sync has been lost. Resync 
}
//***************************************************
void DecodeMavlink() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(Serial2.available() > 0 ) 
  {
    uint8_t c = Serial2.read();
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
 
      switch(msg.msgid)
      {
              case MAVLINK_MSG_ID_HEARTBEAT:
              {
          // E.g. read GCS heartbeat and go into
                                  // comm lost mode if timer times out
              }
              break;
      case MAVLINK_MSG_ID_COMMAND_LONG:
        // EXECUTE ACTION
        break;
      default:
        //Do nothing
        break;
      }
    }
 
    // And get the next one
  }
}
//***************************************************
void SetupBluetooth() {
  // Using HC-06 bluetooth model front-end - Default pin = 1234
  Serial2.begin(9600);               //  HC-06 bluetooth module default speed
  delay(200); 
  Serial2.print("AT+NAMEzs6buj");    //  Optional - Configure your HC-06 bluetooth name
  Serial2.print("AT+BAUD7");         // Set the HC-06 speed to Mavlink default speed 57600 bps
  delay(3000);                       // Wait for HC-06 reboot
}
//***************************************************
void Add_Crc (uint8_t byte) {
  crc += byte;       //0-1FF
  crc += crc >> 8;   //0-100
  crc &= 0x00ff;
  crc += crc >> 8;   //0-0FF
  crc &= 0x00ff;
  }
//***************************************************
byte NextChar() {
byte x;

  iLth=Serial1.available();     //   wait for more data
  
  while (iLth==0) {
    iLth=Serial1.available();
    SendHeartbeat();        // Ensure heartbeat is still sent while waiting for FrSky packets
  }
  // Data is available
  serGood = true;              // We have a good serial connection!
  x = Serial1.read();

  return x;
}
//***************************************************
boolean ParseFrsky() {
 crc=0;
  Add_Crc(pBuff[2]);           // data frame char into crc
 
  for (int i=3; i<=8; i++) {
    chr = NextChar();
    pBuff[i]=chr;
    Add_Crc(chr);
  }
  chr=NextChar(); 
  pBuff[9]=chr;  //  crc

  if (chr==(0xFF-crc)){
    crc_bad = false; 
 //  Serial.println("CRC Good");
  }
  else {
    crc_bad=true;
//   Serial.println("CRC Bad");
  }
  return !crc_bad;
} 
//***************************************************
void ProcessFrsky() {
  // Do the sensor packets according to fr_payload type. We are expecting Mavlink Passthrough only
 uint16_t fr_payloadType = Unpack_uint16(3);
 fr_payload= Unpack_uint32(5);
   //   Serial.print(" fr_payloadType=");
  //    Serial.println(fr_payloadType, HEX);

      
 switch(fr_payloadType) {
 
   case 0x800:                      // Latitude and Longitude
                   fr_latlong= Unpack_uint32(5);
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
                   /*
                   Serial.print(" ms2bits=");
                   Serial.println(ms2bits);
                   */
  
                   if (ms2bits==0) {
                     fr_lat = fr_latlong / 6E5;     // Only ever update lon and lat in pairs. Lon always comes first
                     if (!(fr_lon==0)) lat_800_flag = true;  }
                     else
                       if (ms2bits==1) {
                         fr_lat = 0-(fr_latlong / 6E5); 
                         if (!(fr_lat==0)) lat_800_flag = true;   }
                         else 
                           if (ms2bits==2) {
                             fr_lon = fr_latlong / 6E5;
                             if (!(fr_lon==0)) lon_800_flag = true; } 
                             else
                               if (ms2bits==3) {
                                 fr_lon = 0-(fr_latlong / 6E5);
                                 if (!(fr_lon==0)) lon_800_flag = true;  }
                                 
                   latlon_800_flag = lat_800_flag && lon_800_flag; 
                   
                   if (latlon_800_flag) {
                     Serial.print("FrSky: latitude=");
                     Serial.print(fr_lat,7);
                     Serial.print(" longitude=");
                     Serial.println(fr_lon,7);        
                   }           
                   break;

                 // *****************************************************************
                 case 0xF000:   
                   break;
                 case 0xF101:   
                   break;
                 case 0xF103:   
                   break;
                 case 0xF104:   
                   break;
                 case 0xF105:   
                   break;
                 // *****************************************************************   
                 //   Mavlink Passthrough Protocol below  
                 
                 case 0x5000:                         // Text Message
                   ct[0] = pBuff[8];
                   ct[1] = pBuff[7];
                   ct[2] = pBuff[6];
                   ct[3] = pBuff[5];
                   ct[4] = 0;  // terminate string

                   if (ct[0] == 0 || ct[1] == 0 || ct[2] == 0 || ct[3] == 0) 
                     fr_severity = (bit32Extract(fr_payload,15,1) * 4) + (bit32Extract(fr_payload,23,1) * 2) + (bit32Extract(fr_payload,30,1) * 1);

                   if (strcmp(ct,p_ct)==0){     //  If this one = previous one it's a duplicate
                     ct_dups++;
                     break;
                     }
                     
                   // This is the last (3rd) duplicate of (usually) 4 chunks

                   for ( int i=0; i<5 ;i++ ) {  // Copy current 4 byte chunk to previous
                     p_ct[i]=ct[i];
                    }
                   
                   eot=false; 
 
                   if (ct[1] >= 0x80) { // It should always be this one, first of last 3 bytes
                 //    b1 = ct[1]; 
                     ct[1]=0;
                     ct[2]=0;
                     ct[3]=0;
                     eot=true;
                   }
                   Serial.print("TXT=");  
                   Serial.print(ct);
                   Serial.print(" ");
                   strcat(fr_text, ct);  // Concatenate ct onto the back of fr_text
                   ct_dups=0;
                 
                   if (!eot) break; // Drop through when end-of-text found

                   Serial.print("Frsky 5000: Severity=");  
                   Serial.print(fr_severity);
                   Serial.print(" Msg : ");  
                   Serial.println(fr_text);

                   ST_5000_flag = true;  // Trigger Status Text encode in EncodeMavlink()

                   break; 
                   
                 case 0x5001:                         // AP Status 2 Hz
                   fr_flight_mode = bit32Extract(fr_payload,0,5);
                   fr_simple = bit32Extract(fr_payload,5,2);
                   fr_land_complete = bit32Extract(fr_payload,7,1);
                   fr_armed = bit32Extract(fr_payload,8,1);
                   fr_bat_fs = bit32Extract(fr_payload,9,1);
                   fr_ekf_fs = bit32Extract(fr_payload,10,2);                   

                   AP_5001_flag = true;   

                   Serial.print("FrSky 5001: Flight_mode=");
                   Serial.print(fr_flight_mode);
                   Serial.print(" simple_mode=");
                   Serial.print(fr_simple);
                   Serial.print(" land_complete=");
                   Serial.print(fr_land_complete);
                   Serial.print(" armed=");
                   Serial.print(fr_armed);
                   Serial.print(" battery_failsafe=");
                   Serial.print(fr_bat_fs);    
                   Serial.print(" EKF_failsafe=");
                   Serial.println(fr_ekf_fs);  
                                                    
                   break;                   
                 case 0x5002:                         // GPS Status & Alt msl 1 Hz
                   fr_numsats = bit32Extract(fr_payload, 0, 4);
                   fr_gpsStatus = bit32Extract(fr_payload, 4, 2) + bit32Extract(fr_payload, 14, 2);
                   fr_hdop = bit32Extract(fr_payload, 7, 7) * (10^bit32Extract(fr_payload, 6, 1));
               //    fr_vdop = bit32Extract(fr_payload, 15, 7) * (10^bit32Extract(fr_payload, 14, 1)); 
               
                  fr_gps_alt = bit32Extract(fr_payload,24,7) * (10^bit32Extract(fr_gps_alt,22,2)); //-- dm
                  fr_gps_alt*=10;
                  if (bit32Extract(fr_gps_alt,31,1) == 1)
                     fr_gps_alt = fr_gps_alt * -1;
                  
                //   fr_gps_alt = bit32Extract(fr_payload, 16, 16);  //  For use with modified arducopter 3.5.5

                   Serial.print("FrSky 5002: Num sats=");
                   Serial.print(fr_numsats);
                   Serial.print(" gpsStatus=");
                   Serial.print(fr_gpsStatus);                 
                   Serial.print(" HDOP=");
                   Serial.print(fr_hdop);   
                   Serial.print(" alt amsl=");
                   Serial.println(fr_gps_alt); 

                   GPS_5002_flag=true;
                   break;
                 case 0x5003:                         // Battery 1 Hz
  
                   fr_bat_volts = bit32Extract(fr_payload,0,9);
                   fr_bat_amps = bit32Extract(fr_payload,10,7) * (10^bit32Extract(fr_payload,9,1));
                   fr_bat_mAh = bit32Extract(fr_payload,17,15);

                   Bat_5003_flag = true;
                   
                   Serial.print("FrSky 5003: Battery Volts=");
                   Serial.print(fr_bat_volts, 1);
                   Serial.print("  Battery Amps=");
                   Serial.print(fr_bat_amps, 1);
                   Serial.print("  Battery mAh=");
                   Serial.println(fr_bat_mAh);      
                   break;                          
                 case 0x5004:                         // Home 2 Hz
                   fr_home_dist = bit32Extract(fr_payload,2,10) * (10^bit32Extract(fr_payload,0,2));
                   fr_fhome_dist = (float)fr_home_dist * 0.1;   // metres
                   fr_home_alt = bit32Extract(fr_payload,14,10) * (10^bit32Extract(fr_payload,12,2));
                   fr_fhome_alt = (float)(fr_home_alt) * 0.01;  // metres
                   if (bit32Extract(fr_payload,24,1) == 1) 
                     fr_fhome_alt *=  -1;
                   fr_home_angle = bit32Extract(fr_payload, 25,  7) * 3;
                   
                   Home_5004_flag = true;
                   
                   Serial.print("FrSky 5004: Dist to home=");
                   Serial.print(fr_fhome_dist, 1);              
                   Serial.print(" home_alt=");
                   Serial.print(fr_fhome_alt, 1);    // This is correct but fluctuates slightly. Can be negative
                   Serial.print(" home_angle="); 
                   Serial.println(fr_home_angle);    // degrees
               
                   break;                        
                 case 0x5005:                    // Vert and Horiz Velocity and Yaw angle (Heading) 2 Hz
                   fr_vx = bit32Extract(fr_payload,1,7) * (10^bit32Extract(fr_payload,0,1));
                   if (bit32Extract(fr_payload,8,1) == 1)
                     fr_vx *= -1;
                   fr_vy = bit32Extract(fr_payload,10,7) * (10^bit32Extract(fr_payload,9,1));
                   fr_yaw = bit32Extract(fr_payload,17,11) * 0.2;
    
                   velyaw_5005_flag = true;        
          
                   Serial.print("FrSky 5005: v_veloc=");
                   Serial.print(fr_vx,1);        // m/s
                   Serial.print(" h_veloc=");
                   Serial.print(fr_vy,1);        // m/s       
                   Serial.print(" yaw_angle="); 
                   Serial.println(fr_yaw,1);    // degrees

                   
                   break; 
                 case 0x5006:                         // Roll, Pitch and Range - Max Hz      
                   fr_roll = bit32Extract(fr_payload,0,11);        
                   fr_roll = (fr_roll - 900) * 0.2;             //  -- roll [0,1800] ==> [-180,180] 
                   fr_pitch = bit32Extract(fr_payload,11,10);   
                   fr_pitch = (fr_pitch - 450) * 0.2;           //  -- pitch [0,900] ==> [-90,90]
                   fr_range = bit32Extract(fr_payload,22,10) * (10^bit32Extract(fr_payload,21,1));

                   AT_5006_flag = true;
                   /*
                   Serial.print("Frsky 5006: Range=");
                   Serial.print(fr_range,2);
                   Serial.print(" Roll=");
                   Serial.print(fr_roll);
                   Serial.print("deg   Pitch=");
                   Serial.print(fr_pitch);   
                   Serial.println("deg");               
                     */
                   break;                                         
                 case 0x5007:                         // Parameters
                   fr_param_id = bit32Extract(fr_payload,24,4);
                   fr_param_val = bit32Extract(fr_payload,0,24);
                   if (fr_param_id == 1) {
                     fr_frame_type = fr_param_val;
                     Param_50071_flag = true;
                     Serial.print("Frsky 5007: Frame_type=");
                     Serial.println(fr_frame_type);
                   }
                   else if (fr_param_id == 2) {
                     fr_fs_bat_volts = fr_param_val;
                     Param_50072_flag = true;
                     Serial.print(" Bat failsafe volts=");
                     Serial.println(fr_fs_bat_volts);
                   }
                   else if (fr_param_id == 3) {
                     fr_fs_bat_mAh = fr_param_val;
                     Param_50073_flag = true;
                     Serial.print("Frsky 5007: Bat failsafe mAh=");
                     Serial.println(fr_fs_bat_mAh);         
                   }
                   else if (fr_param_id== 4) {
                     fr_bat1_capacity = fr_param_val;
                     Param_50074_flag = true;
                     Serial.print("Frsky 5007: Bat1 capacity=");
                     Serial.println(fr_bat1_capacity);
                   }         
                   else if (fr_param_id == 5) {
                     fr_bat2_capacity = fr_param_val;
                     Param_50075_flag = true;
                     Serial.print("Frsky 5007: Bat2 capacity=");
                     Serial.println(fr_bat2_capacity); 
                   }
                   
                   Param_5007_flag = true;

                   break;    
                 case 0x5008:                         // Battery 2
                   fr_bat2_volts = bit32Extract(fr_payload,0,9);
                   fr_bat2_amps = bit32Extract(fr_payload,10,7)  * (10^bit32Extract(fr_payload,9,1));
                   fr_bat2_mAh = bit32Extract(fr_payload,17,15);

                   Serial.print("FrSky 5008: Battery2 Volts=");
                   Serial.print(fr_bat_volts, 1);
                   Serial.print("  Battery2 Amps=");
                   Serial.print(fr_bat_amps, 1);
                   Serial.print("  Battery2 mAh=");
                   Serial.println(fr_bat_mAh);  
                   
                   Param_5008_flag = true;  
                   break;       
      }
}
//***************************************************
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
//  Serial.print(" Result=");
 // Serial.println(r);
  return r;
}
uint32_t createMask(uint8_t lo, uint8_t hi) {
  uint32_t r = 0;
  for (unsigned i=lo; i<=hi; i++)
       r |= 1 << i;
//  Serial.print(" Mask 0x=");
//  Serial.println(r, HEX);      
  return r;
}
//***************************************************
void ServiceTheStatusLed() {
/*
    Serial.print("GPS_5002_flag = ");
    Serial.print(GPS_5002_flag);
    Serial.print("   serGood = ");
    Serial.print(serGood);
    Serial.print("   homeInitialised = ");
    Serial.println(homeInitialised);
 
  if (GPS_5002_flag) {
    if (homeInitialised) 
      ledState = HIGH;
    else 
      BlinkLed(300);
    }
  else 
     if (serGood) 
       BlinkLed(1500);
     else
       ledState = LOW;
    digitalWrite(StatusLed, ledState);  
    digitalWrite(BoardLed, !ledState);
 */   
}

//***************************************************
void BlinkLed(int rate) {
  unsigned long cMillis = millis();
     if (cMillis - ledMillis >= rate) {    // blink period
        ledMillis = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}


//***************************************************

uint32_t Unpack_uint32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = pBuff[posn+3];
   byte b2 = pBuff[posn+2];
   byte b3 = pBuff[posn+1];
   byte b4 = pBuff[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
    
    // Now combine the four bytes into an unsigned 32bit integer

   uint32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//***************************************************
int32_t Unpack_int32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = pBuff[posn+3];
   byte b2 = pBuff[posn+2];
   byte b3 = pBuff[posn+1];
   byte b4 = pBuff[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
   
 // Now combine the four bytes into an unsigned 32bit integer
 
   int32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//***************************************************
uint16_t Unpack_uint16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

   byte b1 = pBuff[posn+1];
   byte b2 = pBuff[posn];  
    
    // Now convert the 2 bytes into an unsigned 16bit integer
    
    uint16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
int16_t Unpack_int16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
   byte b1 = pBuff[posn+1];
   byte b2 = pBuff[posn];
    
    // Now convert the 2 bytes into a signed 16bit integer
    
    int16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
uint8_t Unpack_uint8 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is one byte long

  byte b1 = pBuff[posn];
    
    // Now convert the byte into an unsigned 8 bit integer
    
   uint8_t myvar = b1;
   return myvar;
}
//***************************************************
void DisplayTheBuffer (int lth){
  for ( int i = 0; i < lth; i++ ) {
    byte b = pBuff[i];
    if (b<=0xf) Serial.print("0");
    Serial.print(b,HEX);
    Serial.print(" ");
  }
  Serial.println();

}
//***************************************************
void DisplayThePayload (){
  for ( int i = 0; i < 10; i++ ) {
    byte b = pBuff[i];
    if (b<=0xf) Serial.print("0");
    Serial.print(b,HEX);
    Serial.print(" ");
  }
  Serial.print( "  Payload= ");
  for ( int i = 8; i > 4; i-- ) {
    byte b = pBuff[i];
    if (b<=0xf) Serial.print("0");
    Serial.print(b,HEX);
    Serial.print(" ");
  }
  Serial.println();

}
//***************************************************
void DisplayField (int pos, int lth){
  for ( int i = pos; i < pos+lth; i++ ) {
    Serial.print(pBuff[i],HEX);
    Serial.print(" ");
  }
  Serial.print("// ");
}
//***************************************************
String TimeString (unsigned long epoch){
 int hh = (epoch  % 86400L) / 3600;   // remove the days (86400 secs per day) and div the remainer to get hrs
 int mm = (epoch  % 3600) / 60;       // calculate the minutes (3600 secs per minute)
 int ss = (epoch % 60);               // calculate the seconds

  String S = "";
  if (hh<10) S += "0";
  S += String(hh);
  S +=":";
  if (mm<10) S += "0";
  S += String(mm);
  S +=":";
  if (ss<10) S += "0";
  S += String(ss);
  return S;
}
//***************************************************

uint8_t Unpack8 (int posn){
  
    uint8_t myvar = pBuff[posn];
    return myvar;
}
//***************************************************
void ShowElapsed() {
  Serial.print(" Seconds=");
  unsigned long millnow=millis();
  float fSecs = millnow / 1000;
  Serial.print(fSecs,1);
  Serial.print(" ");
}
//***************************************************
void PrintMsg (const void *object){

  char b; 

  const unsigned char * const chars = static_cast<const unsigned char *>(object);

  for ( int i = 21; i < 30; i++ ) {
    Serial.print(chars[i]);
  }
  Serial.println();
}
 

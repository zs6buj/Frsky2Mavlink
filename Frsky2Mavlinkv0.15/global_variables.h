//=================================================================================================  
//================================================================================================= 
//
//                                 G L O B A L   V A R I A B L E S
//
//================================================================================================= 
//=================================================================================================

bool      pb_rx = true;  // For PrintByte() direction indication
uint8_t   clm = 0;  // Columns for PrintByte();
String    pgm_path;
String    pgm_name;

uint8_t   FrsLedState = LOW; 
uint8_t   BufLedState = LOW; 
 
uint32_t  hb_count=0;
uint32_t  buf_full_count = 0;

bool      ap_bat_paramsReq = false;
bool      ap_bat_paramsRead=false; 
bool      parm_msg_shown = false;
bool      ap_paramsList=false;
uint8_t   app_count=0;

bool      homGood = false;      
bool      frsGood = false;
bool      spGood = false;    // Good S.Port serial read
bool      spPrev = false;
bool      rssiGood = false;
bool      rssi35 = false;
bool      rssi65 = false;
bool      rssi109 = false;

bool      wifiSuGood = false;
bool      wifiSuDone = false;
bool      timeGood = false;
bool      ftGetBaud = true;
uint8_t   sdStatus = 0; // 0=no reader, 1=reader found, 2=SD found, 3=open for append 4 = open for read, 9=failed

uint8_t hb_period;
// Mavlink message frequency
const float hb_Hz = 1.5;  
uint32_t  hb_millis=0;
uint32_t  fr_timeout_millis=0;
uint32_t  fr_millis=0;
uint32_t  mv_millis=0;
uint32_t  sp_read_millis=0;
uint32_t  sp_millis=0;
uint32_t  gcs_millis=0;  
uint32_t  fchb_millis=0;
uint32_t  rds_millis=0;
uint32_t  acc_millis=0;
uint32_t  em_millis=0;
uint32_t  frs_led_millis=0;
uint32_t  health_millis = 0;
uint32_t  rssi_millis = 0;
uint32_t  param_millis = 0;
uint32_t  now_millis = 0;
uint32_t  prev_millis = 0;

float   lon1,lat1,lon2,lat2,alt1,alt2; 

struct Battery {
  float    mAh;
  float    tot_mAh;
  float    avg_dA;
  float    avg_mV;
  uint32_t prv_millis;
  uint32_t tot_volts;      // sum of all samples
  uint32_t tot_mW;
  uint32_t samples;
  bool ft;
  };
  
struct Battery bat1     = {
  0, 0, 0, 0, 0, 0, 0, true};   

struct Battery bat2     = {
  0, 0, 0, 0, 0, 0, 0, true};   

//=================================================================================================  
//=================================================================================================  
// 4D Location vectors
 struct Location {
  float lat; 
  float lon;
  float alt;
  float hdg;
  };
volatile struct Location hom     = {
  0,0,0,0};   // home location

volatile struct Location cur      = {
  0,0,0,0};   // current location  
   
struct Loc2D {
  float     lat; 
  float     lon;
  };
#define Max_Waypoints  256          // Note. This is a global RAM trade-off. If exceeded then Debug message and shut down  
// Loc2D WP[Max_Waypoints]; 
 

//=========================================== M A V L I N K =============================================    

mavlink_message_t   S2Gmsg, G2Smsg;   //  S.Port and GCS

uint8_t             GCSbuf[300]; 

bool                GCS_available = false;
uint16_t            len;

// Mavlink Messages

// Mavlink Header
uint8_t    ap_sysid;
uint8_t    ap_compid;
uint8_t    ap_targcomp;

uint8_t    ap_targsys;     //   System ID of target system - outgoing to FC

uint8_t    mvType;

// Message #0  HEARTHBEAT 
uint8_t    ap_type_tmp = 0;              // hold the type until we know HB not from GCS or Tracker
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;
bool       px4_flight_stack = false;
uint8_t    px4_main_mode = 0;
uint8_t    px4_sub_mode = 0;

// Message #0  GCS HEARTHBEAT 

uint8_t    gcs_type = 0;
uint8_t    gcs_autopilot = 0;
uint8_t    gcs_base_mode = 0;
uint32_t   gcs_custom_mode = 0;
uint8_t    gcs_system_status = 0;
uint8_t    gcs_mavlink_version = 0;

// Message #0  Outgoing HEARTHBEAT 
uint8_t    apo_sysid;
uint8_t    apo_compid;
uint8_t    apo_targcomp;
uint8_t    apo_mission_type;              // Mav2
uint8_t    apo_type = 0;
uint8_t    apo_autopilot = 0;
uint8_t    apo_base_mode = 0;
uint32_t   apo_custom_mode = 0;
uint8_t    apo_system_status = 0;

// Message # 1  SYS_status 
uint32_t   ap_onboard_control_sensors_present; // MAV_SYS_STATUS_SENSOR Bitmap
uint32_t   ap_onboard_control_sensors_enabled;
uint32_t   ap_onboard_control_sensors_health;  //Bitmap  0: error. Value of 0: error. Value of 1: healthy.
uint16_t   ap_load;   //d%  Maximum usage in percent of the mainloop time. Values: [0-1000]
uint16_t   ap_voltage_battery1 = 0;    // 1000 = 1V
int16_t    ap_current_battery1 = 0;    //  10 = 1A
uint8_t    ap1_battery_remaining = 0;   // % Battery energy remaining
int16_t    ap_drop_rate_comm = 0;     // Comms drop rate, dropped packets on all links
int16_t    ap_errors_comm = 0;        // Comms errors, dropped packets on all links
uint16_t   ap_errors_count1 = 0;      // Autopilot-specific errors
uint16_t   ap_errors_count2 = 0;      // Autopilot-specific errors
uint16_t   ap_errors_count3 = 0;      // Autopilot-specific errors
uint16_t   ap_errors_count4 = 0;      // Autopilot-specific errors
uint8_t    ap_ccell_count1= 0;
// Message # 2  SYS_status 
uint64_t  ap_time_unix_usec;          // us  Timestamp (UNIX epoch time).
uint32_t  ap_time_boot_ms;            // ms  Timestamp (time since system boot)

// Message #20 PARAM_REQUEST_READ    // outgoing request to read the onboard parameter with the param_id string id
uint8_t  gcs_targsys;            //   System ID
char     gcs_req_param_id[16];   //  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
int16_t  gcs_req_param_index;    //  Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
// param_index . Send -1 to use the param ID field as identifier (else the param id will be ignored)

float ap_bat1_capacity;
float ap_bat2_capacity;

// Message #21 PARAM_REQUEST_LIST 
//  Generic Mavlink Header defined above
  
// Message #22 PARAM_VALUE
char     ap_param_id [16]; 
float    ap_param_value;
uint8_t  ap_param_type;  
uint16_t ap_param_count;              //  Total number of onboard parameters
int16_t  ap_param_index;              //  Index of this onboard parameter

// Message #24  GPS_RAW_INT 
uint64_t   ap24_utime_usec = 0;
uint8_t    ap_fixtype = 3;            // 0= No GPS, 1=No Fix, 2=2D Fix, 3=3D Fix, 4=DGPS, 5=RTK_Float, 6=RTK_Fixed, 7=Static, 8=PPP
uint8_t    ap_sat_visible = 0;        // numbers of visible satelites
int32_t    ap_lat24 = 0;              // 7 assumed decimal places
int32_t    ap_lon24 = 0;              // 7 assumed decimal places
int32_t    ap_amsl24 = 0;             // 1000 = 1m
uint16_t   ap_eph;                    // GPS HDOP horizontal dilution of position (unitless)
uint16_t   ap_epv;                    // GPS VDOP vertical dilution of position (unitless)
uint16_t   ap_vel;                    // GPS ground speed (m/s * 100) cm/s
uint16_t   ap_cog;                    // Course over ground in degrees * 100, 0.0..359.99 degrees
// mav2
int32_t    ap_alt_ellipsoid;          // mm    Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
uint32_t   ap_h_acc;                  // mm    Position uncertainty. Positive for up.
uint32_t   ap_v_acc;                  // mm    Altitude uncertainty. Positive for up.
uint32_t   ap_vel_acc;                // mm    Speed uncertainty. Positive for up.
uint32_t   ap_hdg_acc;                // degE5   Heading / track uncertainty

// Message #26  SCALED_IMU
int16_t   ap26_xacc = 0;
int16_t   ap26_yacc = 0;
int16_t   ap26_zacc = 0;
int16_t   ap26_xgyro = 0;
int16_t   ap26_ygyro = 0;
int16_t   ap26_zgyro = 0;
int16_t   ap26_xmag = 0;
int16_t   ap26_ymag = 0;
int16_t   ap26_zmag = 0;
// mav2
int16_t ap26_temp = 0;             // cdegC

// Message #27 RAW IMU 
int32_t   ap27_xacc = 0;
int32_t   ap27_yacc = 0;
int32_t   ap27_zacc = 0;
int16_t   ap27_xgyro = 0;
int16_t   ap27_ygyro = 0;
int16_t   ap27_zgyro = 0;
int16_t   ap27_xmag = 0;
int16_t   ap27_ymag = 0;
int16_t   ap27_zmag = 0;
// mav2
int8_t    ap27_id = 0;
int16_t   ap27_temp = 0;             // cdegC


// Message #29 SCALED_PRESSURE
float      ap_press_abs;         // Absolute pressure (hectopascal)
float      ap_press_diff;        // Differential pressure 1 (hectopascal)
int16_t    ap_temperature;       // Temperature measurement (0.01 degrees celsius)

// Message ATTITUDE ( #30 )
uint32_t ap30_time_boot_ms;      // Timestamp (time since system boot)
float ap_roll;                   // Roll angle (rad, -pi..+pi)
float ap_pitch;                  // Pitch angle (rad, -pi..+pi)
float ap_yaw;                    // Yaw angle (rad, -pi..+pi)
float ap_rollspeed;              // Roll angular speed (rad/s)
float ap_pitchspeed;             // Pitch angular speed (rad/s)
float ap_yawspeed;               // Yaw angular speed (rad/s)

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
uint32_t ap33_time_boot_ms; // Timestamp (time since system boot)
int32_t  ap_lat33;          // Latitude, expressed as degrees * 1E7
int32_t  ap_lon33;          // Longitude, expressed as degrees * 1E7
int32_t  ap_amsl33;         // Altitude above mean sea level (millimeters)
int32_t  ap_alt_ag;         // Altitude above ground (millimeters)
int16_t  ap_vx;             // Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t  ap_vy;             // Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t  ap_vz;             // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap_gps_hdg;        // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

// Message #35 RC_CHANNELS_RAW
uint8_t ap_rssi;
bool    ap_rssi_ft = true; // first rssi connection
uint8_t ap_rssi35;

// Message #36 Servo_Output
bool      ap_servo_flag = false;  // true when servo_output record received
uint8_t   ap_port; 
uint16_t  ap_servo_raw[16];       // 16 channels, [0] thru [15] 

// Message #39 Mission_Item
//  Generic Mavlink Header defined above
uint16_t  ap_ms_seq;            // Sequence
uint8_t   ap_ms_frame;          // The coordinate system of the waypoint.
uint16_t  ap_ms_command;        // The scheduled action for the waypoint.
uint8_t   ap_ms_current;        // false:0, true:1
uint8_t   ap_ms_autocontinue;   //  Autocontinue to next waypoint
float     ap_ms_param1;         // PARAM1, see MAV_CMD enum
float     ap_ms_param2;         // PARAM2, see MAV_CMD enum
float     ap_ms_param3;         // PARAM3, see MAV_CMD enum
float     ap_ms_param4;         // PARAM4, see MAV_CMD enum
float     ap_ms_x;              // PARAM5 / local: X coordinate, global: latitude
float     ap_ms_y;              // PARAM6 / local: Y coordinate, global: longitude
float     ap_ms_z;              // PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
uint8_t   ap_mission_type;      // MAV_MISSION_TYPE - Mavlink 2

// Message #40 Mission_Request
//  Generic Mavlink Header defined above
//uint8_t   ap_mission_type;  

// Message #42 Mission_Current
//  Generic Mavlink Header defined above
bool ap_ms_current_flag = false;

// Message #43 Mission_Request_List
//  Generic Mavlink Header defined above
bool ap_ms_list_req = false;

// Message #44 Mission_Count
//  Generic Mavlink Header defined above
uint8_t   ap_mission_count = 0;
bool      ap_ms_count_ft = true;

// Message #51 Mission_Request_Int    From GCS to FC - Request info on mission seq #

uint8_t    gcs_target_system;    // System ID
uint8_t    gcs_target_component; // Component ID
uint16_t   gcs_seq;              // Sequence #
uint8_t    gcs_mission_type;      

// Message #62 Nav_Controller_Output
float     ap_nav_roll;           // Current desired roll
float     ap_nav_pitch;          // Current desired pitch
int16_t   ap_nav_bearing;        // Current desired heading
int16_t   ap_target_bearing;     // Bearing to current waypoint/target
uint16_t  ap_wp_dist;            // Distance to active waypoint
float     ap_alt_error;          // Current altitude error
float     ap_aspd_error;         // Current airspeed error
float     ap_xtrack_error;       // Current crosstrack error on x-y plane

// Message #65 RC_Channels
bool      ap_rc_flag = false;    // true when rc record received
uint8_t   ap_chcnt; 
uint16_t  ap_chan_raw[18];       // 16 + 2 channels, [0] thru [17] 

//uint16_t ap_chan16_raw;        // Used for RSSI uS 1000=0%  2000=100%
uint8_t  ap_rssi65;              // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown

// Message #73 Mission_Item_Int
uint8_t   ap73_target_system;    
uint8_t   ap73_target_component;    
uint16_t  ap73_seq;             // Waypoint ID (sequence number)
uint8_t   ap73_frame;           // MAV_FRAME The coordinate system of the waypoint.
uint16_t  ap73_command;         // MAV_CMD The scheduled action for the waypoint.
uint8_t   ap73_current;         // false:0, true:1
uint8_t   ap73_autocontinue;    // Autocontinue to next waypoint
float     ap73_param1;          // PARAM1, see MAV_CMD enum
float     ap73_param2;          // PARAM2, see MAV_CMD enum
float     ap73_param3;          // PARAM3, see MAV_CMD enum
float     ap73_param4;          // PARAM4, see MAV_CMD enum
int32_t   ap73_x;               // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
int32_t   ap73_y;               // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
float     ap73_z;               // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
uint8_t   ap73_mission_type;    // Mav2   MAV_MISSION_TYPE  Mission type.

// Message #74 VFR_HUD  
float    ap_hud_air_spd;
float    ap_hud_grd_spd;
int16_t  ap_hud_hdg;
uint16_t ap_hud_throt;   // %
float    ap_hud_amsl;   
float    ap_hud_climb;        

// Message #109 RADIO_status (Sik radio firmware)
uint8_t ap_rssi109;             // local signal strength
uint8_t ap_remrssi;             // remote signal strength
uint8_t ap_txbuf;               // how full the tx buffer is as a percentage
uint8_t ap_noise;               // background noise level
uint8_t ap_remnoise;            // remote background noise level
uint16_t ap_rxerrors;           // receive errors
uint16_t ap_fixed;              // count of error corrected packets

// Message  #125 POWER_status 
uint16_t  ap_Vcc;                 // 5V rail voltage in millivolts
uint16_t  ap_Vservo;              // servo rail voltage in millivolts
uint16_t  ap_flags;               // power supply status flags (see MAV_POWER_status enum)
/*
 * MAV_POWER_status
Power supply status flags (bitmask)
1   MAV_POWER_status_BRICK_VALID  main brick power supply valid
2   MAV_POWER_status_SERVO_VALID  main servo power supply valid for FMU
4   MAV_POWER_status_USB_CONNECTED  USB power is connected
8   MAV_POWER_status_PERIPH_OVERCURRENT peripheral supply is in over-current state
16  MAV_POWER_status_PERIPH_HIPOWER_OVERCURRENT hi-power peripheral supply is in over-current state
32  MAV_POWER_status_CHANGED  Power status has changed since boot
 */

// Message  #147 BATTERY_status 
uint8_t      ap_battery_id;       
uint8_t      ap_battery_function;
uint8_t      ap_bat_type;  
int16_t      ap_bat_temperature;    // centi-degrees celsius
uint16_t     ap_voltages[10];       // cell voltages in millivolts 
int16_t      ap_current_battery;    // in 10*milliamperes (1 = 10 milliampere)
int32_t      ap_current_consumed;   // mAh
int32_t      ap_energy_consumed;    // HectoJoules (intergrated U*I*dt) (1 = 100 Joule)
int8_t       ap147_battery_remaining;  // (0%: 0, 100%: 100)
int32_t      ap_time_remaining;     // in seconds
uint8_t      ap_charge_state;     

// Message #166 RADIO see #109


// Message #173 RANGEFINDER 
float ap_range; // m

// Message #226 RPM
float ap_rpm1;
float ap_rpm2;

// Message #181 BATTERY2 
uint16_t   ap_voltage_battery2 = 0;    // 1000 = 1V
int16_t    ap_current_battery2 = 0;    //  10 = 1A
uint8_t    ap_cell_count2 = 0;

// Message #253 STATUSTEXT
 uint8_t   ap_severity;
 char      ap_text[60];  // 50 plus padding
 uint8_t   ap_txtlth;
 bool      ap_simple=0;
 
//=====================================  F  R  S  K  Y  ===========================================


//int pLth;
byte chr = 0x00;

/* FrSky Passthrough Frame Structure
 * 
 * 0    Start/stop(0x7E)
 * 1    SensorID e.g. (0x1B)
 * 2    Frame header(0x10) 
 * 3&4  DataID e.g. 0x00 0x80 (0x0800)
 * 5-8  Payload
 * 9    CRC
 */
const uint16_t frBufsize = 512; 
byte          frbuf[frBufsize]; 
uint16_t      fr_lth = 0;
uint16_t      fr_idx = 0; 
int16_t       fr_top = 0; 


uint8_t crc = 0;  
boolean crc_bad; 
// FrSky Passthru Variables
uint32_t  fr_payload;

// 0x800 GPS
uint32_t  fr_latlong = 0;
uint8_t   ms2bits;
int32_t   fr_lat = 0;
int32_t   fr_lon = 0;
float     fr_flat = 0;
float     fr_flon = 0;

// 0x5000 Text Msg
uint32_t fr_textmsg;
char     fr_text[60];
uint8_t  fr_severity;
uint8_t  fr_txtlth;
char     fr_chunk[5];       // 4 plus 1 for termination
char     fr_prev_chunk[5];
uint8_t  fr_dups = 0;       // duplicates
uint8_t  fr_chunk_num;
uint8_t  fr_chunk_idx = 0;  // chunk index
char     fr_chunk_print[5];
boolean  fr_eotext = false;

// 0x5001 AP Status
uint8_t fr_flight_mode;
uint8_t fr_simple;

uint8_t fr_land_complete;
uint8_t fr_armed;
uint8_t fr_bat_fs;
uint8_t fr_ekf_fs;
uint8_t fr_imu_temp;

// 0x5002 GPS Status
uint8_t fr_numsats;
uint8_t fr_gps_status;           // part a
uint8_t fr_gps_adv_status;       // part b
uint8_t fr_hdop;
int32_t fr_amsl;                // decimetres
float   fr_famsl;               // float decimetres
uint8_t neg;

//0x5003 Batt
uint16_t fr_bat1_volts;
uint16_t fr_bat1_amps;
uint16_t fr_bat1_mAh;

// 0x5004 Home
uint16_t fr_home_dist;
float    fr_fhome_dist;       // variable for float in metres
int16_t  fr_home_angle;       // degrees
int16_t  fr_home_arrow;       // 0 = heading pointing to home, unit = 3 degrees
int16_t  fr_home_alt;         // decimetres
float    fr_fhome_alt;        // metres float

short fr_pwr;

// 0x5005 Velocity and yaw
uint32_t fr_velyaw;
float fr_vy;    // climb in decimeters/s
float fr_vx;    // groundspeed in decimeters/s
float fr_yaw;   // heading units of 0.2 degrees

// 0x5006 Attitude and range
uint16_t fr_roll;
uint16_t fr_pitch;
uint16_t fr_range;
float    fr_froll;
float    fr_fpitch;
float    fr_frange;

// 0x5007 Parameters  
uint8_t  fr_param_id ;
uint32_t fr_param_val;
uint32_t fr_frame_type;
uint32_t fr_bat_failsafe_volts;  // centi volts
uint32_t fr_bat_failsafe_mAh;
uint32_t fr_bat1_capacity;
uint32_t fr_bat2_capacity;
uint32_t fr_mission_count;
bool     fr_paramsSent = false;

//0x5008 Batt2
float fr_bat2_volts;
float fr_bat2_amps;
uint16_t fr_bat2_mAh;

//0x5009 Servo_raw         // 4 ch per frame
uint8_t  frPort; 
int8_t   fr_sv[5];       

//0x50F1 HUD
float    fr_air_spd;       // dm/s
uint16_t fr_throt;         // 0 to 100%
float    fr_bar_alt;       // metres

//0x50F2 Missions       
uint16_t  fr_ms_seq;                // WP number
uint16_t  fr_ms_dist;               // To next WP  
float     fr_ms_xtrack;             // Cross track error in metres
float     fr_ms_target_bearing;     // Direction of next WP
float     fr_ms_cog;                // Course-over-ground in degrees
int8_t    fr_ms_offset;             // Next WP bearing offset from COG

//0x50F3 Wind Estimate      
uint16_t  fr_wind_speed;            // dm/s
uint16_t  fr_direction;             // Wind direction relative to yaw, deg / 3

//0xF103
uint32_t fr_rssi;

//=================================================================================================   
//                S E T T I N G S   A N D   O P T I O N S   S T R U C T U R E
//=================================================================================================

    typedef enum gs_io_set { gs_ser = 0, gs_bt = 1 , gs_wifi = 2, gs_wifi_bt = 3, gs_none = 9} gs_io_t; 
    typedef enum gs_sd_set { gs_off = 0 , gs_on = 1} gs_sd_t; 
    typedef enum wfmode_set { ap = 1 , sta = 2, sta_ap = 3 } wfmode_t;   
    typedef enum wf_proto_set { tcp = 1 , udp = 2, } wf_proto_t; 
    typedef enum btmode_set { master = 1, slave = 2 } btmode_t;

    typedef struct  {
      byte          validity_check;  // 0xdc   
      gs_io_t       gs_io;       
      gs_sd_t       gs_sd;
      wfmode_t      wfmode;
      wf_proto_t    wfproto;
      uint32_t      baud;
      uint8_t       channel;
      char          apSSID[30];
      char          apPw[20];
      char          staSSID[30];
      char          staPw[20];
      char          host[20];     
      uint16_t      tcp_localPort;
      uint16_t      udp_localPort;
      uint16_t      udp_remotePort;
      btmode_t      btmode;
      char          btConnectToSlave[20];
      uint8_t       rssi_override;
      bool          web_support;  // this flag is not saved in eeprom    
      char*         gs_io0;       // ser
      char*         gs_io1;       // bt
      char*         gs_io2;       // wifi
      char*         gs_io3;       // wifi_bt 
      char*         gs_io9;       // none     
      char*         gs_sd0;       // off 
      char*         gs_sd1;       // on      
      char*         wfmode1;      // ap 
      char*         wfmode2;      // sta
      char*         wfmode3;      // sta_ap     
      char*         wfproto1;     // tcp
      char*         wfproto2;     // udp       
      char*         btmode1;      // master
      char*         btmode2;      // slave 
      char*         rssioverride; //rssi override  
      
      } settings_struct_t;
      
    settings_struct_t set;  

  

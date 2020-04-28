     
/*

    ZS6BUJ's Frsky to Mavlink

     Eric Stockenstrom - First code January 2018
     

This application reads Frsky passthrough serial telemetry sent from a Pixhawk or similar flight 
controller through Frsky Taranis equipment, emerging from the backbay SPort, converts it back
to Mavlink and sends it out again on Serial, Wifi and/or Bluetooth


 Changelog:
 2019-09-02 v0.10 Adapt for latest Mavlink 2 libraries
 2020-04-08 v0.12 Serious bug uncovered by maciek252. ^ is in not the (raise to a power) operator in CPP, obviously. :(
 2020-04-12 v0.13 Experimental ALPHA code. Please report and help me fix bugs. Thanks. 
                  Remember, Passthrough is presently uni-directional - you can't ask for paramters from the GCS.
 */

#include <cstring>
#include <mavlink_types.h>
#include "global_variables.h"
#include "config.h"                      // ESP_IDF libs included here

#if defined TEENSY3X || defined ESP8266  // Teensy 3.x && ESP8266 
  #undef F   // F defined as m->counter[5]  in c_library_v2\mavlink_sha256.h
             // Macro F()defined in Teensy3/WString.h && ESP8266WebServer-impl.h (forces string literal into prog mem)   
#endif

#include <ardupilotmega/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

//************* Pin Assignments
// Serial1 Frsky telemetry in     RX = A3   (TX = A2 not used)
// Serial to monitor print        RX = A10   TX = A9
// Serial2 for Mav BT out         RX = B11   TX = B10


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

//=================================================================================================   
//                     F O R W A R D    D E C L A R A T I O N S
//=================================================================================================

void HarvestFrskyBuffer();
void OledPrintln(String);
uint32_t GetBaud(uint8_t);
void main_loop();
void SenseWiFiPin(); 
void Send_FC_Heartbeat();
void ServiceStatusLeds();
void checkLinkErrors(mavlink_message_t*); 
bool Read_Bluetooth(mavlink_message_t*);
bool Send_Bluetooth(mavlink_message_t*);
bool Read_TCP(mavlink_message_t*);
bool Read_UDP(mavlink_message_t*);
bool Send_TCP(mavlink_message_t*);
bool Send_UDP(mavlink_message_t*);
uint32_t Get_Volt_Average1(uint16_t);
uint32_t Get_Volt_Average2(uint16_t);
uint32_t Get_Current_Average1(uint16_t);
uint32_t Get_Current_Average2(uint16_t); 
void Accum_mAh1(uint32_t);
void Accum_mAh2(uint32_t);
void Accum_Volts1(uint32_t); 
void Accum_Volts2(uint32_t); 
uint32_t GetConsistent(uint8_t);
uint32_t SenseUart(uint8_t);
void SPort_SendByte(uint8_t, bool);
void SPort_SendDataFrame(uint8_t, uint16_t, uint32_t);
uint32_t createMask(uint8_t, uint8_t);
uint32_t Abs(int32_t);
float RadToDeg (float );
uint16_t prep_number(int32_t, uint8_t, uint8_t);
int16_t Add360(int16_t, int16_t);
float wrap_360(int16_t);
int8_t PWM_To_63(uint16_t);
void ServiceStatusLed();
void BlinkFrsLed(uint32_t);
void DisplayRemoteIP();
bool Leap_yr(uint16_t);
void WebServerSetup();   
void RecoverSettingsFromFlash();
void PrintByte(byte);
void SetupWiFi();
void handleLoginPage();
void handleSettingsPage();
void handleSettingsReturn();
void handleOtaPage();
uint8_t EEPROMRead8(uint16_t);
void ReadSettingsFromEEPROM();
uint32_t EEPROMRead32(uint16_t);
uint16_t EEPROMRead16(uint16_t);
void RefreshHTMLButtons();
void RawSettingsToStruct();
void WriteSettingsToEEPROM();
void EEPROMReadString(uint16_t, char*);
void EEPROMWrite16(uint16_t, uint16_t);
void EEPROMWrite8(uint16_t, uint8_t);
void EEPROMWriteString(uint16_t, char*);
void EEPROMWrite32(uint16_t, uint32_t);
void PrintRemoteIP();
void ReportSportStatusChange();

//=================================================================================================
//=================================================================================================   
//                                      S   E   T   U  P 
//=================================================================================================
//=================================================================================================
void setup() {
  
  Debug.begin(115200);
  delay(2500);
  Debug.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  Debug.print("Starting "); Debug.print(pgm_name); Debug.println(" .....");

//=================================================================================================   
//                                 S E T U P   O L E D
//=================================================================================================
  #if ((defined ESP32) || (defined ESP8266)) && (defined OLED_Support) 
    Wire.begin(SDA, SCL);
    display.begin(SSD1306_SWITCHCAPVCC, i2cAddr);  
    display.clearDisplay();
  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
  
    Debug.println("OLED Support activated");
    OledPrintln("Starting .... ");
  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */

//=================================================================================================   
//                             S E T U P   E E P R O M
//=================================================================================================

  #if defined ESP32    
    if (!EEPROM.begin(EEPROM_SIZE))  {
      Debug.println("Fatal error!  EEPROM failed to initialise.");
      OledPrintln("EEPROM fatal error!");
      while (true) delay(100);  // wait here forever 
     } else {
      Debug.println("EEPROM initialised successfully");
      OledPrintln("EEPROM good"); 
     }
  #endif       
    
  #if defined ESP8266
    EEPROM.begin(EEPROM_SIZE);
    Debug.println("EEPROM initialised successfully");
    OledPrintln("EEPROM good"); 
  #endif

  RawSettingsToStruct();      // So that we can use them regardless of webSupport
  
  #if (defined webSupport) 
    RecoverSettingsFromFlash(); 
  #endif
   

  // =============================================

  Debug.print("Target Board is ");
  #if (defined TEENSY3X) // Teensy3x
    Debug.println("Teensy 3.x");
    OledPrintln("Teensy 3.x");
  #elif (defined ESP32) //  ESP32 Board
    Debug.print("ESP32 / Variant is ");
    OledPrintln("ESP32 / Variant is");
    #if (ESP32_Variant == 1)
      Debug.println("Dev Module");
      OledPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 2)
      Debug.println("Wemos® LOLIN ESP32-WROOM-32");
      OledPrintln("Wemos® LOLIN");
    #endif
    #if (ESP32_Variant == 3)
      Debug.println("Dragonlink V3 slim with internal ESP32");
      OledPrintln("Dragonlink V3 ESP32");
    #endif
    #if (ESP32_Variant == 4)
      Debug.println("Heltec Wifi Kit 32");
      OledPrintln("Heltec Wifi Kit 32");
    #endif
    
  #elif (defined ESP8266) 
    Debug.print("ESP8266 / Variant is ");
    OledPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Debug.println("Lonlin Node MCU 12F");
      OledPrintln("Node MCU 12");
    #endif 
    #if (ESP8266_Variant == 2)
      Debug.println("ESP-F - RFD900X TX-MOD");
      OledPrintln("RFD900X TX-MOD");
    #endif       
  #endif           

  if (set.gs_io == gs_ser)  {
    Debug.println("Mavlink Serial Out");
    OledPrintln("Mavlink Serial Out");
  }

  if (set.gs_io == gs_bt)  {
    Debug.println("Mavlink Bluetooth Out");
    OledPrintln("Mavlink BT Out");
  }

  if (set.gs_io == gs_wifi)  {
    Debug.print("Mavlink WiFi Out - ");
    OledPrintln("Mavlink WiFi Out");
  }

  if (set.gs_io == gs_wifi_bt)  {
    Debug.print("Mavlink WiFi+BT Out - ");
    OledPrintln("Mavlink WiFi+BT Out");
  }

  if ((set.gs_io == gs_wifi) ||  (set.gs_io == gs_wifi_bt) || (set.web_support)) {
   if (set.wfproto == tcp)  {
     Debug.println("Protocol is TCP/IP");
     OledPrintln("Protocol is TCP/IP");
   }
   else if  (set.wfproto == udp) {
     Debug.println("Protocol is UDP");
     OledPrintln("Protocol is UDP");
   }
  }
  
  #if defined SD_Support     
    if (set.gs_sd == gs_on) {
      Debug.println("Mavlink SD Out");
      OledPrintln("Mavlink SD Out");
    }
  #endif
 
  //=================================================================================================   
  //                                S E T U P   W I F I  --  E S P only
  //=================================================================================================

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {

      pinMode(startWiFiPin, INPUT_PULLUP);

    }
  #else
    Debug.println("No WiFi options selected, WiFi support not compiled in");
  #endif
  //=================================================================================================   
  //                                   S E T U P   B L U E T O O T H
  //=================================================================================================

  // Slave advertises hostname for pairing, master connects to slavename

  #if (defined btBuiltin)
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) { 

      if (set.btmode == 1)   {               // master
        SerialBT.begin(set.host, true); 
        bool bt_connected;
        bt_connected = SerialBT.connect(set.btConnectToSlave);
        if(bt_connected) {
          Debug.printf("Bluetooth master mode host %s is connected to slave %s\n", set.host, set.btConnectToSlave);  
          OledPrintln("Bluetooth connected!");
        }          
      } else {                               // slave                            
        SerialBT.begin(set.host); 
        Debug.printf("Bluetooth slave mode, host name for pairing is %s\n", set.host);  
      }    
    }
  #else
    Debug.println("No Bluetooth options selected, BT support not compiled in");
  #endif

  //=================================================================================================   
  //                            S E T U P   S D   C A R D  -  E S P 3 2  O N L Y  for now
  //=================================================================================================
  #if ((defined ESP32) || (defined ESP8266)) && (defined SD_Support)
  
    Debug.println("SD Support activated");
    OledPrintln("SD support activated");

    void listDir(fs::FS &fs, const char *, uint8_t);  // Fwd declare
    
    if(!SD.begin()){   
        Debug.println("No SD card reader found. Ignoring SD!"); 
        OledPrintln("No SD reader");
        OledPrintln("Ignoring!");
        sdStatus = 0; // 0=no reader, 1=reader found, 2=SD found, 3=open for append 
                      // 4=open for read, 5=eof detected, 9=failed
    } else {
      Debug.println("SD card reader mount OK");
      OledPrintln("SD drv mount OK");
      uint8_t cardType = SD.cardType();
      sdStatus = 1;
      if(cardType == CARD_NONE){
          Serial.println("No SD card found");
          OledPrintln("No SD card");
          OledPrintln("Ignoring!");      
      } else {
        Debug.println("SD card found");
        OledPrintln("SD card found");
        sdStatus = 2;

        Debug.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        Debug.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

        listDir(SD, "/", 2);

        if (set.fc_io == fc_sd)  {   //  FC side SD in only   
          std::string S = "";  
          char c = 0x00;
           Debug.println("Enter the number of the SD file to read, and press Send");
           while (c != 0xA) { // line feed
            if (Debug.available())  {
              c = Debug.read();
              S+=c;
             }
             delay(50);
           }
           
           int i;
           // object from the class stringstream 
           std::istringstream myInt(S); 
           myInt >> i;
           Debug.print(i); Debug.print(" ");
           /*
           for (int j= 0 ; fnCnt > j ; j++)  {
           //   cout << i << fnPath[j] << "\n";
             Debug.print(j); Debug.print(" "); Debug.println(fnPath[j].c_str());
            }
           */     

           sprintf(cPath, "%s", fnPath[i].c_str());  // Select the path
           Debug.print(cPath); Debug.println(" selected "); 
           Debug.println("Reading SD card");
           OledPrintln("Reading SD card");
           file = SD.open(cPath);
           if(!file){
             Debug.printf("Can't open file: %s\n", cPath);
             Debug.println(" for reading");
             sdStatus = 9;  // error
           } else {
             sdStatus = 4;
           }
        }
               
        // The SD is initialised/opened for write in Main Loop after timeGood
        // because the path/file name includes the date-time
     }  
   }
  #endif
  //=================================================================================================   
  //                                    S E T U P   S E R I A L
  //=================================================================================================  

  SPort_Init();

  if (set.gs_io == gs_ser)  {          //  GCS Serial
    #if (defined ESP32)  
      mvSerialGCS.begin(set.baud, SERIAL_8N1, GC_Mav_rxPin, GC_Mav_txPin);   //  rx,tx, cts, rts
    #else
      mvSerialGCS.begin(set.baud);    
    #endif    
    Debug.printf("Mavlink serial output on pins rx = %d and tx = %d\n", GC_Mav_rxPin, GC_Mav_txPin);
  }

   
//=================================================================================================   
//                                    S E T U P   O T H E R
//=================================================================================================   

  pinMode(FrsStatusLed , OUTPUT ); 
  if (InvertFrsLed) {
    digitalWrite(FrsStatusLed, HIGH); 
  } else {
    digitalWrite(FrsStatusLed, LOW); 
  }


  // Initialise the outgoing Mavlink header
  ap_sysid = 20;                           // ID 20 for this aircraft
  ap_compid = MAV_COMP_ID_AUTOPILOT1;    // The component sending the message is autopilot
  
  gcs_type = MAV_TYPE_QUADROTOR;         
  gcs_autopilot = MAV_AUTOPILOT_GENERIC;
  gcs_base_mode = MAV_MODE_PREFLIGHT;
  gcs_custom_mode = 0;
  gcs_system_status = MAV_STATE_STANDBY;   // System ready for flight

  // Initialise Mav message frequencies
  hb_millis = millis();
  hb_period = 1000 / hb_Hz;

}

//================================================================================================= 
//                                        L  O  O  P
//================================================================================================= 
void loop() {            // For WiFi only
  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {  

      SenseWiFiPin();

      if (set.wfproto == tcp)  {  // TCP  
        if (wifiSuGood) {
          WiFiSTA = TCPserver.available();              // listen for incoming clients 
          if(WiFiSTA) {
            Debug.println("New client connected"); 
            OledPrintln("New client ok!");      
            while (WiFiSTA.connected()) {            // loop while the client's connected
              main_loop(); 
            }
          WiFiSTA.stop();
          Debug.println("Client disconnected");
          OledPrintln("Client discnnct!");      
          } else {
             main_loop();
         } 
        }  else { 
           main_loop();
        }  
      }
  
     if (set.wfproto == udp)  {  // UDP  
        main_loop();       
       } 
  
    else 
      main_loop();
    } else {
      main_loop();
    }
  #else 
    main_loop();
  #endif
}
//================================================================================================= 
//================================================================================================= 
//                                   M  A  I  N    L  O  O  P
//================================================================================================= 
//================================================================================================= 

void main_loop() {
  if (millis() - sp_millis > 1) {
    sp_millis = millis();
    if (frSerial.available()) {  // no-block async serial read
      sp_millis = millis();
      //  ShowPeriod(false); Debug.println("  PushToBuffer()");    
        PushToBuffer();            // harvest byes in the buffer, if any
    }
  }
  
  if ((fr_top > 0) && (millis() - fr_millis > 4)) {  // if we have bytes in buffer, find a frame & decode
    fr_millis = millis();
 //   ShowPeriod(false); Debug.println("  HandleFrskyFrame()");
    HandleFrskyFrame();    
  }

  if ((frsGood) && (millis() - mv_millis > 4))  {
    mv_millis = millis();
 //   ShowPeriod(false); Debug.println("  EncodeAndWriteMavlink()");
    EncodeAndWriteMavlink();
 //   ShowPeriod(false); Debug.println("  SendGCSHeartbeat()");   
    SendGCSHeartbeat();  // To GCS
  }

  if ((fr_top > 0) && (millis() - gcs_millis > 4)) {
//    ShowPeriod(false); Debug.println("  Read_From_GCS()"); 
    Read_From_GCS();
    if (GCS_available) {
   //  ShowPeriod(false); Debug.println("  DecodeMavlink()"); 
      DecodeMavlink();  // From GCS 
    }
  }

  
  if(frsGood && (millis() - fr_timeout_millis) > 6000)  {   // if no frs telemetry for 6s then timeout
    frsGood=false;
    Debug.println("Frs not connected"); 
    OledPrintln("Frs lost!");       
   } 
   
  ServiceStatusLeds();
 
  #if defined webSupport     //  esp32 and esp8266
    server.handleClient();
  #endif
} 
//=================================================================================================  
void PushToBuffer() {  
  fr_lth=frSerial.available();   // harvest byes in the buffer, if any
  if (fr_lth) {
    #if (defined Frs_Debug_All) || (defined Frs_Debug_SPort) 
      Debug.printf("------------------------------->Read fr_lth=%d\n",fr_lth);
    #endif  
    serGood = true;             // we have a good serial connection
    for (uint16_t i = 1 ; i<=fr_lth ; i++) { 
      if (fr_top >= frBufsize) {
        buf_full_count++;
        if ( (buf_full_count == 0) || (buf_full_count%1000 == 0)) {
          Debug.println("S.Port buffer overflow! Check outgoing link");  // Report every so often
        }
        fr_top--; // ignore overflow, usually because I/O out not connected
      }
      frbuf[fr_top] = frSerial.read();
                                                              // Debug.println(fr_top);  
      #if (defined Frs_Debug_All) || (defined Frs_Debug_SPort) 
        pb_rx = false; PrintByte(frbuf[fr_top]);
        Debug.printf("fr_top=%d\n",fr_top);
      #endif  
      fr_top++;
    }
  }
}  
//=================================================================================================  
void HandleFrskyFrame() {  // one frame only
  
/* FrSky Passthrough Frame Structure
 * 
 * 0    Start/stop(0x7E)
 * 1    SensorID e.g. (0x1B)
 * 2    Frame header(0x10) 
 * 3&4  DataID e.g. 0x00 0x80 (0x0800)
 * 5-8  Payload
 * 9    CRC
 * 
 *     fr_top is first empty slot
 */

  // Start of frame is always 0  
  if (fr_top == 0) return;    // safety test - should never happen
  while (frbuf[0]!=0x7E) {    // sync to the (first) start/stop character 0x7E
    #if (defined Frs_Debug_All) || (defined Frs_Debug_SPort)
      pb_rx = true; PrintByte(frbuf[0]);
      Debug.println(".");
    #endif  
    memcpy ( &frbuf[0], &frbuf[1], fr_top-1 );  // shift all buffer bytes down 1 place until start found
    fr_top--;
    if (fr_top < 0) return;   // get more bytes 
    }  
    
  // got a frame start!

  if ((fr_top > 2) && (frbuf[2] == 0x10)) {    // if more than 2 bytes in the buffer and data frame header
    frsGood = true;
    fr_timeout_millis = millis();
  //  ShowPeriod(false); Debug.println("  DecodeFrskyFrame()");
    DecodeFrskyFrame();

    #if (defined Frs_Debug_All) || (defined Frs_Debug_SPort)   
      Debug.printf("fr_top=%d ",fr_top);
      pb_rx = true; PrintTheBuffer(10);
    #endif    
    memcpy ( &frbuf[0], &frbuf[10], fr_top-10 );  // shift one frame down in the buffer
    fr_top-=10; 
    if (fr_top < 0) fr_top = 0; 
    } else {

    #if (defined Frs_Debug_All) || (defined Frs_Debug_SPort)     
      Debug.printf("fr_top=%d ",fr_top);
      pb_rx = true; PrintTheBuffer(2); // else pre-header only
    #endif  
    memcpy ( &frbuf[0], &frbuf[2], fr_top-2 );  // shift one frame down in the buffer
    fr_top-=2;  
    if (fr_top < 0) fr_top = 0;
    }
}

//=================================================================================================  
void Add_Crc (uint8_t byte) {
  crc += byte;       //0-1FF
  crc += crc >> 8;   //0-100
  crc &= 0x00ff;
  crc += crc >> 8;   //0-0FF
  crc &= 0x00ff;
  }
//=================================================================================================  
void DecodeFrskyFrame() {
  // Do the sensor packets according to fr_payload type. We are expecting Mavlink Passthrough only
 uint16_t fr_DataID = Unpack_uint16(3);
 fr_payload= Unpack_uint32(5);
 #if (defined Frs_Debug_All) || (defined Frs_Print_All_DataIDs)
   Debug.print(" fr_DataID=");
   Debug.println(fr_DataID, HEX);
 #endif
      
 switch(fr_DataID) {
 
   case 0x800:                      // Latitude and Longitude
                   fr_latlong= Unpack_uint32(5);
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
                   
                   //Debug.print(" ms2bits=");
                   //Debug.println(ms2bits);
                   
  
                   if (ms2bits==0) {
                     fr_flat = fr_latlong / 6E5;     // Only ever update lon and lat in pairs. Lon always comes first
                     if (!(fr_flon==0)) lat_800_flag = true;  }
                     else
                       if (ms2bits==1) {
                         fr_flat = 0-(fr_latlong / 6E5); 
                         if (!(fr_flat==0)) lat_800_flag = true;   }
                         else 
                           if (ms2bits==2) {
                             fr_flon = fr_latlong / 6E5;
                             if (!(fr_flon==0)) lon_800_flag = true; } 
                             else
                               if (ms2bits==3) {
                                 fr_flon = 0-(fr_latlong / 6E5);
                                 if (!(fr_flon==0)) lon_800_flag = true;  }
                                 
                   latlon_800_flag = lat_800_flag && lon_800_flag; 
                   #if (defined Frs_Debug_All) || (defined Frs_Debug_LatLon)    
                     if (latlon_800_flag) {
                       Debug.print("FrSky 800: latitude=");
                       Debug.print(fr_flat,7);
                       Debug.print(" longitude=");
                       Debug.println(fr_flon,7);        
                     }     
                   #endif        
                   break;

                 //===================================================================
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
                 //===================================================================
                 //   Mavlink Passthrough Protocol below  
                 
                 case 0x5000:                         // Text Message
                   fr_chunk[0] = frbuf[8];
                   fr_chunk[1] = frbuf[7];
                   fr_chunk[2] = frbuf[6];
                   fr_chunk[3] = frbuf[5];
                   fr_chunk[4] = 0;  // terminate string

                   if (fr_chunk[0] == 0 || fr_chunk[1] == 0 || fr_chunk[2] == 0 || fr_chunk[3] == 0) 
                     fr_severity = (bit32Extract(fr_payload,15,1) * 4) + (bit32Extract(fr_payload,23,1) * 2) + (bit32Extract(fr_payload,30,1) * 1);

                   if (strcmp(fr_chunk,fr_prev_chunk)==0){     //  If this one = previous one it's a duplicate
                     fr_dups++;
                     break;
                     }
                     
                   // This is the last (3rd) duplicate of (usually) 4 chunks

                   for ( int i=0; i<5 ;i++ ) {  // Copy current 4 byte chunk to previous
                     fr_prev_chunk[i]=fr_chunk[i];
                    }
                   
                   fr_eotext=false; 
 
                   if (fr_chunk[1] >= 0x80) { // It should always be this one, first of last 3 bytes
                 //    b1 = fr_chunk[1]; 
                     fr_chunk[1]=0;
                     fr_chunk[2]=0;
                     fr_chunk[3]=0;
                     fr_eotext=true;
                   }
                   #if (defined Frs_Debug_All) || (defined Frs_Debug_StatusText)     
                     Debug.print("TXT=");  
                     Debug.print(fr_chunk);
                     Debug.print(" ");
                   #endif  
                   strcat(fr_text, fr_chunk);  // Concatenate chunk onto the back of fr_text
                   fr_dups=0;
                 
                   if (!fr_eotext) break; // Drop through when end-of-text found
                   #if (defined Frs_Debug_All) || (defined Frs_Debug_StatusText)    
                     Debug.print("Frsky 5000: Severity=");  
                     Debug.print(fr_severity);
                     Debug.print(" Msg : ");  
                     Debug.println(fr_text);
                   #endif  

                   ST_5000_flag = true;  // Trigger Status Text encode in EncodeAndWriteMavlink()

                   break; 
                   
                 case 0x5001:                         // AP Status 2 Hz
                   fr_flight_mode = bit32Extract(fr_payload,0,5);
                   fr_simple = bit32Extract(fr_payload,5,2);
                   fr_land_complete = bit32Extract(fr_payload,7,1);
                   fr_armed = bit32Extract(fr_payload,8,1);
                   fr_bat_fs = bit32Extract(fr_payload,9,1);
                   fr_ekf_fs = bit32Extract(fr_payload,10,2);                   

                   AP_5001_flag = true;   
                   #if (defined Frs_Debug_All) || (defined Frs_Debug_APStatus)    
                     Debug.print("FrSky 5001: Flight_mode=");
                     Debug.print(fr_flight_mode);
                     Debug.print(" simple_mode=");
                     Debug.print(fr_simple);
                     Debug.print(" land_complete=");
                     Debug.print(fr_land_complete);
                     Debug.print(" armed=");
                     Debug.print(fr_armed);
                     Debug.print(" battery_failsafe=");
                     Debug.print(fr_bat_fs);    
                     Debug.print(" EKF_failsafe=");
                     Debug.println(fr_ekf_fs);  
                   #endif  
                                                    
                   break;                   
                 case 0x5002:                         // GPS Status & Alt msl 1 Hz
                   fr_numsats = bit32Extract(fr_payload, 0, 4);
                   fr_gps_status = bit32Extract(fr_payload, 4, 2) + bit32Extract(fr_payload, 14, 2);
                   fr_hdop = bit32Extract(fr_payload, 7, 7) * TenToPwr(bit32Extract(fr_payload, 6, 1));
               //    fr_vdop = bit32Extract(fr_payload, 15, 7) * TenToPwr(bit32Extract(fr_payload, 14, 1)); 
               
                  fr_amsl = bit32Extract(fr_payload,24,7) * TenToPwr(bit32Extract(fr_amsl,22,2)); //-- dm
                  fr_amsl*=10;
                  fr_famsl = (float)fr_amsl;
                  if (bit32Extract(fr_amsl,31,1) == 1) {
                    fr_amsl = fr_amsl * -1; 
                    fr_famsl*= -1;                 
                  }
                  #if (defined Frs_Debug_All) || (defined Frs_Debug_GPS_Status)
                    Debug.print("FrSky 5002: Num sats=");
                    Debug.print(fr_numsats);
                    Debug.print(" gpsStatus=");
                    Debug.print(fr_gps_status);                 
                    Debug.print(" HDOP=");
                    Debug.print(fr_hdop);   
                    Debug.print(" alt amsl=");
                    Debug.println(fr_amsl); 
                  #endif
                 GPS_5002_flag=true;
                 break;
               case 0x5003:                         // Battery 1 Hz
                 fr_bat1_volts = bit32Extract(fr_payload,0,9);
                 fr_bat1_amps = bit32Extract(fr_payload,10,7) * TenToPwr(bit32Extract(fr_payload,9,1));
                 fr_bat1_mAh = bit32Extract(fr_payload,17,15);

                 Bat_5003_flag = true;
                 #if (defined Frs_Debug_All) || (defined Frs_Debug_Batteries)               
                   Debug.print("FrSky 5003: Battery Volts=");
                   Debug.print(fr_bat1_volts, 1);
                   Debug.print("  Battery Amps=");
                   Debug.print(fr_bat1_amps, 1);
                   Debug.print("  Battery mAh=");
                   Debug.println(fr_bat1_mAh); 
                 #endif       
                 break;                          
               case 0x5004:                         // Home 2 Hz
                 fr_home_dist = bit32Extract(fr_payload,2,10) * TenToPwr(bit32Extract(fr_payload,0,2));
                 fr_fhome_dist = (float)fr_home_dist * 0.1;   // metres
                 fr_home_alt = bit32Extract(fr_payload,14,10) * TenToPwr(bit32Extract(fr_payload,12,2)); // decimetres
                 fr_fhome_alt = (float)(fr_home_alt) * 0.01;  // metres
                 if (bit32Extract(fr_payload,24,1) == 1) {
                   fr_fhome_alt *=  -1;
                 }
                 fr_home_arrow = bit32Extract(fr_payload, 25,  7);    
                 fr_home_angle =  fr_home_arrow * 3;
                   
                 Home_5004_flag = true;
                 #if (defined Frs_Debug_All) || (defined Frs_Debug_Home)                    
                   Debug.print("FrSky 5004: Dist to home=");
                   Debug.print(fr_fhome_dist, 1);              
                   Debug.print(" home_alt=");
                   Debug.print(fr_fhome_alt, 1);    // This is correct but fluctuates slightly. Can be negative
                   Debug.print(" home_arrow="); 
                   Debug.print(fr_home_arrow);    // degrees                      
                   Debug.print(" home_angle="); 
                   Debug.println(fr_home_angle);    // degrees
           
                 #endif
                 break;                        
               case 0x5005:                    // Vert and Horiz Velocity and Yaw angle (Heading) 2 Hz
                 fr_vx = bit32Extract(fr_payload,1,7) * TenToPwr(bit32Extract(fr_payload,0,1));
                 if (bit32Extract(fr_payload,8,1) == 1) {
                   fr_vx *= -1;
                 }

                 fr_vy = bit32Extract(fr_payload,10,7) * TenToPwr(bit32Extract(fr_payload,9,1));
                 fr_yaw = bit32Extract(fr_payload,17,11) * 0.2;
    
                 velyaw_5005_flag = true;        
                 #if (defined Frs_Debug_All) || (defined Frs_Debug_VelYaw)           
                   Debug.print("FrSky 5005: v_veloc=");
                   Debug.print(fr_vx,1);        // m/s
                   Debug.print(" h_veloc=");
                   Debug.print(fr_vy,1);        // m/s       
                   Debug.print(" yaw_angle="); 
                   Debug.println(fr_yaw,1);    // degrees
                 #endif
                   
                 break; 
               case 0x5006:                         // Roll, Pitch and Range - Max Hz      
                 fr_roll = bit32Extract(fr_payload,0,11);        
                 fr_froll = (float)(fr_roll - 900) * 0.2;             //  -- roll [0,1800] ==> [-180,180] 
                 fr_pitch = bit32Extract(fr_payload,11,10);   
                 fr_fpitch = (float)(fr_pitch - 450) * 0.2;           //  -- pitch [0,900] ==> [-90,90]
                 fr_frange = bit32Extract(fr_payload,22,10) * TenToPwr(bit32Extract(fr_payload,21,1));

                 AT_5006_flag = true;
                 
                #if (defined Frs_Debug_All) || (defined Frs_Debug_AttiRange)                 
                  Debug.print("Frsky 5006: Range=");
                  Debug.print(fr_frange,2);
                  Debug.print(" Roll=");
                  Debug.print(fr_froll, 2);
                  Debug.print("deg   Pitch=");
                  Debug.print(fr_fpitch, 2);   
                  Debug.println("deg");               
                #endif
                break;                                         
              case 0x5007:                         // Parameters
                fr_param_id = bit32Extract(fr_payload,24,4);
                fr_param_val = bit32Extract(fr_payload,0,24);
                if (fr_param_id == 1) {
                  fr_frame_type = fr_param_val;
                  Param_50071_flag = true;
                  #if (defined Frs_Debug_All) || (defined Frs_Debug_Params)                
                    Debug.print("Frsky 5007: Frame_type=");
                    Debug.println(fr_frame_type);
                  #endif   
                }
                else if (fr_param_id == 2) {
                  fr_bat_failsafe_volts = fr_param_val;
                  Param_50072_flag = true;
                  #if (defined Frs_Debug_All) || (defined Frs_Debug_Params)                   
                    Debug.print(" Bat failsafe volts=");
                    Debug.println(fr_bat_failsafe_volts);
                  #endif    
                }
                else if (fr_param_id == 3) {
                  fr_bat_failsafe_mAh = fr_param_val;
                  Param_50073_flag = true;
                  #if (defined Frs_Debug_All) || (defined Frs_Debug_Params)                   
                    Debug.print("Frsky 5007: Bat failsafe mAh=");
                    Debug.println(fr_bat_failsafe_mAh);   
                  #endif        
                }
                else if (fr_param_id== 4) {
                  fr_bat1_capacity = fr_param_val;
                  Param_50074_flag = true;
                     Debug.print("Frsky 5007: Bat1 capacity=");
                     Debug.println(fr_bat1_capacity);
                }         
                else if (fr_param_id == 5) {
                  fr_bat2_capacity = fr_param_val;
                  Param_50075_flag = true;
                  #if (defined Frs_Debug_All) || (defined Frs_Debug_Params)                   
                    Debug.print("Frsky 5007: Bat2 capacity=");
                    Debug.println(fr_bat2_capacity); 
                  #endif  
                }
                   
                Param_5007_flag = true;

                break;    
              case 0x5008:                         // Battery 2
                fr_bat2_volts = bit32Extract(fr_payload,0,9);
                fr_bat2_amps = bit32Extract(fr_payload,10,7)  * TenToPwr(bit32Extract(fr_payload,9,1));
                fr_bat2_mAh = bit32Extract(fr_payload,17,15);
                #if (defined Frs_Debug_All) || (defined Frs_Debug_Batteries)    
                  Debug.print("FrSky 5008: Battery2 Volts=");
                  Debug.print(fr_bat2_volts, 1);
                  Debug.print("  Battery2 Amps=");
                  Debug.print(fr_bat2_amps, 1);
                  Debug.print("  Battery2 mAh=");
                  Debug.println(fr_bat2_mAh); 
                #endif        
                Param_5008_flag = true;  
                break;       
      }
}
//=================================================================================================  
void DecodeMavlink() {
 
//  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(Serial2.available() > 0 ) 
  {
    uint8_t c = Serial2.read();
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &S2Gmsg, &status)) {
      // Handle message
 
      switch(S2Gmsg.msgid)
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
//==================================================== 
void EncodeAndWriteMavlink() {     
  
  // Heartbeat #00

  // sys_status message #1
  if (Bat_5003_flag) { 
    ap_voltage_battery1 = fr_bat1_volts * 100;  // mV for Mavlink, decivolts for Frsky
    ap_current_battery1 = fr_bat1_amps;         //  in 10*milliamperes (1 = 10 milliampere) for Mavlink = deciamps for Frsky
    ap1_battery_remaining = fr_bat1_mAh;        // mAh for Mavlink
                   
    mavlink_msg_sys_status_pack(ap_sysid, ap_compid, &S2Gmsg, ap_onboard_control_sensors_present, ap_onboard_control_sensors_enabled, 
      ap_onboard_control_sensors_health, ap_load, ap_voltage_battery1, ap_current_battery1, ap1_battery_remaining, ap_drop_rate_comm, ap_errors_comm, 
      ap_errors_count1, ap_errors_count2, ap_errors_count3, ap_errors_count4);
   len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
   Write_To_GCS(); 
   Bat_5003_flag=false; 
   #if (defined Mav_Debug_All) || (defined Mav_Debug_SysStatus)   
     Debug.print(" Mavlink #1: Bat volts=");
     Debug.print(ap_voltage_battery1);
     Debug.print(" Bat amps=");
     Debug.print(ap_current_battery1);        
     Debug.print(" Bat remain=");
     Debug.println(ap1_battery_remaining);
   #endif     
  }
  //static inline uint16_t mavlink_msg_gps_raw_int_pack(uint8_t ap_sysid, uint8_t ap_compid, mavlink_message_t* msg,
  //                           uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, 
  //                           uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid,
  //                           uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw)
  // GPS_RAW_INT #24
  if (GPS_5002_flag && velyaw_5005_flag) {  
    ap_fixtype = fr_gps_status;
    ap_sat_visible = fr_numsats;  
    ap_lat24 = fr_flat * 1E7;
    ap_lon24 = fr_flon * 1E7;
    ap_amsl24 = fr_amsl;
    ap_eph = fr_hdop;
    ap_epv = 0;             // no fr_vdop
    ap_vel = fr_vy;         // h vel
    ap_cog = fr_yaw * 100;  //  direction of movement) in degrees * 100 - yaw will have to do for now

    mavlink_msg_gps_raw_int_pack(ap_sysid, ap_compid,  &S2Gmsg,
        ap24_utime_usec, ap_fixtype, ap_lat24, ap_lon24, ap_amsl24, ap_eph, ap_epv, ap_vel, ap_cog, ap_sat_visible, 0, 0, 0, 0, 5, 0 );
    len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
    Write_To_GCS(); 
  //  GPS_5002_flag=false;       // Done lower at #74
  //  velyaw_5005_flag=false;    // Done lower at #74
   #if (defined Mav_Debug_All) || (defined Mav_Debug_GPS_Raw)  
     Debug.print(" Mavlink #24: fix_type=");
     Debug.print(ap_fixtype);
     Debug.print(" ap_amsl24=");
     Debug.print(ap_amsl24);        
     Debug.print(" eph=");
     Debug.print(ap_eph);
     Debug.print(" epv=");
     Debug.print(ap_epv);    
     Debug.print(" cog=");
     Debug.print(ap_cog);    // Course over ground - GPS direction of travel - black line
     Debug.print(" sats vis=");
     Debug.println(ap_sat_visible);       
   #endif
  }

 // ATTITUDE #30   Max Hz
 if (AT_5006_flag) { 
   ap30_time_boot_ms = millis();   // Timestamp (time since system boot)
   ap_roll = fr_froll/180*PI;       // Degrees to radians for Mavlink
   ap_pitch = fr_fpitch/180*PI; 
   ap_yaw = fr_yaw/180*PI;         // Yaw angle (-pi..+pi)
   mavlink_msg_attitude_pack(ap_sysid, ap_compid, &S2Gmsg, ap30_time_boot_ms, ap_roll, ap_pitch, ap_yaw, ap_rollspeed, ap_pitchspeed, ap_yawspeed);
   len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
   Write_To_GCS(); 
   #if (defined Mav_Debug_All) || (defined Mav_Debug_Attitude) 
     Debug.print(" Mavlink #30: roll=");
     Debug.print(ap_roll, 2);
     Debug.print("rad   pitch=");
     Debug.print(ap_pitch, 2); 
     Debug.println("rad");       
   #endif
   AT_5006_flag=false;
 }
  
  // Global Position Int #33                    lat = fr_flat * 1E7;  // expressed as degrees * 1E7
  
  if (latlon_800_flag && GPS_5002_flag && Home_5004_flag && velyaw_5005_flag) { 
    ap33_time_boot_ms = millis(); // Timestamp (time since system boot)
    ap_lat33= fr_flat * 1E7;
    ap_lon33 = fr_flon * 1E7;
    ap_amsl33 = fr_amsl;
    ap_alt_ag = fr_fhome_alt;   // expressed as * 1000 (millimeters) can be negative
    ap_vx = fr_vx;              // expressed as m/s * 100
    ap_vy = fr_vy;              // horiz vel expressed as m/s * 100
    ap_vz = 0;                  // Ground Z Speed (Altitude, positive down)
    ap_gps_hdg = fr_yaw;        // compass heading in degrees * 100, 0.0..359.99 degrees
    
    mavlink_msg_global_position_int_pack(ap_sysid, ap_compid, &S2Gmsg, 
      ap33_time_boot_ms, ap_lat33, ap_lon33, ap_amsl33, ap_alt_ag, ap_vx, ap_vy, ap_vz, ap_gps_hdg);
    len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
    Write_To_GCS(); 

    // Expire the flags
    latlon_800_flag=false; lat_800_flag = false; lon_800_flag = false;
    //  GPS_5002_flag=false;       // Done lower at #74
    Home_5004_flag=false;
    //  velyaw_5005_flag=false;    // Done lower at #74
    #if (defined Mav_Debug_All) || (defined Mav_Debug_GPS_Int)  
      Debug.print(" Mavlink #33: lat=");
      Debug.print(ap_lat33);
      Debug.print(" lon=");
      Debug.print(ap_lon33);        
      Debug.print(" ap_amsl33=");
      Debug.print(ap_amsl33);
      Debug.print(" ap_alt_ag=");
      Debug.print(ap_alt_ag);    
      Debug.print(" vx=");
      Debug.print(ap_vx);
      Debug.print(" vy=");
      Debug.print(ap_vy);    
      Debug.print(" yaw(hdg)=");
      Debug.println(ap_gps_hdg);       
    #endif
  }

  // VFR_HUD ( #74 )
  if (velyaw_5005_flag && GPS_5002_flag) {
    ap_hud_air_spd = fr_vx * 0.1;      // Current airspeed in m/s - Frsky decimeters/s
    ap_hud_grd_spd = fr_vx * 0.1;      // Current ground speed in m/s - Frsky decimeters/s
    ap_hud_amsl = fr_famsl * 0.1;      // Current altitude (MSL) in meters - Frsky decimeters
    ap_hud_climb = fr_vy * 0.1;        // Current climb rate in meters/second - Frsky decimeters/s
    ap_hud_hdg = fr_yaw;               // Current heading in degrees, in compass units (0..360, 0=north)
    ap_hud_throt = 0;                  // Current throttle setting in integer percent, 0 to 100 - Don't have it :(

    mavlink_msg_vfr_hud_pack(ap_sysid, ap_compid, &S2Gmsg,
                   ap_hud_air_spd, ap_hud_grd_spd, ap_hud_hdg, ap_hud_throt, ap_hud_amsl, ap_hud_climb);
    len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
    Write_To_GCS(); 
    #if (defined Mav_Debug_All) || (defined Mav_Debug_Hud)
      Debug.print(" Mavlink #74: airspeed=");
      Debug.print(ap_hud_air_spd, 1);
      Debug.print(" groundspeed=");
      Debug.print(ap_hud_grd_spd, 1);        
      Debug.print(" alt (msl)=");
      Debug.print(ap_hud_amsl, 1);
      Debug.print(" climb=");
      Debug.print(ap_hud_climb, 1);    
      Debug.print(" heading=");
      Debug.print(ap_hud_hdg);
      Debug.print(" throttle=");
      Debug.println(ap_hud_throt);    
    #endif        
    GPS_5002_flag=false;
    velyaw_5005_flag=false; 
  }

  // STATUSTEXT ( #253 )
  if (ST_5000_flag){ 
    ap_severity =  fr_severity;
    strcpy(ap_text, fr_text);   // 0-48
//static inline uint16_t mavlink_msg_statustext_pack(uint8_t ap_sysid, uint8_t ap_compid, mavlink_message_t* S2Gmsg,
//                             uint8_t severity, const char *text, uint16_t id, uint8_t chunk_seq)
    mavlink_msg_statustext_pack(ap_sysid, ap_compid, &S2Gmsg, ap_severity, ap_text, 1, 1);
    len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
    Write_To_GCS();  
    #if (defined Mav_Debug_All) || (defined Mav_Debug_StatusText)
      Debug.print(" Mavlink #253: Severity=");
      Debug.print(ap_severity);
      Debug.print(" Msg : ");  
      Debug.println(ap_text);
    #endif
   fr_text[0]=0;
   ST_5000_flag=false;                   
  }
}
//=================================================================================================  
void ServiceStatusLed() {
  if (frsGood) {

      if (InvertFrsLed) {
       FrsLedState = LOW;
      } else {
       FrsLedState = HIGH;
      }
      digitalWrite(FrsStatusLed, FrsLedState); 
  }
    else {
      BlinkFrsLed(500);
    }
  digitalWrite(FrsStatusLed, FrsLedState); 
}

//=================================================================================================  
void BlinkLed(uint16_t rate) {
  unsigned long cMillis = millis();
     if (cMillis - ledMillis >= rate) {    // blink period
        ledMillis = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}

//=================================================================================================  

uint32_t Unpack_uint32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = frbuf[posn+3];
   byte b2 = frbuf[posn+2];
   byte b3 = frbuf[posn+1];
   byte b4 = frbuf[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
    
    // Now combine the four bytes into an unsigned 32bit integer

   uint32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//=================================================================================================  
int32_t Unpack_int32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = frbuf[posn+3];
   byte b2 = frbuf[posn+2];
   byte b3 = frbuf[posn+1];
   byte b4 = frbuf[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
   
 // Now combine the four bytes into an unsigned 32bit integer
 
   int32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//=================================================================================================  
uint16_t Unpack_uint16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

   byte b1 = frbuf[posn+1];
   byte b2 = frbuf[posn];  
    
    // Now convert the 2 bytes into an unsigned 16bit integer
    
    uint16_t myvar = b1 << 8 | b2;
    return myvar;
}
//=================================================================================================  
int16_t Unpack_int16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
   byte b1 = frbuf[posn+1];
   byte b2 = frbuf[posn];
    
    // Now convert the 2 bytes into a signed 16bit integer
    
    int16_t myvar = b1 << 8 | b2;
    return myvar;
}
//=================================================================================================  
uint8_t Unpack_uint8 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is one byte long

  byte b1 = frbuf[posn];
    
    // Now convert the byte into an unsigned 8 bit integer
    
   uint8_t myvar = b1;
   return myvar;
}
//=================================================================================================  
void PrintTheBuffer(int lth){
  for ( int i = 0; i < lth; i++ ) {
    byte b = frbuf[i];
    PrintByte(b);
  }
  Debug.println();
}
//=================================================================================================  
void PrintThePayload(){
  for ( int i = 0; i < 10; i++ ) {
    byte b = frbuf[i];
    if (b<=0xf) Debug.print("0");
    Debug.print(b,HEX);
    Debug.print(" ");
  }
  Debug.print( "  Payload= ");
  for ( int i = 8; i > 4; i-- ) {
    byte b = frbuf[i];
    if (b<=0xf) Debug.print("0");
    Debug.print(b,HEX);
    Debug.print(" ");
  }
  Debug.println();

}
//=================================================================================================  
void PrintField(int pos, int lth){
  for ( int i = pos; i < pos+lth; i++ ) {
    Debug.print(frbuf[i],HEX);
    Debug.print(" ");
  }
  Debug.print("// ");
}
//=================================================================================================  
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
//=================================================================================================  

uint8_t Unpack8 (int posn){
  
    uint8_t myvar = frbuf[posn];
    return myvar;
}
//=================================================================================================  
void ShowElapsed() {
  Debug.print(" Seconds=");
  unsigned long millnow=millis();
  float fSecs = millnow / 1000;
  Debug.print(fSecs,1);
  Debug.print(" ");
}
//=================================================================================================  
void PrintMsg (const void *object){

  const unsigned char * const chars = static_cast<const unsigned char *>(object);

  for ( int i = 21; i < 30; i++ ) {
    Debug.print(chars[i]);
  }
  Debug.println();
}
 //================================================================================================= 

void Write_To_GCS() {  

  if ((set.gs_io == gs_ser) || (set.gs_io == gs_bt) || (set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.gs_sd == gs_on)) {

   if (set.gs_io == gs_ser) {  // Serial
     len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
     #ifdef  Debug_GCS_Down
       Debug.println("Passed down to GCS by Serial:");
       PrintMavBuffer(&S2Gmsg);
     #endif
      mvSerialGCS.write(GCSbuf,len);  
   }

  #if (defined btBuiltin)
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt))  {  // Bluetooth
      len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);     
      #ifdef  Debug_GCS_Down
        Debug.println("Passed down to GCS by Bluetooth:");
        PrintMavBuffer(&S2Gmsg);
      #endif
      if (SerialBT.hasClient()) {
        SerialBT.write(GCSbuf,len);
      }
    }
  #endif

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt)) { //  WiFi
    
      if (wifiSuGood) {
           
        if (set.wfproto == tcp)  { // TCP  
          bool msgSent = Send_TCP(&S2Gmsg);  // to GCS
          msgSent = msgSent; // stop stupid compiler warnings         
          #ifdef  Debug_GCS_Down
            Debug.print("Passed down to GCS by WiFi TCP: msgSent="); Debug.println(msgSent);
            PrintMavBuffer(&S2Gmsg);
          #endif
        }
        
        if (set.wfproto == udp)  { // UDP 
          bool msgSent = Send_UDP(&S2Gmsg);  // to GCS
          msgSent = msgSent; // stop stupid compiler warnings
          #ifdef  Debug_GCS_Down
            Debug.print("Passed down to GCS by WiFi UDP: msgSent="); Debug.println(msgSent);
            PrintMavBuffer(&S2Gmsg);
          #endif                 
        }                                                                     
      }  
    }
  #endif

  #if ((defined ESP32) || (defined ESP8266)) && (defined SD_Support) 
    if  (set.gs_sd == gs_on) {   //  SD Card
      if (sdStatus == 3) {     //  if open for write
          File file = SD.open(cPath, FILE_APPEND);
          if(!file){
             Debug.println("Failed to open file for appending");
             sdStatus = 9;
             return;
            }

         memcpy(GCSbuf, (void*)&ap_time_unix_usec, sizeof(uint64_t));
         len=mavlink_msg_to_send_buffer(GCSbuf+sizeof(uint64_t), S2Gmsg);

         if(file.write(GCSbuf, len+18)){   // 8 bytes plus some head room   
            } else {
            Debug.println("Append failed");
           }
         
          file.close();
        
          #ifdef  Debug_SD
            Debug.println("Passed down to SD:");
            PrintMavBuffer(&S2Gmsg);
          #endif        
        }  
    }   
  #endif 
  }
}

//================================================================================================= 
#if (defined btBuiltin)
  bool Send_Bluetooth(mavlink_message_t* msgptr) {

    bool msgSent = false;
    uint8_t buf[300];
     
    uint16_t len = mavlink_msg_to_send_buffer(buf, msgptr);
  
    size_t sent = SerialBT.write(buf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
  }
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool Send_TCP(mavlink_message_t* msgptr) {   
    if (!wifiSuGood) return false;  

    bool msgSent = false;
    uint8_t buf[300];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msgptr);
  
    size_t sent =  WiFiSTA.write(buf,len);  

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
  }
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool Send_UDP(mavlink_message_t* msgptr) {
    if (!wifiSuGood) return false;  
    bool msgSent = false;
    uint8_t buf[300];

    UDP.beginPacket(udp_remoteIP, set.udp_remotePort);

    uint16_t len = mavlink_msg_to_send_buffer(buf, msgptr);
  
    size_t sent = UDP.write(buf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    UDP.endPacket();
    return msgSent;
  }
#endif
//================================================================================================= 
void Read_From_GCS() {
  
  #if defined Enable_GCS_Serial  // only these have a 4th uart
    if (set.gs_io == gs_ser)  {  // Serial 
      mavlink_status_t status;
      while(mvSerialGCS.available()) { 
        uint8_t c = mvSerialGCS.read();
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &G2Smsg, &status)) {  // Read a frame from GCS  
          GCS_available = true;  // Record waiting
          #ifdef  Debug_GCS_Up
            Debug.println("Passed up from GCS Serial to G2Smsg:");
            PrintMavBuffer(&G2Smsg);
          #endif     
        }
      } 
     } 
  #endif

  #if (defined btBuiltin) 
    if ((set.gs_io == gs_bt) || (set.gs_io == gs_wifi_bt)) {  // Bluetooth
 
       bool msgRcvdBT = Read_Bluetooth(&G2Smsg);

       if (msgRcvdBT) {
          GCS_available = true;  // Record waiting to go to FC 
          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS BT to G2Smsg: msgRcvdBT=" ); Debug.println(msgRcvdBT);
            PrintMavBuffer(&G2Smsg);
          #endif      
        }
    }  
  #endif

  #if (defined wifiBuiltin)
    if ((set.gs_io == gs_wifi) || (set.gs_io == gs_wifi_bt) || (set.web_support)) {   //  WiFi
    
      if (set.wfproto == tcp)  { // TCP 
    
        bool msgRcvdWF = Read_TCP(&G2Smsg);

        if (msgRcvdWF) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS WiFi TCP to G2Smsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
            PrintMavBuffer(&G2Smsg);
          #endif      
        }
      }
      
      if (set.wfproto == udp)  { // UDP from GCS
        bool msgRcvdWF = Read_UDP(&G2Smsg);

        if (msgRcvdWF) {
          GCS_available = true;  // Record waiting to go to FC 

          #ifdef  Debug_GCS_Up    
            Debug.print("Passed up from GCS WiFi UDP to G2Smsg: msgRcvdWF=" ); Debug.println(msgRcvdWF);
            PrintMavBuffer(&G2Smsg);
          #endif      
        }   
      } 
    }
  #endif  
}

//================================================================================================= 
#if (defined btBuiltin)
  bool Read_Bluetooth(mavlink_message_t* msgptr)  {
    
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = SerialBT.available();
    uint16_t bt_count = len;
    if(bt_count > 0) {

        while(bt_count--)  {
            int result = SerialBT.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {

                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgptr->sysid;
                            hb_comp_id        = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgptr);
                    }
                 
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
}
#endif
//================================================================================================= 
#if (defined wifiBuiltin)
  bool Read_TCP(mavlink_message_t* msgptr)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = WiFiSTA.available();
    uint16_t tcp_count = len;
    if(tcp_count > 0) {

        while(tcp_count--)  {
            int result = WiFiSTA.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {

                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgptr->sysid;
                            hb_comp_id        = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgptr);
                    }
                 
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
  }
#endif

//================================================================================================= 
#if (defined wifiBuiltin)
  bool Read_UDP(mavlink_message_t* msgptr)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;

    len = UDP.parsePacket();
    #if defined Debug_GCS_Up  
      if (len > 0) Debug.printf("Read_UDP() - len=%d\n", len);
    #endif  
    int udp_count = len;
    if(udp_count > 0) {

        while(udp_count--)  {

            int result = UDP.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {
                  
                    udp_remoteIP = UDP.remoteIP();  // remember which remote client sent this packet so we can target it
                    PrintRemoteIP();
                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from      = true;
                            hb_system_id       = msgptr->sysid;
                            hb_comp_id         = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgptr);
                    }
                    
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
  }
#endif

//================================================================================================= 
#if (defined wifiBuiltin)
  void checkLinkErrors(mavlink_message_t* msgptr)   {

    //-- Don't bother if it's not our sys/comp ids)
    if(msgptr->sysid != hb_system_id || msgptr->compid != hb_comp_id) {
        return;
    }
    uint16_t seq_received = (uint16_t)msgptr->seq;
    uint16_t packet_lost_count = 0;
    //-- Account for overflow during packet loss
    if(seq_received < hb_seq_expected) {
        packet_lost_count = (seq_received + 255) - hb_seq_expected;
    } else {
        packet_lost_count = seq_received - hb_seq_expected;
    }
    hb_seq_expected = msgptr->seq + 1;
    link_status.packets_lost += packet_lost_count;
  }
#endif
//================================================================================================= 


//=================================================================================================  
void SendGCSHeartbeat() {     // To GCS
  /*
   Debug.print("Heartbeat 1    millis=");
   Debug.print(millis());
   Debug.print("   hb_millis=");
   Debug.println(hb_millis);

   static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
   */
  if (millis() > (hb_millis + hb_period)){  
    // Debug.println("Send Heartbeat");
    // Pack the message
    mavlink_msg_heartbeat_pack(ap_sysid, ap_compid, &S2Gmsg, gcs_type, gcs_autopilot, gcs_base_mode, gcs_custom_mode, gcs_system_status);
    // Copy the message to the send buffer
    len = mavlink_msg_to_send_buffer(GCSbuf, &S2Gmsg);
    // Send the message
    Write_To_GCS();
    hb_millis=millis();                              
  }
}

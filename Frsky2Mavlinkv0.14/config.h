//================================================================================================= 
//================================================================================================= 
//
//                                    C O N F I G U R A T I O N 
// 
//================================================================================================= 
//================================================================================================= 

/*
Complete change log and debugging options are at the bottom of this tab

v0.13 Supports WiFi and Bluetooth. ALPHA software, likely to be buggy. Feedback please.
                      
*/
//===========================================================================================
//
//                   PLEASE SELECT YOUR DEFAULT OPTIONS BELOW BEFORE COMPILING
//
//===========================================================================================

#define webSupport                      // ESP only. Enable wifi web support, including OTA firmware updating. Browse to IP.
#define webPassword      "changeme!"    // Web password 
//#define Reset_Web_Defaults            // Settings in eeprom. Do this if eeprom settings are corrupt or wrong.

//#define SD_Support                    // Enable if you have an SD card reader attached
#define OLED_Support                  // Enable if you have an OLED display attached
                                 
#define frBaud                 57600    // S.Port baud setting - default 57600 

#define ESP_SPort_Invert                // Does what it says

#define PlusVersion  // Added support for 0x5009 Mission WPs, 0x50F1 Servo_Channels, 0x50F2 VFR_Hud


//=================================================================================================
//           D E F A U L T   G R O U N D S T A T I O N    I / O   S E T T I N G S   
//=================================================================================================
// Choose only one of these default GCS-side I/O channels
// How does Mavlink telemetry leave this translator?

#define GCS_Mavlink_IO  0    // Serial Port2     
//#define GCS_Mavlink_IO  1    // BlueTooth Classic - ESP32 only
#define GCS_Mavlink_IO  2    // WiFi - ESP32 or ESP8266 only - auto selects on ESP8266
//#define GCS_Mavlink_IO  3    // WiFi AND Bluetooth simultaneously - ESP32 only

// NOTE: The Bluetooth class library uses a lot of application memory. During Compile/Flash
//       you may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...) or similar

//#define GCS_Mavlink_SD       // SD Card - ESP32 only - mutually inclusive with other GCS I/O

//=================================================================================================                             
//                          S E L E C T   E S P   B O A R D   V A R I A N T   
//=================================================================================================

#define ESP32_Variant     1    //  ESP32 Dev Module - Use Partition Scheme: "Minimal SPIFFS(1.9MB APP...)"
//#define ESP32_Variant     2    //  Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
//#define ESP32_Variant     3    //  Dragonlink V3 slim with internal ESP32 - contributed by Noircogi
//#define ESP32_Variant     4    //  Heltec Wifi Kit 32 - Use Partition Scheme: "Minimal SPIFFS(Large APPS ith OTA)" - contributed by Noircogi

#define ESP8266_Variant   1   // NodeMCU ESP 12F - choose "NodeMCU 1.0(ESP-12E)" board in the IDE
//#define ESP8266_Variant   2   // ESP-12E, ESP-F barebones boards. RFD900X TX-MOD, QLRS et al - use Generic ESP8266 on IDE


//=================================================================================================
//                      D E F A U L T   B L U E T O O T H   S E T T I N G S   
//=================================================================================================

#define BT_Mode  1           // Master Mode - we advertise the "host" name and slave connects to us
//#define BT_Mode  2           // Slave Mode - we connect as slave to a master
#define BT_ConnectToSlave     "Crossfire 0277"  // Example


//=================================================================================================
//                            D E F A U L T    W  I  F  I    S E T T I N G S   
//=================================================================================================

#define HostName             "PassToMav"            // This translator's host name
#define APssid               "PassthruToMavlink"    // The AP SSID that we advertise   ====>
#define APpw                 "password"             // Change me!
#define APchannel            9                      // The wifi channel to use for our AP
#define STAssid              "OmegaOffice"          // Target AP to connect to         <====
#define STApw                "changeme!"            // Target AP password        

#define Start_WiFi                       // Start WiFi at startup, override startWiFi Pin

// Choose one default mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
//#define WiFi_Mode   1  //AP            
//#define WiFi_Mode   2  // STA
#define WiFi_Mode   3  // STA failover to AP

// Choose one default protocol - for ESP32 only
//#define WiFi_Protocol 1    // TCP/IP
#define WiFi_Protocol 2    // UDP 


//=================================================================================================
//                            O T H E R   U S E R   O P T I O N S  
//=================================================================================================

#define SPort_Serial        1         // Teensy port1=pin1, port3=pin8. The default is Serial 1, but 3 is possible 


//================================== Set your time zone here ======================================
// Only for SD / TF Card adapter option
// Date and time determines the TLog file name only
//const float Time_Zone = 10.5;    // Adelaide, Australia
const float Time_Zone = 2.0;    // Jo'burg
bool daylightSaving = false;

//=================================================================================================
//                        E X P E R I M E N T A L    O P T I O N S  
//    Don't change anything here unless you are confident you know the outcome

//#define ESP32_SoftwareSerial    // otherwise HardwareSerial is used 
//#define ESP_Onewire             // enable half_duplex on single (tx) pin and wire 
//#define ESP_Air_Relay_Blind_Inject_OK  // Blind inject instead of interleaving

//=================================================================================================   
//                              Auto Determine Target Platform
//================================================================================================= 
//
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define TEENSY3X   
      
#elif defined ESP32
  #define Target_Board   3      // Espressif ESP32 Dev Module

#elif defined ESP8266
  #define Target_Board   4      // Espressif ESP8266
  
#else
  #error "Unsupported board type!"
#endif

//=================================================================================================   
//                              CHECK #define OPTIONS LOGIC
//================================================================================================= 

#ifndef GCS_Mavlink_IO
  error Please define an output channel
#endif

#if defined PlusVersion                 
  #define Request_Mission_Count_From_FC // Needed for yaapu's mission/waypoint script
#endif

#if (not defined ESP32) && (not defined ESP8266)
  #if defined webSupport
    #undef webSupport
    //    #error webSupport only available on ESP32 or ESP8266
  #endif
#endif
  
#if defined ESP32
  #include <iostream> 
  #include <sstream> 
  #include <driver/uart.h>  // In Arduino ESP32 install repo 
  //C:\Users\<YourName>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\tools\sdk\include\driver

#endif
  
  #if (defined ESP32) || (defined ESP8266)  // ESP32 or ESP8266 (UART0, UART1, and UART2)
    #if (SPort_Serial  == 3)    
      #error Board does not have Serial3. This configuration is not possible.
    #endif
  #endif

  #if (defined TEENSY3X) 
    #if (FC_Mavlink_IO == 3) || (defined GCS_Mavlink_SD)  
      #error SD card not currently implemented for Teensy
    #endif
  #endif

  #if (not defined ESP32) && (not defined ESP8266) 
     #if (GCS_Mavlink_IO == 2) || (GCS_Mavlink_IO == 3) || (defined webSupport)
    //   #error WiFi and webSupport only work on an ESP32 or ESP8266 board
       #if defined webSupport
         #undef  webSupport
       #endif  
     #endif  
  #endif

  #if (defined ESP8266)  && ((GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3))  // Can't do BT on 8266
      #undef GCS_Mavlink_IO
      #define GCS_Mavlink_IO  2    // WiFi Only
  #endif
         
  #if (not defined ESP32) 
     #if (FC_Mavlink_IO == 1) || (GCS_Mavlink_IO == 1) || (GCS_Mavlink_IO == 3)
       #error Bluetooth works only on an ESP32 board      
     #endif  
  #endif


  #if (defined ESP32)
    #ifndef WiFi_Mode 
      #error Please define WiFi_Mode
    #endif
  #endif  

  #if (defined ESP32)
    #ifndef WiFi_Protocol
      #error Please define WiFi_Protocol
    #endif
  #endif

  #if (defined ESP32)         
    #ifndef ESP32_Variant 
      #error Please define an ESP32 board variant
    #endif
  #endif

  #if (defined ESP8266)
    #ifndef ESP8266_Variant
         #error Please define an ESP8266 board variant
    #endif
  #endif  

  #if (defined ESP32 || defined ESP8266) && (GCS_Mavlink_IO == 2 || GCS_Mavlink_IO == 3 || defined webSupport)
    #define wifiBuiltin   //  for these features we need wifi support compiled in
  #endif    

  #if (defined ESP32) && (GCS_Mavlink_IO == 1 || GCS_Mavlink_IO == 3)
    #define btBuiltin   //  for these features we need bluetooth support compiled in
  #endif     

//=================================================================================================   
//                          P L A T F O R M   D E P E N D E N T   S E T U P S
//================================================================================================= 

  
#if defined TEENSY3X               // Teensy3x
  #define FrsStatusLed  13
  #define InvertFrsLed false   
  #define BufStatusLed  14        
  #define FC_Mav_rxPin  9  
  #define FC_Mav_txPin  10
  #if (SPort_Serial == 1)
    #define Fr_txPin       1      // SPort tx - Use me in single wire mode 
    #define GC_Mav_rxPin   7    
    #define GC_Mav_txPin   8   
  #elif (SPort_Serial == 3)
    define Fr_txPin        8      // Optional SPort tx 
  #endif  
    
  #if (defined SD_Support) || (defined OLED_Support)
  #endif
 
#elif defined ESP32                 // ESP32 Platform

  #if (ESP32_Variant == 1)          // ESP32 Dev Module
    #define FrsStatusLed  02        // Onboard LED
    #define InvertFrsLed false      
    #define BufStatusLed  27        // untested pin      
    #define GC_Mav_rxPin  16        // Mavlink to GCS
    #define GC_Mav_txPin  17        // Mavlink from GCS
    #define Fr_rxPin      13        // SPort - Not used in 1-wire mode
    #define Fr_txPin      4         // SPort tx - Use me in single wire mode
    #if (defined SD_Support) || (defined OLED_Support)   
      #define SDA           21        // I2C OLED board
      #define SCL           22        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif   
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 15;      // D15
    uint8_t WiFiPinState = 0;
    /*  
      SPI/CS                       Pin 05   For optional TF/SD Card Adapter
      SPI/MOSI                     Pin 23   For optional TF/SD Card Adapter
      SPI/MISO                     Pin 19   For optional TF/SD Card Adapter
      SPI/SCK                      Pin 18   For optional TF/SD Card Adapter  
    */

  #endif

  #if (ESP32_Variant == 2)          // Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
    #define FrsStatusLed  15        // No Onboard LED
    #define InvertFrsLed false     
    #define BufStatusLed  99        // None    
    #define GC_Mav_rxPin  25        // Mavlink to GCS
    #define GC_Mav_txPin  26        // Mavlink from GCS
    #define Fr_rxPin      12        // SPort - Not used in single wire mode
    #define Fr_txPin      14        // SPort tx - Use me in single wire mode
    #if (defined SD_Support) || (defined OLED_Support)
      #define SDA           05        // I2C OLED board
      #define SCL           04        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 13;     
    uint8_t WiFiPinState = 0;
  #endif

  #if (ESP32_Variant == 3)          // Dragonlink V3 slim with internal ESP32

    #define FrsStatusLed  18        // Blue LED
    #define InvertFrsLed false    
    #define BufStatusLed  19        // Green LED        
    #define GC_Mav_rxPin  16        // Mavlink to GCS
    #define GC_Mav_txPin  17        // Mavlink from GCS
    #define Fr_rxPin      12        // SPort - Not used in single wire mode
    #define Fr_txPin      01        // SPort tx - Use me in single wire mode
    #if (defined SD_Support) || (defined OLED_Support)
      #define SDA           05        // I2C OLED board
      #define SCL           04        // I2C OLED board 
      #define i2cAddr      0x3C       // I2C OLED board
    #endif  
    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 13;     
    uint8_t WiFiPinState = 0;
  #endif
  
  #if (ESP32_Variant == 4)          // Heltec Wifi Kit 32 (NOTE! 8MB) 
    #define FrsStatusLed  25        // Onboard LED
    #define InvertFrsLed false     
    #define BufStatusLed  99 
    #define Fr_rxPin      12        // SPort rx 
    #define Fr_txPin      14        // SPort tx - Use me in single wire mode 
    #define GC_Mav_rxPin  27        // Mavlink to GCS  - default UART2 is 16, but clashes with OLED_RESET
    #define GC_Mav_txPin  17        // Mavlink from GCS 

    
    #if !defined OLED_Support    // I2C OLED board is built into Heltec WiFi Kit 32
      #define OLED_Support
    #endif

    #if (defined SD_Support) || (defined OLED_Support)   
      #define SDA           04        // I2C OLED board 
      #define SCL           15        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
      #define OLED_RESET    16        // RESET here so no reset lower down    
    #endif   


    int16_t wifi_rssi;    
    uint8_t startWiFiPin = 13;     
    uint8_t WiFiPinState = 0;
  #endif
  
#elif defined ESP8266                    // ESP8266 Platform

  #if (ESP8266_Variant == 1)        // NodeMCU 12F board - Dev board with usb etc
  
    #define FrsStatusLed  D4        // D4 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertFrsLed true      
    #define BufStatusLed  99        // None     
   //                     D4        // TXD1 - Serial1 debug log out SHARED WITH BOARD LED                         
    #define GC_Mav_rxPin  D9        // RXD0 default  
    #define GC_Mav_txPin  D10       // TXD0 default    
    #define Fr_rxPin      D5        // SPort - Not used in single wire mode
    #define Fr_txPin      D6        // SPort tx - Use me in single wire mode
    #if (defined SD_Support) || (defined OLED_Support)  
      #define SCL           D1        // I2C OLED board   
      #define SDA           D2        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif     
    int16_t wifi_rssi;   
    uint8_t startWiFiPin = D3;      
    uint8_t WiFiPinState = 0;
  #endif
  
  #if (ESP8266_Variant == 2)   // ESP-12E, ESP-F barebones boards. RFD900X TX-MOD, QLRS et al - use Generic ESP8266 on IDE
    //                         GPIO as per node mcu
    static const uint8_t D0   = 16;   // SCL - optional
    static const uint8_t D1   = 5;    // SDA - optional
    static const uint8_t D2   = 4;    // SPort tx - Use me in single wire mode
    static const uint8_t D3   = 0;    // Flash
    static const uint8_t D4   = 2;    // BoardLED & TXD1 optional debug out
    static const uint8_t D5   = 14;   // SPort rx (unused in half-duplex)
    static const uint8_t D6   = 12;   // P2-3 exposed dual row of pins
    static const uint8_t D7   = 13;   // CTS
    static const uint8_t D8   = 15;   // RTS
    static const uint8_t D9   = 3;    // RXD0
    static const uint8_t D10  = 1;    // TXD0 
    
    #define FrsStatusLed  D4        // D4 Board LED - Mav Status LED inverted logic - use 99 while debug
    #define InvertFrsLed true    
    #define BufStatusLed  99        // None
    //                    D4        // TXD1 - Serial1 default debug log out SHARED WITH BOARD LED                           
    #define GC_Mav_rxPin  D9        // RXD0 default  
    #define GC_Mav_txPin  D10       // TXD0 default    
    #define Fr_rxPin      D5        // SPort - Not used in single wire mode
    #define Fr_txPin      D2        // SPort (half-duplex) inverted - Use me in single wire mode
    #if (defined SD_Support) || (defined OLED_Support)
      #define SCL           D0        // I2C OLED board   
      #define SDA           D1        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif       
    int16_t wifi_rssi;   
    uint8_t startWiFiPin = D8;      
    uint8_t WiFiPinState = 0;
  #endif  
  
#endif

  //=================================================================================================   
  //                            E E P R O M    S U P P O R T   -   ESP Only - for now
  //================================================================================================= 

  #if (defined ESP32) || (defined ESP8266)
    #include <EEPROM.h>       // To store AP_Failover_Flag and webSupport Settings
    #define EEPROM_SIZE 160   // 1 + 159 bytes, addr range 0 thru 160
  #endif
    
  //=================================================================================================   
  //                             S D   C A R D   S U P P O R T   -   ESP Only - for now
  //================================================================================================= 
  #if ((defined ESP32)  || (defined ESP8266)) && (defined SD_Support) 

    #include <FS.h>
    #include <SD.h>
    #include <SPI.h>
    #define SD_Libs_Loaded 

    /*
    // Optional SPI interface pins for SD card adapter or SSD1306 OLED display
    #define CS            5        
    #define MOSI          23 
    #define MISO          19 
    #define SCK           18 
    */  

    // Pins generally   CS=5    MOSI=23   MISO=19   SCK=18    3.3V   GND   Dev Board, LilyGO/TTGO
 
 
     
    // Rememeber to change SPI frequency from 4E6 to 25E6, i.e 4MHz to 25MHz in SD.h otherwise MavRingBuff fills up 
    // C:\Users\YourUserName\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.2\libraries\SD\src  
    // bool begin(uint8_t ssPin=SS, SPIClass &spi=SPI, uint32_t frequency=25000000, const char * mountpoint="/sd", uint8_t max_files=5);  

    char     cPath[40];
    std::string   fnPath[30];
    uint8_t  fnCnt;
    uint16_t sdReadDelay = 10;  // mS   Otherwise the reads run through unnaturally quickly

    File     file;  // Create global object from File class for general use

    static  const uint8_t mthdays[]={31,28,31,30,31,30,31,31,30,31,30,31}; 

    typedef struct  { 
      uint16_t yr;   // relative to 1970;  
      uint8_t mth;
      uint8_t day;
      uint8_t dow;   // sunday is day 1 
      uint8_t hh; 
      uint8_t mm; 
      uint8_t ss; 
    }   DateTime_t;

    static DateTime_t dt_tm; 

  #endif  
  //=================================================================================================   
  //                                 O L E D   S U P P O R T    E S P  O N L Y - for now
  //================================================================================================= 
  #if (defined ESP32 || defined ESP8266)  
  
    #if not defined SD_Libs_Loaded   //  by SD block
      #include <SPI.h>                // for SD card and/or OLED
      #include <Wire.h>
    #endif  

    #include <Adafruit_SSD1306.h> 
    
    #define max_col  22
    #define max_row   8
    // 8 rows of 21 characters

    struct OLED_line {
      char OLx[max_col];
      };
  
     OLED_line OL[max_row]; 

    uint8_t row = 0;
    uint8_t col = 0;
  
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
    // 8 rows of 21 characters

    // Declaration for an SSD1306 I2C display
    #ifndef OLED_RESET
      #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
    #endif  
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  #endif

  //=================================================================================================   
  //                          B L U E T O O T H   S U P P O R T -  E S P 3 2  O n l y
  //================================================================================================= 

  #if (defined btBuiltin)
    #include "BluetoothSerial.h"
    #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
      #error Bluetooth is not enabled! Please run `make menuconfig in ESP32 IDF` 
    #endif

    BluetoothSerial SerialBT;

  #endif  

  //=================================================================================================   
  //                           W I F I   S U P P O R T - ESP32 and ES8266 Only
  //================================================================================================= 

    uint16_t TCP_localPort = 5760;
    uint16_t UDP_localPort = 14555;     
    uint16_t UDP_remotePort = 14550;      
    bool FtRemIP = true;

  #if (defined wifiBuiltin)

    // Define link variables
    struct linkStatus {
      uint32_t    packets_received;
      uint32_t    packets_lost;
      uint32_t    packets_sent;
    };
    bool          hb_heard_from = false;
    uint8_t       hb_system_id = 0;
    uint8_t       hb_comp_id = 0;
    uint8_t       hb_seq_expected = 0;
    uint32_t      hb_last_heartbeat = 0;
    linkStatus    link_status;
  
    #if defined ESP32 
      #include <WiFi.h>  
      #include <WiFiClient.h>
      #if defined webSupport
        #include <WebServer.h> 
        #include <Update.h> 
        WebServer server(80);          
      #endif      
      #include <WiFiAP.h>  
    #endif

    #if defined ESP8266
      #include <ESP8266WiFi.h>   // Includes AP class
      #include <WiFiClient.h>
      #if defined webSupport
        #include <ESP8266WebServer.h>    
        ESP8266WebServer server(80);
      #endif      
    #endif
    

    #include <WiFiUdp.h>    

    WiFiClient WiFiSTA;   
       
    WiFiServer TCPserver(set.tcp_localPort);

    IPAddress udp_remoteIP(192, 168, 1, 255);    // Declare UDP broadcast on your likely LAN subnet
    WiFiUDP UDP;       // Create UDP object    
         
    IPAddress localIP;   // tcp and udp
    
  #endif  // end of ESP32 and ESP8266
  
//=================================================================================================   
//                                 S E R I A L
//=================================================================================================

#define mvBaudGCS           57600          // Default

#if (defined ESP32)  
  #define Debug               Serial         // USB
  
  #if defined ESP32_SoftwareSerial
    #include <SoftwareSerial.h>
    SoftwareSerial frSerial; 
  #else     // default HW Serial
    #define frSerial          Serial1     
  #endif  

  #define mvSerialGCS         Serial2       
    
#elif (defined ESP8266)
  #define Debug               Serial1        //  D4   TXD1 debug out  - no RXD1 !
  #define mvSerialGCS         Serial         //  RXD0 and TXD0
  #include <SoftwareDebug.h>
  SoftwareSerial frSerial;  
#endif 

#if (defined TEENSY3X)      //  Teensy 3.1
  #define Debug                   Serial         // USB  
  #if (SPort_Serial == 1) 
    #define frSerial              Serial1        // S.Port 
  #elif (SPort_Serial == 3)
    #define frSerial              Serial3        // S.Port 
 #else
    #error SPort_Serial can only be 1 or 3. Please correct.
  #endif 
  #define mvSerialGCS             Serial2     
#endif 


//=================================================================================================   
//                                 D E B U G G I N G   O P T I O N S
//=================================================================================================

//#define Mav_Debug_All
//#define Frs_Debug_All
//#define Frs_Debug_SPort
//#define Frs_Debug_Payload
//#define Frs_Debug_Period
//#define Frs_Debug_Params     //0x5007
//#define Debug_BT    
//#define Debug_GCS_Down       // traffic to GCS
//#define Debug_GCS_Up         // traffic from GCS
//#define Debug_Rssi
//#define Mav_Debug_RC
//#define Frs_Debug_RC
//#define Mav_Debug_FC_Heartbeat
//#define Mav_Debug_GCS_Heartbeat
//#define Frs_Debug_APStatus    // 0x5001
//#define Mav_Debug_SysStatus   // #1 && battery
//#define Frs_Debug_Batteries   // 0x5003 && 0x5008
//#define Frs_Debug_Home        // 0x5004
//#define Mav_Debug_GPS_Raw     // #24
//#define Mav_Debug_GPS_Int     // #33
//#define Frs_Debug_LatLon      // 0x800
//#define Frs_Debug_VelYaw      // 0x5005
//#define Frs_Debug_GPS_Status  // 0x5002
//#define Mav_Debug_Hud         // #74
//#define Frs_Debug_Hud         // 0x50F2
//#define Mav_Debug_Attitude    // #30
//#define Frs_Debug_AttiRange   // 0x5006
//#define Mav_Debug_StatusText  // #253
//#define Frs_Debug_StatusText  //0x5000  
//#define Mav_Debug_Mission
//#define Frs_Debug_Mission   
//#define Debug_SD    
//#define Mav_Debug_System_Time   
//#define Debug_Baud 
//#define Debug_Radio_Status  
//#define Debug_GCS_Unknown
//#define Mav_Show_Unknown_Msgs
//#define Frs_Print_All_DataIDs
//#define Debug_Eeprom
//#define Debug_Web_Settings
//#define Debug_SPort_Switching


//=================================================================================================   
//                                   C H A N G E   L O G
//=================================================================================================
/*

Change log:
                                    
                                      
*/

//================================================================================================= 
//================================================================================================= 
//
//                                    F  R  S  K  Y  S  P  O  R  T
//
//================================================================================================= 
//================================================================================================= 

/* FrSky Passthrough Frame Structure - 10 bytes
 * 
 * 0    Start/stop(0x7E)
 * 1    SensorID e.g. (0x1B)
 * 2    Frame header(0x10) 
 * 3&4  DataID e.g. 0x00 0x80 (0x0800)
 * 5-8  Payload
 * 9    CRC
 */
 
// Local FrSky variables

byte     nb, lb;                      // NextByt, LastByt of ByteStuff pair     
//    crc;                         // crc of frsky packet
//uint8_t  time_slot_max = 16;              
//uint32_t time_slot = 1;
float a, az, c, dis, dLat, dLon;
//uint8_t sv_count = 0;

  volatile uint8_t *uartC3;
  enum SPortMode { rx , tx };
  SPortMode mode, modeNow;
  
//=================================================================================================  
void SPort_Init(void)  {

#if (defined ESP32) || (defined ESP8266) // ESP only
  int8_t frRx;
  int8_t frTx;
  bool   frInvert;

  frRx = Fr_rxPin;
  frTx = Fr_txPin;

  #if defined ESP_Onewire 
    bool oneWire = true;
  #else
     bool oneWire = false;
  #endif
       
  #if defined (ESP_SPort_Invert)  
    frInvert = true;
    Debug.print("S.Port on ESP is inverted and is "); 
  #else
    frInvert = false;
    Debug.print("S.PORT NOT INVERTED! Hw inverter to 1-wire required. S.Port on ESP is "); 
  #endif

  #if ((defined ESP8266) || (defined ESP32) && (defined ESP32_SoftwareSerial))
  
      if (oneWire) {
        frRx = frTx;     //  Share tx pin. Enable oneWire (half duplex)
        Debug.printf("1-wire half-duplex on pin %d \n", frTx); 
      } else {
        Debug.printf("2-wire on pins rx = %d and tx = %d\n", frRx, frTx);
      }
      
      frSerial.begin(frBaud, SWSERIAL_8N1, frRx, frTx, frInvert);     // SoftwareSerial
      Debug.println("Using SoftwareSerial for S.Port");
      if (oneWire) {
        frSerial.enableIntTx(true);
      }  
  #else  // HardwareSerial
    
      frSerial.begin(frBaud, SERIAL_8N1, frRx, frTx, frInvert); 
      Debug.printf("on rx pin = %d\n", frRx);
           
  #endif
#endif

#if (defined TEENSY3X) 
   frSerial.begin(frBaud); // Teensy 3.x    tx pin hard wired
   #if (SPort_Serial == 1)
     // Manipulate UART registers for S.Port working
     uartC3   = &UART0_C3;  // UART0 is Serial1
     UART0_C3 = 0x10;       // Invert Serial1 Tx levels
     UART0_C1 = 0xA0;       // Switch Serial1 into single wire mode
     UART0_S2 = 0x10;       // Invert Serial1 Rx levels;
   
   //   UART0_C3 |= 0x20;    // Switch S.Port into send mode
   //   UART0_C3 ^= 0x20;    // Switch S.Port into receive mode
   #else
     uartC3   = &UART2_C3;  // UART2 is Serial3
     UART2_C3 = 0x10;       // Invert Serial1 Tx levels
     UART2_C1 = 0xA0;       // Switch Serial1 into single wire mode
     UART2_S2 = 0x10;       // Invert Serial1 Rx levels;
   #endif
 
   Debug.printf("S.Port on Teensy3.x inverted 1-wire half-duplex on pin %d \n", Fr_txPin); 
 
#endif   
}
//=================================================================================================   

  void setSPortMode(SPortMode mode); 
  void setSPortMode(SPortMode mode) {   
    
  #if (defined TEENSY3X) 
    if(mode == tx && modeNow !=tx) {
      *uartC3 |= 0x20;                 // Switch S.Port into send mode
      modeNow=mode;
      #if defined Debug_Sport_Switching
        Debug.println("tx <======");
      #endif
    }
    else if(mode == rx && modeNow != rx) {   
      *uartC3 ^= 0x20;                 // Switch S.Port into receive mode
      modeNow=mode;
      #if defined Debug_Sport_Switching
        Debug.println("rx <======");
      #endif
    }
  #endif

  #if (defined ESP8266) || (defined ESP32) 
      if(mode == tx && modeNow !=tx) { 
        modeNow=mode;
        pb_rx = false;
        #if (defined ESP_Onewire) && (defined ESP32_SoftwareSerial)        
          frSerial.enableTx(true);  // Switch S.Port into send mode
        #endif
        #if defined Debug_Sport_Switching
          Debug.println("tx <======");
        #endif
      }   else 
      if(mode == rx && modeNow != rx) {   
        modeNow=mode; 
        pb_rx = true; 
        #if (defined ESP_Onewire) && (defined ESP32_SoftwareSerial)                  
          frSerial.enableTx(false);  // disable interrupts on tx pin     
        #endif
        #if defined Debug_Sport_Switching
          Debug.println("rx <======");
        #endif
      } 
  #endif
  }
//=================================================================================================  
  void frSerialSafeWrite(byte b) {
    setSPortMode(tx);
    frSerial.write(b);   
    #if defined Debug_Sport_Switching  
      PrintByte(b);
    #endif 
    delay(0); // yield to rtos for wifi & bt to get a sniff
  }
//=================================================================================================
 byte frSerialSafeRead() {
    setSPortMode(rx);
    byte b = frSerial.read(); 
    #if defined Debug_Sport_Switching  
      PrintByte(b);
    #endif 
    delay(0); // yield to rtos for wifi & bt to get a sniff 
  return b;
  }
 
//=================================================================================================  
//=================================================================================================
void SPort_Interleave_Packet(void) {
  setSPortMode(rx);
  uint8_t prevByt=0;
  uint8_t Byt = 0;
  while ( frSerial.available())   {  
    Byt =  frSerialSafeRead();

    if ((prevByt == 0x7E) && (Byt == 0x1B)) { 
      sp_read_millis = millis(); 
      spGood = true;
      ReportSportStatusChange();
      #if defined Debug_Sport_Switching
        Debug.println("match"); 
      #endif
  //    SPort_Inject_Packet();  //  <========================
      return;
    }     
  prevByt=Byt;
  }
  // and back to main loop
}  

//=================================================================================================  

void SPort_SendByte(uint8_t byte, bool addCrc) {
  #if (not defined inhibit_SPort) 
    
   if (!addCrc) {
      frSerialSafeWrite(byte);  
     return;       
   }

   CheckByteStuffAndSend(byte);
 
    // update CRC
    crc += byte;       //0-1FF
    crc += crc >> 8;   //0-100
    crc &= 0x00ff;
    crc += crc >> 8;   //0-0FF
    crc &= 0x00ff;
    
  #endif    
}

//=================================================================================================  
void CheckByteStuffAndSend(uint8_t byte) {
  #if (not defined inhibit_SPort) 
   if (byte == 0x7E) {
     frSerialSafeWrite(0x7D);
     frSerialSafeWrite(0x5E);
   } else if (byte == 0x7D) {
     frSerialSafeWrite(0x7D);
     frSerialSafeWrite(0x5D);    
   } else {
     frSerialSafeWrite(byte);  
     }
  #endif     
}
//=================================================================================================  
void SPort_SendCrc() {
  uint8_t byte;
  byte = 0xFF-crc;

 CheckByteStuffAndSend(byte);
 
 // PrintByte(byte);
 // Debug.println("");
  crc = 0;          // CRC reset
}
//=================================================================================================  
void SPort_SendDataFrame(uint8_t Instance, uint16_t Id, uint32_t value) {

//  if (set.trmode == ground) {    // Only if ground mode send these bytes, else XSR sends them
//    SPort_SendByte(0x7E, false);       //  START/STOP don't add into crc
//    SPort_SendByte(Instance, false);   //  don't add into crc  
//  }
  
  SPort_SendByte(0x10, true );   //  Data framing byte
 
  uint8_t *bytes = (uint8_t*)&Id;
  #if defined Frs_Debug_Payload
    Debug.print("DataFrame. ID "); 
    PrintByte(bytes[0]);
    Debug.print(" "); 
    PrintByte(bytes[1]);
  #endif
  SPort_SendByte(bytes[0], true);
  SPort_SendByte(bytes[1], true);
  bytes = (uint8_t*)&value;
  SPort_SendByte(bytes[0], true);
  SPort_SendByte(bytes[1], true);
  SPort_SendByte(bytes[2], true);
  SPort_SendByte(bytes[3], true);
  
  #if defined Frs_Debug_Payload
    Debug.print("Payload (send order) "); 
    PrintByte(bytes[0]);
    Debug.print(" "); 
    PrintByte(bytes[1]);
    Debug.print(" "); 
    PrintByte(bytes[2]);
    Debug.print(" "); 
    PrintByte(bytes[3]);  
    Debug.print("Crc= "); 
    PrintByte(0xFF-crc);
    Debug.println("/");  
  #endif
  
  SPort_SendCrc();
}
//=================================================================================================  
uint32_t TenToPwr(uint8_t pwr) {
  uint32_t ttp = 1;
  for (int i = 1 ; i<=pwr ; i++) {
    ttp*=10;
  }
  return ttp;
}
//=================================================================================================  
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
  return r;
}
//=================================================================================================  
// Mask then AND the shifted bits, then OR them to the payload
  void bit32Pack(uint32_t dword ,uint8_t displ, uint8_t lth) {   
  uint32_t dw_and_mask =  (dword<<displ) & (createMask(displ, displ+lth-1)); 
  fr_payload |= dw_and_mask; 
}
//=================================================================================================  
  uint32_t bit32Unpack(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
  return r;
}
//=================================================================================================  
uint32_t createMask(uint8_t lo, uint8_t hi) {
  uint32_t r = 0;
  for (unsigned i=lo; i<=hi; i++)
       r |= 1 << i;  
  return r;
}

//=================================================================================================  
int8_t PWM_To_63(uint16_t PWM) {       // PWM 1000 to 2000   ->    nominal -63 to 63
int8_t myint;
  myint = round((PWM - 1500) * 0.126); 
  myint = myint < -63 ? -63 : myint;            
  myint = myint > 63 ? 63 : myint;  
  return myint; 
}

//=================================================================================================  
uint32_t Abs(int32_t num) {
  if (num<0) 
    return (num ^ 0xffffffff) + 1;
  else
    return num;  
}
//=================================================================================================  
float Distance(Loc2D loc1, Loc2D loc2) {
float a, c, d, dLat, dLon;  

  loc1.lat=loc1.lat/180*PI;  // degrees to radians
  loc1.lon=loc1.lon/180*PI;
  loc2.lat=loc2.lat/180*PI;
  loc2.lon=loc2.lon/180*PI;
    
  dLat = (loc1.lat-loc2.lat);
  dLon = (loc1.lon-loc2.lon);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(loc2.lat) * cos(loc1.lat); 
  c = 2* asin(sqrt(a));  
  d = 6371000 * c;    
  return d;
}
//=================================================================================================  
float Azimuth(Loc2D loc1, Loc2D loc2) {
// Calculate azimuth bearing from loc1 to loc2
float a, az; 

  loc1.lat=loc1.lat/180*PI;  // degrees to radians
  loc1.lon=loc1.lon/180*PI;
  loc2.lat=loc2.lat/180*PI;
  loc2.lon=loc2.lon/180*PI;

  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(loc2.lat) * cos(loc1.lat); 
  
  az=a*180/PI;  // radians to degrees
  if (az<0) az=360+az;
  return az;
}
//=================================================================================================  
//Add two bearing in degrees and correct for 360 boundary
int16_t Add360(int16_t arg1, int16_t arg2) {  
  int16_t ret = arg1 + arg2;
  if (ret < 0) ret += 360;
  if (ret > 359) ret -= 360;
  return ret; 
}
//=================================================================================================  
// Correct for 360 boundary - yaapu
float wrap_360(int16_t angle)
{
    const float ang_360 = 360.f;
    float res = fmodf(static_cast<float>(angle), ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}
//=================================================================================================  
// From Arducopter 3.5.5 code
uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

   if ((digits == 1) && (power == 1)) { // number encoded on 5 bits: 4 bits for digits + 1 for 10^power
        if (abs_number < 10) {
            res = abs_number<<1;
        } else if (abs_number < 150) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x0F x 10^1 = 150)
            res = 0x1F;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<5;
        }
    } else if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = abs_number<<1;
        } else if (abs_number < 1270) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0xFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<8;
        }
    } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = abs_number<<2;
         //   Debug.print("abs_number<100  ="); Debug.print(abs_number); Debug.print(" res="); Debug.print(res);
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
         //   Debug.print("abs_number<1000  ="); Debug.print(abs_number); Debug.print(" res="); Debug.print(res);
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
          //  Debug.print("abs_number<10000  ="); Debug.print(abs_number); Debug.print(" res="); Debug.print(res);
        } else if (abs_number < 127000) {
            res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x1FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<9;
        }
    } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<1;
        } else if (abs_number < 10240) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x3FF x 10^1 = 10240)
            res = 0x7FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<11;
        }
    } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<2;
        } else if (abs_number < 10000) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 100000) {
            res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 1024000) {
            res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x3FF x 10^3 = 127000)
            res = 0xFFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<12;
        }
    }
    return res;
}  
//=================================================================================================  

// NOTE: to support GNSS you must edit libraries/TinyGPSPlus/TinyGPS++.cpp and change these definitions
// #define _GPRMCterm   "GPRMC"
// #define _GPGGAterm   "GPGGA"
// You should also add there support for GNRMC to be parsed as GPRMC because Mediatek outputs GPRMC until the fix
//
// HW used:
// - Arduino Pro Mini (3.3V)
// - USB Li-Ion charger
// - Li-Ion battery connected to components; all components must be operable at Li-Ion voltage 3.6V-3.7V (mostly using voltage stabilizer)
// - GPS NMEA module with serial interface (connected to TX / RX via jumpers to be able to reprogram Arduino)
// - optionally 3V GPS backup battery if not embedded in GPS module already
// - SPI SD card module
// - LED with resistor on pin 4
// - speaker on pin 2
// - power switch connecting output of USB charger to VCC

#include <SPI.h>
#include <SdFat.h>
#include <TinyGPS++.h>

#undef DEBUG

#ifdef DEBUG
#define LOG(str) (str) // Serial.println(str)
#define LOGW(str) (str) // Serial.write(str)
#else
#define LOG(str)
#define LOGW(str)
#endif

TinyGPSPlus gps;
boolean cardOk = false;
File dataFile;

uint8_t month, day, hour, minute, second;
uint16_t year = 0;

unsigned long time = 0;
int cmd = 0;

unsigned short sats = 0;
float hdop = 10000;

SdFat SD;

void getNewFilename(char* buf, int size) {
  int i = 0;
  do {
    String num = "00000" + String(i++);
    num = num.substring(num.length() - 6);
    String fname = "DL" + num + ".NMA";
    fname.toCharArray(buf, size);
  } while (SD.exists(buf));
}

char *configs[] = {
  // only RMC and GGA are supported by TinyGPS++

// NOTE: This is for Ublox M8N
//  "$PUBX,41,1,0007,0003,38400,0*20",
//  "$PUBX,40,GSV,0,0,0,0,0,0*59", // disable GSV
//  "$PUBX,40,GLL,0,0,0,0,0,0*5C" // disable GLL
//  "$PUBX,40,VTG,0,0,0,0,0,0*5E" // disable VTG
//  "$PUBX,40,GSA,0,0,0,0,0,0*4E" // disable GSA
  

// NOTE: This is for Mediatek (MT333x)
  "$PMTK301,2*2E", // WAAS
  "$PMTK313,1*2E", // enable SBAS
////  "$PMTK220,250*29", // 4Hz
  "$PMTK220,1000*1F", // 1Hz
  "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29", // RMC, GSA, GGA
  "$PMTK397,0*23", // no speed threshold
};

  // $PMTK397,0*23 no speed threshold
  // $PMTK314,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*19
  // $PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29 // GPRMC
  // $PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29 // GPGGA
  // $PMTK251,4800*14 // $PMTK251,9600*17 // $PMTK251,19200*22 // $PMTK251,38400*27
  // $PMTK220,1000*1F // 1Hz
  // $PMTK220,200*2C // 5Hz
  // $PMTK220,100*2F // 10Hz
  // $PMTK104*37 // cold restart


void setup() {
  // pinMode(6, INPUT); // button for changing baud rates
  pinMode(4, OUTPUT); // LED
//  digitalWrite(4, HIGH);

  pinMode(2, OUTPUT); // SPKR
  tone(2, 1055, 200);

  Serial.begin(9600);
    
  LOG("INIT");

  cardOk = SD.begin(10); // , SPI_HALF_SPEED
  if (!cardOk) {
    LOG("Card failed, or not present");
    
    for (;;) {
      tone(2, 2109, 100);
      delay(900);
    }
    
    return;
  }
  
  tone(2, 1580, 200);

  LOG("card initialized.");
}

void openDataFile() {
  char fname[13];
  getNewFilename(fname, 13);
  LOGW("File: "); LOG(fname);
  SdFile::dateTimeCallback(dateTime);
  dataFile = SD.open(fname, FILE_WRITE);
  if (dataFile) {
    LOG("file opened");
  } else {
    cardOk = false;
  }
}

void loop() {
  if (cardOk && Serial.available()) {
    char c = Serial.read();
    LOGW(c);
//    bt.write(c);
    
    unsigned long m = millis();
    
    if (dataFile) {
      dataFile.write(c);
    
      //we are ok to lost some data, but save write power
      if (c == '\n' && m - time > 30000) {
        time = m;

// some debug stuff
//        char buf[20];
//        dtostrf((double) hdop, 10, 3, buf);
//        dataFile.write(buf);
//        dataFile.write('\n');
//        dataFile.println(readVcc(), DEC);
//        dataFile.flush();
        
        if (!dataFile.sync() || dataFile.getWriteError()) {
          tone(2, 2109, 200);
        }
      }
    }

    if (cmd < sizeof(configs)) {
      if (m > 2000 + cmd * 500) {
        Serial.println(configs[cmd++]);
        if (cmd == 1) {
          //Serial.begin(38400);
        }
      }
    } else { // if all configured
      gps.encode(c);
      
      if (!year && gps.date.isUpdated() && gps.time.isUpdated()) {
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
        day = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();
        if (year > 2030 || year < 2010) {
          year = 0;
        } else {
          openDataFile();
          tone(2, 1580, 500);
        }
       
      }
  
      if (gps.satellites.isUpdated()) {
        sats = gps.satellites.value();
      }
  
      if (gps.hdop.isUpdated()) {
        hdop = gps.hdop.value();
        if (hdop == 0) { // happens for SKM53 (it is actually empty)
          hdop = 10000;
        }
      }
    }
  }

  // num of sats
  // digitalWrite(4, millis() % 100 > 50 && (millis() / 100) % 12 <= sats);

  // hdop
  digitalWrite(4, !cardOk || (millis() % 100 > 50 && (millis() / 100) % 50 < (hdop / 100.0 - 0.95) * 10));
 
}

void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year, month, day);

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour, minute, second);
}

//byte getUbxAck(byte *msgID) {
//  byte ckA = 0, ckB = 0;
//  byte incoming_char;
//  boolean headerReceived = false;
//  unsigned long ackWait = millis();
//  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//  int i = 0;
//
//  for (;;) {
//    if (Serial.available()) {
//      incoming_char = Serial.read();
//      if (incoming_char == ackPacket[i]) {
//        i++;
//      } else if (i > 2) {
//        ackPacket[i] = incoming_char;
//        i++;
//      }
//    }
//    if (i > 9) {
//      break;
//    }
//    if (millis() - ackWait > 1500) {
//      LOG("ACK Timeout");
//      return 5;
//    }
//    if (i == 4 && ackPacket[3] == 0x00) {
//      LOG("NAK Received");
//      return 1;
//    }
//  }
//
//  for (i = 2; i < 8 ;i++) {
//    ckA = ckA + ackPacket[i];
//    ckB = ckB + ckA;
//  }
//
//  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && ckA == ackPacket[8] && ckB == ackPacket[9]) {
//    LOG("Success");
//    return 10;
//  }
//
//  LOG("ACK Checksum Failure");
//  delay(1000);
//  return 1;
//}


//long readVcc() {
//  long result;
//  // Read 1.1V reference against AVcc
//  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//  delay(2); // Wait for Vref to settle
//  ADCSRA |= _BV(ADSC); // Convert
//  while (bit_is_set(ADCSRA,ADSC));
//  result = ADCL;
//  result |= ADCH<<8;
//  result = 1126400L / result; // Back-calculate AVcc in mV
//  return result;
//}

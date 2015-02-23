//#include <AltSoftSerial.h>
#include <SD.h>
#include <TinyGPS++.h>
//#include <SoftwareSerial.h>

#undef DEBUG

#ifdef DEBUG
#define LOG(str) Serial.println(str)
#define LOGW(str) Serial.write(str)
#else
#define LOG(str)
#define LOGW(str)
#endif

//AltSoftSerial mySerial;
//SoftwareSerial bt(4,5);
TinyGPSPlus gps;
boolean cardOk = false;
boolean btn = false;
boolean off = false;
File dataFile;

uint8_t month, day, hour, minute, second;
uint16_t year = 0;

unsigned long time = 0;
int cmd = 0;

unsigned short sats = 0;
float hdop = 10000;

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
  "$PUBX,40,GSV,0,0,0,0*59",
  "$PUBX,40,GLL,0,0,0,0*5C"
  "$PMTK301,2*2E", // WAAS
  "$PMTK313,1*2E", // enable SBAS
//  "$PMTK220,250*29", // 4Hz
  "$PMTK220,1000*1F", // 1Hz
  "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29", // RMC, GSA, GGA
  "$PMTK397,0*23", // no speed threshold
};

void setup() {
  // pinMode(6, INPUT); // button for changing baud rates
  pinMode(4, OUTPUT); // LED
//  digitalWrite(4, HIGH);

  Serial.begin(9600);
    
//  bt.begin(9600);

  LOG("INIT");

  // set the data rate for the SoftwareSerial port
  
  // $PMTK397,0*23 no speed threshold
  // $PMTK314,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*19
  // $PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29 // GPRMC
  // $PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29 // GPGGA
  // $PMTK251,4800*14 // $PMTK251,9600*17 // $PMTK251,19200*22 // $PMTK251,38400*27
  // $PMTK220,1000*1F // 1Hz
  // $PMTK220,200*2C // 5Hz
  // $PMTK220,100*2F // 10Hz
  
  // Serial.print("$PMTK104*37\r\n"); // reset


  cardOk = SD.begin(10);
  if (!cardOk) {
    LOG("Card failed, or not present");
    return;
  }
  
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
//  if (digitalRead(6) && !btn) {
//    btn = true;
//    LOG("BUTTON");
//    off = true;
//    if (dataFile) {
//      dataFile.flush();
//    }
//  }
//  
//  if (!digitalRead(6) && btn) {
//    btn = false;
//  }

  if (cardOk && Serial.available()) {
    char c = Serial.read();
    LOGW(c);
//    bt.write(c);
    
    unsigned long m = millis();
    
    if (dataFile && !off) {
      dataFile.write(c);
      //we are ok to lost some data, but save write power
      if (c == '\n' && m - time > 5000) {
        time = m;

//        char buf[20];
//        dtostrf((double) hdop, 10, 3, buf);
//        dataFile.write(buf);
//        dataFile.write('\n');

        dataFile.flush();
      }
    }
    
    if (cmd < sizeof(configs)) {
      if (m > 2000 + cmd * 500) {
        Serial.println(configs[cmd++]);
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
        if (year == 2081) { // happens for SKM53
          year = 0;
        } else {
          openDataFile();
        }
        
//        if (dataFile) {
//            dataFile.print("XXX ");
//            dataFile.print(day);
//            dataFile.print(" YYY ");
//            dataFile.print(year);
//            dataFile.println(" ZZZ");
//        }

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

//  if (Serial.available()) {
//    char c = Serial.read();
//    Serial.write(c);
//    mySerial.write(c);
//  }
  
}

byte getUbxAck(byte *msgID) {
  byte ckA = 0, ckB = 0;
  byte incoming_char;
  boolean headerReceived = false;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;

  for (;;) {
    if (Serial.available()) {
      incoming_char = Serial.read();
      if (incoming_char == ackPacket[i]) {
        i++;
      } else if (i > 2) {
        ackPacket[i] = incoming_char;
        i++;
      }
    }
    if (i > 9) {
      break;
    }
    if (millis() - ackWait > 1500) {
      LOG("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      LOG("NAK Received");
      return 1;
    }
  }

  for (i = 2; i < 8 ;i++) {
    ckA = ckA + ackPacket[i];
    ckB = ckB + ckA;
  }

  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && ckA == ackPacket[8] && ckB == ackPacket[9]) {
    LOG("Success");
    return 10;
  }

  LOG("ACK Checksum Failure");
  delay(1000);
  return 1;
}

void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year, month, day);

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour, minute, second);
}

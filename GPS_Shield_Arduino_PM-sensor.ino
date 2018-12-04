#include<SdFat.h>
#include<SPI.h> //Load SPI Library
#include <SDS011.h>
#include <Adafruit_GPS.h>    //Install the adafruit GPS library
#include <SoftwareSerial.h> //Load the Software Serial library
SoftwareSerial mySerial(8,7); //Initialize the Software Serial port
Adafruit_GPS GPS(&mySerial); //Create the GPS Object

String NMEA1; //Variable for first NMEA sentence
String NMEA2; //Variable for second NMEA sentence
char c; //to read characters coming from the GPS

const uint8_t chipSelect = 4;
SdFat sd;
SdFile mySensorData;
int error;
float p10, p25;
float ppm = 0.0;
SDS011 sds;
int TX_PM = 1;
int RX_PM = 0;
int count = 0;
boolean asked = false;
boolean onGPS = false;
void setup() {
  
  Serial.begin(115200); //Turn on serial monitor
  GPS.begin(9600); //Turn on GPS at 9600 baud
  GPS.sendCommand("$PGCMD,33,0*6D");  //Turn off antenna update nuisance data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Request RMC and GGA Sentences only
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Set update rate to 1 hz
  delay(1000); 
  
  pinMode(10, OUTPUT); //Must declare 10 an output and reserve it to keep SD card happy
  Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
      Serial.println("Card failed, or not present");
      sd.initErrorHalt();
      // don't do anything more:
    }
  Serial.println("card initialized.");
}

void loop() {
  
while (GPS.fix == 0){
  turnonGPS();
  readGPS();
  asked = false;
}

if(GPS.fix==1) {
  //Only save data if we have a fix
  onGPS = false;
  oneTime();
  error = sds.read(&p25, &p10);
  if (!error) {
    Serial.println("P2.5: " + String(p25));
    Serial.println("P10:  " + String(p10));
  }
  else {
    Serial.println("Error: could not get a reading from PM sensor!");
    p25 = 0.0;
    p10 = 0.0;
  }  
  mySensorData.open("GPS2.csv", O_RDWR | O_CREAT | O_AT_END); //Open file on SD card for writing
  mySensorData.print(GPS.latitude,4); //Write measured latitude to file
  mySensorData.print(GPS.lat); //Which hemisphere N or S
  mySensorData.print(",");
  mySensorData.print(GPS.longitude,4); //Write measured longitude to file
  mySensorData.print(GPS.lon); //Which Hemisphere E or W
  mySensorData.print(",");
  mySensorData.print(GPS.altitude);
  mySensorData.print(",");
  mySensorData.print(GPS.hour, DEC);
  mySensorData.print(",");
  mySensorData.print(GPS.minute, DEC);
  mySensorData.print(",");
  mySensorData.print(GPS.seconds, DEC);
  mySensorData.print(",");
  mySensorData.print(GPS.day, DEC);
  mySensorData.print(",");
  mySensorData.print(GPS.month, DEC);
  mySensorData.print(",");
  mySensorData.print("20");
  mySensorData.print(GPS.year, DEC);
  mySensorData.print(",");
  mySensorData.print(String(ppm));
  mySensorData.print(",");
  mySensorData.print(String(p25));
  mySensorData.print(",");
  mySensorData.print(String(p10));
  mySensorData.println();  
  mySensorData.close();
  }
  delay(30000);
  count = count + 1;
  if (count  >= 4){
    turnonGPS();
    readGPS();
    asked = false;
    count = 0;
  }
}

void turnonGPS(){
  if (onGPS == false){
    GPS.begin(9600);
  }
  onGPS = true;
}


void oneTime(){
  if (asked == false){
    sds.begin(RX_PM, TX_PM);
  }
  asked = true;
}

void readGPS() {
  
  clearGPS();
  while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  NMEA1=GPS.lastNMEA();
  
   while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  NMEA2=GPS.lastNMEA();
  
  Serial.println(NMEA1);
  Serial.println(NMEA2);
  Serial.println("");
  
}

void clearGPS() {  //Clear old and corrupt data from serial port 
  while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  
  while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
   while(!GPS.newNMEAreceived()) { //Loop until you have a good NMEA sentence
    c=GPS.read();
  }
  GPS.parse(GPS.lastNMEA()); //Parse that last good NMEA sentence
  
}

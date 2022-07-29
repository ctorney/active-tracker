
String behaviours[] = { 
  "GRAZING",
  "LYING", 
  "STANDING",
  "WALKING"
};
#include "Adafruit_Keypad.h"

// define your specific keypad here via PID
#define KEYPAD_PID1332
// define your pins here
// can ignore ones that don't apply
#define R1    0
#define C1    2
#define C2    1
#define C3    4
#define C4    3
// leave this import after the above configuration
#include "keypad_config.h"

//initialize an instance of class NewKeypad
Adafruit_Keypad customKeypad = Adafruit_Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



#define GPS_I2C_ADDRESS 0x10

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);

const int chipSelect = SDCARD_SS_PIN;




unsigned long updateTime = 1000;
unsigned long lastUpdate = 0;
int behaviour = -1;

void setup() {
  Serial.begin(9600);
  customKeypad.begin();
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  Serial.print("Initializing SD card...");
//*
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

if (!GPS.begin(GPS_I2C_ADDRESS)) 
  {
    Serial.println("ERROR: GPS");
        while (true);
  }

  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  

}

void loop() {

  char c = GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
  
  
  customKeypad.tick();

  unsigned long currentMillis = millis();  
  if (currentMillis - lastUpdate >= updateTime)
  {
    lastUpdate = currentMillis;
    update_display();
    }
  
  while(customKeypad.available()){
    keypadEvent e = customKeypad.read();
    
    
    if(e.bit.EVENT == KEY_JUST_PRESSED){
      
      behaviour = (int)e.bit.COL;
      update_display();
      save_behaviour();
    }
    
    
  }

  delay(10);
}

void save_behaviour()
{


  String gps_year = String("20" + String(GPS.year, DEC));
  String gps_month;
  String gps_day;
  if (GPS.month<10)
    gps_month =  String("0" +String(GPS.month, DEC));
  else
    gps_month =  String(GPS.month, DEC);

  if (GPS.day<10)
    gps_day =  String("0" +String(GPS.day, DEC));
  else
    gps_day =  String(GPS.day, DEC);

  String filename = String("AL-" + gps_day + "-" + gps_month + ".csv");  

  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) 
  {

    Serial.print("Writing to ");
    Serial.println(filename);

    if (GPS.hour < 10) { dataFile.print('0'); }
    dataFile.print(GPS.hour, DEC); dataFile.print(':');
    if (GPS.minute < 10) { dataFile.print('0'); }
    dataFile.print(GPS.minute, DEC); dataFile.print(':');
    if (GPS.seconds < 10) { dataFile.print('0'); }
    dataFile.print(GPS.seconds, DEC); dataFile.print('.');
    if (GPS.milliseconds < 10) {
      dataFile.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      dataFile.print("0");
    }
    dataFile.print(GPS.milliseconds);
    dataFile.print(",");
    dataFile.println(behaviour);
    dataFile.close();
    
    Serial.println("done.");

  } else {

    // if the file didn't open, print an error:

    Serial.println("error opening file");

  }

  
}

void update_display() 
{
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,16);             // Start at top-left corner


  if (behaviour<0)
  {
    if (GPS.fix){
      display.println("READY");
    }
    else{
      display.println("WAITING");
      display.println("FOR GPS");
    }
  }
  else
  {
    display.print(behaviour+1);
    display.print(":");
    display.println(behaviours[behaviour]);
  }

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,50);             // Start at top-left corner


  if (GPS.hour < 10) { display.print('0'); }
  display.print(GPS.hour, DEC); display.print(':');
  if (GPS.minute < 10) { display.print('0'); }
  display.print(GPS.minute, DEC); display.print(':');
  if (GPS.seconds < 10) { display.print('0'); }
  display.print(GPS.seconds, DEC); 
  
  
  display.display();
  
}

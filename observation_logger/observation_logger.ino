
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


}

void loop() {
  
  customKeypad.tick();

  unsigned long currentMillis = millis();  
  if (currentMillis - lastUpdate >= updateTime)
  {
    lastUpdate = currentMillis;
    update_display();
    }
  
  while(customKeypad.available()){
    keypadEvent e = customKeypad.read();
    
    Serial.print((uint8_t)e.bit.COL);
    if(e.bit.EVENT == KEY_JUST_PRESSED){
      Serial.println(" pressed");
      behaviour = (int)e.bit.COL;
      update_display();
    }
    else if(e.bit.EVENT == KEY_JUST_RELEASED) Serial.println(" released");
    
  }

  delay(10);
}

void testdrawstyles(int behaviour) {
  display.clearDisplay();

  
  
  display.display();
  
}

void update_display() {
  display.clearDisplay();
  unsigned long currentMillis = millis();  

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,16);             // Start at top-left corner

  //display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  if (behaviour<0)
  {
  
  display.println("READY");
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

  display.println(currentMillis);

  
  display.display();
  
}

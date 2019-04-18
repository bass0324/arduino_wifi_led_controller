#define hasBme280 true

#include "Timer.h"
#include "WiFi.h"
#include "Wire.h"
#include "WiFiClient.h"
#include "WebServer.h"
#include "ESPmDNS.h"
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#if hasBme280
#include <Adafruit_BME280.h>
#else
#include <Adafruit_BMP280.h>
#endif
//#include <Fonts/FreeMono9pt7b.h>
//#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/Picopixel.h>
//#include <Fonts/Org_01.h>

#include "WiFiCredential.h"

#define Ws2812DataPin 16
#define RedPin 14
#define GreenPin 27
#define BluePin 12
#define SdaPin 25
#define SclPin 26

#define i2cFrequency 400000

#define oledAddr 0x3C
#define tempSenseAddr 0x76  // addresses could be 0x76 and 0x77 
#define lightSenseAddr 0x39 // addresses could be 0x29, 0x39, or 0x49

#define SEALEVELPRESSURE_HPA (1013.25)

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

int freq = 5000;
int redChannel = 0;
int greenChannel = 1;
int blueChannel = 2;
int resolution = 8;
uint8_t cycleLedId = 0;
uint8_t fadeLedId = 0;
uint8_t fadeControl = 0;

struct RgbLed {
  uint8_t redValue = 0;
  uint8_t greenValue = 0;
  uint8_t blueValue = 0;
};

struct RgbStrip {
  struct RgbLed rgbLed;
  uint8_t brightness = 255;
  uint16_t timeControl = 500;
  uint8_t controlMode = 0;
  uint8_t updateFlag = 0;
};

struct SensorsInfo {
  int light = 0;
  int temp = 0;
  int pressure = 0;
  int humidity = 0;
};

struct RgbStrip rgbStrip;
struct SensorsInfo sensorsInfo;

//WiFiServer server(80);
WebServer server(80);
String header;

Adafruit_SSD1306 display(0x0);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#if hasBme280
Adafruit_BME280 environSensor;
#else
Adafruit_BMP280 environSensor;
#endif
Timer t;

void updateLed(void *context) {
  if (rgbStrip.updateFlag) {
    //uint16_t redValue = (uint16_t)rgbStrip.rgbLed.redValue;
    ledcWrite(redChannel, (rgbStrip.rgbLed.redValue * rgbStrip.brightness)/255);
    ledcWrite(greenChannel, (rgbStrip.rgbLed.greenValue * rgbStrip.brightness)/255);
    ledcWrite(blueChannel, (rgbStrip.rgbLed.blueValue * rgbStrip.brightness)/255);
    rgbStrip.updateFlag = 0;
    //Serial.print("Current LED Mode is: ");
    //Serial.print(rgbStrip.controlMode);
  }
}

void handleLedMode(void *context) {
  if (rgbStrip.controlMode == 0) {         //single led value set remotely
    if (fadeLedId > 0) {
      t.stop(fadeLedId);
      fadeLedId = 0;
    } else if (cycleLedId > 0) {
      t.stop(cycleLedId);
      cycleLedId = 0;
    }
  } else if (rgbStrip.controlMode == 1) {  //cycle through individual rgb led
    //rgbStrip.timeControl = 1000;
    if (fadeLedId > 0) {
      t.stop(fadeLedId);
      fadeLedId = 0;
    }
    if (cycleLedId == 0) {
      cycleLedId = t.every(rgbStrip.timeControl, cycleLeds, (void *)3);
      Serial.print("Cycle LED ID is: ");
      Serial.println(cycleLedId);
    }
  } else if (rgbStrip.controlMode == 2) {  //fade through led color combinations
    //rgbStrip.timeControl = 50;
    if (cycleLedId > 0) {
      t.stop(cycleLedId);
      cycleLedId = 0;
    }
    if (fadeLedId == 0) {
      fadeLedId = t.every(rgbStrip.timeControl,fadeLeds,(void *)4);
      Serial.print("Fade LED ID is: ");
      Serial.println(fadeLedId);
    }
  }
}

void updateDisplay(void *context) {
  sensors_event_t event;
  uint16_t cursory;
  tsl.getEvent(&event);
  if (event.light) {
    sensorsInfo.light = event.light;
  }
  sensorsInfo.temp = ((9*environSensor.readTemperature())/5)+32;
  sensorsInfo.pressure = environSensor.readPressure();
  #if hasBme280
    sensorsInfo.humidity = environSensor.readHumidity();
  #endif
  display.clearDisplay();
  display.setCursor(0,12);
  display.print("IP ADD: ");
  display.println(WiFi.localIP());
  display.print("L: ");
  display.print(sensorsInfo.light);
  cursory = display.getCursorY();
  display.setCursor(84,cursory);
  display.print("T: ");
  display.println(sensorsInfo.temp);
  display.print("P: ");
  display.print(sensorsInfo.pressure);
  #if hasBme280
    //humidity = environSensor.readHumidity();
    cursory = display.getCursorY();
    display.setCursor(84,cursory);
    display.print("H: ");
    display.println(sensorsInfo.humidity);
  #else
    display.println();
  #endif
  cursory = display.getCursorY();
  display.print("R:");
  display.print(rgbStrip.rgbLed.redValue);
  display.setCursor(42,cursory);
  display.print("G:");
  display.print(rgbStrip.rgbLed.greenValue);
  display.setCursor(84,cursory);
  display.print("B:");
  display.print(rgbStrip.rgbLed.blueValue);
  display.println();
  display.display();
}

void cycleLeds(void* context) {
  if (rgbStrip.rgbLed.blueValue > 0) {
    rgbStrip.rgbLed.redValue = 255;
    rgbStrip.rgbLed.greenValue = 0;
    rgbStrip.rgbLed.blueValue = 0;
  } else if (rgbStrip.rgbLed.redValue > 0) {
    rgbStrip.rgbLed.redValue = 0;
    rgbStrip.rgbLed.greenValue = 255;
    rgbStrip.rgbLed.blueValue = 0;
  } else if (rgbStrip.rgbLed.greenValue > 0) {
    rgbStrip.rgbLed.redValue = 0;
    rgbStrip.rgbLed.greenValue = 0;
    rgbStrip.rgbLed.blueValue = 255;
  } else {
    rgbStrip.rgbLed.redValue = 255;
  }
  rgbStrip.updateFlag = 1;
}

void fadeLeds(void* context) {
  if ((rgbStrip.rgbLed.redValue < 255) && (rgbStrip.rgbLed.blueValue == 255)) {
    rgbStrip.rgbLed.redValue++;
  } else if ((rgbStrip.rgbLed.redValue == 255) && (rgbStrip.rgbLed.blueValue > 0)) {
    rgbStrip.rgbLed.blueValue--;
  } else if ((rgbStrip.rgbLed.greenValue < 255) && (rgbStrip.rgbLed.redValue == 255)) {
    rgbStrip.rgbLed.greenValue++;
  } else if ((rgbStrip.rgbLed.redValue > 0) && (rgbStrip.rgbLed.greenValue == 255)) {
    rgbStrip.rgbLed.redValue--;
  } else if ((rgbStrip.rgbLed.blueValue < 255) && (rgbStrip.rgbLed.greenValue == 255)) {
    rgbStrip.rgbLed.blueValue++;
  } else if ((rgbStrip.rgbLed.greenValue > 0)  && (rgbStrip.rgbLed.blueValue == 255)) {
    rgbStrip.rgbLed.greenValue--;
  } else {
    rgbStrip.rgbLed.redValue++;
  }
  rgbStrip.updateFlag = 1;
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

String handleServer(void) {
  String message = "";
  String modeValue = "";
  String redValue = "";
  String greenValue = "";
  String blueValue = "";
  String timeValue = "";
  String brightValue = "";

  modeValue = server.arg("mode");
  redValue = server.arg("red");
  greenValue = server.arg("green");
  blueValue = server.arg("blue");
  timeValue = server.arg("time");
  brightValue = server.arg("bright");
  
  if (modeValue == ""){     //Parameter not found
  
    message = "Mode Argument not found";
  
  } else {     //Parameter found
    if (modeValue.toInt() == 0) {
      rgbStrip.controlMode = 0;
      if ( redValue != "" ) {
        if (redValue.toInt() > 255) {
          rgbStrip.rgbLed.redValue = 255;
        } else if (redValue.toInt() < 0) {
          rgbStrip.rgbLed.redValue = 0;
        } else {
          rgbStrip.rgbLed.redValue = redValue.toInt();
        }
      }
      if ( greenValue != "" ) {
        if (greenValue.toInt() > 255) {
          rgbStrip.rgbLed.greenValue = 255;
        } else if (greenValue.toInt() < 0) {
          rgbStrip.rgbLed.greenValue = 0;
        } else {
          rgbStrip.rgbLed.greenValue = greenValue.toInt();
        }
      }
      if ( blueValue != "" ) {
        if (blueValue.toInt() > 255) {
          rgbStrip.rgbLed.blueValue = 255;
        } else if (blueValue.toInt() < 0) {
          rgbStrip.rgbLed.blueValue = 0;
        } else {
          rgbStrip.rgbLed.blueValue = blueValue.toInt();
        }
      }
      rgbStrip.updateFlag = 1;
    } else if (modeValue.toInt() == 1) {
      rgbStrip.controlMode = 1;
      if (timeValue != "") {
        if (timeValue.toInt() != NULL) {
          rgbStrip.timeControl = timeValue.toInt();
        }
      }
    } else if (modeValue.toInt() == 2) {
      rgbStrip.controlMode = 2;
      if (timeValue != "") {
        if (timeValue.toInt() != NULL) {
          rgbStrip.timeControl = timeValue.toInt();
        }
      } 
    } else {
      message = "Invalid LED Mode Selected";
    }
  }
  if (brightValue != "") {
    rgbStrip.brightness = brightValue.toInt();
  }
  
  return message;
}

void handleLedArgs(void) {
  String message = "";

  message = handleServer();
  if (rgbStrip.controlMode == 0) {
    message = message + "Single Color Mode: R=" + rgbStrip.rgbLed.redValue + " G=" + rgbStrip.rgbLed.greenValue + " B=" + rgbStrip.rgbLed.blueValue;
  } else if (rgbStrip.controlMode == 1) {
    message = message + "Cycle LED Color Mode: Time-" + rgbStrip.timeControl;
  } else if (rgbStrip.controlMode == 2) {
    message = message + "Fade LED Mode: Time-" + rgbStrip.timeControl;
  }

  message = message + "\r\nSensors - Light:" + sensorsInfo.light + " Temp:" + sensorsInfo.temp + " Pressure:" + sensorsInfo.pressure;

  #if hasBme280
    message = message + " Humidity:" + sensorsInfo.humidity;
  #endif
  
  server.send(200, "text/plain", message);          //Returns the HTTP response
}

void handleLedArgsRaw(void) {
  String message = "";

  message = handleServer();
  byte* array = (byte*)&rgbStrip;
  for (int i = 0; i < 8; i++) {
    byte temp = (array[i] & 0xf0) >> 4;
    message = message + String(temp,HEX);
    temp = (array[i] & 0x0f);
    message = message + String(temp,HEX);
  }
  array = (byte*)&sensorsInfo;
  for (int i = 0; i < 16; i++) {
    byte temp = (array[i] & 0xf0) >> 4;
    message = message + String(temp,HEX);
    temp = (array[i] & 0x0f);
    message = message + String(temp,HEX);
  }
  
  server.send(200, "text/plain", message);          //Returns the HTTP response
}

/*void handleClient(void) {
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /mode/0") >= 0) {
              Serial.println("Setting RGB LED strip mode to 0");
              rgbStrip.controlMode = 0;
            } else if (header.indexOf("GET /mode/1") >= 0) {
              Serial.println("Setting RGB LED strip mode to 1");
              rgbStrip.controlMode = 1;
            } else if (header.indexOf("GET /mode/2") >= 0) {
              Serial.println("Setting RGB LED strip mode to 2");
              rgbStrip.controlMode = 2;
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            //client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");

            char controlMode;
            // Display current state, and ON/OFF buttons for GPIO 26  
            //client.println("<p>GPIO 26 - State " + output26State + "</p>");
            //client.println("<p>Current LED Mode: " + String(rgbStrip.controlMode) + "</p>");
            //client.println("<p>Current LED Mode: </p>");
            // If the output26State is off, it displays the ON button       
            /*if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
            if (rgbStrip.controlMode == 0) {
              client.println("<p><a href=\"/mode/1\"><button class=\"button\">MODE</button></a></p>");
            } else if (rgbStrip.controlMode == 1) {
              client.println("<p><a href=\"/mode/2\"><button class=\"button\">MODE</button></a></p>");
            } else { //if (rgbStrip.controlMode == 2) {
              client.println("<p><a href=\"/mode/0\"><button class=\"button\">MODE</button></a></p>");
            }
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            //client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            /*if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
} */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  ledcSetup(redChannel, freq, resolution);
  ledcAttachPin(RedPin, redChannel);
  ledcSetup(greenChannel, freq, resolution);
  ledcAttachPin(GreenPin, greenChannel);
  ledcSetup(redChannel, freq, resolution);
  ledcAttachPin(BluePin, blueChannel);

  ledcWrite(redChannel, 0);
  ledcWrite(greenChannel, 0);
  ledcWrite(blueChannel, 0);

  display.begin(SSD1306_SWITCHCAPVCC, oledAddr, false, SdaPin, SclPin, i2cFrequency);  // initialize with the I2C addr 0x3D (for the 128x64)

  //Wire.begin(SdaPin, SclPin, 100000);

  if(!tsl.begin(&Wire))
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  displaySensorDetails();
  configureSensor();

  if(!environSensor.begin(tempSenseAddr, &Wire)) {
    Serial.println("Could not find a valid BMx280 sensor, check wiring!");
    //while(1);
  }

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  //display.setFont(&FreeMono9pt7b);
  //display.setFont(&FreeSans9pt7b);
  display.setFont(&Picopixel);
  //display.setFont(&Org_01);
  display.setCursor(0,16);
  display.display();

  t.every(1, updateLed, (void *)0);
  t.every(250, updateDisplay, (void *)1);
  t.every(1, handleLedMode, (void *) 2);

  server.on("/", handleLedArgs);
  server.on("/raw", handleLedArgsRaw);

  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  //handleClient();
  server.handleClient();
  t.update();
}

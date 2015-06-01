/* TMP007_t3 Basic Example Code
 by: Kris Winer
 date: May 25, 2015
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic TMP007 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled pyrometry data out. Sketch runs on the 3.3 V Teensy 3.1.
 
 The TMP007 is an infrared (IR) thermopile sensor that measures the temperature of an object without 
 contacting the object. The integrated thermopile absorbs the infrared energy emitted from the object
 in the sensor field of view. The thermopile voltage is digitized and provided as an input to the integrated
 math engine, along with the die temperature (TDIE). The math engine then computes the corresponding object 
 temperature.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the TMP007 breakout board.
 
 Hardware setup:
 TMP007 Breakout ------ Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- pin 17 or 18
 SCL ----------------------- pin 16 or 19
 GND ---------------------- GND
 
  */
//#include "Wire.h"   
#include <i2c_t3.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

// See also TMP007 data sheet: http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00053090.pdf
//
////////////////////////////
// TMP007 Registers //
////////////////////////////
#define  TMP007_WHO_AM_I		0x1F  // Should be 0x78
#define  TMP007_SENSOR_VOLTAGE		0x00
#define  TMP007_LOCAL_TEMPERATURE       0x01
#define  TMP007_CONFIGURATION		0x02
#define  TMP007_OBJECT_TEMPERATURE      0x03
#define  TMP007_STATUS    		0x04
#define  TMP007_STATUD_MASK_AND_ENABLE  0x05
#define  TMP007_OBJECT_HIGH_LIMIT_TEMP  0x06
#define  TMP007_OBJECT_LOW_LIMIT_TEMP   0x07
#define  TMP007_LOCAL_HIGH_LIMIT_TEMP   0x08
#define  TMP007_LOCAL_LOW_LIMIT_TEMP    0x09
#define  TMP007_S0_COEFFICIENT 		0x0A
#define  TMP007_A0_COEFFICIENT 		0x0B
#define  TMP007_A1_COEFFICIENT 		0x0C
#define  TMP007_B0_COEFFICIENT 		0x0D
#define  TMP007_B1_COEFFICIENT 		0x0E
#define  TMP007_B2_COEFFICIENT 		0x0F
#define  TMP007_C_COEFFICIENT 		0x10
#define  TMP007_TC0_COEFFICIENT 	0x11
#define  TMP007_TC1_COEFFICIENT 	0x12
#define  TMP007_MEMORY_ACCESS 		0x2A


#define TMP007_ADDRESS  0x40 // Address of TMP007 accelerometer

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum Conversion_Rate {  // set of allowable conversion rates/averages per conversion
  CR_1 = 0,  // one average per conversion for total conversion time 0.26 s
  CR_2,
  CR_4,  // default
  CR_8,
  CR_16,
  CR_1_LOWPWR,  // one average but total conversion time 1 sec with 1//4 duty cycle for low current usage
  CR_2_LOWPWR,
  CR_4_LOWPWR
};

// Specify TMP007 configuration
uint8_t Conversion_Rate = CR_1;  // Set conversion rate/averaging
uint8_t TC = 1;  // Enable transient filtering

// Pin definitions
int intPin = 15;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;
int count = 0;
uint8_t temp[2] = {0, 0};  //dummy to hold

void setup()
{
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  I2Cscan();  // Identify all devices on the I2C bus
  
/*  
  display.begin(); // Initialize the display
  display.setContrast(40); // Set the contrast
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("TMP007");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("60 ug LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
*/

  // Read the WHO_AM_I registers, this is a good test of communication
  Serial.println("TMP007 9-axis motion sensor...");
  uint16_t c = readWord(TMP007_ADDRESS, TMP007_WHO_AM_I);  // Read WHO_AM_I register for TMP007  
  Serial.println("TMP007 IR thermopile"); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x0078, HEX);
  /*
  display.setCursor(20,0); display.print("TMP007");
  display.setCursor(0,10); display.print("I AM"); display.print(c, HEX);  
  display.setCursor(0,20); display.print("I Should Be"); display.print(0xD4, HEX); 
  display.setCursor(0,30); display.print("I AM"); display.print(d, HEX);  
  display.setCursor(0,40); display.print("I Should Be"); display.print(0x49, HEX); 
  display.display();
  delay(1000); 
  */

  if (c == 0x0078) // WHO_AM_I should always be 0x32 for the accelerometer
  {  
   Serial.println("TMP007 is online...");
 
 // Set conversion rate
 
  // Enable sensor by writing 1 in bit 12
  // Specify conversion rate/averaging
  // Specify transient correction filtering
  // Can also enable interrupt by setting bit 8 to 1
  writeWord(TMP007_ADDRESS, TMP007_CONFIGURATION, 0x1000 | Conversion_Rate << 9 | TC << 6);
  uint16_t d = readWord(TMP007_ADDRESS, TMP007_CONFIGURATION);  // Read CONFIGURATION register for TMP007  
  Serial.print("TMP007 CONFIGURATION"); Serial.println(d, HEX);  
 
  /* display.clearDisplay();
     
  display.setCursor(0, 0); display.print("TMP007bias");
  display.setCursor(0, 8); display.print(" x   y   z  ");

  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
  display.setCursor(72, 16); display.print("mg");
    
  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
  display.setCursor(66, 24); display.print("o/s");   
 
  display.display();
  delay(1000); 
 */ 
 
  }
  else
  {
    Serial.print("Could not connect to TMP007: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
 // Read status register
 uint16_t status = readWord(TMP007_ADDRESS, TMP007_STATUS);
 if(status & 0x0080) Serial.println("IR data overflow!");
 if(status & 0x0100) Serial.println("Memory corrupt!");
 if(status & 0x0200) Serial.println("Data invalid!");
 
 int16_t Sensor_Voltage = readWord(TMP007_ADDRESS, TMP007_SENSOR_VOLTAGE);
 Serial.print("Sensor Voltage = "); Serial.print((float)Sensor_Voltage*156.25, 2); Serial.println(" nanoVolts");

 int16_t Local_Temperature = readWord(TMP007_ADDRESS, TMP007_LOCAL_TEMPERATURE);
 Serial.print("DIE Temperature = "); Serial.print((float)(Local_Temperature/4)*0.03125, 2); Serial.print(" degrees C   "); Serial.print(9.*(float)(Local_Temperature/4)*0.03125/5. + 32., 2); Serial.println(" F");

 int16_t Object_Temperature = readWord(TMP007_ADDRESS, TMP007_OBJECT_TEMPERATURE);
 if(!(Object_Temperature & 0x0001)) {
   Serial.print("Object Temperature = "); Serial.print((float)(Object_Temperature/4)*0.03125, 2); Serial.print(" degrees C   "); Serial.print(9.*(float)(Object_Temperature/4)*0.03125/5. + 32., 2); Serial.println(" F");
 }
    
/*   
    display.clearDisplay();    
 
    display.setCursor(0, 0); display.print(" x   y   z ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
 
    display.setCursor(0, 40); display.print(altitude, 0); display.print("ft"); 
    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0); 
    display.setCursor(42, 40); display.print((float) sumCount / (1000.*sum), 2); display.print("kHz"); 
    display.display();
*/


    digitalWrite(myLed, !digitalRead(myLed));
    count = millis();  
    delay(500);
}

//===================================================================================================================
//====== Set of useful function to access acceleration and temperature data
//===================================================================================================================

// I2C scan function

void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

// I2C read/write functions for the TMP007

        void writeWord(uint8_t address, uint8_t subAddress, uint16_t data)
{
	uint8_t dest[2] = {0, 0};
        Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
        dest[0] = (data & 0xFF00) >> 8;
        dest[1] = (data & 0x00FF);
        Wire.write(dest[0]);              // Put data in Tx buffer
	Wire.write(dest[1]);              // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

       

        // Here, a word consists of two bytes since each register in the TMP007 is 2 bytes wide
        uint16_t readWord(uint8_t address, uint8_t subAddress)
{  
	uint8_t dest[2] = {0, 0};
        uint16_t data = 0;
        Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer 
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(address, (size_t) 2);  // Read two bytes from 16-bit slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
        data = ((uint16_t) dest[0] << 8) | dest[1];
        return data;
}


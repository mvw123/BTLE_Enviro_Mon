/*************************************************** 
 Name: BTLE enviro Mon 
 
 Description:
  Using a modified Arduino pro mini, modified HM-10 clone, 
  incorporating temperature/humidity and barometric pressure sensor. 
 
 Usage:  
  Connect to the device via Bluetooth serial or another arduino, 
  serial environmental data is then auto sent before auto disconnect occurs. 
  
 Origin: 
  Software modified from various examples including ADAfruit, spark fun, and others
  added OLED ascii driver instead of adafruit driver which uses too much memory in the Atmel 328 device.
 
 Details:
  SHT31 temperature and humidity sensor
  BMP280 temperature and barometric pressure sensor
  OLED 128x64 display (SPI) SSD1306
  HM-10 Bluetooth LE module Bolutek CC41-A - currently setup for Bolutek module AT commands which are different from the other modules on the market.
  Altitude - calculated from manually input sea level pressure and your local pressure
  Dew point - Calculated
  
  MW- HM10 module will not respond to AT commands whilst connected.
  MW- HM10 mnodule with wake when a large amount of data is received via Serial input. 
  MW- note that adafruit uses 0x77, wheras the devices bought from ebay use 0x76 I2C address. 
  MW- Address set to 0x76 in adafruit .h library due to this.
  MW- May need to reset downloader and pro mini before download due to a recurring bug.
  MW- use I2C scanner to check your I2C addresses if your not sure
  MW- LF and CR, ARE required for bolutek BLE module AT commands.  Not required for some other manufacturers modules. Therefore there can be incompatability between modules. 

 ****************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>  //requires seperate download of files from adafruit github
#include <Adafruit_BMP280.h>
#include "SSD1306Ascii.h"
//#include "SSD1306AsciiSoftSpi.h" //using hardware spi
#include "SSD1306AsciiSpi.h"
#include <SoftwareSerial.h>
//SoftwareSerial bleSerial(7, 8); // RX, TX  
SoftwareSerial bleSerial(12, 7); // RX, TX  
// see speed setup in setup function

//AltSoftSerial mySerial; //Arduino Uno, Duemilanove, LilyPad, Mini (& other ATMEGA328)  tx9 rx8 unusable pwm10
// uses fixed pin assignments

#include "LowPower.h"
// Use pin 2 as wake up pin

#define BT_EN_PIN       6 
#define BLE_STATE_PIN   2
const int wakeUpPin = BLE_STATE_PIN;

//OLED Hardware SPI pins

// Hardware SPI, sparkfun pinout , must indicate use of hardware SPI later
#define OLED_DC  8
#define OLED_RESET 9
#define OLED_CS   10
//#define OLED_DATA 11  //MOSI not defined here for hardware SPI 
//#define OLED_MISO 12 //not used 
//#define OLED_CLK  13 //SPI CLK not defined here for hardware SPI
/*
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10
*/

/*
#define OLED_DC   11
#define OLED_CS   12
#define OLED_CLK  10
#define OLED_DATA 9
#define OLED_RESET 13
*/
//SSD1306AsciiSoftSpi oled; // uses the soft spi 
SSD1306AsciiSpi oled; //uses hardware spi

Adafruit_BMP280 bme; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

Adafruit_SHT31 sht31 = Adafruit_SHT31();

//Globals
float seaLevelPressure = 1013.25; // Average sea-level pressure is 1013.25 mbar, to be set manually later in s/w
float sht_t = 100.00;
float sht_h = 100.00;
float myDewPt = 100.00;
float BMP280_Temperature = 100.00;
float BMP280_Pressure = 100.00;
float BMP280_Altitude = 100.00;

double dew_a = 6.112; //millibar
double dew_b = 17.62;
double dew_c = 243.12; // degrees C
double dew_temporary = 0;

//Power line analogue ADC measurement
const int analogInPin = A0;         // Analog input pin that the potentiometer is attached to
const float internalAREF = 1.10;    //1.1v on atmega 328p
int inputADCValue = 0;              // value read from the analog input
float powerMonitorR1 = 240000; //ohm, potential divider for power line monitor
float powerMonitorR2 = 75000; //ohm, potential divider for power line monitor
float vBattDivider = 0.00; 
float vBatt = 0.00;

/********************************************/

void setup() {
  // Configure wake up pin as input.
  // This will consumes few uA of current.
  //pinMode(wakeUpPin, INPUT);  // STATE pin is configured as same pin elsewhere
  
  //Power line analogue ADC measurement
  analogReference(INTERNAL); //INTERNAL = 1.1V ref
  
  //setup hardware serial
  Serial.begin(9600);
  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens  
  Serial.print("Init");   

  // If the baudrate of the HM-10 module has been updated,
  // you may need to change 9600 by another value
  // Once you have found the correct baudrate,
  // you can update it using AT+BAUDx command 
  // e.g. AT+BAUD0 for 9600 bauds
  bleSerial.begin(9600);
  delay(10);
    
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  //bluetooth initialise
  initBluetooth();
  
  //OLED Display
  initOLED();
   
  //Init SHT31 temperature and humidity sensor
  Serial.println(F("SHT31 test"));
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println(F("Couldn't find SHT31"));
    while (1) delay(1);
  }

  //Init BMP280 Barometric pressure sensor
  Serial.println(F("BMP280 test"));
  if (!bme.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
}

/***********************/

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1);                         // wait ms
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  //delay(1000);                    // wait ms
  
  //Serial.println("Loop begins.");
  
  //measure the battery voltage
  VCC_Voltage();
  
  //Obtain sensor data 
  //set BMP280 into forced mode so that it will return to standby after readings are complete.
  Wire.beginTransmission((uint8_t)BMP280_ADDRESS);
  Wire.write((uint8_t)BMP280_REGISTER_CONTROL); //only ever called once in the "Adafruit_BMP280.cpp" 
  Wire.write((uint8_t)0x3D);//modified from 3F (normal mode) to forced mode 0x3D, placed here so we don't have to update the library
  Wire.endTransmission();
  delay(300);
  getSensorData();
  
  //output sensor data
  //debugPrintEnviroData();   //Print debug data to serial port
  sendSensorDataToOLED();
  sendSensorDataToBluetooth();
  
  //Reset bluetooth board to disconnect from remote end
  toggleEnableBLE();
  Serial.println("Remote connection terminated.");
  //initBluetooth(); //BUG: outputs AT commands over the bluetooth i'face
  //Serial.println("BLE Enable not triggered");
  
  //transfer serial data between UARTs BLE serial <-> hardware serial
  bleSerialToHarwareSerial();//BUG: is there contention on the enable line to ble? have seen 2.4v and other voltages indicating contention.  wire to reset line instead. also use a 1k resistor as before

  //enter low power mode
  bleSerial.end();
  pinMode(7, INPUT); // TX to input so that it does not drive into a powered down peripheral.
  digitalWrite(7, LOW); 
  delay(100);
  
  wakeOnBluetoothInterrupt();
  //bluetooth Enable line will  go to a FET or regulator EN input so leave GPIO as output.
  
  //exit low power mode and setup software serial again
  pinMode(7, OUTPUT); // TX to input so that it does not drive into a powered down peripheral.
  bleSerial.begin(9600);
  delay(10);

  //initBluetooth();
  
}


/********************************************/

void wakeOnBluetoothInterrupt()
{
    /*
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(20);                         // wait ms
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(2000);                    // wait ms
    */
    
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(0, wakeUp, HIGH); //STATE pin goes high when connected. needs to stay high long enough for interrupt to be generated when MCU powers up fully. 
    
    Serial.println("Going to sleep."); 
    bleSerial.println("Going to sleep.");      
    delay(300);

    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 

    // ***waits here until BLE interrupt triggers exit from power down mode***
    
    delay(1000); //delay to allow everything to setup at both ends
    Serial.println("Woken up.");  
    delay(300);
    
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0); 
    
    // Do something here
    // Example: Read sensor, data logging, data transmission.
    Serial.println("Exit Wake on bluetooth interrupt function.");  

}

/********************************************/

void wakeUp()
{
    // Just a handler for the pin interrupt.
}

/********************************************/

//OLED Display
void initOLED(void){
  //Reset OLED
  pinMode(OLED_RESET, OUTPUT);      // sets the digital pin as output
  digitalWrite(OLED_RESET, LOW);    // sets the rst low
  delay(300);                       // waits for >200ms
  digitalWrite(OLED_RESET, HIGH);   // sets the LED on

  // run driver
  //oled.begin(&Adafruit128x64, OLED_CS, OLED_DC, OLED_CLK, OLED_DATA); //software spi
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC); //hardware spi
  oled.setFont(Adafruit5x7); 
  oled.clear(); 
}

/********************************************/

void initBluetooth(void){
  //Bluetooth bolutek HM-10 module
  //software serial data  
  //LF and CR required on bolutek modules so use println instead of print  
  char charToPrint = 'm';
  int i = 0;
  int charsRequiredForWakeup = 80;  

  pinMode(BT_EN_PIN, OUTPUT);       //initialise the enable pin to HIGH. A LOW will shut it down.
  //digitalWrite(BT_EN_PIN, HIGH); // for regulator
  digitalWrite(BT_EN_PIN, LOW);  // for fet in place of regulator
  pinMode(BLE_STATE_PIN, INPUT);    //set the BLE state pin to an input.
  //toggleEnableBLE();
  delay(100);
  for (i=0; i<=charsRequiredForWakeup; i++){ //wake up bolutek HM10 from sleep mode requires 80+ chars
    bleSerial.print(charToPrint);  
  }
  bleSerial.println("");
  delay(500);
  bleSerialToHarwareSerial(); 
  bleSerial.println("AT");  //bolutek version of the HM-10 (clone) requires LF and NL for AT responses (boultek) AT+HELP gives AT commands, so use Serial.println  with ln at the end of the Serial.print statement
  delay(300);  
  bleSerialToHarwareSerial(); 
  bleSerial.println("AT+RENEW");  //set back to defaults
  delay(1500);  
  bleSerialToHarwareSerial(); 
  bleSerial.println("AT+NAMECC41-A");  //device name
  delay(500);  
  bleSerialToHarwareSerial();   
  //bleSerial.println("AT+ADVI0");      //Set advertising interval to 0 ( milliseconds) 
  bleSerial.println("AT+ADVI5");      //Set advertising interval to 5 (546.25 milliseconds) 
  //bleSerial.println("AT+ADVI9");        //Set advertising interval to 9 (1285 milliseconds), reduces current consumption
  delay(300); 
  bleSerialToHarwareSerial(); 
  bleSerial.println("AT+PWRM0"); // sets it to auto power down mode, but its buggy so in some circumstances it needs to be told to go back to sleep via use of en low/high. AT+PWRM1 is no auto sleep (default)
  delay(300); 
  bleSerialToHarwareSerial(); 
  bleSerial.println("AT+RESET"); // needs a reset for changes to take effect. 
  delay(2000);  

  // un comment this to set up a Master and connect to a Slave
  /*
  Serial.println("BLE CC41-A Bluetooth");
  Serial.println("----------------------------------");
  Serial.println("");
  Serial.println("Trying to connect to Slave Bluetooth");
  delay(1000);
  //AT+RENEW - restores to factory settings
  //AT+RESET - software reset
  bluetooth.println("AT"); // just a check
  delay(2000);
  bluetooth.println("AT+ROLE1"); // set up as Master
  delay(2000);
  bluetooth.println("AT+INQ"); // look for nearby Slave
  delay(5000);
  bluetooth.println("AT+CONN1"); // connect to it (should check its the right one first via its Address.)
  */

  //next section of code would be better for connecting to a device when in master mode, but doesn't appear to work on the bolutek modules. 
  /*
  send: AT+CON[Para1] 
  return: OK+CONN[Para2] 
  Para1: Address
  Like: 0017EA090909
  Para2: A, E, F
  A: Connecting
  E: Connect error
  F: Connect Fail
  */
 
}

/********************************************/

void getSensorData(void){
  sht_t = sht31.readTemperature();
  sht_h = sht31.readHumidity();

  //Soft reset
  //sht31.reset() //maybe include this in the setup routine!
  //Heater to evaporate condensation
  //sht31.heater(true)
  //sht31.heater(false)

  //BMP280 barometric pressure and temperature sensor - from adafruit example
  //First reading should be ignored, previous readings are saved in memory
  BMP280_Temperature = bme.readTemperature();
  BMP280_Pressure = bme.readPressure()/100; //read pressure in Pa and convert to hPa which is equiv to mbar and easier to read.
  /*
  The altitude calculation depends on knowing the barometric pressure at sea level
  If you do not set the correct sea level pressure for your location FOR THE CURRENT DAY
  it will not be able to calculate the altitude accurately
  Barometric pressure at sea level changes daily based on the weather!
  */
  seaLevelPressure = 1013.25; //Sea level barometric pressure for calibration of altitude, should update this from an online resource 
  BMP280_Altitude = bme.readAltitude(seaLevelPressure);
}

/********************************************/

//OLED display sensor data
void sendSensorDataToOLED(void){
/*
  //OLED
  uint32_t m = micros();
  //oled.clear(); 
  oled.println("Hello world!");
  oled.println("A long line may be truncated");
  oled.println();
  oled.set2X();
  oled.println("2X demo");
  oled.set1X();
  oled.print("\nmicros: ");
  oled.print(micros() - m);
*/
  //Dew Point via Magnus formula approximation from wikipedia Sonntag1990, -45C<T<+60C (+/-0.35C) https://en.wikipedia.org/wiki/Dew_point
  dew_temporary = log(sht_h/100) + ((dew_b*sht_t)/(dew_c+sht_t)); //double  log (double __x)   // natural logarithm of x
  myDewPt = (dew_c*dew_temporary)/(dew_b-dew_temporary);  

  oled.setCursor(0, 0);  // Set the text cursor to the upper-left of the screen.  
  oled.println("Environment Monitor"); //top line
  oled.println(" ");
  oled.print("TEMP: "); oled.print(sht_t); oled.println(" c   ");    //Write TEMPERATURE
  oled.print("HUMI: "); oled.print(sht_h); oled.println(" %RH   ");  //Write HUMIDITY
  oled.print("DEWP: "); oled.print(myDewPt); oled.println(" c   ");  //Write Dew Point  
  oled.print("PRES: "); oled.print(BMP280_Pressure); oled.println(" hPa   ");   //Write BAROMETRIC PRESSURE
  oled.print("ALTI: "); oled.print(BMP280_Altitude); oled.println(" m    ");   //Write ELEVATION from BAROMETRIC PRESSURE
  //BUG: Add cal or uncal to the altitude output
  oled.print("SEACAL: "); oled.print(seaLevelPressure); oled.println(" mbar");   //Write sea level cal pressure for altitude calc 
  //oled.println("ENTROPY: (RISING)"); 
  oled.println("");
  //delay(1000); 
}

/********************************************/

void sendSensorDataToBluetooth(void){
//print to bluetooth
//read the BLE state pin, which is high when connected to the bluetooth, (must be wired up)
  if (digitalRead(BLE_STATE_PIN)==HIGH){ 
    //i.e. you have connected via bluetooth.
    delay(500);
    Serial.println("Remote connection started.");
    
    int printDelay=100; //due to print fails add delay
    
    bleSerial.println(" ");
    bleSerial.println(" ");    
    bleSerial.println("Environment Monitor"); //top line
    bleSerial.println("===================");
    delay(printDelay);
    bleSerial.print("TEMP: "); bleSerial.print(sht_t); bleSerial.println(" ºC (SHT31 ±0.3ºC)");    //Write TEMPERATURE
    delay(printDelay);
    bleSerial.print("HUMI: "); bleSerial.print(sht_h); bleSerial.println(" %RH (SHT31 ±2%)");  //Write HUMIDITY
    delay(printDelay);
    bleSerial.print("DEWP: "); bleSerial.print(myDewPt); bleSerial.println(" ºC (CALC'D ±0.35ºC)");  //Write Dew Point  
    delay(printDelay);
    //BMP280 temperature is not very accurate 
    //bleSerial.print("TEMP: "); bleSerial.print(BMP280_Temperature); bleSerial.println(" ºC (BMP280 ±1.0ºC)");    //Write TEMPERATURE 
    //delay(printDelay);
    bleSerial.print("PRES: "); bleSerial.print(BMP280_Pressure); bleSerial.println(" hPa (BMP280 ±1.0hPa)");   //Write BAROMETRIC PRESSURE BMP280
    delay(printDelay);
    bleSerial.print("ALTI: "); bleSerial.print(BMP280_Altitude); bleSerial.println(" m (UNCAL)");   //Write ELEVATION from BAROMETRIC PRESSURE BMP280
    delay(printDelay);
    bleSerial.print("SEACAL: "); bleSerial.print(seaLevelPressure); bleSerial.println(" mbar (UNCAL)");   //Write manually set sea level cal pressure for altitude calc 
    //bleSerial.println("ENTROPY: (RISING)"); 
    delay(printDelay);
    if (vBatt > 4.2) {
          bleSerial.print("VBATT: "); bleSerial.print(vBatt); bleSerial.println(" V"); bleSerial.print("*****BATTERY OVERVOLTAGE***** ");  
    }
    else if ((vBatt >= 2.8) && (vBatt <= 4.2)) {
          bleSerial.print("VBATT: "); bleSerial.print(vBatt); bleSerial.println(" V"); 
    }
    else if (vBatt < 2.8) {
          bleSerial.print("VBATT: "); bleSerial.print(vBatt); bleSerial.println(" V"); bleSerial.print("*****BATTERY LOW***** ");  
    }
    else {
          bleSerial.print("VBATT: "); bleSerial.print(vBatt); bleSerial.println(" V"); bleSerial.print("*****BATTERY READING ERROR***** "); 
    }
    delay(printDelay);
    bleSerial.println("Note hPa = mbar\n");
    delay(printDelay);

  }
}

/********************************************/

void VCC_Voltage(void){
  // read the analog in value:
  inputADCValue = analogRead(analogInPin);
  vBattDivider = float((inputADCValue * internalAREF)/1023); //ADC = (VIN*1023) / VREF.  VIN=(ADC*VREF)/1023. if we divide the input voltage by two then we multiply result by 2
  vBatt = ((powerMonitorR1 + powerMonitorR2) * vBattDivider) / powerMonitorR2;
  //print the results to the serial monitor:
  //Serial.print("ADC value = ");  Serial.print(inputADCValue);
  //Serial.print("\t input voltage = ");  Serial.println(powerLineVoltage);

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);

  return;
}

/********************************************/

void  bleSerialToHarwareSerial(void){
  int i = 0;
  for (i=0; i<50; i++){
    //bluetooth serial <=> hardware serial
    char char_ser;
    byte toggleOUTPUT = 0;
    if (Serial.available()) {
      //delay(10); //removed due to causing AT+HELP to return garbage
      char_ser = Serial.read();
      if (char_ser == '*') {  
        toggleOUTPUT = !digitalRead(BT_EN_PIN); //state pin seems to be an output on the CC41-A, perhaps reconfiguring the AT commands would change it. MODE?
        digitalWrite(BT_EN_PIN, toggleOUTPUT); 
        Serial.println(toggleOUTPUT);
        }
      else  {
        bleSerial.print(char_ser);
        }
    }
    if (bleSerial.available()) {
      //delay(10); //removed due to causing AT+HELP to return garbage
      char_ser = bleSerial.read();
      Serial.print(char_ser);    
    }
  }
}

/********************************************/

void toggleEnableBLE(void){
/*
    //Disable bluetooth to drop connection, for power purposes. 
    // BTLE module will then come back up in auto low power mode due to previous AT+PWRM0 auto power down
    digitalWrite(BT_EN_PIN, LOW);
    Serial.println("BT_EN_PIN LOW");    
    delay(1100); 
    digitalWrite(BT_EN_PIN, HIGH);
    Serial.println("BT_EN_PIN HIGH");   
    delay(1100);
*/

    //P channel FET instead of regulator
    //Disable bluetooth to drop connection, for power purposes. 
    // BTLE module will then come back up in auto low power mode due to previous AT+PWRM0 auto power down
    digitalWrite(BT_EN_PIN, HIGH);
    Serial.println("BT_EN_PIN HIGH");    
    delay(1100); 
    digitalWrite(BT_EN_PIN, LOW);
    Serial.println("BT_EN_PIN LOW");   
    delay(1100);
}

/********************************************/

void debugPrintEnviroData(void){

//Temperature
  if (! isnan(sht_t)) {  // check if 'is not a number'
    Serial.print("SHT31: Temp *C = "); Serial.print(sht_t);  Serial.println(" [ +/-0.3C (typ acc) ]");
  } else { 
    Serial.println("Failed to read SHT31 temperature");
  }

//Humidity
  if (! isnan(sht_h)) {  // check if 'is not a number'
    Serial.print("SHT31: Hum. %RH = "); Serial.print(sht_h); Serial.println("  [ +/-2% (typ acc) ]");
  } else { 
    Serial.println("Failed to read SHT31 humidity");
  }
  Serial.println();
//BMP280
  Serial.print(F("BMP280: Temperature = "));  
  Serial.print(BMP280_Temperature);
  Serial.print(" *C");
  Serial.println("  [ +/-0.5C (typ acc @ 25C), +/-1.0C (0-65C) ]");
  
  Serial.print(F("BMP280: Pressure = "));
  Serial.print(BMP280_Pressure);
  Serial.print(" hPa");
  Serial.println("  [ +/-1.7hPa (@-20 to 0C), +/-1.0hPa (0-65C) ]");

  Serial.print(F("BMP280: Approx altitude = "));  //should be 127m + 4m = 131 metres approx for MVW upstairs home. 
  //Serial.print(bme.readAltitude(1013.25)); // millibars, this should be adjusted to your local forcast
  Serial.print(BMP280_Altitude); // this should be adjusted to your local forcast  
  Serial.println(" m");
  Serial.println();

  //voltage
  Serial.print("Voltage: ");  Serial.print(vBatt);    Serial.println("V"); 
  delay(1000);
  
}

/********************************************/



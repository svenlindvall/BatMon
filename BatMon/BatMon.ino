#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
int eepromAddr = 0;
int val = 0;
byte x = 0;
const int anaTempPin = A0;
int tempVal = 0; 
int sensVal = 0;
SoftwareSerial serLCD(10, 11); // RX, TX

unsigned long currentMillis;     // store the current value from millis()
unsigned long previousMillis;    // for comparison with currentMillis
unsigned int samplingInterval = 32;  // default sampling interval is 33ms
unsigned int i2cReadDelayTime = 0;  // default delay time between i2c read request and Wire.requestFrom()

struct batInfo_t {
  float volt;
  float amps;
  float ah;
};

batInfo_t batInfo[2];
byte i2cRxData[32];


void setup()  
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.println("Batteri Monitor V.1.0.0!");

  ina219.begin();
  
  // set the data rate for the SoftwareSerial port
  serLCD.begin(9600);
  serLCD.println("Batteri Monitor V.1.0.0!");
}




void loop() // run over and over
{
  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) 
  {
    previousMillis += samplingInterval;

    if (Serial.available()) serLCD.write(Serial.read());
      
    Wire.requestFrom(2, 6);    // request 6 bytes from slave device #2
  
    while(Wire.available())    // slave may send less than requested
    { 
      char c = Wire.read(); // receive a byte as character
      Serial.print(c);         // print the character
      
      val = int(c);
      EEPROM.write(eepromAddr, val);
      eepromAddr = eepromAddr + 1;
      if (eepromAddr == 512) eepromAddr = 0;
    
    }
  
    Wire.beginTransmission(4); // transmit to device #4
    Wire.write(x);              // sends one byte  
    Wire.endTransmission();    // stop transmitting
  
    sensVal = analogRead(anaTempPin);
    tempVal = map(sensVal, 0, 1023, 0, 255); 
    
    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
  
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.println("");
  }
}

//  DA-IICT - Gandhinagar
//  EL 213 - Analog Circuits
//  Group 2 :--> Accident Detector using Vibration Sensor and Accelerometer.
//               Sending Message of Accident Detection of particular location by GPS
//               through GSM Module.

//  ====****Hardware****====
//  - Arduino Uno
//  - Accelerometer Sensor :--> MPU-6050
//  - Vibration Sensor :--> SW420
//  - GSM Module :--> SIM 900A
//  - GPS Module :--> NEO-6m Module
//  - Buzzer

//  ====****Software****====
//  - Arduino IDE v1.8.9
//  - Arduino Wire Library
//  - Arduino SoftwareSerial Library
//  - Arduino TinyGPS++ Library

//  ====****Arduino Code****====

#include <Wire.h>
#include<SoftwareSerial.h>
#include<TinyGPS++.h>

long accelX, accelY, accelZ;
long gyroX, gyroY, gyroZ;
float gForceX, gForceY, gForceZ;
float rotX, rotY, rotZ;

int i = 0, j = 0;
int buzzer_Pin = 12;
int vibr_Pin = 3;
long measurement = 0;

SoftwareSerial mySerial(10, 11);          // Tx = 10 & Rx = 11 of GSM
SoftwareSerial serial_connection(9, 8);   // Tx = 9 & Rx = 8 of GPS
TinyGPSPlus gps;                          // This is the GPS object that will pretty much do all the grunt work with the NMEA data

void setup()
{
  mySerial.begin(9600);                   // This opens up communications to the GSM
  Serial.begin(9600);                     // This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);          // This opens up communications to the GPS
  Wire.begin();
  setupMPU();
  pinMode(buzzer_Pin, OUTPUT);            // Set buzzer_Pin Output for measurement
  pinMode(vibr_Pin, INPUT);               // Set vibr_Pin Input for measurment
}

void loop()
{
  while(serial_connection.available())        // While there are characters to come from the GPS
      gps.encode(serial_connection.read());   // This feeds the serial NMEA data into the library one char at a time
     
  if(Serial.available() > 0)
  {
    while(i < 5000)
    {
      measurement = TP_init();
      delay(50);
      recordGyroRegisters();
      printData();
      delay(100);
      i++;
    }
   }
   if(mySerial.available() > 0)
      Serial.write(mySerial.read());
}

long TP_init()
{
  delay(10);
  long measurement = pulseIn(vibr_Pin, HIGH);  // Wait for the pin to get HIGH and Returns measurement
  return measurement;
}

void setupMPU()
{
  Wire.beginTransmission(0b1101000);           // This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high)
  Wire.write(0x6B);                            // Accessing the register 6B - Power Management
  Wire.write(0b00000000);                      // Setting SLEEP register to 0
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000);           // I2C address of the MPU
  Wire.write(0x1B);                            // Accessing the register 1B - Gyroscope Configuration 
  Wire.write(0x00000000);                      // Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000);           // I2C address of the MPU
  Wire.write(0x1C);                            // Accessing the register 1C - Acccelerometer Configuration 
  Wire.write(0b00000000);                      // Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters()
{
  Wire.beginTransmission(0b1101000);           // I2C address of the MPU
  Wire.write(0x3B);                            // Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);               // Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();         // Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read();         // Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read();         // Store last two bytes into accelZ
  processAccelData();
}

void processAccelData()
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters()
{
  Wire.beginTransmission(0b1101000);           // I2C address of the MPU
  Wire.write(0x43);                            // Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);               // Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read();          // Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read();          // Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read();          // Store last two bytes into accelZ
  processGyroData();
}

void processGyroData()
{
    rotX = gyroX / 131.0;
    rotY = gyroY / 131.0; 
    rotZ = gyroZ / 131.0;
}

void printData()
{
    Serial.print("Vibration Detect : ");
    Serial.print(measurement);
    Serial.print("\n");
    Serial.print("Gyro(deg):");
    Serial.print(" X = ");
    Serial.print(rotX);
    Serial.print(" Y = ");
    Serial.print(rotY);
    Serial.print(" Z = ");
    Serial.print(rotZ);
    Serial.print("\n");
    Serial.print("Accel(g): ");
    Serial.print(" X = ");
    Serial.print(gForceX);
    Serial.print(" Y = ");
    Serial.print(gForceY);
    Serial.print(" Z = ");
    Serial.println(gForceZ);
    Serial.print("\n");
  
    Serial.print("Satellite Count : ");
    Serial.print(gps.satellites.value());
    Serial.print("\n");
    Serial.print("Latitude : ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("\n");
    Serial.print("Longitude : ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("\n");
    Serial.print("Speed MPH : ");
    Serial.print(gps.speed.mph());
    Serial.print("\n");
    Serial.print("Altitude Feet : ");
    Serial.print(gps.altitude.feet());
    Serial.print("\n");
  
    if((rotX > 220 || rotX < -220) || (rotY > 220 || rotY < -220) || (rotZ > 220 || rotZ < -220) || (gForceX > 1.25) || (gForceY > 1.25) || (gForceZ > 1.25) || (measurement > 70000))
    {
        Serial.println("Accident has been detected..");
        SendMessage();
        while(j <= 1)
        {
            digitalWrite(buzzer_Pin, HIGH);
            delay(1000);
            j++;
        }
        j=0;
    }
    else
        digitalWrite(buzzer_Pin, LOW);
}


void SendMessage()
{
  
  Serial.println("Message Sending...");
  
  mySerial.println("AT+CMGF=1");                      // Sets the GSM Module in Text Mode
  delay(1000);                                        // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+919081435928\"\r");    // Send Message to Mobile Number
  delay(1000);
  mySerial.println("Attention!!! Accident Occured at Location");
  mySerial.print("");
  mySerial.print("http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
  mySerial.print(gps.location.lat(), 6);
  mySerial.print("+");
  mySerial.print(gps.location.lng(), 6);
  mySerial.println(" \nNeed Emergency ASAP!!!");
  delay(100);
  
  mySerial.println((char)26);                         // ASCII code of CTRL+Z
  delay(1000);
  Serial.println("Message has been sent successfully.");
}

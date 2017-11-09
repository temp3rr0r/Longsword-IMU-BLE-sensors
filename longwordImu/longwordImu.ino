
#include "CurieIMU.h"
#include <CurieBLE.h>

BLEPeripheral blePeripheral; // BLE Peripheral Device (the board you're programming)
BLEService imuService("180F"); // Custom UUID

/** 
*  A float in C takes up 4 bytes of space. We will send the x, y and z components of both the
*  accelerometer and gyroscope values. The byte length of 12 is specified as the last value
*  passed in the function here. Float data is stored in memory as little endian. There are many
*  good tutorials on little and big endian, here is a link to one of them:
*  (https://www.cs.umd.edu/class/sum2003/cmsc311/Notes/Data/endian.html).
*  BLE limits us to 20 bytes of data to transmit. Therefore we cannot send all 6 floats using one 
*  characteristics. So we will create one characteristic for each main element of the IMU.
*/
BLECharacteristic imuAccCharacteristic("3A19", BLERead | BLENotify, 12 );
BLECharacteristic imuAccCharacteristic2("3A20", BLERead | BLENotify, 12 );
BLEDescriptor imuAccDescriptor("2902", "block");
#define BLE_CONNECT 3 // This pin will service as a hardware confirmation of the BLE connection
#define INDICATOR_LEDA 4 // This pin will be used to debug input buttons from mobile app

/**
* The union directive allows 3 variables to share the same memory location. Please see the 
* tutorial covering this project for further discussion of the use of the union
* directive in C.
*
*/
union 
{
int ag[3];
unsigned char bytes[12];         
} accGyroData;


void setup() {

  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(2); // 2g
  CurieIMU.setGyroRange(250); // 250 degrees/second
  pinMode(BLE_CONNECT, OUTPUT);
  pinMode(INDICATOR_LEDA, OUTPUT);
  
  blePeripheral.setLocalName("imu");
  blePeripheral.setAdvertisedServiceUuid(imuService.uuid());  // add the service UUID
  blePeripheral.addAttribute(imuService);   
  blePeripheral.addAttribute(imuAccCharacteristic);
  blePeripheral.addAttribute(imuAccDescriptor);
  blePeripheral.addAttribute(imuAccCharacteristic2);
  
  const unsigned char initializerAccGyro[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 }; 
  imuAccCharacteristic.setValue( initializerAccGyro, 12);
  imuAccCharacteristic2.setValue( initializerAccGyro, 12);  
  blePeripheral.begin();
}

void loop() {

  int axRaw, ayRaw, azRaw;         // raw accelerometer values
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  
  BLECentral central = blePeripheral.central();
  if (central) {
    digitalWrite(BLE_CONNECT, HIGH);
    while (central.connected()) {
      CurieIMU.readAccelerometer(axRaw, ayRaw, azRaw);      
      CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);      
      accGyroData.ag[0] = axRaw;
      accGyroData.ag[1] = ayRaw;
      accGyroData.ag[2] = azRaw;
      unsigned char *accGyro = (unsigned char *)&accGyroData;      
      imuAccCharacteristic.setValue( accGyro, 12 );

      accGyroData.ag[0] = gxRaw;
      accGyroData.ag[1] = gyRaw;
      accGyroData.ag[2] = gzRaw;      
      unsigned char *accGyro2 = (unsigned char *)&accGyroData;
      imuAccCharacteristic2.setValue( accGyro2, 12 );

            
    } // while central.connected  
  } // if central
} // end loop(){}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767  
  return (aRaw * 2.0) / 32768.0;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  return (gRaw * 250.0) / 32768.0;
}

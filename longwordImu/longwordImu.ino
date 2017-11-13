
#include "CurieIMU.h"
#include <CurieBLE.h>

#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

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
BLECharacteristic imuAccCharacteristic("3A19", BLERead | BLENotify, 12 ); // Accelerometer
BLECharacteristic imuAccCharacteristic2("3A20", BLERead | BLENotify, 12 ); // Gyroscope
BLECharacteristic imuAccCharacteristic3("3A21", BLERead | BLENotify, 12 ); // Steps
BLECharacteristic imuAccCharacteristic4("3A22", BLERead | BLENotify, 12 ); // Accelerometer2
BLECharacteristic imuAccCharacteristic5("3A23", BLERead | BLENotify, 12 ); // Gyroscope2
BLECharacteristic imuAccCharacteristic6("3A24", BLERead | BLENotify, 12 ); // Magnetometer
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

int lastStepCount = 0;
unsigned long microsPerReading, microsPrevious;

void setup() {

  /* Initialise the IMU */
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);
  
  CurieIMU.setAccelerometerRange(2); // 2g
  CurieIMU.setGyroRange(250); // 250 degrees/second
  pinMode(BLE_CONNECT, OUTPUT);
  pinMode(INDICATOR_LEDA, OUTPUT);
  // turn on step detection mode:
  CurieIMU.interrupts(CURIE_IMU_STEP);  // turn on step detection
  CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_NORMAL);  
  CurieIMU.setStepCountEnabled(true); // enable step counting:  
  
  blePeripheral.setLocalName("imu");
  blePeripheral.setAdvertisedServiceUuid(imuService.uuid());  // add the service UUID
  blePeripheral.addAttribute(imuService);   
  blePeripheral.addAttribute(imuAccCharacteristic);
  blePeripheral.addAttribute(imuAccDescriptor);
  blePeripheral.addAttribute(imuAccCharacteristic2);
  blePeripheral.addAttribute(imuAccCharacteristic3);
  blePeripheral.addAttribute(imuAccCharacteristic4);
  blePeripheral.addAttribute(imuAccCharacteristic5);
  blePeripheral.addAttribute(imuAccCharacteristic6);

  // Accel2, Gyro2, Temp
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)  
  Wire.endTransmission(true);
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  const unsigned char initializerAccGyro[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };
  imuAccCharacteristic.setValue( initializerAccGyro, 12);
  imuAccCharacteristic2.setValue( initializerAccGyro, 12);
  imuAccCharacteristic3.setValue( initializerAccGyro, 12);
  imuAccCharacteristic4.setValue( initializerAccGyro, 12);  
  imuAccCharacteristic5.setValue( initializerAccGyro, 12);
  imuAccCharacteristic6.setValue( initializerAccGyro, 12);  
  blePeripheral.begin();

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {

  unsigned long microsNow;
  int axRaw, ayRaw, azRaw;         // raw accelerometer values
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  
  BLECentral central = blePeripheral.central();
  if (central) {
    digitalWrite(BLE_CONNECT, HIGH);
    while (central.connected()) {      
      // check if it's time to read data and update the filter
      microsNow = micros();      
      if (microsNow - microsPrevious >= microsPerReading) {
        
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
        imuAccCharacteristic2.setValue((unsigned char *)&accGyroData, 12 );      

        // Accel 2
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
        AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
        AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        accGyroData.ag[0] = 1;//AcX;
        accGyroData.ag[1] = 1;//AcY;
        accGyroData.ag[2] = 1;//AcZ;
        imuAccCharacteristic4.setValue((unsigned char *)&accGyroData, 12 );        

        // Gyro 2
        accGyroData.ag[0] = 1;//GyX;
        accGyroData.ag[1] = 1;//GyY;
        accGyroData.ag[2] = 1;//GyZ;
        imuAccCharacteristic5.setValue((unsigned char *)&accGyroData, 12 );        

        // Magnetometer
        int x,y,z; //triple axis data        
        //Tell the HMC5883L where to begin reading data
        Wire.beginTransmission(address);
        Wire.write(0x03); //select register 3, X MSB register
        Wire.endTransmission();
        //Read data from each axis, 2 registers per axis
        Wire.requestFrom(address, 6);
        if (6<=Wire.available()){
          x = Wire.read()<<8; //X msb
          x |= Wire.read(); //X lsb
          z = Wire.read()<<8; //Z msb
          z |= Wire.read(); //Z lsb
          y = Wire.read()<<8; //Y msb
          y |= Wire.read(); //Y lsb
        }
        accGyroData.ag[0] = 1;//x;
        accGyroData.ag[1] = 1;//y;
        accGyroData.ag[2] = 1;//z;
        imuAccCharacteristic6.setValue((unsigned char *)&accGyroData, 12 );        

        // Step Counter, Temp
        updateStepCount();
        accGyroData.ag[0] = lastStepCount;
        accGyroData.ag[1] = Tmp; // celcius = (Tmp/340.00+36.53);  // TODO: equation for temperature in degrees C from datasheet
        accGyroData.ag[2] = lastStepCount;
        imuAccCharacteristic3.setValue((unsigned char *)&accGyroData, 12 );  

        microsPrevious = microsPrevious + microsPerReading; // increment previous time, so we keep proper pace
      }
    } // while central.connected  
  } // if central
} // end loop(){}


static void updateStepCount() {  
  int stepCount = CurieIMU.getStepCount(); // get the step count  
  if (stepCount != lastStepCount)// if the step count has changed, print it   
    lastStepCount = stepCount; // save the current count for comparison next check
}

static void eventCallback(){
  if (CurieIMU.stepsDetected())
    updateStepCount();
}

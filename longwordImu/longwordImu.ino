
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
BLECharacteristic imuAccCharacteristic("3A19", BLERead | BLENotify, 12 ); // Accelerometer
BLECharacteristic imuAccCharacteristic2("3A20", BLERead | BLENotify, 12 ); // Gyroscope
BLECharacteristic imuAccCharacteristic3("3A21", BLERead | BLENotify, 12 ); // Freefall, shock, steps
BLECharacteristic imuAccCharacteristic4("3A22", BLERead | BLENotify, 12 ); // ZeroMotion
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

int freeFall = 0;
int shock = 0;
int lastStepCount = 0;
int zeroMotion = 0;
unsigned long microsPerReading, microsPrevious;

void setup() {

  /* Initialise the IMU */
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);
  
  CurieIMU.setAccelerometerRange(2); // 2g
  CurieIMU.setGyroRange(250); // 250 degrees/second
  pinMode(BLE_CONNECT, OUTPUT);
  pinMode(INDICATOR_LEDA, OUTPUT);
  /* Enable Free Fall Detection */
  CurieIMU.setDetectionThreshold(CURIE_IMU_FREEFALL, 1000); // 1g=1000mg
  CurieIMU.setDetectionDuration(CURIE_IMU_FREEFALL, 50);  // 50ms
  CurieIMU.interrupts(CURIE_IMU_FREEFALL);
  /* Enable Shock Detection */
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 1500); // 1.5g = 1500 mg
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50);   // 50ms
  CurieIMU.interrupts(CURIE_IMU_SHOCK);
  // turn on step detection mode:
  CurieIMU.interrupts(CURIE_IMU_STEP);  // turn on step detection
  CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_NORMAL);  
  CurieIMU.setStepCountEnabled(true); // enable step counting:  
  /* Enable Zero Motion Detection */
  CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, 50);  // 50mg
  CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, 2);    // 2s
  CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);
  /* Enable Motion Detection */
  CurieIMU.setDetectionThreshold(CURIE_IMU_MOTION, 20);      // 20mg
  CurieIMU.setDetectionDuration(CURIE_IMU_MOTION, 10);       // trigger times of consecutive slope data points
  CurieIMU.interrupts(CURIE_IMU_MOTION);
  
  blePeripheral.setLocalName("imu");
  blePeripheral.setAdvertisedServiceUuid(imuService.uuid());  // add the service UUID
  blePeripheral.addAttribute(imuService);   
  blePeripheral.addAttribute(imuAccCharacteristic);
  blePeripheral.addAttribute(imuAccDescriptor);
  blePeripheral.addAttribute(imuAccCharacteristic2);
  blePeripheral.addAttribute(imuAccCharacteristic3);
  blePeripheral.addAttribute(imuAccCharacteristic4);
  
  const unsigned char initializerAccGyro[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };
  imuAccCharacteristic.setValue( initializerAccGyro, 12);
  imuAccCharacteristic2.setValue( initializerAccGyro, 12);
  imuAccCharacteristic3.setValue( initializerAccGyro, 12);
  imuAccCharacteristic4.setValue( initializerAccGyro, 12);  
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

        // TODO: test reuse same array
        accGyroData.ag[0] = gxRaw;
        accGyroData.ag[1] = gyRaw;
        accGyroData.ag[2] = gzRaw;      
        imuAccCharacteristic2.setValue((unsigned char *)&accGyroData, 12 );
  
        updateStepCount();
        accGyroData.ag[0] = freeFall;
        accGyroData.ag[1] = shock;
        accGyroData.ag[2] = lastStepCount;
        imuAccCharacteristic3.setValue((unsigned char *)&accGyroData, 12 );

        accGyroData.ag[0] = zeroMotion;
        accGyroData.ag[1] = zeroMotion;
        accGyroData.ag[2] = zeroMotion;        
        imuAccCharacteristic4.setValue((unsigned char *)&accGyroData, 12 );
  
        freeFall = 0;
        shock = 0;       
        
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
  if (CurieIMU.getInterruptStatus(CURIE_IMU_FREEFALL))
    freeFall = 1;
  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK))
    shock = 1;
  if (CurieIMU.stepsDetected())
    updateStepCount();
  if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION))
    zeroMotion = 1;
  if (CurieIMU.getInterruptStatus(CURIE_IMU_MOTION))
    zeroMotion = 0;
}

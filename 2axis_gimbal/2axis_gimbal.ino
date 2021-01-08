#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PololuMaestro.h>
#include <EEPROM.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)

//IMU Setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Constants and Variables
float ch;
float desired[4]; //array that radio recieves
//float rollCorrect = -1.3;
//float tiltCorrect = .24;
//int rollCenter = 1465;
int tiltCenter = 1725;
float degPerUs = 10.33;
const int battReadings = 20;
int battValue[battReadings];
int battIndex = 0;
int battTotal = 0;
int battValueAvg = 0;
bool motorDrive = 0;
unsigned long currentTime;
unsigned long lastTime=0;
int loopTime;
float dh, diff, diffX;
float lastDh=0;
int periodM=6000;
float battVoltage;
float tilt;
float roll;
int isCalibrated;
uint8_t system2, gyro, accel, mag;
int resetNum=0;


//Radio Setup
RF24 radio(7, 8); // CE, CSN
//Set this to the serial number written on controler board. Temp Uncomment the one you are using
//const byte addresses[][6] = {"A0001","B0001"}; //Orange
//const byte addresses[][6] = {"A0002","B0002"}; //Green
const byte addresses[][6] = {"A0003","B0003"}; //Black



void initializeSensor(void)
{
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

       // displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
//    displaySensorDetails();

    /* Optional: Display current status */
//    displaySensorStatus();

    
  bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");  //Commented this out on 12/21/20 to help remotely code 
/*       
        while (!bno.isFullyCalibrated())
        {
            isCalibrated=2;
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
            sendIt();
        }
        */
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);
            Serial.print("\tCal: ");
            Serial.println(isCalibrated); 
            Serial.println("Calibration Values:");
            Serial.print("Gyro: ");
            Serial.print(gyro);
            Serial.print("\tAccel: ");
            Serial.print(accel);
            Serial.print("\tMag: ");
            Serial.print(mag);
            Serial.print("\tSys: ");
            Serial.println(system2);

            /* Optional: Display calibration status */
//            displayCalStatus();

            /* New line for the next sample */
//            Serial.println("");

//**************************** Send info back while calibrating******************

isCalibrated=2;

 // uint8_t system2, gyro, accel, mag;
  system2 = gyro = accel = mag = 0;
  bno.getCalibration(&system2, &gyro, &accel, &mag);


sendIt();


            /* Wait the specified delay before requesting new data */
          //  delay(BNO055_SAMPLERATE_DELAY_MS);
            
         
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
//    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    isCalibrated=1;
    delay(500);
  
}

void sendIt(void){

// Convert values to ints for smaller sending, need to stay under 32 bytes
int  battVoltageX100 = battVoltage * 100;
int chX10 = ch * 10;
int tiltX10 = tilt * 10;
int rollX10 = roll * 10;


//Put data into an array for sending back to controller
int payload[] = {battVoltageX100, chX10, tiltX10, rollX10, system2, gyro, accel, mag, isCalibrated, resetNum};

/*
// Print contents of array for debug
for (int i = 0; i < 9; i++) Serial.print(payload[i]);
Serial.println();
*/


//Send info back to controller
delay(5);
radio.stopListening();
radio.write(&payload, sizeof(payload));
  
}


//Servo Controller Setup
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(9, 10);
#endif

MicroMaestro maestro(maestroSerial);



//PID Setup
double Setpoint=0, uhe, Output;
PID myPID(&uhe, &Output, &Setpoint,40,20,.5,P_ON_E, DIRECT);


  
void setup() {

Serial.begin(115200);
Serial.println("Gimbal Startup");

//Maestro Initalize
maestroSerial.begin(9600);
maestro.setAcceleration(1,10);

//PID Initalize
Setpoint = 0;
myPID.SetSampleTime(100);
myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(-1023,1023);



//Radio Initalize
radio.begin();
radio.setChannel(109); //0-124 are avalible
//radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_LOW);
radio.openWritingPipe(addresses[0]);
radio.openReadingPipe(1, addresses[1]);
  
//Battery Readings Initialize to 0
  for (int thisReading = 0; thisReading < battReadings; thisReading++) {
    battValue[thisReading] = 0;
  }
  
//BNO Initalize

initializeSensor();
}

void loop() {


//Loop time
currentTime=millis();
loopTime=currentTime-lastTime;
lastTime=currentTime;


int rst=0;  // Change the reset signal back to 0

//Get Sensor Calibration Status
/* Get the four calibration values (0..3) */
/* Any sensor data reporting 0 should be ignored, */
/* 3 means 'fully calibrated" */
system2 = gyro = accel = mag = 0;
bno.getCalibration(&system2, &gyro, &accel, &mag);


//Get Average Battery Voltage
battTotal = battTotal - battValue[battIndex];
battValue[battIndex] = analogRead(A0);
battTotal = battTotal + battValue[battIndex];
battIndex = battIndex + 1;
if(battIndex >= battReadings){
  battIndex = 0;
}
battValueAvg = battTotal / battReadings;
battVoltage = battValueAvg*(5.3/1023);

  
//Get array of data from Radio
radio.startListening();
if (radio.available()) {
    radio.read(&desired, sizeof(desired));
  }

 
//parse the array that was recieved
  dh=desired[0]; //desired heading
  float dt=desired[1]; //deisred tilt
  rst=desired[2]; //Reset signal
  motorDrive=desired[3]; //drive yes/no


//Calculate dh difference for manual operation
diff=dh-lastDh;



if(diff<=180 && diff>=-180){
  diff=diff;
}
else if(diff>180){
  diff=diff-360;
}
else if(diff<180){
  diff=diff+360;
}

if(diff>10 || diff<10){
  diff=0;
}

diff=diff*100;                                 // use this line to adjust sensitivity of manual movement
diffX=diffX+diff;
lastDh=dh;

//decay differnce over time
if(diffX>0){
  diffX=diffX-loopTime;
    if(diffX<0){
      diffX=0;
    }
}

if(diffX<0){
  diffX=diffX+loopTime;
   if(diffX>0){
    diffX=0;
   }
}

  
// Get new sensor data, euler method
sensors_event_t event;
bno.getEvent(&event);
ch = (event.orientation.x)+90;
{
  if(ch>=360)
  {
  ch=ch-360;
  }
}
tilt = (event.orientation.y);            
roll = (event.orientation.z);


//Quaternion method
/*
imu::Quaternion quat = bno.getQuat();

imu::Vector<3> eul = quat.toEuler();

float ch = eul.x();
float tilt = eul.y();
float roll = eul.z();

ch=ch*-57.2957795;
tilt=tilt*57.2957795;
roll=roll*57.2957795;

if(ch<0){
  ch=360+ch;
}
*/

// Erase calibration data from EEPROM and reset sensor 
if (rst == 1){

  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  
  initializeSensor();

}

//Reset Sensor if tilt or roll get too far off
if (tilt>70 || tilt<-70 || roll>70 || roll<-70){
  maestro.setTarget(0,6000); //Stop Pan Motor
  initializeSensor();  
  resetNum=++resetNum;
  delay(2000);
}

sendIt();




//pan heading correction math
  float rhe = dh-ch; //raw heading error 
{  
  if(rhe<=180 && rhe>=-180)
  {
   uhe=rhe; //unwrapped heading error
  }
  else if(rhe>180)
  {
    uhe=rhe-360;
  }
  else if (rhe<-180)
  {
    uhe=rhe+360;
  }
}

// PID loop
myPID.Compute();

//Output Mapping
int Period=map(Output,-1023,1023,8000,4000);

//Pan Motor Driving
if (motorDrive==1){                              // Auto
  if (uhe<-.4 || uhe>.4)                            // change numbers to adjust deadband
  {
  maestro.setTarget(0,Period);
  myPID.SetOutputLimits(-1023,1023);
  }
  else if (uhe>-.4 && uhe<.4)
  {
  maestro.setTarget(0,6000);
  myPID.SetOutputLimits(-1,1);
  }
}

if (motorDrive==0){                           // Manual
periodM=6000+diffX;
maestro.setTarget(0,periodM);
}



//Tilt Motor Driving
float servoAnglez=0-dt;
float microSecZ=((tiltCenter+(servoAnglez*degPerUs)))*4;
maestro.setTarget(1,microSecZ);


//debug
/*
  Serial.print("Battery Voltage: ");
  Serial.println(battVoltage);
  Serial.println("Sensor Values");
  Serial.print("Pan: ");
  Serial.print(ch);
  Serial.print("\tTilt: ");
  Serial.print(tilt);
  Serial.print("\tRoll: ");
  Serial.println(roll);
  Serial.println("Calibration Values:");
  Serial.print("Gyro: ");
  Serial.print(gyro);
  Serial.print("\tAccel: ");
  Serial.print(accel);
  Serial.print("\tMag: ");
  Serial.print(mag);
  Serial.print("\tSys: ");
  Serial.println(system2);
  Serial.print("Calibration Status: ");
  Serial.println(isCalibrated);
  Serial.print(F("# Of Resets: "));
  Serial.println(resetNum);
  Serial.println();
  */
  Serial.print("Current Heading: ");
  Serial.print(ch);
  Serial.print("\tDesired Heading: ");
  Serial.print(dh);
  Serial.print("\tDiff: ");
  Serial.print(diff);
  Serial.print("\tDiffX: ");
  Serial.println(diffX);



}
  

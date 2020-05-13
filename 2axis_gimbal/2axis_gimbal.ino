#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PololuMaestro.h>


//Servo Controller Setup
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(9, 10);
#endif

MicroMaestro maestro(maestroSerial);

//IMU Setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//PID Setup
double Setpoint=0, uhe, Output;
PID myPID(&uhe, &Output, &Setpoint,60,30,.5,P_ON_E, DIRECT);

//Radio Setup
RF24 radio(7, 8); // CE, CSN
//Set this to the serial number written on controler board. Temp Uncomment the one you are using
//const byte addresses[][6] = {"A0001","B0001"}; //Orange
//const byte addresses[][6] = {"A0002","B0002"}; //Green
const byte addresses[][6] = {"A0003","B0003"}; //Black
  

//Constants and Variables
float ch;
float desired[4]; //array that radio recieves
float rollCorrect = -1.3;
float tiltCorrect = .24;
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


void setup() {



Serial.begin(9600);
Serial.println("Gimbal Startup");

//Maestro Initalize
maestroSerial.begin(9600);
maestro.setAcceleration(1,10);

//PID Initalize
Setpoint = 0;
myPID.SetSampleTime(100);
myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(-1023,1023);

//BNO Initalize
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

//Radio Initalize
radio.begin();
radio.setChannel(109);
//radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_LOW);
radio.openWritingPipe(addresses[0]);
radio.openReadingPipe(1, addresses[1]);
  
//Battery Readings Initialize to 0
  for (int thisReading = 0; thisReading < battReadings; thisReading++) {
    battValue[thisReading] = 0;
  }
  
}

void loop() {
/*
// Read input from serial monitor, 1 turns recording on, 0 turns it off
if (Serial.available()){
incomming = Serial.read();
if (incomming.equals("1")){
  motorDrive = 1;
}
if (incomming.equals("0")){
  motorDrive = 0;
}
delay(500);
}
// end of serial input
*/

//Loop time
currentTime=millis();
loopTime=currentTime-lastTime;
lastTime=currentTime;



/*
if(currentTime<10000){
  motorDrive=0;
}
else{
  motorDrive=1;
}
 */ 
int rst=0;
//Get Sensor Calibration Status
/* Get the four calibration values (0..3) */
/* Any sensor data reporting 0 should be ignored, */
/* 3 means 'fully calibrated" */
uint8_t system, gyro, accel, mag;
system = gyro = accel = mag = 0;
bno.getCalibration(&system, &gyro, &accel, &mag);


//Get Average Battery Voltage
battTotal = battTotal - battValue[battIndex];
battValue[battIndex] = analogRead(A0);
battTotal = battTotal + battValue[battIndex];
battIndex = battIndex + 1;
if(battIndex >= battReadings){
  battIndex = 0;
}
battValueAvg = battTotal / battReadings;
float battVoltage = battValueAvg*(5.3/1023);

  
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

diff=diff*100;                                 // use this line to adjust sensitivity of movement
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

  
// Get new sensor data, old method
sensors_event_t event;
bno.getEvent(&event);
float ch = (event.orientation.x)+90;
{
  if(ch>=360)
  {
  ch=ch-360;
  }
}
float tilt = (event.orientation.y)+tiltCorrect;
float roll = (event.orientation.z)+rollCorrect;


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

// Sensor invert short circut

if (rst == 1){
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  
}


//Put data into an array for sending back to controller
float payload[] = {battVoltage, ch, tilt, roll, system, gyro, accel, mag};

//Send info back to controller
delay(5);
radio.stopListening();
radio.write(&payload, sizeof(payload));


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
  if (uhe<-.4 || uhe>.4)
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
//  displayCalStatus(); 
  //Serial.print("\tBatt ");
  //Serial.print(battVoltage);
  Serial.print("\tPan: ");
  Serial.print(ch);
  Serial.print(" / ");
  Serial.print(dh);
  Serial.print(" / ");
  Serial.print(Period);
  Serial.print("\tDiff: ");
  Serial.print(diffX);
  Serial.print(" / ");
  Serial.println(loopTime);
 // Serial.print("\troll ");
  //Serial.println(roll);
  //Serial.print(" / ");
  //Serial.print(microSecZ);
  //Serial.print(" / ");
  //Serial.println(battVoltage);


}
  

#include <Encoder.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> 
#include <EasyButton.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

//OLED SETUP
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//Radio Setup
RF24 radio(7, 8); // CE, CSN
//Set this to the serial number written on controler board. Temp Uncomment the one you are using
//const byte addresses[][6] = {"A0001","B0001"}; //Orange
//const byte addresses[][6] = {"A0002","B0002"}; //Green
//const byte addresses[][6] = {"A0003","B0003"}; //Black


//Encoder Setup
Encoder panKnob(2,4);
Encoder tiltKnob(3,5);

//Button Setup
#define BUTTON_PINA 6
#define BUTTON_PINB 9
EasyButton panbutton(BUTTON_PINA);
EasyButton tiltbutton(BUTTON_PINB);


//Constants and Variables
int payload[13];
long previousMillis = 0;
long interval = 100;
int data=0;
int rst=0;
int sendCount=0;
bool drive=0;
long oldPanAngle  = -999;
long oldTiltAngle = -999;
float desiredHeading = 0;
float desiredTilt = 0;
const float panSlow = .5;   // changing these in ints would save space
const float panMed = 1;
const float panFast = 2.5;
const float tiltSlow = .5;
const float tiltMed = 1;
const float tiltFast = 2.5;
float panInc=panMed;
float tiltInc=tiltMed;
float ch;
int isCalibrated;
int battVoltageX100;
int chX10;
int rollX10;
int tiltX10;
int gyro;
int accel;
int mag;
int system2;
//int radioChannel = 109;
int resetNum;
int lostConnect=0;
float battVoltageFlt;
float roll;
float tilt;
unsigned long reConnectTime=0;
char serialNum;
byte addrA[5];
byte addrB[5];
int channelVal;
int channel;
int packetCount=1;
int packetLossX100;
float packetLoss;
int autoEnable;
int lastAutoEnable;
bool returnToAuto=0;
int boardTemp;

// Callback function to be called when the button is pressed.
void tiltLong() {
  if(data == 0){
    data=1;
  }
  else if(data == 1){
    data=2;
  }
  else if(data == 2){
    data=0;
  }
}

void tiltShort(){
  if(tiltInc == tiltSlow){
    tiltInc=tiltMed;
  }
  else if(tiltInc == tiltMed){
    tiltInc=tiltFast;
  }
  else if(tiltInc == tiltFast){
    tiltInc=tiltSlow;
  }
}

void panShort(){
  if(panInc == panSlow){
    panInc=panMed;
  }
  else if(panInc == panMed){
    panInc=panFast;
  }
  else if(panInc == panFast){
    panInc=panSlow;
  }
}

void panLong(){
  if(data == 0){
    if(drive == 0){
      desiredHeading= ch; 
      drive=1;
  }
    else if(drive == 1){
      drive=0;
  }
  }
  else if(data == 1){
    rst=1;
  }
  else if(data == 2){
    rst=2;
  }

}

void recieve(void){

radio.startListening();
if (radio.available()){
radio.read(&payload, sizeof(payload));
}
//parse the array that was recieved
battVoltageX100 = payload[0];
chX10 = payload[1];
rollX10 = payload[2];
tiltX10 = payload[3];
system2 = payload[4];
gyro = payload[5];
accel = payload[6];
mag = payload[7];
isCalibrated = payload[8];
resetNum = payload[9];
packetLossX100 = payload[10];
autoEnable = payload[11]; 
boardTemp = payload[12];

                                   
memset(payload, 0, sizeof(payload));            // Delete contents of payload array to ensure fresh data


battVoltageFlt = (float) battVoltageX100/100; // need to fix this,look at meter***********************************
ch = (float) chX10/10;
roll = (float) rollX10/10;
tilt = (float) tiltX10/10;
packetLoss = (float) packetLossX100/100;
 
}

//Splash Screen Logo
const unsigned char logo1 [] PROGMEM = {
  0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00, 0x07, 0xff, 0xff, 
  0xe0, 0x00, 0x00, 0x1f, 0xe0, 0x3f, 0xf8, 0x00, 0x00, 0x3f, 0x00, 0xe0, 0xfc, 0x00, 0x00, 0xfc, 
  0x03, 0xc0, 0x3f, 0x00, 0x01, 0xf0, 0x0f, 0x00, 0x0f, 0x80, 0x03, 0xe0, 0x1c, 0x00, 0x07, 0xc0, 
  0x07, 0x80, 0x38, 0x00, 0x01, 0xe0, 0x07, 0x80, 0x60, 0x00, 0x00, 0xe0, 0x0f, 0x80, 0xff, 0xff, 
  0x80, 0xf0, 0x1f, 0x81, 0xff, 0xff, 0xf0, 0x78, 0x1d, 0x81, 0xff, 0xf0, 0xfc, 0x38, 0x39, 0x83, 
  0xff, 0xe0, 0x6f, 0x1c, 0x39, 0x87, 0xff, 0xc0, 0x63, 0x9c, 0x71, 0x8f, 0xff, 0x80, 0x70, 0xfe, 
  0x71, 0x8f, 0xff, 0x80, 0x38, 0x7e, 0x71, 0x9f, 0xff, 0x80, 0x18, 0x0e, 0xe1, 0x9f, 0xff, 0xc0, 
  0x0c, 0x0f, 0xe1, 0xbf, 0xff, 0xe0, 0x0e, 0x07, 0xe1, 0xb7, 0xff, 0xf8, 0x06, 0x07, 0xe0, 0xe3, 
  0xff, 0xfe, 0x06, 0x07, 0xe0, 0xe1, 0xff, 0xff, 0x86, 0x07, 0xe0, 0xc0, 0x7f, 0xff, 0x83, 0x07, 
  0xe0, 0xc0, 0x0f, 0xff, 0xc3, 0x07, 0xe0, 0x40, 0x00, 0x3f, 0xe3, 0x07, 0xe0, 0x60, 0x00, 0x07, 
  0xe7, 0x07, 0xe0, 0x30, 0x00, 0x03, 0xe5, 0x87, 0xe0, 0x30, 0x00, 0x03, 0xe5, 0x87, 0xf0, 0x30, 
  0x00, 0x03, 0xcd, 0x87, 0x70, 0x18, 0x00, 0x03, 0xcd, 0x8e, 0x7e, 0x0c, 0x00, 0x07, 0x99, 0x8e, 
  0x7f, 0x0c, 0x00, 0x0f, 0x38, 0x8e, 0x39, 0x86, 0x00, 0x0e, 0x30, 0x9c, 0x39, 0xf3, 0x00, 0x0c, 
  0x61, 0x9c, 0x1c, 0x3f, 0x00, 0x0c, 0xe1, 0xb8, 0x1e, 0x1f, 0xc0, 0x0f, 0xc1, 0xf8, 0x0f, 0x01, 
  0xff, 0xff, 0x01, 0xf0, 0x07, 0x00, 0x1f, 0xff, 0x03, 0xe0, 0x07, 0x80, 0x00, 0x0e, 0x03, 0xe0, 
  0x03, 0xe0, 0x00, 0x3c, 0x07, 0xc0, 0x01, 0xf0, 0x00, 0x70, 0x0f, 0x80, 0x00, 0xfc, 0x01, 0xe0, 
  0x3f, 0x00, 0x00, 0x3f, 0x07, 0x80, 0xfc, 0x00, 0x00, 0x1f, 0xff, 0x03, 0xf8, 0x00, 0x00, 0x07, 
  0xff, 0xff, 0xe0, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00
};


void setup() {

//OLED Initalize
display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
display.clearDisplay();
display.drawBitmap(40,16,logo1,48,48, WHITE); //display logo for 5 sec
display.display();
delay(1500);
display.clearDisplay();
display.setTextColor(WHITE);
display.setTextSize(2);
display.setCursor(0,16);
display.print(F("Revolute"));
display.setCursor(0,32);
display.print(F("One"));
display.setTextSize(1);
display.setCursor(90,56);
display.print(F("V 0.1"));
display.display();
delay(1500);



//read jumper pin divider to select radio channel 

channelVal=analogRead(A2);


if(channelVal>=896){
  channel=109;
}
else if(channelVal<896 && channelVal>=640){
  channel=111;
}
else if(channelVal<640 && channelVal>=384){
  channel=113;
}
else if(channelVal<384 && channelVal>=128){
  channel=115;
}
else if(channelVal<128){
  channel=117;
}

//Radio Initalize
EEPROM.get(60, addrA);
EEPROM.get(70, addrB);
radio.begin();
radio.setChannel(channel);
//radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_LOW);
radio.openWritingPipe(addrB);
radio.openReadingPipe(1,addrA);
//int PA=radio.getPALevel();




Serial.begin(115200);

// Initialize the buttons.
panbutton.begin();
tiltbutton.begin();
  
// Add the callback function to be called when the button is pressed for at least the given time.
tiltbutton.onPressed(tiltShort);
tiltbutton.onPressedFor(1000, tiltLong);
panbutton.onPressed(panShort);
panbutton.onPressedFor(1000, panLong);
}

void loop() {

//rst=0;


if(rst==0){
  rst=0;
}

else if(rst!=0 && sendCount < 20){
  sendCount= ++sendCount;
}

else if(rst!=0 && sendCount >= 20){
  rst=0;
  sendCount=0;
}

// Continuously read the status of the button. 
panbutton.read();
tiltbutton.read();
  
// read tilt knob and adjust angle
long newTiltAngle = tiltKnob.read();
 while (newTiltAngle != oldTiltAngle)
 {
  if (newTiltAngle > oldTiltAngle){
    desiredTilt +=tiltInc;
  }
  else{
    desiredTilt -=tiltInc;
  }
  if (desiredTilt > 70){
    desiredTilt = 70;
  }
  if (desiredTilt < 0){
    desiredTilt = 0;
  }
  oldTiltAngle = newTiltAngle;
 }
 
// read pan knob and adjust heading 
long newPanAngle = panKnob.read();
  while (newPanAngle != oldPanAngle)
  { 
    if (newPanAngle > oldPanAngle){
      desiredHeading +=panInc;
    }
    else{
      desiredHeading -=panInc;
    }
    if (desiredHeading > 359.75){
      desiredHeading = 0;
    }
    if (desiredHeading < 0){
      desiredHeading = 359.75;
    }
    oldPanAngle = newPanAngle;
  }

//place desired angles in an array  


float desired[] = {desiredHeading, desiredTilt, rst, drive, packetCount};

// Send that array To Gimbal
radio.stopListening();
radio.write(&desired, sizeof(desired));
packetCount=++packetCount;








recieve();


//set back to manual if connection to gimbal is lost
if(isCalibrated==0 && drive == 1){
  drive=0;
  lostConnect=1;
}

//restore to auto after waiting 
if(isCalibrated != 0 && lostConnect == 1){
  reConnectTime = millis();
  lostConnect = 2;
}

unsigned long currentMillis = millis();

if(lostConnect == 2){
  if(currentMillis - reConnectTime >3000){
      desiredHeading = ch;
      drive = 1;
      lostConnect = 0;
  }
}




//converting battery voltage to meter level

//battVoltageFlt;//dont think this is needed please test 2/20
int battVoltageInt = battVoltageFlt*100;
int battLevel = map(battVoltageInt, 300, 410, 0, 100);
battLevel = constrain(battLevel, 0, 100);
int meterLevel = battLevel/4;


//set desired heading to current heading whenever auto enable comes back

if (drive==1 && autoEnable==0){
  drive=0;
  returnToAuto=1;
}

if (drive==0 && returnToAuto==1 && autoEnable==1){
  desiredHeading = ch;
  drive=1;
  returnToAuto=0;
}
/*
if (autoEnable != lastAutoEnable){
  if (autoEnable == 1){
    desiredHeading = ch;
    drive=1;
  } else{
    drive=0;
  }
}

lastAutoEnable = autoEnable;
*/

//display section

  display.clearDisplay();
  display.setTextColor(WHITE);

//Screen 1, displayed on start up. Desired angles, batt meter, and Adjustment sensitivty
if (data == 0 && isCalibrated == 1){

  
// display of desired angles
  display.setTextSize(2);
  display.setCursor(0,24);
  display.print(F("TILT"));
  display.setCursor(68,24);
  display.print(F("PAN"));
  display.setCursor(0,47); 
  display.print(desiredTilt,1);
  display.setCursor(68,47);
  display.print(desiredHeading,1);
  display.fillRect(55, 16, 3, 52, WHITE); //Center Divider
  display.fillRect(0, 16, 128, 3, WHITE); //Upper Divider

// Motor Drive On or Off

if (autoEnable==1){
  display.setTextSize(1);
  display.setCursor(110,24);
  if(drive==0){
    display.print(F("MAN"));
  }
  else if(drive==1){
    display.print(F("AUT"));
  }
}

if (autoEnable==0){
  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE);
  display.setCursor(110,24);
  if(drive==0){
    display.print(F("MAN"));
  }
  else if(drive==1){
    display.print(F("AUT"));
  }
  display.setTextColor(WHITE);
}

  

// Tilt adjustment sensitivity
  if(tiltInc == tiltSlow){
    display.fillRect(12,11,4,6,WHITE);
  }
  else if(tiltInc == tiltMed){
    display.fillRect(6,5,4,11,WHITE);
    display.fillRect(12,11,4,6,WHITE);  
  }
  else if(tiltInc == tiltFast){
    display.fillRect(0,0,4,16,WHITE);
    display.fillRect(6,5,4,11,WHITE);
    display.fillRect(12,11,4,6,WHITE);
  }

// Pan adjustment sensitivity
  if(panInc == panSlow){
    display.fillRect(112,11,4,6,WHITE);
  }
  else if(panInc == panMed){
    display.fillRect(118,5,4,11,WHITE);
    display.fillRect(112,11,4,6,WHITE);
  }
  else if(panInc == panFast){
    display.fillRect(124,0,4,16,WHITE);
    display.fillRect(118,5,4,11,WHITE);
    display.fillRect(112,11,4,6,WHITE);
  }

// Battery Meter
  display.drawRect(50, 0, 31, 16, WHITE); //Outer battery box
  display.drawRect(51, 1, 29, 14, WHITE); //Inner battery box
  display.drawRect(81, 5, 2, 6, WHITE); //Battery tip
  display.fillRect(53, 3, meterLevel, 10, WHITE); //Battery meter fill

}

else if (data == 1 && isCalibrated == 1){
  
// display IMU senor readings
  display.setTextSize(1);
  display.setCursor(0,16);
  display.print(F("PAN :"));
  display.setCursor(32,16);
  display.print(ch,1);

  display.setCursor(83,16);
  display.print(F("V : "));
  display.setCursor(103,16);
  display.print(battVoltageFlt*2);
  
  display.setCursor(0,26);
  display.print(F("ROLL:"));
  display.setCursor(32,26);
  display.print(roll,1);
  display.setCursor(0,36);
  display.print(F("TILT:"));
  display.setCursor(32,36);
  display.print(tilt,1);
  display.setCursor(0,46);
  display.print(F("TEMP: "));
  display.setCursor(32,46);
  display.print(boardTemp);
  display.setCursor(0,56);
  display.print(F("PACKET LOSS%:"));
  display.setCursor(80,56);
  display.print(packetLoss);
  //display.setCursor(0,56);
  //display.print(F("# OF RESETS: "));
  //display.setCursor(72,56);
  //display.print(resetNum);

//Battery percentage position, moves the starting number based on 1, 2 or 3 digits
  int battPos;
  {
  if (battLevel == 100)
  {
    battPos=104;
  }
  else if (battLevel<100 && battLevel>=10)
  {
    battPos=109;
  }
  else if (battLevel<10)
  {
    battPos=114;
  }
  }

// Battery Percent display
  display.setTextSize(1);
  display.setCursor(104,0);
  display.print(F("BATT"));
  display.setCursor(battPos,8);
  display.print(battLevel);
  display.setCursor(122,8);
  display.print(F("%"));

// display of sensor calibration data
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("G"));
  display.setCursor(11,0);
  display.print(F("A"));
  display.setCursor(23,0);
  display.print(F("M"));
  display.setCursor(35,0);
  display.print(F("S"));
  display.setCursor(0,8);
  display.print(gyro);
  display.setCursor(11,8);
  display.print(accel);
  display.setCursor(23,8);
  display.print(mag);
  display.setCursor(35,8);
  display.print(system2);
  display.drawLine(8, 0, 8, 15, WHITE);
  display.drawLine(19, 0, 19, 15, WHITE);
  display.drawLine(31, 0, 31, 15, WHITE);
  
}

else if (data == 2 && isCalibrated == 1){
  display.setTextSize(1);
  display.setCursor(0,16);
  display.print(F("To Reset Calibration"));
  display.setCursor(0,32);
  display.print(F("Long Press Pan Knob"));  
  
}


else if (isCalibrated == 2){
  
// display of sensor calibration data
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("Please Calibrate"));
  display.setCursor(0,8);
  display.print(F("Sensor"));
  display.setTextSize(2);
  display.setCursor(0,18);
  display.print(F("G: "));
  display.setCursor(24,18);
  display.print(gyro);
  display.setCursor(68,18);
  display.print(F("A: "));
  display.setCursor(92,18);
  display.print(accel);
  display.setCursor(0,42);
  display.print(F("M: "));
  display.setCursor(24,42);
  display.print(mag);
  display.setCursor(68,42);
  display.print(F("S: "));
  display.setCursor(92,42);
  display.print(system2);

  
}
else if (isCalibrated == 0){
  


// display of sensor calibration data
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("No Gimbal Detected"));
  display.setCursor(0,16);
  display.print(F("Radio Channel: "));
  display.setCursor(92,16);
  display.print(channel);


  
}



 display.display();

 
  Serial.print(F("Battery Voltage: "));
  Serial.println(battVoltageFlt);
  Serial.println(F("Sensor Values"));
  Serial.print(F("Pan: "));
  Serial.print(ch);
  Serial.print(F("\tTilt: "));
  Serial.print(tilt);
  Serial.print(F("\tRoll: "));
  Serial.println(roll);
  Serial.println(F("Calibration Values:"));
  Serial.print(F("Gyro: "));
  Serial.print(gyro);
  Serial.print(F("\tAccel: "));
  Serial.print(accel);
  Serial.print(F("\tMag: "));
  Serial.print(mag);
  Serial.print(F("\tSys: "));
  Serial.println(system2);
  Serial.print(F("Calibration Status: "));
  Serial.println(isCalibrated);
  Serial.print(F("# Of Resets: "));
  Serial.println(resetNum);
  Serial.print(F("Serial Number: "));
  String serialNum = (char*)addrA;  //convert byte array to printable string
  Serial.println(serialNum);
  Serial.print(F("Radio Channel: "));
  Serial.println(channel); 
  //Serial.println(autoEnable); 
  Serial.println();





/*
  Serial.print(F("Drive: "));
  Serial.print(drive);
  Serial.print(F("\tlostConnect: "));
  Serial.print(lostConnect);
  Serial.print(F("\tisCalibrated: "));
  Serial.println(isCalibrated);
*/
 
}

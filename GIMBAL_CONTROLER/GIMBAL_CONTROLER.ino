#include <Encoder.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> 
#include <EasyButton.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


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
const byte addresses[][6] = {"A0003","B0003"}; //Black

//ENcoder Setup
Encoder panKnob(2,4);
Encoder tiltKnob(3,5);





//Button Setup
#define BUTTON_PINA 6
#define BUTTON_PINB 9
EasyButton panbutton(BUTTON_PINA);
EasyButton tiltbutton(BUTTON_PINB);


//Constants and Variables
float payload[8];
long previousMillis = 0;
long interval = 500;
int data=0;
int rst=0;
bool drive=0;
long oldPanAngle  = -999;
long oldTiltAngle = -999;
float desiredHeading = 0;
float desiredTilt = 0;
const float panSlow = .5;
const float panMed = 1;
const float panFast = 2.5;
const float tiltSlow = .5;
const float tiltMed = 1;
const float tiltFast = 2.5;
float panInc=panMed;
float tiltInc=tiltMed;


// Callback function to be called when the button is pressed.
void tiltLong() {
  if(data == 0){
    data=1;
  }
  else if(data == 1){
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
      drive=1;
  }
    else if(drive == 1){
      drive=0;
  }
  }
  else if(data == 1){
    rst=1;
  }

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

//Radio Initalize
radio.begin();
radio.setChannel(109);
//radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_LOW);
radio.openWritingPipe(addresses[1]);
radio.openReadingPipe(1,addresses[0]);
//int PA=radio.getPALevel();

Serial.begin(9600);

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

rst=0;

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
float desired[] = {desiredHeading, desiredTilt, rst, drive};

// Send that array To Gimbal
radio.stopListening();
radio.write(&desired, sizeof(desired));


//check millis to see if its time to recieve data
unsigned long currentMillis = millis();

if(currentMillis - previousMillis > interval){
previousMillis = currentMillis;
}
delay(5);
radio.startListening();
if (radio.available()){
radio.read(&payload, sizeof(payload));
}

//parse the array that was recieved
float battVoltageFlt = payload[0];
float ch = payload[1];
float roll = payload[2];
float tilt = payload[3];
int system = payload[4];
int gyro = payload[5];
int accel = payload[6];
int mag = payload[7];


//converting battery voltage to meter level

//battVoltageFlt;//dont think this is needed please test 2/20
int battVoltageInt = battVoltageFlt*100;
int battLevel = map(battVoltageInt, 300, 415, 0, 100);
battLevel = constrain(battLevel, 0, 100);
int meterLevel = battLevel/4;

//display section

  display.clearDisplay();
  display.setTextColor(WHITE);

//Screen 1, displayed on start up. Desired angles, batt meter, and Adjustment sensitivty
if (data == 0){
// display of desired angles
  display.setTextSize(2);
  display.setCursor(0,24);
  display.print("TILT");
  display.setCursor(68,24);
  display.print("PAN");
  display.setCursor(0,47); 
  display.print(desiredTilt,1);
  display.setCursor(68,47);
  display.print(desiredHeading,1);
  display.fillRect(55, 16, 3, 52, WHITE); //Center Divider
  display.fillRect(0, 16, 128, 3, WHITE); //Upper Divider

// Motor Drive On or Off

  display.setTextSize(1);
  display.setCursor(110,24);
  if(drive==0){
    display.print("OFF");
  }
  else if(drive==1){
    display.print("ON");
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

else if (data == 1){
  
// display IMU senor readings
  display.setTextSize(2);
  display.setCursor(5,16);
  display.print("PAN :");
  display.setCursor(65,16);
  display.print(ch,1);
  display.setCursor(5,32);
  display.print("ROLL:");
  display.setCursor(65,32);
  display.print(roll,1);
  display.setCursor(5,48);
  display.print("TILT:");
  display.setCursor(65,48);
  display.print(tilt,1);

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
  display.print("BATT");
  display.setCursor(battPos,8);
  display.print(battLevel);
  display.setCursor(122,8);
  display.print("%");

// display of sensor calibration data
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("G");
  display.setCursor(11,0);
  display.print("A");
  display.setCursor(23,0);
  display.print("M");
  display.setCursor(35,0);
  display.print("S");
  display.setCursor(0,8);
  display.print(gyro);
  display.setCursor(11,8);
  display.print(accel);
  display.setCursor(23,8);
  display.print(mag);
  display.setCursor(35,8);
  display.print(system);
  display.drawLine(8, 0, 8, 16, WHITE);
  display.drawLine(19, 0, 19, 16, WHITE);
  display.drawLine(31, 0, 31, 16, WHITE);
  
}

  










 display.display();

 
 
}

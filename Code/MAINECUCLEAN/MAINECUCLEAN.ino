// GAS + CRUISE - DONT USE ON PUBLIC ROAD
// Uses an MCP2515 Can Bus interface, an SSD1306 128x64 OLED Display, 4x 5V Relay, Voltage divider circuits to sense 12V logic from the car's Brake, Dashboard Light, Blinkers and VSS
//Runs on Arduino Nano / Uno
// Buttons 1 - 4 are hooked up to a single analog pin with a voltage divider circuit to sense which button is pressed


#include <CAN.h>

#include <TimerOne.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"


// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire display;
char topText[4] = "TEST";
char bottomText[6] = "BOTTOM";


//________________These values need to be defined for each car (Don't know)
#define maxACC_CMD (float) 1500 //the max Value which comes from OP on CAN ID 0x200
#define minACC_CMD (float) 475 //the min Value which comes from OP on CAN ID 0x200


//________________define_pins

#define brakePosition_pin (int) A0 //Pot on Brake pedal
#define throttlePosition_pin (int) A1 //Pot on Throttle pedal
#define blinkerL_pin 16 // A2 pulled to 5V when blinker to the left
#define blinkerR_pin 17 // A3 pulled to 5V when blinker to the right
#define button1to4_pin (int) A7 //5V 0Ohm ON_OFF, 3.75V 5kOhm V+ , 2.5V 10kOhm V-, 1.25V OFF 20kOhm, 0V Nothing VIA VOLTAGE DIVIDER
int buttonState;
int buttonVal;
int lastButtonState;
#define brakelight_pin (int) A6 //pulled to 5V when brake is pressed down
#define dashlight_pin 0 //pulled to 5V when CC engaged

#define vssInput_pin 3 //Hooked to the car's Vehicle Speed Sensor (usually from Radio harness)

#define minusRelay_pin 6 //pull to 5V to override VSS
#define offRelay_pin 7 //pull to 5V to set disable cruise control
#define plusRelay_pin 8 //pull to 5V to set acceleration/increase speed

//MCP2515CAN    Int 2, 10, 11, 12, 13
//Screen       A4(SDA), A5(SCL),(18, 19)

//________________Values for ACC command from CAN
float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
boolean cancel = false;
long lastCommandTime = 0;

//______________VALUES TO SEND ON CAN
boolean OP_ON = false;
boolean MAIN_ON = true;
boolean arduinoCCActive = false;
boolean DASHLIGHT_ON = false;
uint8_t set_speed = 0x0;
double average = 0;
boolean BLINKER_LEFT = false;
boolean BLINKER_RIGHT = false;
float LEAD_LONG_DIST = 0;
float LEAD_REL_SPEED = 0;
float LEAD_LONG_DIST_RAW = 0;
float LEAD_REL_SPEED_RAW = 0;

int BRAKE_POSITION = 0;
boolean BRAKE_PRESSED = false;
boolean lastBRAKE_PRESSED = false;


int GAS_POSITION = 0;
boolean GAS_RELEASED = false;
boolean lastGAS_RELEASED = false;


//______________FOR SMOOTHING SPD
#define numReadings 60
float readings[numReadings];
int readIndex = 0;
double total = 0;

//______________FOR READING THE CAR'S ACTUAL VSS SENSOR
#define interruptPin (byte) 3
int inc = 0;
int half_revolutions = 0;
int spd;
unsigned long lastmillis;
unsigned long duration;
uint8_t encoder = 0;
int testAnalog = 0;

void initOled() {
  Wire.begin();
  Wire.setClock(400000L);
  display.begin(&Adafruit128x64, I2C_ADDRESS);
  display.setFont(Adafruit5x7);
  display.set2X();
  display.clear();

}

void setup() {
  //________________Begin Monitoring - only use for debugging
  Serial.begin(115200);
  initOled();
  Serial.println("LOL");
  //________________Begin CAN
  CAN.begin(500E3);
  // CAN.filter(0x200);
  CAN.onReceive(onReceivePacket);


  Serial.println("lola");
  //________________set up pin modes
  pinMode(blinkerR_pin, INPUT);
  pinMode(blinkerL_pin, INPUT);
  pinMode(brakelight_pin, INPUT);
  pinMode(brakePosition_pin, INPUT);
  pinMode(throttlePosition_pin, INPUT);
  pinMode(dashlight_pin, INPUT);
  pinMode(vssInput_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(vssInput_pin), rpm, FALLING);

  pinMode(button1to4_pin, INPUT);

  pinMode(plusRelay_pin, OUTPUT);
  pinMode(offRelay_pin, OUTPUT);
  pinMode(minusRelay_pin, OUTPUT);
  digitalWrite(plusRelay_pin, HIGH);
  digitalWrite(offRelay_pin, HIGH);
  digitalWrite(minusRelay_pin, HIGH);

  //______________Initialize smoothing inputs
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  Serial.println("bite");
}

void loop() {
  int mil = millis();
  readSPDSensor();
  readAllInputs();
  actOnSwitchStates();
  displayStates();
  canBroadcasts();
  if (mil - millis() < 200) {
    delay(100);
  }

  Serial.println("meme");
}
void writeTopString(char str[], bool topOverline = false) {
  display.set2X();
  display.setCursor(0, 0);
  display.setInvertMode(0);
  if (topOverline) display.setInvertMode(1);
  if (str) {
    strcpy(topText, str);
  }
  display.println(topText);
};

void writeBottomString(char str[], int fontSize) {
  switch (fontSize) {
    case 2:
      display.set2X();
      break;
    default:
      display.set1X();
      break;
  }
  display.setInvertMode(0);
  display.setCursor(0, 8);
  if (str) {
    strcpy(bottomText, str);
  }
  display.println(bottomText);
  ;
};

void writeOnDisplay(char top[], char bottom[], int fontSizeBottom = 2, bool topOverline = false) {
  display.clear();
  writeTopString(top, topOverline);
  writeBottomString(bottom, fontSizeBottom);
  //display.display();
};

void displayStates() {
  // display.clear();
  if (OP_ON) {
    writeTopString(" OP ONLINE  ", true);
  } else {
    writeTopString("  OP  OFF   ", false);
  }

  display.set1X();
  display.setCursor(0, 10);
  display.setInvertMode(0);
  display.print("VSS: ");
  display.print(int(average));
  display.print(" km/h    \n");
  // if (BLINKER_LEFT) display.setInvertMode(1);
  display.print(BLINKER_LEFT ? "L" : "_");
  display.setInvertMode(0);
  display.print(" ");
  // if (BLINKER_RIGHT) display.setInvertMode(1);
  display.print(BLINKER_RIGHT ? "R" : "_");
  display.setInvertMode(0);
  display.print(" ");
  // if (BRAKE_PRESSED) display.setInvertMode(1);
  display.print(BRAKE_PRESSED ? "BRK" : "___");
  display.setInvertMode(0);
  display.print(" ");
  // if (DASHLIGHT_ON) display.setInvertMode(1);
  display.print(DASHLIGHT_ON ? "CC" : "__");
  display.setInvertMode(0);
  display.print(" ");
  display.print(arduinoCCActive ? "ACC" : "___");
  display.setInvertMode(0);
  display.print(" ");
  if (buttonState > 0) display.setInvertMode(1);
  display.print(buttonState);
  display.setInvertMode(0);

  display.print("\nButtonVal: ");
  display.print(buttonVal);
  display.print("  \nBrakePosition: ");
  display.print(BRAKE_POSITION);
  display.print("  \nThrottlePosition: ");
  display.print(GAS_POSITION);
  display.setInvertMode(0);
}

void onReceivePacket(int packetSize) {
  if (!OP_ON) return;
  //________________read ACC_CMD from CANbus
  CAN.parsePacket();

  if (CAN.packetId() == 0x200)
  {
    uint8_t dat[8];
    for (int ii = 0; ii <= 7; ii++) {
      dat[ii]  = (char) CAN.read();
    }
    ACC_CMD = (dat[0] << 8 | dat[1] << 0);
  }

  //________________calculating ACC_CMD into ACC_CMD_PERCENT
  if (ACC_CMD >= minACC_CMD) {
    ACC_CMD1 = ACC_CMD;
  }
  else {
    ACC_CMD1 = minACC_CMD;
  }

  ACC_CMD_PERCENT = (100 * (ACC_CMD1 - minACC_CMD) / (maxACC_CMD - minACC_CMD));

  if (arduinoCCActive) {
    if (ACC_CMD_PERCENT == 0 ) {
      shutdownCC();
    }
    else if (ACC_CMD_PERCENT < 40 ) {
      decreaseSpeedHold();
    } else if (ACC_CMD_PERCENT > 60 ) {
      increaseSpeedHold();
    } else {
      speedHold();
    }
  }

  //128x2e6 msg LEAD_INFO
  if (CAN.packetId() == 0x2e6)
  {
    uint8_t dat_2e6[8];
    for (int ii = 0; ii <= 7; ii++) {
      dat_2e6[ii]  = (char) CAN.read();
    }
    LEAD_LONG_DIST_RAW = (dat_2e6[0] << 8 | dat_2e6[1] << 3);
    LEAD_REL_SPEED_RAW = (dat_2e6[2] << 8 | dat_2e6[3] << 4);
  }
  //______________CONVERTING INTO RIGHT VALUE USING DBC SCALE
  LEAD_LONG_DIST = (LEAD_LONG_DIST_RAW * 0.005);
  LEAD_REL_SPEED = (LEAD_REL_SPEED_RAW * 0.009);

  /*
    //0x224 msg BRAKE_MODULE --- WE are using the 0x3b7 message, which is ESP_CONTROL to reduce traffic on the can network
    if (CAN.packetId() == 0x3b7)
    {
      uint8_t dat_3b7[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_3b7[ii]  = (char) CAN.read();
      }
      BRAKE_PRESSED = (dat_3b7[0] << 5);
    }
  */

  //0x2c1 msg GAS_PEDAL
  if (CAN.packetId() == 0x2c1)
  {
    uint8_t dat_2c1[8];
    for (int ii = 0; ii <= 7; ii++) {
      dat_2c1[ii]  = (char) CAN.read();
    }
    GAS_RELEASED = (dat_2c1[0] << 3);
  }

  /*
    //______________LOGIC FOR LANE CHANGE RECOMENDATION TODO
    if (set_speed >= ((average * 100) + 15))
    {
      if (LEAD_REL_SPEED  <= 15)
      {
        if (LEAD_LONG_DIST <= 100)
        {
          BLINKER_LEFT = false;
        }
      }
    }
  */
}


void decreaseSpeedHold() {
  lastCommandTime = millis();
  digitalWrite(plusRelay_pin, HIGH);
  digitalWrite(minusRelay_pin, LOW);
}

void increaseSpeedHold() {
  lastCommandTime = millis();
  digitalWrite(plusRelay_pin, HIGH);
  digitalWrite(minusRelay_pin, LOW);
}

void speedHold() {
  if ( millis() > lastCommandTime + 2000 ) {
    digitalWrite(plusRelay_pin, HIGH);
    digitalWrite(minusRelay_pin, HIGH);
  }
}

void actOnSwitchStates() {
  //______________SET OP OFF WHEN BRAKE IS PRESSED

  if (BRAKE_PRESSED) //TODO
  {
    //OP_ON = false;
  }


  //______________SET OP OFF WHEN GAS IS PRESSED
  if (!GAS_RELEASED)
  {
    // OP_ON = false;
  }

  //________________ShutDown CC if CC is cut
  if (OP_ON && !DASHLIGHT_ON) {
    shutdownCC();
  }

  if (lastButtonState != buttonState) {
    switch (buttonState) {
      case 1:
        toggleOP();
        break;
      case 2:
        if (arduinoCCActive) {
          set_speed = set_speed - 5;
        } else {
          decreaseSpeedHold();
        }
        break;
      case 3:
        if (arduinoCCActive) {
          set_speed = set_speed + 5;
        } else {
          increaseSpeedHold();
        }
        break;
      case 4:
        if (arduinoCCActive)
        {
          shutdownCC();
        }
        else
        {
          startCC();
          set_speed = set_speed + 5;
        }
        break;
      default:
        if (!arduinoCCActive) {
          digitalWrite(minusRelay_pin, HIGH);
          digitalWrite(plusRelay_pin, HIGH);
        }
        break;
    }
    //______________LIMIT FOR SETSPEED
    if (set_speed > 200)
    {
      set_speed = min(average + 3, 200);
    }
  }

}

//helper function
void printFloat(float in) {
  char sprintfbuffer[15];
  dtostrf(in, 5, 3, sprintfbuffer);
  //Serial.print(sprintfbuffer);
}

void tapPlusButton() {
  digitalWrite(plusRelay_pin, LOW);
  delay(200);
  digitalWrite(plusRelay_pin, HIGH);
}

void tapOffButton() {
  digitalWrite(offRelay_pin, LOW);
  delay(200);
  digitalWrite(offRelay_pin, HIGH);
}

void shutdownCC() {
  digitalWrite(plusRelay_pin, HIGH);
  digitalWrite(minusRelay_pin, HIGH);
  tapOffButton();
  delay(200);
  arduinoCCActive = false;
}

void toggleOP() {
  OP_ON = !OP_ON;
}

void shutdownOP() {
  OP_ON = false;
}

void startCC() {
  if (OP_ON) {
    tapPlusButton();
    arduinoCCActive = true;
  }
}

void readAllInputs() {
  buttonVal = analogRead(button1to4_pin);
  lastButtonState = buttonState; //TODO DONE
  if (buttonVal < 390) buttonState = 0;
  else if (buttonVal < 560) buttonState = 1;
  else if (buttonVal < 700) buttonState = 2;
  else if (buttonVal < 950) buttonState = 3;
  else buttonState = 4;

  BLINKER_LEFT = digitalRead(blinkerL_pin) == HIGH;
  BLINKER_RIGHT = digitalRead(blinkerR_pin) == HIGH;
  lastBRAKE_PRESSED = BRAKE_PRESSED;
  testAnalog = analogRead(brakelight_pin);
  BRAKE_PRESSED = testAnalog > 600;
  DASHLIGHT_ON = digitalRead(dashlight_pin) == LOW;
  BRAKE_POSITION = analogRead(brakePosition_pin);
  GAS_POSITION = analogRead(throttlePosition_pin);
  lastGAS_RELEASED = GAS_RELEASED;
  GAS_RELEASED = GAS_POSITION < 200; //TODO: Ajust
}

void readSPDSensor() {
  //______________READ SPD SENSOR
  attachInterrupt(1, rpm, FALLING); 

  if (half_revolutions >= 1) {
    detachInterrupt(1);
    duration = (micros() - lastmillis);
    spd = half_revolutions * (0.000135 / (duration * 0.000001)) * 3600;
    lastmillis = micros();
    half_revolutions = 0;
    attachInterrupt(1, rpm, FALLING);
  }

  //______________SMOOTH SPD TO AVERAGE
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = spd;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits

}

void canBroadcasts () {
  //______________SENDING_CAN_MESSAGES

  // 0x2c1 msg GAS_PEDAL
  uint8_t dat_2c1[8];
  dat_2c1[0] = (GAS_RELEASED << 3) & 0x08;
  dat_2c1[1] = 0x0;
  dat_2c1[2] = 0x0;
  dat_2c1[3] = 0x0;
  dat_2c1[4] = 0x0;
  dat_2c1[5] = 0x0;
  dat_2c1[6] = 0x0;
  dat_2c1[7] = 0x0;
  CAN.beginPacket(0x2c1);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_2c1[ii]);
  }
  CAN.endPacket();

  //0x1d2 msg PCM_CRUISE
  uint8_t dat_1d2[8];
  dat_1d2[0] = (OP_ON << 5) & 0x20 | (GAS_RELEASED << 4) & 0x10;
  dat_1d2[1] = 0x0;
  dat_1d2[2] = 0x0;
  dat_1d2[3] = 0x0;
  dat_1d2[4] = 0x0;
  dat_1d2[5] = 0x0;
  dat_1d2[6] = (OP_ON << 7) & 0x80;
  dat_1d2[7] = can_cksum(dat_1d2, 7, 0x1d2);
  CAN.beginPacket(0x1d2);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_1d2[ii]);
  }
  CAN.endPacket();

  //0x1d3 msg PCM_CRUISE_2
  uint8_t dat_1d3[8];
  dat_1d3[0] = 0x0;
  dat_1d3[1] = (MAIN_ON << 7) & 0x80 | 0x28;
  dat_1d3[2] = set_speed;
  dat_1d3[3] = 0x0;
  dat_1d3[4] = 0x0;
  dat_1d3[5] = 0x0;
  dat_1d3[6] = 0x0;
  dat_1d3[7] = can_cksum(dat_1d3, 7, 0x1d3);
  CAN.beginPacket(0x1d3);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_1d3[ii]);
  }
  CAN.endPacket();

  //0xaa msg defaults 1a 6f WHEEL_SPEEDS
  uint8_t dat_aa[8];
  uint16_t wheelspeed = 0x1a6f + (average * 100);
  dat_aa[0] = (wheelspeed >> 8) & 0xFF;
  dat_aa[1] = (wheelspeed >> 0) & 0xFF;
  dat_aa[2] = (wheelspeed >> 8) & 0xFF;
  dat_aa[3] = (wheelspeed >> 0) & 0xFF;
  dat_aa[4] = (wheelspeed >> 8) & 0xFF;
  dat_aa[5] = (wheelspeed >> 0) & 0xFF;
  dat_aa[6] = (wheelspeed >> 8) & 0xFF;
  dat_aa[7] = (wheelspeed >> 0) & 0xFF;
  CAN.beginPacket(0xaa);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_aa[ii]);
  }
  CAN.endPacket();

  //0x3b7 msg ESP_CONTROL BRAKES DISABLE WHEN IMPLEMENTED TODO
  uint8_t dat_3b7[8];
  dat_3b7[0] = (BRAKE_PRESSED << 5) & 0x20;
  dat_3b7[1] = 0x0;
  dat_3b7[2] = 0x0;
  dat_3b7[3] = 0x0;
  dat_3b7[4] = 0x0;
  dat_3b7[5] = 0x0;
  dat_3b7[6] = 0x0;
  dat_3b7[7] = 0x08;
  CAN.beginPacket(0x3b7);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_3b7[ii]);
  }
  CAN.endPacket();

  //0x620 msg STEATS_DOORS
  uint8_t dat_620[8];
  dat_620[0] = 0x10;
  dat_620[1] = 0x0;
  dat_620[2] = 0x0;
  dat_620[3] = 0x1d;
  dat_620[4] = 0xb0;
  dat_620[5] = 0x40;
  dat_620[6] = 0x0;
  dat_620[7] = 0x0;
  CAN.beginPacket(0x620);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_620[ii]);
  }
  CAN.endPacket();

  // 0x3bc msg GEAR_PACKET
  uint8_t dat_3bc[8];
  dat_3bc[0] = 0x0;
  dat_3bc[1] = 0x0;
  dat_3bc[2] = 0x0;
  dat_3bc[3] = 0x0;
  dat_3bc[4] = 0x0;
  dat_3bc[5] = 0x80;
  dat_3bc[6] = 0x0;
  dat_3bc[7] = 0x0;
  CAN.beginPacket(0x3bc);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_3bc[ii]);
  }
  CAN.endPacket();

  //0x614 msg steering_levers
  uint8_t dat_614[8];
  dat_614[0] = 0x29;
  dat_614[1] = 0x0;
  dat_614[2] = 0x01;
  dat_614[3] = (BLINKER_LEFT << 5) & 0x20 | (BLINKER_RIGHT << 4) & 0x10;
  dat_614[4] = 0x0;
  dat_614[5] = 0x0;
  dat_614[6] = 0x76;
  dat_614[7] = can_cksum(dat_614, 7, 0x614);
  CAN.beginPacket(0x614);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_614[ii]);
  }
  CAN.endPacket();
}

//TOYOTA CAN CHECKSUM
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}
void rpm() {
  half_revolutions++;
  if (encoder > 255)
  {
    encoder = 0;
  }
  encoder++;
}

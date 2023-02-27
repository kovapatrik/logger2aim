#include <TimerOne.h>
#include <SPI.h>
#include <mcp2515.h>

//PINs

const int CAN_PIN = 10;

const int RPM_PIN = 2;
const int SPEED_PIN = 3;

const int TEMP_PIN = A2;
const int TPS_PIN = A3;
const int OILPRES_PIN = A4;
const int BRAKE_PIN = A5;
const int BATTERY_PIN = A6;
const int LAMBDA_PIN = A7;

//Constants
const float THERMISTOR_R1 = 1800.0;
const float BETA_VALUE = 3812.0;
const float BETA_TEMP = 298.15;
const float BETA_R = 1980.0;

const long RPM_COEFF = 30000000;
const long SPEED_COEFF = 1400000;

const float MIN_TPS = 160;
const float MAX_TPS = 795 - MIN_TPS;

const long BRAKE_THR = 200, OIL_THR = 100;

//Variables
unsigned long rpmBegin, speedBegin, rpmDuration, speedDuration, preSpeedDuration, rpmTemp, speedTemp;
long pre_tps, pre_battery, pre_oil, pre_brake, pre_lambda;
long temp, rpm, speed, t, tps, oil, brake, battery, lambda;
float pre_temp;
int rpmFlag, speedFlag;

struct can_frame MSG_RPM_TPS_SPEED,
                 MSG_BREAK,
                 MSG_TEMP,
                 MSG_OIL,
                 MSG_BATTERY,
                 MSG_LAMBDA,
                 MSG_USER;
                 
MCP2515 mcp2515(CAN_PIN);

void setup() {

    Serial.begin(9600);

    initCANMessages();
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS);
    mcp2515.setNormalMode();

    pinMode(RPM_PIN, INPUT);
    pinMode(SPEED_PIN, INPUT);
                                                   
    temp = 0; rpm = 0; speed = 0; tps = 0; oil = 0; brake = 0; battery = 0; lambda = 0;
    pre_tps = 0; pre_brake = 0; pre_battery = 0; pre_lambda = 0;

    rpmFlag = 0; speedFlag = 0;

    Timer1.initialize(50000);
    Timer1.attachInterrupt(sendCANData);


    attachInterrupt(digitalPinToInterrupt(RPM_PIN),
                    RPMInterrupt,
                    CHANGE);

    attachInterrupt(digitalPinToInterrupt(SPEED_PIN),
                    SpeedInterrupt,
                    CHANGE);

    Serial.print("Sending data...");
}

void loop() {
  temp = (NTC2TEMP(analogRead(TEMP_PIN)) * 10 + 450) * 19;
  MSG_TEMP.data[2] = (uint8_t)(temp);
  MSG_TEMP.data[3] = (uint8_t)(temp >> 8);
  
  pre_tps = (analogRead(TPS_PIN) - MIN_TPS) / MAX_TPS * 100;
  pre_tps = min(max(pre_tps, 0),100);
  tps = 65000 * pre_tps / 100;
  MSG_RPM_TPS_SPEED.data[2] = (uint8_t)(tps);
  MSG_RPM_TPS_SPEED.data[3] = (uint8_t)(tps >> 8);
  
  pre_oil = analogRead(OILPRES_PIN);
  if (pre_oil > OIL_THR) {
    oil = 1000;
  } else {
    oil = 0;
  }
  MSG_OIL.data[4] = (uint8_t)(oil);
  MSG_OIL.data[5] = (uint8_t)(oil >> 8);
  
  pre_brake = analogRead(BRAKE_PIN);
  if (pre_brake > BRAKE_THR) {
    brake = 43000;
  } else {
    brake = 0;
  }
  MSG_BREAK.data[4] = (uint8_t)(brake);
  MSG_BREAK.data[5] = (uint8_t)(brake >> 8);
  
  pre_battery = analogRead(BATTERY_PIN);
  pre_battery = pre_battery * (23.8 / 1023.0) * 32000;
  battery = pre_battery / 10;
  MSG_BATTERY.data[2] = (uint8_t)(battery);
  MSG_BATTERY.data[3] = (uint8_t)(battery >> 8);
  
  pre_lambda = analogRead(LAMBDA_PIN);
  pre_lambda = pre_lambda * (5.0 / 1023.0) * 20000; // (2.92 / 616)
  lambda = pre_lambda / 10;
  MSG_LAMBDA.data[0] = (uint8_t)(lambda);
  MSG_LAMBDA.data[1] = (uint8_t)(lambda >> 8);
  
}

void sendCANData() {
  mcp2515.sendMessage(&MSG_RPM_TPS_SPEED);
  mcp2515.sendMessage(&MSG_BREAK);
  mcp2515.sendMessage(&MSG_TEMP);
  mcp2515.sendMessage(&MSG_OIL);
  mcp2515.sendMessage(&MSG_BATTERY);
  mcp2515.sendMessage(&MSG_LAMBDA);
  //mcp2515.sendMessage(&MSG_USER);
}

float NTC2TEMP(int v) {
  float NTC_R = (THERMISTOR_R1 * v) / (1023.0 - v);
  float Tk =  1.0 / ((1.0 / BETA_TEMP) - (log(BETA_R / NTC_R) / BETA_VALUE));
  return round((Tk - 273.15) * 10) / 10.0;
}

void RPMInterrupt()
{
  if (digitalRead(RPM_PIN) == LOW) {
    if (rpmFlag) {
      rpmTemp = micros();
      rpmDuration = RPM_COEFF / (rpmTemp - rpmBegin);

      MSG_RPM_TPS_SPEED.data[0] = (uint8_t)(rpmDuration);
      MSG_RPM_TPS_SPEED.data[1] = (uint8_t)(rpmDuration >> 8);

      rpmBegin = rpmTemp;
      rpmFlag = 0;
    } else {
      rpmBegin = micros();
      rpmFlag = 1;
    }
  }
}

void SpeedInterrupt()
{
  if (digitalRead(SPEED_PIN) == HIGH) {
    if (speedFlag) {
      speedTemp = micros();
      preSpeedDuration = SPEED_COEFF / (speedTemp - speedBegin);
      if (preSpeedDuration < 10) {
        speedDuration = 0;
      } else {
        speedDuration = preSpeedDuration * 100;
      }

      MSG_RPM_TPS_SPEED.data[6] = (uint8_t)(speedDuration);
      MSG_RPM_TPS_SPEED.data[7] = (uint8_t)(speedDuration >> 8);

      speedBegin = speedTemp;
      speedFlag = 0;
    } else {
      speedBegin = micros();
      speedFlag = 1;
    }
  }
}

void initCANMessages() {
  MSG_RPM_TPS_SPEED.can_id  = 0x5F0;
  MSG_RPM_TPS_SPEED.can_dlc = 8;
  MSG_RPM_TPS_SPEED.data[0] = 0x00; // RPM
  MSG_RPM_TPS_SPEED.data[1] = 0x00; // RPM
  MSG_RPM_TPS_SPEED.data[2] = 0x00; // TPS
  MSG_RPM_TPS_SPEED.data[3] = 0x00; // TPS
  MSG_RPM_TPS_SPEED.data[4] = 0x00;
  MSG_RPM_TPS_SPEED.data[5] = 0x00;
  MSG_RPM_TPS_SPEED.data[6] = 0x00; // SPEED
  MSG_RPM_TPS_SPEED.data[7] = 0x00; // SPEED

  MSG_BREAK.can_id  = 0x5F8;
  MSG_BREAK.can_dlc = 6;
  MSG_BREAK.data[0] = 0x00;
  MSG_BREAK.data[1] = 0x00;
  MSG_BREAK.data[2] = 0xFF; // BATTERY PLACEHOLDER
  MSG_BREAK.data[3] = 0x00;
  MSG_BREAK.data[4] = 0x00; // BREAK
  MSG_BREAK.data[5] = 0x00; // BREAK
  MSG_BREAK.data[6] = 0x00;
  MSG_BREAK.data[7] = 0x00;

  MSG_TEMP.can_id  = 0x5F2;
  MSG_TEMP.can_dlc = 4;
  MSG_TEMP.data[0] = 0x00;
  MSG_TEMP.data[1] = 0x00; 
  MSG_TEMP.data[2] = 0x00; // TEMP
  MSG_TEMP.data[3] = 0x00; // TEMP
  MSG_TEMP.data[4] = 0x00;
  MSG_TEMP.data[5] = 0x00;
  MSG_TEMP.data[6] = 0x00;
  MSG_TEMP.data[7] = 0x00;
  
  MSG_OIL.can_id  = 0x5F3;
  MSG_OIL.can_dlc = 6;
  MSG_OIL.data[0] = 0x00;
  MSG_OIL.data[1] = 0x00;
  MSG_OIL.data[2] = 0x00; 
  MSG_OIL.data[3] = 0x00;
  MSG_OIL.data[4] = 0x00; // OIL
  MSG_OIL.data[5] = 0x00; // OIL
  MSG_OIL.data[6] = 0x00;
  MSG_OIL.data[7] = 0x00;

  MSG_BATTERY.can_id  = 0x5F4;
  MSG_BATTERY.can_dlc = 4;
  MSG_BATTERY.data[0] = 0x00;
  MSG_BATTERY.data[1] = 0x00; 
  MSG_BATTERY.data[2] = 0x00; // BATTERY
  MSG_BATTERY.data[3] = 0x00; // BATTERY
  MSG_BATTERY.data[4] = 0x00;
  MSG_BATTERY.data[5] = 0x00;
  MSG_BATTERY.data[6] = 0x00;
  MSG_BATTERY.data[7] = 0x00;

  MSG_LAMBDA.can_id  = 0x5F6; 
  MSG_LAMBDA.can_dlc = 2;
  MSG_LAMBDA.data[0] = 0x00; // LAMBDA
  MSG_LAMBDA.data[1] = 0x00; // LAMBDA
  MSG_LAMBDA.data[2] = 0x00;
  MSG_LAMBDA.data[3] = 0x00;
  MSG_LAMBDA.data[4] = 0x00;
  MSG_LAMBDA.data[5] = 0x00;
  MSG_LAMBDA.data[6] = 0x00;
  MSG_LAMBDA.data[7] = 0x00;

  MSG_USER.can_id  = 0x5FE;
  MSG_USER.can_dlc = 2;
  MSG_USER.data[0] = 0x10;
  MSG_USER.data[1] = 0x00; 
  MSG_USER.data[2] = 0x00; // BATTERY
  MSG_USER.data[3] = 0x00; // BATTERY
  MSG_USER.data[4] = 0x00;
  MSG_USER.data[5] = 0x00;
  MSG_USER.data[6] = 0x00;
  MSG_USER.data[7] = 0x00;

}

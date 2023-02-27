const int TEMP_PIN = A2;
const int TPS_PIN = A3;
const int OILPRES_PIN = A4;
const int BRAKE_PIN = A5;
const int BATTERY_PIN = A6;
const int LAMBDA_PIN = A7;

void setup() {
  Serial.begin(9600);
  while(!Serial){}
  Serial.println("Temp,Battery,Lambda,Oil,Brake,Tps");
}

void loop() {
  Serial.print(analogRead(TEMP_PIN));
  Serial.print(',');
  Serial.print(analogRead(BATTERY_PIN));
  Serial.print(',');
  Serial.print(analogRead(LAMBDA_PIN));
  Serial.print(',');
  Serial.print(analogRead(OILPRES_PIN));
  Serial.print(',');
  Serial.print(analogRead(BRAKE_PIN));
  Serial.print(',');
  Serial.println(analogRead(TPS_PIN));
  delay(50);
}

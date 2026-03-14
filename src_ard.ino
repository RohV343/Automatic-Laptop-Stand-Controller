// Motor pins
const int MotorA_IN1 = 2;
const int MotorA_IN2 = 3;
const int MotorB_IN1 = 4;
const int MotorB_IN2 = 5;

// Fan
const int FAN_PIN = 6;

// Ultrasonic
const int TRIG = 8;
const int ECHO = 9;

// Temperature
const int TEMP_ALERT = 10;
const int TEMP_SENSOR_PIN = A0;

void setup() {
  Serial.begin(9600);

  pinMode(MotorA_IN1, OUTPUT);
  pinMode(MotorA_IN2, OUTPUT);
  pinMode(MotorB_IN1, OUTPUT);
  pinMode(MotorB_IN2, OUTPUT);

  pinMode(FAN_PIN, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(TEMP_ALERT, INPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);

  stopMotors();
  fanOff();
}

long getDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 255;

  return duration / 58;
}

float readTemperatureC() {
  int analogValue = analogRead(TEMP_SENSOR_PIN);
  float voltage = analogValue * (5.0 / 1023.0);
  return voltage * 100.0;
}

void moveForward() {
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, HIGH);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, HIGH);
}

void moveBackward() {
  digitalWrite(MotorA_IN1, HIGH);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, HIGH);
  digitalWrite(MotorB_IN2, LOW);
}

void stopMotors() {
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
}

void fanOn() {
  digitalWrite(FAN_PIN, HIGH);
}

void fanOff() {
  digitalWrite(FAN_PIN, LOW);
}

void loop() {

  long distance = getDistanceCM();
  float temperature = readTemperatureC();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm  Temp: ");
  Serial.println(temperature, 2);

  if (temperature > 30.0)
    fanOn();
  else
    fanOff();

  if (distance < 20)
    moveBackward();
  else if (distance > 40)
    moveForward();
  else
    stopMotors();

  delay(500);
}
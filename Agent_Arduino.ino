#include <WiFi.h>
#include <Wire.h>
#include <Servo.h>

const char* ssid = "SKYTNJUK";
const char* password = "RjkZs5fAXsTZ";
WiFiServer server(5000);
WiFiClient client;

const int PWMA = 10;
const int AIN1 = 8;
const int AIN2 = 7;
const int PWMB = 11;
const int BIN1 = 12;
const int BIN2 = 13;

int leftSpeed  = 150;
int rightSpeed = 108; // I have no clue why this has such a severe difference in speed but this was as straight as i could get it

int turnSpeedA = 120;
int turnSpeedB = 120;

#define STEP_DISTANCE_CM      2.0
#define MOVE_TIME_MS          240
#define TURN_TIME_MS          250
#define SETTLE_AFTER_MOVE_MS  300
#define SETTLE_AFTER_TURN_MS  200

#define TRIG_PIN     5
#define ECHO_PIN     6
#define SERVO_PIN    9
#define STEP_DEG     5
#define READINGS     5
#define MAX_DIST_CM  350
#define MIN_DIST_CM  2
#define SETTLE_MS    150
#define SERVO_TRIM   -22

Servo sweepServo;
#define MPU_ADDR 0x68

int16_t ax, ay, az, gx, gy, gz;
float gyroZoffset = 0.0;
float yawDeg = 0.0;
unsigned long prevTime = 0;
float posX_cm = 0.0;
float posY_cm = 0.0;
float lastStepDX_cm = 0.0;
float lastStepDY_cm = 0.0;
float lastStepDYaw_deg = 0.0;
bool stopRequested = false;

float wrapAngleDeg(float a)
{
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

void readMPU()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); 
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void updateYaw()
{
  readMPU();
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  if (dt <= 0.0 || dt > 0.2) return;
  float gyroZ = (gz - gyroZoffset) / 131.0; 
  yawDeg = wrapAngleDeg(yawDeg + gyroZ * dt);
}

void stopMotors()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void startForward()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);
}

void startTurnLeft()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, turnSpeedA);
  analogWrite(PWMB, turnSpeedB);
}

void startTurnRight()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, turnSpeedA);
  analogWrite(PWMB, turnSpeedB);
}

bool checkForStopCommand()
{
  if (client && client.connected() && client.available())
  {
    String cmd = client.readStringUntil('\n');
    cmd.trim();
    Serial.print("CMD(interrupt): ");
    Serial.println(cmd);
    if (cmd == "STOP")
    {
      stopRequested = true;
      stopMotors();
      client.println("STOP_DONE");
      return true;
    }
  }
  return false;
}

void moveForwardStep()
{
  stopRequested = false;
  float yawBefore = yawDeg;
  unsigned long startTime = millis();
  startForward();
  while (millis() - startTime < MOVE_TIME_MS)
  {
    if (checkForStopCommand()) return;
    updateYaw();
    delay(5);
  }
  stopMotors();
  unsigned long settleStart = millis();
  while (millis() - settleStart < SETTLE_AFTER_MOVE_MS)
  {
    if (checkForStopCommand()) return;
    updateYaw();
    delay(5);
  }
  float yawAfter = yawDeg;
  float yawRad = yawAfter * PI / 180.0;
  lastStepDX_cm = STEP_DISTANCE_CM * sin(yawRad);
  lastStepDY_cm = STEP_DISTANCE_CM * cos(yawRad);
  lastStepDYaw_deg = wrapAngleDeg(yawAfter - yawBefore);
  posX_cm += lastStepDX_cm;
  posY_cm += lastStepDY_cm;
}

void turnTimed(bool turnLeft)
{
  stopRequested = false;
  if (turnLeft)
    startTurnLeft();
  else
    startTurnRight();
  unsigned long startTime = millis();
  while (millis() - startTime < TURN_TIME_MS)
  {
    if (checkForStopCommand()) return;
    updateYaw();
    delay(5);
  }
  stopMotors();
  unsigned long settleStart = millis();
  while (millis() - settleStart < SETTLE_AFTER_TURN_MS)
  {
    if (checkForStopCommand()) return;
    updateYaw();
    delay(5);
  }
  lastStepDX_cm = 0.0;
  lastStepDY_cm = 0.0;
  lastStepDYaw_deg = turnLeft ? 15.0 : -15.0;
}

float readDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return -1.0;
  return (duration * 0.0343) / 2.0;
}

float filteredReading()
{
  float sum = 0.0;
  int valid = 0;
  for (int i = 0; i < READINGS; i++)
  {
    float d = readDistance();
    if (d >= MIN_DIST_CM && d <= MAX_DIST_CM)
    {
      sum += d;
      valid++;
    }
    delay(15);
  }
  if (valid == 0) return -1.0;
  return sum / valid;
}

void sendOdom()
{
  client.print("ODOM,");
  client.print(lastStepDX_cm, 3);
  client.print(",");
  client.print(lastStepDY_cm, 3);
  client.print(",");
  client.println(lastStepDYaw_deg, 3);
}

void performSweep()
{
  client.println("SWEEP_START");
  for (int angle = 0; angle <= 180; angle += STEP_DEG)
  {
    sweepServo.write(angle + SERVO_TRIM);
    delay(SETTLE_MS);
    float dist = filteredReading();
    client.print(angle);
    client.print(",");
    if (dist < 0)
      client.println("INVALID");
    else
      client.println(dist);
  }
  sweepServo.write(90 + SERVO_TRIM);
  client.println("SWEEP_END");
  client.print("POSE,");
  client.print(posX_cm, 3);
  client.print(",");
  client.print(posY_cm, 3);
  client.print(",");
  client.println(yawDeg, 3);
}

void resetPose()
{
  posX_cm = 0.0;
  posY_cm = 0.0;
  yawDeg = 0.0;
  lastStepDX_cm = 0.0;
  lastStepDY_cm = 0.0;
  lastStepDYaw_deg = 0.0;
  prevTime = millis();
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Wire.begin();
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  stopMotors();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  sweepServo.attach(SERVO_PIN);
  sweepServo.write(90 + SERVO_TRIM);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("Calibrating gyro...");
  delay(2000);
  for (int i = 0; i < 1000; i++)
  {
    readMPU();
    gyroZoffset += gz;
    delay(2);
  }
  gyroZoffset /= 1000.0;
  Serial.print("gyroZoffset = ");
  Serial.println(gyroZoffset);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  delay(1000);
  IPAddress ip = WiFi.localIP();
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(ip);
  if (ip[0] == 0)
  {
    Serial.println("ERROR: invalid IP address");
  }
  server.begin();
  resetPose();
}

void loop()
{
  WiFiClient newClient = server.available();
  if (newClient && !client.connected())
  {
    client = newClient;
    Serial.println("Python connected");
    client.println("CONNECTED");
  }
  updateYaw();
  if (client && client.connected() && client.available())
  {
    String cmd = client.readStringUntil('\n');
    cmd.trim();
    Serial.print("CMD: ");
    Serial.println(cmd);
    if (cmd == "RESET")
    {
      resetPose();
      client.println("RESET_DONE");
    }
    else if (cmd == "SCAN")
    {
      performSweep();
    }
    else if (cmd == "MOVE")
    {
      moveForwardStep();
      if (!stopRequested) sendOdom();
    }
    else if (cmd == "TURNL")
    {
      turnTimed(true);
      if (!stopRequested) sendOdom();
    }
    else if (cmd == "TURNR")
    {
      turnTimed(false);
      if (!stopRequested) sendOdom();
    }
    else if (cmd == "STOP")
    {
      stopRequested = true;
      stopMotors();
      client.println("STOP_DONE");
    }
  }
}
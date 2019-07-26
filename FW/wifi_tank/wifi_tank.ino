#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Math.h>
#include <MPU6050.h>
#include <I2Cdev.h>

#ifndef STASSID
#define STASSID "your_ssid"
#define STAPSK  "your_password"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

static float voltage = 5.0;

#define pinA1 12
#define pinA2 13
#define pinB1 4
#define pinB2 15

#define pinCRG 16

#define GYRO_TO_DEGREE (1.0 / 16.4) // 16.4 LSB/dps
#define ACCEL_TO_G (1.0 / 8192) // 8192 per g

int16_t targetSpeed[2] = { 0, 0 }, actualSpeed[2] = { 0, 0 };
float forwardSpeed, angleSpeed;
float temperature;

#define SPEED_UPDATE_INTERVAL 0

// max analogWrite output for esp8266 is 1023
#define MAX_SPEED_VALUE 1023
#define MIN_SPEED_VALUE 50

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[256];
char replyPacket[] = "{\"type\":\"ack\"}";
char updMsg[256];

#define I2C_PIN_SDA      2
#define I2C_PIN_SCL     14

MPU6050 gyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const char* mdnsName = "tank1";

static uint32_t lastPacketTime = 0;

static float radToDeg = 180/3.141592654;  
static float gyroErrorZ = 0;
static double tankDirection = 0, dirError = 0, pidOut = 0, target = 0;
static float targetDirection = 0;
static float pidP = 0.05, pidI = 0.1, pidD = 0.01;
//                            speed by accel data
static float accelErrorX = 0, realSpeed = 0;

static uint32_t timeFromLastCharge = 0;

#define ANGLE_SPEED_MULT 1.0

PID dirPID(&dirError, &pidOut, &target, pidP, pidI, pidD, DIRECT);

void startMDNS() { // Start the mDNS responder
  if (MDNS.begin(mdnsName)) {
    MDNS.addService("tank", "udp", localUdpPort);
    Serial.print("mDNS responder started: http://");
    Serial.print(mdnsName);
    Serial.println(".local");
  }
}

void setup() {
  WiFi.setOutputPower(0);
    
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);

  analogWriteFreq(20000);
  analogWrite(pinA1, 0);
  analogWrite(pinA2, 0);
  analogWrite(pinB1, 0);
  analogWrite(pinB2, 0);

  pinMode(pinCRG, INPUT_PULLUP);
  
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } 
    else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } 
    else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } 
    else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } 
    else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  startMDNS();
  
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  

  Udp.begin(localUdpPort);

  SetSpeed(0, 0);
  SetSpeed(1, 0);

  Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);

  gyro.setClockSource(0x00); // internal clock source
  gyro.setFullScaleGyroRange(0x03); // 2000 dps
  gyro.setFullScaleAccelRange(0x01); // 4g
  gyro.setSleepEnabled(false);
  
  Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  InitGyroError();

  dirPID.SetMode(AUTOMATIC);
  dirPID.SetOutputLimits(-0.5, 0.5);
  dirPID.SetSampleTime(20);
}

// init gyro and accel offset
// do not move the tank after power on until ready
void InitGyroError() {
  delay(1000);
  
  for(uint8_t i = 0; i < 200; i++) {
    // wait for new IMU data available
    while (!gyro.getIntDataReadyStatus());
    
    gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    gyroErrorZ = gyroErrorZ + (((float)gz) * GYRO_TO_DEGREE);
    accelErrorX = accelErrorX + (((float)ax) * ACCEL_TO_G);
  }

  gyroErrorZ = gyroErrorZ / 200;
  accelErrorX = accelErrorX / 200;

  Serial.printf("Gyro error: %f degrees\n", gyroErrorZ);
  Serial.printf("Accel error: %f g\n", accelErrorX);
}

void SetSpeed(uint8_t traverse, int16_t value) {
  if (abs(value) < MIN_SPEED_VALUE) {
    value = 0;
  }
  
  if (!traverse) {
    if (value > 0) {
      analogWrite(pinA1, abs(value));
      analogWrite(pinA2, 0);
    }
    else {
      analogWrite(pinA1, 0);
      analogWrite(pinA2, abs(value));
    }
  }
  else {
    if (value > 0) {
      analogWrite(pinB1, abs(value));
      analogWrite(pinB2, 0);
    }
    else {
      analogWrite(pinB1, 0);
      analogWrite(pinB2, abs(value));
    }
  }
}

void UpdateSpeed() {
  static uint32_t prevUpdate;

  if (millis() - prevUpdate < SPEED_UPDATE_INTERVAL) {
    return;
  }
  
  for (uint8_t i = 0; i < 2; i++) {
    if (targetSpeed[i] > MAX_SPEED_VALUE) {
      targetSpeed[i] = MAX_SPEED_VALUE;
    }

    if (targetSpeed[i] < -MAX_SPEED_VALUE) {
      targetSpeed[i] = -MAX_SPEED_VALUE;
    }

    if (actualSpeed[i] < targetSpeed[i]) {
      actualSpeed[i] += 5;

      if (actualSpeed[i] > MAX_SPEED_VALUE) {
        actualSpeed[i] = MAX_SPEED_VALUE;
      }

      SetSpeed(i, actualSpeed[i]);
    }

    if (actualSpeed[i] > targetSpeed[i]) {
      actualSpeed[i] -= 5;

      if (actualSpeed[i] < -MAX_SPEED_VALUE) {
        actualSpeed[i] = -MAX_SPEED_VALUE;
      }

      SetSpeed(i, actualSpeed[i]);
    }
  }

  prevUpdate = millis();
}

void UpdateUDP() {
  int packetSize = Udp.parsePacket();
  
  if (packetSize) {
    lastPacketTime = millis();
    
    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);

    switch (incomingPacket[0]) {
      case 0x01: { // control command
          memcpy(&forwardSpeed, &incomingPacket[1], 4);
          memcpy(&angleSpeed,   &incomingPacket[5], 4);

          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(replyPacket);
          Udp.endPacket();
        }
        break;
      case 'v': {
          voltage = analogRead(A0);
          voltage /= 198.3; // voltage divider with 47k/200k
          uint8_t charging = !digitalRead(pinCRG);

          if (charging) {
            timeFromLastCharge = millis();
          }
            
          sprintf(updMsg, "{\"type\":\"status\",\"data\":{\"voltage\":%f,\"crg\":%d,\"t\":%f,\"last_pwr\":%u}}", voltage, charging, temperature, (millis() - timeFromLastCharge) / 1000);
        
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(updMsg);
          Udp.endPacket();
        }
        break;
      case 'd': { 
          sprintf(updMsg, "{\"type\":\"direction\",\"data\":{\"angleZ\":%f,\"errZ\":%f,\"pidOut\":%f,\"targetZ\":%f,\"speed\":%f}}", -tankDirection, dirError, pidOut, targetDirection, realSpeed);
        
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(updMsg);
          Udp.endPacket();
        }
        break;
      case 0x02: {
          memcpy(&pidP, &incomingPacket[1], 4);
          memcpy(&pidI, &incomingPacket[5], 4);
          memcpy(&pidD, &incomingPacket[9], 4);

          dirPID.SetTunings(pidP, pidI, pidD);

          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(replyPacket);
          Udp.endPacket();
        }
        break;
    }
  }
}

void UpdateDir() {
  static uint32_t prevUpdate;

  if (millis() - prevUpdate < SPEED_UPDATE_INTERVAL) {
    return;
  }

  if (fabs(forwardSpeed * MAX_SPEED_VALUE) < MIN_SPEED_VALUE) {
    targetSpeed[0] = 0;
    targetSpeed[1] = 0;
    
    realSpeed = 0;
  }
  else {
    targetDirection = tankDirection - angleSpeed * ANGLE_SPEED_MULT;
    
    if (forwardSpeed > 0) {
      if (pidOut > 0) {
        targetSpeed[1] = (int16_t)round(MAX_SPEED_VALUE * forwardSpeed);
        targetSpeed[0] = (int16_t)round(MAX_SPEED_VALUE * (forwardSpeed * (1.0 - fabs(pidOut))));
      }
      else {   
        targetSpeed[1] = (int16_t)round(MAX_SPEED_VALUE * (forwardSpeed * (1.0 - fabs(pidOut))));
        targetSpeed[0] = (int16_t)round(MAX_SPEED_VALUE * forwardSpeed);
      }
    }
    else {
      if (pidOut > 0) {
        targetSpeed[0] = (int16_t)round(MAX_SPEED_VALUE * forwardSpeed);
        targetSpeed[1] = (int16_t)round(MAX_SPEED_VALUE * (forwardSpeed * (1.0 - fabs(pidOut))));
      }
      else {   
        targetSpeed[0] = (int16_t)round(MAX_SPEED_VALUE * (forwardSpeed * (1.0 - fabs(pidOut))));
        targetSpeed[1] = (int16_t)round(MAX_SPEED_VALUE * forwardSpeed);
      }
    }
  }

  while (targetDirection > 360) {
    targetDirection -= 360;
  }

  while (targetDirection < -360) {
    targetDirection += 360;
  }
}

void UpdateGyro() {
  static uint32_t prevTick = 0;
  uint8_t gyroStatus = gyro.getIntDataReadyStatus();

  if (!prevTick) {
    prevTick = micros();
  }

  if (gyroStatus) {
    float timeDelta = micros() - prevTick;
    prevTick = micros();
    
    gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (timeDelta) {
      tankDirection += (((float)gz) * GYRO_TO_DEGREE - gyroErrorZ) * (timeDelta / 1000000.0);

      while (tankDirection < -360.0) {
        tankDirection += 360.0;
      }

      while (tankDirection > 360.0) {
        tankDirection -= 360.0;
      }

      dirError = targetDirection - tankDirection;

      while (dirError < -180.0) {
        dirError += 360.0;
      }

      while (dirError > 180.0) {
        dirError -= 360.0;
      }

      realSpeed += (((float)ax) * ACCEL_TO_G - accelErrorX) * (timeDelta / 1000000.0);
    }
  }
}

void UpdateTemp() {
  temperature = ((float)(gyro.getTemperature())) / 340.0 + 36.53;
}

void loop() { 
  static uint32_t notifyTick;

  if (millis() - notifyTick > 1000) {
    notifyTick = millis(); 

    UpdateTemp();
    
    Serial.printf("Tank direction: %f\n", tankDirection);
    Serial.printf("Gyro Z: %d\n", gz);
    Serial.printf("Temp: %f\n", temperature);
  }
  
  if (millis() - lastPacketTime > 3000) {    
    targetSpeed[0] = 0;
    targetSpeed[1] = 0;
    angleSpeed = 0;
    forwardSpeed = 0;
  }

  UpdateGyro();  
  dirPID.Compute();
  UpdateDir();
  UpdateUDP();
  ArduinoOTA.handle();
  UpdateSpeed();
}

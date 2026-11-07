/*
  ESP32 DevKit v1 -- PCA9685 pan/tilt servo bridge
  ==================================================
  Reads lines like "PAN,TILT\n" (e.g. "97.3,88.1\n") over USB serial
  and drives the two SG90s accordingly via the PCA9685.

  Wiring (ESP32 DevKit v1 default I2C pins):
    ESP32 3V3  -> PCA9685 VCC (logic supply)
    ESP32 GND  -> PCA9685 GND  -> ALSO tie to external 5V supply GND
    ESP32 GPIO21 (SDA) -> PCA9685 SDA
    ESP32 GPIO22 (SCL) -> PCA9685 SCL
    PCA9685 V+  -> external 5V 2A+ supply (NOT the ESP32's 5V pin -- servos draw
                   too much current for that, especially both moving at once)
    Servos      -> PCA9685 channel 0 (pan) and channel 1 (tilt)

  Library needed (Arduino IDE -> Library Manager):
    "Adafruit PWM Servo Driver Library"
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Tune these two for your specific SG90s if 0/180 deg feel off --
// measure with a servo tester or just eyeball travel limits.
#define SERVO_MIN_PULSE  102   // pulse count (out of 4096) corresponding to 0 deg
#define SERVO_MAX_PULSE  512   // pulse count corresponding to 180 deg

#define PAN_CHANNEL   0
#define TILT_CHANNEL  1

#define SDA_PIN 21
#define SCL_PIN 22

String inputLine = "";

void setAngle(uint8_t channel, float angleDeg) {
  angleDeg = constrain(angleDeg, 0.0, 180.0);
  int pulse = map((int)(angleDeg * 10), 0, 1800, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  pwm.setPWM(channel, 0, pulse);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50);  // standard servo refresh rate

  // center both servos on boot
  setAngle(PAN_CHANNEL, 90.0);
  setAngle(TILT_CHANNEL, 90.0);
 
  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int commaIdx = inputLine.indexOf(',');
      if (commaIdx > 0) {
        float panAngle = inputLine.substring(0, commaIdx).toFloat();
        float tiltAngle = inputLine.substring(commaIdx + 1).toFloat();
        
        // Debug print to confirm parsing
        Serial.print("Parsed pan=");
        Serial.print(panAngle);
        Serial.print(", tilt=");
        Serial.println(tiltAngle);
        
        setAngle(PAN_CHANNEL, panAngle);
        setAngle(TILT_CHANNEL, tiltAngle);
      } else {
        Serial.println("Invalid command: " + inputLine);
      }
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}


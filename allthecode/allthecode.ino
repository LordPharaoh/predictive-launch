#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3DH.h>
#include <string.h>


// Barometer pinout
#define SEA_PRESSURE 1013.25
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
// Accelerometer pinout
#define ACC_CLK 2
#define ACC_MISO 3
#define ACC_MOSI 5
#define ACC_CS 6
#define ACC_RANGE LIS3DH_RANGE_4_G

#define LED 8
#define BUTTON 9

#define NUM_OBSERVATIONS 500

#define FORMAT_OBS_LEN 500

//accelerometer setup
Adafruit_LIS3DH acc = Adafruit_LIS3DH(ACC_CS, ACC_MOSI, ACC_MISO, ACC_CLK);
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);


typedef struct {
  float ms;
  float altitude;
  float acceleration;
  float velocity;
  float calculated_drag;
  float predicted_altitude;
  float airbrakes;
} Observation;

float altitude_zero;
//Observation storage
Observation observation_log[NUM_OBSERVATIONS];
size_t observation_idx;

int formatObs(char* buf, Observation obs) {
    snprintf(
        buf, FORMAT_OBS_LEN, 
        "%10f, %10f, %10f, %10f, %10f, %10f, %10f\n",
        obs.ms, obs.altitude, obs.acceleration, obs.velocity,
        obs.calculated_drag, obs.predicted_altitude, obs.airbrakes
    );
    // 7 floats each with 10 characters, a comma, a space, plus one newline
    //should return the length of the string
    return 83;
}

float getAcceleration() {
  //TODO kalman?
  acc.read();
  sensors_event_t event;
  acc.getEvent(&event);
  return event.acceleration.z;
}

void zeroAltitude() {
  altitude_zero = bmp.readAltitude(SEA_PRESSURE);
}

float getAltitude() {
  //TODO kalman?
  return bmp.readAltitude(SEA_PRESSURE) - altitude_zero;
}

void updateObservation() {
  if(observation_idx >= NUM_OBSERVATIONS) {
    return;
  }
  observation_log[observation_idx].ms = millis();
  observation_log[observation_idx].altitude = getAltitude();
  observation_log[observation_idx].acceleration = getAcceleration();
  if(observation_idx == 0) {
    observation_log[observation_idx].velocity = 0;
  }
  else {
    unsigned long int delta_t = observation_log[observation_idx].ms - observation_log[observation_idx - 1].ms;
    observation_log[observation_idx].velocity = observation_log[observation_idx - 1].velocity + observation_log[observation_idx].acceleration * delta_t;
  }
  observation_log[observation_idx].altitude = getAltitude();

  observation_idx++;
}

void setup() {
  // set led on
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  if (!acc.begin(0x18) || !bmp.begin()) {   // change this to 0x19 for alternative i2c address
    while (1) {
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(50);
    }
  }
  acc.setRange(ACC_RANGE);
  zeroAltitude();
  observation_idx = 0;
}

void loop() {
  if(digitalRead(BUTTON) == HIGH) {
    Serial.begin(9600);
    for(int i = 0; i < observation_idx; i++) {
      char buf[FORMAT_OBS_LEN];
      int len = formatObs(buf, observation_log[i]);
      Serial.write(buf, len);
    }
  }
  updateObservation();
  digitalWrite(LED, HIGH);
  delay(25);
  digitalWrite(LED, LOW);
  delay(75);
}

extern "C" {
#include "esp_heap_caps.h"
}

#include <DHT.h>

#define DHTPIN 32      // what pin we're connected to
#define DHTTYPE DHT22  // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);

#include <Adafruit_SCD30.h>
Adafruit_SCD30 scd30;

// State transition matrix F
float** F;

// Observation matrix H
float** H;

// Measurement noise covariance R
float** R;

// Process noise covariance Q
float** Q;

// State covariance matrix P
float** P;

void setupArrays() {
  F = new float*[4];
  H = new float*[2];
  R = new float*[2];
  Q = new float*[4];
  P = new float*[4];

  for(int i = 0; i < 4; i++) {
    F[i] = new float[4];
    Q[i] = new float[4];
    P[i] = new float[4];
    for(int j = 0; j < 4; j++) {
      F[i][j] = (i == j) ? 1 : 0;
      Q[i][j] = (i == j) ? 1 : 0;
      P[i][j] = (i == j) ? 1 : 0;
    }
  }

  for(int i = 0; i < 2; i++) {
    H[i] = new float[4];
    R[i] = new float[2];
    for(int j = 0; j < 4; j++) {
      H[i][j] = (i == j) ? 1 : 0;
    }
    for(int j = 0; j < 2; j++) {
      R[i][j] = (i == j) ? ((i == 0) ? 0.0004 : 4) : 0;
    }
  }
}

void freeArrays() {
  for(int i = 0; i < 4; i++) {
    delete [] F[i];
    delete [] Q[i];
    delete [] P[i];
  }

  for(int i = 0; i < 2; i++) {
    delete [] H[i];
    delete [] R[i];
  }

  delete [] F;
  delete [] H;
  delete [] R;
  delete [] Q;
  delete [] P;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!scd30.begin()) {
    Serial.println("Failed to initialize SCD30 sensor!");
    while (1) {
      delay(10);
    }
  }

  scd30.setMeasurementInterval(2);
  dht.begin();

  setupArrays();

  delay(1000);
}


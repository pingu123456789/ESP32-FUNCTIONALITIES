#include <DHT.h>

#define DHTPIN 32 
#define DHTTYPE DHT22 
#define SAMPLE_SIZE 10 // Change this to the number of readings you want to use for the calculation

DHT dht(DHTPIN, DHTTYPE);
float readings[SAMPLE_SIZE]; 
int readIndex = 0;

void setup() {
  Serial.begin(9600);
  dht.begin();

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    readings[i] = 0;
  }
}

void loop() {
  float temp = dht.readTemperature();

  if (isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  readings[readIndex] = temp;
  readIndex = (readIndex + 1) % SAMPLE_SIZE;

  float sum = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    sum += readings[i];
  }
  float mean = sum / SAMPLE_SIZE;

  float sumOfSquares = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    sumOfSquares += pow((readings[i] - mean), 2);
  }
  float stddev = sqrt(sumOfSquares / SAMPLE_SIZE);

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" *C  ");
  Serial.print("Mean: ");
  Serial.print(mean);
  Serial.print(" *C  ");
  Serial.print("StdDev: ");
  Serial.println(stddev);

  delay(2000); // wait 2 seconds before taking the next reading
}

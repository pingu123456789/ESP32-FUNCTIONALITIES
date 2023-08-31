#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_SCD30.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <Arduino.h>
#include <SensirionI2cScd30.h>
#include <Wire.h>

SensirionI2cScd30 sensor;

#include "DHT.h"
#define DHTTYPE DHT22
#define dht_dpin 32
DHT dht(dht_dpin, DHTTYPE);

QueueHandle_t xQueue;

static char errorMessage[128];
static int16_t error;

// Store the data in a struct
struct sensorData {
  float temperature_DHT22;
  float temperature_SCD30;
  float humidity_SCD30;
  float humidity_DHT22;
  float co2Concentration;
};

void task1(void *pvParameters) {
  while (true) {
    uint32_t startTime1 = xTaskGetTickCount();
    float co2Concentration = 0.0;
    float humidity_SCD30 = 0.0;
    float temperature_SCD30 = 0.0;
    float humidity_DHT22 = 0.0;
    float temperature_DHT22 = 0.0;
    //delay(1000);
    error = sensor.blockingReadMeasurementData(co2Concentration, temperature_SCD30,
                                               humidity_SCD30);
    if (error != NO_ERROR) {
      Serial.print("Error trying to execute blockingReadMeasurementData(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }
    Serial.print("co2Concentration: ");
    Serial.print(co2Concentration);
    Serial.print("\t");
    Serial.print("temperature_SCD30: ");
    Serial.print(temperature_SCD30);
    Serial.print("\t");
    Serial.print("humidity_SCD30: ");
    Serial.print(humidity_SCD30);
    temperature_DHT22 = dht.readTemperature();
    Serial.print("temperature_DHT22: ");
    Serial.print(humidity_DHT22);
    Serial.print("\t");
    humidity_DHT22 = dht.readHumidity();
    Serial.print("humidity_DHT22: ");
    Serial.print(humidity_DHT22);
    Serial.println();

    // Update the values using xQueueSend
    // ...

    sensorData data = { co2Concentration, temperature_SCD30, humidity_SCD30, temperature_DHT22, humidity_DHT22 };

    // Send the data to task2
    xQueueSend(xQueue, &data, portMAX_DELAY);

    uint32_t endTime1 = xTaskGetTickCount();
    uint32_t elapsedTime1 = endTime1 - startTime1;

    Serial.print("Task1 execution time: ");
    Serial.println(elapsedTime1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

void task2(void *pvParameters) {
  // Initialize the Kalman filter variables
  float kalmanGain = 0.0;
  float estimatedMeasurement = 0.0;
  float previousEstimate = 0.0;
  float processNoiseCovariance = 0.1;
  float measurementNoiseCovariance = 1.0;

  while (true) {
    uint32_t startTime2 = xTaskGetTickCount();
    // Receive the data from task1
    struct sensorData data;
    xQueueReceive(xQueue, &data, portMAX_DELAY);

    // Perform Kalman filtering on the temperature_DHT22 data
    estimatedMeasurement = data.temperature_DHT22;
    kalmanGain = processNoiseCovariance / (processNoiseCovariance + measurementNoiseCovariance);
    previousEstimate = estimatedMeasurement;
    estimatedMeasurement = previousEstimate + kalmanGain * (estimatedMeasurement - previousEstimate);
    processNoiseCovariance = (1 - kalmanGain) * processNoiseCovariance;

    // Print the filtered temperature_DHT22 value
    Serial.print("Temperature_DHT22 (filtered): ");
    Serial.println(estimatedMeasurement);

    // Repeat the same process for humidity_DHT22, temperature_SCD30, humidity_SCD30, and CO2 concentration data

    // Calculate the average of the sensor measurements
    /*static float temperaturedht22Sum = 0;
    static float humiditydht22Sum = 0;
    static float temperaturescd30Sum = 0;
    static float humidityscd30Sum = 0;
    static float co2Sum = 0;
    static int count = 0;

    temperaturedht22Sum += data.temperature_DHT22;
    humiditydht22Sum += data.humidity_DHT22;
    temperaturescd30Sum += data.temperature_SCD30;
    humidityscd30Sum += data.humidity_SCD30;
    co2Sum += data.co2Concentration;
    count++;

    float temperatureAveragedht22 = temperaturedht22Sum / count;
    float humidityAveragedht22 = humiditydht22Sum / count;
    float temperatureAveragescd30 = temperaturescd30Sum / count;
    float humidityAveragescd30 = humidityscd30Sum / count;
    float co2Average = co2Sum / count;*/

    // Print the average values
    /*Serial.print("Temperature averagedht22: ");
    Serial.println(temperatureAveragedht22);
    Serial.print("Humidity averagedht22: ");
    Serial.println(humidityAveragedht22);
    Serial.print("Temperature averagescd30: ");
    Serial.println(temperatureAveragescd30);
    Serial.print("Humidity averagescd30: ");
    Serial.println(humidityAveragescd30);
    Serial.print("CO2 average: ");
    Serial.println(co2Average);*/
  
    // Update the WiFi input values using xQueueSend
    // ...
    uint32_t endTime2 = xTaskGetTickCount();
    uint32_t elapsedTime2 = endTime2 - startTime2;

    Serial.print("Task2 execution time: ");
    Serial.println(elapsedTime2);
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for 100 milliseconds
  }
}

void task3(void *pvParameters) {
  while (true) {
    // Get the latest sensor measurements from the queue
    // ...

    if (WiFi.status() == WL_CONNECTED) {
      // Send the sensor measurements to a server
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  dht.begin();
  delay(1000);
  Wire.begin();
  sensor.begin(Wire, SCD30_I2C_ADDR_61);

  sensor.stopPeriodicMeasurement();
  sensor.softReset();
  delay(1000);
  uint8_t major = 0;
  uint8_t minor = 0;
  error = sensor.readFirmwareVersion(major, minor);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute readFirmwareVersion(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
  Serial.print("firmware version major: ");
  Serial.print(major);
  Serial.print("\t");
  Serial.print("minor: ");
  Serial.print(minor);
  Serial.println();
  error = sensor.startPeriodicMeasurement(0);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
  xQueue = xQueueCreate(10, sizeof(struct sensorData));

  WiFi.begin();

  xTaskCreate(task1, "Task1", 2048, NULL, 3, NULL);
  xTaskCreate(task2, "Task2", 2048, NULL, 2, NULL);
  xTaskCreate(task3, "Task3", 2048, NULL, 1, NULL);
}

void loop() {
  // Nothing to be done in the loop as the tasks are running in parallel
}
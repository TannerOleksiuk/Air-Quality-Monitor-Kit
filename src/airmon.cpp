/*
 * Project env-sensor-kit
 * Description: Modified Basic Tutorial project for the Particle Envrionmental Sensor Kit
 * Modifications Made:
 * - Added battery charging detection and approximation of SoC based on voltage reading.
 * - Flipped display on startup.
 * - Added more data to be sent, including raw sensor values from aq sensor
 * Modified by: Tanner Oleksiuk.
 * Original Author: Brandon Satrom <brandon@particle.io>
 * Date: 09/16/2019
 */
#include "Particle.h"

SYSTEM_THREAD(ENABLED);

#include "Air_Quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "SeeedOLED.h"
#include "JsonParserGeneratorRK.h"
#include <math.h>

#define AQS_PIN A2
#define DUST_SENSOR_PIN D4
#define SENSOR_READING_INTERVAL 60000
#define VER_NUM "1.00.01"
#define CLEAR_LINE "                "

AirQualitySensor aqSensor(AQS_PIN);
Adafruit_BME280 bme;

unsigned long lastInterval;
unsigned long lowpulseoccupancy = 0;
unsigned long last_lpo = 0;
unsigned long duration;

float ratio = 0;
float concentration = 0;

int getBMEValues(float &temp, float &humidity, float &pressure);
void getDustSensorReadings();
String getAirQuality();
void createEventPayload(float temp, float humidity, float pressure, String airQuality, float current_v, float delta_v);
void updateDisplay(int temp, int humidity, int pressure, String airQuality, float voltage, int batt_percent, bool charging);
int volt_to_percent(float voltage);

void setup()
{
  Serial.begin(9600);
  delay(50);

  // Configure the dust sensor pin as an input
  pinMode(DUST_SENSOR_PIN, INPUT);
  pinMode(BATT, AN_INPUT);
  pinMode(CHG, INPUT);
  pinMode(PWR, INPUT);


  if (aqSensor.init())
  {
    Serial.println("Air Quality Sensor ready.");
  }
  else
  {
    Serial.println("Air Quality Sensor ERROR!");
  }

  Wire.begin();
  SeeedOled.init();

  // Rotate Screen
  SeeedOled.sendCommand(0xC8);
  SeeedOled.sendCommand(0xA1);
  SeeedOled.setBrightness(0x00);
  SeeedOled.clearDisplay();
  SeeedOled.setNormalDisplay();
  SeeedOled.setPageMode();
  SeeedOled.setTextXY(0, 0);

  SeeedOled.setTextXY(2, 0);
  SeeedOled.putString("Particle");
  SeeedOled.setTextXY(3, 0);
  SeeedOled.putString("Air Quality");
  SeeedOled.setTextXY(4, 0);
  SeeedOled.putString("Monitor");
  SeeedOled.setTextXY(7,0);
  SeeedOled.putString("VER. ");
  SeeedOled.putString(VER_NUM);

  if (bme.begin())
  {
    Serial.println("BME280 Sensor ready.");
  }
  else
  {
    Serial.println("BME280 Sensor ERROR!");
  }

  lastInterval = millis();
}

void loop()
{
  float temp, pressure, humidity = 0;
  int batt_percent = 0;
  float voltage = 0.0;
  float current_aq_voltage = 0;
  float last_aq_voltage = 0;
  float delta_aq_voltage = 0;
  bool charging = false;

  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if ((millis() - lastInterval) > SENSOR_READING_INTERVAL)
  {
    voltage = analogRead(BATT) * 0.0011224;
    batt_percent = volt_to_percent(voltage);
    charging = (digitalRead(PWR) && !digitalRead(CHG));
    String quality = getAirQuality();
    last_aq_voltage = current_aq_voltage;
    current_aq_voltage = aqSensor.getValue();
    delta_aq_voltage = current_aq_voltage - last_aq_voltage;

    Serial.printlnf("Air Quality: %s", quality.c_str());

    getBMEValues(temp, pressure, humidity);
    Serial.printlnf("Temp: %f", temp);
    Serial.printlnf("Pressure: %f", pressure);
    Serial.printlnf("Humidity: %f", humidity);

    getDustSensorReadings();
    // Display uses integers to save on char space
    updateDisplay((int)temp, (int)humidity, (int)pressure, quality, voltage, batt_percent, charging);

    createEventPayload(temp, humidity, pressure, quality, current_aq_voltage, delta_aq_voltage);

    lowpulseoccupancy = 0;
    lastInterval = millis();
  }
}

int volt_to_percent(float voltage)
{
  if(voltage < 3.1)
  {
    return 0;
  }
  if(voltage > 4.1)
  {
    return 100;
  }
  return (int)((voltage-3.1)*100);
}

String getAirQuality()
{
  int quality = aqSensor.slope();
  String qual = "None";

  if (quality == AirQualitySensor::FORCE_SIGNAL)
  {
    qual = "Danger";
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION)
  {
    qual = "High Pollution";
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION)
  {
    qual = "Low Pollution";
  }
  else if (quality == AirQualitySensor::FRESH_AIR)
  {
    qual = "Fresh Air";
  }

  return qual;
}

int getBMEValues(float &temp, float &pressure, float &humidity)
{
  temp = bme.readTemperature();
  pressure = (bme.readPressure() / 100.0F);
  humidity = bme.readHumidity();

  return 1;
}

void getDustSensorReadings()
{
  // This particular dust sensor returns 0s often, so let's filter them out by making sure we only
  // capture and use non-zero LPO values for our calculations once we get a good reading.
  if (lowpulseoccupancy == 0)
  {
    lowpulseoccupancy = last_lpo;
  }
  else
  {
    // Store previous LPO incase of 0 value
    last_lpo = lowpulseoccupancy;
  }

  ratio = lowpulseoccupancy / (SENSOR_READING_INTERVAL * 10.0);                   // Integer percentage 0=>100
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve

  Serial.printlnf("LPO: %lu", lowpulseoccupancy);
  Serial.printlnf("Ratio: %f%%", ratio);
  Serial.printlnf("Concentration: %f pcs/L", concentration);
}

void createEventPayload(float temp, float humidity, float pressure, String airQuality, float current_v, float delta_v)
{
  // Equation results in LPO of 0 coming out to 0.62, just set it to 0.
  if(concentration == 0.62)
  {
    concentration = 0;
  }
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("ver", VER_NUM);
    jw.insertKeyValue("temp", temp);
    jw.insertKeyValue("humidity", humidity);
    jw.insertKeyValue("pressure", pressure);
    jw.insertKeyValue("air-quality", airQuality);
    jw.insertKeyValue("aq_voltage", current_v);
    jw.insertKeyValue("aq_delta_v", delta_v);
    jw.insertKeyValue("dust-lpo", lowpulseoccupancy);
    jw.insertKeyValue("dust-ratio", ratio);
    jw.insertKeyValue("dust-concentration", concentration);
  }

  Particle.publish("env-vals", jw.getBuffer(), PRIVATE);
}

void updateDisplay(int temp, int humidity, int pressure, String airQuality, float voltage, int batt_percent, bool charging)
{
  SeeedOled.clearDisplay();

  SeeedOled.setTextXY(0, 3);
  SeeedOled.putString(airQuality);

  SeeedOled.setTextXY(2, 0);
  SeeedOled.putString("Temp: ");
  SeeedOled.putNumber(temp);
  SeeedOled.putString("C");

  SeeedOled.setTextXY(3, 0);
  SeeedOled.putString("Humidity: ");
  SeeedOled.putNumber(humidity);
  SeeedOled.putString("%");

  SeeedOled.setTextXY(4, 0);
  SeeedOled.putString("Press: ");
  SeeedOled.putNumber(pressure);
  SeeedOled.putString(" hPa");

  if (concentration > 1)
  {
    SeeedOled.setTextXY(5, 0);
    SeeedOled.putString("Dust: ");
    SeeedOled.putNumber(concentration); // Will cast our float to an int to make it more compact
    SeeedOled.putString(" pcs/L");
  }

  // If charging, or running off battery power, print the following:
  if(charging || !digitalRead(PWR))
  {
    SeeedOled.setTextXY(6, 0);
    SeeedOled.putString("Bat:");
    SeeedOled.putNumber(batt_percent);
    SeeedOled.putString("% (");
    SeeedOled.putFloat(voltage);
    SeeedOled.putString("V)");

    SeeedOled.setTextXY(7,0);
    SeeedOled.putString("Charging: ");
    charging ? SeeedOled.putString("True") : SeeedOled.putString("False");
  }
  else
  {
    // Clear those lines
    SeeedOled.setTextXY(6, 0);
    SeeedOled.putString(CLEAR_LINE);
    SeeedOled.setTextXY(7, 0);
    SeeedOled.putString(CLEAR_LINE);
  }
}
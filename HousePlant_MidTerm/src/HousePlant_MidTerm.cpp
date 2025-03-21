/*
 * Project operationHousePlant
 * Author: Victor Roman
 * Date: 03.19.25
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_BME280.h"
#include "Adafruit_SSD1306.h"
#include "Air_Quality_Sensor.h"
#include "IoTClassroom_CNM.h"
#include "credentials.h"

String dateTime, timeOnly;
Adafruit_BME280 bme;
const int PUMP = D16;
const int AQSENOR = D12;
const int MS = A2;

unsigned int _timerStart, _timerTarget;
unsigned int currentTime;
unsigned int lastSecond, lastTime;
int timer;
int value;
int buttonPress;
int brightness;
int moisture;
int inputValue;
int status;
float subValue, pubValue;
float tempC, tempF;
float pressPA;
float humidRH;
float pressureHg;

const int OLED_RESET = -1;
Adafruit_SSD1306 display(OLED_RESET);

IoTTimer timer1;

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();

/************ Global State (you don't need to change this!) ***   ***************/
TCPClient TheClient;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
AirQualitySensor airSensor(AQSENOR);

/****************************** Feeds ***************************************/
// Setup Feeds to publish or subscribe
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Subscribe subButtonPress = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ButtonPress");
Adafruit_MQTT_Publish pubHumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish pubSoilMoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/SoilMoisture");
Adafruit_MQTT_Publish pubTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temp");
Adafruit_MQTT_Publish pubAirQuality = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality");

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
//SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'

// setup() runs once, when the device is first turned on
void setup()
{
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  // OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // BME
  status = bme.begin(0x76);
  if (status == false)
  {
    Serial.printf("BME280 at address 0x%02X failed to start", 0x76);
  }
  display.display();
  display.clearDisplay();
  display.display();

  // Setup MQTT subscription
  mqtt.subscribe(&subButtonPress);

  // PINMODE
  pinMode(MS, INPUT);
  pinMode(PUMP, OUTPUT);

  // TIME SET UP
  Time.zone(-7);
  Particle.syncTime();
  Serial.printf("Made it");
  delay(6000);


}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  MQTT_connect();
  MQTT_ping();

  currentTime = millis();
  inputValue = analogRead(MS);
  Serial.printf("Moisure lvl: %f\n", inputValue);

  // BME INPUT READINGS
  humidRH = bme.readHumidity();
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  tempF = tempC * (9.0 / 5.0) + 32.0;
  pressureHg = pressPA / 3386.39;

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100)))
  {
    // PRINTING READING TO DASHBOARD
    if ((millis() - lastTime > 10000))
    {
      if (mqtt.Update())
      {
        pubHumidity.publish(humidRH);
        pubSoilMoisture.publish(inputValue);
        pubTemp.publish(tempC);
        // pubAirQuality.publish(AirQuality);
      }
      lastTime = millis();
    }
  }
  // AQSENSOR LOOP
  int quality = airSensor.slope();
  value = airSensor.getValue();

  Serial.print("AQSensor value: ");
  Serial.println(airSensor.getValue());

  if (quality == AirQualitySensor::FORCE_SIGNAL)
  {
    Serial.printf("High pollution! Force signal active.");
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION)
  {
    Serial.println("High pollution!");
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION)
  {
    Serial.println("Low pollution!");
  }
  else if (quality == AirQualitySensor::FRESH_AIR)
  {
    Serial.println("Fresh air.");
  }

  // delay(1000);

  // TIME LOOP
  dateTime = Time.timeStr();
  timeOnly = dateTime.substring(11, 19);

  //Moisture
  moisture = analogRead(MS);
  if(moisture > 2500){
    timer1.startTimer(100);
    while(!timer1.isTimerReady()){
      digitalWrite(PUMP, HIGH);
    }
    digitalWrite(PUMP, LOW);
  }

  // BME PRINT TO MONITOR
  if ((currentTime - lastSecond) > 500)
  {
    lastSecond = millis();
    Serial.printf("Humidity =%f\n", humidRH);
    Serial.printf("TempF: %f F\n", tempF);
    Serial.printf("\nMoisture lvl: %i", inputValue);
  }

  // BME && MS READINGS PRINT TO OLED
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.printf(timeOnly);
  display.printf("T:%f\n H:%f\n MS:%i\n", tempF, humidRH, inputValue);
  // display.setTextColor(BLACK, WHITE);
  display.display();
  display.clearDisplay();
}

void MQTT_connect()
{
  int8_t ret;

  // Return if already connected.
  if (mqtt.connected())
  {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0)
  { // connect will return 0 for connected
    Serial.printf("Error Code %s\n", mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds...\n");
    mqtt.disconnect();
    delay(5000); // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping()
{
  static unsigned int last;
  bool pingStatus;

  if ((millis() - last) > 120000)
  {
    Serial.printf("Pinging MQTT \n");
    pingStatus = mqtt.ping();
    if (!pingStatus)
    {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }
  return pingStatus;
}
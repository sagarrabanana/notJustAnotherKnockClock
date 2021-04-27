/*
  Rui Santos
  Complete project details at Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-http-get-open-weather-map-thingspeak-arduino/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// ejemplo para MAX31855-> https://github.com/enjoyneering/MAX31855/blob/master/examples/MAX31855_ESP8266_hw_SPI_Demo/MAX31855_ESP8266_hw_SPI_Demo.ino

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <Arduino_JSON.h>
//---https://lastminuteengineers.com/esp8266-ntp-server-date-time-tutorial/
#include <NTPClient.h>
#include <WiFiUdp.h>

//--------------- PID
#include <SPI.h>
#include <MAX31855.h>
#include <PID_v1.h>
//------------

const long utcOffsetInSeconds = 3600;
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

//-----------------------https://randomnerdtutorials.com/esp8266-nodemcu-http-get-open-weather-map-thingspeak-arduino/

const char* ssid = "yourSSID";
const char* password = "yourPassword";

// Your Domain name with URL path or IP address with path
String openWeatherMapApiKey = "03eded784c6d1a4b61f8a6e6fa928a8f";


// Replace with your country code and city
String city = "Leioa";
String countryCode = "ES";

// THE DEFAULT TIMER IS SET TO 10 SECONDS FOR TESTING PURPOSES
// For a final application, check the API call limits per hour/minute to avoid getting blocked/banned
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;

String jsonBuffer;
//---------------------
const int piezo = A0;
int cont = 0;
int umbral = 20;
float umbralTime = 300;
long timeUlt = 0;
int knock;

//-----------------PID
int32_t rawData = 0;
double ambiente;

int alcanzado = 0;
uint8_t solenoidePin = D8;
long cuentaTiempo;

uint8_t PIN_OUTPUT_COLD = D2;
uint8_t PIN_OUTPUT_HOT = D1;

uint8_t fanPin = D7;

//Define Variables we'll be connecting to
double Setpoint, Input, Output, tempObj;
double SetpointPre;

//Specify the links and initial tuning parameters
double cKp = 200, cKi = 20, cKd = 30;
double hKp = 20, hKi = 8, hKd = 600;
PID coldPID(&Input, &Output, &Setpoint, cKp, cKi, cKd, REVERSE);//DIRECT
PID hotPID(&Input, &Output, &Setpoint, hKp, hKi, hKd, DIRECT);//DIRECT

MAX31855 myMAX31855(D4);   //chip select pin, ESP8266 fails to BOOT/FLASH if D4 is LOW

//--------------------

void setup() {
  WiFi.persistent(false);  //disable saving wifi config into SDK flash area

  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
  myMAX31855.begin();
  while (myMAX31855.getChipID() != MAX31855_ID)
  {
    Serial.println(F("MAX6675 error")); //(F()) saves string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("MAX6675 OK"));
  while (myMAX31855.detectThermocouple() != MAX31855_THERMOCOUPLE_OK)
  {
    switch (myMAX31855.detectThermocouple())
    {
      case MAX31855_THERMOCOUPLE_SHORT_TO_VCC:
        Serial.println(F("Thermocouple short to VCC"));
        break;

      case MAX31855_THERMOCOUPLE_SHORT_TO_GND:
        Serial.println(F("Thermocouple short to GND"));
        break;

      case MAX31855_THERMOCOUPLE_NOT_CONNECTED:
        Serial.println(F("Thermocouple not connected"));
        break;

      case MAX31855_THERMOCOUPLE_UNKNOWN:
        Serial.println(F("Thermocouple unknown error"));
        break;

      case MAX31855_THERMOCOUPLE_READ_FAIL:
        Serial.println(F("Thermocouple read error, check chip & spi cable"));
        break;
    }
    delay(3000);
  }
  coldPID.SetMode(AUTOMATIC);
  coldPID.SetOutputLimits(0, 1023);//con esto modifico el limite del output del PID pasandolo de 255 a 1023 para adecuarlo a la resolucion del NodeMCU

  hotPID.SetMode(AUTOMATIC);
  hotPID.SetOutputLimits(0, 1023);//con esto modifico el limite del output del PID pasandolo de 255 a 1023 para adecuarlo a la resolucion del NodeMCU

  pinMode(solenoidePin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);
}

void loop() {



  // Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED) {
    knock = analogRead(piezo);
    Serial.print("knock Val: ");
    Serial.println(knock);
    if ((knock > umbral) and cont == 0)
    {
      timeUlt = millis();
      cont++;
      delay(50);
    }
    else if ((knock > umbral) and (cont > 0) and (millis() - timeUlt) < umbralTime)
    {
      timeUlt = millis();
      cont++;
      delay(80);
    }
    else if ((millis() - timeUlt) > umbralTime and (cont > 0))
    {
      Serial.print("numero de golpes: ");
      Serial.println(cont);
      if (cont == 2)
      {
        timeClient.update();
        int hora = timeClient.getHours() + 1;
        int minutos = timeClient.getMinutes();
        Serial.print(hora);
        Serial.print(":");
        Serial.println(minutos);
        if (hora > 12)
        {
          for (int i = 1; i <= (hora - 12); i++)
          {
            digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(200);
          }
        }
        else if (hora <= 12)
        {
          for (int i = 1; i <= hora; i++)
          {
            digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(200);
          }
        }
        delay(500);
        for (int i = 1; i <= (int)(minutos / 15); i++)
        {
          digitalWrite(solenoidePin, HIGH);
          delay(100);
          digitalWrite(solenoidePin, LOW);
          delay(200);
        }
      }
      else if (cont == 3)
      {
        tempObj = getTemp();
        //setTemp((int)tempObj);
        SetpointPre = (int)tempObj;
        digitalWrite(fanPin, HIGH);
        //---------------
        while (true)
        {

          rawData = myMAX31855.readRawData();
          ambiente = myMAX31855.getColdJunctionTemperature(rawData);
          if (ambiente > SetpointPre)
          {
            Setpoint = SetpointPre;
            Input = myMAX31855.getTemperature(rawData);
            coldPID.Compute();
            analogWrite(PIN_OUTPUT_COLD, Output);
            Serial.println("enfriamos");
          }
          else if (ambiente < SetpointPre)
          {
            Setpoint = SetpointPre;
            Input = myMAX31855.getTemperature(rawData);
            hotPID.Compute();
            analogWrite(PIN_OUTPUT_HOT, Output);
            Serial.println("calentamos");
          }
          Serial.print("input: ");
          Serial.println(Input);
          Serial.print("output: ");
          Serial.println(Output);
          Serial.print("setpoint: ");
          Serial.println(Setpoint);
          Serial.print("ambiente: ");
          Serial.println(ambiente);
          //delay(5000);
          if ((Input == Setpoint) && (alcanzado == 0))
          {
            Serial.print("Alcanzado 1 -----------------------------------------------------");
           /* digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(100);
            digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(100);*/
            alcanzado = 1;
            cuentaTiempo = millis();
          }
          if ((alcanzado == 1) && ((millis() - cuentaTiempo) > 20000))
          {
            //alcanzado = 0;
            Serial.print("Alcanzado  ---------------------------2--------------------------");
            Serial.println("fuera");
            digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(100);
            digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(100);
            alcanzado = 2;
            /*
            digitalWrite(PIN_OUTPUT_COLD, LOW);
            digitalWrite(PIN_OUTPUT_HOT, LOW);
            digitalWrite(fanPin, LOW);
            break;*/

          }
           if ((alcanzado == 2) && ((millis() - cuentaTiempo) > 60000))
          {
            
            Serial.print("Alcanzado  ----------------------------------------------------- 3");
            Serial.println("fuera");
            digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(100);
            digitalWrite(solenoidePin, HIGH);
            delay(100);
            digitalWrite(solenoidePin, LOW);
            delay(100);
            digitalWrite(PIN_OUTPUT_COLD, LOW);
            digitalWrite(PIN_OUTPUT_HOT, LOW);
            digitalWrite(fanPin, LOW);
            alcanzado = 0;
            break;

          }
        }
      }
      cont = 0;
    }

  }

  else {
    Serial.println("WiFi Disconnected");
    delay(500);
    WiFi.begin(ssid, password);
    Serial.println("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
  }
}

String httpGETRequest(const char* serverName) {
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

double getTemp()
{
  String serverPath = "http://api.openweathermap.org/data/2.5/weather?q=" + city + "," + countryCode + "&APPID=" + openWeatherMapApiKey + "&units=metric";

  jsonBuffer = httpGETRequest(serverPath.c_str());
  //Serial.println(jsonBuffer);
  JSONVar myObject = JSON.parse(jsonBuffer);

  Serial.print("datatype: ");
  Serial.println(JSON.typeof(myObject["main"]["temp"]));

  double temp = myObject["main"]["temp"];

  Serial.print("int temp: ");
  Serial.println(temp);
  return temp;

}
/*
  void setTemp(double Setpoint)
  {

  while (true)
  {

    rawData = myMAX31855.readRawData();
    ambiente = myMAX31855.getColdJunctionTemperature(rawData);
    if (ambiente > Setpoint)
    {
      Input = myMAX31855.getTemperature(rawData);
      coldPID.Compute();
      analogWrite(PIN_OUTPUT_COLD, Output);
      Serial.println("enfriamos");
    }
    else if (ambiente < Setpoint)
    {
      Input = myMAX31855.getTemperature(rawData);
      hotPID.Compute();
      analogWrite(PIN_OUTPUT_HOT, Output);
      Serial.println("calentamos");
    }
    Serial.print("input: ");
    Serial.println(Input);
    Serial.print("output: ");
    Serial.println(Output);
    Serial.print("setpoint: ");
    Serial.println(Setpoint);
    Serial.print("ambiente: ");
    Serial.println(ambiente);
    //delay(5000);
    if ((Input == Setpoint) && (alcanzado == 0))
    {
      digitalWrite(solenoidePin, HIGH);
      delay(100);
      digitalWrite(solenoidePin, LOW);
      delay(100);
      digitalWrite(solenoidePin, HIGH);
      delay(100);
      digitalWrite(solenoidePin, LOW);
      delay(100);
      alcanzado = 1;
      cuentaTiempo = millis();
    }
    if ((alcanzado == 1) && ((millis() - cuentaTiempo) > 30000))
    {
      alcanzado = 0;
      Serial.println("fuera");
      digitalWrite(solenoidePin, HIGH);
      delay(100);
      digitalWrite(solenoidePin, LOW);
      delay(100);
      digitalWrite(solenoidePin, HIGH);
      delay(100);
      digitalWrite(solenoidePin, LOW);
      delay(100);
      break;
    }
  }
  }

*/

void setAlarm() //Esto esta sin terminar! nos quedamos aqui
{
  delay(100);
  digitalWrite(solenoidePin, HIGH);
  delay(50);
  digitalWrite(solenoidePin, LOW);

}

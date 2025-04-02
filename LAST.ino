#include <TinyGPSPlus.h>
#include <UniversalTelegramBot.h>
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
#define LCD_ADDRESS 0x27

// Set the LCD dimensions (columns x rows)
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define engineonoff V7

char auth[] = "_PPmW9JzUtU1gO6jD71SHla1tdLkIiEu"; //Ambil dari email atau project dalam Blynk
char ssid[] = ""; //namE wifi
char pass[] = ""; //password wifi
//char server[] = "139.59.224.74";  // IP for your Local Server
//int port = 8080;
unsigned int move_index = 1;
const int relayPin = 12;  // Change this to the pin connected to the relay
// Pins connected to the green and red LEDs
const int greenLedPin = 13;  // Change this to the pin connected to the green \\\\\LED CODE IMAN AFIQ HENSEM DI SINI
const int redLedPin = 15;    // Change this to the pin connected to the red LED
const int alcoholSensorPin = 34;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
BlynkTimer timer;
TinyGPSPlus gps;
WidgetMap myMap(V6);

float latitude;  
float longitude;    
float velocity;

String functiontrig = "";

BLYNK_CONNECTED() {
  Blynk.syncVirtual(engineonoff);
}

BLYNK_WRITE(engineonoff) // Executes when the value of virtual pin 0 changes
{
  if (param.asInt() == 1)
  {
    functiontrig="ON";
  }
  else
  {
    functiontrig="OFF";
  }
}

void setup() {
  delay(2000);
  pinMode(relayPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  Serial.begin(9600);
  Serial2.begin(9600);
  Blynk.begin(auth, ssid, pass, IPAddress(139,59,224,74), 8080);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("LOaDing.");
  delay(2000);
  lcd.clear();
  lcd.print("LOaDing..");
  delay(2000);
  lcd.clear();
  lcd.print("LOaDing...");
  delay(2000);
  lcd.clear();
  lcd.print("LOaDing....");
  delay(2000);
  lcd.clear();
  lcd.print("LOaDing.....");
  delay(2000);
  lcd.clear();
  lcd.print("GREETINGS SIR =)");
  delay(5000);

}


void loop() {
  //updateSerial();

  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }

    Blynk.run();
    timer.run();
}


void displayInfo()
{

  if (gps.location.isValid()){

    latitude = (gps.location.lat());     //Storing the Lat. and Lon.
    longitude = (gps.location.lng());
    velocity = gps.speed.kmph();         //get velocity
  int sensorValue = analogRead(alcoholSensorPin);
  float voltage = (sensorValue * 5.0) / 511.5;
  float thresholdVoltage = 35.0;
  String sendblynklev, engine;


  Serial.print("ALCOHOL VALUE:");
  Serial.println(voltage);
  Serial.println("--------------------------------");
    Serial.print("LATITUDE:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONGITUDE: ");
    Serial.println(longitude, 6);
    Serial.print("SPEED: ");
    Serial.print(velocity);

   delay(1000);
  // Ch5ck if the push-button is pressed (active LOW)
    String locationURL = "Hello buddies, We are Drunk Emergency Service. Right now you're drunk. Please wait the emergency coming. The your current location is here : https://www.google.com/maps/place/" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
     //Serial.println(locationURL);
  if (functiontrig=="ON"){

    lcd.clear();
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(relayPin, LOW);
    lcd.print("ENGINE ON");

    // Wait for a short debounce time to avoid rapid toggling due to noise
    delay(50);

  if (voltage > thresholdVoltage) {
    // Turn on the RED LED and the relay
    lcd.clear();
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(relayPin, HIGH);
    lcd.print("ALCOHOL DETECTED");
    sendblynklev = "DRUNK";
    engine = "ENGINE LOCK";
    Blynk.notify("You're Drunk. Tap to see the location");
    Blynk.email("irfanraziq2910gmail.com","DRUNK ALERT!", String(locationURL));
    // Wait for a short debounce time to avoid rapid toggling due to noise
    delay(50);
  }}

  else if(functiontrig=="OFF"){

    lcd.clear();
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(relayPin, LOW);
    lcd.print("ENGINE OFF");
    // Wait for a short debounce time to avoid rapid toggling due to noise
    delay(50);

  if (voltage < thresholdVoltage){

    lcd.clear();
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(relayPin, LOW);
    lcd.print("READY TO DRIVE");
    sendblynklev = "NORMAL";
    engine = "READY DRIVE";
    // Wait for a short debounce time to avoid rapid toggling due to noise
    delay(50);
  }}

  if (functiontrig=="ON"){

Blynk.virtualWrite(V0, voltage);
Blynk.virtualWrite(V1, sendblynklev);
Blynk.virtualWrite(V2, engine);
Blynk.virtualWrite(V4, String(latitude, 6));
Blynk.virtualWrite(V5, String(longitude, 6));
Blynk.virtualWrite(V3, velocity);
myMap.location(move_index, latitude, longitude, "GPS_Location");

  }
  else if(functiontrig=="OFF"){

float a = 0.00;

Blynk.virtualWrite(V0, a);
Blynk.virtualWrite(V1, "OFF");
Blynk.virtualWrite(V2, "OFF");
Blynk.virtualWrite(V4, "OFF");
Blynk.virtualWrite(V5, "OFF");
Blynk.virtualWrite(V3, "OFF");
myMap.location(move_index, "0", "0", "GPS_Location");

  }

   // String locationURL = "https://www.google.com/maps/place/" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
     //Serial.println(locationURL);
     //delay(1000);
  }  
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print("");
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }

  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}
#define TINY_GSM_MODEM_SIM800
#include <ArduinoJson.h>
#include <Wire.h> //I2C needed for sensors
#include <Adafruit_MPL3115A2.h>
#include <WiFi.h>
#include <base64.h>
#include "Base64.h"
#include <Update.h>
#include <TinyGsmClient.h>
#include "FS.h"
#include "SPIFFS.h"

#include "HardwareSerial.h"
HardwareSerial Serial3(1);
// Increase RX buffer
#define TINY_GSM_RX_BUFFER 1030
const char apn[] = "airtelgprs.com";
//#define Serial2  Serial2

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(Serial2, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(Serial2);
#endif

TinyGsmClient client(modem);

const char server[] = "mspo-development.s3.ap-south-1.amazonaws.com";
const int port = 80;
const char resource[] = "/solimet/solimet_proplus_esp32_v4.ino.esp32da.bin"; //bin file

uint32_t knownFileSize = 2000; // In case server does not send it

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600        /* Time ESP32 will go to sleep (in seconds) */

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
// SI7021 I2C address is 0x40(64)
#define si7021Addr 0x40

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


// digital I/O pins
const byte WSPEED = 12;
const byte RAIN = 13;
const byte STAT1 = 14;
const byte STAT2 = 27;

// analog I/O pins
const byte REFERENCE_3V3 = 26;
const byte LIGHT = 25;
const byte BATT = 35;
const byte WDIR = 34;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average


#define WIND_DIR_AVG_SIZE 120
int winddiravg[WIND_DIR_AVG_SIZE]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float humidity = 0; // [%]
float tempc = 0; // [temperature F]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile float dailyrainin = 0; // [rain inches so far today in local time]
//float baromin = 30.03;// [barom in] - It's hard to calculate baromin locally, do this in the agent
float pressure = 0;
//float dewptf; // [dewpoint F] - It's hard to calculate dewpoint locally, do this in the agent


float batt_lvl = 11.8; //[analog value from 0 to 1023]
float light_lvl = 455; //[analog value from 0 to 1023]

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;
String timeStamp;
String mac;
long sentdata;
long sentavg;

float avgwindspeedmph=0.0;
float windavg[1500];
float count = 1;

float avgpressure=0;
float pres[1500];
float count1 = 1;

float avgtempc=0;
float temperature[1500];
float count2 = 1;

float avghumidity=0;
float hum[1500];
float count3 = 1;

float mintemp = 100;
float maxtemp = 0;

float minpressure = 1100;
float maxpressure = 0;

float minhumidity = 100;
float maxhumidity = 0;

String img;
String image;

float ec=0.0, soil_temperature=0.0, soil_moisture=0.0, ph=0.0, nitrogen=0.0, phosphorous=0.0, potassium=0.0, salinity=0.0; 

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ(void)
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
    raintime = millis(); // grab current time
    raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
    {
        dailyrainin += 0.011; //Each dump is 0.011" of water
        //rainHour[minutes] += 0.011; //Increase this minute's amount of rain
        rainin += 0.011;
        rainlast = raintime; // set up for next event
    }
}

void wspeedIRQ(void)  //IRAM_ATTR
// Activated by the magnet in the anemometget_wind_direction()er (2 ticks per rotation), attached to input D3
{
    if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
    {
        lastWindIRQ = millis(); //Grab the current time
        windClicks++; //There is 1.492MPH for each click per second.
    }
}


void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 23, 22);
    Serial.println("Weather Shield Example");
    Wire.begin(19, 18);   ////I2C_SDA, I2C_SCL
    Wire.beginTransmission(si7021Addr);
    Wire.endTransmission();
    delay(300);
    //Serial.println(resource);
    pinMode(STAT1, OUTPUT); //Status LED Blue
    pinMode(STAT2, OUTPUT); //Status LED Green
    
    pinMode(REFERENCE_3V3, INPUT);
    pinMode(LIGHT, INPUT);
    pinMode(BATT, INPUT);
    pinMode(WDIR, INPUT);

    seconds = 0;
    lastSecond = millis();

//     attach external interrupt pins to IRQ functions
    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, RISING);
    //delay(100);
    
    pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor
    attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
    //delay(100);
    
    // turn on interrupts
    //interrupts();

    // GPRS baud rate
    //Serial2.begin(9600);
    Serial2.begin(115200, SERIAL_8N1, 16, 17); 
    
    TaskHandle_t taskHandle;
    xTaskCreatePinnedToCore(
      asyncTask,         // Task function
      "asyncTask",       // Task name
      10000,             // Stack size (in bytes)
      NULL,              // Task parameter
      1,                 // Task priority
      &taskHandle,       // Task handle
      1                  // Core to run the task on (0 or 1)
    );
    
    Serial.println("Weather Shield online!");
    mac=WiFi.macAddress();
    //Serial.println(mac);
    delay(5000);

    if (!SPIFFS.begin(true))
    {
         Serial.println("SPIFFS Mount Failed");
        return;
    }
    SPIFFS.format();
    listDir(SPIFFS, "/", 0);

    // Set GSM module baud rate
    
    delay(1000);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    modem.restart();

    String modemInfo = modem.getModemInfo();
    Serial.print("Modem: ");
    Serial.println(modemInfo);
    // Unlock your SIM card with a PIN
    //modem.simUnlock("1234");
    otaupdate();
    delay(1000);
}

void loop()
{
    
    //Keep track of which minute it is
  if(millis() - lastSecond >= 1000)
    {
        digitalWrite(STAT1, HIGH); //Blink stat LED

        lastSecond += 1000;
        sentdata += 1;
//      Serial.print("loop: ");
//      Serial.println(sentdata);
        //Take a speed and direction reading every second for 2 minute average
        if(++seconds_2m > 60) seconds_2m = 0;
        //Calc the wind speed and direction every second for 120 second to get 2 minute average
        float currentSpeed = get_wind_speed();
        windspeedmph = currentSpeed;//update global variable for windspeed when using the printWeather() function
        //float currentSpeed = random(5); //For testing
        int currentDirection = get_wind_direction();

        //Averaging wind data for every 1 hour
        windavg[sentdata] = currentSpeed;
        float temp2=0;
        if (currentSpeed > 0){
        for (int i = 0; i<1501; i++){
            temp2+=windavg[i];
          }
          count++;          
        }
        temp2 /= count;
        avgwindspeedmph = temp2;
        
        windspdavg[seconds_2m] = (int)currentSpeed;
        winddiravg[seconds_2m] = currentDirection;
        //if(seconds_2m % 10 == 0) displayArrays(); //For testing

        //Check to see if this is a gust for the minute
        if(currentSpeed > windgust_10m[minutes_10m])
        {
            windgust_10m[minutes_10m] = currentSpeed;
            windgustdirection_10m[minutes_10m] = currentDirection;
        }

        //Check to see if this is a gust for the day
        if(currentSpeed > windgustmph)
        {
            windgustmph = currentSpeed;
            windgustdir = currentDirection;
        }
        
        if(++seconds > 59)
        {
            seconds = 0;

            if(++minutes > 59) minutes = 0;
            if(++minutes_10m > 9) minutes_10m = 0;

            //rainHour[minutes] = 0; //Zero out this minute's rainfall amount
            windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
        }

        //Report all readings every second
        printWeather();
        digitalWrite(STAT1, LOW); //Turn off stat LED
        delay(500);
        
        //Code to reset Daily Rain for 24 hr
        if((timeStamp>"23:00") and (timeStamp<"23:02")){
            dailyrainin = 0;
            windgustmph = 0;
            delay(60000);
            dailyrainin = rainin; //So avoid the rain data lost during the delay
        }

//        if((sentdata == 90) or (sentdata == 1)){
//           soilData();
//        }

        
        //OTA updates
        if((timeStamp>"15:25") and (timeStamp<"15:30")) {    //Time range to run OTA update:
//         Serial.print("time inside loop: ");
//         Serial.print(timeStamp);    
              if (!SPIFFS.begin(true))
              {
                   Serial.println("SPIFFS Mount Failed");
                  return;
              }
              SPIFFS.format();
              listDir(SPIFFS, "/", 0);
          
              // Set GSM module baud rate
              
              delay(1000);
          
              // Restart takes quite some time
              // To skip it, call init() instead of restart()
              Serial.println("Initializing modem...");
              modem.restart();
          
              String modemInfo = modem.getModemInfo();
              Serial.print("Modem: ");
              Serial.println(modemInfo);
              // Unlock your SIM card with a PIN
              //modem.simUnlock("1234");
                otaupdate();
                delay(1000);
          
        }
               
        //ESP goes to sleep for every 10 minutes after sending data.
//        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
//        Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
//        " Seconds");
//        Serial.println("Going to sleep now");
//        Serial.flush(); 
//        esp_deep_sleep_start();
    }
  //delay(60000);
}

void asyncTask(void* parameter) {
  for (;;) {
    
    while (Serial1.available()){
      img = Serial1.readString();
      img.trim();
      if (img.startsWith("/")){
        image = img;
        }
      else{
        image = img.substring(325);
        }
    }
    
   // vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

//Calculates each of the variables that wunderground is expecting
void calcWeather()
{
    //Calc winddir
    winddir = get_wind_direction();
//    Serial.print("WInd Dir: ");
//    Serial.println(winddir);

    //Calc windspeed
    //windspeedmph = get_wind_speed(); //This is calculated in the main loop on line 185

    //Calc windgustmph
    //Calc windgustdir
    //These are calculated in the main loop

    //Calc windspdmph_avg2m
    float temp = 0;
    for(int i = 0 ; i < 120 ; i++){
        temp += windspdavg[i];
    }
    temp /= 120.0;
    windspdmph_avg2m = temp;
       
    //Calc winddir_avg2m, Wind Direction
    //You can't just take the average. Google "mean of circular quantities" for more info
    //We will use the Mitsuta method because it doesn't require trig functions
    //And because it sounds cool.
    //Based on: http://abelian.org/vlf/bearings.html
    //Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
    long sum = winddiravg[0];
    int D = winddiravg[0];
    for(int i = 1 ; i < WIND_DIR_AVG_SIZE ; i++)
    {
        int delta = winddiravg[i] - D;

        if(delta < -180)
            D += delta + 360;
        else if(delta > 180)
            D += delta - 360;
        else
            D += delta;

        sum += D;
    }
    winddir_avg2m = sum / WIND_DIR_AVG_SIZE;
    if(winddir_avg2m >= 360) winddir_avg2m -= 360;
    if(winddir_avg2m < 0) winddir_avg2m += 360;

    //Calc windgustmph_10m
    //Calc windgustdir_10m
    //Find the largest windgust in the last 10 minutes
    windgustmph_10m = 0;
    windgustdir_10m = 0;
    //Step through the 10 minutes
    for(int i = 0; i < 10 ; i++)
    {
        if(windgust_10m[i] > windgustmph_10m)
        {
            windgustmph_10m = windgust_10m[i];
            windgustdir_10m = windgustdirection_10m[i];
        }
    }

    //Total rainfall for the day is calculated within the interrupt
    //Calculate amount of rainfall for the last 60 minutes
//    rainin = 0;
//    for(int i = 0 ; i < 60 ; i++)
//      rainin += rainHour[i];
//        Serial.print("rainin:");
//        Serial.println(rainin);

    //Calc light level
    light_lvl = get_light_level();

    //Calc battery level
    batt_lvl = get_battery_level();

    //Calc Pressure, Temperature and Humidity.
    detachInterrupt(digitalPinToInterrupt(RAIN));
    tph();
    attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
}

void tph()
{
  //Serial.println(sentdata);
  if (! baro.begin()) 
  {
    //Serial.println("Couldnt find sensor");
    return;
  }

  //Get Pressure reading in Pascals
  pressure = baro.getPressure();
  //Serial.println(pressure);
  //Averaging Pressure data for every 1 hour
  pres[sentdata] = pressure;
  float temp3=0;
  if (pressure > 0){
  for (int i = 0; i<1501; i++){
    //Serial.print(pres[i]);
      temp3+=pres[i];
    }
    count1++;          
  }
  temp3 /= (count1);
  avgpressure = temp3;
//  if(pressure < minpressure){
//    minpressure = pressure;
////    Serial.print("Minimum Pressure:");
////    Serial.println(minpressure);
//  }
//
//  if(pressure > maxpressure){
//    maxpressure = pressure;
////    Serial.print("Maximum Pressure:");
////    Serial.println(maxpressure);
//  }
  
  //Get Temperaturereading in Celcius
  tempc = baro.getTemperature();

  //Averaging temperature data for every 1 hour
  temperature[sentdata] = tempc;
  float temp4=0;
  if (tempc > 0){
  for (int i = 0; i<1501; i++){
//    Serial.print(temperature[i]);
      temp4+=temperature[i];
    }
    count2++;          
  }
  temp4 /= (count2);
  avgtempc = temp4;
  delay(250);

//  if (tempc < mintemp) mintemp = tempc;
//  if (tempc > maxtemp) maxtemp = tempc;

  //Get Humidity reading
  unsigned int data[2];
   
  Wire.beginTransmission(si7021Addr);
  //Send humidity measurement command
  Wire.write(0xF5);
  Wire.endTransmission();
  delay(500);
     
  // Request 2 bytes of data
  Wire.requestFrom(si7021Addr, 2);
  // Read 2 bytes of data to get humidity
  if(Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
     
  // Convert the data
  humidity  = ((data[0] * 256.0) + data[1]);
  humidity = ((125 * humidity) / 65536.0) - 6;

  //Averaging humidity data for every 1 hour
//  hum[sentdata] = humidity;
//  float temp5=0;
//  if (humidity > 0){
//  for (int i = 0; i<1501; i++){
//    //Serial.print(hum[i]);
//      temp5+=hum[i];
//    }
//    count3++;          
//  }
//  temp5 /= (count3);
//  avghumidity = temp5; 
//
  if (humidity > 100) humidity = 100;  //avoid values above 100

//  if (humidity < minhumidity) minhumidity = humidity;
//  if (humidity > maxhumidity) maxhumidity = humidity;
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
    float operatingVoltage = analogRead(REFERENCE_3V3);

    float lightSensor = analogRead(LIGHT);

    operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

    lightSensor = operatingVoltage * lightSensor;

    return(lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
    float operatingVoltage = analogRead(REFERENCE_3V3);

    float rawVoltage = analogRead(BATT);

    operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V

    rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin

    rawVoltage *= 5.3; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

    return(rawVoltage);
}

//Returns the instataneous wind speed
float get_wind_speed()
{
    float deltaTime = millis() - lastWindCheck; //750ms

    deltaTime /= 1000.0; //Covert to seconds
//    Serial.print("deltaTime: ");
//    Serial.println(deltaTime);

    float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

    windClicks = 0; //Reset and start watching for new wind
    lastWindCheck = millis();

    windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

//    Serial.println();
//     Serial.print("Windspeed:");
//     Serial.println(windSpeed);

    return(windSpeed);
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
    unsigned int adc;

    adc = analogRead(WDIR); // get the current reading from the sensor
    //Serial.println(adc);
    // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
    // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
    // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

    if (adc < 900) return (90);
    if (adc < 1034) return (135);
    if (adc < 1057) return (158);
    if (adc < 1095) return (113);
    if (adc < 1366) return (180);
    if (adc < 1790) return (23);
    if (adc < 1834) return (45);
    if (adc < 1880) return (68);
    if (adc < 2310) return (203);
    if (adc < 2325) return (248);
    if (adc < 2350) return (225);
    if (adc < 2529) return (338);
    if (adc < 2797) return (0);
    if (adc < 2915) return (293);
    if (adc < 3150) return (315);
    if (adc < 3500) return (270);
    return (-1); // error, disconnected?

}

void soilData() {
  
  Serial1.end();
  Serial3.begin(9600, SERIAL_8N2, 15, 2);
  
  const int requestframe[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x0C};
  int response[24];
  //Serial3.listen();
  int i=0, waittime=30;

  delay(10);
  for (int i = 0; i < 8; i++) 
  {
    Serial3.write(requestframe[i]);
  }

  i=0;
  while(waittime>0 && i<30)  
  {
   if( Serial3.available()>0)
     response[i++] =  Serial3.read();
   delay(40);
   waittime--;
  }
  
  for(int i=0; i<24; i++) 
     Serial3.write(response[i]);

  soil_temperature = (response[3] << 8 | response[4])/10.0;
  soil_moisture = (response[5] << 8 | response[6])/10.0;
  ec = (response[7] << 8 | response[8])/1.0;
  ph = (response[9] << 8 | response[10])/100.0;
  nitrogen =(response[11] << 8 | response[12])/1.0;
  phosphorous =(response[13] << 8 | response[14])/1.0;
  potassium =(response[15] << 8 | response[16])/1.0;
  salinity =(response[17] << 8 | response[18])/1.0;
  delay(1000);
  
  Serial3.end();
  Serial1.begin(115200, SERIAL_8N1, 23, 22);
}

void ShowSerialData()
{
  delay(100);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
    //Serial.println(Serial2.read());
  }
}

String getDateTime()
{
   char times[50];
   Serial2.find("+CCLK: \"");
   Serial2.readBytesUntil('\+', times, 50);
   //Serial.println(times);
   return(times);
}

void printWeather()
{
    calcWeather(); //Go calc all the various sensors
    //soilData();
    
    StaticJsonDocument<700> doc1;
    JsonObject object1 = doc1.to<JsonObject>();
    object1["image_type"]="crop_view",
    object1["base64Image"]=image.c_str();
    serializeJson(doc1, Serial);
    Serial.println(" "); 
    delay(100);
    StaticJsonDocument<700> arr;
    JsonArray array = arr.to<JsonArray>();
    arr.add(doc1);
    
    StaticJsonDocument<700> doc;
    JsonObject object = doc.to<JsonObject>();
    //object["ws_id"] ="016"; 
    object["ws_mac_address"]=mac;
    object["mac_address"]=mac;
    object["type"] ="pro_plus";
    //object["timestamp"] ="updated";
    object["wind_direction"]=winddir;
    object["wind_speed"]=avgwindspeedmph;
    object["wind_gust"]=windgustmph;
    object["wind_gust_direction"]=windgustdir;
    object["wind_speed_average"]=windspdmph_avg2m;
    object["wind_direction_average"]=winddir_avg2m;
    object["wind_gust_10min"]=windgustmph_10m;
    object["wind_gust_direction_10min"]=windgustdir_10m;
    object["humidity"]=humidity;
    object["temperature"]=avgtempc;
    //object["rain"]=(rainin*25.4);
    object["daily_rain"]=(dailyrainin*25.4);    
    object["pressure"]= avgpressure;
    object["battery_level"]=batt_lvl;
    object["ambient_light_level"]=light_lvl;  
    object["soil_temperature"]=soil_temperature;
    object["soil_moisture"]=soil_moisture;
    object["electrical_conductivity"]=ec;
    object["pH"]=ph;    
    object["nitrogen"]= nitrogen;
    object["phosphorous"]=phosphorous;
    object["potassium"]=potassium;
    object["salinity"]=salinity;    
    object["image_details"]= arr;
                                 
   /********************GSM Communication Starts********************/
    Serial2.println("AT+CCLK?");
    delay(100);
    String date_time= getDateTime();
    timeStamp = date_time.substring(9,14);
    //Serial.println(timeStamp);
    delay(100);
    ShowSerialData();
    
   if((sentdata == 1000) or (sentdata == 1)){                 //900 count because roughly the loop takes 4 s to run once. So for 1 hour the loop should run approx 900 times.

    if (Serial2.available())
    Serial.write(Serial2.read());
    Serial2.println("AT");
    ShowSerialData();
    
    Serial2.println("AT+CCLK?");
    delay(1000);
    String date_time= getDateTime();
    timeStamp = date_time.substring(9,14);
    //Serial.println(timeStamp);
    delay(100);
    ShowSerialData();
    
    Serial2.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    delay(100);
    ShowSerialData();
    Serial2.println("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"");//APN
    delay(100);
    ShowSerialData();
    Serial2.println("AT+SAPBR=1,1");
    delay(100);
    ShowSerialData();
    Serial2.println("AT+SAPBR=2,1");
    delay(100);
    ShowSerialData();
    Serial2.println("AT+HTTPINIT");
    delay(100);
    ShowSerialData();
    Serial2.println("AT+HTTPPARA=\"CID\",1");
    delay(100);
    ShowSerialData();
    object["rain"]=(rainin*25.4);
    serializeJson(doc, Serial);
    Serial.println(" ");
    String sendtoserver;
    serializeJson(doc, sendtoserver);
    delay(500);
    Serial2.println("AT+HTTPPARA=\"URL\",\"http://soil-test.solidaridadasia.net/api/store-wind-data-by-mac-address\""); //Server address http://soil-test.solidaridadasia.net/api/store-wind-data
    delay(500);
    ShowSerialData();
    Serial2.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    delay(500);
    ShowSerialData();
    Serial2.println("AT+HTTPDATA=" + String(sendtoserver.length()) + ",100000");
    Serial.println(sendtoserver);
    delay(100);
    ShowSerialData();
    Serial2.println(sendtoserver);
    delay(500);
    ShowSerialData();
    //rainHour[minutes] = 0; //Zero out this minute's rainfall amount
    Serial2.println("AT+HTTPACTION=1");
    delay(100);
    ShowSerialData();
    Serial2.println("AT+HTTPREAD");
    delay(100);
    ShowSerialData();
    Serial2.println("AT+HTTPTERM");
    delay(100);
    ShowSerialData();
    
    rainin = 0;
    sentdata=2;
    windavg[sentdata]=0;
    count=0;
    pres[sentdata]=0;
    count1=0;
    temperature[sentdata]=0;
    count2=0;
    hum[sentdata]=0;
    count3=0;
    image=" ";
   }
}

//code for OTA update
void otaupdate(){
  Serial.print("Waiting for network...");
    if (!modem.waitForNetwork())
    {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

    Serial.print("Connecting to ");
    Serial.print(apn);
    if (!modem.gprsConnect(apn))
    {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");

    Serial.print("Connecting to ");
    Serial.print(server);

    // if you get a connection, report back via serial:
    if (!client.connect(server, port))
    {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" OK");
    
    // Make a HTTP request:
    client.print(String("GET ") + resource + " HTTP/1.0\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.print("Connection: close\r\n\r\n");

    long timeout = millis();
    while (client.available() == 0)
    {
        if (millis() - timeout > 5000L)
        {
            Serial.println(">>> Client Timeout !");
            client.stop();
            delay(10000L);
            return;
        }
    }

    Serial.println("Reading header");
    uint32_t contentLength = knownFileSize;

    File file = SPIFFS.open("/update.bin", FILE_APPEND);


    while (client.available())
    {
        String line = client.readStringUntil('\n');
        line.trim();
        Serial.println(line);    // Uncomment this to show response header
        line.toLowerCase();
        if (line.startsWith("content-length:"))
        {
            contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
        }
        else if (line.length() == 0)
        {
            break;
        }
    }


    timeout = millis();
    uint32_t readLength = 0;

    unsigned long timeElapsed = millis();
    printPercent(readLength, contentLength);
    

    while (readLength < contentLength && client.connected() && millis() - timeout < 10000L)
    {
        int i = 0;
        while (client.available())
        {
            // read file data to spiffs
            if (!file.print(char(client.read())))
            {
                Serial.println("Appending file");
            }
            //Serial.print((char)c);       // Uncomment this to show data
            readLength++;

            if (readLength % (contentLength / 13) == 0)
            {
                printPercent(readLength, contentLength);
            }
            timeout = millis();
        }
    }

    file.close();

    printPercent(readLength, contentLength);
    timeElapsed = millis() - timeElapsed;
    Serial.println();

    client.stop();
    Serial.println("stop client");

    modem.gprsDisconnect();
    Serial.println("gprs disconnect");
    Serial.println();

    float duration = float(timeElapsed) / 1000;

    //readFile(SPIFFS, "/update.bin");

    updateFromFS();
    return;
//    // Do nothing forevermore
//    while (true)
//    {
//        delay(1000);
//    }
}


void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("APOK");
    }
    else
    {
        Serial.println("APX");
    }
}

void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
        Serial.write(file.read());
        delayMicroseconds(100);
    }
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
    return;
}

void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\n", path);
    if (fs.remove(path))
    {
        Serial.println("File deleted");
    }
    else
    {
        Serial.println("Delete failed");
    }
}

void updateFromFS()
{
    File updateBin = SPIFFS.open("/update.bin");
    if (updateBin)
    {
        if (updateBin.isDirectory())
        {
            Serial.println("Directory error");
            updateBin.close();
            return;
        }

        size_t updateSize = updateBin.size();

        if (updateSize > 0)
        {
            Serial.println("Starting update");
            performUpdate(updateBin, updateSize);
            return;
        }
        else
        {
            Serial.println("Error");
        }

        updateBin.close();

    }
    else
    {
        Serial.println("no such binary");
    }
    return;
}

void performUpdate(Stream &updateSource, size_t updateSize)
{
    if (Update.begin(updateSize))
    {
        size_t written = Update.writeStream(updateSource);
        if (written == updateSize)
        {
            Serial.println("Writes : " + String(written) + " successfully");
        }
        else
        {
            Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
        }
        if (Update.end())
        {
            Serial.println("OTA finished!");
            if (Update.isFinished())
            {
                Serial.println("Restart ESP device!");
                ESP.restart();   
            }
            else
            {
                Serial.println("OTA not finished");
            }
        }
        else
        {
            Serial.println("Error occured #: " + String(Update.getError()));
            return;
        }
    }
    else
    {
        Serial.println("Cannot begin update");
    }
    return;
}


void printPercent(uint32_t readLength, uint32_t contentLength)
{
    // If we know the total length
    if (contentLength != -1)
    {
        Serial.print("\r ");
        Serial.print((100.0 * readLength) / contentLength);
        Serial.print('%');
    }
    else
    {
        Serial.println(readLength);
    }
}

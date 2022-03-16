#include <RTCZero.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define DEBUG 0

#define RAIN_PIN 5

#define WSPEED_PIN 6
#define WDIR_PIN A0

#define SCTRL_PIN 10
#define SVOLT_PIN A1

#if DEBUG == 1
  #define SAMPLE_INTERVAL 5
  #define TRANSMIT_INTERVAL 5
#else
  #define SAMPLE_INTERVAL 5
  #define TRANSMIT_INTERVAL 300
#endif

#define NUM_SAMPLES (TRANSMIT_INTERVAL / SAMPLE_INTERVAL)

#define WIND_MULTIPLIER 1.492   // MPH
#define RAIN_MULTIPLIER 0.2794  // MM
#define SOLAR_MULTIPLIER (3300.0 / 1024.0) // MILLIVOLTS

RTCZero rtc;

Adafruit_BME280 bme;

uint16_t windSpeeds[NUM_SAMPLES];
uint8_t windDirs[NUM_SAMPLES];

uint16_t solarIntensities[NUM_SAMPLES];

volatile bool sample = false;
uint8_t currSample = 0;

volatile uint16_t windClicks = 0;
volatile uint16_t rainClicks = 0;
volatile uint8_t alarmSecs = 0;

uint8_t get_wind_direction() {

    uint16_t adc = analogRead(WDIR_PIN);
    
    if (adc < 380) return (5);
    if (adc < 397) return (3);
    if (adc < 414) return (4);
    if (adc < 456) return (7);
    if (adc < 508) return (6);
    if (adc < 551) return (9);
    if (adc < 615) return (8);
    if (adc < 680) return (1);
    if (adc < 746) return (2);
    if (adc < 801) return (11);
    if (adc < 833) return (10);
    if (adc < 878) return (15);
    if (adc < 913) return (0);
    if (adc < 940) return (13);
    if (adc < 967) return (14);
    //if (adc < 990) return (12);
    return (12);
    
}

void rainIRQ() {  
  rainClicks++;
}

void wspeedIRQ() {
  windClicks++;
}

void alarmIRQ() {
  alarmSecs += SAMPLE_INTERVAL;
  if (alarmSecs >= 60) {
    alarmSecs = 0;
  }
  rtc.setAlarmSeconds(alarmSecs);
  sample = true;
}

void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(SCTRL_PIN, OUTPUT);
  digitalWrite(SCTRL_PIN, LOW);
  
  pinMode(RAIN_PIN, INPUT_PULLUP);
  pinMode(WSPEED_PIN, INPUT_PULLUP);

  bme.begin(0x76, &Wire);
  bme.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_OFF);

  rtc.begin();
  rtc.setTime(0,0,0);
  rtc.setAlarmSeconds(0);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(alarmIRQ);

  LoRa.setPins(8,4,7);
  LoRa.begin(869525E3);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(8);

  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(WSPEED_PIN), wspeedIRQ, FALLING);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
  
}

void loop() {

  if (sample) {

    sample = false;
    
    windSpeeds[currSample] = windClicks;
    windClicks = 0;
    
    windDirs[currSample] = get_wind_direction();

    digitalWrite(SCTRL_PIN, HIGH);
    delayMicroseconds(50);
    solarIntensities[currSample] = analogRead(SVOLT_PIN) - 6;
    digitalWrite(SCTRL_PIN, LOW);

    
    
    currSample++;

    if (currSample >= NUM_SAMPLES) {
      
      currSample = 0;

      int16_t windDir = windDirs[0];
      int8_t D = windDirs[0];

      uint16_t windSpeed = windSpeeds[0];
      uint16_t windGust = windSpeeds[0];
      uint16_t solarIntensity = solarIntensities[0];

      for (uint8_t i = 1; i < NUM_SAMPLES; i++) {

        int8_t delta = windDirs[i] - D;
        
        if (delta < -7)
          D += delta + 16;
        else if (delta > 7)
          D += delta - 16;
        else
          D += delta;
          
        windDir += D;

        if (windSpeeds[i] > windGust) windGust = windSpeeds[i];
        windSpeed += windSpeeds[i];
        solarIntensity += solarIntensities[i];          
        
      }

      if (solarIntensity > 60000) solarIntensity = 0;

      windDir /= NUM_SAMPLES;

      if (windDir > 15) windDir -= 16;
      if (windDir < 0) windDir += 16;

      bme.takeForcedMeasurement();
  
      LoRa.beginPacket();

      #if DEBUG == 1
        LoRa.print("DBG");
      #else
        LoRa.print("MET");
      #endif    

      LoRa.print(" D");
      LoRa.print(windDir * 22.5, 0);
      LoRa.print(" S");
      LoRa.print(windSpeed * WIND_MULTIPLIER / TRANSMIT_INTERVAL, 2);
      LoRa.print(" G");
      LoRa.print(windGust * WIND_MULTIPLIER / SAMPLE_INTERVAL, 2);
      LoRa.print(" R");
      LoRa.print(rainClicks * RAIN_MULTIPLIER, 2);
      rainClicks = 0;
      LoRa.print(" T");
      LoRa.print(bme.readTemperature(), 2);
      LoRa.print(" H");
      LoRa.print(bme.readHumidity(), 2);
      LoRa.print(" P");
      LoRa.print(bme.readPressure() / 100.0, 2);
      LoRa.print(" I");
      LoRa.print(solarIntensity * SOLAR_MULTIPLIER / NUM_SAMPLES, 2);
      LoRa.print(" B");
      LoRa.print(analogRead(A7) * (6.6 / 1024.0), 2);

      for (int i=0;i<10;i++) {
        if (LoRa.rssi() < -80) {
          LoRa.endPacket();
          break;
        }
        delay(100);
      }
  
      LoRa.sleep();
      
    }
    
  }

  //#if DEBUG != 1
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    rtc.standbyMode();
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  //#endif
  
}


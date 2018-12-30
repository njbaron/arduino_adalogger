#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

#define LED_PIN 8
#define LED_PIN2 13
#define VBATPIN A7

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

#define cardSelect 4
#define numAvg 1000
#define timerDelay 100

Adafruit_BMP280 bme;

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();
void writeToSD();
void recordData();
void error();

char filename[15];
File logfile;

float measuredvbat;
float temperature;
float pressure;
float altitude;
int numZ = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);

  digitalWrite(LED_PIN2, HIGH);

  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }
  strcpy(filename, "TEMALT00.csv");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  //-----------------
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    error(4);
  }
  
  startTimer(timerDelay);

  digitalWrite(LED_PIN2, LOW);
  Serial.println("Ready!");
  
}

void loop() {
    measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage

    if(measuredvbat < 3.5){
      error(1);
    }
    
    delay(10000);
  }

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(LED_PIN2, HIGH);
      delay(100);
      digitalWrite(LED_PIN2, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void writeToSD(){
  digitalWrite(LED_PIN, HIGH);
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);
  logfile.print(measuredvbat); logfile.print(",");
  logfile.print(temperature); logfile.print(",");
  logfile.print(pressure); logfile.print(",");
  logfile.println(altitude);
  Serial.print(measuredvbat); Serial.print(",");
  Serial.print(temperature); Serial.print(",");
  Serial.print(pressure); Serial.print(",");
  Serial.println(altitude);
  logfile.close();
  digitalWrite(LED_PIN, LOW);
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    temperature += bme.readTemperature();
    pressure += bme.readPressure();
    altitude += bme.readAltitude(1013.25);
    numZ = (numZ+1)%numAvg;

    if(numZ == 0) {
      temperature = temperature / numAvg;
      pressure = pressure / numAvg;
      altitude = (altitude / numAvg) * 3.28084;
      writeToSD();
      temperature = 0;
      pressure = 0;
      altitude = 0;
    }
  }
}

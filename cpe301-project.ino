// Noah Snelson and Rachel Burnett
// December 2020
// Code for swamp cooler running on Arduino MEGA
// Utilizes libraries included w/ components from
// Elegoo kit, as well as code from previous labs.

// Elegoo DHT11 Temperature & Humidity Sensor setup
#include "dht_nonblocking.h"

#define DHT_SENSOR_TYPE DHT_TYPE_11
#define DHT_SENSOR_PIN 2
#define TEMPERATURE_THRESHOLD 25.0
#define WATER_LEVEL_THRESHOLD 100

volatile unsigned char* adcsra = (unsigned char*) 0x7A;
volatile unsigned char* adcsrb = (unsigned char*) 0x7B;
volatile unsigned char* portf = (unsigned char*) 0x31;
volatile unsigned char* ddrf = (unsigned char*) 0x30;
volatile unsigned char* admux = (unsigned char*) 0x7C;
volatile unsigned char* didr0 = (unsigned char*) 0x7E;
volatile unsigned int* adcdata = (unsigned int*) 0x78;

DHT_nonblocking temp_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

float temperature;
float humidity;
float water_level;

enum STATE {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR
};
int state = IDLE;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  adc_init();
}

void loop() {
  log_atmosphere();
  adjust_vent();
  check_disabled();
  // put your main code here, to run repeatedly:
  if(state == IDLE){
    if(water_level_low()){
      state = ERROR;
    }
    else if(temperature > TEMPERATURE_THRESHOLD){
      toggle_fan(1);
      state = RUNNING;
    }
  }
  else if(state == RUNNING){
    if(water_level_low()){
      state = ERROR;
    }
    else if(temperature < TEMPERATURE_THRESHOLD){
      toggle_fan(0);
      state = IDLE;
    }
  }
  else if(state == ERROR){
    if(!water_level_low()){
      state = IDLE;
    }
  }
}

void check_disabled(){
  
}

void idle_routine(){
  
}

void log_atmosphere(){
  // write humidity/temp
  for(int i = 0; i < 1000; i++){}
  if(temp_sensor.measure(&temperature, &humidity)) {
    Serial.print(state);
    Serial.print(" temp = ");
    Serial.print(temperature);
    Serial.print(" humidity = ");
    Serial.println(humidity);
  }
}

bool water_level_low(){
  return adc_read(0) < WATER_LEVEL_THRESHOLD;
}

void toggle_fan(bool state){
  log_motor(state);
  // TODO
}

void adjust_vent(){
  // TODO
}

void log_motor(bool state){
  // TODO
}

void adc_init() {
  *adcsra |= 0b10000000;
  *adcsra &= 0b11010000; // enable ADC, disable interrupts and autotrigger

  *ddrf &= 0b11111110; // set pf0 to input
  *didr0 |= 0b00000001; // disable digital input
  
  *adcsrb &= 0b11110000; // set free running mode, reset channel/gain bits
  
  *admux = 0b01000000; // right adjust ADLAR, set voltage ref
}

unsigned int adc_read(unsigned char adc_channel){
  *admux &= 0b11100000; // clear channel select bits 4:0
  *adcsrb &= 0b11110111; // clear channel select bit 5

  if(adc_channel >= 8){
    *adcsrb |= 0b00001000; // set channel select bit 5 to 1
  }

  char lastThree = adc_channel & 0b00000111; // get last three bits of channel
  *admux |= lastThree; // set channel select bits 4:0

  *adcsra |= 0x40; // start ADC conversion

  while((*adcsra & 0x40) != 0) {} // wait while converting

  return (*adcdata & 0x03FF);
}

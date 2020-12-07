// Noah Snelson and Rachel Burnett
// December 2020
// Code for swamp cooler running on Arduino MEGA
// Utilizes libraries included w/ components from
// Elegoo kit, as well as code from previous labs.

// Elegoo DHT11 Temperature & Humidity Sensor setup
#include "dht_nonblocking.h"

// Elegoo Servo
#include <Servo.h>

// DS3231 Real Time Clock
#include "DS3231.h"

#include "LiquidCrystal.h"

#define DHT_SENSOR_TYPE DHT_TYPE_11
#define DHT_SENSOR_PIN 2
#define TEMPERATURE_THRESHOLD 25.0
#define WATER_LEVEL_THRESHOLD 500

volatile unsigned char* adcsra = (unsigned char*) 0x7A;
volatile unsigned char* adcsrb = (unsigned char*) 0x7B;
volatile unsigned char* portf = (unsigned char*) 0x31;
volatile unsigned char* ddrf = (unsigned char*) 0x30;
volatile unsigned char* admux = (unsigned char*) 0x7C;
volatile unsigned char* didr0 = (unsigned char*) 0x7E;
volatile unsigned int* adcdata = (unsigned int*) 0x78;

volatile unsigned char* portb = (unsigned char*) 0x25;
volatile unsigned char* ddrb = (unsigned char*) 0x24;
volatile unsigned char* pinb  = (unsigned char*) 0x23; 

#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

DHT_nonblocking temp_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

float temperature;
float humidity;
float water_level;

Servo servo;

DS3231 clock;
RTCDateTime datetime;

LiquidCrystal lcd(4,5,6,7,8,9);

enum STATE {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR
};
int state = IDLE;

enum portb_pins {
  GREEN = 0,
  BLUE = 1,
  YELLOW = 2,
  RED = 3,
  MOTOR = 4,
  SWITCH = 5
};

void setup() {
  // put your setup code here, to run once:
  U0init(9600);
  adc_init();

  servo.attach(3);

  *ddrb |= 0b00011111; // set led pins to output
  *ddrb &= ~(0b01 << SWITCH); // set input for disable switch
  *portb &= ~(0b01 << MOTOR); // set motor to output low

  write_pb(GREEN, 1);

  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);

  lcd.begin(16, 2);
}

void loop() {
  adjust_vent();
  check_disable();
  if(state == IDLE){
    log_atmosphere();
    if(water_level_low()){
      lcd.clear();
      alert_water_level();
      write_pb(GREEN, 0);
      write_pb(RED, 1);
      state = ERROR;
    }
    else if(temperature > TEMPERATURE_THRESHOLD){
      write_pb(GREEN, 0);
      write_pb(BLUE, 1);
      toggle_fan(1);
      state = RUNNING;
    }
  }
  else if(state == RUNNING){
    log_atmosphere();
    if(water_level_low()){
      lcd.clear();
      alert_water_level();
      write_pb(BLUE, 0);
      write_pb(RED, 1);
      toggle_fan(0);
      state = ERROR;
    }
    else if(temperature < TEMPERATURE_THRESHOLD){
      write_pb(BLUE, 0);
      write_pb(GREEN, 1);
      toggle_fan(0);
      state = IDLE;
    }
  }
  else if(state == ERROR){
    if(!water_level_low()){
      lcd.clear();
      write_pb(RED, 0);
      write_pb(GREEN, 1);
      state = IDLE;
    }
  }
}

void check_disable(){
  if(*pinb & (0b01 << SWITCH)){
    state = state == DISABLED ? IDLE : DISABLED;
    if(state == IDLE){
      write_pb(YELLOW, 0);
      write_pb(GREEN, 1);
    }
    else{
      for(int pin = 0; pin < 5; pin++){
        write_pb(pin, 0); // turn all leds & motor off
      }
      write_pb(YELLOW, 1);
    }
    for(int i = 0; i < 10000; i++) {} // debounce
    while(*pinb & (0b01 << SWITCH)){}
  }
}

void log_atmosphere(){
  if(temp_sensor.measure(&temperature, &humidity)) {
    lcd.setCursor(0,0);
    lcd.print("temp = ");
    lcd.print(temperature);
    lcd.setCursor(0,1);
    lcd.print("humidity = ");
    lcd.print(humidity);
  }
  for(int i = 0; i < 10000; i++){}
}


bool water_level_low(){
  water_level = adc_read(0);
  for(int i = 0; i < 10000; i++){}
  return (water_level < WATER_LEVEL_THRESHOLD);
}

void alert_water_level(){
  lcd.setCursor(0,0);
  lcd.print("ALERT!");
  lcd.setCursor(0,1);
  lcd.print("LOW WATER");
}

void toggle_fan(bool state){
  log_motor(state);
  write_pb(MOTOR, state);
}

int angle = 0;
void adjust_vent(){
  int new_angle = adc_read(1);
  if((new_angle > angle && new_angle - angle > 100) || 
      (angle > new_angle && angle - new_angle > 100)){
    angle = new_angle;
    servo.write(angle);
  }
}

// some decent string utils (that don't segfault) would be nice
void log_motor(bool motor_state){
  datetime = clock.getDateTime();
  String log_message = String(datetime.year);
  log_message += "-";
  log_message += datetime.month;
  log_message += "-";
  log_message += datetime.day;
  log_message += "T";
  log_message += datetime.hour;
  log_message += ":";
  log_message += datetime.minute;
  log_message += ":";
  log_message += datetime.second;
  log_message += " Motor turned ";
  log_message += motor_state ? "on\n" : "off\n";
  print_string(log_message.c_str());
}

void print_string(char* string){
  for(int i = 0; i < strlen(string); i++){
    U0putchar(string[i]);
  }
}

void adc_init() {
  *adcsra |= 0b10000000;
  *adcsra &= 0b11010000; // enable ADC, disable interrupts and autotrigger

  *ddrf &= 0b11111100; // set pf0 to input
  *didr0 |= 0b00000011; // disable digital input
  
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

void write_pb(int pin, bool state){
  if(state){
    *portb |= 0b00000001 << pin;
  }
  else{
    *portb &= ~(0b00000001 << pin);
  }
}

void U0init(unsigned long U0baud)
{
//  Students are responsible for understanding
//  this initialization code for the ATmega2560 USART0
//  and will be expected to be able to intialize
//  the USART in differrent modes.
//
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

void U0putchar(unsigned char U0pdata)
{
  while( (*myUCSR0A & TBE) == 0) {}
  *myUDR0 = U0pdata;
}

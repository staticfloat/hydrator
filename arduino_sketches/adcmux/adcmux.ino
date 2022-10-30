#include "Arduino.h"
static uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Increasing this smooths out ADC readings by
// integrating over it by 2^N, e.g. a value of
// 5 means to take 32 readings and average them.
#define STABLE_ADC_ITERATIONS_POW    5

#define ADC_MUX_CTL_BIT0             GPIO1


TimerEvent_t sleepTimer;
bool sleepTimerExpired = true;

static void wakeUp()
{
  sleepTimerExpired=true;
}
static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired=false;
  TimerInit( &sleepTimer, &wakeUp );
  TimerSetValue( &sleepTimer, sleeptime );
  TimerStart( &sleepTimer );
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}

void setup() {
  // We will use chipID as our devEUID
  ((uint64_t*)devEui)[0] = getID();
  
  Serial.begin(115200);
  Serial.println("Comlink online");
  delay(1000);

  Serial.printf("Chip-based devEui: %02X%02X%02X%02X%02X%02X%02X%02X\r\n", devEui[0], devEui[1], devEui[2], devEui[3], devEui[4], devEui[5], devEui[6], devEui[7]);

  // Get GPIO1 ready to control the ADC mux
  pinMode(ADC_MUX_CTL_BIT0, OUTPUT);
}

uint16_t stable_adc_read() {
  uint32_t accumulator = 0;
  for (uint8_t idx=0; idx<(1<<STABLE_ADC_ITERATIONS_POW); ++idx) {
    accumulator += analogRead(ADC);
  }
  // Divide by 2^N to average things out
  return accumulator >> STABLE_ADC_ITERATIONS_POW;
}

void set_adc_mux_channel(uint8_t channel) {
  digitalWrite(ADC_MUX_CTL_BIT0, (channel >> 0 ) % 2);
  //digitalWrite(ADC_MUX_CTL_BIT1, (channel >> 1) % 2);
  //digitalWrite(ADC_MUX_CTL_BIT2, (channel >> 2) % 2);
  //digitalWrite(ADC_MUX_CTL_BIT3, (channel >> 3) % 2);
}

void read_analog_inputs(uint16_t * battery_level, uint16_t * moisture_level) {
  // Turn on power to the ADC mux and capacitive sensor via Vext
  pinMode(GPIO6,OUTPUT);
  digitalWrite(GPIO6,LOW);

  // Switch to channel zero (which is internal battery)
  set_adc_mux_channel(0);
  lowPowerSleep(1);
  *battery_level = stable_adc_read();
  
  // Switch to channel one (which is moisture sensor)
  set_adc_mux_channel(1);
  // moisture sensor takes a while to settle
  lowPowerSleep(30);
  *moisture_level= stable_adc_read();

  // Turn off Vext
  pinMode(GPIO6,INPUT);
  set_adc_mux_channel(0);
}

void loop() {
  uint16_t battery_level = 0;
  uint16_t moisture_level = 0;

  read_analog_inputs(&battery_level, &moisture_level);
  Serial.printf("SEND: batt[%d], moisture[%d]\r\n", battery_level, moisture_level);
  delay(50);
}

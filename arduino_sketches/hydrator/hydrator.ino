#include "Arduino.h"
#include "LoRaWanMinimal_APP.h"


// Un-comment this to turn on serial debugging
//#define SERIAL_DEBUGGING
#define LOOP_PERIOD_IN_SECONDS       60
// Increasing this smooths out ADC readings by
// integrating over it by 2^N, e.g. a value of
// 5 means to take 32 readings and average them.
#define STABLE_ADC_ITERATIONS_POW    5

// Control our ADC mux with various bits of control
#define ADC_MUX_CTL_BIT0             GPIO1

// This will be filled in by `chipID()` in `setup()`
static uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t appEui[] = { 0x68, 0x79, 0x64, 0x72, 0x6f, 0x00, 0x00, 0x00 };
// Appkey for copy-pasta 6879647261746f726b65793132333435
static uint8_t appKey[] = { 0x68, 0x79, 0x64, 0x72, 0x61, 0x74, 0x6f, 0x72, 0x6b, 0x65, 0x79, 0x31, 0x32, 0x33, 0x34, 0x35 };

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
uint32_t tx_counter = 0;

///////////////////////////////////////////////////
//Some utilities for going into low power mode
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
  
  // put your setup code here, to run once:
#if defined(SERIAL_DEBUGGING)
  Serial.begin(115200);
  Serial.println("Comlink online");
  delay(1000);

  Serial.printf("Chip-based devEui: %02X%02X%02X%02X%02X%02X%02X%02X\r\n", devEui[0], devEui[1], devEui[2], devEui[3], devEui[4], devEui[5], devEui[6], devEui[7]);
#endif

  // Get GPIOs ready to control the ADC mux
  pinMode(ADC_MUX_CTL_BIT0, OUTPUT);
  digitalWrite(ADC_MUX_CTL_BIT0, LOW);

  // Setup LoRaWAN connection
  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  LoRaWAN.setAdaptiveDR(true);

  while (1) {
#if defined(SERIAL_DEBUGGING)
    Serial.print("Joining... ");
#endif

    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    if (!LoRaWAN.isJoined()) {
#if defined(SERIAL_DEBUGGING)
      Serial.println("JOIN FAILED! Sleeping for 30 seconds");
#endif
      lowPowerSleep(30000);
    } else {
#if defined(SERIAL_DEBUGGING)
      Serial.println("JOINED");
#endif
      break;
    }
  }
}

uint8_t payload[] = {
  // 32 bits for Tx counter
  0x00, 0x00, 0x00, 0x00,
  // 16 bits for battery voltage
  0x00, 0x00,
  // 16 bits for moisture sensor
  0x00, 0x00,
};

uint16_t stable_adc_read() {
  uint32_t accumulator = 0;
  for (uint8_t idx=0; idx<(1<<STABLE_ADC_ITERATIONS_POW); ++idx) {
    accumulator += analogRead(ADC);
  }
  // Divide by 2^(N-1) to average things out and correct for voltage divider
  return accumulator >> (STABLE_ADC_ITERATIONS_POW - 1);
}

void set_adc_mux_channel(uint8_t channel) {
  digitalWrite(ADC_MUX_CTL_BIT0, (channel >> 0 ) % 2);
  //digitalWrite(ADC_MUX_CTL_BIT1, (channel >> 1) % 2);
  //digitalWrite(ADC_MUX_CTL_BIT2, (channel >> 2) % 2);
  //digitalWrite(ADC_MUX_CTL_BIT3, (channel >> 3) % 2);
}

void read_analog_inputs(uint16_t * battery_level, uint16_t * moisture_level) {
  // Turn on power to the ADC mux and capacitive sensor via Vext
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW);

  // Switch to channel zero (which is internal battery)
  set_adc_mux_channel(0);
  lowPowerSleep(1);
  *battery_level = stable_adc_read();
  
  // Switch to channel one (which is moisture sensor)
  set_adc_mux_channel(1);
  // moisture sensor takes a while to settle
  lowPowerSleep(100);
  *moisture_level= stable_adc_read();

  // Turn off Vext
  set_adc_mux_channel(2);
  pinMode(Vext,INPUT);
}

void loop() {
  // Read battery level (internally switches ADC to battery)
  uint16_t battery_level = 0;

  // Read moisture sensor level by reading 64 times in a row and averaging
  uint16_t moisture_level = 0;

  read_analog_inputs(&battery_level, &moisture_level);

  *((uint32_t *)(payload + 0)) = tx_counter;
  *((uint16_t *)(payload + 4)) = battery_level;
  *((uint16_t *)(payload + 6)) = moisture_level;

#if defined(SERIAL_DEBUGGING)
  Serial.printf("SEND: tx[%d], batt[%d], moisture[%d]\r\n", tx_counter, battery_level, moisture_level);
#endif  

  if (!LoRaWAN.send(sizeof(payload), payload, 1, true)) {
#if defined(SERIAL_DEBUGGING)
    Serial.println("Send FAILED");
#endif
  }

  // Increment our Tx counter
  tx_counter += 1;

#if defined(SERIAL_DEBUGGING)
  // Delay to allow serial to finish its thing
  delay(100);
#endif

  // Enter low-power sleep for a while
  lowPowerSleep(LOOP_PERIOD_IN_SECONDS*1000);
}

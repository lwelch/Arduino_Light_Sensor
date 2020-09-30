#include <Arduino.h>
#include <avr/sleep.h>

/* Photocell simple testing sketch. 
Connect one end of the photocell to 5V, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground 
Connect LED from pin 11 through a resistor to ground 
For more information see http://learn.adafruit.com/photocells */
// #define DEBUG

#define PHOTOCELL_PIN A0
#define RELAY_PIN 12
#define PHOTOCELL_ON_COUNT 450
#define PHOTOCELL_OFF_COUNT 550

#define PIT_INT_TIME PIT_INT_TIME_32_S /* Must be from the enum type Pit_Interrupt_Times_T */
#define SLEEP_TIME_S 64 /* It will be rounded to the nearest SLEEP_TIME seconds */
#define SLEEP_LOOPS (uint16_t)(SLEEP_TIME_S / (uint16_t)PIT_INT_TIME)

#define ON_TIME_MINS 240 /* Time in minutes to keep the pumpkins on */

#define PIT_INT_CONST PIT_INT_TIME

typedef enum
{
  PIT_INT_TIME_1_S = 1u,
  PIT_INT_TIME_2_S = 2u,
  PIT_INT_TIME_4_S = 4u,
  PIT_INT_TIME_8_S = 8u,
  PIT_INT_TIME_16_S = 16u,
  PIT_INT_TIME_32_S = 32u
} Pit_Interrupt_Times_T;

typedef enum
{
  PIT_INT_REGISTER_1_S = 0x9,
  PIT_INT_REGISTER_2_S = 0xA,
  PIT_INT_REGISTER_4_S = 0xB,
  PIT_INT_REGISTER_8_S = 0xC,
  PIT_INT_REGISTER_16_S = 0xD,
  PIT_INT_REGISTER_32_S = 0xE
} Pit_Interrupt_Register_Value_T;

void disable_adc(void);
void enable_adc(void);
void go_to_sleep(void);
void init_pit_interrupt(void);
void heartbeat(void);

bool determine_desired_output(uint16_t photocell_reading, bool relay_on);
uint16_t read_photocell(void);


int photocellPin = PHOTOCELL_PIN;     // the cell and 10K pulldown are connected to a0

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  
  //Save Power by writing all Digital IO LOW - note that pins just need to be tied one way or another, do not damage devices!
  for (int i = 0; i < 20; i++) {
    if (i != PHOTOCELL_PIN)
    {
      pinMode(i, OUTPUT);
    }
  }
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PHOTOCELL_PIN, INPUT);

  disable_adc();

  init_pit_interrupt();
  
  //ENABLE SLEEP - this enables the sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Disable BOD in Sleep
  BOD.CTRLA &= ~(BOD_SLEEP_gm);
}

void loop() {
  static uint16_t skip_loops = SLEEP_LOOPS;
  static uint16_t val = 0;
  static uint16_t loop_val = 0;
  static uint8_t pin_state = LOW;
  static uint16_t adc_val;
  static bool relay_on = false;

  if (skip_loops >= SLEEP_LOOPS)
  { 
    adc_val = read_photocell();

    relay_on = determine_desired_output(adc_val, relay_on);

    digitalWrite(RELAY_PIN, relay_on);
    loop_val++;
    skip_loops = 0;

#ifdef DEBUG
    Serial.print("ADC Value = ");
    Serial.println(adc_val);     // the raw analog reading
    Serial.print("relay_on = ");
    Serial.println(relay_on);
    delay(100);
#endif
  }
  
  heartbeat();
  skip_loops++;

  val++;

  go_to_sleep();

}

void disable_adc(void)
{
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADC0.CTRLA  &= ~(1 << AC_ENABLE_bp);
}

void enable_adc(void)
{
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADC0.CTRLA  |= (1 << AC_ENABLE_bp);
}

void go_to_sleep(void)
{
  sleep_cpu();
}

uint16_t read_photocell(void)
{
  int16_t photocellReading;
  enable_adc();
  photocellReading = analogRead(photocellPin); 
  disable_adc();

  return abs(photocellReading);
}

bool determine_desired_output(uint16_t photocell_reading, bool relay_on)
{
  static uint16_t time_output_on = 0;
  if (true == relay_on)
  {
    if (PHOTOCELL_OFF_COUNT < photocell_reading)
    {
      relay_on = false;
    }
    else
    {
      time_output_on++;
    }
    if (time_output_on > (ON_TIME_MINS * 60 / SLEEP_TIME_S))
    {
      relay_on = false;
    }
  }
  else
  {
    if (time_output_on > (ON_TIME_MINS * 60 / SLEEP_TIME_S ))
    {
      if ( PHOTOCELL_OFF_COUNT < photocell_reading)
      {
        time_output_on = 0;
      }
    }
    else
    {
      if (PHOTOCELL_ON_COUNT > photocell_reading)
      {
        relay_on = true;
      }
    }
  }
  return (relay_on);
}

static void init_pit_interrupt(void)
{
  uint16_t time_reg_val;
  //To operate the PIT, follow these steps:
  //1. Configure the RTC clock CLK_RTC as described in   Configure the Clock CLK_RTC.

  RTC.CLKSEL = 0x01; //Use OSCULP1K oscillator
   
  //2. Enable the interrupt by writing a '1' to the Periodic Interrupt bit (PI) in the PIT Interrupt Control
  //register (RTC.PITINTCTRL).

  while (1 & RTC.PITSTATUS); //check if register isn't busy
 
  RTC.PITINTCTRL = 0x01; //Enable interrupts
 
  //3. Select the period for the interrupt and enable the PIT by writing the desired value to the PERIOD bit
  //field and a '1' to the PIT Enable bit (PITEN) in the PIT Control A register (RTC.PITCTRLA).

  while (1 & RTC.STATUS); //check if register isn't busy

  switch (PIT_INT_TIME)
  {
  case PIT_INT_TIME_1_S:
    time_reg_val = PIT_INT_REGISTER_1_S;
    break;
  case PIT_INT_TIME_2_S:
    time_reg_val = PIT_INT_REGISTER_2_S;
    break;
  case PIT_INT_TIME_4_S:
    time_reg_val = PIT_INT_REGISTER_4_S;
    break;
  case PIT_INT_TIME_8_S:
    time_reg_val = PIT_INT_REGISTER_8_S;
    break;
  case PIT_INT_TIME_16_S:
    time_reg_val = PIT_INT_REGISTER_16_S;
    break;
  case PIT_INT_TIME_32_S:
    time_reg_val = PIT_INT_REGISTER_32_S;
    break;
  default:
    time_reg_val = 0;
    break;
  }

  RTC.PITCTRLA = ((time_reg_val << 3) | 0x1); //This sets the PIT interrupt frequency and enables it

  return;
}
void set_builtin_led(bool on)
{
  digitalWrite(LED_BUILTIN, on);
}

void heartbeat(void)
{
  static uint16_t loops_since_last_beat = 0;

  if (loops_since_last_beat > (32u / PIT_INT_TIME)) /* Beat once every 32 seconds */
  {
#ifdef DEBUG
    Serial.println("Triggered Heartbeat");
#endif
    set_builtin_led(true);
    delay(2000);
    set_builtin_led(false);
    loops_since_last_beat = 0;
  }
  loops_since_last_beat++;

#ifdef DEBUG
  Serial.print("Hit Heartbeat ");
  Serial.print(loops_since_last_beat);
  Serial.println(" times");
  delay(100);
#endif
}

ISR(RTC_PIT_vect) {
  static uint8_t test = 0;
  test++;
#ifdef DEBUG
  Serial.print("PIT Interrupt hit! # = ");
  Serial.println(test);
#endif
  RTC.PITINTFLAGS = 1; // reset interrupt
}
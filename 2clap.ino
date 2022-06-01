
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#include <avr/interrupt.h>


int on = 0;
int interruptPin = 2;
uint64_t last_clap[] = {0,0};
uint64_t last_wakeup = 0;
uint32_t last_turn_on;
uint8_t  sleep = 0;
const uint16_t MAX_BLOW_INTERVAL = 35;
const uint16_t BLOW_PERIOD_MS = 300;
const uint16_t BLOW_COUNT_LIMIT = BLOW_PERIOD_MS / MAX_BLOW_INTERVAL;
uint8_t  blowcount = 0;

uint32_t last_noise = 0;
const uint32_t NOISE_TIMEOUT_MILLIS = 1000;

const uint32_t TURN_OFF_MILLIS = 7200000; // 2 hours
//const uint32_t TURN_OFF_MILLIS = 10000; // 10 seks
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pins as an output.
  pinMode(4, INPUT);
  randomSeed(analogRead(A2));
  pinMode(3, OUTPUT); // Microphone mode pin
  pinMode(2, INPUT);  // Sound detected pin
  analogWrite(0, 0);  // Turn LED off
  DIDR0 = DIDR0 | B00111111;

  if(0) {
    digitalWrite(0, HIGH);
    delay(500);
    digitalWrite(0, LOW);
    delay(500);
    digitalWrite(0, HIGH);
    delay(500);
    digitalWrite(0, LOW);
    delay(500);
    digitalWrite(0, HIGH);
    delay(500);
    digitalWrite(0, LOW);
    delay(500);
  }
  // Start delay
  delay(1000);
  // Set the microphone in wake on sound mode
  digitalWrite(3, LOW);
  delay(10);
  digitalWrite(3, HIGH);

}


void Going_To_Sleep(){
    last_clap[0] = 0;
    last_clap[1] = 0;
    sleep = 0;
    blowcount = 0;
    digitalWrite(0,LOW);
    digitalWrite(3, HIGH);
    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK &= ~(_BV(PCINT1));                // Turn off PB1 interrupt
    PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep
    

    cli();                                  // Disable interrupts
    sleep_disable();                        // Clear SE bit
    PCMSK &= ~(_BV(PCINT2));                // Turn off PB2 interrupt
    PCMSK |= _BV(PCINT1);                   // Use PB1 as interrupt pin
    sei();                                  // Enable interrupts
    digitalWrite(3, LOW);
    last_wakeup = millis();
  }


ISR(PCINT0_vect) {
        last_clap[0] = last_clap[1];
        last_clap[1] = millis();
        delay(25);
        int turn_on = 0;
        if(last_clap[0] != 0) {
          uint64_t diff = last_clap[1] - last_clap[0];
          turn_on   = ((diff < 750) && (diff > 100)) && (on == 0) ? 1 : 0;
          blowcount = diff <= MAX_BLOW_INTERVAL ? blowcount + 1 : 0;
          sleep = turn_on == 0 ? 1 : 0;
        }
        bool blowout = (blowcount >= BLOW_COUNT_LIMIT);
        on = blowout ? 0 : (turn_on ? 1 : on);
        if(blowout) {
          PCMSK &= ~(_BV(PCINT1)); // Turn off interupts to let ligth turn off
        }
        last_turn_on = turn_on ? last_clap[1] : last_turn_on;
    }



// the loop function runs over and over again forever
void loop() {
  if(on == 1) {
    analogWrite(0, random(120)+135);
    delay(random(120));
    if((millis() < last_turn_on) || ((millis() - last_turn_on) > TURN_OFF_MILLIS)) {
      // Timeout or millis overflow
      on = 0;
    }
  } else {
    digitalWrite(0,LOW);
    if((millis() - last_wakeup) > 750 || millis() < last_wakeup) {
      Going_To_Sleep();
    }
  }
}

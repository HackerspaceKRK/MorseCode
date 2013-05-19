#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

// PIN CONFIGURATION:
//  INPUTS:
//   PD2 (INT0) - CALL INPUT (also wake-up interrupt), external pull-down, goes low when triggered
//   PD3 (INT1) - PROGRAM CODE PUSH BUTTON, need internal pull-up, goes low when triggered
//  OUTPUTS:
//   PB0 - OPTOCOUPLER (high active)
//   PB3 - TRANSISTOR FOR GENERATING AUDIO (high active)
//   PB2 - STATUS LED (high active)

// HOW THIS SHIT IS SUPPOSED TO WORK:
// 1. Code is stored in EEPROM as 1 byte. eg. 10111000 means 1-long, 1-short, 3-long
// 2. Setting the code is done by pushing PROGRAM CODE PUSH BUTTTON for ~10s (STATUS LED will start
//    blinking, then enter the new code and wait for the STATUS LED to blink (it should blink the new code)
// 3. Current code is displayed after pressing PROGRAM CODE PUSH BUTTON for ~4s.
// 4. Pulses are measured using internal timer
// I'm using 8-bit Timer/Counter0 for measuring length of pulses
// Inputs are "blocking" (does it make any sense to make it non-blocking?)
// Board is prepared for putting uC to sleep... (all inputs are active on LOW state), but probably this is not necessary.

// DEFINES
#define INPUT_DDR DDRD
#define INPUT_PORT PORTD
#define INPUT_PIN PIND

#define CALL_INPUT PD2
#define PROGRAM_INPUT PD3
//--
#define OUTPUT_DDR DDRB
#define OUTPUT_PORT PORTB

#define OPTOCOUPLER_OUTPUT PB0
#define AUDIO_OUTPUT PB3
#define STATUS_LED_OUTPUT PB2

// tweaks
#define PULSE_MAX 20
#define PULSE_LONG 2

volatile uint8_t code = 0;
volatile uint8_t timer_count = 0;

uint8_t good_code = 0;
uint8_t EEMEM CODE_EEPROM;

// good software debouncing (intercom buttons are crappy!)
static uint8_t pin_is(uint8_t pin, uint8_t state) {
    uint8_t i = 255;
    uint8_t hits = 0;

    while(--i) {
        if(bit_is_set(INPUT_PIN, pin))
            hits++;

        _delay_us(70);
    }

    return (hits<127)^state;
}

// returns length of pulse at pin (at INPUT_PORT)
// high_low set if we will measure low or high state
static uint8_t pulse_length(uint8_t pin, uint8_t high_low, uint8_t timeout) {
    _delay_ms(50); // for debouncing

    // configure Timer/Counter0
    TCNT0 = 0;
    TCCR0B |= _BV(CS02) | _BV(CS00); // CK/1024
    TIMSK |= _BV(TOIE0); // enable overflow interrupt
    timer_count = 0;

    while(pin_is(pin, high_low) && timer_count<timeout) {};

    TCCR0B = 0;

    return timer_count;
}

static uint8_t read_morse(uint8_t pin, uint8_t high_low, uint8_t timeout) {
    uint8_t i = 0; // position in code
    uint8_t code = 0; // empty code

    for(i=0; i<8; i++) {
        // wait for pulse
        for(;;) {
            uint8_t wait_len = pulse_length(pin, high_low^1, timeout);

            if(wait_len == timeout) // timeout
                return code;
            else
                break;
        }

        uint8_t r = pulse_length(pin, high_low, PULSE_MAX);

        if(r > PULSE_LONG) {
            code |= _BV(i);
            if(r == PULSE_MAX)
                break;
        }
    }

    return code;
}

static void blink_morse(uint8_t morse) {
    uint8_t i = 0;
    uint8_t len = 7;

    while(!(_BV(len) & morse) && len) {
        len--;
    }

    for(i=0; i<=len; i++) {
        OUTPUT_PORT |= _BV(STATUS_LED_OUTPUT);
        if(_BV(i) & morse) { // 1
            _delay_ms(900);
        } else {
            _delay_ms(100);
        }
        OUTPUT_PORT &= ~_BV(STATUS_LED_OUTPUT);

        _delay_ms(300);
    }
}

static void audio_bad() {
    // software audio generation, we should do this by PWM...

    int timer = 2553; // must be odd

    while(--timer) {
        OUTPUT_PORT ^= _BV(AUDIO_OUTPUT);
        _delay_us(200);
    }    
}

static void handle_call_input(void) {
    uint8_t read = read_morse(CALL_INPUT, 0, 4);

    // this is the only place we use WDT... why here? if this hung
    // intercom would "beep" all the time... sure, somebody could get angry because of this
    // or if this failed we would burn electrotap (neighbor would get angry...)
    wdt_enable(WDTO_2S);
    if(read == good_code) {
        OUTPUT_PORT |= _BV(OPTOCOUPLER_OUTPUT) | _BV(STATUS_LED_OUTPUT);
        _delay_ms(1000);
        wdt_reset();
        _delay_ms(1000);
        wdt_reset();
        _delay_ms(1000);
        wdt_reset();
        OUTPUT_PORT &= ~(_BV(OPTOCOUPLER_OUTPUT) | _BV(STATUS_LED_OUTPUT));
    } else {
        audio_bad();
    }
    wdt_disable();

    blink_morse(read);
}

static void handle_program_input(void) {
    uint8_t len = pulse_length(PROGRAM_INPUT, 0, PULSE_MAX);

    if(len == PULSE_MAX) { // timeout occured, enter programming mode
        while(bit_is_clear(INPUT_PIN, PROGRAM_INPUT)) {
            OUTPUT_PORT ^= _BV(STATUS_LED_OUTPUT);
            _delay_ms(50);
        }

        // turn led off
        OUTPUT_PORT &= ~_BV(STATUS_LED_OUTPUT);

        uint8_t read = read_morse(PROGRAM_INPUT, 0, PULSE_MAX);

        // ignore empty code
        if(!read) {
            OUTPUT_PORT |= _BV(STATUS_LED_OUTPUT);
            _delay_ms(1000);

            return;
        }

        _delay_ms(200);

        int i = 10;
        while(--i) {
            OUTPUT_PORT ^= _BV(STATUS_LED_OUTPUT);
            _delay_ms(50);
        }
        _delay_ms(200);

        blink_morse(read);

        eeprom_update_byte(&CODE_EEPROM, read);
        good_code = read;
    } else if(len > 3) {
        blink_morse(good_code);
    }
}

int main(void) {
    cli();
    _delay_ms(500); // for power stabilization

    // configure input ports
    INPUT_DDR = 0; // all inputs
    INPUT_PORT = _BV(PROGRAM_INPUT) | _BV(CALL_INPUT); // set internal pullup for program button and CALL

    // configure output ports
    OUTPUT_PORT = 0; // set all to low
    OUTPUT_DDR = _BV(OPTOCOUPLER_OUTPUT) | _BV(AUDIO_OUTPUT) | _BV(STATUS_LED_OUTPUT);

    // read code from eeprom and store in RAM (we will save some mA later)
    good_code = eeprom_read_byte(&CODE_EEPROM);

    // action!
    sei();

    // blink that we are ready
    OUTPUT_PORT ^= _BV(STATUS_LED_OUTPUT);
    _delay_ms(100);
    OUTPUT_PORT ^= _BV(STATUS_LED_OUTPUT);

    // main loop
    for(;;) {
        if(bit_is_clear(INPUT_PIN, CALL_INPUT)) {
            handle_call_input();
            OUTPUT_PORT ^= _BV(STATUS_LED_OUTPUT);
            _delay_ms(10);
            OUTPUT_PORT ^= _BV(STATUS_LED_OUTPUT);
        } else if(bit_is_clear(INPUT_PIN, PROGRAM_INPUT)) {
            handle_program_input();
        }
    }
}

// well... the following stuff looks awful :<
ISR(TIMER0_OVF_vect) {
    timer_count++;
    // OUTPUT_PORT ^= _BV(STATUS_LED_OUTPUT);
}

#include <avr/interrupt.h>
#include "WProgram.h"
#include "PulseRadio.h"

volatile unsigned int ICR4_old;
volatile unsigned char ppm_counter = 0;
volatile uint16_t pwm_raw[8] = {2400, 2400, 2400, 2400, 2400, 2400, 2400, 2400};
volatile unsigned char radio_status = 0;

ISR(TIMER4_CAPT_vect)
{
    unsigned int pulse;
    unsigned int pulse_width;
    
    pulse = ICR4;
    
    if (pulse < ICR4_old) {
        pulse_width = (pulse + 40000) - ICR4_old;
    }
    else {
        pulse_width = pulse - ICR4_old;
    }
    
    if (pulse_width > 8000) {
        ppm_counter = 0;
    }
    else {
        if (ppm_counter < (sizeof(pwm_raw) / sizeof(pwm_raw[0]))) {
            pwm_raw[ppm_counter++] = pulse_width;
            if (ppm_counter >= NUM_CHANNELS) {
                radio_status = 1;
            }
        }
    }
    
    ICR4_old = pulse;
}

PulseRadio_Class::PulseRadio_Class()
{
    // constructor...
}

void PulseRadio_Class::init()
{
    //
    // Initialize Timer1
    // Provides PWM outputs on pins 11, 12, and 13
    // 50Hz frequency
    //
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    
    TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1);
    TCCR1B = (1 << WGM13) | (1 << WGM12)  | (1 << CS11);
    OCR1A  = 2000;
    OCR1B  = 2000;
    OCR1C  = 2000;
    ICR1   = 40000;
    
    //
    // Initialize Timer 3
    // Provides PWM outputs on pins 2, 3, and 4
    // 50Hz frequency
    //
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    
    TCCR3A = (1 << WGM31) | (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1);
    TCCR3B = (1 << WGM33) | (1 << WGM32)  | (1 << CS31);
    OCR3A  = 2000;
    OCR3B  = 2000;
    OCR3C  = 2000;
    ICR3   = 40000;
    
    //
    // Initialize Timer 5
    // Provides PWM outputs on pins 44, 45, and 46
    // 50Hz frequency
    //
    pinMode(44, OUTPUT);
    pinMode(45, OUTPUT);
    pinMode(46, OUTPUT);
    
    TCCR5A = (1 << WGM51) | (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1);
    TCCR5B = (1 << WGM53) | (1 << WGM52)  | (1 << CS51);
    OCR5A  = 2000;
    OCR5B  = 2000;
    OCR5C  = 2000;
    ICR5   = 40000;
    
    //
    // Initialize Timer 4
    // Provides PWM outputs on pins 7 and 8, and PPM input on pin 49
    // 50Hz frequency
    //
    pinMode(49, INPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    
    TCCR4A = (1 << WGM40) | (1 << WGM41) | (1 << COM4C1) | (1 << COM4B1) | (1 << COM4A1);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41)   | (1 << ICES4);
    OCR4A  = 40000;
    OCR4B  = 2000;
    OCR4C  = 2000;
    
    //
    // enable input capture interrupt
    //
    TIMSK4 |= (1 << ICIE4);
}

void PulseRadio_Class::outputCh(unsigned char chan, uint16_t pos)
{
    pos = constrain(pos, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pos <<= 1;
    
    switch (chan) {
        case 0: OCR5B = pos; break;
        case 1: OCR5C = pos; break;
        case 2: OCR1B = pos; break;
        case 3: OCR1C = pos; break;
        case 4: OCR4C = pos; break;
        case 5: OCR4B = pos; break;
        case 6: OCR3C = pos; break;
        case 7: OCR3B = pos; break;
    }
}

uint16_t PulseRadio_Class::inputCh(unsigned char chan)
{
    uint16_t result;
    uint16_t result_check;
    
    //
    // Sanity check to make sure the value is not being written as we are
    // trying to read it, since stopping interrupts would be too costly.
    //
    result = pwm_raw[chan] >> 1;
    result_check = pwm_raw[chan] >> 1;
    
    if (result != result_check) {
        result = pwm_raw[chan] >> 1;
    }
    
    result = constrain(result, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    radio_status = 0;
    
    return result;
}

unsigned char PulseRadio_Class::ready()
{
    return radio_status;
}

PulseRadio_Class PulseRadio;


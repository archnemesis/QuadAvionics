#ifndef PULSERADIO
#define PULSERADIO

#define NUM_CHANNELS 8
#define MIN_PULSE_WIDTH 900
#define MAX_PULSE_WIDTH 2100

#include <inttypes.h>

class PulseRadio_Class
{
    public:
        PulseRadio_Class();
        void init();
        void outputCh(unsigned char chan, uint16_t pos);
        uint16_t inputCh(unsigned char chan);
        unsigned char ready();
};

extern PulseRadio_Class PulseRadio;

#endif

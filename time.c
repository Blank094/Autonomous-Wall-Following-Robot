#include "time.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Timer2 configured in CTC mode with prescaler 64 and OCR2A=249 -> 1ms tick
static volatile uint32_t g_ms_count = 0;

void init_time(void) {
    // CTC mode using OCR2A as TOP
    TCCR2A = (1 << WGM21);
    // prescaler 64 (CS22=1, CS21=0, CS20=0)
    TCCR2B = (1 << CS22);
    OCR2A = 249; // (16MHz / 64) = 250kHz -> 250 ticks = 1ms -> OCR2A = 249
    TIMSK2 |= (1 << OCIE2A); // enable compare A interrupt
}

ISR(TIMER2_COMPA_vect) {
    g_ms_count++;
}

uint32_t millis(void) {
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        t = g_ms_count;
    }
    return t;
}

uint32_t micros(void) {
    uint32_t m;
    uint8_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        m = g_ms_count;
        t = TCNT2;
    }
    // Timer2 tick is 4us with prescaler 64 at 16MHz
    return (m * 1000UL) + ((uint32_t)t * 4UL);
}

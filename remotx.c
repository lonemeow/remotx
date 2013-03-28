#include <avr/io.h>
#include <avr/interrupt.h>

#include "globals.h"

#define CHANNELS 7

/* Clock runs at 2MHz to avoid overflowing unsigned 16bit in PPM output */
#define USEC_TO_CYCLES(x) (x*2)

/* Our hardware has inverting buffers for PWM input */
#define PWM_INVERTED

/* PWM input routines */

struct pwm_entry
{
    uint16_t time;
    uint8_t pins;
    uint8_t pad;
};

#define PWM_PULSE_MIN_WIDTH USEC_TO_CYCLES(800)
#define PWM_PULSE_MAX_WIDTH USEC_TO_CYCLES(2200)

extern struct pwm_entry pwm_buffer[PWM_BUFFER_SIZE] __attribute__((aligned(256)));
volatile uint8_t pwm_overflow = 0;
static uint16_t pwm_rise_times[CHANNELS] = { 0 };

#ifdef PWM_INVERTED
#define IS_RISING_EDGE(x) (!(x))
#else
#define IS_RISING_EDGE(x) (x)
#endif

static void pwm_process(uint16_t *buffer)
{
    while (pwm_read_pos != pwm_write_pos)
    {
        static uint8_t pins = 0;
        uint16_t time = pwm_buffer[pwm_read_pos].time;
        uint8_t current = pwm_buffer[pwm_read_pos].pins;
        uint8_t changed = pins ^ current;
        pins = current;

        uint8_t pin = _BV(7);

        for (int i=0; i<CHANNELS; i++)
        {
            if (changed & pin)
            {
                if (IS_RISING_EDGE(current & pin))
                {
                    /* Rising edge, store time */
                    pwm_rise_times[i] = time;
                }
                else
                {
                    /* Falling edge, calculate pulse length */
                    uint16_t pulse_width = time - pwm_rise_times[i];

                    if (pulse_width < PWM_PULSE_MIN_WIDTH)
                        pulse_width = PWM_PULSE_MIN_WIDTH;
                    else if (pulse_width > PWM_PULSE_MAX_WIDTH)
                        pulse_width = PWM_PULSE_MAX_WIDTH;

                    buffer[i] = pulse_width;
                }
            }

            pin >>= 1;
        }

        /* This is not atomic, so protect it with cli/sei */
        cli();
        pwm_read_pos = (pwm_read_pos + 1) & PWM_BUFFER_MASK;
        sei();
    }
}

#define PWM_PINS (_BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7))

static void pwm_init(uint16_t *buffer)
{
    /* PD1-PD7 to input */
    DDRD &= ~PWM_PINS;
    /* Pin Change Interrupt 2 enable */
    PCICR |= _BV(PCIE2);
    /* Pin Change Interrupt 2 mask to PD1-PD7 */
    PCMSK2 |= PWM_PINS;

    pwm_read_pos = 0;
    pwm_write_pos = 0;

    for (int i=0; i<CHANNELS; i++)
        buffer[i] = USEC_TO_CYCLES(1500);
}

/* PPM output routines */

#define PPM_FRAME_LENGTH (22500U*2)
#define PPM_PULSE_LENGTH (400*2)

/* Falling/Rising edge for each channel and sync pulse */
#define PPM_BUFFER_SIZE ((CHANNELS+1) * 2)

uint16_t ppm_buffer[PPM_BUFFER_SIZE];
uint8_t ppm_position;

ISR(TIMER1_COMPA_vect)
{
    OCR1A += ppm_buffer[ppm_position++];
}

static void ppm_process(uint16_t *buffer)
{
    if (ppm_position < PPM_BUFFER_SIZE)
        return;

    uint16_t *ptr = ppm_buffer;
    uint16_t frame_remaining = PPM_FRAME_LENGTH;

    for (int i=0; i<CHANNELS; i++)
    {
        *ptr++ = PPM_PULSE_LENGTH;
        frame_remaining -= PPM_PULSE_LENGTH;

        *ptr++ = buffer[i] - PPM_PULSE_LENGTH;
        frame_remaining -= buffer[i] - PPM_PULSE_LENGTH;
    }

    *ptr++ = PPM_PULSE_LENGTH;
    frame_remaining -= PPM_PULSE_LENGTH;

    *ptr++ = frame_remaining;

    /* XXX Overflow handling XXX */

    ppm_position = 0;
}

static void ppm_start(uint16_t *buffer)
{
    ppm_process(buffer);

    DDRB |= _BV(1);  /* PB1 to output */
    PORTB |= _BV(1); /* PB1 high */

    OCR1A = TCNT1 + ppm_buffer[0];
    ppm_position = 1;

    TIMSK1 |= _BV(OCIE1A);
    TCCR1A |= _BV(COM1A0);
}

/* Program entry point */

int main(void)
{
    uint16_t pulse_widths[CHANNELS];

    TCCR1A = 0;
    TCCR1B = _BV(CS11); /* Clk/8 */

    pwm_init(pulse_widths);

    sei(); /* Ready to handle interrupts */

    ppm_start(pulse_widths);

    while (1)
    {
        pwm_process(pulse_widths);

        ppm_process(pulse_widths);
    }
}

/* vim: set shiftwidth=4 expandtab: */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "globals.h"

#define PWM_CHANNELS 6

struct pwm_entry
{
    uint16_t time;
    uint8_t pins;
    uint8_t pad;
};

#define PWM_USEC_TO_CYCLES(x) (x*2)

#define PWM_PULSE_MIN_WIDTH PWM_USEC_TO_CYCLES(800)
#define PWM_PULSE_MAX_WIDTH PWM_USEC_TO_CYCLES(2200)

extern struct pwm_entry pwm_buffer[PWM_BUFFER_SIZE] __attribute__((aligned(256)));
volatile uint8_t pwm_overflow = 0;
uint16_t pwm_rise_times[PWM_CHANNELS] = { 0 };

/* Our hardware has inverting buffers for PWM input */
#define PWM_INVERTED

#ifdef PWM_INVERTED
#define IS_RISING_EDGE(x) (!(x))
#else
#define IS_RISING_EDGE(x) (x)
#endif

void pwm_process(uint16_t *buffer)
{
    while (pwm_read_pos != pwm_write_pos)
    {
        static uint8_t pins = 0;
        uint16_t time = pwm_buffer[pwm_read_pos].time;
        uint8_t current = pwm_buffer[pwm_read_pos].pins;
        uint8_t changed = pins ^ current;
        pins = current;

        uint8_t pin = _BV(7);

        for (int i=0; i<PWM_CHANNELS; i++)
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

#define CHANNELS 6

uint16_t ppm_widths[CHANNELS] = { 0 };

#define PPM_FRAME_LENGTH (22500U*2)
#define PPM_PULSE_LENGTH (400*2)

uint16_t ppm_buffer[(CHANNELS+1)*2];
uint8_t ppm_position;

static void ppm_update(void)
{
    uint16_t *ptr = ppm_buffer;
    uint16_t frame_remaining = PPM_FRAME_LENGTH;

    for (int i=0; i<CHANNELS; i++)
    {
        *ptr++ = PPM_PULSE_LENGTH;
        frame_remaining -= PPM_PULSE_LENGTH;

        *ptr++ = ppm_widths[i] - PPM_PULSE_LENGTH;
        frame_remaining -= ppm_widths[i] - PPM_PULSE_LENGTH;
    }

    *ptr++ = PPM_PULSE_LENGTH;
    frame_remaining -= PPM_PULSE_LENGTH;

    *ptr++ = frame_remaining;
}

ISR(TIMER1_COMPA_vect)
{
    OCR1A += ppm_buffer[ppm_position++];

    if (ppm_position == 14) /* XXX */
    {
        /* This should be safe vrt. recursion as the sync pulse at end of PPM frame
         * should always be long enough to let us finish. */
        sei();
        ppm_update();
        ppm_position = 0;
    }
}

void ppm_start(void)
{
    ppm_update();

    OCR1A = TCNT1 + ppm_buffer[0];
    ppm_position = 1;

    DDRB |= _BV(PINB1); /* PB1 to output */

    TIMSK1 |= _BV(OCIE1A);
    TCCR1A |= _BV(COM1A0);
}

#define PWM_PINS (_BV(1) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7))

void pwm_init(uint16_t *buffer)
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
        buffer[i] = PWM_USEC_TO_CYCLES(1500);
}

int main(void)
{
    uint16_t pulse_widths[CHANNELS];

    TCCR1A = 0;
    TCCR1B = _BV(CS11); /* Clk/8 */

    pwm_init(pulse_widths);

    ppm_start();

    sei(); /* Ready to handle interrupts */

    while (1)
    {
        pwm_process(pulse_widths);

        for (int i=0; i<CHANNELS; i++)
            ppm_widths[i] = pulse_widths[i];
    }

    return 0;
}

/* vim: set shiftwidth=4 expandtab: */

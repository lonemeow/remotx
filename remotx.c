/*
 *  This file is part of remotx.
 *
 *  remotx is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  remotx is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with remotx.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Copyright 2013 Ilpo "LoneWolf" Ruotsalainen <lonewolf@iki.fi>
*/
#include <avr/io.h>
#include <avr/interrupt.h>

#include "globals.h"

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

extern uint16_t ppm_buffer[PPM_BUFFER_SIZE]  __attribute__((aligned(256)));

static void ppm_process(uint16_t *buffer)
{
    if (ppm_position < PPM_BUFFER_SIZE)
        return;

    uint16_t *ptr = ppm_buffer;
    uint16_t frame_remaining = PPM_FRAME_LENGTH;
    static uint16_t sum = 0;

    for (int i=0; i<CHANNELS; i++)
    {
        *ptr++ = sum + PPM_PULSE_LENGTH;
        frame_remaining -= PPM_PULSE_LENGTH;
        sum += PPM_PULSE_LENGTH;

        *ptr++ = sum + buffer[i] - PPM_PULSE_LENGTH;
        frame_remaining -= buffer[i] - PPM_PULSE_LENGTH;
        sum += buffer[i] - PPM_PULSE_LENGTH;
    }

    *ptr++ = sum + PPM_PULSE_LENGTH;
    frame_remaining -= PPM_PULSE_LENGTH;
    sum += PPM_PULSE_LENGTH;

    *ptr++ = sum + frame_remaining;
    sum += frame_remaining;

    /* XXX Overflow handling XXX */

    ppm_position = 0;
}

static void ppm_start(uint16_t *buffer)
{
    ppm_process(buffer);

    DDRB |= _BV(1);  /* PB1 to output */
    PORTB |= _BV(1); /* PB1 high */

    OCR1A = ppm_buffer[0];
    TCNT1 = 0;
    ppm_position = 1;

    TIMSK1 |= _BV(OCIE1A);
    TCCR1A |= _BV(COM1A0);
}

/* S-Bus routines */

#define BAUD 100000
#include <util/setbaud.h>

static void sbus_init(void)
{
    /* Set baud rate */
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
#warning USART using 2X mode
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif

    /* RX interrupt enabled, receiver enabled, data size = 8 */
    UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | (3 << UCSZ00);

    /* Clear errors */
    UCSR0A  &= ~(_BV(FE0) | _BV(DOR0) | _BV(UPE0));
}

ISR(USART_RX_vect)
{
}

/* Program entry point */

int main(void)
{
    uint16_t pulse_widths[CHANNELS];

    TCCR1A = 0;
    TCCR1B = CLK_DIV;

    sbus_init();

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

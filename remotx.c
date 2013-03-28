#include <avr/io.h>
#include <avr/interrupt.h>

#include "globals.h"

#define PWM_CHANNELS 6
#define PINS (_BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

struct pwm_entry
{
	uint16_t time;
	uint8_t pins;
	uint8_t pad;
};

extern struct pwm_entry pwm_buffer[PWM_BUFFER_SIZE] __attribute__((aligned(256)));
volatile uint8_t pwm_overflow = 0;
uint16_t pwm_pulse_width[PWM_CHANNELS] = { 0 };
uint16_t pwm_rise_times[PWM_CHANNELS] = { 0 };

void pwm_process(void)
{
	while (1)
	{
		if (pwm_read_pos == pwm_write_pos)
			break;

		static uint8_t pins = 0;
		uint16_t time = pwm_buffer[pwm_read_pos].time;
		uint8_t current = pwm_buffer[pwm_read_pos].pins;
		uint8_t changed = pins ^ current;
		pins = current;

		uint8_t pin = _BV(PC0);

		for (int i=0; i<PWM_CHANNELS; i++)
		{
			if (changed & pin)
			{
				if (current & pin)
				{
					/* Rising edge */
					pwm_rise_times[i] = time;
				}
				else
				{
					/* Falling edge */
					pwm_pulse_width[i] = time - pwm_rise_times[i];
				}
			}

			pin <<= 1;
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

int main(void)
{
	TCCR1A = 0;
	TCCR1B = _BV(CS11); /* Clk/8 */

	DDRC = 0;
	PCICR = _BV(PCIE1);
	PCMSK1 = _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11) | _BV(PCINT12) | _BV(PCINT13);

	pwm_read_pos = 0;
	pwm_write_pos = 0;

	sei(); /* Ready to handle interrupts */

	ppm_start();

	while (1)
	{
		pwm_process();

		for (int i=0; i<CHANNELS; i++)
			ppm_widths[i] = pwm_pulse_width[i];
	}

	return 0;
}

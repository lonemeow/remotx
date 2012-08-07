#include <avr/io.h>
#include <avr/interrupt.h>

#include "globals.h"

#define PWM_CHANNELS 6
#define PINS (_BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

/* Debug stuff starts */
#define BAUD 115200
#include <util/setbaud.h>

void setup_serial(void)
{
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */  
}

void serial_putchar(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);

	UDR0 = c;
}

void serial_putstring(const char *s)
{
	while (*s)
		serial_putchar(*s++);
}

void serial_putuint16(uint16_t v)
{
	char tmp[8];
	unsigned char i = 0;

	do
	{
		char digit = (v % 10) + '0';
		tmp[i++] = digit;
		v /= 10;
	}
	while (v);

	while (i > 0)
		serial_putchar(tmp[--i]);
}

/* Debug stuff ends */

struct pwm_entry
{
	uint16_t time;
	uint8_t pins;
	uint8_t pad;
};

struct pwm_entry pwm_buffer[PWM_BUFFER_SIZE];
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
	setup_serial();

	TCCR1A = 0;
	TCCR1B = _BV(CS11); /* Clk/8 */

	DDRB = _BV(PINB5); /* B5 to output */

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

		if (pwm_overflow)
			PORTB |= _BV(PB5); /* LED indicates error */
	}

	return 0;
}

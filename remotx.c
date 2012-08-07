#define BAUD 9600

#include <avr/io.h>
#include <avr/interrupt.h>


#include "globals.h"

#define PWM_CHANNELS 6
#define PINS (_BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

/* Debug stuff starts */
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

uint16_t pulse_widths[PWM_CHANNELS] = { 0 };
uint16_t rise_times[PWM_CHANNELS] = { 0 };

#if 0
ISR(PCINT1_vect)
{
	uint16_t time = TCNT1;

	uint8_t current_pins = PINC;
	uint8_t changed_pins = last_pins ^ current_pins;
	last_pins = current_pins;

	uint8_t pin = _BV(PC0);

	for (int i=0; i<PWM_CHANNELS; i++)
	{
		if (changed_pins & pin)
		{
			if (current_pins & pin)
			{
				/* Rising edge */
				rise_times[i] = time;
			}
			else
			{
				/* Falling edge */
				pulse_widths[i] = (time - rise_times[i]) >> 1;
			}
		}

		pin <<= 1;
	}

	intr_time = TCNT1 - time;
}
#endif

struct pwm_entry
{
	uint16_t time;
	uint8_t pins;
	uint8_t pad;
};

struct pwm_entry pwm_buffer[PWM_BUFFER_SIZE];
uint8_t pwm_overflow = 0;
uint16_t pwm_pulse_width[PWM_CHANNELS];

void pwm_process(void)
{
	while (pwm_read_pos != pwm_write_pos)
	{
		pwm_read_pos = (pwm_read_pos + 1) & PWM_BUFFER_MASK;
	}
}

int main(void)
{
	setup_serial();

	TCCR1A = 0;
	TCCR1B = _BV(CS11); /* Clk/8 */

	DDRC = 0;
	PCICR = _BV(PCIE1);
	PCMSK1 = _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11) | _BV(PCINT12) | _BV(PCINT13);

	sei(); /* Ready to handle interrupts */

	serial_putstring("testink: ");
	serial_putuint16(12765);
	serial_putstring("\r\n");

	uint16_t last = TCNT1;

	while (1)
	{
		pwm_process();

		if (TCNT1 - last > 1000)
		{
			last = TCNT1;

			if (!pwm_overflow)
			{
				serial_putstring("C1: ");
				serial_putuint16(pwm_pulse_width[0]);
				serial_putstring(" C2: ");
				serial_putuint16(pwm_pulse_width[1]);
				serial_putstring(" C2: ");
				serial_putuint16(pwm_pulse_width[1]);
				serial_putstring(" C2: ");
				serial_putuint16(pwm_pulse_width[1]);
				serial_putstring(" C2: ");
				serial_putuint16(pwm_pulse_width[1]);
				serial_putstring(" C2: ");
				serial_putuint16(pwm_pulse_width[1]);
				serial_putstring("\r\n");
			}
			else
				serial_putstring("PWM overflow\r\n");
		}
	}

	return 0;
}

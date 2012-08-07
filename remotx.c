#define BAUD 9600

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/setbaud.h>
#include <util/delay.h>

#include <stdio.h>

#define PWM_CHANNELS 6
#define PINS (_BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

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

void serial_putchar(char c, FILE *stream)
{
	if (c == '\n')
		serial_putchar('\r', stream);

	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}

FILE serial_output = FDEV_SETUP_STREAM(serial_putchar, NULL, _FDEV_SETUP_WRITE);

volatile uint16_t timer_overflow = 0;

ISR(TIMER1_OVF_vect)
{
	timer_overflow++;
}

uint16_t pulse_widths[PWM_CHANNELS] = { 0 };
uint8_t last_pins = 0;
uint16_t rise_times[PWM_CHANNELS] = { 0 };

uint16_t intr_time = 0;

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

#define BUFFER_SIZE 8
#define BUFFER_MASK 7

uint16_t time_buffer[BUFFER_SIZE];
uint8_t pin_buffer[BUFFER_SIZE];

uint8_t write_pos = 0;
uint8_t read_pos = 0;

uint8_t overflow = 0;

ISR(PCINT0_vect)
{
	time_buffer[write_pos] = TCNT1;
	pin_buffer[write_pos] = PINC;

	write_pos = (write_pos + 1) & BUFFER_MASK;

	if (write_pos == read_pos)
		overflow = 1;
}

int main(void)
{
	setup_serial();
	stdout = &serial_output;

	TCCR1A = 0;
	TCCR1B = _BV(CS11); /* Clk/8 */
	TIMSK1 = _BV(TOIE1); /* Enable Timer0 overflow interrupt */

	DDRC = 0;
	PCICR = _BV(PCIE1);
	PCMSK1 = _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11) | _BV(PCINT12) | _BV(PCINT13);

	sei(); /* Ready to handle interrupts */

	while (1)
	{
		printf("C1: %4hu C2: %4hu C3: %4hu C4: %4hu C5: %4hu C6: %4hu intr: %hu\n", pulse_widths[0], pulse_widths[1], pulse_widths[2], pulse_widths[3], pulse_widths[4], pulse_widths[5], intr_time);
		_delay_ms(100);
	}

	return 0;
}

#ifndef GLOBALS_H
#define GLOBALS_H

#define CHANNELS 7

#if F_CPU == 16000000
/* Clock runs at 2MHz to avoid overflowing unsigned 16bit in PPM output */
#define CLK_DIV _BV(CS11) /* Clk/8 = 2MHz */
#define USEC_TO_CYCLES(x) (x*2)
#else
#error Unsupported CPU frequency
#endif

/* Our hardware has inverting buffers for PWM input */
#define PWM_INVERTED

/* This has to be a power of 2 */
/* PWM_BUFFER_SIZE * PWM_ENTRY_SIZE has to be less than 256 */
#define PWM_BUFFER_SIZE 8
#define PWM_BUFFER_MASK (PWM_BUFFER_SIZE - 1)

/* Has to match sizeof(pwn_entry) */
#define PWM_ENTRY_SIZE 4

#define PPM_BUFFER_SIZE ((CHANNELS+1) * 2)

#ifdef __ASSEMBLER__

#define pwm_write_pos r16
#define pwm_read_pos  r17
#define ppm_position  r15

#else

volatile register uint8_t ppm_position asm("r15");
volatile register uint8_t pwm_write_pos asm("r16");
volatile register uint8_t pwm_read_pos asm("r17");

#endif

#endif

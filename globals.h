#ifndef GLOBALS_H
#define GLOBALS_H

/* This has to be a power of 2 */
/* PWM_BUFFER_SIZE * PWM_ENTRY_SIZE has to be less than 256 */
#define PWM_BUFFER_SIZE 8
#define PWM_BUFFER_MASK (PWM_BUFFER_SIZE - 1)

/* Has to match sizeof(pwn_entry) */
#define PWM_ENTRY_SIZE 4

#ifdef __ASSEMBLER__

#define pwm_SREG_save r2
#define pwm_scratch1  r3
#define pwm_scratch2  r4
#define pwm_scratch3  r5
#define pwm_write_pos r16
#define pwm_read_pos  r17

#else

/* Registers reserved for PWM interrupt handler */
register uint8_t pwm_SREG_save asm("r2");
register uint8_t pwm_scratch1 asm("r3");
register uint8_t pwm_scratch2 asm("r4");
register uint8_t pwm_scratch3 asm("r5");

register uint8_t pwm_write_pos asm("r16");
register uint8_t pwm_read_pos asm("r17");

#endif

#endif

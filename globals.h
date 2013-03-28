#ifndef GLOBALS_H
#define GLOBALS_H

/* This has to be a power of 2 */
/* PWM_BUFFER_SIZE * PWM_ENTRY_SIZE has to be less than 256 */
#define PWM_BUFFER_SIZE 8
#define PWM_BUFFER_MASK (PWM_BUFFER_SIZE - 1)

/* Has to match sizeof(pwn_entry) */
#define PWM_ENTRY_SIZE 4

#ifdef __ASSEMBLER__

#define pwm_write_pos r16
#define pwm_read_pos  r17

#else

volatile register uint8_t pwm_write_pos asm("r16");
volatile register uint8_t pwm_read_pos asm("r17");

#endif

#endif

#include <avr/io.h>

#include "globals.h"

.extern pwm_buffer
.extern pwm_overflow

.global PCINT0_vect
PCINT1_vect:
        in      pwm_SREG_save, _SFR_IO_ADDR(SREG)
        push    r30
        push    r31

        lds     pwm_scratch1, _SFR_MEM_ADDR(TCNT1L)
        lds     pwm_scratch2, _SFR_MEM_ADDR(TCNT1H)

        in      pwm_scratch3, _SFR_IO_ADDR(PINC)

        mov     r30, pwm_write_pos
        ldi     r31, 0

        /* Can not overflow if BUFFER_SIZE is chosen properly */
        lsl     r30
        lsl     r30 /* sizeof(pin_change_entry) = 4 */
 
        subi    r30, lo8(-(pwm_buffer)) /* There's no addi/adci */
        sbci    r31, hi8(-(pwm_buffer))
 
        st      Z+, pwm_scratch1 /* Store TCNT1 */
        st      Z+, pwm_scratch2

        st      Z+, pwm_scratch3 /* Store PINC */

        inc     pwm_write_pos
        andi    pwm_write_pos, PWM_BUFFER_MASK

        cp      pwm_read_pos, pwm_write_pos
        breq    2f

1:
        pop     r31
        pop     r30
        out     _SFR_IO_ADDR(SREG), pwm_SREG_save
        reti

2:
        ldi     r30, 1
        sts     pwm_overflow, r30
        rjmp    1b

/* vim: set expandtab: */

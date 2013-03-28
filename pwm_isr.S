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

#include "globals.h"

.extern pwm_overflow

.section .bss
.global pwm_buffer
.balign 256
pwm_buffer:
.skip PWM_ENTRY_SIZE * PWM_BUFFER_SIZE

.section .text

.global PCINT2_vect
PCINT2_vect:
        push    r0                                                      /* 2 */
        in      r0, _SFR_IO_ADDR(SREG)                                  /* 1 */

        push    r1                                                      /* 2 */
        push    r2                                                      /* 2 */
        push    r3                                                      /* 2 */

        lds     r1, _SFR_MEM_ADDR(TCNT1L)                               /* 2 */
        lds     r2, _SFR_MEM_ADDR(TCNT1H)                               /* 2 */
        in      r3, _SFR_IO_ADDR(PIND)                                  /* 1 */

        /* Grab a "local" copy of pwm_write_pos to use after unmasking
           interrupts */
        push    r30                                                     /* 2 */
        mov     r30, pwm_write_pos                                      /* 1 */

        inc     pwm_write_pos                                           /* 1 */
        andi    pwm_write_pos, PWM_BUFFER_MASK                          /* 1 */

        cp      pwm_read_pos, pwm_write_pos                             /* 1 */

        /* Next instruction after "sei" will be executed before interrupts are
           serviced, so it should be a 1-cycle instruction for best results */
        sei                                                             /* 1 */

        /* Uses flags from earlier "cp" */
        breq    2f                                                      /* 1 */
                                                                 /* total 22 */

        /* It is safe to run the following code with interrupts unmasked
           because we have already calculated the position in buffer we are
           going to write to and updated the write pointer.

           This cannot race with the reading code, as the reading code will
           only be able to execute once all the nested interrupt handlers have
           returned. */

        /* Can not overflow if BUFFER_SIZE is chosen properly */
        /* See comment in globals.h */
        lsl     r30
        lsl     r30 /* sizeof(pin_change_entry) = 4 */
 
        /* pwm_buffer is aligned to 256 bytes so just ignore the low byte */
        push    r31
        ldi     r31, hi8(pwm_buffer)
 
        st      Z+, r1 /* Store TCNT1 */
        st      Z+, r2

        st      Z+, r3 /* Store PIND */

        pop     r31
1:
        pop     r30
        pop     r3
        pop     r2
        pop     r1

        out     _SFR_IO_ADDR(SREG), r0
        pop     r0

        reti

2:
        ldi     r30, 1
        sts     pwm_overflow, r30
        rjmp    1b

/* vim: set expandtab: */

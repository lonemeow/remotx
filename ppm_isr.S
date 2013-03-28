#include <avr/io.h>

#include "globals.h"

.section .bss
.global ppm_buffer
.balign 256
ppm_buffer:
.skip PPM_BUFFER_SIZE * 2 /* sizeof(uint16_t) */

.section .text

.global TIMER1_COMPA_vect
TIMER1_COMPA_vect:
        push    r0                                                      /* 2 */
        in      r0, _SFR_IO_ADDR(SREG)                                  /* 1 */

        push    r31                                                     /* 2 */
        push    r30                                                     /* 2 */

        ldi     r31, hi8(ppm_buffer)                                    /* 1 */
        mov     r30, ppm_position                                       /* 1 */
        lsl     r30 /* sizeof(uint16_t) = 2 */                          /* 1 */

        push    r1                                                      /* 2 */
        push    r2                                                      /* 2 */

        /* Load 16bit value from ppm_buffer[ppm_position] */
        ld      r1, Z                                                   /* 2 */
        ldd     r2, Z+1                                                 /* 2 */

        /* Store value to OCR1A */
        sts     _SFR_MEM_ADDR(OCR1AH), r2                               /* 2 */
        sts     _SFR_MEM_ADDR(OCR1AL), r1                               /* 2 */

        /* Next instruction after "sei" will be executed before interrupts are
           serviced, so it should be a 1-cycle instruction for best results */
        sei                                                             /* 1 */

        inc     ppm_position                                            /* 1 */
                                                                 /* total 24 */

        pop     r2
        pop     r1

        pop     r30
        pop     r31

        out     _SFR_IO_ADDR(SREG), r0
        pop     r0

        reti

/* vim: set expandtab: */

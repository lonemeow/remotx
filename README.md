remotx
======

What is it?
-----------

remotx is a PWM/S-Bus to PPM converter. It can be used to replace a
[Simstick](http://www.simstick.co.uk/) or to generate PPM input for a
multicopter flight controller or any other device that needs PPM input.

The project was originally conceived because Simstick had issues with Futaba
receivers, however they have since released the "Pro" version that should fix
those issues.

Author
------

Ilpo "LoneWolf" Ruotsalainen <lonewolf@iki.fi>

License
-------

remotx is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

remotx is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with remotx.  If not, see <http://www.gnu.org/licenses/>.

Hardware
--------

The code was originally developed on an Arduino Uno and should still work on
one with only minor changes. Schematics and board layout files for the current
reference hardware can be found from
<https://github.com/perttierkkila/ppm-converter>.

The hardware is built around an ATmega328p running at 16MHz, but the code could
be adapted to any Atmel CPU that can run at a high enough clock speed (8MHz is
probably the lowest you can get reasonable timings on, and even that will give
far less precision on PWM input) and has enough pins on a single PCINT group
available.

Default pin layout is as follows:

    PD7 - PWM input channel 1
    PD6 - PWM input channel 2
    PD5 - PWM input channel 3
    PD4 - PWM input channel 4
    PD3 - PWM input channel 5
    PD2 - PWM input channel 6
    PD1 - PWM input channel 7
    PD0 - S-Bus input signal
    PB1 - PPM output

You could also use pin C for PWM inputs, but that will only allow 6 PWM input
channels unless you are prepared to not have a RESET pin...

If your hardware does not have inverting buffers for PWM input or runs at a
different clock frequency you need to change the values defined in globals.h.

Code structure
--------------

The key design point was use interrupts for timing and minimize time spent with
interrupts disabled to improve measuring/output accuracy.

### PWM input

PWM input uses a hand crafted assembler interrupt handler (see pwm_isr.S) to
store input pin state and time into a ring buffer whenever an input pin changes
state. The interrupt handler is written in a way that allows it to safely
re-enable interrupts before it has finished storing the data to minimize
interrupt latency.

The actual processing to measure PWM pulse widths is done in `pwm_process()`
called from the main loop. It will look whether the previous PPM frame has
finished (ie. we're in the sync pulse now) and if so, calculate new OCR1A
values for next frame based on input pulse widths.

### S-Bus input

Not implemented yet.

### PPM output

PPM output is done using OCR1A to automatically toggle the output pin state. A
hand crafted interrupt routine (see ppm_isr.S) writes the pre-calculated timer
values to OCR1A.  The values are calculated in `ppm_process()` called from the
main loop.

TODO
----

 * S-Bus support
 * Automatic input detection
 * Improved error handling

<!-- vim: fo=a tw=80 colorcolumn=80 syntax=markdown :
-->

ATTiny10-powered temperatur/humidity sensor value printer
=========================================================

This firmware polls i2c-connected temperatur and humidity sensors via i2c and
prints relevant values in a special format via UART.

This being a trivial task, it uses a software implementation of i2c and of UART
that is not very cycle-performant (as all transfers are blocking), but very
timing accurate up to baudrates of 80k at 8MHz CPU clock and rather
power-efficient, as the CPU spends most time sleeping.

So this firmware is mostly an exercise in UART, i2c, AVR timers and sleep modes,
and coding in constrained spaces.

The original puprose was to connect this to a 433MHz transmitter and scatter
solar-powered sensors like this in the house, thus gaining a RF-based temperatur
sensor network. However, the 433MHz transceivers I obtained seem to be rather
unreliable. Also I assume that solar power might not be enough to yield good
measurement frequencies. If you want to do something like this, I suggest:

* Switch UART to inverted level (defines in sensor.c) and use a >=10k pull-down
resistor on the UART line. If you don't do this, the radio will transmit almost
all the time, except when sending '0' bits. The current setup is only
appropriate for testing with a UART transceiver for your computer.

* Increase time between transmissions. That could be done using a WDT interrupt
handler that will count down an integer every 8 seconds and thus can send e.g.
every n*8 seconds. WDT interrupt needs to be configured appropriately.


Authors
-------

David R. Piegdon <dgit@piegdon.de>


License
-------

All files in this repository are released unter the GNU General Public License
Version 3 or later. This includes any schematics, the routed PCB and the firmware
as well as the documentation and accompanying materials. See the file COPYING
for more information.



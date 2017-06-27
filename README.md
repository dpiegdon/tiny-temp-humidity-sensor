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


Authors
-------

David R. Piegdon <dgit@piegdon.de>


License
-------

All files in this repository are released unter the GNU General Public License
Version 3 or later. This includes any schematics, the routed PCB and the firmware
as well as the documentation and accompanying materials. See the file COPYING
for more information.



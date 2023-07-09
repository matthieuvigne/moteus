# Hardware design guidelines

 - To perform a star-shaped routing, V- denotes the power components ground, while GND stands
 for logical ground. Both nets are connected at the input power connector (XT-30 connector).
 - Power is provided by V+ (from power supply / battery) and by a 3.3V rail. TODO: what happens when
 JST is connected ?
 - The PCB has 4 layers:
    - Top: V+ ; include almost all components to facilitate manual assembly.
    - 2: V-
    - 3: Signals and 3.3V
    - Bottom: GND and signals.

DRC rules are from https://www.multi-circuit-boards.eu/en/support/pcb-data/kicad.html

Via-in-pad is explicitly allowed, cf. https://www.multi-circuit-boards.eu/en/pcb-design-aid/drills-throughplating/via-in-pad.html


Shunt resistor value:

 - System is designed for 10A nominal, 30A continuous
 - DRV8353 ADC takes a max value of 0.3V as input
 - Proposal: 1mOhm shunt resistor
   - Voltage range: 30mV ; with a x40 amplifier this gives 1.2V
   - Power loss: 0.9W max ; voltage loss 30mV.


Questions:
 - should all mounting holes be plated ?
 - why is pin 24 of DRV8353 tied to GNDS and not PWR_GND ?
 - soldering of the H-bridges: is it a good idea to increase thermal spoke width ? This seems to be recommanded by TI.
 - easiest design for 3.3V supply at debug (flash) port ?
 - H-Bridges: how to fill zones between pins ?
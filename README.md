# Hydrophones Microcontroller

Some important information for operation of hydrophones:

CONVST pwm signal output is resonsible for for setting the frequency at which the ADC performs conversions.

EOC signal is responsible for noting the availability of a single conversion results available on the 14 bit data bus

EOLC signal is an additional converion ready signal that activates simultaneously with fourth (and last) conversion result being ready on the data bus.

Additional notes on powering up hydrophones:
To activate ADC chip, configuration registers in the ADC are written to during startup via the 14 bit data bus. After startup, the pins change from data inputs to sending out data from the ADC.

Various micro pin mappings:
Data Pins:
Pins D0 - D13 on ADC map to PC0 - PC13 on Micro

Other Control & Misc Pins:
CS:PB0
RD:PB1
WR:PB2
CONVST:PB4
SHDN:PB5
ALLON:PB8
EOC:PA9
EOLC:PB7
SWO:PB3
CLK_CNTRL:PB10
CLK:PA8
LED:PB12

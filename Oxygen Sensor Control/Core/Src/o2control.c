/* 

Oxygen Sensor Control Code
Curtis Lam 2023

Functions:

checkid() SPI
- get the ID of the cj125

checkstat() SPI
- gets the current status of the chip

calibrate()
- remove the condensed water in the sensor
- enter the CJ125 calibration mode
- apply 2V to the heater for 4 seconds
- apply 8.5V to the heater, and within each consecutive second increase by 0.4V up to the battery voltage
- store UR value as a PWM reference point, quit CJ125 calibration
- change over to PWM heater control mode

run()
- get ADC of UR pin
- do a PID calculation and write a value to the heater pin

get_pinput()
- returns input for PID

get_poutput()
- returns output for PID

get_psetp()
- returns setpoint for PID

get_oxygen()
- gets oxygen value from UA pin

*/

#include "o2control.h"
#include <math.h>
#include <stdbool.h>

bool checkID() {
    
}
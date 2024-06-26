# esp32wp_controller
Irrigation controller.
It's a rework of generic esp32_controller dedicated to a small irrigation automation. It controls a water pump and 2 tap actuators for each irrigation zone.

## Introduction
It follows the same approach like [esp32_controller](https://github.com/ves011/esp32_controller).<br>
This implementation uses an old [ESP32 dev board with ESP32WROOM32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html). 
In addition, because the poor performance of internal ADC it uses an external [AD7811](https://www.analog.com/en/products/ad7811.html) connected over SPI. <br>
Has a small user interface built with a 2.8" LCD panel basd on ILI9341 and a rotary encoder switch.

## How it works

![plot](./doc/init.png)

>**Water program operation**<br>
>>The parameters for water program are stored in dv_program.txt file located in user partition on the ESP32 flash.
Each irrigation interval is defined by an entry in the file and has 7 fields: <br>
>>>**zone no, #wtering/day, start H, start min, stop H, stop min, q water, completion status, fault**<br>

>>with H (hour) in 24H format<br>
Completion status can be: NOT_STARTED, IN_PROGRESS, START_ERROR, STOP_ERROR, or ABORTED.<br>
>>>NOT_STARTED - is the state of program after reset status; only programs in NO_STARTED state can be started<br>
IN_PROGRESS - is the state of a program after started succesfully<br>
SATRT_ERROR - if start_watering function is not successful the program enter START_ERROR state; fault field provides the reason for failure<br>
STOP_ERROR - if stop_watering function is not successful the program enter STOP_ERROR state; fault field provides the reason for failure<br>
ABORTED - if the program is aborted by user<br>
PROG_SKIP - is completion status is set to PROG_SKIP water program is not executed

>>Completion status is reset to NOT_STARTED once per day at RESET_PROGRAM_H:RESET_PROGRAM_M time, only if != PROG_SKIP.<br>
**fault** field contains the error numerical value<br>
>>In my case, because the limited water pressure provided by pump, the logic allows only single zone irrigation at given point in time.<br><br>
**Water op commands**<br>
>The water op comand has the following format:
>>**dv arg1 arg2 ... argn**<br>
>>with arguments separated by spaces<br>
>>>**dv** - water op command<br>
>>>**arg1** - operation to be performed which can be<br>
>>>>**open** - open #dv provided by arg2<br>
>>>>**close** - close #dv provided by arg2<br>
>>>>**state** - provides state information for DVs and watering program<br>
>>>>**resetps** - reset completion status to NOT_STARTED for all programs which are not in PROG_SKIP state<br>
>>>>**program** - if no argument provided displays the list of watering programs. If required arguments are provided the specified program is updated<br>
<br><br>

>>When actual time gets equal or higher than specified start time of some zone and completion status is NOT_STARTED, start the program by:<br>
>>>ensure all the water taps are closed<br>
start water pump<br>
wait water pressure to be over max pressure limit<br>
>>>>if water pressure too low: stop water pump, abort program and return error<br>

>>>open tap for the zone<br>
check if water pressure is between limits<br>
>>>>if water pressure not between limits: stop water pump, close the tap, abort program and return error<br>

>>>set completion status "in progress"<br>
>>If start programs returns error, updates completion status to ABORTED or START_ERROR, else its set to IN_PROGRESS.<br>
writes program status in "program_status.txt"



# Temperature Controlled Ryobi Hot Melt Adhesive Gun

A Replacement Control Board for the R18GLU Hot Glue Gun to display voltage, 
battery percentage, current, and allow for PID Temperature Control.


## Features
This is a drop-in ((Screw in and solder to)) replacement Controller PCB,
which provides a few upgraded features:

* OLED Display which shows
    * Requested and Actual Temperature
    * Current draw from battery
    * Voltage of the battery pack
    * Estimated battery percentage/battery bar

* Overcurrent and Undervoltage Protection
* Adjustable PID Control loop
* LED Activity Light
* Precise Temperature Control (+/-5 degrees celcius)


## How do I install this mod?
This upgrade will require Ordering PCBs & Parts, Soldering, Modifying the
chassis, and barrel of the Glue Gun as well as 3D Printing.  
TCRHMAG is a hobby project and is not intended to be a beginner project for
everyone, and may be rough around the edges.  

1) Remove the Original Controller board and remove the wires from the Power Switch
2) **MODIFICATION STEPS**
2a) Drill a small hole in the Glue Gun's Heater Barrel for better thermal control,
and use a thermally rated (280c) adhesive to attach an NTC Thermistor (TODO: images and specs of thermistor)
2b) Create a notch in both halves of the Glue Gun Shell for the OLED and Control wires (TODO: Images)
using a Dremel or sharp knife etc.
2c) (TODO: add display control 3d printed housing)
3) Install the pre-assembed and programmed PCB
4) Solder the Power Switch to the PCB, and solder the Thermistor, Heater, LED
and Battery cables to the PCB
5) **3D PRINTED OLED HOUSING STEPS**
5a)......


## Programming The Microcontorller
**IMPORTANT** Solder and Flash the MCU **BEFORE** Assembling the rest of the PCB  
The NRST Line is used by an OpAmp circuit so it CANNOT be flashed in-situ until
the NRST line is disabled by the first Flash Cycle.
After this initial Programming Step, the board can be fully assembled and 
re-programmed if necessary without removing parts.


## Parts List


## To Do
* Display in Faranheit?


----
Copyright (c) 2026 ADBeta

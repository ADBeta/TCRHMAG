# Temperature Controlled Ryobi Hot Melt Adhesive Gun

An Upgrade Control Board for the R18GLU Hot Glue Gun to provide PID Temperature
Control, and display battery voltage, battery current.  
It also improves battery life to 1hr 45min on a 1.5Ah Ryobi Battery (At 150c).


## Features
This is a drop-in replacement Controller PCB, which provides:

* OLED Display which shows
    * Target and Actual Temperature
    * Battery Current draw
    * Battery Voltage (and Battery Bar)
* Overcurrent and Undervoltage Protection
* Adjustable PID Control loop
* LED Status Light
* Precise Temperature Control (+/-5 degrees celcius)


## How do I install this mod?
This upgrade will require ordering PCBs & components, soldering, modifying the
chassis, and barrel of the Glue Gun as well as 3D Printing.  
TCRHMAG is a medium difficulty hobby project.

1) Remove the Original Controller board and remove the wires from the Power Switch
2) **MODIFICATION STEPS**  
2a) Drill a small hole in the Glue Gun's heater barrel for better thermal control,
and use a thermally rated (280c) adhesive to attach an NTC Thermistor (TODO: images and specs of thermistor)
2b) Create a notch in both halves of the Glue Gun Shell for the OLED and Control wires (TODO: Images)
using a Dremel or sharp knife etc.
2c) (TODO: add display control 3d printed housing)
3) Install the programmed PCB
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

Use [minichlink](https://github.com/cnlohr/ch32fun/tree/master/minichlink) to
program the MCU.  
This project is built using [ch32fun](https://github.com/cnlohr/ch32fun).

## Bill of Materials


## To Do
* Display in Faranheit?
* Add better Rotary Encoder filter circuit
* Fix noisy battery voltage sensing
* Larget OLED / Rotary Encoder solder pads

----
Copyright (c) 2026 ADBeta

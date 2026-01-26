import math

# TODO: Use Steinhart-Hard or some other method for more accuracy

# Calculates a Look Up Table of Resitances for the Thermistor, based on temperature,
# then formats that data into an array of struct items in c - It them outputs the
# final output to a header file.
#
# In the current configuration, it outputs ADC Values based on the given 
# Microcontroller variables.
# You can configure it to output raw Resistances using outtype = "Reistance"
# or outtype = "ADC"
#
# (c) ADBeta    Ver1.0    09 Jan 2026

### Thermistor Variables ###
Rf = 4700                              # Fixed Resistor Value            (Ohm)
R0 = 100000.0                          # Thermistor Nominal Resistance   (Ohm)
T0 = 25.0                              # Thermistor Nominal Temperature  (°C)
B  = 3950                              # Thermistor Gain


### Output Variables ###
outfile    = "./thermistor_lut.h"      # Output Filename
outtype    = "ADC"
mintemp    =  0                        # Minimum Temperature to start   (°C)
maxtemp    =  300                      # Maximum Temperature to end     (°C)
tempstep   =  4                        # Temperature Steps in the array (°C)


### Microcontroller Variables (if using ADC Mode) ###
adc_max    =  1023                     # Maximum value the ADC Can read
mcu_vcc    =  5.0                      # Microcontroller input voltage


# Converts a given Temperature in Degrees Celsius to a Resistance Value (Ohms)
def temp_to_resistance(TIC):
    TIK = TIC + 273.15
    T0K = T0  + 273.15

    # Calculate the resistance using the Gain Curve
    R = R0 * math.exp(B * (1/TIK - 1/T0K))
    return round(R)


# Converts a given Resistance value into an ADC Reading
def resistance_to_adc(RES):
    vout = (mcu_vcc * RES) / (Rf + RES)
    adc  = (vout / mcu_vcc) * adc_max
    return min(adc_max, max(0, round(adc)))




try:
    of = open(outfile, "w", encoding="utf-8")
except OSError as e:
    print(f"Failed to open file: {e}")
    raise

# Write the Prefix c code to the new file
outprefix = f"""
#ifndef THERMISTOR_LUT_H
#define THERMISTOR_LUT_H


typedef struct {{
	uint16_t temp_c;    // Temperate Sample (Degrees Celsius)
	uint16_t adc;       // MCU ADC Value at given temperature
}} thermistor_pair_t;

// NTC Thermistor LUT assuming Rf of {Rf}R, VCC of {mcu_vcc}V and ADC with {adc_max} Counts
static const thermistor_pair_t thermistor_pair[] __attribute__((section(".rodata"))) = {{
"""
of.write(outprefix)


# Write each temp/resistance value
for temp in range(mintemp, maxtemp + tempstep, tempstep):
    res = temp_to_resistance(temp)
    adc = resistance_to_adc(res)

    if outtype == "Resistance":
        outstring = f"\t{{{temp:>{3}},\t{res}}},\n"

    if outtype == "ADC":
        outstring = f"\t{{{temp:>{3}},\t{adc}}},\n" 
    

    of.write(outstring)

# Write the end };
of.write("};\n\n\n")


# Write the Length definition and header endif
of.write("#define THERMISTOR_PAIR_ENTRIES \\\n\t(sizeof(thermistor_pair) / sizeof(thermistor_pair[0]))\n\n")
of.write("#endif")


of.close()

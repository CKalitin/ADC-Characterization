import pyvisa
import time
import numpy as np

rm = pyvisa.ResourceManager()

print(rm.list_resources())
print(rm)

instrument = rm.open_resource('USB0::1689::851::1516308::0::INSTR')
print(instrument.query('*IDN?'))

# Guide: https://github.com/pyvisa/pyvisa/issues/503
# AFG 1022 Programmer Manual: https://download.tek.com/manual/AFG1022-Programmer-Manual-EN.pdf

# Guide on how to send waveform: https://my.tek.com/en/tektalk/signal-gen/205c8abe-dc53-ed11-a81b-000d3a324ccc

# BETTER DOCS: https://mmrc.caltech.edu/Tektronics/AFG3021B/AFG3021B%20Programmer%20Manual.pdf

# Reset the instrument to a known state
instrument.write("*RST")

# Clears all event registers and queues
instrument.write('*CLS')


time.sleep(1)

# Query amplitude voltage
print(instrument.write("SOURce1:FUNCtion:SHAPe PULSe"))
#print(instrument.write("SOURce1:VOLTage:LEVel:IMMediate:AMPLitude 5.0"))

time.sleep(1)

print(instrument.write("SOURce1:PULSe:DCYC 99.9"))

# enable output
time.sleep(1)

print(instrument.write('OUTPut1:STATe ON'))

time.sleep(1)

delay_per_obv = 1 # seconds

voltage = 0

while True:
    voltage += 0.05
    voltage = round(voltage * 100) / 100
    offset = round(voltage/2 * 100) / 100
    if (voltage >= 5): break
    print(f"Voltage: {voltage}, Offset: {offset}")
    
    # get current time in nanoseconds
    start_time = time.time_ns()
    print("SOURce1:VOLTage:LEVel:IMMediate:AMPLitude " + str(voltage) )
    print("SOURce1:VOLTage:LEVel:IMMediate:OFFSet " + str(offset))
    print(instrument.write("SOURce1:VOLTage:LEVel:IMMediate:AMPLitude " + str(voltage)))
    print(instrument.write("SOURce1:VOLTage:LEVel:IMMediate:OFFSet " + str(offset)))
    end_time = time.time_ns()
    
    #print(f"{start_time}, {end_time}, {end_time - start_time} {(end_time - start_time)/1e9}")
    time.sleep(delay_per_obv - (end_time - start_time)/1e9)
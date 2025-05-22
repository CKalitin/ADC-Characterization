import pyvisa
import time

rm = pyvisa.ResourceManager('@py')

print(rm.list_resources())
print(rm)

instrument = rm.open_resource('USB0::0x05E6::0x2110::8005314::INSTR')
print(instrument.query('*IDN?'))

# print(instrument.read())

delay_per_obv = 0.5 # seconds

f = open("data.csv", "w")
f.write(f"Current,Obv Time\n")
current = 1
while abs(current) > 0.1:
    # get current time in nanoseconds
    start_time = time.time_ns()
    current = float(instrument.query('MEAS:CURR:DC?').replace("\n", "").strip())
    end_time = time.time_ns()
    
    #print(f"{start_time}, {end_time}, {end_time - start_time} {(end_time - start_time)/1e9}")
    time.sleep(delay_per_obv - (end_time - start_time)/1e9)
    print(f"{str(current)}, {(end_time - start_time)/1e9}") # Time (s), Voltage (v), Obv Time (s)
    f.write(f"{str(current)}, {(end_time - start_time)/1e9}\n")
f.close()
# This test automatically gets data from the DMM in voltmeter mode, DMM in ammeter mode, and the STM32 ADC reading
# You manually change the PSU input since it isn't programmable

import pyvisa
import serial
import time
from datetime import datetime

import HelperFunctions as hf

rm = pyvisa.ResourceManager('@py')

# You'll need to copy paste some of the IDs here into the rm.open_resource() functions below
print(rm.list_resources())

# Note that you need to match the DMM device to the correct USB ID
# Table: Date on top of the DMM - USB ID on my laptop (maybe it's the same for yours, who knows)
# 23-Oct-2018 - USB0::0x05E6::0x2110::8006144::INSTR
# 25-Sep-2018 - USB0::0x05E6::0x2110::8005314::INSTR

dmm_voltmeter = rm.open_resource('USB0::0x05E6::0x2110::8006144::INSTR')
dmm_ammeter = rm.open_resource('USB0::0x05E6::0x2110::8005314::INSTR')

serial_bus = hf.open_serial("COM7", 115200)

f = open("data.csv", "w")
f.write(f"DMM Voltage,DMM Current,ADC Raw Value,ADC Adjusted Value,ADC Current Adjusted Value,Iteration Time,Current Time\n")

while True:
    timer = hf.Timer() # Python garbage collector should deal with it
    
    raw_adc_value, adc_adjusted_value, adc_current_adjusted_value = hf.get_adc_value_from_stm32(serial_bus)
    dmm_voltage = hf.get_dmm_voltage(dmm_voltmeter)
    dmm_current = hf.get_dmm_current(dmm_ammeter)
    
    current_time = datetime.now().strftime("%H:%M:%S")+ '.' + datetime.now().strftime('%f')[:3] # Get time in HH:MM:SS.sss format
    elapsed_time = timer.get_elapsed_time()
    
    raw_str_out = f"{dmm_voltage},{dmm_current},{raw_adc_value},{adc_adjusted_value},{adc_current_adjusted_value},{elapsed_time},{current_time}"
    labeled_str_out = f"DMM Voltage: {dmm_voltage}, DMM Current: {dmm_current}, ADC Raw Value: {raw_adc_value}, ADC Adjusted Value {adc_adjusted_value}, ADC Current Adjusted Value {adc_current_adjusted_value}, Iteration Time: {elapsed_time}, Current Time: {current_time}"
    
    print(labeled_str_out)
    f.write(raw_str_out + "\n")
    
    time.sleep(0.5) # 1 second delay between each iteration to give me time to manually change the current
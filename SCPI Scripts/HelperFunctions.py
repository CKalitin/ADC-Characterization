# Docs with afg commands: https://mmrc.caltech.edu/Tektronics/AFG3021B/AFG3021B%20Programmer%20Manual.pdf

import pyvisa
import serial
import time

wait_time_between_commands = 0.5 # Delay between commands to let the very old hardware process it

class Timer:
    def __init__(self):
        self.start_time = time.time()
    
    def get_elapsed_time(self):
        return time.time() - self.start_time

# Replace all terms in a string that equal delete_chars with ""
# Eg. remove_chars("1212, 8989\n\x00", ["\n", "\x00", " "]) returns "1212,8989"
def remove_chars(text, delete_chars):
    for char in delete_chars:
        text = text.replace(char, "")
    return text

def afg_setup_pulse_mode(afg):
    afg.write("*RST") # Reset instrument to the default state (sine wave)
    afg.write("*CLS") # Clear all event registers and queues
    
    time.sleep(wait_time_between_commands) # Wait until after reset/clear is complete, give the afg some time to breathe
    
    afg.write("SOURce1:FUNCtion:SHAPe PULSe") # Pulse function mode

    time.sleep(wait_time_between_commands) # Wait until after reset/clear is complete, give the afg some time to breathe
    
    afg.write("SOURce1:PULSe:DCYC 99.9") # Max 99.9% duty cycle
    
    time.sleep(wait_time_between_commands)
    
    afg.write("SOURce1:FREQuency 0.000001")
    
    time.sleep(wait_time_between_commands)
    
    afg.write("SOURce1:VOLTage:LEVel:IMMediate:AMPLitude 1.0") # Default voltage of 1V
    afg.write("SOURce1:VOLTage:LEVel:IMMediate:OFFSet 0.5") # We want a high of 1V and low of 0V, so we have to adjust the waveform up by 0.5V, since 1V is the wave amplitude
    
    time.sleep(wait_time_between_commands)
    
def set_afg_pulse_high_voltage(afg, high_voltage):
    # Hopefully there'll be no floating point issues with high_voltage/2 !!!
    afg.write(f"SOURce1:VOLTage:LEVel:IMMediate:AMPLitude {str(high_voltage)}")
    afg.write(f"SOURce1:VOLTage:LEVel:IMMediate:OFFSet {str(high_voltage/2)}")
    time.sleep(wait_time_between_commands) # Give it some time to breathe

def enable_afg_output(afg):
    afg.write("OUTPUT1:STATe ON")
    time.sleep(wait_time_between_commands)
    
def disable_afg_output(afg):
    afg.write("OUTPUT1:STATe OFF")
    time.sleep(wait_time_between_commands)

def get_dmm_voltage(dmm):
    return float(dmm.query("MEAS:VOLT:DC?"))

def get_dmm_current(dmm):
    return float(dmm.query("MEAS:CURR:DC?"))

def open_serial(port, baudrate):
    serial_bus = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    return serial_bus

# This returns both the raw value and adjusted value
# Return: raw,adjusted as ints
def get_adc_value_from_stm32(serial_bus):
    try:
        serial_bus.write("s".encode()) # This commands the STM32 to start reading ADC values and return the average
        uart_data = serial_bus.readline().decode("utf-8") # readline() waits for a '\n' and pauses the program until we've received it
        uart_data = remove_chars(uart_data, ["\n", "\x00", " ", "\r"]) # Remove all bullshit from the string
        raw_adc_value, adc_adjusted_value, adc_current_adjusted_value = uart_data.split(",")
        return int(raw_adc_value), int(adc_adjusted_value), int(adc_current_adjusted_value) # Return the values as a ints
    except:
        return -1, -1, -1

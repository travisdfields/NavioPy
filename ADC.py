import sys, time
import numpy as np
import navio.adc
import navio.util

navio.util.check_apm()
adc = navio.adc.ADC()
data = np.zeros(1000)

start_time = time.clock()*1000.0
for i in range(0,1000):
	analog_val = adc.read(4)*0.001
	data[i] = analog_val
	print analog_val
	time.sleep(0.25)
end_time = time.clock()*1000.0
average = np.mean(data)
print end_time - start_time, average


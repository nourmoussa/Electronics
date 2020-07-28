   
import pyb
from pyb import Pin, Timer, ADC, DAC, LED
from array import array         # need this for memory allocation to buffers
from oled_938 import OLED_938   # Use OLED display driver
from mic import MICROPHONE
from g24_choreo_functions import *
import time
import micropython


#  The following two lines are needed by micropython 
#   ... must include if you use interrupt in your program
micropython.alloc_emergency_exception_buf(100)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=61)  
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Dance to Beat')
oled.draw_text(0,20, 'Press USR button.')
oled.display()

print('g24_dance.py')
print('Waiting for button press.')
sw = pyb.Switch()
while not sw():
    time.sleep(0.001)
while sw(): pass
print('Button pressed. Running.')

oled.draw_text(0,20, 'Running dance script.')
oled.display()

# Create microphone object
SAMP_FREQ = 8000
N = 160
mic = MICROPHONE(Timer(7,freq=SAMP_FREQ),ADC('Y11'),N)

# define ports for microphone, LEDs and trigger out (X5)
b_LED = LED(4)      # flash for beats on blue LED

# Define constants for main program loop - shown in UPPERCASE
M = 50                      # number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 1.8        # threshold for c to indicate a beat

# initialise variables for main program loop 
e_ptr = 0                   # pointer to energy buffer
e_buf = array('L', 0 for i in range(M)) # reserve storage for energy buffer
sum_energy = 0              # total energy in last 50 epochs
pyb.delay(100)
tic = pyb.millis()          # mark time now in msec

# read the characters into a list, intialise stuff
movelist = readlist('g24_choreo2.txt')
counter = 0
# print(movelist) # for debugging
# real-time program loop

try:
    while True:             # Main program loop
        if mic.buffer_full():       # semaphore signal from ISR - set if buffer is full
            b_LED.off()             # flash off
            # Get instantaneous energy
            E = mic.inst_energy()   # get the instantaneous energy from the microphone

            # compute moving sum of last 50 energy epochs
            sum_energy = sum_energy - e_buf[e_ptr] + E
            e_buf[e_ptr] = E        # over-write earliest energy with most recent
            e_ptr = (e_ptr + 1) % M # increment e_ptr with wraparound - 0 to M-1

            # Compute ratio of instantaneous energy/average energy
            c = E*M/sum_energy
            # Look for a beat
            if (pyb.millis()-tic > 500):    # if more than 500ms since last beat
                if (c>BEAT_THRESHOLD) or (pyb.millis()-tic > 600):
                    # look for a beat, or if not found, timeout
                    tic = pyb.millis()      # reset tic
                    b_LED.on()# beat found, flash blue LED ON REPLACE THIS WITH THE MOVES
                    # execute move function (if=='c' etc)
                    readmove(movelist, counter, 25) # execute a move depending on the counter
                    counter += 1            # increment a counter when beat detected

            mic.set_buffer_empty()          # reset the buffer_full flag

finally: # in the event of a crash or keyboard interrupt turn of motors before exiting program
    motor.A_stop()
    motor.B_stop()
        
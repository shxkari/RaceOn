from picamera.array import PiYUVArray, PiRGBArray
from picamera import PiCamera
from scipy.signal import find_peaks, butter, filtfilt
import time
import matplotlib.pyplot as plt
import skimage as ski
import numpy as np
import time
from pwm import PWM
from ipywidgets import interact
import PID

#global variables
line = 200
Kp = 1560
Kd = 0
Ki = 43

#setup servo and motor contol
pwm0 = PWM(0)
pwm0.period = 20000000
pwm0.duty_cycle = 10000000
pwm0.enable = True

pwm1 = PWM(1)
pwm1.period = 20000000
pwm1.duty_cycle = 1500000
pwm1.enable = True

#Setup PID
pid = PID.PID(Kp,Kd,Ki) #Kp is set to 2000, I & D are disabled
pid.SetPoint = 320
pid.setSampleTime(0.01) #sample time is 10ms

# set default value
line = 200

# setup camera settings
res = (640, 480)
camera = PiCamera()
camera.sensor_mode = 7
camera.resolution = res
camera.framerate = 120
#initialize camera feed
rawCapture = PiYUVArray(camera, size=res)
stream = camera.capture_continuous(rawCapture, format="yuv", use_video_port=True)


## main loop stuff
pwm0.duty_cycle = 1200000
first_run = True
threshold = 100
p_old = []
p_current = []
i = 0
line = 135
j = 0

# To filter the noise in the image we use a 3rd order Butterworth filter
b, a = butter(3, 0.02) 
for f in stream:
    if (abs(pid.output) > 100000) and not (pwm0.duty_cycle == 1000000):
        pwm0.duty_cycle = 1200000
        i = 0
        line = 180
#         pid.setKp(1550)
#         pid.setKi(40)
    else:
        i += 1
        if(i > 15):
            pwm0.duty_cycle = 1275000
            line = 100
     #       i = 7
#             pid.setKp(1550)
#             pid.setKi(40)
            
    # Get the intensity component of the image (a trick to get black and white images)
    I = f.array[:, :, 0]
    # Reset the buffer for the next image
    rawCapture.truncate(0)
    # Select a horizontal line in the middle of the image
    L = I[line, :]
    # Smooth the transitions so we can detect the peaks 
    Lf = filtfilt(b, a, L)
    # Due to the noise in the image the algorithm finds many peaks heigher than 0.5
    p_current = find_peaks(Lf, height=100)
    
    if first_run:
        if len(p_current[0] > 0):
            # calculations
            # print(p)
            pid.update(p_current[0][0])
            feedback = int(pid.output + 1500000)
            pwm1.duty_cycle = feedback
            #print(feedback)
            first_run = False
            p_old = p_current
    elif not first_run:
#         # good data
#         if len(p_current[0]) > 0:
#             p_best = p_current[0][0]
#             x_old = 100
#             x = 0
#             for p in p_current[0]:
#                 x = abs(p - p_old[0][0])
#                 if x < x_old:
#                     x_old = x
#                     p_current[0][0] = p
        if((len(p_current[0]) > 0) and (abs(p_current[0][0] - p_old[0][0]) < threshold)):
            # calculations
            # print(p)
            pid.update(p_current[0][0])
            feedback = int(pid.output + 1500000)
            pwm1.duty_cycle = feedback
            p_old = p_current
            #print(feedback)
            #print("new")
        else:
            #ignore the new image -- it is not the center line or there are no peaks
            pid.update(p_old[0][0])
            feedback = int(pid.output + 1500000)
            pwm1.duty_cycle = feedback
            #print(feedback)
            #print("old")
    if(pwm0.duty_cycle < 1050000):
        break

# once out of the loop, turn motor off, reset steering to straight. 
pwm0.duty_cycle = 1000000
pwm0.enable = False
pwm1.duty_cycle = 1500000
pwm1.enable = False

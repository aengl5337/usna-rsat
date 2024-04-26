# Arm Code V.1.0

# Imports
import math
from math import sqrt
from math import atan
from math import cos
from math import pi

# take inputs (x,y,z) of object

x = 4 # example
y = 4 # example
z = 6 # example

# Initial position of arm 1 - (x0,y0,z0)
x01 = 5 # x0 - along back of cubseat to arm (5? inches)
y01 = 2 # y0 - out from cubesat - (2? inches) from motor 1 as motor 1 is set to pi/2 to fully deploy arms
z01 = 3 # z0 - up from camera (3? inches)

xa = x-x01 # determines where the arm needs to go. I have the arm go to a specific x,y coordinate such that when it rotates into the z dimension, it collides with the object position
ya = y-y01
za = z-z01

# Algorithm for (xa,ya,za) movement to object

if xa < 0 and za > 0: # upper left quadrant from base of arm
    theta2 = atan(za/xa) # angle for motor 2 to turn
    r = sqrt(xa**2+za**2) # radius of circle pushing arm into 3rd dimension
    # iterate below for best values of theta3 and theta5
    for theta3 in range(-314,0):
        for theta5 in range(-314,0):
            rd = 7*cos((theta3)/100)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e1 = r-rd
            yd = 11+7*cos((theta3)/100+pi)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e2 = ya-yd
            if e1 < 0.01 and e2 < 0.01: # if theta3 and theta5 are such that the error between desired and actual position are 0, values will be stored
                theta3i = theta3
                theta5i = theta5
                
if xa < 0 and za > 0: # bottom left quadrant
    theta2 = atan(za/xa) # angle for motor 2 to turn
    r = sqrt(xa**2+za**2) # radius of circle pushing arm into 3rd dimension
    # iterate below for best values of theta3 and theta5
    for theta3 in range(-314,0):
        for theta5 in range(-314,0):
            rd = 7*cos((theta3)/100)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e1 = r-rd
            yd = 11+7*cos((theta3)/100+pi)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e2 = ya-yd
            if e1 < 0.01 and e2 < 0.01: # if theta3 and theta5 are such that the error between desired and actual position are 0, values will be stored
                theta3i = theta3
                theta5i = theta5
                
if xa > 0 and za > 0: # upper right quadrant
    theta2 = atan(za/xa) # angle for motor 2 to turn
    r = sqrt(xa**2+za**2) # radius of circle pushing arm into 3rd dimension
    # iterate below for best values of theta3 and theta5
    for theta3 in range(0,314):
        for theta5 in range(0,314):
            rd = 7*cos((theta3)/100)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e1 = r-rd
            yd = 11+7*cos((theta3)/100+pi)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e2 = ya-yd
            if e1 < 0.01 and e2 < 0.01: # if theta3 and theta5 are such that the error between desired and actual position are 0, values will be stored
                theta3i = theta3
                theta5i = theta5
            
if xa > 0 and za < 0: # bottom right quadrant
    theta2 = atan(za/xa) # angle for motor 2 to turn
    r = sqrt(xa**2+za**2) # radius of circle pushing arm into 3rd dimension
    # iterate below for best values of theta3 and theta5
    for theta3 in range(0,314):
        for theta5 in range(0,314):
            rd = 7*cos((theta3)/100)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e1 = r-rd
            yd = 11+7*cos((theta3)/100+pi)+4.5*cos((theta3/100)+(theta5/100)+pi)
            e2 = ya-yd
            if e1 < 0.01 and e2 < 0.01: # if theta3 and theta5 are such that the error between desired and actual position are 0, values will be stored
                theta3i = theta3
                theta5i = theta5
                
# Algorithm to place claw in desired position?

# Output for each motor
M1r1 = math.pi/2 # deploys arm straight out
M2r1 = theta2 # rotates arm to third dimension
M3f1 = theta3i # determined above
M4r1 = 0 # 0 for now until I figure out the math
M5f1 = theta5i # determined above
M6r1 = 0 # rotates claw
M7c1 = math.pi/2 # opens claw

# OUPUT(M1r1,M2r2,M3f1,M4r1,M5f1,M6r1,M7c1) to motors

#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

# COMMS PORTION

from smbus import SMBus


# Movement of Arm 1

# Initialize
addr = 0x9 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
motor = 8# Motor ID#
forward = 1 # forwards?(Y:1/N:0)
multiple = 0 # Greater than 180? (Y:1/N:0)

# Filter data
theta1 = M1r1*180/pi # convert to degrees
theta1 = int(round(theta1)) # rounds and makes integer
if theta1 < 0: # for backwards movements
    theta1 = -1*theta1 # make value positive
    forward = 0 # set movement backward
if theta1 > 180: # evaluate if theta is too large for transmission
    multiple = 1 # add to multiplier counter
    theta1 = theta1-180 # reduce by 180 for transmission

thetaD = theta1 # Desired angle change

bus.write_block_data(addr,motor,[forward,multiple,thetaD]) # send






# Arm Code V.2.0

# Imports
"""
import math
from math import sqrt
from math import atan
from math import cos
from math import pi
"""
import numpy as np
from math import sqrt
from math import atan
from math import cos
from math import pi

## PHYSICAL CHARACTERISTICS
# Physical coordinate system
# xy plane is the plane of the bottom of RECS, and furthermore the STAND
# x length is shorter, y length is longer
# z dimension is the height of the RECS, or where the arms are reaching into
# NOTE: this is relative to each arm -- imagine the origin sitting at the shaft of the first (shoulder) motor

# Lengths, to two significant digits (cm)

l12=0
l23=31.224
l34=18.202
l45=0
l56=7.66
l67=0
l78=4.45
l78c=2.225 # Measured to centerpoint of end effector, 
           # or approximate grabbing centroid
l = [l12,
     l23,
     l34,
     l45,
     l56,
     l67,
     l78]

l = np.asarray(l) # convert lengths to cm

## NOTE: all angles are relative to the arm's position when it is completely stretched out, and parallel to the y-axis

# Initial angular positions of each motor
# * = somewhat arbitrary intial angles
thetas0 = [0,       #*stowed arm
           0,       #*motor board facing positive z dir
           -180,    #**revise, as it doesn't fold exactly into perfect place**
           0,       #*motor board facing towards positive x dir
           7.2,      #**revise, as there's a slight bend
           0,       #*motor board facing towards positive x dir
           0]       #*Gripper closed

# Valid angle ranges for each motor (degrees)
thetasLimRng = [[-225,0],        # **guess**
               [-180,180],
               [-180,0],
               [-180,180],
               [-123.95,34.29], # **revise**
               [-180,180],
               [-45,135]]       # **guess**

thetasLimRng = np.asarray(thetasLimRng)
thetasLimRngR = thetasLimRng*pi/180 # convert to radians


## ARM CONTROL/PHYSICAL PATH TO ARM TRAJECTORY
# NOTE: this revision does not concern itself with intermediate points, only a target
# Inputs physical points (x,y,z) of target
# Outputs vector of motor angles

# Target cartesian coordinates (in cm) ****MODIFY, MEASURED FROM ARM ROOT
x = 0 # example
y = 20 # example
z = 40 # example

# Initial position of arm 1 - (x0,y0,z0) -- ***MODIFY, USED RUDIMENTARY SIMULATOR, MEASURED FROM ARM ROOT
x01 = 15.1779
y01 = 10.0749
z01 = 0

xa = x-x01 # determines where the arm needs to go. I have the arm go to a specific x,y coordinate such that when it rotates into the z dimension, it collides with the object position
ya = y-y01
za = z-z01

## SIMPLE ALGORITHM

# Assumed angles:
theta1 = -90 # shoulder pivot straight out into z direction
theta2 = -atan(ya/xa)*180/pi # forearm adjusts so that the point is in the plane of action of the arm
theta4 = theta6 = 0
theta7 = 90 # gripper open

# Solve for thetas 3,5
r_xy = sqrt(xa**2+ya**2)  # radius of circle pushing arm into 3rd dimension
za_prime = za - (l12+l23)
r_a = sqrt(za_prime**2+r_xy)
theta_aR = atan(za/r_xy) #***********************change to atan2()?
# iterate below for best values of theta3 and theta5
for stheta3R in range(int(thetasLimRngR[3-1][0]*100),int(thetasLimRngR[3-1][1]*100)):
    for stheta5R in range(int(thetasLimRngR[5-1][0]*100),int(thetasLimRngR[5-1][1]*100)):
        itheta3R=stheta3R/100
        itheta5R=stheta5R/100
        
        r_d = (l34)*cos(itheta3R-theta_aR)-(l56+l67)*cos(itheta3R+itheta5R-theta_aR)
        r_err = r_a-r_d
        
        if r_err < 0.01: # if theta3 and theta5 are such that the error between desired and actual position are 0, values will be stored
            theta3 = itheta3R*180/pi
            theta5 = itheta5R*180/pi


"""
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
"""

# Algorithm to place claw in desired position?

# Output for each motor
"""
M1r1 = math.pi/2 # deploys arm straight out
M2r1 = theta2 # rotates arm to third dimension
M3f1 = theta3 # determined above
M4r1 = 0 # 0 for now until I figure out the math
M5f1 = theta5 # determined above
M6r1 = 0 # rotates claw
M7c1 = math.pi/2 # opens claw
"""
# thetas = [theta1,
#           theta2,
#           theta3,
#           theta4,
#           theta5,
#           theta6,
#           theta7]

thetas = [-4,
          4,
          -4,
          0,
          0,
          0,
          0]

# print(theta3,theta5)
# raise Exception("Stopping here for now")

# OUPUT(M1r1,M2r2,M3f1,M4r1,M5f1,M6r1,M7c1) to motors

#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

## COMMS PORTION

from smbus import SMBus
import time

# Movement of Arm 1

# Initialize I2C Bus
addr = 0x9 # bus address
bus = SMBus(1) # indicates /dev/ic2-1 (default, some RaspPi may use /dev/ic2-0)

motor = 1# Motor ID#
# for theta in thetas:
for i in range(3):
    theta=thetas[motor-1]
    if (theta > thetasLimRng[motor-1][0]) and (theta < thetasLimRng[motor-1][1]):
        # Reset the following two variables for each motor
        forward = 1 # forwards?(Y:1/N:0) (DEFAULTS TO FORWARDS)
        multiple = 0 # Greater than 180? (Y:1/N:0)

        # Filter data
        # Note that input is in radians, and is expected to be <360 (as 'multiple' portion wouldn't work with multiple>1)
        theta = int(round(theta)) # rounds and makes integer *************************
                                    # NOTE ON ROUND(): The round() function rounds values of .5 towards an even integer (Python Docs, n.d. a). So .5 is round up for positive values and round down for negative value, both round(0.5) and round(-0.5) return 0, while round(1.5) gives 2 and round(-1.5) gives -2. This Python behaviour is a bit different from how rounding usually goes.
                                    
        if theta < 0:              # for backwards movements
            theta = -1*theta      # make value positive
            forward = 0             # set movement backward
        if theta > 180:            # evaluate if theta is too large for transmission
            multiple = 1            # add to multiplier counter
            theta = theta-180     # reduce by 180 for transmission
        
        thetaD = theta             # Desired angle change
        
        if theta > 0: # Only bother sending if is a nonzero angle
            bus.write_block_data(addr,motor,[forward,multiple,thetaD]) # send
            # FIND A more sophisticated WAY TO WAIT UNTIL CONCLUSION (takes about 2 seconds, packets should send in the same time regardless of angle sent)
            # more challenging to wait until motor done actuating than it is to wait until I2C is done sending.  Not sure if fully 2-way?
            time.sleep(10)
        motor+=1
        
    else:
        raise Exception("Commanded angle is out of range for motor %d" % motor)
        





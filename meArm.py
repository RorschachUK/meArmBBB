# meArm.py - York Hack Space May 2014
# A motion control library for Phenoptix meArm for the Beaglebone Black

import kinematics
import time
from math import pi
import Adafruit_BBIO.PWM as PWM

class meArm():
    def __init__(self, sweepMinBase = 145, sweepMaxBase = 49, angleMinBase = -pi/4, angleMaxBase = pi/4,
    			 sweepMinShoulder = 118, sweepMaxShoulder = 22, angleMinShoulder = pi/4, angleMaxShoulder = 3*pi/4,
    			 sweepMinElbow = 144, sweepMaxElbow = 36, angleMinElbow = pi/4, angleMaxElbow = -pi/4,
    			 sweepMinGripper = 75, sweepMaxGripper = 115, angleMinGripper = pi/2, angleMaxGripper = 0):
        """Constructor for meArm - can use as default arm=meArm(), or supply calibration data for servos."""
    	self.servoInfo = {}
    	self.servoInfo["base"] = self.setupServo(sweepMinBase, sweepMaxBase, angleMinBase, angleMaxBase)
    	self.servoInfo["shoulder"] = self.setupServo(sweepMinShoulder, sweepMaxShoulder, angleMinShoulder, angleMaxShoulder)
    	self.servoInfo["elbow"] = self.setupServo(sweepMinElbow, sweepMaxElbow, angleMinElbow, angleMaxElbow)
    	self.servoInfo["gripper"] = self.setupServo(sweepMinGripper, sweepMaxGripper, angleMinGripper, angleMaxGripper)
    	
    # Adafruit servo driver has four 'blocks' of four servo connectors, 0, 1, 2 or 3.
    def begin(self, pinBase="P9_14", pinShoulder="P9_21", pinElbow="P9_42", pinGripper="P8_13"):
        """Call begin() before any other meArm calls.  Optional parameters to select a different block of servo connectors or different I2C address."""
    	self.base = pinBase
    	self.shoulder = pinShoulder
    	self.elbow = pinElbow
    	self.gripper = pinGripper
    	PWM.start(self.base, 60, 60.0)
    	PWM.start(self.shoulder, 60, 60.0)
    	PWM.start(self.elbow, 60, 60.0)
    	PWM.start(self.gripper, 60, 60.0)
    	self.goDirectlyTo(0, 100, 50)
    	self.openGripper()
    	
    def setupServo(self, n_min, n_max, a_min, a_max):
        """Calculate servo calibration record to place in self.servoInfo"""
    	rec = {}
    	n_range = n_max - n_min
    	a_range = a_max - a_min
    	if a_range == 0: return
    	gain = n_range / a_range
    	zero = n_min - gain * a_min
    	rec["gain"] = gain
    	rec["zero"] = zero
    	rec["min"] = n_min
    	rec["max"] = n_max
    	return rec
    
    def angle2pwm(self, servo, angle):
        """Work out pulse length to use to achieve a given requested angle taking into account stored calibration data"""
    	dutyMin = 3
        dutyMax = 14.5
        dutySpan = dutyMax - dutyMin
    	angDeg = self.servoInfo[servo]["zero"] + self.servoInfo[servo]["gain"] * angle
    	duty = 100 - ((angDeg / 180) * dutySpan + dutyMin) 
    	return duty
    	
    def goDirectlyTo(self, x, y, z):
        """Set servo angles so as to place the gripper at a given Cartesian point as quickly as possible, without caring what path it takes to get there"""
    	angles = [0,0,0]
    	if kinematics.solve(x, y, z, angles):
    		radBase = angles[0]
    		radShoulder = angles[1]
    		radElbow = angles[2]
    		PWM.set_duty_cycle(self.base, self.angle2pwm("base", radBase))
    		PWM.set_duty_cycle(self.shoulder, self.angle2pwm("shoulder", radShoulder))
    		PWM.set_duty_cycle(self.elbow, self.angle2pwm("elbow", radElbow))
    		self.x = x
    		self.y = y
    		self.z = z

    def gotoPoint(self, x, y, z):
        """Travel in a straight line from current position to a requested position"""
    	x0 = self.x
    	y0 = self.y
    	z0 = self.z
    	dist = kinematics.distance(x0, y0, z0, x, y, z)
    	step = 10
    	i = 0
    	while i < dist:
    		self.goDirectlyTo(x0 + (x - x0) * i / dist, y0 + (y - y0) * i / dist, z0 + (z - z0) * i / dist)
    		time.sleep(0.05)
    		i += step
    	self.goDirectlyTo(x, y, z)
    	time.sleep(0.05)
    	
    def openGripper(self):
        """Open the gripper, dropping whatever is being carried"""
        print self.gripper
    	PWM.set_duty_cycle(self.gripper, self.angle2pwm("gripper", pi/4.0))
    	time.sleep(0.3)
    	
    def closeGripper(self):
        """Close the gripper, grabbing onto anything that might be there"""
    	PWM.set_duty_cycle(self.gripper, self.angle2pwm("gripper", -pi/4.0))
    	time.sleep(0.3)
    
    def isReachable(self, x, y, z):
        """Returns True if the point is (theoretically) reachable by the gripper"""
    	radBase = 0
    	radShoulder = 0
    	radElbow = 0
    	return kinematics.solve(x, y, z, radBase, radShoulder, radElbow)
    
    def getPos(self):
        """Returns the current position of the gripper"""
    	return [self.x, self.y, self.z]

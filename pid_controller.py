from math import *
import time

class pid:
	def __init__(self, kP = 1.0, kI = 1.0, kD = 1.0, setpoint = 0.0):
		self.kP = kP
		self.kI = kI
		self.kD = kD
		self.setpoint = setpoint
		self.out = 0.0
		self.I = 0.0
		self.t_prev = time.time()
		self.e_prev = 0.0

	def setSetpoint(self, setpoint):
		self.setpoint = setpoint

	def getOutput(self):
		return self.out

	def resetTime(self):
		self.t_prev = time.time()

	def update(self, sensorIn):
		error = self.setpoint - sensorIn
		t_delta = time.time() - self.t_prev #use time
		#t_delta = 1.0; #don't use time
		P = self.kP * error
		self.I += self.kI * error * t_delta
		D = self.kD * (error - self.e_prev)/t_delta
		self.out = P+self.I+D
		self.t_prev = time.time()
		self.e_prev = error

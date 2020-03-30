import logging
from math import *
import time
import board
import busio
import os
from adafruit_motorkit import MotorKit
import RPi.GPIO as GPIO
import adafruit_bno055

log = logging.getLogger('RemoTV.hardware.kiwinkidink')

kit = None
i2c = None
sensor = None
pid = None

camServoMin = 6.0
camServoMax = 12.0
camServo = 8.0

stationary = False

arduino_address = 0x08

#PID Coefficients
kP = 2.0
kI = 0.0
kD = 0.01

class pid_controller:
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

def stopMotors():
    kit.motor1.throttle = 0.0
    kit.motor2.throttle = 0.0
    kit.motor3.throttle = 0.0
    kit.motor4.throttle = 0.0

def stopServo():
    cs.ChangeDutyCycle(0.0)

def incrementCamServo(amount):
    global camServo
    camServo += amount

    log.debug("cam servo positions:", camServo)

    if camServo > camServoMax:
        camServo = camServoMax
    if camServo < camServoMin:
        camServo = camServoMin
    cs.ChangeDutyCycle(camServo)

def RCConnected():
    data = bytearray(5)
    i2c.readfrom_into(arduino_address, data, start = 0, end=len(data))
    return (data[0] > 0);

def getGyroData():
    angle_z, angle_x, angle_y = sensor.euler
    return angle_z

def constrain(x, Min, Max):
    if x > Max:
        x = Max
    if x < Min:
        x = Min
    return x

def combine(Roll, Pitch, Yaw, Theta):
    rVal = Yaw + cos(Theta)*Pitch + cos(Theta+pi/2.0)*Roll
    return rVal

def setKiwiMotors(roll, pitch, yaw, scale):
    motor2_speed = combine(roll, pitch, yaw, pi/3.0)
    motor3_speed = combine(roll, pitch, yaw, 5.0*pi/6.0)
    motor4_speed = combine(roll, pitch, yaw, 3.0*pi/2.0)
    motor_max = max([abs(motor2_speed), abs(motor3_speed), abs(motor4_speed)])
    throttle_mult = 0.0
    try:
        throttle_mult = scale/motor_max
    except (ZeroDivisionError):
        throttle_mult = 1.0
    kit.motor2.throttle = constrain(throttle_mult*motor2_speed, -1.0, 1.0)
    kit.motor3.throttle = constrain(throttle_mult*motor3_speed, -1.0, 1.0)
    kit.motor4.throttle = constrain(throttle_mult*motor4_speed, -1.0, 1.0)

def translate(x_speed, y_speed, rotate_speed, speed, delta_t):
    prev_t = time.time()
    pid.resetTime()
    while ((time.time() - prev_t) < delta_t):
        yaw = rotate_speed
        in_angle = getGyroData()
        if in_angle == None:
            pid.resetTime()
        else:
            angle = in_angle*2*pi/370.0
            if abs(pid.setpoint-(angle+2*pi)) < abs(pid.setpoint-angle):
                angle += 2*pi
            elif abs(pid.setpoint-(angle-2*pi)) < abs(pid.setpoint-angle):
                angle -= 2*pi
            pid.update(angle)
            yaw -= pid.getOutput()
            pid.setpoint += abs(constrain(rotate_speed, -1.0, 1.0))*(angle - pid.setpoint)
            if pid.setpoint > 2.0*pi:
                pid.setpoint -= 2.0*pi
            if pid.setpoint < 0.0:
                pid.setpoint += 2.0*pi
        setKiwiMotors(x_speed, y_speed, yaw, speed)
    stopMotors()

def setLedMode(mode):
    buf = bytearray(1)
    buf[0] = mode
    i2c.writeto(arduino_address, buf, start = 0, end = len(buf), stop = True)

def resetHeading():
    in_angle = getGyroData()
    while in_angle == None:
        in_angle = getGyroData()
    pid.setSetpoint(in_angle*2*pi/370.0)

def setup(robot_config):
    global cs
    global i2c
    global sensor
    global kit
    global pid
    #global camServo

    kit = MotorKit()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    cs = GPIO.PWM(18, 50)
    cs.start(camServo)
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055(i2c)
    pid = pid_controller(kP, kI, kD, 0.0)
    resetHeading()
    stopMotors()
    stopServo()
    setLedMode(3) #set LEDs to orange

def move(args):
    global stationary
    command=args['button']['command']

    log.debug("move kiwinkidink command : %s", command)

    if True:
        if command == 'u':
            incrementCamServo(-0.2)
            time.sleep(0.02)
        if command == 'd':
            incrementCamServo(0.2)
            time.sleep(0.02)
        if command == 'o':
            kit.motor1.throttle = -1.0
            time.sleep(0.05)
        if command == 'c':
            kit.motor1.throttle = 1.0
            time.sleep(0.05)
        if command == 'off':
            setLedMode(0)
        if command == 'white':
            setLedMode(1)
        if command == 'red':
            setLedMode(2)
        if command == 'orange':
            setLedMode(3)
        if command == 'yellow':
            setLedMode(4)
        if command == 'green':
            setLedMode(5)
        if command == 'blue':
            setLedMode(6)
        if command == 'purple':
            setLedMode(7)
        if command == 'rc':
            os.system("python3 /home/pi/kiwinkidinkilyRemo/remo_swerve.py &")
        if command == 'stationary_on':
            stationary = True
        if command == 'stationary_off':
            stationary = False
    if RCConnected() == False:
        if command == 'l':
            translate(0.0, 0.0, 1.0, 1.0, 0.05) #rotate left
            resetHeading()
        if command == 'r':
            translate(0.0, 0.0, -1.0, 1.0, 0.05) #rotate right
            resetHeading()
        if stationary == False:
            if command == 'f':
                translate(0.0, 1.0, 0.0, 1.0, 0.25) #forward
            if command == 'b':
                translate(0.0, -1.0, 0.0, 1.0, 0.25) #backwards
            if command == 'q':
                translate(-1.0, 0.0, 0.0, 1.0, 0.2) #travel left
            if command == 'e':
                translate(1.0, 0.0, 0.0, 1.0, 0.2) #travel right
        stopMotors()
    stopServo()

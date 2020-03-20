import logging
import time
import board
import busio
from adafruit_motorkit import MotorKit
import RPi.GPIO as GPIO
import adafruit_bno055

log = logging.getLogger('RemoTV.hardware.kiwinkidink')

kit = None
i2c = None
sensor = None
pid = None

camServoMin = 6.0
camServoMax = 11.5
camServo = 8.0

#PID Coefficients
kP = 1.5
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
    i2c.readfrom_into(address, data, start = 0, end=len(data))
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

def translate(x, y, turn_angle, delta_t, speed):
    prev_t = time.time()
    pid.setSetpoint(turn_angle)
    in_angle = getGyroData()
    while in_angle == None:
        in_angle = getGyroData()
    field_zero = in_angle*2*pi/370.0
    pid.resetTime()
    while ((time - prev_t) < delta_t):
        yaw = 0.0
        in_angle = getGyroData()
        if in_angle == None:
            pid.resetTime()
        else:
            angle = in_angle*2*pi/370.0 - field_zero
            if abs(setpoint-(angle+2*pi)) < abs(setpoint-angle):
                angle += 2*pi
            elif abs(setpoint-(angle-2*pi)) < abs(setpoint-angle):
                angle -= 2*pi
            pid.update(angle)
            yaw = -pid.getOutput()
        setKiwiMotors(x, y, yaw, speed)
    stopMotors()

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
    stopMotors()

def move(args):
    command=args['button']['command']

    log.debug("move kiwinkidink command : %s", command)

    if RCConnected() == False:
        if command == 'f':
            translate(0.0, 1.0, 0.0, 1.0, 0.25) #forward
        if command == 'b':
            translate(0.0, -1.0, 0.0, 1.0, 0.25) #backwards
        if command == 'l':
            translate(0.0, 0.0, -0.25, 1.0, 0.25) #rotate left
        if command == 'r':
            translate(0.0, 0.0, 0.25, 1.0, 0.25) #rotate right
        if command == 'q':
            translate(-1.0, 0.0, 0.0, 1.0, 0.25) #travel left
        if command == 'e':
            translate(1.0, 0.0, 0.0, 1.0, 0.25) #travel right
        if command == 'u':
            incrementCamServo(-0.2)
            time.sleep(0.02)
        if command == 'd':
            incrementCamServo(0.2)
            time.sleep(0.02)
        if command == 'o':
            kit.motor1.throttle = 1.0
            time.sleep(0.05)
        if command == 'c':
            kit.motor1.throttle = -1.0
            time.sleep(0.05)
        stopMotors()

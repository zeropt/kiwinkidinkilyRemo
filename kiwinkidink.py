import logging
import time
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import RPi.GPIO as GPIO
import adafruit_bno055

log = logging.getLogger('RemoTV.hardware.kiwinkidink')

mh = None
sensor = None

camServoMin = 6.0
camServoMax = 11.5
camServo = 8.0

def stopMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
    #mhArm.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    #mhArm.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    #mhArm.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    #mhArm.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
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

def setup(robot_config):
    global mh
    global cs
    global sensor
    #global camServo

    mh = Adafruit_MotorHAT(addr=0x60)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    cs = GPIO.PWM(18, 50)
    cs.start(camServo)
    sensor = adafruit_bno055.BNO055()
    stopMotors()

def move(args):
    command=args['button']['command']

    log.debug("move kiwinkidink command : %s", command)

    if command == 'F':
        # Your hardware movement code for forward goes here
        return
    if command == 'B':
        # Your hardware movement code for backwards goes here
        return
    if command == 'L':
        # Your hardware movement code for left goes here
        return
    if command == 'R':
        # Your hardware movement code for right goes here
        return
    if command == 'u':
        incrementCamServo(-0.2)
        time.sleep(0.02)
    if command == 'd':
        incrementCamServo(0.2)
        time.sleep(0.02)
    stopMotors()

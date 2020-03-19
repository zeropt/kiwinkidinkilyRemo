import logging
import time
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import RPi.GPIO as GPIO

log = logging.getLogger('RemoTV.hardware.kiwinkidink')

mh = None

camServoMin = 6.0
camServoMax = 11.5
camServo = 8.0

def incrementCamServo(amount):
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
    global camServo

    mh = Adafruit_MotorHAT(addr=0x60)
    GPIO.setup(18, GPIO.OUT)
    cs = GPIO.PWM(18, 50)
    cs.start(camServo)

def move(args):
    command=args['button']['command']

    log.debug("move kiwinkidink command : %s", command)

    if command == 'F':
        # Your hardware movement code for forward goes here
        return
    elif command == 'B':
        # Your hardware movement code for backwards goes here
        return
    elif command == 'L':
        # Your hardware movement code for left goes here
        return
    elif command == 'R':
        # Your hardware movement code for right goes here
        return
    elif command == 'u':
        incrementCamServo(-0.2)
        time.sleep(0.02)
    elif command == 'd':
        incrementCamServo(0.2)
        time.sleep(0.02)

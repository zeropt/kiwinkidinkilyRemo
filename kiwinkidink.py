import logging
import time

log = logging.getLogger('RemoTV.hardware.kiwinkidink')

def setup(robot_config):
  # your hardware setup code goes here

def move(args):
  command=args['button']['command']
  
  log.debug("move kiwinkidink command : %s", command)

  if command == 'F':
    # Your hardware movement code for forward goes here
  elif command == 'B':
    # Your hardware movement code for backwards goes here
  elif command == 'L':
    # Your hardware movement code for left goes here
  elif command == 'R':
    # Your hardware movement code for right goes here

import logging
import time

log = logging.getLogger('RemoTV.hardware.kiwinkidink')

def setup(robot_config):
    # your hardware setup code goes here
    return

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

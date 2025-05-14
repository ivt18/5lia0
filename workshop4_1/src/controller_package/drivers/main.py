#!/usr/bin/env python2

from .motorDriver import *
from time import sleep
import termios
import sys, tty
import select

# Motor values
velocity = 1 # Must be contained in [-1,1]
gain = .75
trim = -0.0052

def getch_nonblocking(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        else:
            return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

try:
    motor = DaguWheelsDriver()
    while True:
        key = getch_nonblocking(0.05)
        if key:
            if key.lower() == 'w':
                #print("Forward")
                motor.set_wheels_speed(left = velocity * (gain - trim), right = velocity * (gain + trim))
            elif key.lower() == 's':
                #print("Backward")
                motor.set_wheels_speed(left = -velocity * (gain - trim), right = -velocity * (gain + trim))
            elif key.lower() == 'a':
                #print("Left")
                motor.set_wheels_speed(left = 0, right = velocity * (gain + trim))
            elif key.lower() == 'd':
                #print("Right")
                motor.set_wheels_speed(left = velocity * (gain - trim), right = 0)
            elif key.lower() == 'q':
                print("Exiting...")
                break
        else:
            print("Idle")
            motor.set_wheels_speed(0, 0)
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    motor.close()

#!/usr/bin/env python3

from serial import Serial
from pyPS4Controller.controller import Controller
import sys
import time


FREQ_HZ = 10

PITCH_MAX = 10
ROLL_MAX = 10
YAW_MAX = 360
THROTTLE_MAX = 100

PITCH_STEP = 1
ROLL_STEP = 1
YAW_STEP = 10
THROTTLE_STEP = 5

pitch = 0.0
roll = 0.0
yaw = 0.0
throttle = 0.0


# TODO: implement functions to end commands over serial
# TODO: at some point implement functions to connect to bluetooth with rfcomm
#       so it doesn't have to be done outside this script
class DroneController(Controller):
    def __init__(self, port, **kwargs):
        self.com = Serial(port, baudrate=115200)
        self.start = time.time()
        Controller.__init__(self, **kwargs)

    def send_cmd(self):
        t = (time.time() - self.start)
        if t > 1/FREQ_HZ:
            tcmd = int.to_bytes(int(throttle), 2, byteorder='big', signed=True)
            pcmd = int.to_bytes(int(pitch), 2, byteorder='big', signed=True)
            rcmd = int.to_bytes(int(roll), 2, byteorder='big', signed=True)
            ycmd = int.to_bytes(int(yaw), 2, byteorder='big', signed=True)
            cmd = tcmd + pcmd + rcmd + ycmd
            self.com.write(cmd)
            self.start = time.time()

    def on_triangle_press(self):
        global pitch
        pitch += PITCH_STEP
        if pitch > PITCH_MAX:
            pitch = PITCH_MAX
        self.send_cmd()

    def on_x_press(self):
        global pitch
        pitch -= PITCH_STEP
        if pitch < -PITCH_MAX:
            pitch = -PITCH_MAX
        self.send_cmd()

    def on_circle_press(self):
        global roll
        roll += ROLL_STEP
        if roll > ROLL_MAX:
            roll = ROLL_MAX
        self.send_cmd()

    def on_square_press(self):
        global roll
        roll -= ROLL_STEP
        if roll < -ROLL_MAX:
            roll = -ROLL_MAX
        self.send_cmd()

    def on_up_arrow_press(self):
        global throttle
        throttle += THROTTLE_STEP
        if throttle > THROTTLE_MAX:
            throttle = THROTTLE_MAX
        self.send_cmd()

    def on_down_arrow_press(self):
        global throttle
        throttle -= THROTTLE_STEP
        if throttle < 0:
            throttle = 0
        self.send_cmd()

    def on_right_arrow_press(self):
        global yaw
        yaw += YAW_STEP
        if yaw > YAW_MAX:
            yaw = YAW_MAX
        self.send_cmd()

    def on_left_arrow_press(self):
        global yaw
        yaw -= YAW_STEP
        if yaw < -YAW_MAX:
            yaw = -YAW_MAX
        self.send_cmd()


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Please provide serial port!")
        exit(1)

    with open("log.txt", 'w') as f:
        controller = DroneController(
            sys.argv[1],
            interface="/dev/input/js0",
            connecting_using_ds4drv=False
        )
        controller.listen(timeout=60)

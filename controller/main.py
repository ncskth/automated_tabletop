from five_bar import FiveBar
import dynamixel_sdk as dx
import time
import math
import sys
from tkinter import *

DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi


# Control table address
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_MIN_POSITION_LIMIT = 52
ADDR_MAX_POSITION_LIMIT = 48
ADDR_OPERATING_MODE = 11

# Protocol version
PROTOCOL_VERSION = 2

r1 = 23
r2 = 23
r3 = 23
r4 = 23
r5 = 10

left_id = 1
right_id = 2
left_trim = 0
right_trim = 210

center = 4095 // 2

def degree_to_dx(angle):
    # minimum = -1_048_575
    maximum = 4095
    can_move = 2 * math.pi
    degress_per_step = can_move / maximum
    return int(angle/degress_per_step)

def initialize():
    global port
    global handler
    port = dx.PortHandler("/dev/ttyACM0")

    # Initialize PacketHandler Structs
    handler = dx.PacketHandler(2)

    # Open port
    port.openPort()

    # Set port baudrate
    port.setBaudRate(1000000)

    handler.write4ByteTxRx(port, right_id, ADDR_OPERATING_MODE, 4)
    handler.write4ByteTxRx(port, left_id, ADDR_OPERATING_MODE, 4)


    handler.write1ByteTxRx(port, left_id, ADDR_TORQUE_ENABLE, 1)
    handler.write1ByteTxRx(port, right_id, ADDR_TORQUE_ENABLE, 1)
    handler.write4ByteTxRx(port, right_id, ADDR_OPERATING_MODE, 4)
    handler.write4ByteTxRx(port, left_id, ADDR_OPERATING_MODE, 4)
    handler.write1ByteTxRx(port, left_id, ADDR_TORQUE_ENABLE, 1)
    handler.write1ByteTxRx(port, right_id, ADDR_TORQUE_ENABLE, 1)

def move_to(x, y):
    global port
    global handler
    x += 5
    linkage = FiveBar(r1, r2, r3, r4, r5)
    linkage.inverse(x, y)
    if math.isnan(linkage.get_a11()) or math.isnan(linkage.get_a11()):
        return False
    left_angle  = -(math.pi - linkage.get_a11() - math.pi / 2)
    right_angle = -(math.pi / 2 - linkage.get_a42())

    print("-------")
    print(linkage.get_a11() * RAD_TO_DEG, linkage.get_a12() * RAD_TO_DEG, linkage.get_a41() * RAD_TO_DEG, linkage.get_a42() * RAD_TO_DEG)
    print(left_angle * RAD_TO_DEG, right_angle * RAD_TO_DEG)
    left_discrete = center + degree_to_dx(left_angle) + left_trim
    right_discrete = center + degree_to_dx(right_angle) + right_trim

    print(left_discrete, right_discrete)

    if 0 < left_discrete < 4095 and 0 < right_discrete < 4095:
        handler.write4ByteTxRx(port, right_id, ADDR_GOAL_POSITION, right_discrete)
        handler.write4ByteTxRx(port, left_id, ADDR_GOAL_POSITION, left_discrete)
        return True
    return False



initialize()

y = 30
x = 0
def keypress(event):
    global y
    global x
    new_x = x
    new_y = y
    if event.char == 'w':
        new_y += 0.1
    elif event.char == 's':
        new_y -= 0.1
    elif event.char == 'a':
        new_x -= 0.1
    elif event.char == 'd':
        new_x += 0.1
    else:
        pass
    if move_to(new_x, new_y):
        x = new_x
        y = new_y
        print(x, y)

root = Tk()
mainCanvas = Canvas(root, width=200, height=200)
root.bind('w', keypress)
root.bind('s',keypress)
root.bind('a',keypress)
root.bind('d',keypress)
move_to(x, y)
root.mainloop()

time.sleep(0.5)
while True:
    move_to(0, 25)
    time.sleep(0.2)
    move_to(0, 42)
    time.sleep(0.2)

time.sleep(2)
handler.write1ByteTxRx(port, left_id, ADDR_TORQUE_ENABLE, 0)
handler.write1ByteTxRx(port, right_id, ADDR_TORQUE_ENABLE, 0)

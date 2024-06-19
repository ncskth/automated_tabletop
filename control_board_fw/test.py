import argparse
import os
import time
import socket
import sys
import struct
import math
import serial

# regular frames
ID_NACK = 0
ID_ACK = 1
ID_HANDSHAKE = 2

ID_SET_CONTROL = 3
ID_GET_POSITION = 4
ID_SET_FAN = 5

PORT = 1337

def send(buf):
    if args.type == "wifi":
        totalsent = 0
        while totalsent < len(buf):
            sent = s.send(buf[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    if args.type == "uart":
        print("chungus")
        ser.write(buf)

def receive(msg_len):
    if args.type == "wifi":
        chunks = []
        bytes_recd = 0
        while bytes_recd < msg_len:
            chunk = s.recv(min(msg_len - bytes_recd, 2048))
            if chunk == b'':
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
            buf = b''.join(chunks)
        return buf
    if args.type == "uart":
        buf = ser.read(msg_len)
        return buf

def wait_ack():
    buf = receive(1)
    if buf[0] == 1:
        return True
    else:
        return False

def get_position():
    buf = bytearray(struct.pack("<B", ID_GET_POSITION))
    send(buf)
    rx_buf = receive(8)
    (pos_x, pos_y) = struct.unpack("<ff", rx_buf)
    return (pos_x, pos_y)

def wait_arrive(x, y):
    while True:
        pos_x, pos_y = get_position()
        if math.sqrt((pos_x - x) ** 2 + (pos_y - y) ** 2) < 0.5:
            break

parser = argparse.ArgumentParser()
parser.add_argument('type', choices=['uart', 'wifi'])
parser.add_argument('address', type=str, help = "[serial path]/[ip_address:port]")
parser.add_argument('--control', type=float, nargs=3,)
parser.add_argument('--enable-fan', action="store_true")
parser.add_argument('--disable-fan', action="store_true")

args = parser.parse_args()



print("connecting...")
s = None
ser = None
if args.type == "wifi":
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.address, 1337))

if args.type == "uart":
    ser = serial.Serial(args.address, 1e6, rtscts=True)
    ser.read_all()
print("connected")

if args.control:
    buf = bytearray(struct.pack("<Bfff", ID_SET_CONTROL, args.control[0], args.control[1], args.control[2]))
    print("sending control command")
    send(buf)
    if not wait_ack():
        print("failed to send control command")
elif args.enable_fan:
    buf = bytearray(struct.pack("<BB", ID_SET_FAN, 1))
    print("sending fan command")
    send(buf)
    if not wait_ack():
        print("failed to send fan command")
elif args.disable_fan:
    buf = bytearray(struct.pack("<BB", ID_SET_FAN, 0))
    print("sending fan command")
    send(buf)
    if not wait_ack():
        print("failed to send fan command")
else:
    while True:
        speed = 160

        left = -10
        right = 10
        up = 28
        down = 28

        buf = bytearray(struct.pack("<Bfff", ID_SET_CONTROL, left, up, speed))
        send(buf)
        wait_ack();
        wait_arrive(left, up)
        begin = time.time()
        # time.sleep(2)
        # print(get_position())


        buf = bytearray(struct.pack("<Bfff", ID_SET_CONTROL, right, up, speed))
        send(buf)
        wait_ack();
        wait_arrive(right, up)
        end = time.time()
        # time.sleep(2)

        print("delta", end - begin)
        # buf = bytearray(struct.pack("<Bfff", ID_SET_CONTROL, right, up, speed))
        # send(buf)
        # time.sleep(1)

        # buf = bytearray(struct.pack("<Bfff", ID_SET_CONTROL, left, down, speed))
        # send(buf)
        # time.sleep(1)

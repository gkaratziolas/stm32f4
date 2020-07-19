#!/usr/bin/python3

import serial
import sys
import time
import math
import random

BAUD_RATE = 115200

STARTA = 0x12
STARTB = 0x34

COMMAND_RESET       = 1
COMMAND_LED         = 2
COMMAND_MOVE        = 3
COMMAND_GCODE       = 4
COMMAND_GCODE_BUSY  = 5
COMMAND_GCODE_ERR   = 6

def serial_init(port):
	try:
		ser = serial.Serial(port, BAUD_RATE, timeout=1)
		print("connected to port {}".format(ser.name))
		return ser
	except serial.serialutil.SerialException:
		return None

def serial_deinit(ser):
	port_name = ser.name
	ser.close()
	print("closed connection to port {}".format(port_name))

def send_gcode_string(ser, gcode_string):
	ser.write((STARTA).to_bytes(1, "little"))
	ser.write((STARTB).to_bytes(1, "little"))

	ser.write((COMMAND_GCODE).to_bytes(1, "little"))
	ser.write((len(gcode_string)).to_bytes(1, "little"))
	for char in gcode_string:
		byte = ord(char)
		ser.write((byte).to_bytes(1, "little"))
	ser.write((0x0f).to_bytes(1, "little"))

def send_command(ser, command = 0x00):
	ser.write((STARTA).to_bytes(1, "little"))
	ser.write((STARTB).to_bytes(1, "little"))
	ser.write((command).to_bytes(1, "little"))
	ser.write((0x00).to_bytes(1, "little"))
	ser.write((0x0f).to_bytes(1, "little"))

def move_motors(ser, x, y):
	ser.write((STARTA).to_bytes(1, "little"))
	ser.write((STARTB).to_bytes(1, "little"))
	ser.write((0x03).to_bytes(1, "little"))
	ser.write((0x08).to_bytes(1, "little"))
	ser.write((x).to_bytes(4, "little", signed=True))
	ser.write((y).to_bytes(4, "little", signed=True))
	ser.write((0x0f).to_bytes(1, "little"))

def leds_on(ser, led0, led1, led2, led3):
	ser.write((STARTA).to_bytes(1, "little"))
	ser.write((STARTB).to_bytes(1, "little"))
	ser.write((0x02).to_bytes(1, "little"))
	ser.write((0x04).to_bytes(1, "little"))
	ser.write((led0).to_bytes(1, "little"))
	ser.write((led1).to_bytes(1, "little"))
	ser.write((led2).to_bytes(1, "little"))
	ser.write((led3).to_bytes(1, "little"))
	ser.write((0x0f).to_bytes(1, "little"))

def read_serial(ser):
	bytes_available = ser.in_waiting
	if bytes_available == 0:
		return
	x = ser.read(bytes_available)
	print(x, len(x))
	print()

def transmit_gcode_file(ser, file_name):
	with open(file_name, "r") as f:
		while True:
			line = f.readline()
			if not line:
				break
			line = line.replace("\n", "")
			print(line)
			x = input()
			send_gcode_string(ser, line)
			time.sleep(0.1)
			read_serial(ser)


if __name__ == "__main__":
	if len(sys.argv) < 3:
		print("error: please specify serial port and gcode file")
		sys.exit(1)
	ser = serial_init(sys.argv[1])
	if ser == None:
		print("error: can't open serial port {}".format(sys.argv[1]))
		sys.exit(1)
	
	transmit_gcode_file(ser, sys.argv[2])
	serial_deinit(ser)

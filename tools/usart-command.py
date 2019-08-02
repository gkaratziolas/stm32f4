#!/usr/bin/python3

import serial
import sys
import time
import random

BAUD_RATE = 115200

STARTA = 0x12;
STARTB = 0x34;

def serial_init(port):
	try:
		ser = serial.Serial(port, BAUD_RATE)
		print("connected to port {}".format(ser.name))
		return ser
	except serial.serialutil.SerialException:
		return None

def serial_deinit(ser):
	port_name = ser.name
	ser.close()
	print("closed connection to port {}".format(port_name))

def send_command(ser, command = 0x00):
	ser.write((STARTA).to_bytes(1, "little"))
	ser.write((STARTB).to_bytes(1, "little"))
	ser.write((command).to_bytes(1, "little"))
	ser.write((0x00).to_bytes(1, "little"))
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

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("error: please specify serial port.")
		sys.exit(1)
	ser = serial_init(sys.argv[1])
	if ser == None:
		print("error: can't open serial port {}".format(sys.argv[1]))
		sys.exit(1)
	try:
		command = sys.argv[2]
		try:
			command = int(command, 16)
			send_command(ser, command)
			sys.exit()
		except ValueError:
			print("error: second argument should be single hex byte. i.e. 0x13")
			sys.exit(1)
	except IndexError:
		command = 0x00

	while(1):
		leds_on(ser, random.randint(0,1), random.randint(0,1), random.randint(0,1), random.randint(0,1))
		time.sleep(0.1)
	serial_deinit(ser)

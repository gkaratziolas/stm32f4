import serial
import sys

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

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("error: please specify serial port.")
		sys.exit(1)
	ser = serial_init(sys.argv[1])
	if ser == None:
		print("error: can't open serial port {}".format(sys.argv[1]))
	try:
		command = sys.argv[2]
		try:
			command = int(command, 16)
		except ValueError:
			print("error: second argument should be single hex byte. i.e. 0x13")
	except IndexError:
		command = 0x00

	send_command(ser, command)
	serial_deinit(ser)

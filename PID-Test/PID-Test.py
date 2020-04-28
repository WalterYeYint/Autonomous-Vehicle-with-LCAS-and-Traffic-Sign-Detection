import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(5)
while True:
	pid_parameters = input("Enter data[0]:").split()
	Kp, Ki, Kd = [int(i) for i in pid_parameters]
	ser.write(b'%d\t%d\t%d\t' % (Kp, Ki, Kd))

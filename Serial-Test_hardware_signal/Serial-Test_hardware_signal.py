import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(7)
while True:
    for x in range(0, 256, 1):
        print(x)
        ser.write(b'%f\t%f\t%f\t%f\t' % (float(x), 10, 10, 10))
        # data = ser.readline()
        # print("from", int(data))
        # time.sleep(0.01)
    for y in range(255, -1, -1):
        print(y)
        ser.write(b'%f\t%f\t%f\t%f\t' % (float(y), 10, 10, 10))
        # data = ser.readline()
        # print("from", data)
        # time.sleep(0.01)
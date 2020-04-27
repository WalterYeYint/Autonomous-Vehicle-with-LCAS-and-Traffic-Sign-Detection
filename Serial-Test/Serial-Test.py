# #!/usr/bin/env python3
# import serial
# import time
# x = 35
# y = 0
# z = 1
# if __name__ == '__main__':
#     ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
#     ser.flush()
#     while True:
#         ser.write(b'%d\t%d\t%d\t' % (x, y, z))
#         #ser.write(b"1\t")


import serial
import time
i = 0
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(5)
while i<3:
	ser.write(b'35\t0\t7\t')
	i += 1

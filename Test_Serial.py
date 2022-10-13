import serial
import time
import numpy as np
import random
from math import sin, cos, pi

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'
ser.timeout = 0


ser.open()


R = 1000
n = 50

R2 = 100;

xx = np.arange(-R2,2,R2)
xx = np.append(xx,np.zeros(R2, dtype=int)+R2)
xx = np.append(xx,np.flip(np.arange(-R2,2,R2))+1)
xx = np.append(xx,np.zeros(R2, dtype=int)-R2)


yy = np.zeros(R2, dtype=int)+R2
yy = np.append(yy,np.flip(np.arange(-R2,2,R2))+1)
yy = np.append(yy,np.zeros(R2, dtype=int)-R2)
yy = np.append(yy,np.arange(-R2,2,R2))

print(len(xx),len(yy))



for i in range(5):
	print("Motor enable in:" + str(5-i) + "sec")
	time.sleep(1)

for i in range(5):
	print("Sequence start in:" + str(5-i) + "sec")
	time.sleep(1)


"""
for j in range(50):
	for i in range(len(xx)):
		x = xx[i]
		y = yy[i]
		data = b'S' + int(x).to_bytes(2, byteorder='little', signed=True) + int(y).to_bytes(2, byteorder='little', signed=True)
		time.sleep(0.005)
		ser.write(data)
		print(x, y)
	#print(ser.readline())
"""


for j in range(n):
	for i in range(n):
		x = int(cos(i/n*pi*2)*R)
		y = int(sin(i/n*pi*2)*R)
		data = b'S' + x.to_bytes(2, byteorder='little', signed=True) + y.to_bytes(2, byteorder='little', signed=True)
		time.sleep(0.05)
		ser.write(data)
		print(x, y)
		#print(ser.readline())

"""
while True:
	x = (int(random.random()*R*2-R))
	y = (int(random.random()*R*2-R))
	data = b'S' + x.to_bytes(2, byteorder='little', signed=True) + y.to_bytes(2, byteorder='little', signed=True)
	time.sleep(1)
	ser.write(data)
	print()
	print(x, y)
	print(ser.readline())

	
#"""
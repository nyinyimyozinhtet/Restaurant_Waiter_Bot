import serial
import time

ser = serial.Serial(
    port = '/dev/ttyS0'
    baudrate = 9600,
    parity = seial.PARITY_NONE,
    stopbits = serial.EIGHTBITS,
    timeout =1
)

while 1:
    x = ser.readline()
    print(x)
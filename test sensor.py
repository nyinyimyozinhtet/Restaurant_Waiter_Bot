import RPi.GPIO as GPIO
import time

a=b=c=d=0

sensor1 = 8
sensor2 = 22
sensor3 = 18
sensor4 = 16

sensors = {
    "sensor1":sensor1,
    "sensor2":sensor2,
    "sensor3":sensor3,
    "sensor4":sensor4,
    }
GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensor1,GPIO.IN)
GPIO.setup(sensor2,GPIO.IN)
GPIO.setup(sensor3,GPIO.IN)
GPIO.setup(sensor4,GPIO.IN)

try:
    while True:
        for x,y in sensors.items():
            if GPIO.input(y) == False:
                print(f"{x}", end="", flush=True)
                
        time.sleep(0.5)
        print("")
except KeyboardInterrupt:

    GPIO.cleanup()
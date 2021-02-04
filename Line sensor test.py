from __future__ import division
import time
# from goto import with_goto
import RPi.GPIO as GPIO
lf1,lf2,lf3,lf4=0,0,0,0

#line follow module port define
l1 = 6
l2 = 13
r2 = 19
r1 = 26

#sensors init
lf1,lf2,lf3,lf4,lf=0,0,0,0,0

#Initialise GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(l1,GPIO.IN)
GPIO.setup(l2,GPIO.IN)
GPIO.setup(r1,GPIO.IN)
GPIO.setup(r2,GPIO.IN)
# Read tracking senbsors's data
def read_sensors():
    global lf1,lf2,lf3,lf4,lf
    lf1 = GPIO.input(l1)
    lf2 = GPIO.input(l2)
    lf3 = GPIO.input(r2)
    lf4 = GPIO.input(r1)

def destroy():
    GPIO.cleanup()
   
def main():
    global lf1,lf2,lf3,lf4,lf
    y=0
    while True:
        read_sensors()
        lf=str(lf1)+str(lf2)+str(lf3)+str(lf4)
        print (lf)
        time.sleep(0.5)
        
 
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
	#robot car stop
        destroy()

from __future__ import division
import time
import RPi.GPIO as GPIO

#L298N port define
in1 = 3
in2 = 2
in3 = 15
in4 = 18
ena = 14
enb = 4

#PWM set up
#setup mortor pin input output
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)

#initialize all motor output 0
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
#------------------------------------------------

#set Ultrasonic GPIO Pins
TRIGGER = 23
ECHO = 24

#set GPIO direction (IN / OUT) for Ultrasonic sensor
GPIO.setup(TRIGGER, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
#---------------------------------------------------------

#from left to right ,4 sensors are connected
lf1,lf2,lf3,lf4,lf=0,0,0,0,0
l1 = 6
l2 = 13
r2 = 19
r1 = 26

#Initialise  for line sensor GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(l1,GPIO.IN)
GPIO.setup(l2,GPIO.IN)
GPIO.setup(r1,GPIO.IN)
GPIO.setup(r2,GPIO.IN)


p1 = GPIO.PWM(ena,1000)
p2 = GPIO.PWM(enb,1000)

p1.start(0)
p2.start(0)

def distance():
    # set Trigger to HIGH
    GPIO.output(TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

# def dis():
#     dist = distance()
#     print("Distance = %.1f cm" % dist)

# Read tracking senbsors's data
def read_sensors():
    global lf1,lf2,lf3,lf4
    lf1 = GPIO.input(l1)
    lf2 = GPIO.input(l2)
    lf3 = GPIO.input(r2)
    lf4 = GPIO.input(r1)
#Set motor speed
def normal_condition():
    print("Normal Condition")
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)

#Robot car forward
def go_forward(speed):
    print("forward")
    p1.ChangeDutyCycle(speed)
    p2.ChangeDutyCycle(speed)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
#Robot car backwards
def go_back(speed):
    print("backward")
    p1.ChangeDutyCycle(speed)
    p2.ChangeDutyCycle(speed)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

#Robot car turn left
def turn_left(speed):
    print("left")
    p1.ChangeDutyCycle(speed)
    p2.ChangeDutyCycle(speed)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

#Robot turn right
def turn_right(speed):
    print("right")
    p1.ChangeDutyCycle(speed)     #right motor speed
    p2.ChangeDutyCycle(speed)     #left motor speed
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

#Robot stop move
def stop():
#     print("stop")
    p1.ChangeDutyCycle(0)     #right motor speed
    p2.ChangeDutyCycle(0)     #left motor speed
    GPIO.output(in1,GPIO.LOW)   #right motor
    GPIO.output(in2,GPIO.LOW)   #right motor
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)

#Reset all channels    
def destroy():
    GPIO.cleanup()
    #p1.start(s)
    #p2.start(s)

#Robot car moves along the black line
def tracking():
    tn = int(input("Table No:"))
    print("Going to Table:", tn)
    tnc = 0
    tdc = 0
    while True:
        dist = distance()
        if(dist < 5):
            stop()
            print("stop")
            time.sleep(0.5)
        else:
            read_sensors()
            lf=str(lf1)+str(lf2)+str(lf3)+str(lf4)
            print (lf)
            if(lf=='0000'):
                go_back(25)
                continue
            if(lf == '1111' and tdc == 0 and tnc == 0 and (tn == 1 or tn == 3)):
                tdc += 1
                turn_left(38)
                time.sleep(1)
            if(lf == '1111' and tdc == 1 and (tn == 1 or tn == 3)):
                tnc += 1
                time.sleep(1)
            if(lf == '1111' and tnc == 2 and tn == 1):
                stop()
                time.sleep(0.5)
                print("Arrived Table 1!")
                break
            if(lf == '1111' and tnc == 3 and tn == 3):
                stop()
                time.sleep(0.2)
                print("Arrived Table 3!")
                break
            if(lf == '1111' and tdc == 0 and tnc == 0 and (tn == 2 or tn == 4)):
                tdc += 1
                turn_right(32)
                time.sleep(0.5)
            if(lf == '1111' and tdc == 1 and (tn == 2 or tn == 4)):
                tnc += 1
                time.sleep(0.5)
            if(lf == '1111' and tnc == 2 and tn == 2):
                stop()
                print("Arrived Table 2!")
                break
            if(lf == '1111' and tnc == 3 and tn == 4):
                stop()
                print("Arrived Table 4!")
                break
            
            if(lf=='0110'):
                go_forward(35)
                continue
                
            if(lf=='0111' or lf=='0001'):
                turn_right(35)
                continue
            
            if(lf=='1000' or lf=='1110'):
                turn_left(35)
                continue
            
            if(lf=='0011'):
            #             #set_speed(low_speed,low_speed)
                turn_right(33)
                continue
        #         
            if(lf=='1100'): 
        #             set_speed(mid_speed,mid_speed)
                turn_left(33)
                continue
        #         
            else:
                stop()
            
            
if __name__ == '__main__':
    try:
    #start to line follow
        tracking()
    except KeyboardInterrupt:
    #robot car stop
        destroy()


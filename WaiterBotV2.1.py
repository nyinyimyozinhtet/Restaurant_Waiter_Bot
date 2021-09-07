from __future__ import division
import time
import datetime
import RPi.GPIO as GPIO
import pigpio
import serial

ser = serial.Serial(
        port = '/dev/ttyS0',
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
)

l1 = 6
l2 = 12
r2 = 19
r1 = 26


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
    lf1,lf2,lf3,lf4,lf=0,0,0,0,0
    lf1 = GPIO.input(l1)
    lf2 = GPIO.input(l2)
    lf3 = GPIO.input(r2)
    lf4 = GPIO.input(r1)

def destroy():
    GPIO.cleanup()

#set Ultrasonic GPIO Pins
TRIGGER = 23
ECHO = 24

#set GPIO direction (IN / OUT) for Ultrasonic sensor
GPIO.setup(TRIGGER, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
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

#Setup Pins for GPIO
GPIO.setmode(GPIO.BCM)
#Create pigpio object called pi
pi = pigpio.pi()

in1 = 3
in2 = 2
in3 = 27
in4 = 22
ena = 13
enb = 18

GPIO.setup(in1, GPIO.OUT, initial = 0)
GPIO.setup(in2, GPIO.OUT, initial = 0)
GPIO.setup(in3, GPIO.OUT, initial = 0)
GPIO.setup(in4, GPIO.OUT, initial = 0)

#Make sure car wont move until everything is set up
pi.hardware_PWM(ena, 2000, 0) # 2kHz 0% dutycycle
pi.hardware_PWM(enb, 2000, 0) # 2kHz 0% dutycycle
#Set both wheels to go forwards
GPIO.output(in1, GPIO.HIGH)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.HIGH)
GPIO.output(in4, GPIO.LOW)


#Setup PID constants
Kp = 210744
Ki = 2200
Kd = 1694
P = 0
I = 0
D = 0
previous_error = 0
time_previous = 0
speedvalue = 350000 #___% of Full speed

#setup sliders to change PID constants
# root = Tk()
# Prop = DoubleVar()
# Inte = DoubleVar()
# Deri = DoubleVar()
# scaleP = Scale(root, variable = Prop, from_=0, to=1000000, length = 1000, resolution = 1, orient=HORIZONTAL)
# scaleP.pack(anchor = CENTER)
# scaleI = Scale(root, variable = Inte, from_=0, to=10000, length = 1000, resolution = 1, orient=HORIZONTAL)
# scaleI.pack(anchor = CENTER)
# scaleD = Scale(root, variable = Deri, from_=0, to=10000, length = 1000, resolution = 1, orient=HORIZONTAL)
# scaleD.pack(anchor = CENTER)
# speedval = DoubleVar()
# scaleS = Scale(root, variable = speedval, from_=0, to=1000000, length = 1000, resolution = 1, orient=HORIZONTAL)
# scaleS.pack(anchor = CENTER)

def PIDcontrol(error, Kp, Ki, Kd):
    global P, I, D, previous_error, time_previous
    
    time_current = time.time()
    delta_time = time_current - time_previous
    
    P = error
    I += (error*delta_time)
    D = (error - previous_error)/delta_time
    
    PID = Kp*P + Kd*D + Ki*I
#     
#     print('P Value:', P)
#     print('I Value:', I)
#     print('D Value:', D)
    
    previous_error = error
    time_previous = time_current
    
    return PID

def constrain(value, min_value, max_value):
    return min(max_value, max(min_value, value))

def controlMot(PID, speedvalue):
    PWML = int(constrain(speedvalue + PID, -125000, 1000000))
    PWMR = int(constrain(speedvalue - PID, -125000, 1000000))
    
    return PWML, PWMR

def toError():
    read_sensors()
    lf=str(lf1)+str(lf2)+str(lf3)+str(lf4)
    print (lf)
    
    if (lf == '1000'):
        return -1.15
    if (lf =='1110'): #Left
        return -1.15
    if (lf == '1100'):
        return -1
    if (lf == '0011'):
        return 1
    if (lf =='0111'): #Right
        return 1.15
    if (lf =='0001'):
        return 1.15
    if (lf =='0110' or lf =='0100' or lf =='0010'): #Straight
        return 0
    if (lf == '1111'):
        return 69
    if (lf == '0000'):
        return 42
    else:
        return 99 #Something is wrong
    
def stop():
    PWMvalueL = 0
    PWMvalueR = 0
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    
# def tableinput():
#     for y > 4:
#         x = ser.read(1)
#         y = x.decode('utf-8')
#         return y    

def start_bot():
    global I
    x = ser.read(1)
    y = x.decode('utf-8')
    

#     Tb = input("Please input Table Number: ")
    Tb = int(y)
    tnc = 0
    tncc = 0
    while Tb <= 4:
        
        error = toError()
        if (error == 99):
            I = 0
            PWMvalueL,PWMvalueR = 0,0
        if (error == 69):
#             stop()
            PID = PIDcontrol(error, Kp, Ki, Kd)
            PWMvalueL, PWMvalueR = controlMot(PID, speedvalue)
        if (error == 69 and tncc == 0 and tnc == 0):
            tnc += 1
        if (error != 69 and tncc == 0 and tnc == 1):
            tncc += 1
        if (error == 69 and tncc == 1 and tnc == 1):
            tnc += 1
        if (error != 69 and tncc == 1 and tnc == 2):
            tncc += 1
        if (error == 69 and tncc == 2 and tnc == 2):
            tnc += 1
        if (error != 69 and tncc == 2 and tnc == 3):
            tncc += 1
        if (error == 69 and tncc == 3 and tnc == 3):
            tnc += 1
        if (error != 69 and tncc == 3 and tnc == 4):
            tncc += 1
        if (error == 69 and tncc == 4 and tnc == 4):
            tnc += 1
        if (error != 69 and tncc == 4 and tnc == 5):
            tncc += 1
        if (error == 69 and tncc == 5 and tnc == 5):
            tnc += 1
        if (error != 69 and tncc == 5 and tnc == 6):
            tncc += 1
        if (Tb == 1 and tnc == 1):
            stop()
            break
        if (Tb == 2 and tnc == 2):
            stop()
            break
        if (Tb == 3 and tnc == 3):
            stop()
            break
        if (Tb == 4 and tnc == 4):
            stop()
            break
        if (tnc == 5):
            stop()
            break
#             x = input("press 1 to continue: ")
#             if x == 1:
#                 continue
#             else:
#                 stop()
            
#             continue
#             time.sleep(0.5)
#         if (Tb == 1 and tnc == 200):
#             stop()
#             break
        else:
            PID = PIDcontrol(error, Kp, Ki, Kd)
            PWMvalueL, PWMvalueR = controlMot(PID, speedvalue)
            print('PID value:', PID)
            
        print('Table Number counter: ',tnc)
        print('TNCC: ',tncc)
        print('Going table:', Tb)
        print('Left Mot:', abs(PWMvalueL), 'Right Mot:', abs(PWMvalueR))
        
        if (PWMvalueL < 0):
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.HIGH)
        elif (PWMvalueL == 0):
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.LOW)
        else:
            GPIO.output(in3, GPIO.HIGH)
            GPIO.output(in4, GPIO.LOW)
        if (PWMvalueR < 0):
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        elif(PWMvalueR == 0):
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            
        pi.hardware_PWM(ena, 2000, abs(PWMvalueR)) # 2kHz varying% dutycycle
        pi.hardware_PWM(enb, 2000, abs(PWMvalueL)) # 2kHz varying% dutycycle



if __name__ == '__main__':
    try:
        init_time = time.time()
        dist = distance()
        if(dist < 20):
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.LOW)
            print("Something Blocking!!")
        else:
            start_bot()
            time.sleep(0.1)
            print('Loop time:',time.time()-init_time)
    except KeyboardInterrupt:
        destroy()


import tensorflow as tf
#from tensorflow.examples.tutorials.mnist import input_data
import numpy as np
import cv2
import keyboard
import picamera
import time
import datetime
import RPi.GPIO as GPIO
import pigpio
import os

#Setup Pins for GPIO
GPIO.setmode(GPIO.BCM)
#Create pigpio object called pi
pi = pigpio.pi()

in1 = 17
in2 = 22
in3 = 23
in4 = 24

GPIO.setup(in1, GPIO.OUT, initial = 0)
GPIO.setup(in2, GPIO.OUT, initial = 0)
GPIO.setup(in3, GPIO.OUT, initial = 0)
GPIO.setup(in4, GPIO.OUT, initial = 0)
#Make sure car wont move until everything is set up
pi.hardware_PWM(18, 2000, 0) # 2kHz 0% dutycycle
pi.hardware_PWM(13, 2000, 0) # 2kHz 0% dutycycle
#Set both wheels to go forwards
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.HIGH)
GPIO.output(in3, GPIO.HIGH)
GPIO.output(in4, GPIO.LOW)

#Setup PID constants
Kp = 64876
Ki = 9297.5
Kd = 8367.8
P = 0
I = 0
D = 0
previous_error = 0
time_previous = 0
speed = 450000 #45% Full speed

#Set numpy to print all values
np.set_printoptions(threshold=np.inf)

#set resolution of input images
n_size = 12
imageSize = n_size*n_size

#initialize placeholders for input data and labels
x = tf.placeholder('float',[None, imageSize]) #flatten image, the data
y = tf.placeholder('float') #label of the data

def PIDcontrol(error, Kp, Ki, Kd):
    global P, I, D, previous_error, time_previous
    
    time_current = time.time()
    delta_time = time_current - time_previous
    
    P = error
    I += (error*delta_time)
    D = (error - previous_error)/delta_time
    
    PID = Kp*P + Kd*D + Ki*I
    
    previous_error = error
    time_previous = time_current
    
    return PID

def constrain(value, min_value, max_value):
    return min(max_value, max(min_value, value))

def controlMot(PID, speedvalue):
    PWML = int(constrain(speedvalue - PID, 0, 1000000))
    PWMR = int(constrain(speedvalue + PID, 0, 1000000))
    return PWML, PWMR

def getTime():
    currenttime = datetime.datetime.now()
    return currenttime.strftime("%H:%M:%S")

def startMoving():
    if keyboard.is_pressed('x'):
        return 1
    else:
        return 0

def openPiCamera():
    width = 640
    height = 480
    camera = picamera.PiCamera() #Initialize camera
    camera.resolution = (width, height)
    camera.exposure_mode = 'off'
    camera.shutter_speed = 33333 #30fps, 1/30 = 33333us
    camera.iso = 100
    time.sleep(2) #Wait for camera to turn on

    img1 = np.empty((height, width,3 ), dtype = np.uint8)
    return img1, camera, width, height

def toError(predictions):
    result = [predictions[0][0], predictions[0][1], predictions[0][2]]
    if result[0] > result[1] and result[0] > result[2]: #Left
        return -1
    if result[1] > result[0] and result[1] > result[2]: #Right
        return 1
    if result[2] > result[0] and result[2] > result[1]: #Straight
        return 0
    else:
        return 99 #Something is wrong

def Init(Size):
    #Matrix must be shape(l-1,l)
    if len(Size) == 1: #Only for biases
        return np.asarray(np.full(Size, 0.0), np.float32) #Usually 0, but 0.01 will maybe or maybe not work out
    else: #Only for ANN, wont work for other types of networks
        return np.asarray(np.random.randn(Size[0],Size[1])*np.sqrt(2/(Size[0])), np.float32) #Return a samples from normal distribution and use He Init

def Make_ANN(data, layers, neurons_per_layer, no_outputs):
    hidden_layer_values = []
    hidden_layers = []
    for layer in range(0,layers):
        if layer == 0:
            prev_layer_neurons = imageSize
        else:
            prev_layer_neurons = neurons_per_layer[layer-1]

        hidden_layer_values.append([{'weights':tf.Variable(Init([prev_layer_neurons, neurons_per_layer[layer]])),
                                 'biases':tf.Variable(Init([neurons_per_layer[layer]]))}])

        print(hidden_layer_values[layer][0])

        if layer == 0:
            prev_layer = data
        else:
            prev_layer = hidden_layers[layer-1]

        hidden_layers.append(tf.add(tf.matmul(prev_layer, hidden_layer_values[layer][0]['weights']), hidden_layer_values[layer][0]['biases']))
        hidden_layers[layer] = tf.nn.leaky_relu(hidden_layers[layer], alpha = 0.01, name = None)

        if layer == layers-1:
            keep_prob = tf.placeholder(tf.float32)
            dropout_layer =  tf.nn.dropout(hidden_layers[layer], keep_prob)

            output_layer_values = {'weights':tf.Variable(tf.random_normal([neurons_per_layer[layer], no_outputs])),
                                        'biases':tf.Variable(tf.random_normal([no_outputs]))}
            output_layer = tf.matmul(dropout_layer, output_layer_values['weights']) + output_layer_values['biases']

    return output_layer, keep_prob

def LeakyRelu(inputs):
        output = np.where(inputs > 0, inputs, inputs * 0.01)
        return output

def Make_npANN(data, layerWeights, layerBiases):
        HiddenLayer = np.add(np.matmul(data, layerWeights[0]),layerBiases[0])
        HiddenLayer = LeakyRelu(HiddenLayer)
        OutputLayer = np.add(np.matmul(HiddenLayer, layerWeights[1]), layerBiases[1])
        OutputLayer = LeakyRelu(OutputLayer)
        return OutputLayer
    
def ExtractWeights(all_vars, sess):
    layerWeights = []
    layerBiases = []
    i = 0
    biascount = 0
    weightcount = 0
    for v in all_vars:
        print('i = ',i)
        if(i % 2 == 0):
            v_ = np.array(sess.run(v))
            weightvalues = []
            for values in v_:
                weightvalues.append(values)
            layerWeights.append(weightvalues)
            layerWeights[weightcount] = np.array(layerWeights[weightcount])
            print(layerWeights[weightcount])
            # print(layerWeights[weightcount].shape)
            #cv2.imshow('Weights '+str(weightcount), cv2.resize(layerWeights[weightcount], (300,300)))
            weightcount += 1
        else:
            v_ = np.array(sess.run(v))
            biasvalues = []
            for values in v_:
                biasvalues.append(values)
            layerBiases.append(biasvalues)
            layerBiases[biascount] = np.array(layerBiases[biascount])
            print(layerBiases[biascount])
            # print(layerBiases[biascount].shape)
            #cv2.imshow('Biases '+str(biascount), cv2.resize(layerBiases[biascount], (300,300)))
            biascount += 1
        i += 1
    return layerWeights, layerBiases

def train_neural_network(data, layers, neurons_per_layer, no_outputs):
    global time_previous, manual, Kp, Ki, Kd, previous_error
    
    prediction, keep_prob = Make_ANN(data, layers, neurons_per_layer, no_outputs)
    
    with tf.Session() as sess:
        saver = tf.train.Saver()
        saver.restore(sess, "/home/pi/Desktop/For Analysis/12x12_04_12_18_All_21_BalRan90000_leaky/model_04_12_18_All_21_BalRan90000_leaky.ckpt")
        print("Model restored.")
        all_vars = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES)
        print(all_vars)
        layerWeights, layerBiases = ExtractWeights(all_vars, sess)

    print("Opening Camera...")
    img, camera, width, height = openPiCamera()

    fps_list = []
    fps = 0
    max_fps = 0;
    min_fps = 100;
    output_list = []
    init_time = time.time()
    while(1):
        currenttime = time.time()
            
        camera.capture(img, 'bgr', use_video_port = True)
            
        #currenttime = time.time()
        img1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img2 = img1[240:480]
        img2 = cv2.resize(img2, (n_size,n_size))
        #cv2.imshow('Input',cv2.resize(img2,(300,300)))
        img2 = np.array(img2).reshape([-1,imageSize])
        #print('Image Processing Time Taken:', time.time()-currenttime, 'sec')

        #currenttime = time.time()
        pred = Make_npANN(img2, layerWeights, layerBiases)
        #print('NN Time Taken:', time.time()-currenttime, 'sec')

        error = toError(pred)
        move = startMoving()
        print('Error Value:', error)
            
        if move == 0:
            PWMvalueL = 0
            PWMvalueR = 0
            time_previous = time.time()
            move_time = time.time()
            
            #Set both wheels to go forwards
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            GPIO.output(in3, GPIO.HIGH)
            GPIO.output(in4, GPIO.LOW)
            
            if keyboard.is_pressed('a'):
                if keyboard.is_pressed('j'):
                    PWMvalueL = 450000
                    PWMvalueR = 0
                if keyboard.is_pressed('l'):
                    PWMvalueL = 0
                    PWMvalueR = 450000
                if keyboard.is_pressed('i'):
                    PWMvalueL = 450000
                    PWMvalueR = 450000
                if keyboard.is_pressed('k'):
                    PWMvalueL = 450000
                    PWMvalueR = 450000
                    #Set both wheels to go Backwards
                    GPIO.output(in1, GPIO.HIGH)
                    GPIO.output(in2, GPIO.LOW)
                    GPIO.output(in3, GPIO.LOW)
                    GPIO.output(in4, GPIO.HIGH)
            if keyboard.is_pressed('r'):
                K = 0
                I = 0
                D = 0
                time_previous - 0
                previous_error = 0
                print('Values Reset')
        else:                
            PID = PIDcontrol(error, Kp, Ki, Kd)
            PWMvalueL, PWMvalueR = controlMot(PID, speed)
            
##            if time.time()-move_time < 0.1: #if car is starting, use higher voltage than usual to overcome starting torque
##                PWMvalueL = 500000
##                PWMvalueR = 500000
                
            output_list.append([error, PID, PWMvalueL, PWMvalueR, time.time()-init_time, img2]) #Store Network output
            
            print('PID value:', PID)
            print('Left Mot:', PWMvalueL, 'Right Mot:', PWMvalueR)
            
        pi.hardware_PWM(18, 2000, PWMvalueR) # 2kHz varying% dutycycle
        pi.hardware_PWM(13, 2000, PWMvalueL) # 2kHz varying% dutycycle
        
        cv2.waitKey(1)
        if keyboard.is_pressed('q'):
            break
        
        difftime= time.time()-currenttime
        #print('Loop Time Taken (s):', difftime)
        fps_list.append(difftime)
        if len(fps_list) == 50:
            fps = 50/sum(fps_list)
            if fps > max_fps :
                max_fps = fps
            if fps < min_fps:
                min_fps = fps
            fps_list = []
        #print('FPS:', fps, 'fps')
            
    print('Max FPS:', max_fps, ' Min FPS:', min_fps)
    print('Final PID Constants:')
    print('K:', Kp, 'I:',Ki, 'D:', Kd)
    #Turn off GPIO pins
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    pi.hardware_PWM(18, 2000, 0) # 2kHz 0% dutycycle
    pi.hardware_PWM(13, 2000, 0) # 2kHz 0% dutycycle
        
    camera.close()
    filename = "_"+str(Kp)+"_"+str(Ki)+"_"+str(Kd)+"MLPchicane_Run_on_"+getTime()
    np.save(filename, np.array(output_list, dtype = object)) #Save Ouput
    if os.path.isfile(filename+".npy"):
        print('Output Saved')
    else:
        print('Output Not Saved')
    cv2.destroyAllWindows()

train_neural_network(data = x,layers = 1, neurons_per_layer = [21], no_outputs = 3)


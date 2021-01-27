import smbus
import time
from evdev import InputDevice, categorize, ecodes
import RPi.GPIO as GPIO
import cv2
import multiprocessing
import numpy as np

bus = 1 #Ist bei meinem Pi immer so
adresse = 0x40 #über "iscdetect -y 1" herausgefunden
_PRESCALE = 0xFE #Aus PCA9685 Datasheet (Register für prescaler)
_MODE1 = 0 #Mode1 register

cameraServoPortV = 2
cameraServoPortH = 1
steeringServoPort = 0

#Set maximum turning angle (in terms of PWM range 0-4096 from PCA9685)
servoMax = 485
servoMin = 295

#Create connection to the gamepad (topleft USB plug)
gamepad = InputDevice('/dev/input/event0')

#Set output Pins to control DC motor driving direction for bith motors
GPIO.setmode(GPIO.BCM)
directionChannelLR = 17 #LeftRear Pin
directionChannelRR = 27 #RightRear Pin

GPIO.setup(directionChannelLR, GPIO.OUT)
GPIO.setup(directionChannelRR, GPIO.OUT)

#Boolean for image thread
#This creates a boolean wrapped in a variable that can be used by multiple processes
cameraProcessRunning = multiprocessing.Value('b', False)
redTrackingRunning = multiprocessing.Value('b', False)
movementTrackingRunning = multiprocessing.Value('b', False)
drivingBackwards = multiprocessing.Value('b', False)


#Momentanen Prescale auslesen
myBus = smbus.SMBus(bus)
currentPrescale = myBus.read_byte_data(adresse, _PRESCALE)
if currentPrescale != 101:
    print("Momentan ist prescale nicht 101, sondern " + str(currentPrescale))
    #Der Wert der bei PreScaler angegeben wird bestimmt die Frequenz, mit der der Serve gesteuert wird
    #PreScaler = (25000000 / (4096 - Frequenz)) - 1(Datasheet Seite 25)
    #Dabei ist 25000000 die Clockspeed, mit der der PCA9685 per default arbeitet. 4096 ist die Anzahl der möglichen
    #Analigwerte, die man mit 12 bit einstellen kann.
    #Der Default PreScaler ist 30. Dadurch ergibt sich eine Frequenz von ca 200Hz. Der Prescaler soll nun so gewählt
    #werden, dass sich eine Frequenz von 60 Hz einstellt --> 25000000 / (4096 * 60 ) - 1 = 101 = 0x65
    myBus.write_byte_data(adresse, _PRESCALE, 101)#0x65)
    print("Prescale wurde nun gesetzt auf " + str(myBus.read_byte_data(adresse, _PRESCALE)))
else:
    print("Prescaler ist bereits bei 101")
    
def convert(value, inputMin = -32768, inputMax = 32767, outputMin = servoMin, outputMax = servoMax, inverse = False):
    #Function to convert between controller input and servo output
    if value > inputMax:
        value = inputMax
    if value < inputMin:
        value = inputMin
        
    if inverse:
        return outputMax - ((value - inputMin) / (inputMax - inputMin) ) * (outputMax - outputMin)
    else:
        return ((value - inputMin) / (inputMax - inputMin) ) * (outputMax - outputMin) + outputMin


def write_Servoauslenkung(adresse, servo, off):
#     verschiedenen bytes des ausgewählten Servo einstellen
#     on Zeitpunkt wird immer als 0 angenommen. Off wird adjustiert
#     270 wurde als minimum ausgewählt, weil es eine mechanische grenze gibt
    off = int(secure(off)) # in case it is not an integer. Also ensures if the servo can handle the given value
        
    #Servos have a maximum current they can sustain
    if off > servoMax or off < servoMin:
        print("Off-Time must be between min (" + str(seroMin) + ") and max (" + str(servoMax) + ")")
        return
        
#     es sind nur die Servos 0 - 2 angeschlossen und 3+4 sind die DC Motoren
    if servo not in [0, 1, 2]:
        print("Only ports 0, 1, and 2 are connected to servos")
        return
    
    myBus.write_byte_data(adresse, 6 + 4*servo, 0 & 255)
    myBus.write_byte_data(adresse, 7 + 4*servo, 0 >> 8)
    myBus.write_byte_data(adresse, 8 + 4*servo, off & 255)
    myBus.write_byte_data(adresse, 9 + 4*servo, off >> 8)
    
def secure(servoValue, servoMin = servoMin, servoMax = servoMax):
    if servoValue > servoMax:
        servoValue = servoMax
    elif servoValue < servoMin:
        servoValue = servoMin
    return servoValue
    
def writeDCMotorSpeed(adresse, speed, backwards = False):
    
    #Set driving direction
        #output accepts a varargs of pin numbers
    GPIO.output((directionChannelLR, directionChannelRR), backwards)
    
    #Set speed
    speed = int(speed) # in case it is not an integer
    
    if speed > 4096 or speed < 0:
        print("PWM can only be set to values from 0 to 4096")
        return
    
    for pwmPin in [4, 5]:
        myBus.write_byte_data(adresse, 6 + 4*pwmPin, 0 & 255)
        myBus.write_byte_data(adresse, 7 + 4*pwmPin, 0 >> 8)
        myBus.write_byte_data(adresse, 8 + 4*pwmPin, speed & 255)
        myBus.write_byte_data(adresse, 9 + 4*pwmPin, speed >> 8)
    
def wakeup(adresse):
    oldMode = myBus.read_byte_data(adresse, _MODE1)
    newMode = oldMode & 111 #set sleep to 0, rest stays as it is
    myBus.write_byte_data(adresse, _MODE1, newMode)
    
    
def showVideo(isActiveThread):
    cap = cv2.VideoCapture(0)
    
    while True:
        result, img = cap.read()
        
        #imshow() only works for the first invokation of this function.
        #If I do it again, it doesnt work anymore. This is unsolved
        cv2.imshow("Video", img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if not isActiveThread.value:
            time.sleep(1)
            break

    cap.release()
    cv2.destroyAllWindows()
    return

def trackRed(isActiveThread, backwardFlag, cameraServoValueH = (servoMin + servoMax) / 2,
             cameraServoValueV = (servoMin + servoMax) / 2, steeringServoValueH = (servoMin + servoMax) / 2, maxIncrease = 10):
    cap = cv2.VideoCapture(0)
    
    while True:
        result, img = cap.read()
        #maxBrightH, maxBrightV, differenceRedGrey = diffRedGrey(img)
        maxBrightH, maxBrightV, filterdByHSV = filterByHSV(img, invertFilter = True)
        #cv2.imshow("differenceRedGrey", differenceRedGrey)
        cv2.imshow("filterdByHSV", filterdByHSV)
        #Nicht die absolute auslenkung, sondern nur der increment der auslenkung sollte durch die farbe bestimmt werden
        cameraServoValueH += int(convert(maxBrightH,
                                   inputMin = 0, inputMax = 640,
                                   outputMin = -maxIncrease, outputMax = maxIncrease,
                                   inverse = True))
        steeringServoValueH += int(convert(maxBrightH,
                                   inputMin = 0, inputMax = 640,
                                   outputMin = -maxIncrease, outputMax = maxIncrease,
                                   inverse = backwardFlag.value))
        cameraServoValueV += int(convert(maxBrightV,
                                   inputMin = 0, inputMax = 480,
                                   outputMin = -maxIncrease,
                                   outputMax = maxIncrease,
                                   inverse = True))
        #Update servo positions
        write_Servoauslenkung(adresse, steeringServoPort, steeringServoValueH)
        write_Servoauslenkung(adresse, cameraServoPortH, cameraServoValueH)
        write_Servoauslenkung(adresse, cameraServoPortV, cameraServoValueV)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        if not isActiveThread.value:
            time.sleep(1)
            break

    cap.release()
    cv2.destroyAllWindows()
    return

def findMaxBright(greyScaleImage, axis = 0):
    return np.argmax(np.mean(greyScaleImage, axis = axis))

def filterByHSV(img, lowerBound = (12, 0, 0), upperBound = (168, 126, 90), invertFilter = False, drawBrightCenterLines = True):
    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #I am considering red when hue is <=8 oder >=175 (hue scale: 0-179)
    
    #First filter only colors that are not red
    passOn = 0 if invertFilter else 255 
    inverted = inRangeT(hsvImg, lowerBound, upperBound)
    
    #Calculated center of red color
        #This is only done to calculate maxBrightH/V. It will not be shown on the output image
    greyScale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    outstandingBright = np.where(inverted == passOn,
                             np.where(img[:, :, 2] > greyScale,
                                      img[:, :, 2] - greyScale,
                                      0),
                             0)
    
        #Find where the maximum brightness is
    maxBrightH = int(findMaxBright(outstandingBright))
    maxBrightV = int(findMaxBright(outstandingBright, axis = 1))
    
    #Calculate output image
    mask = cv2.cvtColor(inverted, cv2.COLOR_GRAY2BGR) #Converted, so it fits the dimensions of img
    allChannelGrey = cv2.cvtColor(greyScale, cv2.COLOR_GRAY2BGR) #Converted, so it fits the dimensions of img
    outputImg = np.where(mask == passOn, img, allChannelGrey)
    
    if drawBrightCenterLines:
        #Show taken image
        cv2.line(img = outputImg, pt1 = (maxBrightH, 0), pt2 = (maxBrightH, 480), color = 255, thickness = 1)
        cv2.line(img = outputImg, pt1 = (0, maxBrightV), pt2 = (640, maxBrightV), color = 255, thickness = 1)
    
    return maxBrightH, maxBrightV, outputImg

def inRangeT(hsvImg, lowerBound, upperBound):
    filterOne = cv2.inRange(hsvImg[:, :, 0], lowerBound[0], upperBound[0])
    filterTwo = cv2.inRange(hsvImg[:, :, 1], lowerBound[1], upperBound[1])
    filterThree = cv2.inRange(hsvImg[:, :, 2], lowerBound[2], upperBound[2])
    return np.where(filterOne == 255, 255, np.where(filterTwo == 255, 255, np.where(filterThree == 255, 255, 0))).astype(np.uint8)


wakeup(adresse)   

#Now handle the controller events
for event in gamepad.read_loop():
    if event.code != 0 or event.type != 0 or event.value != 0: #There are some signals seemingly without content
        if not (redTrackingRunning.value or movementTrackingRunning.value): 
            if event.type == 3 and event.code == 4: #vertical camera servo
                write_Servoauslenkung(adresse, cameraServoPortV, convert(event.value))
            elif event.type == 3 and event.code == 3: #horuzontal camera servo
                write_Servoauslenkung(adresse, cameraServoPortH, convert(event.value, inverse = True))
            elif event.type == 3 and event.code == 0: #horuzontal steering servo
                write_Servoauslenkung(adresse, steeringServoPort, convert(event.value))
        if event.type == 3 and event.code == 5: #drive foreward
            #outputMax can theoretically be 4096, I just did not try it
            writeDCMotorSpeed(adresse,
                              convert(event.value, inputMin = 0, inputMax = 1023, outputMin = 0, outputMax = 3500))
            drivingBackwards.value = False
        elif event.type == 3 and event.code == 2: #drive backward
            #outputMax can theoretically be 4096, I just diid not try it
            writeDCMotorSpeed(adresse,
                              convert(event.value, inputMin = 0, inputMax = 1023, outputMin = 0, outputMax = 3500),
                              backwards = True)
            drivingBackwards.value = True
        elif event.type == 1 and event.code == 308 and event.value == 1: #Y-Button shows video from the camera
            if not cameraProcessRunning.value:
                cameraProcessRunning.value = True
                cameraProcess = multiprocessing.Process(target = showVideo, args = (cameraProcessRunning, ))
                cameraProcess.start()
            else:
                cameraProcessRunning.value = False
                cameraProcess.join()
        elif event.type == 1 and event.code == 311 and event.value == 1: #Rechte Schultertaste
            #implement trace red color here
            if not redTrackingRunning.value:
                redTrackingRunning.value = True
                redTrackingProcess = multiprocessing.Process(target = trackRed, args = [redTrackingRunning, drivingBackwards])
                redTrackingProcess.start()
            else:
                redTrackingRunning.value = False
                redTrackingProcess.join()
                
        elif event.type == 1 and event.code == 310 and event.value == 1: #Linke Schultertaste
            #implement trace movement here
            pass
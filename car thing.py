import cv2
import mediapipe as mp
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
from gpiozero import AngularServo 
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)

# Right Motor
in1 = 16
in2 = 20
en_a = 21
# Left Motor
in3 = 5
in4 = 6
en_b = 13


GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en_a,GPIO.OUT)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en_b,GPIO.OUT)

q=GPIO.PWM(en_a,100)
p=GPIO.PWM(en_b,100)
p.start(25)
q.start(25)

GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
factory=PiGPIOFactory()
joint_base=AngularServo(18,min_angle=-180,max_angle=180, pin_factory=factory)
angle_base=0
angle=0



cap = cv2.VideoCapture(0)  
xp=0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    # Check if both hands are detected
    if results.multi_hand_landmarks and len(results.multi_hand_landmarks) == 2:
        landmarks_0 = results.multi_hand_landmarks[0].landmark
        landmarks_1 = results.multi_hand_landmarks[1].landmark
        #print(landmarks_0[0].x,'         ',landmarks_1[0].x)
        if landmarks_0[0].x < landmarks_1[0].x:
            left=landmarks_0
            right=landmarks_1
        elif landmarks_0[0].x > landmarks_1[0].x:
            left=landmarks_1
            right=landmarks_0
            
        
        if(left[8].y < left[6].y and right[8].y < right[6].y and not(left[5].y > left[9].y and right[5].y > right[9].y)):
            cv2.putText(frame,"REVERSE -_-",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)

            GPIO.output(in4,GPIO.LOW)
            GPIO.output(in3,GPIO.HIGH)
            
        elif (left[9].y < left[12].y and right[9].y < right[12].y):
            cv2.putText(frame,"STOP",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in4,GPIO.LOW)
            GPIO.output(in3,GPIO.LOW)
            
        elif(left[9].y < left[12].y and right[5].y > right[9].y):
            cv2.putText(frame,"LEFT",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)

            GPIO.output(in4,GPIO.LOW)
            GPIO.output(in3,GPIO.LOW)
        elif(left[5].y > left[9].y and right[9].y < right[12].y):
            cv2.putText(frame,"RIGHT",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)

            GPIO.output(in4,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
        elif(left[5].y > left[9].y and right[5].y > right[9].y):
            cv2.putText(frame,"FORWARD",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)

            GPIO.output(in4,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
        xp=(left[0].x+right[0].x)/2-0.5
        print(xp)
        
        if xp<=-0.15 or xp>=0.15:angle+=-xp*40
        if angle>180:angle=180
        if angle<-180:angle=-180 
           
        angle_base=angle
        joint_base.angle=angle_base
    # Draw landmarks on the frame
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            for landmark in hand_landmarks.landmark:
                h, w, _ = frame.shape
                x, y = int(landmark.x * w), int(landmark.y * h)
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

    # Display the frame
    cv2.imshow('Hand Gestures', frame)
    
   

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

import cv2
import mediapipe as mp
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()


cap = cv2.VideoCapture(0)  

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
        
        if(landmarks_0[8].y < landmarks_0[6].y and landmarks_1[8].y < landmarks_1[6].y and not(landmarks_0[5].y > landmarks_0[9].y and landmarks_1[5].y > landmarks_1[9].y)):
            cv2.putText(frame,"REVERSE -_-",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
        elif (landmarks_0[9].y < landmarks_0[12].y and landmarks_1[9].y < landmarks_1[12].y):
            cv2.putText(frame,"STOP",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
        elif(landmarks_0[9].y < landmarks_0[12].y and landmarks_1[5].y > landmarks_1[9].y):
            cv2.putText(frame,"LEFT",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
        elif(landmarks_0[5].y > landmarks_0[9].y and landmarks_1[9].y < landmarks_1[12].y):
             cv2.putText(frame,"RIGHT",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
        elif(landmarks_0[5].y > landmarks_0[9].y and landmarks_1[5].y > landmarks_1[9].y):
            cv2.putText(frame,"FORWARD",(50, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2, cv2.LINE_AA,)
        

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

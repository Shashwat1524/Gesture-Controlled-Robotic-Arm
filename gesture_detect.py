import cv2
import mediapipe as mp
import numpy as np
import math
import serial
import time

class HandGestureRecognizer:
    def __init__(self, serial_port=None):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.finger_angles_thresholds = {
            "open": 120,  
            "closed": 60   
        }
        
        self.gestures = {
            "fist": [0, 0, 0, 0, 0],  
            "open_hand": [1, 1, 1, 1, 1], 
            "thumbs_up": [1, 0, 0, 0, 0],  
            "peace": [0, 1, 1, 0, 0],  
            "point": [0, 1, 0, 0, 0],  
            "rock": [1, 1, 0, 0, 1],  
            "ok": [1, 0, 0, 0, 1]  
        }
        
        self.gesture_codes = {
            "fist": 1,
            "open_hand": 2,
            "thumbs_up": 3,
            "peace": 4,
            "rock": 5,
            "point": 6,
            "ok": 7
        }
        
        self.serial_port = serial_port
        self.last_sent_gesture = None
        self.last_sent_time = 0
        self.send_interval = 0.3  

    def detect_gestures(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            detected_gestures = []
            
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())
                
                finger_states = self.finger_states(hand_landmarks)
                gesture = self.identify_gesture(finger_states)
                detected_gestures.append(gesture)
                
                if gesture != "unknown": 
                    gesture_code = self.gesture_codes.get(gesture)
                    self.send_to_stm32(gesture, gesture_code)
                    cv2.putText(frame, f"Gesture: {gesture}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, f"Gesture: unknown", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            return frame, detected_gestures
        else:
            cv2.putText(frame, "No hands detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return frame, []

    def finger_states(self, hand_landmarks):
        landmarks = []
        for landmark in hand_landmarks.landmark:
            landmarks.append((landmark.x, landmark.y, landmark.z))
            
        finger_states = []
        thumb_angle = self.calculate_angle(landmarks[0], landmarks[2], landmarks[4])
        finger_states.append(1 if thumb_angle > self.finger_angles_thresholds["open"] else 0)
        index_finger_extended = landmarks[8][1] < landmarks[6][1]
        finger_states.append(1 if index_finger_extended else 0)
        middle_finger_extended = landmarks[12][1] < landmarks[10][1]
        finger_states.append(1 if middle_finger_extended else 0)
        ring_finger_extended = landmarks[16][1] < landmarks[14][1]
        finger_states.append(1 if ring_finger_extended else 0)
        pinky_finger_extended = landmarks[20][1] < landmarks[18][1]
        finger_states.append(1 if pinky_finger_extended else 0)
        
        return finger_states

    def calculate_angle(self, a, b, c):
        ba = np.array([a[0] - b[0], a[1] - b[1]])
        bc = np.array([c[0] - b[0], c[1] - b[1]])
        ba_normalized = ba / np.linalg.norm(ba)
        bc_normalized = bc / np.linalg.norm(bc)
        dot_product = np.dot(ba_normalized, bc_normalized)
        dot_product = max(min(dot_product, 1.0), -1.0)
        angle = math.degrees(math.acos(dot_product))
        
        return angle

    def identify_gesture(self, finger_states):
        for gesture_name, gesture_states in self.gestures.items():
            if finger_states == gesture_states:
                return gesture_name
        return "unknown"
    
    def send_to_stm32(self, gesture, code):
        if self.serial_port is not None and code is not None:  
            current_time = time.time()
            if self.last_sent_gesture is None or gesture != self.last_sent_gesture or (current_time - self.last_sent_time) > self.send_interval:
                try:
                    self.serial_port.write(str(code).encode())  # <- changed line
                    self.last_sent_gesture = gesture
                    self.last_sent_time = current_time
                except:
                    pass


def main():
    try:
        ser = serial.Serial(
            port='/dev/tty.usbmodem1403',  
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        time.sleep(2)  # << CRUCIAL STEP
    except:
        ser = None
    
    cap = cv2.VideoCapture(0)
    recognizer = HandGestureRecognizer(serial_port=ser)
    
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break
        
        frame = cv2.flip(frame, 1)
        frame, _ = recognizer.detect_gestures(frame)
        cv2.imshow("Hand Gesture Recognition for STM32", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    if ser is not None:
        ser.close()


if __name__ == "__main__":
    main()

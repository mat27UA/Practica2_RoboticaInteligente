#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# Declarar el detector de pose de mediapipe a utilizar
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Publicador de mensajes de control
ackermann_command_publisher = None

def gesture_classify(hand_landmarks):
    thumb_opened = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x < hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].x
    index_opened = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x < hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].x
    middle_opened = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x < hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].x
    ring_opened = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x < hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].x
    pinky_opened = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].x < hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].x

    if index_opened and not middle_opened and not ring_opened and not pinky_opened and thumb_opened:
        return 1 #"Girar Derecha"
    elif index_opened and middle_opened and not ring_opened and not pinky_opened and thumb_opened:
        return 2 #"Girar Izquieda"
    elif thumb_opened and index_opened and middle_opened and ring_opened and pinky_opened:
        return 3 #"Recto"
    else:
        return 0 #"Parar"


#Procesar la imagen del operador
def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        
    #Procesar la imagen con MediaPipe
    cv_image_processed = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = hands.process(cv_image_processed)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Dibujar los landmarks sobre la imagen
            mp_drawing.draw_landmarks(
                cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
            # Reconocer el gesto mediante alguna clasificación a partir de los landmarks
            gesture = gesture_classify(hand_landmarks)
            rospy.loginfo(f"Detected gesture: {gesture}")

            # Interpretar el gesto obtenido y enviar la orden de control ackermann
            ackermann_msg = ackermann_msgs.msg.AckermannDrive()
            
            if gesture == 1:
                ackermann_msg.speed = 2.0  # Avanzar 
                ackermann_msg.steering_angle = -1.0  # Girar a la derecha
            elif gesture == 2:
                ackermann_msg.speed = 2.0  # Mover Recto
                ackermann_msg.steering_angle = 1.0  # Girar a la izquierda
            elif gesture == 3:
                ackermann_msg.speed = 2.0  # Avanzar
                ackermann_msg.steering_angle = 0.0  # Recto
            else:
                ackermann_msg.speed = 0.0  # Detener
                ackermann_msg.steering_angle = 0.0  # Recto

            ackermann_command_publisher.publish(ackermann_msg)

    # Mostrar la imagen con los landmarks/gestos detectados
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)

def main():
    global ackermann_command_publisher
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

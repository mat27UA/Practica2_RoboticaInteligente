#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('obstacle_avoidance')

        self.obstacle_detected = False

        # Suscriptor para la nube de puntos de los obstáculos captados por el lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Suscriptor para comandos de control Ackermann
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        #Publicador para comandos Ackermann modificados
        self.ackermann_command_publisher = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)
        
        # Almacenar el último mensaje Ackermann recibido
        self.last_ackermann_cmd = AckermannDrive()

    def obstacle_callback(self, msg):
        # Procesar la nube de puntos con los obstáculos teniendo en cuenta el último mensaje de movimiento recibido para evitar colisiones
        points = list(pc2.read_points(msg, skip_nans=True))

        for point in points:
            x, y, z = point[:3]
            if -1.5 < y < 1.5 and -1.5 < x < 1.5:
                self.obstacle_detected = True
        
        cmd = self.last_ackermann_cmd
        if self.obstacle_detected:
            rospy.loginfo("Prar el robot - Obtáculo detectado.")
            cmd.speed = 0.0 
            cmd.steering_angle = 0.0

        # Modificar mensaje de ackermann si es necesario
        cmd = self.last_ackermann_cmd
        self.ackermann_command_publisher.publish(cmd)
        
        return

    def ackermann_callback(self, msg):
        # Almacena el último comando recibido
        self.last_ackermann_cmd = msg

    def modify_ackermann_command(self):
        # Modifica el comando Ackermann para evitar obstáculos (Se puede modificar si es necesario)
        cmd = self.last_ackermann_cmd
        cmd.drive.speed = 0.0  # Reduce la velocidad
        cmd.drive.steering_angle = 0.0 # Cambia el ángulo de dirección
        

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()

#!/usr/bin/env python3
import rospy
from rdl_robot_msgs.msg import MarkingPoint, PathMarking

import numpy as np


class PathGeneratorNode:
    def __init__(self):
        rospy.init_node('path_generator_node')
        self.points_pub = rospy.Publisher("/interest_points", PathMarking, queue_size=10)  # SE PUEDE AÑADIR UN PUBLICADOR PARA VISUALIZAR LAS INFERENCIAS DEL MODELO
        
        self.points_of_interest = []

    def publicar_path(self):

        # Lista de puntos en 2D
        pointsOpt = [(1.0, 0.0, 0.0), (1.2, 0.0, 0.0), (1.4, 0.0, 0.0), ((1.6, 0.0, 0.0)), (1.8, 0.0, 0.0), (2.0, 0.0, 0.0)]
        paintOpt = [True, True, True, True, True, False]
        if pointsOpt: # Verificar que la lista no este vacía
            pathMarks_msg = PathMarking()
            pathMarks_msg.header.stamp = rospy.Time.now()
            pathMarks_msg.header.frame_id = "map"  # Cambia el marco según sea necesario

            for i, point in enumerate(pointsOpt):
                PointMark = MarkingPoint()
                PointMark.header = pathMarks_msg.header
                PointMark.pose.position.x = point[0]
                PointMark.pose.position.y = point[1]
                PointMark.pose.position.z = 0.0  # Si es necesario
                PointMark.pose.orientation.w = 1.0 # np.cos(point[3]/2)  # La orientacion theta en el eje z
                PointMark.pose.orientation.z = point[2] #np.sin(point[3]/2)
                PointMark.paint = paintOpt[i]

                pathMarks_msg.points.append(PointMark)

            self.points_pub.publish(pathMarks_msg) # Publicar los puntos de interés

if __name__ == '__main__':
    try:
        node = PathGeneratorNode()
        node.publicar_path()
        rospy.sleep(3)
        node.publicar_path()
        rospy.sleep(3)
        node.publicar_path()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

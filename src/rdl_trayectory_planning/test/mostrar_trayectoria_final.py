#!/usr/bin/env python3

import rospy
from rdl_robot_msgs.msg import PathMarking, MarkingPoint
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

class PosePublisherNode:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('pose_publisher_node')

        # Publicar en el tópico "/poses"
        self.pose_pub = rospy.Publisher('/poses', PoseArray, queue_size=10)

        # Subscribirse al tópico que publica el mensaje PathMarking
        self.pose_sub = rospy.Subscriber('/final_trayectory', PathMarking, self.path_marking_callback)

    def path_marking_callback(self, msg):
        # Crear un objeto PoseArray para publicar
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "odom"  # Especifica el marco de referencia (puede ser 'map' o 'odom')

        # Extraer las poses de los puntos de marcado y añadirlas al PoseArray
        for marking_point in msg.points:
            pose_array.poses.append(marking_point.pose)  # marking_point.pose es de tipo Pose

        # Publicar el PoseArray en RViz
        self.pose_pub.publish(pose_array)

    def run(self):
        # Mantener el nodo en ejecución
        rospy.spin()

if __name__ == '__main__':
    try:
        pose_publisher = PosePublisherNode()
        pose_publisher.run()
    except rospy.ROSInterruptException:
        pass

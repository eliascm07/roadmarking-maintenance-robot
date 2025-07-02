#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

def imu_callback(msg):
    # Crear un buffer y un listener para tf
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    try:
        # Obtener la transformación de imu_link a base_footprint
        transform = buffer.lookup_transform('base_footprint', 'imu_link', rospy.Time(0))
        
        # Transformar el mensaje IMU manteniendo el timestamp original
        imu_in_base_footprint = tf2_geometry_msgs.do_transform_imu(msg, transform)
        
        # Asegurarse de que el timestamp original se conserve
        imu_in_base_footprint.header.stamp = msg.header.stamp
        
        # Publicar o procesar el IMU transformado
        rospy.loginfo("Mensaje IMU transformado y con timestamp conservado: %s", imu_in_base_footprint)
        imu_pub.publish(imu_in_base_footprint)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("Error al obtener la transformación: %s", e)

def listener():
    rospy.init_node('imu_transform_listener')

    # Publicador para el mensaje IMU transformado
    global imu_pub
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

    # Suscribirse al mensaje IMU original
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    
    rospy.spin()

if __name__ == "__main__":
    listener()

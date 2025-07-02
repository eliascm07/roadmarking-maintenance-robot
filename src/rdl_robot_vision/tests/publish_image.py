#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Ruta de la imagen local a publicar
IMAGE_PATH = "/home/eliascm07/catkin_ws/src/rdl_robot_vision/tests/images/image_0020.jpg"

def image_publisher():
    rospy.init_node('image_publisher_node', anonymous=True)
    image_pub = rospy.Publisher('/image_raw', Image, queue_size=10)

    # Inicializar el puente ROS-OpenCV
    bridge = CvBridge()

    # Cargar la imagen local
    if not os.path.exists(IMAGE_PATH):
        rospy.logerr(f"Imagen no encontrada: {IMAGE_PATH}")
        return

    cv_image = cv2.imread(IMAGE_PATH)
    if cv_image is None:
        rospy.logerr(f"No se pudo cargar la imagen desde {IMAGE_PATH}")
        return

    rate = rospy.Rate(10)  # Publicar cada 100 ms (10 Hz)

    while not rospy.is_shutdown():
        try:
            # Convertir la imagen OpenCV a un mensaje tipo Image de ROS
            ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # Publicar la imagen
            image_pub.publish(ros_image)
            rospy.loginfo("Imagen publicada.")

        except Exception as e:
            rospy.logerr(f"Error al publicar la imagen: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Crear la carpeta donde se guardarán las imágenes
OUTPUT_FOLDER = "/home/eliascm07/catkin_ws/src/rdl_robot_vision/tests/images"
if not os.path.exists(OUTPUT_FOLDER):
    os.makedirs(OUTPUT_FOLDER)

# Inicializar el puente entre ROS y OpenCV
bridge = CvBridge()
image_count = 0

def image_callback(msg):
    global image_count
    try:
        # Convertir el mensaje ROS Image a una imagen OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Guardar la imagen en la carpeta de salida
        image_filename = os.path.join(OUTPUT_FOLDER, f"imageIPM_{image_count:04d}.jpg")
        cv2.imwrite(image_filename, cv_image)
        rospy.loginfo(f"Imagen guardada: {image_filename}")
        image_count += 1

    except Exception as e:
        rospy.logerr(f"Error al procesar la imagen: {e}")

def main():
    rospy.init_node('image_saver_node', anonymous=True)
    rospy.Subscriber("/image_IPM", Image, image_callback)
    rospy.loginfo("Nodo de guardado de imágenes iniciado.")
    rospy.spin()

if __name__ == '__main__':
    main()

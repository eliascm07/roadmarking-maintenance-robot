#!/usr/bin/env python3


# ------------------------------------------------- ACTUALIZAR, esto aun no esta bien
import rospy
from sensor_msgs.msg import CompressedImage

def image_callback(msg):
    # El tamaño de la imagen comprimida está en bytes
    image_size = len(msg.data)
    rospy.loginfo(f"Tamaño de la imagen comprimida: {image_size} bytes")

def image_size_node():
    # Inicializar el nodo
    rospy.init_node('image_size_node', anonymous=True)

    # Suscribirse al tópico de imágenes comprimidas
    rospy.Subscriber('/camera/image/compressed', CompressedImage, image_callback)

    # Mantener el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        image_size_node()
    except rospy.ROSInterruptException:
        pass
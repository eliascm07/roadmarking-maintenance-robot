#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf.transformations as tft

import configtest
from kine_functions import Trotx, Troty, Trotz, Trasl, T_scale, Trasl2D, conversion_vista_pajaro

pi = np.pi

###------------------- TRANSFORMACIONES FIJAS -------------------------
# {C} = camera frame, {B}= base_footprint frame, {I}= imu_frame, {vp}= imagen en vista de pajaro (pixeles)
# {W} = world frame o coordenadas iniciales del map, {vi}= imagen original (pixeles), {fi}= imagen final recortada de la original (pixeles)
# Matriz de parámetros intrínsecos
K = np.array([[configtest.fx, 0, configtest.cx], [0, configtest.fy, configtest.cy], [0, 0, 1]])
K_prime = Trasl2D(-configtest.deltaX, -configtest.deltaY) @ K # Considerando la imagen final recortada
# La base del movil esta situado en el z=0 global - REVISAR .......... Se puede hacer que primero haya una translación en la imagen original y no B a vp
T_corr = tft.euler_matrix(np.radians(0),np.radians(0),np.radians(0),'sxyz')
T_BC =Trasl(configtest.cameraX,configtest.cameraY,configtest.cameraHeight)@T_corr@Troty(pi/2)@Trotz(-pi/2) # Transformación que lleva de {B} a {C}  o {C} expresado en {B}
T_CB = np.linalg.inv(T_BC) # Transformación que lleva de {C} a {B}
T_Bvp = Trasl(configtest.trasX,configtest.trasY,0)@Trotx(pi)@Trotz(pi/2)@T_scale(configtest.scaleFactorX, configtest.scaleFactorY, configtest.scaleFactorZ)
T_Cvp =T_CB@T_Bvp # Transformación de {vp} expresado en {C}

# Calcular la matriz de homografía que va de {fi} a {vp} = H_fivp y de {vp} a {vi} = H_vpfi
H_fivp = K_prime @ np.hstack((T_Cvp[:3, :2], T_Cvp[0:3, 3].reshape(-1, 1)))
H_vpfi = np.linalg.inv(H_fivp)
# print(H_vpfi)
H_Bvp = np.hstack((T_Bvp[0:3,0:2], T_Bvp[0:3,3].reshape(-1,1)))

#  Puntos para filtrar resultado de segmentación (sección media inferior en vp a imagen final recortada)
pUl_fi = H_fivp @ np.array([[0], [0], [1]])
pUl_fi = pUl_fi[0:2,:]/pUl_fi[2,0]
pUr_fi = H_fivp @ np.array([[configtest.width_vp], [0], [1]])
pUr_fi = pUr_fi[0:2,:]/pUr_fi[2,0]
pDl_fi = H_fivp @ np.array([[0], [configtest.height_vp], [1]])
pDl_fi = pDl_fi[0:2,:]/pDl_fi[2,0]
pDr_fi = H_fivp @ np.array([[configtest.width_vp], [configtest.height_vp], [1]])
pDr_fi = pDr_fi[0:2,:]/pDr_fi[2,0]

# Llevar un punto distancia del plano XY a la vista de pájaro
H_vpB = np.linalg.inv(H_Bvp)
distancia= H_vpB @ np.array([[0.2], [0], [0]]) # Cambiar por el umbral en metros
print("Distancia es: ", distancia)
#distancia = distancia[0:2,:]/distancia[2,0] # Verificar si se necesita, pienso que no porque antes ya lo pense y analice y puse con mayusculas que no necesita de normalización en el sentido inverso
#print("Distancia es: ", distancia)


class ImageProcessorNode:
    def __init__(self):
        rospy.init_node('image_processor_IPM')

        # Crear un objeto CvBridge para convertir entre imágenes ROS y OpenCV
        self.bridge = CvBridge()

        # Suscribirse al tópico de la cámara
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        # Publicar la imagen procesada en otro tópico
        self.image_pub = rospy.Publisher('/image_IPM', Image, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convertir la imagen ROS a una imagen OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Recortar la zona de interés para la predicción
            finalImage = frame[int(configtest.deltaY):, :]
            image_vp = conversion_vista_pajaro(finalImage, H_vpfi, configtest.width_vp, configtest.height_vp)

            # Dibujar un círculo de radio 5 con color rojo
            cv2.circle(image_vp, (0, 22), 5, (0, 0, 255), -1)  # El color es rojo y el radio es 5

            # Convertir la imagen procesada de nuevo a formato ROS
            processed_image_msg = self.bridge.cv2_to_imgmsg(image_vp, encoding='bgr8')

            # Publicar la imagen procesada
            self.image_pub.publish(processed_image_msg)

        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen: {e}")

if __name__ == '__main__':
    try:
        node = ImageProcessorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

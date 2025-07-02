#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rdl_robot_msgs.msg import InterestMarkingPoint, InterestPointArray
from tf.transformations import euler_from_quaternion

# Importar librerías
import cv2
import numpy as np
import ultralytics
# ultralytics.checks()
from ultralytics import YOLO

import config
from funciones import filtrar_instancias, conversion_vista_pajaro, division_subareas
from kine_functions import Trotx, Troty, Trotz, Trasl, T_scale, Trasl2D
from cv_bridge import CvBridge, CvBridgeError
from shapely.geometry import Polygon
import time
import gc
import json

pi = np.pi

# Carga modelo preentrenado
model = YOLO('/home/eliascm07/catkin_ws/src/rdl_robot_vision/scripts/weights/best2.pt') # , task='segment')

# ------------------- VARIABLES GENERALES ------------------------
# Tamaño del recorte para la nueva imagen
finalWidth = int(config.frameWidth - config.deltaX)
finalHeight = int(config.frameHeight - config.deltaY)

###------------------- TRANSFORMACIONES FIJAS -------------------------
# {C} = camera frame, {B}= base_footprint frame, {I}= imu_frame, {vp}= imagen en vista de pajaro (pixeles)
# {W} = world frame o coordenadas iniciales del map, {vi}= imagen original (pixeles), {fi}= imagen final recortada de la original (pixeles)
# Matriz de parámetros intrínsecos
K = np.array([[config.fx, 0, config.cx], [0, config.fy, config.cy], [0, 0, 1]])
K_prime = Trasl2D(-config.deltaX, -config.deltaY) @ K # Considerando la imagen final recortada
# La base del movil esta situado en el z=0 global - REVISAR .......... Se puede hacer que primero haya una translación en la imagen original y no B a vp
T_BC = Trotz(np.radians(2))@Trasl(0,0,config.cameraHeight)@Troty(pi/2)@Trotz(-pi/2) # Transformación que lleva de {B} a {C}  o {C} expresado en {B}
T_CB = np.linalg.inv(T_BC) # Transformación que lleva de {C} a {B}
T_Bvp = Trasl(config.trasX,config.trasY,0)@Trotx(pi)@Trotz(pi/2)@T_scale(config.scaleFactorX, config.scaleFactorY, config.scaleFactorZ)
T_Cvp =T_CB@T_Bvp # Transformación de {vp} expresado en {C}

# Calcular la matriz de homografía que va de {fi} a {vp} = H_fivp y de {vp} a {vi} = H_vpfi
H_fivp = K_prime @ np.hstack((T_Cvp[:3, :2], T_Cvp[0:3, 3].reshape(-1, 1)))
H_vpfi = np.linalg.inv(H_fivp)
print(H_vpfi)
H_Bvp = np.hstack((T_Bvp[0:3,0:2], T_Bvp[0:3,3].reshape(-1,1)))

#  Puntos para filtrar resultado de segmentación (sección media inferior en vp a imagen final recortada)
pUl_fi = H_fivp @ np.array([[0], [0], [1]])
pUl_fi = pUl_fi[0:2,:]/pUl_fi[2,0]
pUr_fi = H_fivp @ np.array([[config.width_vp], [0], [1]])
pUr_fi = pUr_fi[0:2,:]/pUr_fi[2,0]
pDl_fi = H_fivp @ np.array([[0], [config.height_vp], [1]])
pDl_fi = pDl_fi[0:2,:]/pDl_fi[2,0]
pDr_fi = H_fivp @ np.array([[config.width_vp], [config.height_vp], [1]])
pDr_fi = pDr_fi[0:2,:]/pDr_fi[2,0]

# Llevar un punto distancia del plano XY a la vista de pájaro
H_vpB = np.linalg.inv(H_Bvp)
distancia= H_vpB @ np.array([[0], [0.15], [0]]) # Cambiar por el umbral en metros
# distancia = distancia[0:2,:]/distancia[2,0] # Verificar si se necesita, pienso que no porque antes ya lo pense y analice y puse con mayusculas que no necesita de normalización en el sentido inverso
print("Distancia es: ", distancia)


print(pUr_fi.T, pUl_fi.T, pDl_fi.T, pDr_fi.T)
# Define el área rectangular del filtrado  de segmentación usando los 4 puntos
area_filtrado = Polygon([pUr_fi, pUl_fi, pDl_fi, pDr_fi])


class ImageProcessorNode:
    def __init__(self):
        rospy.init_node('image_processor_node')
        # Define suscriptores
        #self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.image_callback, queue_size=0)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber("/ekf_pose", PoseWithCovarianceStamped, self.pose_callback)
        
        # REEMPLAZAR ESTO DE ARRIBA POR EL NUEVO TIPO DE MENSAJE
        # Define publicador
        self.points_pub = rospy.Publisher("/interest_points", InterestPointArray, queue_size=10)  # SE PUEDE AÑADIR UN PUBLICADOR PARA VISUALIZAR LAS INFERENCIAS DEL MODELO
        
        # Crear un publicador para el tópico
        self.image_pub = rospy.Publisher('/segmented_image', Image, queue_size=1)
        self.bridge = CvBridge()
        self.current_pose = None
        self.T_WB = Trasl(0,0,0) # Inicializar la transformadas de T_WB, como identidad  (Si es que no hay odometria 0 )
        self.points_of_interest = []
        self.clsInterestPoints = []

    def image_callback(self, data):

        start_time = time.time()
        try:
            # Convertir de sensor_msgs/Image a una imagen OpenCV
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Convertir el mensaje de ROS a imagen de OpenCV
            # np_arr = np.frombuffer(data.data, np.uint8)
            # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # rospy.loginfo("Frame shape is: " + str(frame.shape))

            # Procesamiento de la imagen y detección de puntos de interés
            points, cls, clss_names = self.process_image(frame) # Lista de puntos, ##############   FALTA PUBLICAR LA LISTA DE CLASES
            clss_names = json.dumps(clss_names)

            print(clss_names)

            if points: # Verificar que la lista no este vacía
                interestPoints_msg = InterestPointArray()
                rospy.loginfo("Interest points found are: " + str(points))
                interestPoints_msg.header.stamp = rospy.Time.now()
                interestPoints_msg.header.frame_id = "map"  # Cambia el marco según sea necesario
                interestPoints_msg.class_names = clss_names

                for i, point in enumerate(points):
                    interestPoint = InterestMarkingPoint()
                    interestPoint.header = interestPoints_msg.header
                    interestPoint.marking_class = int(cls[i])
                    interestPoint.pose.position.x = point[0]
                    interestPoint.pose.position.y = point[1]
                    interestPoint.pose.position.z = 0.0  # Si es necesario
                    interestPoint.pose.orientation.w = 1.0  # Orientación nula

                    interestPoints_msg.points.append(interestPoint)

                self.points_pub.publish(interestPoints_msg) # Publicar los puntos de interés
        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen: {e}")

        # Calcular el tiempo de procesamiento
        rospy.loginfo(f"Processing time: {time.time() - start_time:.4f} seconds")
        gc.collect()

    def pose_callback(self, data):
        # Almacena la posición del robot
        self.current_pose = data.pose.pose
        x_pos = self.current_pose.position.x
        y_pos = self.current_pose.position.y
        # Obtener la orientación en cuaterniones
        q = self.current_pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.T_WB = Trasl(x_pos,y_pos,0)*Trotz(theta) # Transformación del world a la base del robot
        rospy.loginfo(f"Position received: X: {x_pos}, Y: {y_pos}, Theta: {theta}")
    
    def process_image(self, frame):
        # Recortar la zona de interés para la predicción
        finalImage = frame[int(config.deltaY):, :]

        # Realizar predicciones con el modelo YOLO
        results = model.predict(finalImage, conf=0.5)
        masks = results[0].masks
        
        cls_names = results[0].names
        if masks is None: # Si no predijo nada, retorna lista vacía
            ros_image = self.bridge.cv2_to_imgmsg(finalImage, encoding="bgr8")
            # self.image_pub.publish(ros_image)
            return [], [], cls_names
        
        print(results)
        cls = results[0].boxes.cls

        # Extraer la imagen procesada con las máscaras de segmentación
        segmented_image = results[0].plot()  # Devuelve una imagen en formato numpy (OpenCV)
        # Convertir la imagen de OpenCV a sensor_msgs/Image
        ros_image = self.bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")
        # self.image_pub.publish(ros_image)
        
        # Filtrar instancias o polígonos en un área específica (de interés), que será convertida a vista de pájaro
        instancias_filtradas, _, cls_filt = filtrar_instancias(area_filtrado, masks, cls)
        image_vp, instancias_vp = conversion_vista_pajaro(finalImage, instancias_filtradas, H_vpfi, config.width_vp, config.height_vp)
    
        subareas, cls_sub = division_subareas(instancias_vp, cls_filt, config.alturaDivision, config.umbralAgrupacion)
        # Convertir la imagen_vp en escala de grises
        image_grayscale = cv2.cvtColor(image_vp, cv2.COLOR_RGB2GRAY)
        # Aplicar el umbral adaptativo en la imagen
        umbral_adaptativo = cv2.adaptiveThreshold(image_grayscale, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 17, -1) #PARÁMETROS A TESTEAR
        # Convertir la imagen de OpenCV a un mensaje ROS
        image_msg = self.bridge.cv2_to_imgmsg(umbral_adaptativo, encoding="mono8")
        self.image_pub.publish(image_msg)
    
        # Evaluación de las subáreas
        for idx, subp in enumerate(subareas):
            coordenadas = np.array(subp.exterior.coords) # Conversión de POLYGON a np.array
            coordenadas = coordenadas.astype(int)
            
            # Crear una máscara binaria del mismo tamaño que la imagen
            mask = np.zeros_like(umbral_adaptativo, dtype=np.uint8)
            cv2.fillPoly(mask, [coordenadas], 255)  # Dibujar el polígono en la máscara, llena el polígono con color blanco (255)
            masked_image = cv2.bitwise_and(umbral_adaptativo, umbral_adaptativo, mask=mask) # Aplicar la máscara a la imagen original
            count_above_127 = np.sum(masked_image > 127)
            # Calcular el área del polígono (número de píxeles blancos en la máscara)
            area_polygon = np.sum(mask == 255)
            ratioPreservacion = count_above_127/area_polygon
            print(ratioPreservacion)

            # NOMINACIÓN 
            if ratioPreservacion < config.umbralPreservacion: # Si pasa es menor al umbral entonces se selecciona como punto de interes
                point = subareas[idx].centroid
                self.points_of_interest.append(point)
                self.clsInterestPoints.append(cls_sub[idx])
        if len(self.points_of_interest) == 0:
            return [], [], cls_names
        
        self.interestPointsArray= np.array([(point.x, point.y) for point in self.points_of_interest]) # Convertir a numpy array
        # Añade la tercera coordenada con 1s a los puntos 2D.
        self.interestPointsArray = np.hstack([self.interestPointsArray, np.ones((self.interestPointsArray.shape[0], 1))])
        # Aplicar matriz de homografía para pasar de {vp} a {B} frame
        self.interestPointsArray = self.interestPointsArray @ H_Bvp.T 
        # self.points_of_interest = self.points_of_interest / self.points_of_interest[:, -1][:, np.newaxis]  NO NECESITA ESTA NORMALIZACIÓN
        H_WB = np.hstack((self.T_WB[0:3,0:2], self.T_WB[0:3,3].reshape(-1,1)))
        # Transformación de {B} a {W}, solo en el plano XY
        self.interestPointsArray = self.interestPointsArray @ H_WB.T

        self.interestPointsArray = self.interestPointsArray[:,0:2]

        return self.interestPointsArray.tolist(), self.clsInterestPoints, cls_names

if __name__ == '__main__':
    try:
        node = ImageProcessorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

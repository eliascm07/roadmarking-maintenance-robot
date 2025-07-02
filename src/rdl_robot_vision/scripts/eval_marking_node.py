#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from rdl_robot_msgs.msg import InterestMarkingPoint, InterestPointArray
from rdl_robot_msgs.msg import ImageWithPose, ImageWithPoseArray

# Importar librerías
import cv2
import numpy as np
import ultralytics
# ultralytics.checks()
from ultralytics import YOLO

import config
from funciones import filtrar_instancias, conversion_vista_pajaro, division_subareas
from kine_functions import Trotx, Troty, Trotz, Trasl, T_scale, Trasl2D, pose_to_homogeneous_matrix
from cv_bridge import CvBridge, CvBridgeError
from shapely.geometry import Polygon
import time
import gc
import json
import math
import os

pi = np.pi

###
# Ruta absoluta donde quieres guardar las imágenes
output_folder_seg = "/home/eliascm07/img/imagenes_03/segmented"
output_folder_adap = "/home/eliascm07/img/imagenes_03/umbral"
output_folder_areasvp = "/home/eliascm07/img/imagenes_03/areasvp"
os.makedirs(output_folder_seg, exist_ok=True)  # Crea la carpeta si no existe
os.makedirs(output_folder_adap, exist_ok=True)  # Crea la carpeta si no existe
os.makedirs(output_folder_areasvp, exist_ok=True)  # Crea la carpeta si no existe


# Carga modelo preentrenado
model = YOLO('/home/eliascm07/catkin_ws/src/rdl_robot_vision/scripts/weights/best2.pt') # , task='segment') # Para Jetson Nano usar formato TensorRT

# ------------------- VARIABLES GENERALES ------------------------
# Tamaño del recorte para la nueva imagen
finalWidth = int(config.frameWidth - config.deltaX)
finalHeight = int(config.frameHeight - config.deltaY)

###------------------- TRANSFORMACIONES FIJAS -------------------------
# {C} = camera frame, {B}= base_footprint frame (ubicado en el punto del laser), {I}= imu_frame, {vp}= imagen en vista de pajaro (pixeles)
# {W} = world frame o coordenadas iniciales del map, {vi}= imagen original (pixeles), {fi}= imagen final recortada de la original (pixeles)
# {Cc} center camera frame (centro ubicado entre las dos cámaras o sin deplazamiento en y en el frame B o distancia del centro a la camara en y en B)
# Matriz de parámetros intrínsecos
K = np.array([[config.fx, 0, config.cx], [0, config.fy, config.cy], [0, 0, 1]])
K_prime = Trasl2D(-config.deltaX, -config.deltaY) @ K # Considerando la imagen final recortada
# La base del movil esta situado en el z=0 global - REVISAR .......... Se puede hacer que primero haya una translación en la imagen original y no B a vp
T_BCc = Trotz(np.radians(2))@Trasl(config.cameraX, 0,config.cameraHeight)@Troty(pi/2)@Trotz(-pi/2) # Transformación que lleva de {B} a {Cc}  o {Cc} expresado en {B}
# T_CcC = Trasl(-config.cameraY, 0, 0)#  el eje y de B es el eje -x de Cc
# T_BC = T_BCc@T_CcC # Transformación que lleva de {B} a {C}  o {C} expresado en {B}
T_BC = Trotz(np.radians(2))@Trasl(0,0,config.cameraHeight)@Troty(pi/2)@Trotz(-pi/2) # Transformación que lleva de {B} a {C}  o {C} expresado en {B}
T_CB = np.linalg.inv(T_BC) # Transformación que lleva de {C} a {B}
T_Bvp = Trasl(config.trasX,config.trasY,0)@Trotx(pi)@Trotz(pi/2)@T_scale(config.scaleFactorX, config.scaleFactorY, config.scaleFactorZ)
T_Cvp =T_CB@T_Bvp # Transformación de {vp} expresado en {C}
# T_Ccvp = T_CcC@T_Cvp # Transformación de {vp} expresado en {Cc}

# Calcular la matriz de homografía que va de {fi} a {vp} = H_fivp y de {vp} a {vi} = H_vpfi
H_fivp = K_prime @ np.hstack((T_Cvp[:3, :2], T_Cvp[0:3, 3].reshape(-1, 1)))
H_vpfi = np.linalg.inv(H_fivp)
print(H_vpfi)
H_Bvp = np.hstack((T_Bvp[0:3,0:2], T_Bvp[0:3,3].reshape(-1,1)))
# H_Ccvp = np.hstack((T_Ccvp[0:3,0:2], T_Ccvp[0:3,3].reshape(-1,1)))

#  Puntos para filtrar resultado de segmentación (sección media inferior en vp a imagen final recortada)
pUl_fi = H_fivp @ np.array([[0], [0], [1]])
pUl_fi = pUl_fi[0:2,:]/pUl_fi[2,0]
pUr_fi = H_fivp @ np.array([[config.width_vp], [0], [1]])
pUr_fi = pUr_fi[0:2,:]/pUr_fi[2,0]
pDl_fi = H_fivp @ np.array([[0], [config.height_vp], [1]])
pDl_fi = pDl_fi[0:2,:]/pDl_fi[2,0]
pDr_fi = H_fivp @ np.array([[config.width_vp], [config.height_vp], [1]])
pDr_fi = pDr_fi[0:2,:]/pDr_fi[2,0]


print(pUr_fi.T, pUl_fi.T, pDl_fi.T, pDr_fi.T)
# Define el área rectangular del filtrado  de segmentación usando los 4 puntos
area_filtrado = Polygon([pUr_fi, pUl_fi, pDl_fi, pDr_fi])


class ImageProcessorNode:
    def __init__(self):
        rospy.init_node('image_processor_node')
        # Define suscriptor
        self.suscriptor = rospy.Subscriber("/image_with_pose_array", ImageWithPoseArray, self.imagePose_callback, queue_size=5)
        # Define publicador
        self.points_pub = rospy.Publisher("/interest_points", InterestPointArray, queue_size=10)  # SE PUEDE AÑADIR UN PUBLICADOR PARA VISUALIZAR LAS INFERENCIAS DEL MODELO
        
        # Publicador de los resultados de segmentación
        self.image_pub = rospy.Publisher('/segmented_image', Image, queue_size=1)
        #self.T_WB = Trasl(0,0,0) # Inicializar la transformadas de T_WB, como identidad  (Si es que no hay odometria 0 )
        self.bridge = CvBridge()
        self.points_of_interest = []
        self.clsInterestPoints = []
        self.i = 0
        self.j = 0

    def imagePose_callback(self, msg):

        start_time = time.time()

        #Inicializar el mensaje final de puntos de interés
        interestPoints_msg = InterestPointArray()
        interestPoints_msg.header.stamp = rospy.Time.now()
        interestPoints_msg.header.frame_id = "odom"  # Cambia el marco según sea necesario

        for img_pose in msg.data:
            img_data = img_pose.image
            pose_data = img_pose.pose.pose
            # VERIFICACIONES

            if img_data is None:
                rospy.logwarn("El mensaje CompressedImage es None.")
                return
            if pose_data is None:
                rospy.logwarn("El mensaje PoseStamped es None.")
                return
                
            # Verificar que los datos no estén vacíos
            if not img_data.data:
                rospy.logwarn("El mensaje CompressedImage tiene datos vacíos.")
                return
            # Verificar si la posición está en sus valores por defecto
            if (pose_data.position.x == 0.0 and pose_data.position.y == 0.0 and pose_data.position.z == 0.0 and pose_data.orientation.x == 0.0 and 
                pose_data.orientation.y == 0.0 and pose_data.orientation.z == 0.0 and pose_data.orientation.w == 0.0):
                rospy.logwarn("El mensaje PoseStamped contiene valores por defecto.")
                return
            # Verificar que el quaternion sea válido (no debe ser cero ni NaN)
            if (pose_data.orientation.x != pose_data.orientation.x or pose_data.orientation.y != pose_data.orientation.y or
                pose_data.orientation.z != pose_data.orientation.z or pose_data.orientation.w != pose_data.orientation.w or
                (pose_data.orientation.x == 0.0 and pose_data.orientation.y == 0.0 and pose_data.orientation.z == 0.0 and pose_data.orientation.w == 0.0)):
                rospy.logwarn("El mensaje PoseStamped tiene un quaternion inválido.")
                return
            # Verificar si los datos son solo ceros (opcional)
            if all(byte == 0 for byte in img_data.data):
                rospy.logwarn("El mensaje CompressedImage está lleno con ceros.")
                return
            
            # Convertir de sensor_msgs/Image a una imagen OpenCV
            # frame = self.bridge.imgmsg_to_cv2(img_data, desired_encoding='bgr8')

            # Convertir el mensaje de ROS sensor_msgs/CompressedImage a imagen de OpenCV
            np_arr = np.frombuffer(img_data.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # rospy.loginfo("Frame shape is: " + str(frame.shape))
            #cv2.imshow('Ventana de Imagen',frame)
            # Procesamiento de la imagen y detección de puntos de interés
            points, cls, clss_names = self.process_image(frame, pose_data) # Lista de puntos
            clss_names = json.dumps(clss_names)

            if points: # Verificar que la lista no este vacía
                rospy.loginfo("Interest points found are: " + str(points))
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
        # Calcular el tiempo de procesamiento
        rospy.loginfo(f"Processing time: {time.time() - start_time:.4f} seconds")
        gc.collect()
    
    def process_image(self, frame, pose):
        #self.T_WB = pose_to_homogeneous_matrix(pose) # Transformación del world al punto medio entre las dos cámaras

        # Extraer los valores del cuaternión
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # Calcular el ángulo de rotación en el eje Z (yaw) usando la fórmula analítica
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*2 + qz*2))

        self.T_WB = Trasl(pose.position.x,pose.position.y,0)*Trotz(pose.orientation.z) # Transformación del world a la base del robot
        
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
        #ros_image = self.bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")
        #self.image_pub.publish(ros_image)

        # Crear un nombre único para cada imagen
        nombre_imagen = f"imagen_{self.i+1}.jpg"  # imagen_1.jpg, imagen_2.jpg, etc.
        # Ruta completa donde se guardará la imagen
        output_path = os.path.join(output_folder_seg, nombre_imagen)
        # Guardar la imagen en la ruta especificada
        cv2.imwrite(output_path, segmented_image)
        
        
        # Filtrar instancias o polígonos en un área específica (de interés), que será convertida a vista de pájaro
        instancias_filtradas, _, cls_filt = filtrar_instancias(area_filtrado, masks, cls)
        image_vp, instancias_vp = conversion_vista_pajaro(finalImage, instancias_filtradas, H_vpfi, config.width_vp, config.height_vp)
    
        subareas, cls_sub = division_subareas(instancias_vp, cls_filt, config.alturaDivision, config.umbralAgrupacion)
        # Convertir la imagen_vp en escala de grises
        image_grayscale = cv2.cvtColor(image_vp, cv2.COLOR_RGB2GRAY)
        # Aplicar el umbral adaptativo en la imagen
        umbral_adaptativo = cv2.adaptiveThreshold(image_grayscale, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 17, -1) #PARÁMETROS A TESTEAR
        # Convertir la imagen de OpenCV a un mensaje ROS
        #image_msg = self.bridge.cv2_to_imgmsg(umbral_adaptativo, encoding="mono8")
        #self.image_pub.publish(image_msg)

        # Ruta completa donde se guardará la imagen
        output_path = os.path.join(output_folder_adap, nombre_imagen)
        # Guardar la imagen en la ruta especificada
        cv2.imwrite(output_path, umbral_adaptativo)
    
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
            image_msg = self.bridge.cv2_to_imgmsg(masked_image, encoding="mono8")
            self.image_pub.publish(image_msg)

            # Convertir el polígono a una lista de puntos enteros (OpenCV requiere int)
            puntos = np.array(list(subp.exterior.coords), dtype=np.int32)
            # Dibujar el contorno del polígono en la imagen
            cv2.polylines(image_vp, [puntos], isClosed=True, color=(0, 255, 0), thickness=1)

            # NOMINACIÓN 
            if ratioPreservacion < config.umbralPreservacion: # Si pasa es menor al umbral entonces se selecciona como punto de interes
                point = subareas[idx].centroid
                self.points_of_interest.append(point)
                self.clsInterestPoints.append(cls_sub[idx])
        
        # Ruta completa donde se guardará la imagen
        output_path = os.path.join(output_folder_areasvp, nombre_imagen)
        # Guardar la imagen en la ruta especificada
        cv2.imwrite(output_path, image_vp)
        
        self.i = self.i +1

        if len(self.points_of_interest) == 0:
            return [], [], cls_names
        
        self.interestPointsArray= np.array([(point.x, point.y) for point in self.points_of_interest]) # Convertir a numpy array
        # Añade la tercera coordenada con 1s a los puntos 2D.
        self.interestPointsArray = np.hstack([self.interestPointsArray, np.ones((self.interestPointsArray.shape[0], 1))])
        # Aplicar matriz de homografía para pasar de {vp} a {B} frame
        self.interestPointsArray = self.interestPointsArray @ H_Bvp.T  # H_Bvp.T 
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

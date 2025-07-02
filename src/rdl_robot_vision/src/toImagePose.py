#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from rdl_robot_msgs.msg import ImageWithPose, ImageWithPoseArray
from std_srvs.srv import Trigger, TriggerResponse
from collections import deque
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImagePoseSynchronizer:
    def __init__(self):
        rospy.init_node('image_pose_synchronizer', anonymous=True)

        # Parámetro de intervalo de grabación (en segundos)
        self.record_interval = rospy.get_param("~record_interval", 1.0)

        # Variables para almacenar las imágenes y poses más recientes
        self.last_image = None
        self.last_pose = None

        # Contenedor para los datos a almacenar
        self.data_array = []

        # Para mantener el tiempo de grabación en intervalos
        self.last_record_time = time.time()

        # Subscriptores de imagen y pose
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.pose_callback)

        # Publicador del array de datos con imagen y pose
        self.image_pose_pub = rospy.Publisher('/image_with_pose_array', ImageWithPoseArray, queue_size=10)

        # Servicio para disparar la publicación de los datos
        self.service = rospy.Service('lane_marker_map_merge', Trigger, self.handle_service_call)

        # Inicializamos el objeto CvBridge para convertir imágenes
        self.bridge = CvBridge()

    def image_callback(self, image_msg):
        """
        Recibe un mensaje de tipo Image y lo convierte a CompressedImage
        """
        try:
            # Convertir la imagen de ROS a un formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Comprimir la imagen en formato JPEG
            _, compressed_image_data = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

            # Crear un mensaje CompressedImage
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header = image_msg.header
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = compressed_image_data.tobytes()

            # Guardamos la imagen comprimida
            #rospy.loginfo("Imagen comprimida recibida.")
            self.last_image = compressed_image_msg

        except CvBridgeError as e:
            rospy.logerr("Error al convertir la imagen: %s", e)

    def pose_callback(self, pose_msg):
        # Guardamos la última pose recibida
        #rospy.loginfo("Pose recibida.")
        self.last_pose = pose_msg

    def record_data(self):
        """
        Graba los datos de imagen y pose solo cuando hay ambos y publica
        """
        # Solo grabamos si tenemos tanto imagen como pose
        if self.last_image is not None and self.last_pose is not None:
            # Crear el mensaje ImageWithPose
            image_with_pose = ImageWithPose()
            image_with_pose.image = self.last_image
            image_with_pose.pose.pose = self.last_pose.pose.pose  # solo la parte de pose de Odometry

            # Añadir el nuevo dato al array de datos
            self.data_array.append(image_with_pose)

    def handle_service_call(self, req):
        """
        Manejador del servicio. Publica los datos una sola vez cuando es invocado
        """
        if self.data_array:
            # Crear el mensaje ImageWithPoseArray y llenarlo con los datos
            image_with_pose_array = ImageWithPoseArray()
            image_with_pose_array.header.stamp = rospy.Time.now()
            image_with_pose_array.header.frame_id = "world"  # o el frame que desees
            image_with_pose_array.data = self.data_array  # Usa los datos acumulados

            # Publicar los datos sincronizados
            self.image_pose_pub.publish(image_with_pose_array)

            rospy.loginfo("Datos publicados exitosamente.")
            # Retorna una respuesta exitosa
            return TriggerResponse(success=True, message="Datos publicados.")
        else:
            # Si no hay datos en el array, informar que no hay nada para publicar
            rospy.logwarn("No hay datos para publicar.")
            return TriggerResponse(success=False, message="No hay datos para publicar.")

    def run(self):
        """
        Ejecuta el nodo, procesando y grabando datos en intervalos regulares
        """
        while not rospy.is_shutdown():
            self.record_data()  # Intenta grabar los datos cuando haya imagen y pose
            rospy.sleep(2)  # Revisa en intervalos cortos para no bloquear el hilo principal

if __name__ == '__main__':
    try:
        node = ImagePoseSynchronizer()
        node.run()  # Inicia el proceso de grabación
    except rospy.ROSInterruptException:
        pass

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from rdl_robot_msgs.msg import ImageWithPose, ImageWithPoseArray  # Asegúrate de compilar tus mensajes personalizados.

import threading

class OdomImageSync:
    def __init__(self):
        rospy.init_node('sync_pose_image_node')

        # Variables para almacenar los últimos mensajes recibidos
        self.last_odom = None
        self.last_image = None
        self.synced_data = []  # Lista de datos sincronizados
        self.lock = threading.Lock()  # Bloqueo para acceso seguro

        # Suscriptores
        rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_image)

        # Publicador
        self.publisher = rospy.Publisher('/image_with_pose_array', ImageWithPoseArray, queue_size=10)

        # Servicio para devolver los datos sincronizados
        rospy.Service('/get_synced_data', Trigger, self.handle_get_synced_data)

        rospy.loginfo("Nodo de sincronización de Odometry y CompressedImage en ejecución.")

    def callback_odom(self, msg):
        """Callback para manejar mensajes de Odometry."""
        with self.lock:
            self.last_odom = msg
            self.check_and_store()

    def callback_image(self, msg):
        """Callback para manejar mensajes de CompressedImage."""
        with self.lock:
            self.last_image = msg
            self.check_and_store()

    def check_and_store(self):
        """Verifica si ambos mensajes están disponibles y sincronizados."""
        if self.last_odom is not None and self.last_image is not None:
            odom_time = self.last_odom.header.stamp
            image_time = self.last_image.header.stamp
            time_diff = abs(odom_time.to_sec() - image_time.to_sec())
            max_time_diff = 0.1  # 100 ms de tolerancia

            if time_diff <= max_time_diff:
                synced_entry = {
                    'pose': self.last_odom.pose.pose,
                    'image': self.last_image
                }
                self.synced_data.append(synced_entry)
                self.publish_synced_data()

                # Resetear los últimos datos para esperar nuevos mensajes
                self.last_odom = None
                self.last_image = None

    def publish_synced_data(self):
        """Publica los datos sincronizados acumulados en los tópicos."""
        poses_msg = []
        images_msg = []
        current_time = rospy.Time.now()

        # Convertir los datos sincronizados en mensajes PoseStamped y CompressedImage
        for entry in self.synced_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = current_time
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.pose = entry['pose']
            poses_msg.append(pose_stamped)

            images_msg.append(entry['image'])

        # Publicar en los tópicos
        for pose in poses_msg:
            self.pub_synced_poses.publish(pose)
        for image in images_msg:
            self.pub_synced_images.publish(image)

    def handle_get_synced_data(self, req):
        """Maneja la solicitud del servicio para devolver los datos sincronizados."""
        with self.lock:
            if not self.synced_data:
                return TriggerResponse(success=False, message="No hay datos sincronizados disponibles.")

            poses = [entry['pose'] for entry in self.synced_data]
            images = [entry['image'] for entry in self.synced_data]

            # Limpiar la lista después de entregar los datos
            self.synced_data = []

        rospy.loginfo(f"Servicio solicitado: se enviaron {len(poses)} pares sincronizados.")
        return TriggerResponse(success=True, message=f"Se han enviado {len(poses)} pares sincronizados.")

    def run(self):
        """Ejecuta el nodo."""
        rospy.spin()

if __name__ == '__main__':
    sync_node = OdomImageSync()
    sync_node.run()

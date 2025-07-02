#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from rdl_robot_msgs.msg import InterestPointArray
import struct  # Importar struct para convertir RGB a float

class PointCloudPublisher:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('interest_point_cloud_publisher')

        # Crea un publicador para publicar la nube de puntos en el tópico '/point_cloud'
        self.pc_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

        # Suscribirse al tópico donde se reciben los puntos de interés
        rospy.Subscriber('/interest_points', InterestPointArray, self.interest_points_callback)

        # Inicializa la lista de puntos y el flag de procesamiento
        self.points = []
        self.processing_done = False

    def interest_points_callback(self, msg):
        rospy.loginfo("Recibiendo %d puntos", len(msg.points))

        # Vaciar la lista de puntos para evitar duplicados de iteraciones anteriores
        self.points = []

        # Procesar los puntos recibidos y agregarlos a la lista 'points'
        for point in msg.points:
            self.points.append((point.pose.position.x, point.pose.position.y, 0.0))

        # Cambiar el flag indicando que ya se terminó el procesamiento
        self.processing_done = True
        rospy.loginfo("Procesamiento de puntos finalizado")

        # Guardar los puntos en un archivo txt
        self.save_points_to_file()

        # Publicar la nube de puntos
        self.publish_pointcloud()

    def save_points_to_file(self):
        """Guarda los puntos procesados en un archivo txt"""
        file = open('/home/eliascm07/interest_points.txt', 'w')
        for point in self.points:
            x, y, z = point
            file.write(f"{x} {y} {z}\n")
            print("guardando.....")
        file.close()
        rospy.loginfo("Puntos guardados en 'interest_points.txt'")

    def publish_pointcloud(self):
        if not self.processing_done:
            return

        # Crear un header para la nube de puntos
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "odom"  # Asegúrate de que este frame esté disponible en RViz

        # Convertir los puntos a un formato adecuado para PointCloud2 con color verde
        cloud_points = []
        for p in self.points:
            x, y, z = p
            rgb = self.rgb_to_float(0, 255, 0)  # Color verde (R=0, G=255, B=0)
            cloud_points.append((x, y, z, rgb))

        # Crear el mensaje PointCloud2
        cloud = pc2.create_cloud(header, self.create_fields(), cloud_points)

        rospy.loginfo("Cantidad de puntos: " + str(len(self.points)))
        # Publicar la nube de puntos
        self.pc_pub.publish(cloud)

        # Restablecer el flag para esperar nuevos datos
        self.processing_done = False

    def create_fields(self):
        """Define los campos del PointCloud2 (x, y, z, rgb)."""
        return [
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            pc2.PointField('rgb', 12, pc2.PointField.FLOAT32, 1)
        ]

    def rgb_to_float(self, r, g, b):
        """Convierte un color RGB a un float para usar en PointCloud2."""
        rgb = (int(r) << 16) | (int(g) << 8) | int(b)
        return struct.unpack('f', struct.pack('I', rgb))[0]

    def run(self):
        """Mantiene el nodo activo mientras se espera por puntos de interés."""
        rospy.spin()

if __name__ == "__main__":
    try:
        node = PointCloudPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass

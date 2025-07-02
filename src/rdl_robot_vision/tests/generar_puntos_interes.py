#!/usr/bin/env python3
import rospy
from rdl_robot_msgs.msg import InterestMarkingPoint, InterestPointArray

class FileProcessorNode:
    def __init__(self):
        rospy.init_node('file_processor_node')
        # Publicador
        self.points_pub = rospy.Publisher("/interest_points", InterestPointArray, queue_size=10)
        self.file_path = "/home/eliascm07/catkin_ws/src/rdl_robot_vision/tests/interest_points.txt"  # Cambia esta ruta por la del archivo de texto

    def read_coordinates_from_file(self):
        """
        Lee un archivo de texto con formato de dos columnas (x y) y devuelve una lista de puntos.
        """
        points = []
        try:
            with open(self.file_path, 'r') as file:
                for line in file:
                    try:
                        x, y, z = map(float, line.strip().split())  # Lee las dos columnas
                        points.append((x, y))
                    except ValueError:
                        rospy.logwarn(f"Línea inválida en el archivo: {line.strip()}")
        except FileNotFoundError:
            rospy.logerr(f"Archivo no encontrado en: {self.file_path}")
        return points

    def publish_points(self):
        """
        Publica los puntos leídos del archivo como un InterestPointArray.
        """
        points = self.read_coordinates_from_file()
        if not points:
            rospy.logwarn("No se encontraron puntos en el archivo.")
            return

        interest_points_msg = InterestPointArray()
        interest_points_msg.header.stamp = rospy.Time.now()
        interest_points_msg.header.frame_id = "map"  # Cambia el frame según sea necesario

        for x, y in points:
            interest_point = InterestMarkingPoint()
            interest_point.header = interest_points_msg.header
            interest_point.marking_class = 0
            interest_point.pose.position.x = x
            interest_point.pose.position.y = y
            interest_point.pose.position.z = 0.0  # Asignar z según sea necesario
            interest_point.pose.orientation.w = 1.0  # Orientación nula

            interest_points_msg.points.append(interest_point)

        self.points_pub.publish(interest_points_msg)
        rospy.loginfo(f"Se publicaron {len(points)} puntos.")

    def spin(self):
        rate = rospy.Rate(1)  # Publicar una vez por segundo
        while not rospy.is_shutdown():
            self.publish_points()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = FileProcessorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

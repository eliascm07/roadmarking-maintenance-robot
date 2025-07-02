#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistStamped

def odometry_callback(odom_msg):
    try:
        # Obtener la transformación entre 'odom' y 'base_link'
        (trans, rot) = tf_listener.lookupTransform(target_frame, odom_msg.header.frame_id, rospy.Time(0))

        # Transformar la pose
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose
        transformed_position = tf_listener.transformPose(target_frame, pose_stamped)

        # Transformar el twist (velocidades)
        twist_stamped = TwistStamped()
        twist_stamped.header = odom_msg.header
        twist_stamped.twist = odom_msg.twist.twist
        transformed_twist = transform_twist(twist_stamped, odom_msg.header.frame_id, target_frame)

        # Crear un nuevo mensaje de Odometry con la pose transformada
        transformed_odom = Odometry()
        transformed_odom.header.stamp = odom_msg.header.stamp  # Mantener el tiempo original
        transformed_odom.header.frame_id = target_frame        # Frame objetivo (por ejemplo, "base_link")
        transformed_odom.child_frame_id = odom_msg.child_frame_id
        transformed_odom.pose.pose = transformed_position.pose # Asignar la pose transformada
        transformed_odom.twist = odom_msg.twist                # Copiar la velocidad original
        transformed_odom.twist.twist = transformed_twist.twist # Asignar el twist transformado

        # Publicar el mensaje de odometría transformado
        odom_pub.publish(transformed_odom)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn(f"Transformación fallida: {e}")

def transform_twist(twist_stamped, source_frame, target_frame):
    """
    Transforma el twist (velocidad lineal y angular) desde source_frame a target_frame.
    """

    linearstamped = Vector3Stamped()
    linearstamped.header = twist_stamped.header
    linearstamped.vector = twist_stamped.twist.linear

    angularstamped = Vector3Stamped()
    angularstamped.header = twist_stamped.header
    angularstamped.vector = twist_stamped.twist.angular


    # Transformar velocidades al frame objetivo
    linear_transformed = tf_listener.transformVector3(target_frame, linearstamped)
    angular_transformed = tf_listener.transformVector3(target_frame, angularstamped)

    # Crear un nuevo TwistStamped con las velocidades transformadas
    transformed_twist = TwistStamped()
    transformed_twist.header.frame_id = target_frame
    transformed_twist.twist.linear = linear_transformed.vector
    transformed_twist.twist.angular = angular_transformed.vector

    return transformed_twist

if __name__ == "__main__":
    rospy.init_node("odom_transformer_node")

    # Definir el frame objetivo (por ejemplo, base_link)
    target_frame = "base_footprint"

    # Inicializar el listener de tf
    tf_listener = tf.TransformListener()

    # Publicador de la odometría transformada
    odom_pub = rospy.Publisher("/transformed_odom", Odometry, queue_size=10)

    # Suscriptor a la odometría original
    rospy.Subscriber("/odom", Odometry, odometry_callback)

    rospy.spin()

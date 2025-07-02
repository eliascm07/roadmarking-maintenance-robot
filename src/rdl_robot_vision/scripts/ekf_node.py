#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import utm

# Estado inicial
x = np.array([0, 0, 0, 0, 0])  # [x, y, theta, v, omega]
P = np.diag([0.70, 0.70, 0.01, 0.01, 0.04])  # Matriz de covarianza inicial

# Matrices de proceso y medición
Q = np.diag([0.3, 0.3, 0.04, 0.1, 0.04])  # Covarianza del proceso ##tuning
# Covarianzas de gps e imu que luego se van a sumar en la etapa de correccion. Los separé porque la covarianza del imu es fija con esos valores segun el datasheet. La covarianza del gps se obtiene en el callback del mismo, osea que puede variar entre mediciones
R_gps = np.zeros((4, 4))
R_imu = np.zeros((4, 4))
#R_gps[0, 0] = 0.864
#R_gps[1, 1] = 0.864
R_imu[2, 2] = 0.0159  # orientación            # tuning
R_imu[3, 3] = 0.04  # velocidad angular

dt = 0.01  # Ajustado a la frecuencia del IMU

# Bool que luego de la primera medición de GPS será false. Es la posición y orientación en la que se inicializa el frame /odom
first_gps_reading = True
first_imu_reading = True
initial_x = 0
initial_y = 0
initial_theta = 0

latest_gps_x = 0
latest_gps_y = 0
gps_received = False

tf_broadcaster = tf2_ros.TransformBroadcaster()
static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

def predict(x, P, dt, a_x):
    theta = x[2]
    v = x[3]
    omega = x[4]
    # Modelo del sistema
    x_pred = np.array([
        x[0] + v * np.cos(theta) * dt,
        x[1] + v * np.sin(theta) * dt,
        x[2] + omega * dt,
        v + a_x * dt,
        omega
    ])
    # Jacobiano del modelo del sistema    
    F = np.array([
        [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt, 0],
        [0, 1,  v * np.cos(theta) * dt, np.sin(theta) * dt, 0],
        [0, 0, 1, 0, dt],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])

    P_pred = np.dot(np.dot(F, P), F.T) + Q
    return x_pred, P_pred

def update(x, P, z, R):
    # Jacobiano del modelo de medición    
    H = np.array([
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1]
    ])

    y = z - np.dot(H, x)  # Residual
    S = np.dot(np.dot(H, P), H.T) + R  # Innovación
    K = np.dot(np.dot(P, H.T), np.linalg.inv(S))  # Ganancia de Kalman
    x_upd = x + np.dot(K, y)
    P_upd = P - np.dot(np.dot(K, H), P)
    return x_upd, P_upd

# Función de expansión de la matriz de covarianza a 6x6 porque el mensaje del tipo PoseWithCovarianceStamped solo admite covarianzas de ese tamaño. Nosotros tenemos covarianza de 5x5
def expand_covariance_matrix(P):
    # Crear una matriz de 36 elementos inicializada con ceros
    covariance = np.zeros(36)

    # Copiar los elementos significativos de P en la matriz de covarianza
    covariance[0] = P[0, 0]  # P(0,0)
    covariance[1] = P[0, 1]  # P(0,1)
    covariance[6] = P[0, 2]  # P(0,2)
    covariance[7] = P[1, 1]  # P(1,1)
    covariance[11] = P[1, 2]  # P(1,2)
    covariance[14] = P[2, 2]  # P(2,2)
    covariance[21] = P[3, 3]  # P(3,3)
    covariance[28] = P[4, 4]  # P(4,4)

    # Asignar los elementos simétricos
    covariance[5] = P[1, 0]  # P(1,0) -> Igual a P(0,1)
    covariance[15] = P[2, 0]  # P(2,0) -> Igual a P(0,2)
    covariance[16] = P[2, 1]  # P(2,1) -> Igual a P(1,2)

    return covariance.flatten().tolist()

def imu_callback(data):
    global x, P, gps_received, latest_gps_x, latest_gps_y, R_gps, first_imu_reading, initial_theta

    a_x = data.linear_acceleration.x
    omega_imu = data.angular_velocity.z
    # La orientación en IMU es en quaternion, se convierte a angulo
    theta_imu = 2 * np.arctan2(data.orientation.z, data.orientation.w)

    # Inicializar theta con la primera lectura del IMU
    if first_imu_reading:
        initial_theta = theta_imu
        x[2] = theta_imu
        first_imu_reading = False

    # Realizar la predicción
    x, P = predict(x, P, dt, a_x)

    # Desde que recibe lectura de GPS, actualiza
    if gps_received:
        z = np.array([latest_gps_x, latest_gps_y, theta_imu, omega_imu])
        R = R_gps + R_imu
        x, P = update(x, P, z, R)

        # Publicar la posición estimada
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom_ekf"  # 'odom' como frame padre
        pose.pose.pose.position.x = x[0] - initial_x
        pose.pose.pose.position.y = x[1] - initial_y
        pose.pose.pose.orientation.z = np.sin((x[2] - initial_theta) / 2)
        pose.pose.pose.orientation.w = np.cos((x[2] - initial_theta) / 2)
        pose.pose.covariance = expand_covariance_matrix(P)

        pose_pub.publish(pose)

def gps_callback(data):
    global x, P, R_gps, first_gps_reading, initial_x, initial_y, latest_gps_x, latest_gps_y, gps_received

    latitude = data.latitude
    longitude = data.longitude
    x_utm, y_utm, zone_number, zone_letter = utm.from_latlon(latitude, longitude)

    if first_gps_reading:
        initial_x = x_utm
        initial_y = y_utm
        x[0] = x_utm
        x[1] = y_utm
        first_gps_reading = False

        # Publicar la transformación estática de map a odom_ekf
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom_ekf"
        t.transform.translation.x = initial_x
        t.transform.translation.y = initial_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = np.sin(initial_theta / 2.0)
        t.transform.rotation.w = np.cos(initial_theta / 2.0)
        static_tf_broadcaster.sendTransform(t)

    position_covariance = data.position_covariance
    R_gps[0, 0] = position_covariance[0]  # Covarianza de la posición x del GPS
    R_gps[1, 1] = position_covariance[4]  # Covarianza de la posición y del GPS

    latest_gps_x = x_utm
    latest_gps_y = y_utm
    gps_received = True

def broadcast_odom_to_base_link(event):
    global x, initial_x, initial_y, initial_theta
    # Publicar la transformación de odom_ekf a base_link
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom_ekf"
    t.child_frame_id = "base_link"
    t.transform.translation.x = x[0] - initial_x
    t.transform.translation.y = x[1] - initial_y
    t.transform.translation.z = 0.0
    t.transform.rotation.z = np.sin((x[2] - initial_theta) / 2.0)
    t.transform.rotation.w = np.cos((x[2] - initial_theta) / 2.0)
    tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('ekf_node')

    # Suscribirse a los tópicos de IMU y GPS
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.Subscriber('/fix', NavSatFix, gps_callback)

    # Publicador para la posición estimada
    pose_pub = rospy.Publisher('/ekf_pose', PoseWithCovarianceStamped, queue_size=10)

    # Configurar un timer para publicar la transformación de odom a base_link a una frecuencia más alta
    rospy.Timer(rospy.Duration(0.01), broadcast_odom_to_base_link)  # Publicar a 100Hz

    rospy.spin()

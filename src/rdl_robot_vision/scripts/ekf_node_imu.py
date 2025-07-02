#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros

# Estado inicial
x = np.array([0, 0, 0, 0, 0])  # [x, y, theta, v, omega]
P = np.diag([0.70, 0.70, 0.01, 0.01, 0.04])  # Matriz de covarianza inicial

# Matrices de proceso y medición
Q = np.diag([0.3, 0.3, 0.04, 0.1, 0.04])  # Covarianza del proceso ## tuning
R_imu = np.zeros((4, 4))
R_imu[2, 2] = 0.0159  # orientación
R_imu[3, 3] = 0.04  # velocidad angular

dt = 0.01  # Ajustado a la frecuencia del IMU

# Flags y valores iniciales
first_imu_reading = True
initial_theta = 0

tf_broadcaster = tf2_ros.TransformBroadcaster()

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

def expand_covariance_matrix(P):
    covariance = np.zeros(36)
    covariance[0] = P[0, 0]  # P(0,0)
    covariance[7] = P[1, 1]  # P(1,1)
    covariance[14] = P[2, 2]  # P(2,2)
    covariance[21] = P[3, 3]  # P(3,3)
    covariance[28] = P[4, 4]  # P(4,4)
    return covariance.flatten().tolist()

def imu_callback(data):
    global x, P, first_imu_reading, initial_theta

    a_x = data.linear_acceleration.x
    omega_imu = data.angular_velocity.z
    theta_imu = 2 * np.arctan2(data.orientation.z, data.orientation.w)

    if first_imu_reading:
        initial_theta = theta_imu
        x[2] = theta_imu
        first_imu_reading = False

    # Predicción
    x, P = predict(x, P, dt, a_x)

    # Actualización con datos del IMU
    z = np.array([theta_imu, omega_imu])
    x, P = update(x, P, z, R_imu)

    # Publicar la posición estimada
    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "odom_ekf"
    pose.pose.pose.position.x = x[0]
    pose.pose.pose.position.y = x[1]
    pose.pose.pose.orientation.z = np.sin((x[2] - initial_theta) / 2)
    pose.pose.pose.orientation.w = np.cos((x[2] - initial_theta) / 2)
    pose.pose.covariance = expand_covariance_matrix(P)
    pose_pub.publish(pose)

def broadcast_odom_to_base_link(event):
    global x, initial_theta
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom_ekf"
    t.child_frame_id = "base_link"
    t.transform.translation.x = x[0]
    t.transform.translation.y = x[1]
    t.transform.rotation.z = np.sin((x[2] - initial_theta) / 2.0)
    t.transform.rotation.w = np.cos((x[2] - initial_theta) / 2.0)
    tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('ekf_node')

    # Suscribirse al tópico del IMU
    rospy.Subscriber('/imu', Imu, imu_callback)

    # Publicador para la posición estimada
    pose_pub = rospy.Publisher('/ekf_pose', PoseWithCovarianceStamped, queue_size=10)

    # Configurar un timer para publicar la transformación
    rospy.Timer(rospy.Duration(0.01), broadcast_odom_to_base_link)

    rospy.spin()

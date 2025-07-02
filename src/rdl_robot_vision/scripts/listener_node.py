#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

# Función de callback que se llama cuando se recibe un mensaje en el tópico
def callback(data):
    # Imprimir los datos recibidos en la terminal
    rospy.loginfo("Datos recibidos: %s", data.data)

def listener():
    # Inicializar el nodo llamado 'float_array_listener'
    rospy.init_node('float_array_listener', anonymous=True)
    
    # Suscribirse al tópico. Cambia 'nombre_del_topico' por el nombre real del tópico.
    rospy.Subscriber("/interest_points", Float32MultiArray, callback)
    
    # Mantener el nodo en funcionamiento hasta que se detenga
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

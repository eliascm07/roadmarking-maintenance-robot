#!/usr/bin/env python3

import rospy
from rdl_robot_msgs.msg import MarkingPoint, PathMarking, InterestPointArray
from nav_msgs.msg import Odometry
import numpy as np
from python_tsp.exact import solve_tsp_branch_and_bound
from functions import aplicar_dbscan, generar_rectas, construir_grafo, crear_matriz_distancias
import json
import gc

espaciado_muestreo = 0.3 # en metros - de los puntos de la recta ------------------------------------------- AJUSTAR ESTE PARAMETRO --------------------------

def crear_archivo_txt(lista_puntos, nombre_archivo="/home/eliascm07/trayectoria.txt"):
    """
    Crea un archivo de texto con los datos x, y, theta a partir de una lista de puntos.
    
    Args:
    - lista_puntos: lista de tuplas (x, y, theta).
    - nombre_archivo: nombre del archivo de salida (por defecto: "trayectoria.txt").
    """
    with open(nombre_archivo, 'w') as archivo:
        for punto in lista_puntos:
            x, y, theta = punto
            archivo.write(f"{x:.3f} {y:.3f} {theta:.6f}\n")
    print(f"Archivo '{nombre_archivo}' creado con éxito.")

def procesar_lista(booleanos, trayectoria):
    resultado = []
    dentro_subconjunto = False
    print(booleanos)
    for i, valor in enumerate(booleanos):
        if valor:  # Inicia o termina un subconjunto
            if dentro_subconjunto:  # Si ya estaba dentro de un subconjunto
                resultado[-1] = False  # El elemento anterior es el final del subconjunto
            dentro_subconjunto = not dentro_subconjunto  # Alterna entre dentro y fuera


        
        if dentro_subconjunto: # Cuando empieza o esta dentro una recta
            x1, y1 = trayectoria[i]
            x2, y2 = trayectoria[i + 1]
            # Calcular el ángulo
            dx = x2 - x1
            dy = y2 - y1
            angulo = np.arctan2(dy, dx)  # Ángulo en radianes
    
            # Modificar el punto actual añadiendo el ángulo
            trayectoria[i] = (x1, y1, angulo)
            resultado.append(True)
        else: # Cuando termina la recta
            # Para el último punto, asignar el mismo ángulo que el penúltimo
            print(i)
            x_last, y_last = trayectoria[i]
            angulo_ultimo = trayectoria[i-1][2]  # Ángulo del penúltimo punto
            trayectoria[i] = (x_last, y_last, angulo_ultimo)
            resultado.append(False)


    return resultado, trayectoria


class PathProcessorNode:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('path_processor_node')

        self.points = [] # Lista para almacenar puntos
        self.clases = [] # Lista para almacenar todas las clases para cada punto
        # Suscriptores
        self.sub_interestPoints = rospy.Subscriber('/interest_points', InterestPointArray, self.path_callback)
        # Define publicador de trayetoria final
        self.publicador = rospy.Publisher("/final_trayectory", PathMarking, queue_size=10)
        # Suscriptor al tópico de odometría
        rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)
        
        rospy.loginfo("Path Processor Node iniciado.")

    def odometry_callback(self, msg):
        """
        Callback para manejar los datos de odometría.
        """
        # Extraer la posición x, y de la odometría
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convertirlo a un arreglo NumPy
        self.current_position = np.array([x, y])


    def path_callback(self, msg):
        # if self.is_storing: # Si aún no se ha terminado el recorrido
        print(len(msg.points))
        for point in msg.points:
            self.points.append([point.pose.position.x, point.pose.position.y]) # Se procesa todos los puntos y todas las clases asociadas a esos puntos
            self.clases.append(point.marking_class)

        rospy.loginfo("Mensaje recibido. Puntos de interés recibido y generando trayectoria del robot.")

        pointsOpt, clasesOpt, paintOpt = self.process_points() # Generación de trayectoria optima
        # clases_names = json.loads(msg.class_names) # Diccionario con el nombre las clases, talvez ni se necesita

        if pointsOpt: # Verificar que la lista no este vacía
            pathMarks_msg = PathMarking()
            pathMarks_msg.header.stamp = rospy.Time.now()
            pathMarks_msg.header.frame_id = "odom"  # Cambia el marco según sea necesario
            pathMarks_msg.class_names = msg.class_names

            for i, point in enumerate(pointsOpt):
                PointMark = MarkingPoint()
                PointMark.header = pathMarks_msg.header
                PointMark.marking_class = int(clasesOpt[i])
                PointMark.pose.position.x = point[0]
                PointMark.pose.position.y = point[1]
                PointMark.pose.position.z = 0.0  # Si es necesario
                PointMark.pose.orientation.w = 1.0 #np.cos(point[2]/2)  #1.0 #  La orientacion theta en el eje z
                PointMark.pose.orientation.z = point[2] # np.sin(point[2]/2) #
                PointMark.paint = paintOpt[i]

                pathMarks_msg.points.append(PointMark)

            self.publicador.publish(pathMarks_msg) # Publicar los puntos de interés
            rospy.loginfo("Trayectoria óptima publicada, empezando control ...")
        else:
            rospy.logwarn(" No se econtró necesidad de realizar mantenimiento") # Liberar para que pueda hacer reconocimiento a otras partes
        gc.collect()

    def process_points(self):
        # Procesamiento con los puntos almacenados
        rospy.loginfo(f"Total de puntos recibidos del nodo anterior: {len(self.points)}")
        # Aquí puedes agregar tu procesamiento, por ejemplo:
        if self.points:
            # Generar puntos y aplicar DBSCAN
            puntos = np.array(self.points)
            np_clases = np.array(self.clases)
            etiquetas = aplicar_dbscan(puntos)

            rectas = generar_rectas(puntos, etiquetas, np_clases, espaciado_muestreo) # Generar rectas de ajuste
            G =construir_grafo(rectas) #, self.current_position)
            distance_matrix, nodelist = crear_matriz_distancias(G)
            distance_matrix[:, 0] = 0 # Para Open TSP

            try:
                if distance_matrix.shape[0] < 2 or distance_matrix.shape[0] != distance_matrix.shape[1]:
                    rospy.logwarn("La matriz de distancias no es válida para resolver el TSP.")
                    return
                
                if len(nodelist) != distance_matrix.shape[0]:
                    rospy.logwarn("El tamaño de la lista de nodos no coincide con la matriz de distancias.")
                    return

                permutation, distance = solve_tsp_branch_and_bound(distance_matrix)
                rospy.loginfo(f"Distancia mínima de recorrido del robot es: {distance}")
                trayectoria_optimizada = [nodelist[i] for i in permutation] # Lista de puntos de manera optima
                trayectoria_optimizada = trayectoria_optimizada[1:]

            except Exception as e:
                rospy.logdebug(f"Matriz de distancias:\n{distance_matrix}")
                rospy.logwarn(f"Error resolviendo TSP: {e}, ahora calculando con el vecino más cercano")
                # Resolver de la forma el vecino más cercano -----------------------------------------------
                trayectoria_optimizada =[]            

            clases_opt = [G.nodes[nodo]['clase'] for nodo in trayectoria_optimizada]
            listExtremPoint = [G.nodes[nodo]['extremePoint'] for nodo in trayectoria_optimizada]
            paint_opt, trayectoria_optimizada = procesar_lista(listExtremPoint, trayectoria_optimizada)
            print(trayectoria_optimizada)
            crear_archivo_txt(trayectoria_optimizada)
            
        else: # Cuando no se recibieron puntos
            # Opcionalmente, vaciar la lista de puntos después del procesamiento
            self.points = []
            trayectoria_optimizada =[]
            clases_opt = []
            paint_opt = []

        return trayectoria_optimizada, clases_opt, paint_opt

    def run(self):
        # Mantener el nodo corriendo
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PathProcessorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido.")
#!/usr/bin/env python3
import rospy
import numpy as np
import networkx as nx
from std_msgs.msg import Float32MultiArray

class NodoPublicadorMatriz:
    def __init__(self):
        # Inicializamos el nodo
        rospy.init_node('nodo_publicador_matriz', anonymous=True)
        
        # Creamos el publicador
        self.pub = rospy.Publisher('/matriz_distancias', Float32MultiArray, queue_size=10)
        
        # Definimos la frecuencia de publicación (en Hz)
        self.rate = rospy.Rate(1)  # 1 Hz
        
        # Creamos un grafo de ejemplo
        self.grafo = self.crear_grafo()
        
        # Generamos la matriz de distancias
        self.matriz_distancias = self.generar_matriz_distancias()

    def crear_grafo(self):
        """
        Función para crear un grafo no dirigido de ejemplo.
        Cada nodo representa un punto en 2D.
        """
        G = nx.Graph()

        # Añadimos nodos con posiciones (simulando un grafo en 2D)
        G.add_node((0, 0), pos=(0, 0))
        G.add_node((1, 0), pos=(1, 0))
        G.add_node((0, 1), pos=(0, 1))
        G.add_node((1, 1), pos=(1, 1))

        # Añadimos aristas entre los nodos
        G.add_edge((0, 0), (1, 0))
        G.add_edge((0, 0), (0, 1))
        G.add_edge((1, 0), (1, 1))
        G.add_edge((0, 1), (1, 1))

        return G

    def generar_matriz_distancias(self):
        """
        Función para generar la matriz de distancias (2D) a partir de un grafo no dirigido.
        """
        # Inicializamos la matriz de distancias (con 0's inicialmente)
        nodos = list(self.grafo.nodes)
        num_nodos = len(nodos)
        matriz = np.zeros((num_nodos, num_nodos), dtype=np.float32)

        # Llenamos la matriz de distancias utilizando las aristas del grafo
        for i, nodo_i in enumerate(nodos):
            for j, nodo_j in enumerate(nodos):
                if self.grafo.has_edge(nodo_i, nodo_j):
                    distancia = np.linalg.norm(np.array(self.grafo.nodes[nodo_i]['pos']) - np.array(self.grafo.nodes[nodo_j]['pos']))
                    matriz[i, j] = round(distancia, 3)
        
        return matriz

    def publicar_matriz(self):
        """
        Publica la matriz de distancias en el tópico '/matriz_distancias'.
        """
        # Creamos el mensaje de tipo Float32MultiArray
        mensaje = Float32MultiArray()
        
        # Aplanamos la matriz a una lista 1D para publicarla
        mensaje.data = self.matriz_distancias.flatten().tolist()

        # Publicamos el mensaje
        self.pub.publish(mensaje)
        
        # Imprimimos la matriz para verificar
        rospy.loginfo("Publicando matriz de distancias: %s", mensaje.data)

    def run(self):
        """
        Ejecuta el nodo, manteniendo la frecuencia de publicación.
        """
        while not rospy.is_shutdown():
            # Publicar la matriz
            self.publicar_matriz()

            # Controlar la tasa de publicación
            self.rate.sleep()


if __name__ == '__main__':
    try:
        nodo = NodoPublicadorMatriz()
        nodo.run()
    except rospy.ROSInterruptException:
        pass

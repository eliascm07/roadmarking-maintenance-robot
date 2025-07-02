import numpy as np
from sklearn.cluster import DBSCAN
from collections import Counter
import networkx as nx
import config_param as param
import rospy

def aplicar_dbscan(puntos):
    """
    Aplica el algoritmo DBSCAN para agrupar puntos.
    """
    clustering = DBSCAN(eps=param.eps, min_samples=param.min_samples).fit(puntos) # Se configura eps
    print(clustering)
    return clustering.labels_ # retorna una lista de etiquetas para cada punto

def generar_rectas(puntos, etiquetas, clases, espaciado):
    """
    Genera rectas a partir de puntos agrupados por etiquetas de DBSCAN.
    """
    rectas = []

    print(np.unique(etiquetas))
    for i in range(len(np.unique(etiquetas))):
        #if i == -1:  # Ignorar ruido, etiqueta == -1
         #   continue
        print("Este es el valor de i", i)
        grupo = puntos[etiquetas == i] # Filtra o extrae unicamente los puntos que pertenecen a la actual etiqueta i
        cls = clases[etiquetas == i] # Todas las clases con la etiqueta i

        # Contar las ocurrencias de cada valor
        conteo = Counter(cls)
        print("Conteo:", conteo)
        valor_mas_repetido, repeticiones = conteo.most_common(1)[0] # Encontrar el valor más repetido y su cantidad
        porcentaje = (repeticiones / len(cls)) * 100 # Encontrar el valor más repetido y su cantidad
        if porcentaje > param.umbralPorcentaje:
            clase = valor_mas_repetido
        else:
            clase = 3 # Que sería una clase indefinida

        X, Y = grupo[:, 0], grupo[:, 1]
        # Agregar la columna de unos para el término constante C
        X_matriz = np.c_[X, Y, np.ones(len(X))]
        # Aplicar SVD
        _, _, Vt = np.linalg.svd(X_matriz)
        A, B, C = Vt[-1]  # Tomar el último vector de Vt
        if abs(A/B) < 1: # Desde pendiente 1 a -1, comportamiento mas horizontal
            x_min, x_max = X.min() - 0.05, X.max() + 0.05
            rospy.logwarn(str(x_min) + ", "+ str(x_max))
            y_min, y_max = -A / B * x_min - C / B, -A / B * x_max - C / B
            print("Entre aqui")
        else: # abs(B/A)  # Vertical
            print("O aqui")
            print(type(grupo[:, 1].min()))
            y_min, y_max = Y.min() - 0.05, Y.max() + 0.05
            x_min, x_max = -B / A * y_min - C / A, -B / A * y_max - C / A

        x_vals = np.array([x_min, x_max])
        y_vals = np.array([y_min, y_max])
        num_puntos = max(int(np.hypot(x_max - x_min, y_max - y_min) / espaciado), 2)
        t = np.linspace(0, 1, num_puntos)
        # Calcular los puntos muestreados a lo largo de la recta
        sampled_points = (1 - t)[:, np.newaxis] * np.array([x_min, y_min]) + t[:, np.newaxis] * np.array([x_max, y_max])
        sampled_points = np.round(sampled_points, 3)
        cls = np.array([clase] * num_puntos)
        print("Los puntos de una recta son: ",sampled_points)
        rectas.append((x_vals, y_vals, sampled_points, cls))
    return rectas

def construir_grafo(rectas, curr_pos=np.array([113.5, 0])):   # ---------------------------------- FALTA SUBSCRIBIRSE PARA OBTENER LA POSE ACTUAL ------------------------------------------
    """
    Construye un grafo no dirigido a partir de una lista de sublistas con puntos en 2D.

    Parámetros
    ----------
    rectas : list
        Lista de tuplas o estructuras donde el tercer elemento contiene sublistas con puntos en 2D.
    curr_pos : np.ndarray, opcional
        Posición inicial del nodo de referencia, por defecto [0, 60].

    Retorna
    -------
    nx.Graph
        Grafo no dirigido con nodos representando los puntos 2D y aristas con peso basado en la distancia euclidiana.
    """
    # Extraer las sublistas de puntos y de clases de `rectas`
    puntos = [sublista for _, _, sublista, _ in rectas]
    clases = [clase for _, _, _, clase in rectas]

    # Crear el grafo no dirigido
    G = nx.Graph()

    # Nodo inicial
    first_nodo = tuple(curr_pos)
    G.add_node(first_nodo, pos=first_nodo, clase = 0, extremePoint = False)

    # Función para calcular la distancia euclidiana entre dos puntos
    def distancia_euclidiana(p1, p2):
        return round(np.linalg.norm(np.array(p2) - np.array(p1)), 3)

    # Añadir nodos y conectar puntos secuencialmente dentro de cada sublista
    for i, sublista in enumerate(puntos):
        num_puntos = len(sublista)
        for j in range(num_puntos):
            nodo = tuple(sublista[j])  # Usar la posición (x, y) del punto como nombre del nodo
            if j == num_puntos-1 or j==0:  # Verificar si es el último o primer elemento de cada lista
                G.add_node(nodo, pos=nodo, clase = clases[i][j], extremePoint = True)
            else:
                G.add_node(nodo, pos=nodo, clase = clases[i][j], extremePoint = False)
            #G.add_node(nodo, pos=nodo)

            if j > 0:
                nodo_anterior = tuple(sublista[j - 1])
                peso = distancia_euclidiana(nodo_anterior, nodo)
                G.add_edge(nodo_anterior, nodo, weight=peso)

    # Conectar puntos iniciales y finales entre sublistas
    for i in range(len(puntos)):
        sublista_i = puntos[i]
        inicio_i, fin_i = tuple(sublista_i[0]), tuple(sublista_i[-1])

        G.add_edge(first_nodo, inicio_i, weight=distancia_euclidiana(first_nodo, inicio_i))
        G.add_edge(first_nodo, fin_i, weight=distancia_euclidiana(first_nodo, fin_i))

        for j in range(i + 1, len(puntos)):
            sublista_j = puntos[j]
            inicio_j, fin_j = tuple(sublista_j[0]), tuple(sublista_j[-1])

            # Conectar nodos entre sublistas
            G.add_edge(inicio_i, inicio_j, weight=distancia_euclidiana(inicio_i, inicio_j))
            G.add_edge(inicio_i, fin_j, weight=distancia_euclidiana(inicio_i, fin_j))
            G.add_edge(fin_i, inicio_j, weight=distancia_euclidiana(fin_i, inicio_j))
            G.add_edge(fin_i, fin_j, weight=distancia_euclidiana(fin_i, fin_j))

    return G

def crear_matriz_distancias(G):
    """
    Crea una matriz de distancias a partir de un grafo.

    Parámetros
    ----------
    G : nx.Graph
        Grafo no dirigido con nodos y aristas con pesos.

    Retorna
    -------
    np.ndarray
        Matriz de distancias entre los nodos del grafo.
    """
    # Obtener la lista de nodos en orden
    nodes = list(G.nodes)
    n = len(nodes)

    # Crear una matriz de distancias inicializada en infinito
    distance_matrix = np.full((n, n), np.inf)

    # Asignar 0 a la diagonal principal (distancia de un nodo a sí mismo)
    np.fill_diagonal(distance_matrix, 0)

    # Llenar la matriz con las distancias (pesos de las aristas)
    for i, node1 in enumerate(nodes):
        for j, node2 in enumerate(nodes):
            if G.has_edge(node1, node2):  # Si existe una arista entre node1 y node2
                distance_matrix[i, j] = G[node1][node2]['weight']

    return distance_matrix, nodes
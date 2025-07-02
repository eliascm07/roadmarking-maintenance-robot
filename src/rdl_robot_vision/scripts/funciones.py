from shapely.geometry import Polygon, Point, MultiPolygon
from shapely.geometry import LineString
from shapely.ops import split
import cv2
import numpy as np
import rospy

# Filtra las mascaras de la segmentación dentro de una área determinada
def filtrar_instancias(area, lista_masks, clases):
    instancias_pol = []
    clases_filtradas = []

    #print(area)
    for i, mask in enumerate(lista_masks.xy):
        poligono = Polygon(mask)
        if not poligono.is_valid:
            poligono = poligono.buffer(0) # Esto puede ayudar a "limpiar" el polígono

        #print(poligono)
        if area.contains(poligono):
            instancias_pol.append(poligono)
            clases_filtradas.append(clases[i].item())
        elif poligono.intersects(area):
            #print(poligono)
            # Intersección para recortar la parte dentro del área
            poligono_recortado = poligono.intersection(area)
            # Verificar si el resultado es un Polygon o un MultiPolygon
            if isinstance(poligono_recortado, Polygon):
                instancias_pol.append(poligono_recortado)
                clases_filtradas.append(clases[i].item())
            elif isinstance(poligono_recortado, MultiPolygon):
                # Si es un MultiPolygon, puedes decidir si iterar sobre sus componentes
                for pol in poligono_recortado.geoms:
                    instancias_pol.append(pol)
                    clases_filtradas.append(clases[i].item())

    # # Lista de polígonos (cada polígono definido por una lista de puntos)
    # for elemento in lista_masks.xy:
    #     instancias.append(Polygon(elemento))
    # # Filtrar y recortar los polígonos
    # instancias_pol = []
    # for i, instancia in enumerate(instancias):
    #     if not instancia.is_valid:
    #         # Intenta arreglar la geometría
    #         instancia = instancia.buffer(0)  
    #     if area.contains(instancia):
    #         instancias_pol.append(instancia)
    #         clases_filtradas.append(clases[i].item())
    #     elif instancia.intersects(area):
    #         # Intersección para recortar la parte dentro del área
    #         instancia_recortada = instancia.intersection(area)
    #         # Verificar si el resultado es un Polygon o un MultiPolygon
    #         if isinstance(instancia_recortada, Polygon):
    #             instancias_pol.append(instancia_recortada)
    #             clases_filtradas.append(clases[i].item())
    #         elif isinstance(instancia_recortada, MultiPolygon):
    #             # Si es un MultiPolygon, puedes decidir si iterar sobre sus componentes
    #             for pol in instancia_recortada.geoms:
    #                 instancias_pol.append(pol)
    #                 clases_filtradas.append(clases[i].item())
    # Convertir la conleccion de polígonos en un numpy array [[[x1_1,y1_1],[x1_2,y1_2],..],[[x2_1,y2_1],[x2_2,y2_2],..],...]
    instancias_filtradas = [
        np.array([[round(x, 2), round(y, 2)] for (x, y) in polygon.exterior.coords]) 
        for polygon in instancias_pol
    ]
    # instancias_filtradas = [np.round(np.array(polygon.exterior.coords), 2) for polygon in instancias_pol]
    # del instancias
    return instancias_filtradas, instancias_pol, clases_filtradas



# Aplica IPM para hallar la vista de pájaro a la imágen y a las máscaras (instancias) 
def conversion_vista_pajaro(image, instancias, matrix, width_out, height_out):
    """
    Convierte tanto la imagen original como las instancias de la segmentación por yolo en vista de pájaro (otro plano)
    a través de la multiplicación de una matriz de homografía

    Parámetros:
    image: Imagen original.
    instancias: Lista de intancias del resultado de segmentacion.
    matrix (3x3): Matriz de homografía obtenida a través de Inverse Perspective Mapping (IPM).
    width_out (int): Ancho de la imagen de salida
    height_out (int): Altura de la imagen de salida

    Retorna:
    image_vp: imagen en vista de pájaro.
    instancias_vp: Lista de puntos por instancia en el nuevo plano.
    """
    # Transformar la imagen original a vista de pájaro

    rospy.logwarn(f"El ancho es: {width_out}, Y el alto es: {height_out}")

    image_vp = cv2.warpPerspective(image, matrix, (width_out, height_out))
    image_vp = np.uint8(image_vp)

    # Transformar los puntos de los polígonos o instancias a vista de pájaro
    instancias_vp = list()
    # for instance in instancias:
    #     #instance = instance.astype(int)
    #     #print(instance)
    #     puntos_transformados = cv2.perspectiveTransform(instance.reshape(-1, 1, 2), matrix)
    #     # Convertir el resultado a un formato más fácil de leer
    #     puntos_transformados = puntos_transformados.reshape(-1, 2) #.astype(int)
    #     instancias_vp.append(puntos_transformados)
    #     del puntos_transformados
    instancias_vp = [
        cv2.perspectiveTransform(instance.reshape(-1, 1, 2), matrix).reshape(-1, 2)
        for instance in instancias
    ]
    return image_vp, instancias_vp



def division_subareas(instancias_IPM, clases, altura_division, umbral_agrupacion):
    # Crear una lista para almacenar las subáreas
    subareas, cls_sub = [], []
    
    for i, instance in enumerate(instancias_IPM):
        subareas_pol = []
        # Definir cada instancia como un polígono
        poligono = Polygon(instance)
        
        # Obtener los límites del polígono
        min_y, max_y = poligono.bounds[1], poligono.bounds[3]
        
        # Iterar desde el límite inferior hasta el superior
        current_y = min_y
        #print("min_y:", min_y, "; max_y:", max_y)
        # if max_y - min_y > altura _division + umbral_agrupacion:
        while altura_division + umbral_agrupacion < max_y - current_y:
            # Crear una línea horizontal para dividir el polígono
            linea_horizontal = LineString([(poligono.bounds[0], current_y + altura_division), 
                                           (poligono.bounds[2], current_y + altura_division)])
            
            # Dividir el polígono: la que divide es la línea creada
            poligono_dividido = split(poligono, linea_horizontal)
            
            print("Número de polígonos:", len(poligono_dividido.geoms))
            
            # Actualizar el polígono y almacenar el corte
            subareas_pol.append(poligono_dividido.geoms[0])
            poligono = poligono_dividido.geoms[1]
            #current_y += altura_division # poligono.bounds[1] #altura_division
            current_y = poligono.bounds[1]
            max_y = poligono.bounds[3]
            
        if max_y - current_y >= umbral_agrupacion and max_y - current_y <= umbral_agrupacion + altura_division:
            subareas_pol.append(poligono)
        subareas.extend(subareas_pol)
        ## Añadir TANTOS elementos de la clase de la instancia como el tamaño de subareas_pol
        cls_sub.extend([clases[i]] * len(subareas_pol))
        
    return subareas, cls_sub
import numpy as np
import tf.transformations as tft

cos = np.cos
sin = np.sin
pi = np.pi


def Trotx(ang):
    """ Transformación homogénea que representa rotación en x
    """
    T = np.array([[1., 0., 0.,0.],
                  [0., cos(ang), -sin(ang), 0.],
                  [0., sin(ang), cos(ang), 0.],
                  [0., 0., 0., 1.]])
    return T

def Troty(ang):
    """" Transformación homogénea que representa rotación en y
    """
    T = np.array([[cos(ang), 0., sin(ang), 0.],
                  [0., 1., 0., 0.],
                  [-sin(ang), 0., cos(ang), 0.],
                  [0., 0., 0., 1.]])	
    return T

def Trotz(ang):
    """ Transformación homogénea que representa rotación en z
    """   
    T = np.array([[cos(ang), -sin(ang), 0., 0.],
                  [sin(ang), cos(ang), 0., 0.],
                  [0., 0., 1., 0.],
                  [0., 0., 0., 1.]])
    return T

def Trasl(x, y, z):
    """ Transformación homogénea que representa traslación pura
    """
    T = np.array([[1,0,0,x],
                  [0,1,0,y],
                  [0,0,1,z],
                  [0,0,0,1]])
    return T

def T_scale(s_x=1, s_y=1, s_z=1):
    """
    Retorna la matriz de transformación homogénea para un escalado en los ejes X e Y.
    
    :param s_x: Escala en el eje X.
    :param s_y: Escala en el eje Y.
    :return: Matriz de transformación homogénea 4x4.
    """
    matriz = np.array([[s_x, 0,   0, 0],
                       [0,  s_y,  0, 0],
                       [0,   0, s_z, 0],
                       [0,   0,   0, 1]])
    return matriz

def skew(w):
    """ Matriz antisimétrica (skew) a partir de un vector
    """
    S = np.array([[0., -w[2], w[1]],
                  [w[2], 0., -w[0]],
                  [-w[1], w[0], 0.]])
    return S

def Trasl2D(x, y):
    """ Transformaci´on homogenea en 2D, ejemplo de uso en imágenes
    """
    T = np.array([[1,0,x],
                  [0,1,y],
                  [0,0,1]])
    return T


def pose_to_homogeneous_matrix(pose):
    """
    Convierte una Pose en una matriz homogénea 4x4.

    :param pose_stamped: geometry_msgs.msg.PoseStamped
    :return: Matriz homogénea 4x4 (numpy array)
    """
    # Extraer posición y orientación
    position = pose.position
    orientation = pose.orientation

    # Convertir orientación (quaternion) a matriz de rotación
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    homogeneous_matrix = tft.quaternion_matrix(quaternion)
    # Agregar la traslación
    homogeneous_matrix[:3, 3] = np.array([position.x, position.y, position.z])
    print("La matriz de transformacion es", homogeneous_matrix)
    return homogeneous_matrix
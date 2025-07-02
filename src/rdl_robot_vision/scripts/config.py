# Elección de cámara True si es la ubicada en derecha, False si es la izquierda
RightCamera = False

# Tamaño obtenido despues de descomprimir imagen
frameWidth = 640 # 1920
frameHeight = 480 # 1080

# Translación del origen para tomar en cuenta la parte de la imagen que se realizó la predicción
deltaX = 0  
deltaY = 0.45*frameHeight

# Tamaño de imagen en vista de pájaro
width_vp = 450
height_vp = 450

# Parámetros intrínsecos de la cámara Brio
f = 455.165117#510.165117
#a = 1.00180041044
a = 1.1
fx = f # 467.165117 # 601.365 # 934.165117
fy = a*f #475.486259 # 603.166 # 932.486259
cx = 701.91178# 302.91178 # 6511.657
cy = 303.944762 # 242.244762 #172.8540

# Parametros extrínsecos de la cámara
cameraHeight = 0.5 # Altura de la cámara con respecto al z=0, calle o base_footprint (considera que puede faltar la altura en la misma camara)
cameraX = 0.07 # Translación en x del base_footprint a la cámara
if RightCamera:
    cameraY = -0.07
else:
    cameraY = 0.07

# Traslación del punto {vp} con respecto a {B}
trasX = 5 # 20 # a probar y buscar
trasY = 1.9 # 5 # a probar y buscar

# Escalamiento de la imagen vista de pájaro
scaleFactorX = 0.009 # 0.02  # Ejemplo: 0.05 metros por píxel
scaleFactorY = 0.009 # 0.02  # Ejemplo: 0.05 metros por píxel
scaleFactorZ = 0.009 # 0.02  # En teoría no influye

# Definir el tamaño de las divisiones verticales
alturaDivision = 22
umbralAgrupacion = 8

# Umbral de área preservada en línea
umbralPreservacion = 0.3

# Es decir que porcentaje de la línea se encuentra preservada, 
# un valor cercano a uno significa, que la marca esta casi intacta
# mientras que valores mas bajos significan que la mayor parte esta desgastada

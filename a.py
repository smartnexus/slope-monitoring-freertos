import math

def calcular_angulo_z(aceleracion_x, aceleracion_y, aceleracion_z):
    angulo_z = math.atan2(math.sqrt(aceleracion_x**2 + aceleracion_y**2), aceleracion_z)
    return math.degrees(angulo_z)

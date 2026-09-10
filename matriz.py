"""Funciones para identificar la matriz de colores.

Este archivo contiene la lectura estática por votación y la lógica que asigna
un número de matriz según los colores observados por el sensor.
"""

from pybricks.parameters import Color
from pybricks.tools import wait


# -----------------------------------------------------------------------------
# _realizar_lectura_estatica
# Detiene el robot, toma varias lecturas del sensor y devuelve el color con
# mayor cantidad de votos si alcanza el nivel mínimo de confianza.
# -----------------------------------------------------------------------------
def _realizar_lectura_estatica(
    self,
    cantidad_lecturas=12,
    espera_inicial_ms=100,
    intervalo_lecturas_ms=20,
    votos_minimos=4
):
    self.frenar()
    wait(espera_inicial_ms)

    # Conteo directo: evita crear una lista y hacer .count() repetidamente.
    conteos = {
        Color.GREEN: 0,
        Color.YELLOW: 0,
        Color.BLUE: 0,
        Color.RED: 0,
        Color.WHITE: 0
    }

    lecturas_validas = 0
    for _ in range(cantidad_lecturas):
        color = self.seguidor.color()
        if color in conteos:
            conteos[color] += 1
            lecturas_validas += 1
        wait(intervalo_lecturas_ms)

    if lecturas_validas == 0:
        return None

    color_ganador = max(conteos, key=conteos.get)

    if conteos[color_ganador] < votos_minimos:
        return None

    return color_ganador


# -----------------------------------------------------------------------------
# escanear_matriz
# Lee únicamente la fila que el robot tiene frente al sensor:
#
#     |  1 |  4 |  7 | 10 |
#     |  2 |  5 |  8 | 11 |  <- fila de detección
#     |  3 |  6 |  9 | 12 |
#
# El robot entra por el lado del bloque 11, por lo que la primera lectura es
# 11. El orden de las cinco matrices proporcionadas es:
#
#   Matriz 1: 11=VERDE,    8=VERDE
#   Matriz 2: 11=AMARILLO
#   Matriz 3: 11=BLANCO
#   Matriz 4: 11=VERDE,    8=AMARILLO
#   Matriz 5: 11=AZUL
#
# Por eso VERDE no puede decidirse con una sola lectura: se avanza al bloque 8
# y se toma la segunda lectura. Cualquier combinación distinta se rechaza para
# no iniciar un recorrido equivocado por una lectura errónea.
# -----------------------------------------------------------------------------
def escanear_matriz(self):
    primer_color = self._realizar_lectura_estatica()
    if primer_color is None:
        print("No se detecto un color de matriz valido.")
        return None

    if primer_color == Color.GREEN:
        # Mantener este avance: lleva el sensor de la posición 11 a la 8.
        self.avanzar_recto(
            distancia_cm=4,
            velocidad_max=300,
            perfil="rapido"
        )
        segundo_color = self._realizar_lectura_estatica()

        if segundo_color == Color.GREEN:
            matriz_detectada = 1
        elif segundo_color == Color.YELLOW:
            matriz_detectada = 4
        else:
            matriz_detectada = None
            print("Segunda lectura invalida para verde:", segundo_color)

    elif primer_color == Color.YELLOW:
        matriz_detectada = 2
    elif primer_color == Color.WHITE:
        matriz_detectada = 3
    elif primer_color == Color.BLUE:
        matriz_detectada = 5
    else:
        matriz_detectada = None

    self.matriz_detectada = matriz_detectada
    print("Matriz detectada:", matriz_detectada)
    return matriz_detectada


def dejar_bloques_matriz(robot):
    robot.seguir_linea(
        sensor_color=robot.seguidor,
        velocidad_max=65,
        distancia_cm=15,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=140,
        kp=1.25,
        kd=2.7,
        k_freno=0.16,
        correccion_max=100,
        objetivo_reflexion=27,
        captura_inicial=True,
        tiempo_captura_ms=280,
        potencia_captura=60,
        kp_captura=2.5,
        perfil_salida="encadenado"
    )
    robot.mover_garra_principal(900, 230, apretar=False, duty_cierre=60)
    robot.mover_garra_delantera(230)

    robot.avanzar_recto(
        distancia_cm=-14,
        velocidad_max=400,
        perfil="seguro"
    )

    robot.mover_garra_delantera(270)
    robot.seguir_linea(
        sensor_color=robot.seguidor,
        velocidad_max=100,
        distancia_cm=13,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=140,
        kp=1.25,
        kd=2.7,
        k_freno=0.16,
        correccion_max=100,
        objetivo_reflexion=27,
        captura_inicial=True,
        tiempo_captura_ms=280,
        potencia_captura=60,
        kp_captura=2.5,
        perfil_salida="encadenado"
    )
    robot.mover_garra_principal(
        500,
        esperar=False,
        potencia_apriete=150,
        apretar=True
    )
    robot.mover_garra_delantera(100)
    robot.seguir_linea_hasta_color(
        color_objetivo=Color.BLUE,
        velocidad_max=100,
        lado="derecha"
    )

    wait(400)
    robot.girar_corto(-10)
    robot.avanzar_recto(
        distancia_cm=12.5,
        velocidad_max=650,
        perfil="encadenado"
    )

    robot.mover_garra_delantera(220)
    robot.mover_garra_rapida(130)
    robot.avanzar_recto(
        distancia_cm=-0.6,
        velocidad_max=650,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    robot.mover_garra_delantera(290)
    robot.avanzar_recto(
        distancia_cm=1.3,
        velocidad_max=650,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )

    for _ in range(3):
        robot.girar_corto(9, potencia_max=70, potencia_min=40)
        robot.girar_corto(-9, potencia_max=70, potencia_min=40)

    robot.avanzar_recto(
        distancia_cm=-1.3,
        velocidad_max=500,
        zona_rampa_cm=0.5,
        perfil="seguro"
    )
    robot.mover_garra_delantera(100)
    robot.avanzar_recto(
        distancia_cm=-17,
        velocidad_max=500,
        perfil="seguro"
    )
    robot.girar_hasta_negro("derecha", potencia=75, potencia_correccion=32)


def dejar_bloques_matriz2(robot):
    """Secuencia correspondiente al recorrido auxiliar de matriz 2."""
    robot.avanzar_cruzando_lineas(
        cruces_objetivo=1,
        velocidad=900,
        escape_inicial_cm=8,
        retraso_freno_ms=90
    )
    robot.girar(
        95,
        potencia_max=85,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )
    robot.seguir_linea(
        sensor_color=robot.seguidor,
        velocidad_max=50,
        distancia_cm=7,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=140,
        kp=1.25,
        kd=2.7,
        k_freno=0.16,
        correccion_max=100,
        objetivo_reflexion=27,
        captura_inicial=True,
        tiempo_captura_ms=280,
        potencia_captura=60,
        kp_captura=2.5,
        perfil_salida="encadenado"
    )
    robot.mover_garra_principal(900, 230, apretar=False, duty_cierre=60)
    robot.mover_garra_delantera(230)
    robot.avanzar_recto(distancia_cm=-17, velocidad_max=400, perfil="seguro")
    robot.mover_garra_delantera(270)
    robot.seguir_linea(
        sensor_color=robot.seguidor,
        velocidad_max=100,
        distancia_cm=16,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=140,
        kp=1.25,
        kd=2.7,
        k_freno=0.16,
        correccion_max=100,
        objetivo_reflexion=27,
        captura_inicial=True,
        tiempo_captura_ms=280,
        potencia_captura=60,
        kp_captura=2.5,
        perfil_salida="encadenado"
    )
    robot.mover_garra_principal(
        500,
        esperar=False,
        potencia_apriete=150,
        apretar=True
    )
    robot.mover_garra_delantera(100)
    robot.seguir_linea_hasta_color(
        color_objetivo=Color.BLUE,
        velocidad_max=100,
        lado="derecha"
    )

    wait(200)
    robot.girar_corto(-11.5)
    robot.avanzar_recto(
        distancia_cm=3,
        velocidad_max=650,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    robot.mover_garra_delantera(220)
    robot.mover_garra_rapida(130)
    robot.avanzar_recto(
        distancia_cm=-0.6,
        velocidad_max=650,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    robot.mover_garra_delantera(290)
    robot.avanzar_recto(
        distancia_cm=1.8,
        velocidad_max=750,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    for _ in range(3):
        robot.girar_corto(9, potencia_max=70, potencia_min=40)
        robot.girar_corto(-9, potencia_max=70, potencia_min=40)

    robot.avanzar_recto(
        distancia_cm=-1,
        velocidad_max=900,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    robot.mover_garra_delantera(0)
    robot.avanzar_recto(
        distancia_cm=-29,
        velocidad_max=900,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    robot.girar(
        182,
        potencia_max=85,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )
    robot.avanzar_recto(
        distancia_cm=-21,
        velocidad_max=900,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )

def dejar_bloques_matriz3(robot, distancia_entrada=0):
    robot.mover_garra_delantera(80)
    robot.avanzar_recto(-8)
    robot.mover_garra_principal(100, grados=180, esperar=False)
    robot.mover_garra_delantera(275)
    robot.seguir_linea(
        sensor_color=robot.seguidor,
        velocidad_max=100,
        distancia_cm=15,
        lado="derecha",
        tiempo_acomodo_ms=140,
        tiempo_aceleracion_ms=140,
        kp=1.25,
        kd=2.7,
        k_freno=0.16,
        correccion_max=100,
        objetivo_reflexion=27,
        captura_inicial=True,
        tiempo_captura_ms=280,
        potencia_captura=60,
        kp_captura=2.5,
        perfil_salida="encadenado"
    )
    robot.mover_garra_principal(
        300,
        grados=50,
        esperar=False,
        potencia_apriete=180,
        apretar=True
    )
    robot.mover_garra_delantera(100)
    robot.seguir_linea_hasta_color(
        color_objetivo=Color.BLUE,
        velocidad_max=100,
        lado="derecha"
    )
    wait(400)
    robot.girar_corto(-10.5)
    robot.avanzar_recto(
        distancia_cm=distancia_entrada,
        velocidad_max=650,
        perfil="encadenado"
    )
    robot.mover_garra_delantera(220)
    robot.mover_garra_rapida(125)
    robot.avanzar_recto(
        distancia_cm=-0.6,
        velocidad_max=650,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    robot.mover_garra_delantera(290)
    robot.avanzar_recto(
        distancia_cm=2,
        velocidad_max=650,
        zona_rampa_cm=0.1,
        perfil="encadenado"
    )
    for _ in range(4):
        robot.girar_corto(8, potencia_max=75, potencia_min=45)
        robot.girar_corto(-8, potencia_max=75, potencia_min=45)
    robot.avanzar_recto(
        distancia_cm=-1,
        velocidad_max=500,
        zona_rampa_cm=0.5,
        perfil="seguro"
    )
    robot.mover_garra_delantera(190)
    robot.avanzar_recto(
        distancia_cm=-18,
        velocidad_max=500,
        perfil="seguro"
    )
    robot.girar(
        180,
        potencia_max=90,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )

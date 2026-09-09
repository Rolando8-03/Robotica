"""Configuración principal del robot.

Este archivo crea el hardware y reúne en una sola clase las funciones activas
que están organizadas en otros módulos. El recorrido puede seguir utilizando
la misma interfaz: robot.girar(...), robot.seguir_linea(...), etc.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port
from pybricks.robotics import DriveBase

import utilidades
import navegacion
import seguimiento_linea
import deteccion_color
import actuadores
import matriz

class Base:
    # -------------------------------------------------------------------------
    # __init__
    # Configura el Hub, los motores de tracción, el motor de torque, el sensor
    # de color y las medidas físicas necesarias para calcular movimientos.
    # -------------------------------------------------------------------------
    def __init__(self):
        self.Hub = PrimeHub()

        # Motores de tracción.
        # Las direcciones compensan la orientación física opuesta de ambos motores.
        self.motor_derecho = Motor(Port.A, Direction.CLOCKWISE)
        self.motor_izquierdo = Motor(Port.E, Direction.COUNTERCLOCKWISE)

        # Motor utilizado para levantar, tomar o soltar los cementos y la pala.
        self.motor_torque = Motor(Port.D)

        #Motor de la garra principal
        self.motor_garra = Motor(Port.F)

        #Motor de la garra delantera
        self.motor_garra_delantera = Motor(Port.B)

        # Sensor utilizado para seguir líneas, detectar colores y leer la matriz.
        self.seguidor = ColorSensor(Port.C)

        # Medidas físicas de las ruedas.
        self.diametro_rueda = 56
        self.circunferencia = self.diametro_rueda * 3.14159
        self.grados_por_mm = 360 / self.circunferencia

        # Control conjunto de los motores de tracción.
        self.drive_base = DriveBase(
            self.motor_izquierdo,
            self.motor_derecho,
            wheel_diameter=self.diametro_rueda,
            axle_track=205
        )

    # ========================= UTILIDADES INTERNAS =========================
    # Detiene ambos motores de tracción.
    frenar = utilidades.frenar

    # Mantiene un valor dentro de un mínimo y un máximo.
    limitar = utilidades.limitar

    # Reinicia los ángulos medidos por los motores de tracción.
    reset_motores = utilidades.reset_motores

    # Calcula el promedio de grados recorridos por las dos ruedas.
    distancia_promedio_grados = utilidades.distancia_promedio_grados

    # Calcula el error angular más corto entre dos rumbos.
    _error_angular = utilidades._error_angular

    # Prepara sensores, motores y mediciones antes de un movimiento.
    preparar_movimiento = utilidades.preparar_movimiento

    # Aplica el frenado y la pausa correspondientes al terminar un movimiento.
    terminar_movimiento = utilidades.terminar_movimiento

    # ============================== NAVEGACIÓN ==============================
    # Avanza o retrocede una distancia manteniendo el rumbo con el IMU.
    avanzar_recto = navegacion.avanzar_recto

    # Avanza recto manteniendo el rumbo hasta dejar de detectar negro.
    avanzar_hasta_salir_negro = navegacion.avanzar_hasta_salir_negro

    avanzar_con_torque = navegacion.avanzar_con_torque

    # Realiza un giro sobre el centro del robot utilizando el IMU.
    girar = navegacion.girar

    # Realiza un giro curvo haciendo que una rueda recorra más que la otra.
    giro_de_arco = navegacion.giro_de_arco

    # Función interna compartida por giro_derecha y giro_izquierda.
    _giro_un_motor = navegacion._giro_un_motor

    # Gira utilizando únicamente la rueda derecha.
    giro_derecha = navegacion.giro_derecha

    # Gira utilizando únicamente la rueda izquierda.
    giro_izquierda = navegacion.giro_izquierda

    # ========================= SEGUIMIENTO DE LÍNEA =========================
    # Sigue el borde de una línea durante una distancia determinada.
    seguir_linea = seguimiento_linea.seguir_linea

    # Sigue el borde de una línea hasta detectar un color objetivo.
    seguir_linea_hasta_color = seguimiento_linea.seguir_linea_hasta_color

    seguir_linea_y_mover_torque = seguimiento_linea.seguir_linea_y_mover_torque

    # ====================== DETECCIÓN DE COLOR Y CRUCES =====================
    # Avanza recto hasta encontrar un color o una cantidad de cruces.
    avanzar_hasta_color = deteccion_color.avanzar_hasta_color

    # Avanza recto contando líneas negras atravesadas.
    avanzar_cruzando_lineas = deteccion_color.avanzar_cruzando_lineas

    _es_color = deteccion_color._es_color
    
    # Combina un avance por distancia con una búsqueda posterior por color.
    avanzar_hibrido = deteccion_color.avanzar_hibrido

    # Comprueba continuamente la reflexión del sensor de color.
    comprobar_reflexion_negro = deteccion_color.comprobar_reflexion_negro

    # ============================== ACTUADORES ==============================
    # Mueve el mecanismo de torque para tomar o soltar objetos.
    mover_torque = actuadores.mover_torque

    #Mueve la garra delantera
    mover_garra_delantera = actuadores.mover_garra_delantera

    #Abre o cierra la garra principal
    mover_garra_principal = actuadores.mover_garra_principal

    mover_garra_rapida = actuadores.mover_garra_rapida


    # ================================ MATRIZ ================================
    # Función interna que determina un color mediante varias lecturas estáticas.
    _realizar_lectura_estatica = matriz._realizar_lectura_estatica

    # Lee los colores de la matriz y determina su número.
    escanear_matriz = matriz.escanear_matriz

    # Realiza giros pequeños y rápidos sin bloquearse al final.
    girar_corto = navegacion.girar_corto

    #Función para dejar los bloques en la matriz.
    dejar_bloques_matriz = matriz.dejar_bloques_matriz

    # ============================== NAVEGACIÓN ==============================
    # (Tus otras funciones aquí...)
    
       # Gira hacia un rumbo absoluto.
    girar_a_rumbo = navegacion.girar_a_rumbo

    girar_hasta_negro = navegacion.girar_hasta_negro

    # Establece la orientación actual como referencia.
    establecer_norte = navegacion.establecer_norte
    

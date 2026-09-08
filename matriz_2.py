"""Recorrido adaptado para la matriz 2.
Todas las llamadas usan las funciones del proyecto organizado.
"""

from control_drivebase import Base
from pybricks.parameters import Color
from pybricks.tools import wait
import gc

from matriz import dejar_bloques_matriz
from matriz import dejar_bloques_matriz2
from matriz import dejar_bloques_matriz3


def ejecutar_matriz_2(robot):

    """Ejecuta la secuencia de navegación y manipulación para la matriz 2."""

    
    print("Voltaje:", robot.Hub.battery.voltage(), "mV")
    print("Ejecutando recorrido de matriz 2")

    # Limpieza inicial antes de comenzar el recorrido
    gc.collect()

    # ==========================================
    # PRIMERA PARTE DE LA MATRIZ: BLOQUES AZULES
    # ==========================================

    robot.motor_garra.reset_angle(0)
    robot.motor_garra_delantera.reset_angle(0)
    
    robot.avanzar_recto(distancia_cm=-9, velocidad_max=900)
    robot.girar(
        90,
        potencia_max=65,
        potencia_min=45,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )
    
    wait(200)
    robot.avanzar_cruzando_lineas(
        cruces_objetivo=2,
        velocidad=900,
        escape_inicial_cm=8,
        retraso_freno_ms=91
    )

    gc.collect()
    wait(200)

    robot.girar(
        -90,
        potencia_max=65,
        potencia_min=45,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )

    robot.mover_garra_principal(900, 300, apretar=False, duty_cierre=100)

    robot.avanzar_hasta_salir_negro(
        velocidad_max=900,
        velocidad_min=200,
        objetivo_reflexion=15,
        lecturas_salida=4
    )

    robot.avanzar_recto(distancia_cm=4.3, velocidad_max=900, perfil="encadenado")
    robot.mover_garra_delantera(290)

    robot.avanzar_recto(distancia_cm=-20.7, velocidad_max=900, perfil="seguro")
    wait(200)

    robot.girar(
        -90,
        potencia_max=75,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )

    robot.seguir_linea(
        sensor_color=robot.seguidor, 
        velocidad_max=60, 
        distancia_cm=8, 
        lado="izquierda", 
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

    gc.collect()
    wait(200)

    # ============================================
    # PRIMERA PARTE DE LA MATRIZ: BLOQUES AMARILLOS
    # ============================================

    robot.girar(
        90,
        potencia_max=65,
        potencia_min=45,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )


    robot.mover_garra_delantera(255)
    robot.mover_garra_principal(900, 140, apretar=False, duty_cierre=100)
    wait(200)

    robot.avanzar_recto(distancia_cm=12.5, velocidad_max=750, perfil="encadenado")
    robot.mover_garra_delantera(260)

    robot.mover_garra_principal(
        300,
        esperar=False,
        potencia_apriete=100,
        apretar=True
    )

    robot.avanzar_recto(distancia_cm=-13, velocidad_max=550, perfil="seguro")
    wait(200)

    robot.girar(
        90,
        potencia_max=65,
        potencia_min=45,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )

    robot.seguir_linea(
        sensor_color=robot.seguidor, 
        velocidad_max=90, 
        distancia_cm=12, 
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

    gc.collect()
    wait(200)

    robot.girar(
        90,
        potencia_max=90,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )

    dejar_bloques_matriz(robot)

    # Liberar memoria después de la rutina de descarga
    gc.collect()

    # ==========================================
    # SEGUNDA PARTE DE LA MATRIZ: RECOLECCIÓN Y DESCARGA
    # ==========================================

    robot.mover_garra_principal(900, 250, apretar=False, duty_cierre=60)

    robot.seguir_linea(
        sensor_color=robot.seguidor, 
        velocidad_max=80, 
        distancia_cm=25, 
        lado="izquierda", 
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

    gc.collect()
    wait(300)

    robot.girar(
        -45,
        potencia_max=75,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )
    robot.avanzar_recto(distancia_cm=4.5, velocidad_max=500, perfil="seguro")
    wait(200)
    robot.girar(
        45,
        potencia_max=80,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )

    robot.seguir_linea(
        sensor_color=robot.seguidor, 
        velocidad_max=60, 
        distancia_cm=8, 
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

    gc.collect()

    robot.avanzar_recto(distancia_cm=20, velocidad_max=350, perfil="seguro")

    robot.mover_garra_delantera(300)
    robot.avanzar_recto(distancia_cm=-25.5, velocidad_max=600, perfil="seguro")
    robot.girar(
        -90,
        potencia_max=90,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )

    robot.seguir_linea(
        sensor_color=robot.seguidor, 
        velocidad_max=70, 
        distancia_cm=13, 
        lado="izquierda", 
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

    gc.collect()
    wait(200)

    robot.girar(
        90,
        potencia_max=90,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )
    robot.mover_garra_delantera(265)
    robot.mover_garra_principal(900, 150, apretar=False, duty_cierre=60)
    wait(300)

    robot.avanzar_recto(distancia_cm=12.5, velocidad_max=750, perfil="encadenado")
    robot.mover_garra_delantera(275)

    robot.mover_garra_principal(
        300,
        esperar=False,
        potencia_apriete=80,
        apretar=True
    )
    robot.mover_garra_delantera(190)

    gc.collect()

    robot.avanzar_recto(distancia_cm=16, velocidad_max=350, perfil="seguro")
    robot.mover_garra_delantera(260)

    robot.avanzar_recto(distancia_cm=-38.5, velocidad_max=600, perfil="seguro")

    gc.collect()
    wait(300)

    robot.girar(
        89.9,
        potencia_max=70,
        potencia_min=35,
        kp_base=5.0,
        tolerancia_fin=1.0,
        perfil="encadenado"
    )
    robot.avanzar_recto(distancia_cm=-18, velocidad_max=600, perfil="seguro")

    robot.mover_torque(
        grados_torque=-175,
        velocidad_torque=900,
        esperar=False
    )

    gc.collect()

    dejar_bloques_matriz2(robot)

    # Limpieza final
    gc.collect()


if __name__ == "__main__":
    robot = Base()
    ejecutar_matriz_2(robot)

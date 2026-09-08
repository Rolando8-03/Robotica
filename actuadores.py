"""Funciones de los mecanismos y actuadores del robot.

Este archivo controla el motor de torque, la garra principal y la garra
delantera. No contiene movimientos de las ruedas ni lectura de sensores.
"""

from pybricks.parameters import Stop
from pybricks.tools import wait, StopWatch


# -----------------------------------------------------------------------------
# mover_torque
# Mueve el mecanismo de torque una cantidad determinada de grados.
# Puede ejecutarse de forma bloqueante o al mismo tiempo que otra acción.
# -----------------------------------------------------------------------------
def mover_torque(
    self,
    grados_torque,
    velocidad_torque=180,
    esperar=True,
    modo_final=Stop.HOLD,
    retraso_inicial_ms=0
):
    if retraso_inicial_ms > 0:
        wait(retraso_inicial_ms)

    if grados_torque == 0:
        return

    self.motor_torque.run_angle(
        velocidad_torque,
        grados_torque,
        then=modo_final,
        wait=esperar
    )


#Funcion en desarollo JAJSHAJASHJH
def mover_garra_principal(
    self,
    velocidad,
    grados=0,
    esperar=True,
    potencia_apriete=150,
    tiempo_apriete_ms=120,
    apretar=False,
    modo_soltar=None,
    duty_cierre=100
):
    # ==========================================
    # 1. CONTROL MANUAL
    # ==========================================
    if modo_soltar == "hold":
        self.motor_garra.hold()
        return
    elif modo_soltar == "stop":
        self.motor_garra.stop()
        return
    elif modo_soltar == "brake":
        self.motor_garra.brake()
        return

    velocidad = abs(velocidad)
    if velocidad <= 0:
        velocidad = 200

    # ==========================================
    # 2. APRIETE (agarrar un objeto de tamaño variable)
    #
    # Cuando apretar=True, el ángulo objetivo (grados) se ignora:
    # la garra cierra hasta encontrar resistencia real (el objeto),
    # no hasta una posición fija.
    # ==========================================
    if apretar:
        if hasattr(self, "limitar"):
            potencia_apriete = self.limitar(potencia_apriete, -100, 100)
        else:
            potencia_apriete = max(-100, min(100, potencia_apriete))

        self.motor_garra.run_until_stalled(
            -velocidad,
            then=Stop.HOLD,
            duty_limit=abs(potencia_apriete)
        )
        return

    # ==========================================
    # 3. LIMITAR ESCALA 0-300 (solo aplica si NO es apriete)
    # ==========================================
    grados = max(0, min(300, grados))

    # ==========================================
    # 4. MOVER A LA POSICIÓN
    # ==========================================
    if grados == 0:
        self.motor_garra.run_until_stalled(
            -velocidad,
            then=Stop.HOLD,
            duty_limit=duty_cierre
        )
        self.motor_garra.reset_angle(0)
    else:
        posicion_motor = grados
        self.motor_garra.run_target(
            velocidad,
            posicion_motor,
            then=Stop.HOLD,
            wait=esperar
        )

def mover_torque_seguro(
    self,
    grados_torque,
    velocidad_torque=180,
    esperar=True,
    modo_final=Stop.HOLD,
    retraso_inicial_ms=0,
    duty_limit=50,
    detener_en_tope=True
):
    if retraso_inicial_ms > 0:
        wait(retraso_inicial_ms)

    if grados_torque == 0:
        return False

    self.motor_torque.stop()
    wait(10)
    self.motor_torque.reset_angle(0)

    velocidad_real = (
        abs(velocidad_torque)
        if grados_torque > 0
        else -abs(velocidad_torque)
    )

    modo_tope = (
        Stop.HOLD
        if detener_en_tope
        else Stop.COAST
    )

    angulo_final = self.motor_torque.run_until_stalled(
        speed=velocidad_real,
        then=modo_tope,
        duty_limit=duty_limit
    )

    angulo_real = abs(angulo_final)
    angulo_objetivo = abs(grados_torque)

    # Si se detuvo antes del ángulo de referencia, probablemente encontró
    # resistencia antes de completar el recorrido esperado.
    tope_detectado = angulo_real < angulo_objetivo

    if not detener_en_tope:
        if modo_final == Stop.HOLD:
            self.motor_torque.hold()

        elif modo_final == Stop.BRAKE:
            self.motor_torque.brake()

        else:
            self.motor_torque.stop()

    if esperar:
        wait(20)

    return tope_detectado

# -----------------------------------------------------------------------------
# mover_garra_delantera
# Mueve la garra delantera una cantidad determinada de grados.
# Al terminar mantiene la posición usando Stop.HOLD.
# -----------------------------------------------------------------------------
#FUNCION ACTUALIZADA DE MOVER GARRA DELANTERA
def mover_garra_delantera(
    self,
    posicion,
    velocidad=700,
    simultaneo=False,
    modo_final=Stop.HOLD,
    limite_minimo=0,
    limite_maximo=300
):
    """
    Mueve la garra delantera.

    0°   = Garra completamente arriba.
    300° = Garra completamente abajo.

    Cualquier valor fuera del rango será limitado automáticamente.
    """

    # Limitar el rango permitido
    posicion = max(limite_minimo, min(posicion, limite_maximo))

    self.motor_garra_delantera.run_target(
        speed=abs(velocidad),
        target_angle=posicion,
        then=modo_final,
        wait=not simultaneo
    )

    return posicion

def mover_garra_rapida(
    self,
    grados=130,
    potencia=120,
    tiempo_max_ms=1200
):
    if grados <= 0:
        print("mover_garra_rapida: grados <= 0, no se movió")
        return

    potencia = self.limitar(potencia, 10, 100)

    self.motor_garra.stop()
    wait(10)
    self.motor_garra.reset_angle(0)

    cronometro = StopWatch()

    self.motor_garra.dc(potencia)

    while True:
        if abs(self.motor_garra.angle()) >= grados:
            break
        if cronometro.time() >= tiempo_max_ms:
            print(
                "mover_garra_rapida: se agotó el tiempo, llegó a",
                self.motor_garra.angle(),
                "de",
                grados
            )
            break

    self.motor_garra.hold()

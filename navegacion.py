"""Movimientos de navegación.

Contiene desplazamientos rectos, giros sobre el centro, giros en arco y giros con una sola rueda.
Las funciones conservan su lógica y sus argumentos originales.
"""

from pybricks.tools import wait, StopWatch

# -----------------------------------------------------------------------------
# avanzar_recto
# Avanza o retrocede una distancia manteniendo el rumbo con el giroscopio
# y rampas de velocidad.
#
# Opcionalmente puede iniciar un movimiento de torque después de cierto tiempo,
# sin detener el avance.
# -----------------------------------------------------------------------------
def avanzar_recto(
    self,
    distancia_cm,
    velocidad_max=900,
    velocidad_min=100,
    kp_gyro=20.0,
    zona_rampa_cm=8,
    perfil="encadenado",
    rumbo_esperado=None,

    # Torque opcional durante el avance.
    torque_grados=None,
    torque_velocidad=180,
    torque_retraso_ms=0
):
    if distancia_cm == 0:
        return

    self.preparar_movimiento(
        reset_motores=False,
        reset_gyro=False,
        perfil=perfil
    )

    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(90)

    self.drive_base.reset()

    # =========================================================
    # MEMORIA DE RUMBO
    # Si se proporciona un rumbo, utiliza ese valor.
    # De lo contrario, toma el rumbo actual del IMU.
    # =========================================================
    if rumbo_esperado is not None:
        heading_objetivo = rumbo_esperado
    else:
        heading_objetivo = self.Hub.imu.heading()

    # Conversión de centímetros a grados de las ruedas.
    grados_por_cm = (
        360 / (self.circunferencia / 10)
    )

    grados_objetivo = (
        abs(distancia_cm) * grados_por_cm
    )

    rampa_efectiva_cm = min(
        zona_rampa_cm,
        abs(distancia_cm) / 2.0
    )

    grados_rampa = (
        rampa_efectiva_cm * grados_por_cm
    )

    signo = (
        1
        if distancia_cm > 0
        else -1
    )

    cronometro = StopWatch()
    cronometro.reset()

    # Impide que el torque se active más de una vez.
    torque_iniciado = False

    while True:
        # Distancia real recorrida por el DriveBase.
        recorrido = (
            abs(self.drive_base.distance())
            * self.grados_por_mm
        )

        restante = (
            grados_objetivo - recorrido
        )

        # Finalizar al alcanzar la distancia.
        if restante <= 1.5:
            break

        actual_heading = (
            self.Hub.imu.heading()
        )

        error_gyro = self._error_angular(
            heading_objetivo,
            actual_heading
        )

        # Rampa de aceleración.
        if recorrido < grados_rampa:
            proporcion = (
                recorrido / grados_rampa
            )

            vel_base = (
                velocidad_min
                + (
                    velocidad_max
                    - velocidad_min
                ) * proporcion
            )

        # Rampa de desaceleración.
        elif restante < grados_rampa:
            proporcion = (
                restante / grados_rampa
            )

            vel_base = (
                velocidad_min * 0.4
                + (
                    velocidad_max
                    - velocidad_min * 0.4
                ) * proporcion
            )

        else:
            vel_base = velocidad_max

        tiempo_ms = cronometro.time()

        # =====================================================
        # TORQUE RETRASADO DURANTE EL AVANCE
        #
        # Cuando se cumple el tiempo indicado, inicia el torque
        # sin bloquear el movimiento del robot.
        # =====================================================
        if (
            torque_grados is not None
            and not torque_iniciado
            and tiempo_ms >= torque_retraso_ms
        ):
            self.mover_torque(
                grados_torque=torque_grados,
                velocidad_torque=torque_velocidad,
                esperar=False
            )

            torque_iniciado = True

        # Rampa temporal inicial original.
        if tiempo_ms < 150:
            vel = (
                vel_base
                * (tiempo_ms / 150)
            )
        else:
            vel = vel_base

        vel = self.limitar(
            vel,
            25,
            velocidad_max
        )

        correccion_giro = (
            error_gyro * kp_gyro
        )

        self.drive_base.drive(
            vel * signo,
            correccion_giro
        )

    # =======================================================
    # Frenado original de dos fases.
    # =======================================================
    self.drive_base.stop()

    self.motor_izquierdo.brake()
    self.motor_derecho.brake()
    wait(60)

    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)

# -----------------------------------------------------------------------------
# girar
# Realiza un giro sobre el centro con control PD del IMU y detección de posibles bloqueos.
# -----------------------------------------------------------------------------
def girar(
    self,
    angulo_deg,
    potencia_max=85,
    potencia_min=40,
    kp_base=3.5,
    kd_base=5.0,
    tiempo_curva_s_ms=100,
    tolerancia_fin=1.9,
    perfil="encadenado"
):
    """
    Giro con control de giroscopio y detección automática de bloqueos.

    En los giros de 180 grados, el signo de angulo_deg determina
    obligatoriamente la dirección:

        girar(180)  -> una dirección
        girar(-180) -> dirección contraria
    """

    if angulo_deg == 0:
        return

    # Preparar motores.
    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(40)

    self.preparar_movimiento(
        reset_motores=False,
        reset_gyro=True,
        perfil=perfil
    )

    inicio = self.Hub.imu.heading()
    objetivo = inicio + angulo_deg

    # ============================================================
    # CONTROL DE DIRECCIÓN PARA GIROS DE 180°
    #
    # En 180°, ambos sentidos terminan en la misma orientación.
    # Se conserva el signo recibido para obligar la dirección.
    # ============================================================
    es_giro_180 = abs(abs(angulo_deg) - 180) < 0.01
    signo_giro_180 = 1 if angulo_deg > 0 else -1

    error_anterior = self._error_angular(
        objetivo,
        inicio
    )

    # Evita la ambigüedad inicial entre +180 y -180.
    if (
        es_giro_180
        and abs(abs(error_anterior) - 180) < 0.5
    ):
        error_anterior = 180 * signo_giro_180

    derivada_anterior = 0

    cronometro = StopWatch()
    cronometro.reset()

    # ============================================================
    # VARIABLES PARA DETECCIÓN DE BLOQUEO
    # ============================================================
    ultimo_angulo_imu = inicio

    ultimo_angulo_motores = (
        self.motor_izquierdo.angle()
        + self.motor_derecho.angle()
    ) / 2

    contador_sin_cambio = 0
    bloqueo_detectado = False

    while True:
        # ========================================================
        # 1. LECTURA DE SENSORES
        # ========================================================
        actual_imu = self.Hub.imu.heading()

        error = self._error_angular(
            objetivo,
            actual_imu
        )

        # Si el error está exactamente en la zona ambigua de 180°,
        # se obliga a conservar el signo escrito en la llamada.
        if (
            es_giro_180
            and abs(abs(error) - 180) < 0.5
        ):
            error = 180 * signo_giro_180

        # Lectura de motores mediante odometría.
        angulo_motores = (
            self.motor_izquierdo.angle()
            + self.motor_derecho.angle()
        ) / 2

        # ========================================================
        # 2. DETECCIÓN DE BLOQUEO
        # ========================================================
        if abs(error) > 5:
            # Cuánto cambió el ángulo según el IMU.
            delta_imu = abs(
                actual_imu - ultimo_angulo_imu
            )

            # Cuánto avanzaron los motores.
            delta_motores = abs(
                angulo_motores
                - ultimo_angulo_motores
            )

            # Si el IMU no cambia, pero los motores sí se mueven,
            # se considera un posible bloqueo o patinaje.
            if delta_imu < 1.0 and delta_motores > 5.0:
                contador_sin_cambio += 1

                if contador_sin_cambio > 5:
                    bloqueo_detectado = True

                    # Identificar qué rueda está patinando.
                    movimiento_izquierdo = abs(
                        self.motor_izquierdo.angle()
                        - ultimo_angulo_motores
                    )

                    movimiento_derecho = abs(
                        self.motor_derecho.angle()
                        - ultimo_angulo_motores
                    )

                    if movimiento_izquierdo > movimiento_derecho:
                        # La rueda izquierda está patinando.
                        self.motor_izquierdo.dc(-20)
                        wait(20)

                    else:
                        # La rueda derecha está patinando.
                        self.motor_derecho.dc(-20)
                        wait(20)

                    # Reiniciar mediciones y continuar.
                    self.reset_motores()
                    angulo_motores = 0

                    bloqueo_detectado = False
                    contador_sin_cambio = 0

            else:
                contador_sin_cambio = max(
                    0,
                    contador_sin_cambio - 1
                )

            ultimo_angulo_imu = actual_imu
            ultimo_angulo_motores = angulo_motores

        # ========================================================
        # 3. CONTROL NORMAL DEL GIRO
        # ========================================================
        if abs(error) <= tolerancia_fin:
            break

        # Ganancias dinámicas.
        if abs(error) > 25:
            kp_dinamico = kp_base * 1.1
            kd_dinamico = kd_base * 1.1

        else:
            kp_dinamico = kp_base * 1.5
            kd_dinamico = kd_base * 0.3

        derivada_cruda = error - error_anterior

        derivada = (
            derivada_cruda * 0.7
            + derivada_anterior * 0.3
        )

        # Se conserva el cálculo original de tu compañero.
        correccion = (
            error * kp_dinamico
            + derivada * kd_base
        )

        tiempo_transcurrido = cronometro.time()

        # Curva inicial de potencia.
        if tiempo_transcurrido < tiempo_curva_s_ms:
            limite_potencia = (
                potencia_min
                + (potencia_max - potencia_min)
                * (
                    tiempo_transcurrido
                    / tiempo_curva_s_ms
                )
            )
        else:
            limite_potencia = potencia_max

        potencia_final = self.limitar(
            correccion,
            -limite_potencia,
            limite_potencia
        )

        # Si hay bloqueo, aplicar una respuesta más agresiva.
        if bloqueo_detectado:
            potencia_final *= 1.5

            potencia_final = self.limitar(
                potencia_final,
                -limite_potencia,
                limite_potencia
            )

            if abs(potencia_final) < potencia_min * 0.5:
                if potencia_final > 0:
                    potencia_final = potencia_min * 0.5
                else:
                    potencia_final = -potencia_min * 0.5

        # Aplicar potencia opuesta a ambos motores.
        pot_izq = int(potencia_final)
        pot_der = int(-potencia_final)

        self.motor_izquierdo.dc(
            self.limitar(pot_izq, -100, 100)
        )

        self.motor_derecho.dc(
            self.limitar(pot_der, -100, 100)
        )

        error_anterior = error
        derivada_anterior = derivada

        wait(2)

    # Frenado final.
    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)


# -----------------------------------------------------------------------------
# giro_de_arco
# Describe un arco haciendo que cada rueda recorra una trayectoria distinta y corrigiendo con el IMU.
# -----------------------------------------------------------------------------
def giro_de_arco(
    self,
    radio_cm,
    angulo_deg,
    potencia_max=80,
    potencia_min=35,        # Potencia mínima para mantener movimiento suave
    lado="derecha",
    distancia_ruedas_cm=12.3, # Ajusta según tu robot real
    kp_gyro=2.8,            # Ganancia proporcional para corregir el error
    tolerancia_fin=1.5,     # Grados de tolerancia para considerar que el giro terminó
    perfil="encadenado"
):
    """
    Giro en arco con control de giroscopio para compensar resbalones.

    Args:
        radio_cm: Radio del arco en centímetros (positivo = arco a la derecha, 
                negativo = arco a la izquierda)
        angulo_deg: Ángulo a girar en grados (positivo = horario, negativo = antihorario)
        potencia_max: Potencia máxima para el motor externo (0-100)
        potencia_min: Potencia mínima para mantener el movimiento
        lado: "derecha" o "izquierda" - determina qué rueda va por fuera del arco
        distancia_ruedas_cm: Distancia entre las ruedas del robot
        kp_gyro: Ganancia del controlador del giroscopio
        tolerancia_fin: Tolerancia en grados para finalizar el giro
        perfil: Perfil de finalización ("seguro", "encadenado", etc.)
    """
    if angulo_deg == 0 or radio_cm == 0:
        return

    # Preparamos el robot (motores y giroscopio)
    self.preparar_movimiento(reset_motores=False, reset_gyro=True, perfil=perfil)

    # Aseguramos que los motores estén listos
    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)

    # ============================================================
    # PASO 1: CALCULAR LAS VELOCIDADES PARA EL ARCO
    # ============================================================
    pi = 3.1416

    # Distancia desde el centro de giro hasta cada rueda
    radio_interno = abs(radio_cm) - (distancia_ruedas_cm / 2)
    radio_externo = abs(radio_cm) + (distancia_ruedas_cm / 2)

    # Protección contra radio muy pequeño
    if radio_interno <= 0:
        print("⚠️ Radio demasiado pequeño, ajustando...")
        radio_interno = 2  # Valor mínimo seguro

    # Distancia que recorre cada rueda para el ángulo deseado
    distancia_interna = 2 * pi * radio_interno * (abs(angulo_deg) / 360)
    distancia_externa = 2 * pi * radio_externo * (abs(angulo_deg) / 360)

    # Relación de velocidades para que el robot describa el arco perfecto
    relacion = distancia_interna / distancia_externa

    # Ajustamos la potencia para que el motor externo trabaje a la potencia deseada
    potencia_externa = potencia_max
    potencia_interna = potencia_max * relacion

    # ============================================================
    # PASO 2: DETERMINAR QUÉ MOTOR VA MÁS RÁPIDO
    # ============================================================
    signo = 1 if angulo_deg > 0 else -1
    direccion = -1 if radio_cm > 0 else 1  # Si radio positivo = arco a derecha

    if (radio_cm > 0 and lado == "derecha") or (radio_cm < 0 and lado == "izquierda"):
        # El motor externo es el izquierdo (gira a la derecha)
        potencia_izq = potencia_externa * signo
        potencia_der = potencia_interna * signo
        motor_rapido = self.motor_izquierdo
        motor_lento = self.motor_derecho
    else:
        # El motor externo es el derecho (gira a la izquierda)
        potencia_izq = potencia_interna * signo
        potencia_der = potencia_externa * signo
        motor_rapido = self.motor_derecho
        motor_lento = self.motor_izquierdo

    # ============================================================
    # PASO 3: EJECUTAR EL GIRO CON CONTROL DE GIROSCOPIO
    # ============================================================
    heading_inicial = self.Hub.imu.heading()
    heading_objetivo = heading_inicial + angulo_deg

    # Variables de control para suavizar
    error_anterior = self._error_angular(heading_objetivo, heading_inicial)

    # Limitamos la potencia mínima para que el motor lento no se detenga
    potencia_interna = max(potencia_interna, potencia_min * 0.3)
    potencia_externa = max(potencia_externa, potencia_min)

    # Variables para monitoreo
    cronometro = StopWatch()
    cronometro.reset()

    while True:
        # Verificamos el ángulo actual con el giroscopio
        heading_actual = self.Hub.imu.heading()
        error_actual = self._error_angular(heading_objetivo, heading_actual)

        # Si llegamos al objetivo, salimos
        if abs(error_actual) <= tolerancia_fin:
            break

        # ========================================================
        # CONTROL PROPORCIONAL PARA CORREGIR DESVIACIONES
        # ========================================================
        correccion_gyro = error_actual * kp_gyro

        # Limitamos la corrección para que no sea demasiado agresiva
        correccion_gyro = self.limitar(correccion_gyro, -25, 25)

        # Aplicamos la corrección a las potencias base
        pot_izq_final = potencia_izq + correccion_gyro
        pot_der_final = potencia_der - correccion_gyro

        # ========================================================
        # ZONA DE FRENADO SUAVE (Rampa al final)
        # ========================================================
        # Si estamos cerca del objetivo, reducimos la velocidad suavemente
        if abs(error_actual) < 10:
            factor_freno = abs(error_actual) / 10
            pot_izq_final *= factor_freno
            pot_der_final *= factor_freno

            # Aseguramos que no bajemos de la potencia mínima
            pot_izq_final = max(pot_izq_final, potencia_min * signo * 0.5)
            pot_der_final = max(pot_der_final, potencia_min * signo * 0.5)

        # Limitamos las potencias finales
        pot_izq_final = self.limitar(pot_izq_final, -100, 100)
        pot_der_final = self.limitar(pot_der_final, -100, 100)

        # Aplicamos las potencias a los motores
        self.motor_izquierdo.dc(pot_izq_final)
        self.motor_derecho.dc(pot_der_final)

        # Actualizamos variables de control
        error_anterior = error_actual

        # Pequeña pausa para no saturar el bucle
        wait(2)

    # ============================================================
    # PASO 4: FRENADO CONTROLADO
    # ============================================================
    # Frenado en dos fases para evitar rebotes
    self.motor_izquierdo.brake()
    self.motor_derecho.brake()
    wait(40)

    # Hold para mantener la posición
    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)

    # Finalización según el perfil solicitado
    self.terminar_movimiento(perfil=perfil, modo="hold", pausa=None)

# -----------------------------------------------------------------------------
# _giro_un_motor
# Función interna para girar usando una rueda activa mientras la otra permanece frenada.
# -----------------------------------------------------------------------------
def _giro_un_motor(
    self,
    motor_activo,
    motor_fijo,
    angulo_deg,
    sentido_motor,
    velocidad=1000,
    velocidad_min=180,
    zona_freno=22,
    tolerancia=1.2,
    tiempo_max_ms=None,
    perfil="seguro"
):
    if angulo_deg == 0:
        return

    # No es necesario reiniciar el giroscopio.
    # Medimos el giro desde el heading actual.
    self.preparar_movimiento(
        reset_motores=False,
        reset_gyro=False,
        perfil=perfil
    )

    inicio = self.Hub.imu.heading()
    objetivo = inicio + angulo_deg

    # La rueda de apoyo debe permanecer firmemente inmóvil.
    motor_fijo.hold()
    wait(3)

    # Timeout proporcional al tamaño del giro.
    if tiempo_max_ms is None:
        tiempo_max_ms = max(
            350,
            int(abs(angulo_deg) * 18)
        )

    cronometro = StopWatch()
    cronometro.reset()

    lecturas_estables = 0

    while cronometro.time() < tiempo_max_ms:
        actual = self.Hub.imu.heading()
        error = self._error_angular(objetivo, actual)
        error_abs = abs(error)

        # Dos lecturas consecutivas dentro de tolerancia.
        if error_abs <= tolerancia:
            lecturas_estables += 1

            if lecturas_estables >= 2:
                break
        else:
            lecturas_estables = 0

        # Velocidad máxima lejos del objetivo.
        if error_abs >= zona_freno:
            velocidad_actual = velocidad
        else:
            # Reducción progresiva al acercarse al objetivo.
            proporcion = error_abs / zona_freno

            velocidad_actual = (
                velocidad_min
                + (velocidad - velocidad_min) * proporcion
            )

        velocidad_actual = int(
            self.limitar(
                velocidad_actual,
                velocidad_min,
                velocidad
            )
        )

        # El signo del error permite corregir un sobrepaso.
        direccion_error = 1 if error > 0 else -1

        velocidad_motor = (
            velocidad_actual
            * direccion_error
            * sentido_motor
        )

        motor_activo.run(velocidad_motor)

        wait(2)

    # Frenado corto para eliminar inercia.
    motor_activo.brake()
    motor_fijo.hold()
    wait(12)

    if perfil == "encadenado":
        motor_activo.stop()
        wait(2)
    else:
        motor_activo.hold()
        motor_fijo.hold()
        wait(15)

# -----------------------------------------------------------------------------
# giro_derecha
# Gira apoyándose en un solo motor del lado derecho mediante la función interna de giro.
# -----------------------------------------------------------------------------
def giro_derecha(
    self,
    angulo_deg,
    velocidad=1000,
    velocidad_min=180,
    zona_freno=22,
    tolerancia=1.2,
    perfil="seguro"
):
    self._giro_un_motor(
        motor_activo=self.motor_derecho,
        motor_fijo=self.motor_izquierdo,
        angulo_deg=angulo_deg,
        sentido_motor=-1,
        velocidad=velocidad,
        velocidad_min=velocidad_min,
        zona_freno=zona_freno,
        tolerancia=tolerancia,
        perfil=perfil
    )

#Giro izquierda
def giro_izquierda(
    self,
    angulo_deg,
    velocidad=1000,
    velocidad_min=180,
    zona_freno=22,
    tolerancia=1.2,
    perfil="seguro"
):
    self._giro_un_motor(
        motor_activo=self.motor_izquierdo,
        motor_fijo=self.motor_derecho,
        angulo_deg=angulo_deg,
        sentido_motor=1,
        velocidad=velocidad,
        velocidad_min=velocidad_min,
        zona_freno=zona_freno,
        tolerancia=tolerancia,
        perfil=perfil
    )

# -----------------------------------------------------------------------------
# girar_corto
# Realiza giros pequeños y rápidos, evitando que el robot quede bloqueado
# intentando corregir los últimos grados.
# -----------------------------------------------------------------------------
def girar_corto(
    self,
    angulo_deg,
    potencia_max=45,
    potencia_min=24,
    kp=4.0,
    tolerancia=1.8,
    tiempo_max_ms=500,
    perfil="encadenado"
):
    if angulo_deg == 0:
        return

    # Cada giro se mide desde la posición actual.
    self.Hub.imu.reset_heading(0)
    wait(10)

    objetivo = angulo_deg

    cronometro = StopWatch()
    cronometro.reset()

    lecturas_estables = 0

    while cronometro.time() < tiempo_max_ms:
        angulo_actual = self.Hub.imu.heading()

        error = self._error_angular(
            objetivo,
            angulo_actual
        )

        # Se exigen dos lecturas dentro de tolerancia para evitar
        # terminar por una lectura aislada del IMU.
        if abs(error) <= tolerancia:
            lecturas_estables += 1

            if lecturas_estables >= 2:
                break
        else:
            lecturas_estables = 0

        potencia = error * kp

        potencia = self.limitar(
            potencia,
            -potencia_max,
            potencia_max
        )

        # Evita que la potencia quede demasiado baja para mover el robot.
        if abs(potencia) < potencia_min:
            potencia = (
                potencia_min
                if error > 0
                else -potencia_min
            )

        self.motor_izquierdo.dc(potencia)
        self.motor_derecho.dc(-potencia)

        wait(2)

    # Salida rápida, sin mantener los motores bloqueados demasiado tiempo.
    self.motor_izquierdo.brake()
    self.motor_derecho.brake()

    if perfil == "encadenado":
        wait(5)
    else:
        self.motor_izquierdo.hold()
        self.motor_derecho.hold()
        wait(20)

def avanzar_hasta_salir_negro(
    self,
    velocidad_max=900,
    velocidad_min=100,
    kp_gyro=20.0,
    zona_rampa_cm=8,
    perfil="encadenado",
    rumbo_esperado=None,

    sensor_color=None,
    objetivo_reflexion=15,
    lecturas_salida=5,

    torque_grados=None,
    torque_velocidad=180,
    torque_retraso_ms=0
):
    if sensor_color is None:
        sensor_color = self.seguidor

    self.preparar_movimiento(
        reset_motores=False,
        reset_gyro=False,
        perfil=perfil
    )

    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(90)

    self.drive_base.reset()

    if rumbo_esperado is not None:
        heading_objetivo = rumbo_esperado
    else:
        heading_objetivo = self.Hub.imu.heading()

    grados_por_cm = (
        360 / (self.circunferencia / 10)
    )

    grados_rampa = (
        zona_rampa_cm * grados_por_cm
    )

    cronometro = StopWatch()
    cronometro.reset()

    torque_iniciado = False

    # Contador para confirmar que realmente salió del negro.
    contador_salida = 0

    while True:

        reflexion = sensor_color.reflection()

        # =====================================================
        # CONFIRMACIÓN DE SALIDA DEL NEGRO
        # =====================================================
        if reflexion > objetivo_reflexion:
            contador_salida += 1
        else:
            contador_salida = 0

        if contador_salida >= lecturas_salida:
            break

        # =====================================================
        # DISTANCIA RECORRIDA
        # =====================================================
        recorrido = (
            abs(self.drive_base.distance())
            * self.grados_por_mm
        )

        # =====================================================
        # RAMPA DE ACELERACIÓN
        # =====================================================
        if recorrido < grados_rampa:

            proporcion = (
                recorrido / grados_rampa
            )

            vel_base = (
                velocidad_min
                + (
                    velocidad_max
                    - velocidad_min
                ) * proporcion
            )

        else:
            vel_base = velocidad_max

        tiempo_ms = cronometro.time()

        # =====================================================
        # TORQUE
        # =====================================================
        if (
            torque_grados is not None
            and not torque_iniciado
            and tiempo_ms >= torque_retraso_ms
        ):
            self.mover_torque(
                grados_torque=torque_grados,
                velocidad_torque=torque_velocidad,
                esperar=False
            )

            torque_iniciado = True

        # =====================================================
        # RAMPA TEMPORAL
        # =====================================================
        if tiempo_ms < 150:
            vel = (
                vel_base
                * (tiempo_ms / 150)
            )
        else:
            vel = vel_base

        vel = self.limitar(
            vel,
            25,
            velocidad_max
        )

        # =====================================================
        # GIROSCOPIO
        # =====================================================
        actual_heading = self.Hub.imu.heading()

        error_gyro = self._error_angular(
            heading_objetivo,
            actual_heading
        )

        correccion_giro = (
            error_gyro * kp_gyro
        )

        # =====================================================
        # AVANCE
        # =====================================================
        self.drive_base.drive(
            vel,
            correccion_giro
        )

    # =========================================================
    # FRENADO
    # =========================================================
    self.drive_base.stop()

    self.motor_izquierdo.brake()
    self.motor_derecho.brake()
    wait(60)

    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)

def avanzar_con_torque(
    self,
    distancia_cm,
    distancia_activacion_torque_cm,
    torque_grados,
    torque_velocidad=900,
    velocidad_max=900,
    velocidad_min=100,
    kp_gyro=20.0,
    zona_rampa_cm=8,
    perfil="encadenado",
    rumbo_esperado=None
):
    if distancia_cm == 0:
        return

    self.preparar_movimiento(
        reset_motores=False,
        reset_gyro=False,
        perfil=perfil
    )

    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(90)

    self.drive_base.reset()

    # =========================================================
    # MEMORIA DE RUMBO
    # =========================================================
    if rumbo_esperado is not None:
        heading_objetivo = rumbo_esperado
    else:
        heading_objetivo = self.Hub.imu.heading()

    # Conversión de centímetros a grados de las ruedas
    grados_por_cm = (
        360 / (self.circunferencia / 10)
    )

    grados_objetivo = (
        abs(distancia_cm) * grados_por_cm
    )

    # Convertimos la distancia a la que quieres que inicie el torque en grados
    grados_activacion_torque = (
        abs(distancia_activacion_torque_cm) * grados_por_cm
    )

    rampa_efectiva_cm = min(
        zona_rampa_cm,
        abs(distancia_cm) / 2.0
    )

    grados_rampa = (
        rampa_efectiva_cm * grados_por_cm
    )

    signo = 1 if distancia_cm > 0 else -1

    cronometro = StopWatch()
    cronometro.reset()

    # Impide que el torque se active más de una vez
    torque_iniciado = False

    while True:
        # Distancia real recorrida calculada en grados
        recorrido = (
            abs(self.drive_base.distance())
            * self.grados_por_mm
        )

        restante = (
            grados_objetivo - recorrido
        )

        # Finalizar al alcanzar la distancia
        if restante <= 1.5:
            break

        actual_heading = self.Hub.imu.heading()

        error_gyro = self._error_angular(
            heading_objetivo,
            actual_heading
        )

        # Rampa de aceleración
        if recorrido < grados_rampa:
            proporcion = recorrido / grados_rampa
            vel_base = (
                velocidad_min
                + (velocidad_max - velocidad_min) * proporcion
            )

        # Rampa de desaceleración
        elif restante < grados_rampa:
            proporcion = restante / grados_rampa
            vel_base = (
                velocidad_min * 0.4
                + (velocidad_max - velocidad_min * 0.4) * proporcion
            )

        else:
            vel_base = velocidad_max

        tiempo_ms = cronometro.time()

        # =====================================================
        # TORQUE BASADO EN DISTANCIA
        # =====================================================
        if (
            torque_grados is not None
            and not torque_iniciado
            and recorrido >= grados_activacion_torque
        ):
            self.mover_torque(
                grados_torque=torque_grados,
                velocidad_torque=torque_velocidad,
                esperar=False  # Crucial: No detiene el bucle while
            )
            torque_iniciado = True

        # Rampa temporal inicial original
        if tiempo_ms < 150:
            vel = vel_base * (tiempo_ms / 150)
        else:
            vel = vel_base

        vel = self.limitar(
            vel,
            25,
            velocidad_max
        )

        correccion_giro = error_gyro * kp_gyro

        self.drive_base.drive(
            vel * signo,
            correccion_giro
        )

    # =======================================================
    # Frenado de dos fases
    # =======================================================
    self.drive_base.stop()

    self.motor_izquierdo.brake()
    self.motor_derecho.brake()
    wait(60)

    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)

# -----------------------------------------------------------------------------------------------------------------------------
# establecer_norte
# Establece la orientación actual del robot como el rumbo absoluto 0°.
# Debe llamarse con el robot quieto y correctamente alineado.
# ------------------------------------------------------------------------------------------------------------------------------
def establecer_norte(self, rumbo_inicial=0):
    """
    Establece la dirección actual como referencia absoluta.

    Ejemplo:
        robot.establecer_norte()

    Después de esta llamada:
        0°   = dirección inicial
        90°  = giro horario desde el norte
        180° = dirección contraria
        -90° = giro antihorario desde el norte
    """

    # Asegurar que el robot esté completamente detenido.
    self.drive_base.stop()
    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(100)

    # Establecer la orientación actual como el rumbo indicado.
    self.Hub.imu.reset_heading(rumbo_inicial)

    wait(50)


# -----------------------------------------------------------------------------
# girar_a_rumbo
# Gira hacia una orientación absoluta tomando como referencia el norte virtual.
# -----------------------------------------------------------------------------
def girar_a_rumbo(
    self,
    rumbo_objetivo,
    potencia_max=100,
    potencia_media=55,
    potencia_min=45,
    kp=2.6,
    kd=4.2,
    zona_media=35,
    zona_precisa=9,
    tolerancia=1.0,
    ciclos_estables=5,
    tiempo_maximo_ms=3000,
    perfil="encadenado"
):
    """
    Gira rápidamente hacia un rumbo absoluto usando el IMU.

    El rumbo no es relativo a la posición actual. Siempre toma como
    referencia el norte establecido mediante establecer_norte().

    Rumbos principales:
        0°    = norte inicial
        90°   = derecha
        180°  = dirección contraria
        270°  = izquierda
        -90°  = equivalente a 270°

    La velocidad se divide en tres zonas:
        - Lejos: potencia máxima.
        - Aproximación: potencia media.
        - Precisión: potencia baja.

    Ejemplos:
        robot.girar_a_rumbo(90)
        robot.girar_a_rumbo(180)
        robot.girar_a_rumbo(0)
    """

    # ----------------------------------------------------------
    # Función interna para calcular el camino angular más corto.
    #
    # Permite trabajar correctamente aunque el heading del Hub
    # sea mayor que 360° o menor que -360°.
    # ----------------------------------------------------------
    def error_angular_absoluto(objetivo, actual):
        error = (objetivo - actual + 180) % 360 - 180
        return error

    # Asegurar que los valores de potencia sean positivos.
    potencia_max = abs(potencia_max)
    potencia_media = abs(potencia_media)
    potencia_min = abs(potencia_min)

    # Preparar el movimiento SIN reiniciar el giroscopio.
    # Reiniciarlo aquí eliminaría el norte virtual.
    self.preparar_movimiento(
        reset_motores=False,
        reset_gyro=False,
        perfil=perfil
    )

    self.drive_base.stop()
    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)

    actual = self.Hub.imu.heading()

    error_anterior = error_angular_absoluto(
        rumbo_objetivo,
        actual
    )

    derivada_filtrada_anterior = 0
    contador_estable = 0

    cronometro = StopWatch()
    cronometro.reset()

    while True:
        actual = self.Hub.imu.heading()

        error = error_angular_absoluto(
            rumbo_objetivo,
            actual
        )

        # ------------------------------------------------------
        # FINALIZACIÓN ESTABLE
        #
        # No termina con una sola lectura correcta. Exige varias
        # lecturas consecutivas dentro de la tolerancia.
        # ------------------------------------------------------
        if abs(error) <= tolerancia:
            contador_estable += 1

            # Detener temporalmente para comprobar que la inercia
            # no siga moviendo al robot fuera de la tolerancia.
            self.motor_izquierdo.brake()
            self.motor_derecho.brake()

            if contador_estable >= ciclos_estables:
                break

            wait(4)
            continue

        else:
            contador_estable = 0

        # Evitar que un bloqueo mecánico deje la función atrapada.
        if cronometro.time() >= tiempo_maximo_ms:
            print(
                "Aviso: girar_a_rumbo superó el tiempo máximo.",
                "Objetivo:",
                rumbo_objetivo,
                "Actual:",
                actual,
                "Error:",
                error
            )
            break

        # ------------------------------------------------------
        # CONTROL PD
        # ------------------------------------------------------
        derivada_cruda = error - error_anterior

        # Filtro para evitar respuestas bruscas por ruido del IMU.
        derivada_filtrada = (
            derivada_cruda * 0.7
            + derivada_filtrada_anterior * 0.3
        )

        correccion = (
            error * kp
            + derivada_filtrada * kd
        )

        distancia_al_objetivo = abs(error)

        # ------------------------------------------------------
        # VELOCIDAD POR ZONAS
        # ------------------------------------------------------
        if distancia_al_objetivo > zona_media:
            limite_potencia = potencia_max

        elif distancia_al_objetivo > zona_precisa:
            # Reduce progresivamente entre potencia máxima
            # y potencia media.
            proporcion = (
                (distancia_al_objetivo - zona_precisa)
                / (zona_media - zona_precisa)
            )

            limite_potencia = (
                potencia_media
                + (potencia_max - potencia_media)
                * proporcion
            )

        else:
            # Zona final de precisión.
            proporcion = (
                distancia_al_objetivo
                / zona_precisa
            )

            limite_potencia = (
                potencia_min
                + (potencia_media - potencia_min)
                * proporcion
            )

        potencia_final = self.limitar(
            correccion,
            -limite_potencia,
            limite_potencia
        )

        # Vencer la fricción solamente cuando todavía estamos
        # fuera de la tolerancia.
        if abs(potencia_final) < potencia_min:
            if error > 0:
                potencia_final = potencia_min
            else:
                potencia_final = -potencia_min

        # El mismo sentido de motores utilizado por girar().
        potencia_izquierda = int(potencia_final)
        potencia_derecha = int(-potencia_final)

        self.motor_izquierdo.dc(
            self.limitar(
                potencia_izquierda,
                -100,
                100
            )
        )

        self.motor_derecho.dc(
            self.limitar(
                potencia_derecha,
                -100,
                100
            )
        )

        error_anterior = error
        derivada_filtrada_anterior = derivada_filtrada

        wait(4)

    # Frenado final.
    self.motor_izquierdo.brake()
    self.motor_derecho.brake()
    wait(30)

    self.motor_izquierdo.hold()
    self.motor_derecho.hold()
    wait(20)

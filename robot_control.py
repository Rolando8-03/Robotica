from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Side, Stop, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class Base:
    def __init__(self):
        self.Hub = PrimeHub()
        
        # --- ÚNICA COSA AGREGADA: Calibración profunda del giroscopio ---
        self.calibrar_giroscopio()
        # ----------------------------------------------------------------
        
        wait(2000)  # pausa para esperar el robot (lo dejamos por si acaso)

        # Motores
        self.motor_derecho   = Motor(Port.B, Direction.CLOCKWISE)
        self.motor_izquierdo = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.motor_torque    = Motor(Port.C)
        self.motor_garra     = Motor(Port.A)

        # Sensores
        self.seguidor = ColorSensor(Port.E)

        # DriveBase necesario para algunas funciones (como reset de distancia)
        self.drive_base = DriveBase(
            self.motor_izquierdo,
            self.motor_derecho,
            wheel_diameter=56,
            axle_track=170
        )
        
        self.drive_base.use_gyro(True)

        # Ajustes de movimiento (usando valores del código 2)
        self.drive_base.settings(
            straight_speed=900,          # velocidad máxima en línea recta
            straight_acceleration=900,   # qué tan rápido llega a la velocidad máxima
            turn_rate=400,               # velocidad máxima al girar sobre su propio eje
            turn_acceleration=400        # qué tan rápido empieza y frena el giro
        )
        
        # Factor de calibración para los motores (ajusta según necesites)
        self.factor_calibracion = 0.9875  # Tomado de tu código del giroscopio
        
        # Para seguimiento de línea con giroscopio
        self.heading_inicial = 0

        # Constantes
        self.diametro_rueda = 56  # mm
        self.distancia_entre_ruedas = 170  # mm
        self.pi = 3.1416

        # No activamos el giroscopio en DriveBase para tener control manual

    # ---------- UTILIDADES ----------
    def esperar(self, ms):
        wait(ms)

    def frenar_rolando(self):
        self.motor_derecho.brake()
        self.motor_izquierdo.brake()
        wait(30)

    def reset_imu(self):
        """Resetea el IMU asegurando que el robot está quieto"""
        wait(500)  # Espera a que se estabilice
        self.Hub.imu.reset_heading(0)
        # Lee el heading varias veces para asegurar que es 0
        for _ in range(5):
            if abs(self.Hub.imu.heading()) > 0.01:
                # Si hay desviación, resetea de nuevo
                self.Hub.imu.reset_heading(0)
            wait(100)
        print("IMU listo, heading:", self.Hub.imu.heading())

    # ---------- MOVIMIENTO RECTILÍNEO CON GIROSCOPIO (CORREGIDO) ----------
    def mover(self, distancia_cm, velocidad=None):
        # Convertir cm a mm (DriveBase trabaja en mm)
        distancia_mm = distancia_cm * 10

        # Si se especifica velocidad, la convertimos a velocidad lineal en mm/s
        if velocidad is not None:
            vel_lineal_mm_s = velocidad * 0.5
            self.drive_base.settings(straight_speed=vel_lineal_mm_s, turn_rate=200)

        # Ejecutar movimiento
        self.drive_base.straight(distancia_mm)

        # Frenar (opcional, straight ya frena al final)
        self.frenar()

    # ---------- GIROS MANUALES (SIN GIROSCOPIO) ----------
    def girar(self, velocidad, angulo_grados):
        """
        Gira sobre su propio eje.
        - angulo_grados > 0: derecha, < 0: izquierda
        """
        angulo_abs = abs(angulo_grados)
        sentido = 1 if angulo_grados > 0 else -1  # positivo = derecha

        # Cálculo: distancia lineal que recorre cada rueda = (ángulo_rad * distancia_entre_ruedas) / 2
        angulo_rad = angulo_abs * self.pi / 180
        distancia_lineal = (angulo_rad * self.distancia_entre_ruedas) / 2

        circunferencia = self.diametro_rueda * self.pi
        grados_motor = (distancia_lineal / circunferencia) * 360 * self.factor_calibracion

        self.motor_izquierdo.reset_angle(0)
        self.motor_derecho.reset_angle(0)

        # Asignar velocidades según sentido
        if sentido > 0:  # derecha
            vel_izq = -abs(velocidad)
            vel_der = abs(velocidad)
        else:  # izquierda
            vel_izq = abs(velocidad)
            vel_der = -abs(velocidad)

        while abs(self.motor_izquierdo.angle()) < grados_motor and abs(self.motor_derecho.angle()) < grados_motor:
            restante = grados_motor - abs(self.motor_izquierdo.angle())
            v = abs(velocidad)
            if restante < 100:
                v = max(150, v * 0.3)

            self.motor_izquierdo.run(v * (1 if vel_izq > 0 else -1))
            self.motor_derecho.run(v * (1 if vel_der > 0 else -1))
            wait(10)

        self.frenar()
        self.reset_imu()

    def giro_izquierdo_llanta(self, velocidad, angulo_grados):
        """
        Gira con rueda izquierda fija.
        - angulo_grados > 0: derecha, < 0: izquierda
        """
        angulo_abs = abs(angulo_grados)
        sentido = 1 if angulo_grados > 0 else -1

        angulo_rad = angulo_abs * self.pi / 180
        distancia_lineal = angulo_rad * self.distancia_entre_ruedas

        circunferencia = self.diametro_rueda * self.pi
        grados_motor = (distancia_lineal / circunferencia) * 360 * self.factor_calibracion

        self.motor_izquierdo.hold()
        self.motor_derecho.reset_angle(0)

        # Para girar a la derecha (+), la rueda derecha avanza (+)
        # Para girar a la izquierda (-), la rueda derecha retrocede (-)
        direccion = 1 if sentido > 0 else -1

        while abs(self.motor_derecho.angle()) < grados_motor:
            restante = grados_motor - abs(self.motor_derecho.angle())
            v = abs(velocidad)
            if restante < 100:
                v = max(150, v * 0.3)

            self.motor_derecho.run(v * direccion)
            wait(10)

        self.motor_derecho.brake()
        self.motor_izquierdo.stop()
        self.reset_imu()

    def giro_derecho_llanta(self, velocidad, angulo_grados):
        """
        Gira con rueda derecha fija.
        - angulo_grados > 0: izquierda, < 0: derecha
        """
        angulo_abs = abs(angulo_grados)
        sentido = 1 if angulo_grados > 0 else -1

        angulo_rad = angulo_abs * self.pi / 180
        distancia_lineal = angulo_rad * self.distancia_entre_ruedas

        circunferencia = self.diametro_rueda * self.pi
        grados_motor = (distancia_lineal / circunferencia) * 360 * self.factor_calibracion

        self.motor_derecho.hold()
        self.motor_izquierdo.reset_angle(0)

        # Para girar a la derecha (+), la rueda derecha avanza (+)
        # Para girar a la izquierda (-), la rueda derecha retrocede (-)
        direccion = 1 if sentido > 0 else -1

        while abs(self.motor_izquierdo.angle()) < grados_motor:
            restante = grados_motor - abs(self.motor_izquierdo.angle())
            v = abs(velocidad)
            if restante < 100:
                v = max(150, v * 0.3)

            self.motor_izquierdo.run(v * direccion)
            wait(10)

        self.motor_izquierdo.brake()
        self.motor_derecho.stop()
        self.reset_imu()

    # ---------- SEGUIDOR DE LÍNEA CORREGIDO (USA DRIVE_BASE PARA DISTANCIA) ----------
    def seguir_linea(self, velocidad, distancia_cm=None):
        """
        Sigue línea negra. Para retroceso (distancia_cm < 0) usa offset propio.
        - velocidad: rapidez (0-1000), siempre positiva.
        - distancia_cm: >0 avanza, <0 retrocede.
        """
        # Parámetros comunes (los mismos que funcionan en avance)
        REF = 30
        KP = 8.0
        KI = 0.01
        KD = 10.0

        # Offset independiente para retroceso (ajústalo aquí)
        OFFSET_ATRAS = -1   # negativo = corrige izquierda, positivo = derecha

        rapidez = abs(velocidad)

        # Dirección según distancia_cm
        if distancia_cm is not None:
            direccion = 1 if distancia_cm > 0 else -1
            distancia_objetivo_mm = abs(distancia_cm) * 10
            self.motor_izquierdo.reset_angle(0)
            self.motor_derecho.reset_angle(0)
            circunferencia = self.diametro_rueda * self.pi
            usar_distancia = True
        else:
            direccion = 1 if velocidad > 0 else -1
            usar_distancia = False

        # Seleccionar offset
        offset = 0 if direccion == 1 else OFFSET_ATRAS

        vel_actual = 500
        last_error = 0
        integral = 0
        dt = 10
        max_correccion = 200  # Límite suave para evitar saturación

        while True:
            if usar_distancia:
                angulo_prom = (self.motor_izquierdo.angle() + self.motor_derecho.angle()) / 2
                distancia_actual_mm = (angulo_prom / 360) * circunferencia * self.factor_calibracion
                if (direccion == 1 and distancia_actual_mm >= distancia_objetivo_mm) or \
                (direccion == -1 and distancia_actual_mm <= -distancia_objetivo_mm):
                    break

            # Rampa
            if vel_actual < rapidez:
                vel_actual += 200
                if vel_actual > rapidez:
                    vel_actual = rapidez

            # Sensor
            reflexion = self.seguidor.reflection()
            error = REF - reflexion

            # Reseteamos integral si el error es muy grande (hemos perdido la línea)
            if abs(error) > 20:
                integral = 0

            # PID con límite de integral
            integral += error * dt * 0.001
            integral = max(-100, min(100, integral))  # Límite fijo para integral
            derivada = (error - last_error) / (dt / 1000) if dt > 0 else 0

            correccion = (error * KP) + (integral * KI) + (derivada * KD) + offset

            # Limitar corrección a un valor razonable
            correccion = max(-max_correccion, min(max_correccion, correccion))

            # Potencia base (con signo)
            potencia_base = vel_actual * direccion

            # Aplicar corrección (siempre igual)
            potencia_izq = potencia_base + correccion
            potencia_der = potencia_base - correccion

            # Limitar a ±1000
            potencia_izq = max(-1000, min(1000, potencia_izq))
            potencia_der = max(-1000, min(1000, potencia_der))

            # Depuración (opcional)
            # print(f"dir:{direccion} err:{error:.1f} corr:{correccion:.1f} izq:{potencia_izq:.0f} der:{potencia_der:.0f}")

            self.motor_izquierdo.run(potencia_izq)
            self.motor_derecho.run(potencia_der)

            last_error = error

        self.frenar()

    # ---------- FUNCIONES PARA MECANISMOS ----------
    def mover_torque_rolando(self, velocidad, grados):
        """
        Mueve el motor torque con aceleración y desaceleración suave.
        - velocidad: máxima velocidad (0-1000), siempre positiva (la dirección la da grados)
        - grados: >0 un sentido, <0 sentido opuesto
        """
        direccion = 1 if grados > 0 else -1
        grados_abs = abs(grados)
        
        # Configuración del perfil
        vel_max = abs(velocidad)
        vel_actual = 0
        incremento = 200  # aceleración rápida
        zona_desaceleracion = 100  # grados antes del final donde empezar a frenar
        vel_min = 150  # velocidad mínima al final
        
        self.motor_torque.reset_angle(0)
        
        while abs(self.motor_torque.angle()) < grados_abs:
            restante = grados_abs - abs(self.motor_torque.angle())
            
            # Aceleración
            if vel_actual < vel_max:
                vel_actual += incremento
                if vel_actual > vel_max:
                    vel_actual = vel_max
            
            # Desaceleración si estamos cerca del final
            if restante < zona_desaceleracion:
                # Velocidad proporcional a la distancia restante
                factor = restante / zona_desaceleracion
                vel_actual = max(vel_min, vel_max * factor)
            
            # Aplicar potencia con la dirección adecuada
            self.motor_torque.run(vel_actual * direccion)
            wait(10)
        
        # Parar al final
        self.motor_torque.hold()

     # ------------- ÚNICA FUNCIÓN NUEVA AGREGADA ----------------
    def calibrar_giroscopio(self):
        """
        Calibración profunda del giroscopio.
        El robot DEBE estar quieto durante este proceso.
        """
        print("Calibrando giroscopio... NO MOVER EL ROBOT")
        self.Hub.light.on(Color.RED)
        
        # Paso 1: Reset inicial y espera para que el giroscopio se estabilice
        self.Hub.imu.reset_heading(0)
        wait(500)
        
        # Paso 2: Lecturas repetidas para asentar la calibración
        # Hacemos 10 lecturas con pequeñas pausas para que el IMU se ajuste
        for i in range(10):
            _ = self.Hub.imu.heading()  # Leemos para forzar actualización
            wait(100)
        
        # Paso 3: Reset final después de la calibración
        self.Hub.imu.reset_heading(0)
        wait(500)
        
        # Verificación rápida (opcional)
        lectura_final = self.Hub.imu.heading()
        print(f"Calibración completada. Lectura inicial: {lectura_final}°")
        
        self.Hub.light.on(Color.GREEN)
        wait(500)
    # ------------------------------------------------------------

    # ------------- FUNCIONES DE UTILIDAD ----------------

    def frenar(self):
        # hacemos que ambos motores frenen y esperamos un momento
        self.motor_derecho.brake()
        self.motor_izquierdo.brake()
        wait(30)

    def reset_imu_y_base(self):
        # resetea la distancia recorrida
        self.drive_base.reset()
        # resetea la orientación
        self.Hub.imu.reset_heading(0)
        wait(50)  # Pequeña pausa después del reset

    # ---------- MOVIMIENTOS DE LA BASE --------------------
    def avanzar(self, distancia_mm):
        self.drive_base.straight(distancia_mm)

    def girar(self, angulo_deg):
        self.drive_base.turn(angulo_deg)

    # ----------------- FUNCIONES DEL GIROSCOPIO (de tu código) -----------------
    def movimiento_giroscopio(self, velocidad, distancia):
        """Movimiento recto calibrado con giroscopio"""
        self.Hub.imu.reset_heading(0)
        self.motor_izquierdo.reset_angle(0)

        grados = (distancia*10 / (56 * 3.1416)) * 360 * self.factor_calibracion

        while abs(self.motor_izquierdo.angle()) < abs(grados):
            restante = abs(grados) - abs(self.motor_izquierdo.angle())

            v = velocidad
            if restante < 150:           # desaceleración final
                v = velocidad * 0.4

            # mantiene el signo correcto
            direccion = 1 if distancia*10 > 0 else -1

            error = self.Hub.imu.heading()
            if direccion > 0:
                correccion = error * 2
            else:
                correccion = error * 1.2

            self.motor_izquierdo.run((v * direccion) - correccion)
            self.motor_derecho.run((v * direccion) + correccion)

            wait(10)

        self.motor_izquierdo.stop()
        self.motor_derecho.stop()
        wait(50)
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()

    def movimiento_adelante(self, velocidad, distancia):
        """Movimiento hacia adelante con corrección giroscópica"""
        self.Hub.imu.reset_heading(0)
        self.motor_izquierdo.reset_angle(0)

        grados = (distancia*10 / (56 * 3.1416)) * 360 * self.factor_calibracion

        while abs(self.motor_izquierdo.angle()) < abs(grados):
            restante = abs(grados) - abs(self.motor_izquierdo.angle())

            v = velocidad
            if restante < 150:
                v = velocidad * 0.4

            error = self.Hub.imu.heading()
            correccion = error * 1.8  # Ajusta este valor según necesites

            self.motor_izquierdo.run(v - correccion)
            self.motor_derecho.run(v + correccion)

            wait(10)

        self.motor_izquierdo.stop()
        self.motor_derecho.stop()
        wait(50)
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()

    def girar_giroscopio(self, velocidad, angulo):
        """Giro con ambas ruedas usando giroscopio"""
        self.Hub.imu.reset_heading(0)

        while abs(self.Hub.imu.heading()) < abs(angulo * self.factor_calibracion):
            restante = abs(angulo * self.factor_calibracion) - abs(self.Hub.imu.heading())

            v = velocidad
            if restante < 20:          # desaceleración final
                v = velocidad * 0.35

            direccion = 1 if angulo > 0 else -1

            self.motor_izquierdo.run(v * direccion)
            self.motor_derecho.run(-v * direccion)

            wait(10)

        self.motor_izquierdo.stop()
        self.motor_derecho.stop()
        wait(50)
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()

    # ----------------- GIROS CON UNA SOLA RUEDA -----------------
    def giro_con_derecha(self, grados_rueda):
        self.drive_base.stop()
        self.motor_derecho.run_angle(900, grados_rueda)
        self.reset_imu_y_base()

    def giro_con_izquierda(self, grados_rueda):
        self.drive_base.stop()
        self.motor_izquierdo.run_angle(900, grados_rueda)
        self.reset_imu_y_base()

    def giro_izquierda(self, velocidad, grados):
        """Giro usando solo la rueda izquierda con corrección giroscópica"""
        self.Hub.imu.reset_heading(0)
        self.motor_derecho.hold()

        direccion = 1 if grados > 0 else -1
        objetivo = abs(grados)

        while True:
            angulo = abs(self.Hub.imu.heading())
            error = objetivo - angulo

            if error <= 0:
                break

            if error < 20:
                vel = max(130, velocidad * 0.35)
            elif error < 8:
                vel = 110
            else:
                vel = velocidad

            self.motor_izquierdo.run(vel * direccion)
            wait(5)  # Pequeña pausa para no saturar

        self.motor_izquierdo.brake()
        self.motor_derecho.stop()

    def giro_derecha(self, velocidad, grados):
        """Giro usando solo la rueda derecha con corrección giroscópica"""
        self.Hub.imu.reset_heading(0)
        self.motor_izquierdo.hold()

        direccion = 1 if grados > 0 else -1
        objetivo = abs(grados)

        while True:
            angulo = abs(self.Hub.imu.heading())
            error = objetivo - angulo

            if error <= 0:
                break

            if error < 20:
                vel = max(130, velocidad * 0.35)
            elif error < 8:
                vel = 110
            else:
                vel = velocidad

            self.motor_derecho.run(vel * direccion)
            wait(5)

        self.motor_derecho.brake()
        self.motor_izquierdo.stop()

    # ----------------- MECANISMOS -----------------
    def mover_torque(self, velocidad, grados):
        self.motor_torque.run_angle(velocidad, grados)

    def mover_garra(self, velocidad, grados):
        self.motor_garra.run_angle(velocidad, grados, then=Stop.HOLD, wait=True)

    def seguir_linea_con_giroscopio(self, distancia_cm=None, velocidad=900, ref=30):
        """
        SEGUIDOR MAGNÉTICO - Se adhiere a la línea como un imán
        - Máxima fijación a la línea recta
        - Sin curvas ni oscilaciones
        - Corrección agresiva pero estable
        """
        print("🧲 SEGUIDOR MAGNÉTICO ACTIVADO - ADHERENCIA TOTAL A LA LÍNEA")
        
        # ===== CONSTANTES OPTIMIZADAS PARA COMPORTAMIENTO MAGNÉTICO =====
        kp = 8.0      # ALTO - Responde fuertemente al error (imán)
        kd = 12.0     # MUY ALTO - Elimina oscilaciones (amortiguador)
        ki = 0.01     # MÍNIMO - Solo para compensación de error constante
        
        # ===== BANDA MUERTA Y UMBRALES =====
        # Si el error es menor que esto, consideramos que está perfectamente centrado
        banda_muerta = 2
        
        # ===== FACTORES DE CORRECCIÓN DE HARDWARE =====
        # Compensación fija si el robot tiende a desviarse siempre al mismo lado
        sesgo_izquierdo = 0.98    # Reduce ligeramente izquierda (1.0 = neutral)
        sesgo_derecho = 1.02      # Aumenta ligeramente derecha
        
        # ===== VARIABLES PID =====
        last_error = 0
        integral = 0
        last_time = StopWatch()
        last_time.reset()
        
        # ===== CONFIGURACIÓN INICIAL =====
        self.Hub.imu.reset_heading(0)
        
        # Rampa de velocidad (más rápida para llegar pronto al modo magnético)
        velocidad_actual = 400
        incremento = 30
        
        # Para distancia
        if distancia_cm is not None:
            self.drive_base.reset()
            distancia_mm = distancia_cm * 10
        
        # Contador para estabilidad
        contador_estable = 0
        
        print(f"⚡ Velocidad objetivo: {velocidad}")
        
        while True:
            # ===== CONTROL DE DISTANCIA =====
            if distancia_cm is not None and abs(self.drive_base.distance()) >= distancia_mm:
                break
            
            # ===== RAMPA DE VELOCIDAD =====
            if velocidad_actual < velocidad:
                velocidad_actual += incremento
                if velocidad_actual > velocidad:
                    velocidad_actual = velocidad
            
            # ===== LECTURA DEL SENSOR =====
            current_reflection = self.seguidor.reflection()
            error_linea = ref - current_reflection
            
            # ===== BANDA MUERTA =====
            # Si el error es muy pequeño, lo tratamos como cero (robot centrado)
            if abs(error_linea) < banda_muerta:
                error_linea = 0
                contador_estable += 1
            else:
                contador_estable = 0
            
            # ===== CÁLCULO DEL TIEMPO =====
            dt = last_time.time()
            if dt > 0:
                # Integral limitada
                integral += error_linea * dt * 0.001
                integral = max(-20, min(20, integral))  # Anti-windup estricto
                
                # Derivada filtrada
                derivative = (error_linea - last_error) / (dt/1000 + 0.001)
                
                # Filtro pasa-bajos para la derivada (suaviza)
                derivative = derivative * 0.7 + self._last_derivative * 0.3 if hasattr(self, '_last_derivative') else derivative
                self._last_derivative = derivative
            else:
                derivative = 0
            
            # ===== CORRECCIÓN PID PRINCIPAL =====
            correccion_linea = (error_linea * kp) + (integral * ki) + (derivative * kd)
            
            # ===== CORRECCIÓN DEL GIROSCOPIO (MÍNIMA) =====
            error_giroscopio = self.Hub.imu.heading()
            
            # Solo corregimos si el error de giro es significativo
            if abs(error_giroscopio) > 1.5:
                # Corrección suave pero efectiva
                correccion_giro = error_giroscopio * 1.5
            else:
                correccion_giro = 0
            
            # ===== CORRECCIÓN TOTAL =====
            # La línea tiene prioridad, el giroscopio solo ayuda
            correccion_total = correccion_linea + (correccion_giro * 0.5)
            
            # ===== POTENCIAS BASE =====
            # Invertimos la corrección: positivo = derecha más rápido
            potencia_izq_base = velocidad_actual + correccion_total
            potencia_der_base = velocidad_actual - correccion_total
            
            # ===== APLICAR SESGO DE HARDWARE =====
            # Esto compensa diferencias mecánicas entre motores
            potencia_izq = potencia_izq_base * sesgo_izquierdo
            potencia_der = potencia_der_base * sesgo_derecho
            
            # ===== LÍMITES DE SEGURIDAD =====
            potencia_izq = max(-1000, min(1000, potencia_izq))
            potencia_der = max(-1000, min(1000, potencia_der))
            
            # ===== COMPORTAMIENTO MAGNÉTICO =====
            # Si estamos muy centrados (error pequeño), vamos a máxima velocidad magnética
            if abs(error_linea) < 5 and contador_estable > 3:
                # Modo "imán" - máxima adherencia
                # Ajuste fino para mantener el centro perfecto
                if error_linea != 0:
                    # Corrección microscópica
                    potencia_izq += error_linea * 2
                    potencia_der -= error_linea * 2
            
            # ===== APLICAR A MOTORES =====
            self.motor_izquierdo.run(int(potencia_izq))
            self.motor_derecho.run(int(potencia_der))
            
            # ===== ACTUALIZAR VARIABLES =====
            last_error = error_linea
            last_time.reset()
            
            # Loop rápido para respuesta inmediata
            wait(5)
        
        # ===== FRENADO SUAVE =====
        print("🛑 Finalizando seguidor magnético")
        self.motor_izquierdo.brake()
        self.motor_derecho.brake()
        
        # Pequeña pausa para estabilizar
        wait(50)

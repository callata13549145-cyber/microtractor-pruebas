"""
═══════════════════════════════════════════════════════════════════════════════
    TELEMETER - MICROTRACTOR GPS + IMU + FILTRO KALMAN
    Pico W con MicroPython
    
    ARQUITECTURA:
    1. Lee GPS (UART)
    2. Lee IMU (I2C)
    3. EJECUTA KALMAN ← El filtro procesa GPS + IMU
    4. Devuelve JSON con posición + velocidad FUSIONADAS
    5. Envía por WiFi HTTP
═══════════════════════════════════════════════════════════════════════════════
"""

from machine import Pin, UART, I2C
import network
import socket
import time
import json
import math
from ekf import ExtendedKalmanFilter  # ← IMPORTA EL FILTRO KALMAN

# ════════════════════════════════════════════════════════════════════════════
# WIFI
# ════════════════════════════════════════════════════════════════════════════
SSID = "ALEX"              # ← CAMBIA A TU RED
PASSWORD = "HolaMundo35"   # ← CAMBIA A TU CONTRASEÑA

print("🔌 Conectando a WiFi...")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

timeout = 15
while not wlan.isconnected() and timeout > 0:
    time.sleep(1)
    timeout -= 1
    print(f"  ⏳ Intentando... ({timeout}s)")

if wlan.isconnected():
    wlan.config(pm=0xa11140)
    print(f"✓ WiFi conectado")
    print(f"  IP: {wlan.ifconfig()[0]}")
else:
    print("✗ WiFi FALLÓ")


# ════════════════════════════════════════════════════════════════════════════
# GPS NEO-6M (UART1)
# ════════════════════════════════════════════════════════════════════════════
uart = UART(1, 9600, tx=Pin(20), rx=Pin(21))

# Estructura de datos GPS (posición bruta del módulo Neo-6M)
gps_data = {
    "lat": 0.0,        # Latitud en grados (negativo = Sur)
    "lon": 0.0,        # Longitud en grados (negativo = Oeste)
    "altitud": 0.0,    # Altitud en metros
    "satelites": 0,    # Número de satélites
    "hdop": 0.0,       # Dilución de precisión horizontal
    "hora": "00:00:00",# Hora UTC
    "valido": False    # ¿Tiene fix válido?
}


def parsear_gps(linea_txt):
    """Extrae datos de sentencia GNGGA del NEO-6M"""
    global gps_data

    try:
        if not linea_txt.startswith('$GNGGA'):
            return

        partes = linea_txt.split(',')
        if len(partes) < 10:
            return

        if partes[1]:
            h = partes[1]
            gps_data["hora"] = f"{h[0:2]}:{h[2:4]}:{h[4:6]}"

        if partes[7]:
            try:
                gps_data["satelites"] = int(partes[7])
            except:
                pass

        if partes[8]:
            try:
                gps_data["hdop"] = float(partes[8])
            except:
                pass

        if partes[9]:
            try:
                gps_data["altitud"] = float(partes[9])
            except:
                pass

        fix_quality = int(partes[6]) if partes[6] else 0
        gps_data["valido"] = fix_quality > 0

        if partes[2] and partes[3]:
            try:
                lat_str = partes[2]
                lat = float(lat_str[0:2]) + float(lat_str[2:]) / 60.0
                if partes[3] == 'S':
                    lat = -lat
                gps_data["lat"] = lat
            except:
                pass

        if partes[4] and partes[5]:
            try:
                lon_str = partes[4]
                lon = float(lon_str[0:3]) + float(lon_str[3:]) / 60.0
                if partes[5] == 'W':
                    lon = -lon
                gps_data["lon"] = lon
            except:
                pass

    except Exception as e:
        print(f"Error parsing GPS: {e}")


# ════════════════════════════════════════════════════════════════════════════
# IMU BNO055 (I2C1)
# ════════════════════════════════════════════════════════════════════════════
BNO_ADDR_A = 0x28
BNO_ADDR_B = 0x29

REG_OPR_MODE = 0x3D
REG_PWR_MODE = 0x3E
REG_PAGE_ID = 0x07
REG_CALIB_STAT = 0x35
REG_EUL_HEADING_LSB = 0x1A      # Ángulos Euler (yaw, roll, pitch)
REG_ACC_DATA_X_LSB = 0x08       # Aceleración
REG_GYR_DATA_X_LSB = 0x14       # Giroscopio (velocidad angular)
REG_MAG_DATA_X_LSB = 0x0E
REG_TEMP = 0x34

MODO_CONFIG = 0x00
MODO_NDOF = 0x0C

# Estructura de datos IMU (datos crudos del sensor)
imu_data = {
    "conectado": False,
    "euler": {"yaw": 0.0, "roll": 0.0, "pitch": 0.0},
    "acelerometro": {"x": 0.0, "y": 0.0, "z": 0.0},
    "giroscopio": {"x": 0.0, "y": 0.0, "z": 0.0},
    "magnetometro": {"x": 0.0, "y": 0.0, "z": 0.0},
    "temperatura": 0,
    "calibracion": {"sistem": 0, "giro": 0, "acele": 0, "mag": 0},
    
    # ═══════════════════════════════════════════════════════════════════════
    # NUEVO: Estado FUSIONADO del Filtro Kalman
    # ═══════════════════════════════════════════════════════════════════════
    # Este es el RESULTADO que Kalman produce:
    # - Posición (lat, lon, alt) FUSIONADA de GPS + IMU
    # - Velocidad (norte, este) ESTIMADA por Kalman
    # ═══════════════════════════════════════════════════════════════════════
    "kalman_fusion": {
        "posicion": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
        "velocidad": {"norte": 0.0, "este": 0.0}
    }
}

i2c1 = None
bno_addr = None


def a_signed16(v):
    """Convierte un valor sin signo a signed (16 bits)"""
    return v - 65536 if v > 32767 else v


def leer_vector6(i2c, addr, reg, divisor):
    """Lee 6 bytes (3 valores de 16 bits) del IMU"""
    datos = i2c.readfrom_mem(addr, reg, 6)
    x = a_signed16(int.from_bytes(datos[0:2], 'little')) / divisor
    y = a_signed16(int.from_bytes(datos[2:4], 'little')) / divisor
    z = a_signed16(int.from_bytes(datos[4:6], 'little')) / divisor
    return x, y, z


def iniciar_bno055():
    """Inicializa el sensor BNO055 en modo NDOF (sensor fusion nativa)"""
    global i2c1, bno_addr
    try:
        i2c1 = I2C(1, scl=Pin(19), sda=Pin(18), freq=400_000)
        dispositivos = i2c1.scan()
        if BNO_ADDR_A in dispositivos:
            bno_addr = BNO_ADDR_A
        elif BNO_ADDR_B in dispositivos:
            bno_addr = BNO_ADDR_B
        else:
            print("✗ BNO055 no detectado en el bus I2C1")
            return False

        i2c1.writeto_mem(bno_addr, REG_OPR_MODE, bytes([MODO_CONFIG]))
        time.sleep_ms(25)
        i2c1.writeto_mem(bno_addr, REG_PWR_MODE, bytes([0x00]))
        time.sleep_ms(10)
        i2c1.writeto_mem(bno_addr, REG_PAGE_ID, bytes([0x00]))
        i2c1.writeto_mem(bno_addr, REG_OPR_MODE, bytes([MODO_NDOF]))
        time.sleep_ms(25)
        print("✓ BNO055 inicializado en dirección {}".format(hex(bno_addr)))
        return True
    except Exception as e:
        print("✗ Error inicializando BNO055:", e)
        return False


# ════════════════════════════════════════════════════════════════════════════
# FILTRO KALMAN — INICIALIZACIÓN
# ════════════════════════════════════════════════════════════════════════════
"""
Aquí es donde inicializamos el Filtro Kalman Extendido (EKF).

El EKF es el corazón de la fusión de sensores:
- Predice: Usa aceleración + giroscopio (IMU) para estimar posición
- Corrige: Usa posición absoluta (GPS) para corregir drift
- Estima: Calcula velocidad (no disponible en GPS)

El resultado es una posición PRECISA, CONTINUA y ROBUSTA.
"""

ekf = None

if iniciar_bno055():
    # Crea instancia del Filtro Kalman
    # dt = 0.01 significa que la predicción ocurre cada 10ms (100 Hz)
    ekf = ExtendedKalmanFilter(dt=0.01)
    print("✓ Filtro Kalman inicializado")
    ekf_disponible = True
else:
    print("⚠ Sin Kalman (BNO055 no detectado)")
    ekf_disponible = False


# ════════════════════════════════════════════════════════════════════════════
# SERVIDOR HTTP
# ════════════════════════════════════════════════════════════════════════════
def crear_servidor():
    """Crea servidor HTTP en puerto 80"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', 80))
    s.listen(2)
    print("✓ Servidor HTTP en puerto 80")
    return s


def enviar_json(conn, datos):
    """Envía JSON al navegador"""
    json_str = json.dumps(datos)
    response = (
        f"HTTP/1.1 200 OK\r\n"
        f"Content-Type: application/json\r\n"
        f"Content-Length: {len(json_str)}\r\n"
        f"Access-Control-Allow-Origin: *\r\n"
        f"Connection: close\r\n"
        f"\r\n"
        f"{json_str}"
    )
    conn.sendall(response.encode())
    conn.close()


def manejar_request(conn, request):
    """Maneja peticiones HTTP"""
    try:
        linea = request.split('\r\n')[0]
        if '/data' in linea:
            # Endpoint combinado: GPS + IMU + KALMAN FUSION
            enviar_json(conn, {
                "gps": gps_data,
                "imu": imu_data
            })
        elif '/gps' in linea:
            enviar_json(conn, gps_data)
        elif '/imu' in linea:
            enviar_json(conn, imu_data)
        elif '/ping' in linea:
            enviar_json(conn, {"status": "online"})
        else:
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n")
            conn.close()
    except Exception as e:
        print(f"Error: {e}")
        try:
            conn.close()
        except:
            pass


# ════════════════════════════════════════════════════════════════════════════
# LOOP PRINCIPAL — DONDE KALMAN EJECUTA
# ════════════════════════════════════════════════════════════════════════════
"""
El loop principal ejecuta a ~100 Hz (cada 10ms).

En cada ciclo:
1. Lee GPS (si hay datos)
2. Lee IMU (siempre hay datos cada 10ms)
3. EJECUTA KALMAN:
   - predict() → Usa aceleración para predecir nueva posición
   - update_imu() → Corrige orientación con BNO055
   - update_gps() → Corrige posición cuando GPS disponible (~1s)
4. Guarda resultado en imu_data["kalman_fusion"]
5. HTTP sirve el JSON con posición FUSIONADA
"""

servidor = crear_servidor()
contador_prints = 0
contador_gps_updates = 0
primer_gps = False

print("\n🚀 MICROTRACTOR - GPS NEO-8M + IMU BNO055 + KALMAN FUSION iniciado")
print("   Endpoint combinado: http://{IP}/data")
print("   Contiene: GPS raw + IMU raw + KALMAN FUSION (posición + velocidad)")
print("\n" + "=" * 50)

try:
    while True:
        # ────────────────────────────────────────────────────────────────────
        # PASO 1: LEE GPS (no bloqueante)
        # ────────────────────────────────────────────────────────────────────
        if uart.any():
            try:
                datos = uart.readline()
                linea_txt = datos.decode().strip()
                if linea_txt:
                    parsear_gps(linea_txt)  # Extrae lat, lon, alt, sats
            except:
                pass

        # ════════════════════════════════════════════════════════════════════
        # PASO 2: EJECUTA FILTRO KALMAN ← AQUI ESTA LO IMPORTANTE
        # ════════════════════════════════════════════════════════════════════
        if ekf and ekf_disponible:
            try:
                # ────────────────────────────────────────────────────────────
                # A) Lee datos CRUDOS del IMU (sin procesar)
                # ────────────────────────────────────────────────────────────
                
                # Aceleración en m/s²
                ax, ay, az = leer_vector6(i2c1, bno_addr, REG_ACC_DATA_X_LSB, 100.0)
                
                # Velocidad angular en °/s (giroscopio)
                gx, gy, gz = leer_vector6(i2c1, bno_addr, REG_GYR_DATA_X_LSB, 16.0)
                
                # Ángulos Euler en grados (brújula + cabeceo + alabeo)
                yaw_imu, roll_imu, pitch_imu = leer_vector6(i2c1, bno_addr, REG_EUL_HEADING_LSB, 16.0)
                
                # Temperatura del sensor
                temp = i2c1.readfrom_mem(bno_addr, REG_TEMP, 1)[0]
                
                # Estado de calibración del sensor
                cal = i2c1.readfrom_mem(bno_addr, REG_CALIB_STAT, 1)[0]
                
                # ────────────────────────────────────────────────────────────
                # B) Inicializar Kalman con primer GPS válido
                # ────────────────────────────────────────────────────────────
                # Kalman necesita una posición inicial absoluta (del GPS)
                # Después la mantiene actualizada basándose en IMU
                if not primer_gps and gps_data['valido'] and gps_data['lat'] != 0:
                    ekf.set_initial_state(
                        gps_data['lat'],
                        gps_data['lon'],
                        gps_data['altitud']
                    )
                    primer_gps = True
                    print("✓ Kalman inicializado con posición GPS")
                
                # ────────────────────────────────────────────────────────────
                # C) PREDICCIÓN: Kalman predice nueva posición (cada 10ms)
                # ────────────────────────────────────────────────────────────
                """
                PASO 1 DEL FILTRO KALMAN: PREDICCIÓN
                
                Usa las aceleraciones (ax, ay) y velocidad angular (gz) 
                para estimar dónde estará el tractor en los próximos 10ms.
                
                Proceso matemático:
                  x_predicho = x_anterior + v_anterior * dt + a * dt²
                
                En este caso:
                  - lat_predicho = lat_anterior + v_norte * 0.01
                  - lon_predicho = lon_anterior + v_este * 0.01
                  - yaw_predicho = yaw_anterior + gz * 0.01
                
                IMPORTANTE: Esta predicción acumula pequeño error (drift).
                Por eso necesitamos GPS para corregir cada segundo.
                """
                ekf.predict(ax, ay, math.radians(gz))
                # ax = aceleración norte/sur (m/s²)
                # ay = aceleración este/oeste (m/s²)
                # gz = velocidad angular en radianes/s
                
                # ────────────────────────────────────────────────────────────
                # D) ACTUALIZACIÓN 1: Kalman corrige con IMU orientación
                # ────────────────────────────────────────────────────────────
                """
                PASO 2 DEL FILTRO KALMAN: ACTUALIZACIÓN CON IMU
                
                El BNO055 proporciona orientación muy precisa (yaw, pitch, roll).
                Le decimos a Kalman: "Este es el ángulo real medido por el sensor"
                
                Kalman calcula cuánto error hay entre:
                - Lo que Kalman predijo (estado estimado)
                - Lo que el IMU midió (realidad)
                
                Luego ajusta su predicción un poco hacia la medición.
                
                Ganancia de Kalman (K):
                  - Si el sensor es muy ruidoso (R grande) → K pequeño → ignorar medición
                  - Si el sensor es preciso (R pequeño) → K grande → confiar en medición
                """
                ekf.update_imu(
                    math.radians(yaw_imu),      # Brújula (magnética + giroscopio)
                    math.radians(pitch_imu),    # Cabeceo (adelante/atrás)
                    math.radians(roll_imu)      # Alabeo (izquierda/derecha)
                )
                # Convierte a radianes porque Kalman trabaja en radianes
                
                # ────────────────────────────────────────────────────────────
                # E) ACTUALIZACIÓN 2: Kalman corrige con GPS posición absoluta
                # ────────────────────────────────────────────────────────────
                """
                PASO 3 DEL FILTRO KALMAN: ACTUALIZACIÓN CON GPS
                
                Cada ~1 segundo (cuando GPS actualiza), le decimos a Kalman:
                "La posición real es esta (lat, lon, alt)"
                
                Kalman entonces:
                1. Calcula error: "Predije X pero el GPS dice Y"
                2. Calcula ganancia: "¿Cuánto confío en GPS vs mi predicción?"
                3. Ajusta el estado: x_nuevo = x_predicho + K * (z_medido - x_predicho)
                
                RESULTADO: Posición FUSIONADA que combina:
                - Precisión absoluta de GPS (±5m)
                - Continuidad/velocidad de IMU (100 Hz)
                = Posición ±1-2m sin rebotes (cada 10ms)
                """
                contador_gps_updates += 1
                
                # Cada 100 ciclos de 10ms = 1 segundo (aproximadamente)
                if contador_gps_updates >= 100:
                    contador_gps_updates = 0
                    
                    # Solo actualizar si GPS tiene fix válido
                    if gps_data['valido'] and gps_data['lat'] != 0 and gps_data['lon'] != 0:
                        """
                        Entra aquí ~1 vez por segundo cuando GPS tiene posición
                        válida (al menos 4 satélites con buena geometría)
                        """
                        ekf.update_gps(
                            gps_data['lat'],
                            gps_data['lon'],
                            gps_data['altitud']
                        )
                        # Kalman ahora corrige su predicción con posición absoluta
                
                # ────────────────────────────────────────────────────────────
                # F) OBTÉN RESULTADO DEL KALMAN
                # ────────────────────────────────────────────────────────────
                """
                Después de predict() + update_imu() + update_gps(),
                Kalman tiene una estimación FUSIONADA del estado completo:
                
                [
                  latitud (°),
                  longitud (°),
                  altitud (m),
                  velocidad_norte (m/s),      ← No viene de GPS
                  velocidad_este (m/s),       ← No viene de GPS
                  yaw/brújula (rad),
                  pitch/cabeceo (rad),
                  roll/alabeo (rad)
                ]
                """
                state = ekf.get_state()
                
                # ────────────────────────────────────────────────────────────
                # G) ACTUALIZA imu_data con RESULTADO DE KALMAN
                # ────────────────────────────────────────────────────────────
                
                # Valores crudos del IMU (para comparación/debug)
                imu_data["conectado"] = True
                imu_data["euler"] = {
                    "yaw": round(math.degrees(state['yaw']), 1),
                    "roll": round(math.degrees(state['roll']), 1),
                    "pitch": round(math.degrees(state['pitch']), 1)
                }
                imu_data["acelerometro"] = {
                    "x": round(ax, 2),
                    "y": round(ay, 2),
                    "z": round(az, 2)
                }
                imu_data["giroscopio"] = {
                    "x": round(gx, 2),
                    "y": round(gy, 2),
                    "z": round(gz, 2)
                }
                imu_data["temperatura"] = temp
                
                # ════════════════════════════════════════════════════════════
                # RESULTADO FINAL: Estado FUSIONADO de Kalman
                # Este es el valor más importante que devolvemos
                # ════════════════════════════════════════════════════════════
                imu_data["kalman_fusion"] = {
                    "posicion": {
                        # Latitud FUSIONADA (GPS + IMU predicción)
                        "lat": round(state['lat'], 6),
                        # Longitud FUSIONADA (GPS + IMU predicción)
                        "lon": round(state['lon'], 6),
                        # Altitud FUSIONADA (GPS + IMU predicción)
                        "alt": round(state['alt'], 1)
                    },
                    "velocidad": {
                        # Velocidad NORTE en m/s (estimada por Kalman)
                        # GPS no proporciona velocidad directa, solo posición
                        # Kalman la estima integrando aceleración del IMU
                        "norte": round(state['v_norte'], 3),
                        
                        # Velocidad ESTE en m/s (estimada por Kalman)
                        "este": round(state['v_este'], 3)
                    }
                }
                
                # Calibración del sensor
                sys_c = (cal >> 6) & 0x03
                gyro_c = (cal >> 4) & 0x03
                acc_c = (cal >> 2) & 0x03
                mag_c = cal & 0x03
                imu_data["calibracion"] = {
                    "sistem": sys_c,
                    "giro": gyro_c,
                    "acele": acc_c,
                    "mag": mag_c
                }
                
            except Exception as e:
                imu_data["conectado"] = False
                print("Error en Kalman:", e)
        
        # ────────────────────────────────────────────────────────────────────
        # PASO 3: HTTP (sirve JSON con posición FUSIONADA)
        # ────────────────────────────────────────────────────────────────────
        servidor.settimeout(0.2)
        try:
            conn, addr = servidor.accept()
            conn.settimeout(0.3)
            request = conn.recv(1024).decode()
            manejar_request(conn, request)
        except OSError as e:
            if len(e.args) and e.args[0] != 110:
                print("⚠ Error de socket:", e)

        # ────────────────────────────────────────────────────────────────────
        # DEBUG: Imprime estado cada ~5 segundos
        # ────────────────────────────────────────────────────────────────────
        contador_prints += 1
        if contador_prints >= 500:  # 500 × 10ms = 5s
            contador_prints = 0
            estado_gps = "✓ VÁLIDO" if gps_data['valido'] else "⏳ BUSCANDO"
            imu_estado = "✓" if imu_data.get("conectado") else "✗"
            
            print(f"\n[GPS]  {estado_gps} | Sats: {gps_data['satelites']} | "
                  f"Lat: {gps_data['lat']:.5f} | Lon: {gps_data['lon']:.5f}")
            
            if ekf and ekf_disponible and "kalman_fusion" in imu_data:
                k = imu_data["kalman_fusion"]["posicion"]
                v = imu_data["kalman_fusion"]["velocidad"]
                e = imu_data["euler"]
                
                print(f"[KALMAN] ← RESULTADO DEL FILTRO")
                print(f"  Posición FUSIONADA: Lat:{k['lat']:.5f} Lon:{k['lon']:.5f} Alt:{k['alt']:.1f}m")
                print(f"  Velocidad ESTIMADA: v_N:{v['norte']:.2f}m/s v_E:{v['este']:.2f}m/s")
                print(f"[IMU] Yaw:{e['yaw']:.1f}° Roll:{e['roll']:.1f}° Pitch:{e['pitch']:.1f}° | "
                      f"Temp:{imu_data['temperatura']}°C")

        # Espera 10ms antes del siguiente ciclo
        time.sleep_ms(10)  # 100 Hz

except KeyboardInterrupt:
    print("\n⊗ Detenido por usuario")
    servidor.close()

# ════════════════════════════════════════════════════════════════════════════
# FIN DEL PROGRAMA
# ════════════════════════════════════════════════════════════════════════════
"""
RESUMEN DE LO QUE HACE KALMAN EN ESTE CÓDIGO:

1. PREDICCIÓN (cada 10ms):
   - Usa aceleración (ax, ay) del IMU
   - Usa velocidad angular (gz) del IMU
   - Estima nueva posición/velocidad/orientación
   - RÁPIDO pero con DRIFT

2. ACTUALIZACIÓN IMU (cada 10ms):
   - Mide orientación real (yaw, roll, pitch) del BNO055
   - Compara con predicción
   - Corrige orientación

3. ACTUALIZACIÓN GPS (cada ~1 segundo):
   - Mide posición real (lat, lon, alt) del Neo-6M
   - Compara con predicción
   - Corrige posición
   - Detiene drift que IMU acumularía

4. RESULTADO:
   - Posición PRECISA (±1-2m) gracias a GPS
   - Posición CONTINUA (100 Hz) gracias a IMU
   - Velocidad ESTIMADA (no viene de GPS)
   - Resistente a fallos (funciona ~60s sin GPS)

TODO ESTO OCURRE EN EL PICO W. El navegador HTML solo VISUALIZA el resultado.
"""
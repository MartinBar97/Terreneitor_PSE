#!/usr/bin/env python3
"""
Robot Seguidor de Líneas - VERSION SIMPLE
Solo sensores infrarrojos + Control web con PID ajustable

CARACTERÍSTICAS:
- Solo sensores de línea (infrarojos)
- Control PID ajustable en tiempo real desde navegador
- Sin sensor ultrasónico
- Interfaz web simple
"""

import sys
import os
import time
import threading
import json
from collections import deque
import asyncio
import websockets
from http.server import HTTPServer, SimpleHTTPRequestHandler
import socket

# Agregar el directorio Server al path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Server'))

from infrared import Infrared
from motor import Ordinary_Car


# ==================== CONFIGURACIÓN ====================

# Puertos del servidor web
HTTP_PORT = 8080  # Servidor HTTP para el HTML
WS_PORT = 8765    # Servidor WebSocket para datos en tiempo real

# Velocidades
BASE_SPEED = 800
BASE_SPEED_CURVE = 500

# PID (valores por defecto)
KP = 400.0
KI = 30.0
KD = 180.0
MAX_CORRECTION = 1500

# Otros parámetros
RAMP_RATE = 50
SAMPLE_TIME = 0.015
QUEUE_SIZE = 8
CURVE_DETECTION_THRESHOLD = 0.7

# Mapeo de patrones de sensores a error
SENSOR_ERROR_MAP = {
    0b000: None,     # Fuera de línea
    0b001: 1.0,      # Derecha
    0b010: 0.0,      # Centro
    0b011: 0.6,      # Centro+derecha
    0b100: -1.0,     # Izquierda
    0b101: 0.0,      # Izq+Der
    0b110: -0.6,     # Centro+izq
    0b111: 0.0       # Todos
}


# ==================== SERVIDOR WEB ====================

class WebMonitorServer:
    """Servidor HTTP + WebSocket para el monitor web"""
    
    def __init__(self, robot_data_callback):
        self.robot_data_callback = robot_data_callback
        self.websocket_clients = set()
        self.http_server = None
        self.ws_server = None
        self.running = False
        
        # Obtener IP del robot
        self.ip_address = self._get_ip_address()
        
        print(f"✓ Servidor web configurado")
        print(f"  Abre en navegador: http://{self.ip_address}:{HTTP_PORT}")
    
    def _get_ip_address(self):
        """Obtiene la IP del robot en la red local"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"
    
    def start(self):
        """Inicia servidores HTTP y WebSocket"""
        self.running = True
        
        # Thread para servidor HTTP
        http_thread = threading.Thread(target=self._start_http_server, daemon=True)
        http_thread.start()
        
        # Thread para servidor WebSocket
        ws_thread = threading.Thread(target=self._start_ws_server, daemon=True)
        ws_thread.start()
        
        print("✓ Servidores web iniciados")
    
    def _start_http_server(self):
        """Servidor HTTP para servir el HTML"""
        try:
            # Cambiar al directorio donde está monitor.html
            os.chdir(os.path.dirname(os.path.abspath(__file__)))
            
            handler = SimpleHTTPRequestHandler
            self.http_server = HTTPServer(('0.0.0.0', HTTP_PORT), handler)
            
            print(f"✓ HTTP server en puerto {HTTP_PORT}")
            self.http_server.serve_forever()
        except Exception as e:
            print(f"Error HTTP server: {e}")
    
    def _start_ws_server(self):
        """Servidor WebSocket para datos en tiempo real"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            async def handler(websocket, path):
                self.websocket_clients.add(websocket)
                print(f"✓ Cliente web conectado ({len(self.websocket_clients)} total)")
                
                try:
                    async for message in websocket:
                        # Recibir comandos del navegador
                        self._handle_command(message)
                except websockets.exceptions.ConnectionClosed:
                    pass
                finally:
                    self.websocket_clients.discard(websocket)
                    print(f"  Cliente web desconectado ({len(self.websocket_clients)} restantes)")
            
            start_server = websockets.serve(handler, '0.0.0.0', WS_PORT)
            loop.run_until_complete(start_server)
            print(f"✓ WebSocket server en puerto {WS_PORT}")
            loop.run_forever()
            
        except Exception as e:
            print(f"Error WS server: {e}")
    
    def _handle_command(self, message):
        """Maneja comandos recibidos del navegador"""
        try:
            data = json.loads(message)
            command = data.get('command')
            
            if command == 'emergency_stop':
                emergency_active = data.get('active', False)
                self.robot_data_callback('emergency', emergency_active)
            
            elif command == 'update_pid':
                param = data.get('param')
                value = data.get('value')
                self.robot_data_callback('pid_update', {'param': param, 'value': value})
            
            elif command == 'reset_pid':
                kp = data.get('kp')
                ki = data.get('ki')
                kd = data.get('kd')
                self.robot_data_callback('pid_reset', {'kp': kp, 'ki': ki, 'kd': kd})
        except Exception as e:
            print(f"Error manejando comando: {e}")
    
    async def broadcast_data(self, data):
        """Envía datos a todos los clientes web conectados"""
        if self.websocket_clients:
            message = json.dumps(data)
            websockets_to_remove = set()
            
            for ws in self.websocket_clients:
                try:
                    await ws.send(message)
                except:
                    websockets_to_remove.add(ws)
            
            # Limpiar websockets muertos
            self.websocket_clients -= websockets_to_remove
    
    def send_data_sync(self, data):
        """Versión síncrona para enviar datos"""
        if self.websocket_clients:
            try:
                loop = asyncio.new_event_loop()
                loop.run_until_complete(self.broadcast_data(data))
                loop.close()
            except:
                pass


# ==================== CLASE SENSOR INFRARROJO ====================

class LineSensorThread:
    """Gestión de sensores de línea con filtrado"""
    
    def __init__(self):
        self.infrared = Infrared()
        self.buffer = deque(maxlen=QUEUE_SIZE)
        
        self.current_value = 0
        self.current_error = 0.0
        
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
        print("✓ Sensores IR inicializados")
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("✓ Thread sensores iniciado")
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.infrared.close()
        print("✓ Thread sensores detenido")
    
    def _read_loop(self):
        while self.running:
            try:
                sensor_value = self.infrared.read_all_infrared()
                self.buffer.append(sensor_value)
                
                if len(self.buffer) > 0:
                    # Filtro de moda (valor más frecuente)
                    counts = {}
                    for val in self.buffer:
                        counts[val] = counts.get(val, 0) + 1
                    filtered_value = max(counts, key=counts.get)
                    
                    error = SENSOR_ERROR_MAP.get(filtered_value, None)
                    
                    with self.lock:
                        self.current_value = filtered_value
                        self.current_error = error
                
                time.sleep(SAMPLE_TIME)
                
            except Exception as e:
                print(f"Error sensores: {e}")
                time.sleep(0.1)
    
    def get_error(self):
        with self.lock:
            return self.current_error
    
    def get_raw_value(self):
        with self.lock:
            return self.current_value
    
    def get_individual_sensors(self):
        """Devuelve estado de cada sensor (para visualización web)"""
        value = self.get_raw_value()
        return {
            'left': bool(value & 0b100),
            'center': bool(value & 0b010),
            'right': bool(value & 0b001)
        }
    
    def is_sharp_curve(self):
        """Detecta si está en una curva cerrada"""
        error = self.get_error()
        return abs(error) >= CURVE_DETECTION_THRESHOLD if error is not None else False


# ==================== CLASE CONTROLADOR PID ====================

class SmoothPIDController:
    """PID optimizado para movimientos suaves con ajuste en tiempo real"""
    
    def __init__(self, kp, ki, kd, sample_time):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sample_time = sample_time
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.last_output = 0.0
        
        self.integral_max = 300.0
        self.integral_min = -300.0
        
        self.derivative_filter = deque(maxlen=3)
        
        self.lock = threading.Lock()
        
        print(f"✓ PID suave (Kp={kp}, Ki={ki}, Kd={kd})")
    
    def compute(self, error):
        with self.lock:
            current_time = time.time()
            dt = current_time - self.last_time
            
            if dt < 0.001:
                return self.last_output
            
            # Término Proporcional
            p_term = self.kp * error
            
            # Término Integral con anti-windup
            self.integral += error * dt
            self.integral = max(self.integral_min, min(self.integral_max, self.integral))
            i_term = self.ki * self.integral
            
            # Término Derivativo con filtrado
            raw_derivative = (error - self.last_error) / dt
            self.derivative_filter.append(raw_derivative)
            filtered_derivative = sum(self.derivative_filter) / len(self.derivative_filter)
            d_term = self.kd * filtered_derivative
            
            # Salida total
            output = p_term + i_term + d_term
            output = max(-MAX_CORRECTION, min(MAX_CORRECTION, output))
            
            self.last_error = error
            self.last_time = current_time
            self.last_output = output
            
            return output
    
    def reset(self):
        with self.lock:
            self.integral = 0.0
            self.last_error = 0.0
            self.last_time = time.time()
            self.last_output = 0.0
            self.derivative_filter.clear()
    
    def update_parameter(self, param, value):
        """Actualiza un parámetro PID en tiempo real"""
        with self.lock:
            if param == 'kp':
                self.kp = value
                print(f"  PID actualizado: Kp = {value}")
            elif param == 'ki':
                self.ki = value
                print(f"  PID actualizado: Ki = {value}")
            elif param == 'kd':
                self.kd = value
                print(f"  PID actualizado: Kd = {value}")
    
    def set_all_parameters(self, kp, ki, kd):
        """Actualiza todos los parámetros PID a la vez"""
        with self.lock:
            self.kp = kp
            self.ki = ki
            self.kd = kd
            print(f"  PID actualizado: Kp={kp}, Ki={ki}, Kd={kd}")


# ==================== CLASE CONTROL DE MOTORES ====================

class SmoothMotorControl:
    """Control de motores con rampa de aceleración suave"""
    
    def __init__(self):
        self.motor = Ordinary_Car()
        
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
        self.target_speeds = {'FL': 0, 'BL': 0, 'FR': 0, 'BR': 0}
        self.current_speeds = {'FL': 0, 'BL': 0, 'FR': 0, 'BR': 0}
        
        print("✓ Motores inicializados")
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
        print("✓ Thread motores iniciado")
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.motor.set_motor_model(0, 0, 0, 0)
        self.motor.close()
        print("✓ Thread motores detenido")
    
    def _control_loop(self):
        while self.running:
            try:
                with self.lock:
                    targets = self.target_speeds.copy()
                
                changed = False
                for motor in self.current_speeds:
                    diff = targets[motor] - self.current_speeds[motor]
                    
                    if abs(diff) > RAMP_RATE:
                        self.current_speeds[motor] += RAMP_RATE if diff > 0 else -RAMP_RATE
                        changed = True
                    else:
                        self.current_speeds[motor] = targets[motor]
                        if diff != 0:
                            changed = True
                
                if changed:
                    self.motor.set_motor_model(
                        int(self.current_speeds['FL']),
                        int(self.current_speeds['BL']),
                        int(self.current_speeds['FR']),
                        int(self.current_speeds['BR'])
                    )
                
                time.sleep(SAMPLE_TIME)
                
            except Exception as e:
                print(f"Error motores: {e}")
                time.sleep(0.1)
    
    def set_motion(self, forward_speed, correction):
        """Configura movimiento con corrección"""
        with self.lock:
            left = int(max(-4095, min(4095, forward_speed - correction)))
            right = int(max(-4095, min(4095, forward_speed + correction)))
            
            self.target_speeds['FL'] = left
            self.target_speeds['BL'] = left
            self.target_speeds['FR'] = right
            self.target_speeds['BR'] = right
    
    def emergency_stop(self):
        """Parada de emergencia inmediata"""
        with self.lock:
            self.target_speeds = {'FL': 0, 'BL': 0, 'FR': 0, 'BR': 0}
        self.motor.set_motor_model(0, 0, 0, 0)


# ==================== ROBOT CON MONITOR WEB ====================

class SimpleLineFollower:
    """Robot seguidor de líneas simple con monitor web"""
    
    def __init__(self):
        print("\n" + "="*60)
        print("  ROBOT SEGUIDOR DE LINEAS - VERSION SIMPLE")
        print("  Solo sensores IR + Control web PID")
        print("="*60 + "\n")
        
        # Componentes del robot
        self.line_sensors = LineSensorThread()
        self.pid = SmoothPIDController(KP, KI, KD, SAMPLE_TIME)
        self.motors = SmoothMotorControl()
        
        # Servidor web
        self.web_server = WebMonitorServer(self._handle_web_command)
        
        # Estado
        self.running = False
        self.emergency_stop_active = False
        self.main_thread = None
        self.last_valid_error = 0.0
        self.current_speed = 0
        self.current_correction = 0
        
        # Estadísticas
        self.stats = {
            'iterations': 0,
            'start_time': None
        }
    
    def _handle_web_command(self, command, value):
        """Maneja comandos desde el navegador"""
        if command == 'emergency':
            self.emergency_stop_active = value
            if value:
                print("\nPARADA DESDE NAVEGADOR")
                self.motors.emergency_stop()
        
        elif command == 'pid_update':
            param = value.get('param')
            val = value.get('value')
            self.pid.update_parameter(param, val)
        
        elif command == 'pid_reset':
            kp = value.get('kp')
            ki = value.get('ki')
            kd = value.get('kd')
            self.pid.set_all_parameters(kp, ki, kd)
            self.pid.reset()  # También resetear integral y derivada
    
    def start(self):
        print("Iniciando sistema...")
        
        # Iniciar componentes
        self.line_sensors.start()
        self.motors.start()
        self.web_server.start()
        
        time.sleep(1.0)
        
        self.pid.reset()
        self.running = True
        self.stats['start_time'] = time.time()
        
        self.main_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_thread.start()
        
        print("\n✓ Sistema activo")
        print("  Monitor web activo")
        print("  Presiona Ctrl+C para detener\n")
    
    def stop(self):
        print("\nDeteniendo...")
        self.running = False
        
        if self.main_thread:
            self.main_thread.join(timeout=1.0)
        
        self.motors.stop()
        self.line_sensors.stop()
        
        print("✓ Sistema detenido\n")
    
    def _main_loop(self):
        """Bucle principal + envío de datos al navegador"""
        while self.running:
            try:
                # Parada de emergencia desde web
                if self.emergency_stop_active:
                    time.sleep(0.1)
                    continue
                
                # Leer sensores
                error = self.line_sensors.get_error()
                if error is None:
                    error = self.last_valid_error
                else:
                    self.last_valid_error = error
                
                # Detectar curvas y ajustar velocidad
                is_curve = self.line_sensors.is_sharp_curve()
                self.current_speed = BASE_SPEED_CURVE if is_curve else BASE_SPEED
                
                # Calcular corrección PID
                self.current_correction = self.pid.compute(error)
                
                # Mover motores
                self.motors.set_motion(self.current_speed, self.current_correction)
                
                # Actualizar estadísticas
                self.stats['iterations'] += 1
                
                # Enviar datos al navegador cada 5 iteraciones
                if self.stats['iterations'] % 5 == 0:
                    self._send_web_update()
                
                time.sleep(SAMPLE_TIME)
                
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(0.1)
    
    def _send_web_update(self):
        """Envía actualización al navegador"""
        try:
            sensors = self.line_sensors.get_individual_sensors()
            
            data = {
                'sensor_left': sensors['left'],
                'sensor_center': sensors['center'],
                'sensor_right': sensors['right'],
                'error': self.last_valid_error if self.last_valid_error is not None else 0.0,
                'correction': self.current_correction,
                'speed': self.current_speed,
                'distance': 999,  # No hay sensor ultrasónico
                'iterations': self.stats['iterations']
            }
            
            self.web_server.send_data_sync(data)
        except:
            pass


# ==================== PROGRAMA PRINCIPAL ====================

def main():
    robot = None
    
    try:
        # Verificar directorio
        if not os.path.exists('Server/infrared.py'):
            print("❌ Error: Directorio 'Server' no encontrado")
            print("   Ejecuta desde: ~/Terreneitor_PSE")
            sys.exit(1)
        
        # Crear y ejecutar robot
        robot = SimpleLineFollower()
        robot.start()
        
        # Mantener ejecución
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\nCtrl+C detectado")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if robot:
            robot.stop()


if __name__ == "__main__":
    main()
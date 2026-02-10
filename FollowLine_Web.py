#!/usr/bin/env python3
import sys
import os
import time
import threading
import json
import asyncio
import websockets
import select
import tty
import termios
from http.server import HTTPServer, BaseHTTPRequestHandler
import socket

#IMPORTS DENDE FREENOVE
from Server.infrared import Infrared     
from Server.motor import Ordinary_Car     
#PORTOS PARA INTERCAMBIO DE DATOS (HTTP E SOCKETS)
HTTP_PORT = 8080  
WS_PORT = 8765   

BASE_SPEED = -500  #velocidade por defecto

#cada 3 segundos, aplicamos unha frenada no motor pequena para conseguir rectificar mellor
PULSE_EVERY = 3    
PULSE_TIME = 0.004 

# tradución do mapa de sensores
SENSOR_ERROR_MAP = {
    0b000: None,   
    0b001: 1.0,    #bastante a dereita
    0b010: 0.0,    # no centro
    0b011: 0.5,    # levemente a dereita
    0b100: -1.0,   # bastante a esquerda
    0b101: 0.0,    # cerca do centro
    0b110: -0.5,   # levemente a esquerda
    0b111: 0.0     # en liña ancha ou nun cruce
}

#INTERFACE HTML

#NESTA INTERFACE PODEMOS VER OS VALORES DOS SENSORES, O ESTADO DE CONEXIÓN CO ROBOT, E ACTUAR SOBRE EL MEDIANTE PARADAS USANDO A BARRA ESPACIADORA E AXUSTES DO PID
HTML_INTERFACE = HTML_INTERFACE = """
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <title>TERRENEITOR</title>
  <style>
    body { background:#0e0e0e; color:#00ff88; font-family:monospace; text-align:center; }
    .panel { display:inline-block; border:2px solid #333; border-radius:12px; padding:20px; margin-top:30px; width:380px; }
    .sensor { display:inline-block; width:40px; height:40px; margin:5px; border:1px solid #444; border-radius:6px; background:#111; }
    .sensor.active { background:#00ff88; box-shadow:0 0 10px #00ff88; }
    button { background:#aa0000; color:white; border:none; padding:12px 20px; font-size:16px; border-radius:8px; cursor:pointer; }
    button.active { background:#00aa00; }
    .slider { width:100%; }
    .value { float:right; }
  </style>
</head>
<body>

  <h1>-- TERRENEITOR --</h1>

  <div class="panel">
    <div id="status"> DESCONECTADO</div>
    <p>Iteraciones: <span id="it">0</span></p>
    <p>Error: <span id="err">0</span></p>

    <div>
      <div id="sL" class="sensor"></div>
      <div id="sC" class="sensor"></div>
      <div id="sR" class="sensor"></div>
    </div>

    <button id="stopBtn">PARAR</button>

    <h3>PID</h3>

    <label>Kp <span id="kpVal" class="value"></span></label>
    <input id="kp" type="range" min="0" max="1500" step="10" value="700" class="slider">

    <label>Ki <span id="kiVal" class="value"></span></label>
    <input id="ki" type="range" min="0" max="500" step="5" value="0" class="slider">

    <label>Kd <span id="kdVal" class="value"></span></label>
    <input id="kd" type="range" min="0" max="800" step="10" value="300" class="slider">
  </div>

  <script>
    let ws;
    let emergency = false;

    function connect() {
      ws = new WebSocket(`ws://${location.hostname}:8765`);
      ws.onopen = () => status.innerText = " CONECTADO";
      ws.onmessage = e => {
        const d = JSON.parse(e.data);
        it.innerText = d.iterations;
        err.innerText = d.error.toFixed(2);
        sL.className = d.sensor_left ? "sensor active" : "sensor";
        sC.className = d.sensor_center ? "sensor active" : "sensor";
        sR.className = d.sensor_right ? "sensor active" : "sensor";
      };
      ws.onclose = () => {
        status.innerText = "DESCONECTADO";
        setTimeout(connect, 1000);
      }
    }
    

    function sendPID() {
      ws.send(JSON.stringify({
        type: "pid",
        kp: Number(kp.value),
        ki: Number(ki.value),
        kd: Number(kd.value)
      }));
    }

    ["kp", "ki", "kd"].forEach(id => {
      let el = document.getElementById(id);
      document.getElementById(id + "Val").innerText = el.value;
      el.oninput = () => {
        document.getElementById(id + "Val").innerText = el.value;
        sendPID();
      }
    });

    stopBtn.onclick = () => {
      emergency = !emergency;
      ws.send(JSON.stringify({ type: "emergency", value: emergency }));
      stopBtn.innerText = emergency ? "REANUDAR" : " PARAR";
      stopBtn.className = emergency ? "active" : "";
    };

    connect();
  </script>
</body>
</html>
""" 

# CLASE PARA XESTIONAR AS CONEXIÓNS AO HTML
class RobotHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type","text/html")
        self.end_headers()
        self.wfile.write(HTML_INTERFACE.encode())
    def log_message(self,*a): pass 

#CLASE PRINCIPAL DO ROBOT QUE XESTIONA O SEU COMPORTAMENTO
class RobotSystem:
    def __init__(self):
        #INICIALIZACIÓN DO HARDWARE NO CONSTRUTOR
        self.ir = Infrared()
        self.car = Ordinary_Car()
        self.lock = threading.Lock() # EVITAMOS CONFLITOS NOS FÍOS
        self.clients = set()         # LISTA DE NAVEGADORES CONECTADOS AOS SOCKETS

        # PARÁMETROS PID POR DEFECTO
        self.kp, self.ki, self.kd = 700.0, 0.0, 300.0
        self.emergency = False       #POSIBEL PARADA RECIBIDA POR TECLADO DE SEGURIDADE
        self.running = True
        #VARIABLES SOBRE LECTURA DOS SENSORES E ITERACIÓNS PARA CONTROL DA SENSORIZACIÓN
        self.raw_sensor = 0
        self.current_error = 0.0
        self.iterations = 0
        self.current_data = {}

    # FÍO DE SENSORIZACIÓN
    def _sensor_thread(self):
        
        last=0
        #MENTRES ESTÁ CORRENDO O FÍO
        while self.running:
            raw=self.ir.read_all_infrared() # LESE O GPIO
            err=SENSOR_ERROR_MAP.get(raw)
            with self.lock:
                self.raw_sensor=raw
                # SE NON DETECTA NADA, MANTEMOS A ÚLTIMA MEDIDA
                self.current_error=err if err is not None else last
                last=self.current_error
            time.sleep(0.001) # PERÍODO DE MOSTRA

    # FÍO DE CONTROL E ACTUACIÓN 
    def _motor_thread(self):
      #APLICASE UN PID CLASICO A ACTUACION DOS MOTORES
        prev=0
        integ=0
        t=time.time()
        while self.running:
            now=time.time()
            dt=now-t #DIFERENCIAL DE TEMPO QUE SE USA PARA TD e TI
            t=now

            with self.lock:
                err=self.current_error
                em=self.emergency    
                kp,ki,kd=self.kp,self.ki,self.kd 

            if em: #parada de emerxencia por detección de teclado
                self.car.set_motor_model(0,0,0,0)
                time.sleep(0.01)
                continue

            # aplicación do pid
            integ=max(-500,min(500,integ+err*dt)) # CÁLCULO PARTE INTEGRAL USANDO UN FILTRO TIPO WIND UP PARA EVITAR A SATURACION DO SISTEMA
            der=(err-prev)/dt if dt>0 else 0      # CÁLCULO PARTE DERIVATIVA
            prev=err

            # CÁLCULO PID
            corr=kp*err+ki*integ+kd*der
            
            # APLICACION DO PID AS RODAS
            l=int(BASE_SPEED-corr)
            r=int(BASE_SPEED+corr)

            # uso de pequenas frenadas para conseguir un mellor axuste nas curvas pechadas
            if self.iterations%PULSE_EVERY==0:
                self.car.set_motor_model(0,0,0,0)
                time.sleep(PULSE_TIME)

            self.car.set_motor_model(l,l,r,r) #envío da actuación aos motores

            with self.lock:
                self.iterations+=1
                raw=self.raw_sensor

                #envío de datos a web
                self.current_data={
                    "sensor_left":bool(raw&0b100),
                    "sensor_center":bool(raw&0b010),
                    "sensor_right":bool(raw&0b001),
                    "error":err,
                    "iterations":self.iterations
                }
            time.sleep(0.001)

    #FÍO DE COMUNICACIÓN
    async def _ws_handler(self,ws):
        #ENGADO DE CLIENTE SOCKET
        self.clients.add(ws)
        try:
            async for msg in ws:
                #PREPARACIÓN DA DATA RECIBIDA USANDO CODIFICACIÓN EN JSON
                data=json.loads(msg)
                with self.lock:
                    if data["type"]=="pid": #ACTUALIZACIÓN DOS PARÁMETROS 
                        self.kp=data["kp"]
                        self.ki=data["ki"]
                        self.kd=data["kd"]
                       
                    elif data["type"]=="emergency": # ACTUALIZACIÓN DO RECIBO DUNHA PARADA por teclado
                        self.emergency=data["value"]
                       
        finally:
            self.clients.discard(ws)

    def _start_ws(self):
        "#inicializacion servidor para envio
        async def main():
            async with websockets.serve(self._ws_handler,"0.0.0.0",WS_PORT):
                while self.running:
                    if self.clients:
                        msg=json.dumps(self.current_data)
                        # envío de datos aos clientes navegadores abertos
                        await asyncio.gather(*[c.send(msg) for c in self.clients],return_exceptions=True)
                    await asyncio.sleep(0.05) # Envío a 20Hz (suficiente para el ojo humano)
        asyncio.run(main())

    #chamada aos fíos
    def start(self):
        # chamada aos fíos tipo daemon para que pechen se morre o programa principal
        threading.Thread(target=self._sensor_thread,daemon=True).start()
        threading.Thread(target=self._motor_thread,daemon=True).start()
        threading.Thread(target=self._start_ws,daemon=True).start()
        threading.Thread(target=lambda:HTTPServer(("0.0.0.0",HTTP_PORT),RobotHandler).serve_forever(),daemon=True).start()

        ip=socket.gethostbyname(socket.gethostname())
       

        # captura da barra espaciadora para causar parada
        fd=sys.stdin.fileno()
        old=termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while True:
                if select.select([sys.stdin],[],[],0.1)[0]:
                    if sys.stdin.read(1)==" ":
                        with self.lock:
                            self.emergency=not self.emergency
                           
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,old) # Restaura la terminal al salir

#chamada ao programa principal
if __name__=="__main__":
    try:
        #comezo da función que chama aos fíos
        RobotSystem().start()
    except KeyboardInterrupt:
        Ordinary_Car().set_motor_model(0,0,0,0) #se se presiona o final control C párase o robot
      
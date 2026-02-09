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

# ==================== RUTAS FREENOVE ====================
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Server'))
from Server.infrared import Infrared
from Server.motor import Ordinary_Car

# ==================== CONFIGURACIÓN ====================
HTTP_PORT = 8080
WS_PORT = 8765

BASE_SPEED = -500  # más lento = más estable

PULSE_EVERY = 3
PULSE_TIME = 0.004

SENSOR_ERROR_MAP = {
    0b000: None,
    0b001: 1.0,
    0b010: 0.0,
    0b011: 0.5,
    0b100: -1.0,
    0b101: 0.0,
    0b110: -0.5,
    0b111: 0.0
}

# ==================== INTERFAZ WEB ====================
HTML_INTERFACE = """<!DOCTYPE html>
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
<div id="status">🔴 DESCONECTADO</div>
<p>Iteraciones: <span id="it">0</span></p>
<p>Error: <span id="err">0</span></p>

<div>
<div id="sL" class="sensor"></div>
<div id="sC" class="sensor"></div>
<div id="sR" class="sensor"></div>
</div>

<button id="stopBtn">⛔ PARAR</button>

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
let emergency=false;

function connect(){
 ws=new WebSocket(`ws://${location.hostname}:8765`);
 ws.onopen=()=>status.innerText="🟢 CONECTADO";
 ws.onmessage=e=>{
  const d=JSON.parse(e.data);
  it.innerText=d.iterations;
  err.innerText=d.error.toFixed(2);
  sL.className=d.sensor_left?"sensor active":"sensor";
  sC.className=d.sensor_center?"sensor active":"sensor";
  sR.className=d.sensor_right?"sensor active":"sensor";
 };
 ws.onclose=()=>{status.innerText="🔴 DESCONECTADO"; setTimeout(connect,1000);}
}

function sendPID(){
 ws.send(JSON.stringify({
  type:"pid",
  kp:Number(kp.value),
  ki:Number(ki.value),
  kd:Number(kd.value)
 }));
}

["kp","ki","kd"].forEach(id=>{
 let el=document.getElementById(id);
 document.getElementById(id+"Val").innerText=el.value;
 el.oninput=()=>{document.getElementById(id+"Val").innerText=el.value; sendPID();}
});

stopBtn.onclick=()=>{
 emergency=!emergency;
 ws.send(JSON.stringify({type:"emergency",value:emergency}));
 stopBtn.innerText=emergency?"▶ REANUDAR":"⛔ PARAR";
 stopBtn.className=emergency?"active":"";
};

connect();
</script>
</body>
</html>
"""

class RobotHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type","text/html")
        self.end_headers()
        self.wfile.write(HTML_INTERFACE.encode())
    def log_message(self,*a): pass

# ==================== ROBOT ====================
class RobotSystem:
    def __init__(self):
        self.ir = Infrared()
        self.car = Ordinary_Car()
        self.lock = threading.Lock()
        self.clients = set()

        self.kp, self.ki, self.kd = 700.0, 0.0, 300.0
        self.emergency = False
        self.running = True

        self.raw_sensor = 0
        self.current_error = 0.0
        self.iterations = 0
        self.current_data = {}

    def _sensor_thread(self):
        last=0
        while self.running:
            raw=self.ir.read_all_infrared()
            err=SENSOR_ERROR_MAP.get(raw)
            with self.lock:
                self.raw_sensor=raw
                self.current_error=err if err is not None else last
                last=self.current_error
            time.sleep(0.001)

    def _motor_thread(self):
        prev=0
        integ=0
        t=time.time()
        while self.running:
            now=time.time()
            dt=now-t
            t=now

            with self.lock:
                err=self.current_error
                em=self.emergency
                kp,ki,kd=self.kp,self.ki,self.kd

            if em:
                self.car.set_motor_model(0,0,0,0)
                time.sleep(0.01)
                continue

            integ=max(-500,min(500,integ+err*dt))
            der=(err-prev)/dt if dt>0 else 0
            prev=err

            corr=kp*err+ki*integ+kd*der
            l=int(BASE_SPEED-corr)
            r=int(BASE_SPEED+corr)

            if self.iterations%PULSE_EVERY==0:
                self.car.set_motor_model(0,0,0,0)
                time.sleep(PULSE_TIME)

            self.car.set_motor_model(l,l,r,r)

            with self.lock:
                self.iterations+=1
                raw=self.raw_sensor
                self.current_data={
                    "sensor_left":bool(raw&0b100),
                    "sensor_center":bool(raw&0b010),
                    "sensor_right":bool(raw&0b001),
                    "error":err,
                    "iterations":self.iterations
                }

            print(f"[{self.iterations:6}] ERR:{err:+.2f} KP:{kp:.1f} KI:{ki:.1f} KD:{kd:.1f}")
            time.sleep(0.001)

    async def _ws_handler(self,ws):
        self.clients.add(ws)
        try:
            async for msg in ws:
                data=json.loads(msg)
                with self.lock:
                    if data["type"]=="pid":
                        self.kp=data["kp"]
                        self.ki=data["ki"]
                        self.kd=data["kd"]
                        print(f"🧠 PID → KP:{self.kp} KI:{self.ki} KD:{self.kd}")
                    elif data["type"]=="emergency":
                        self.emergency=data["value"]
                        print(f"🚨 EMERGENCIA WEB: {self.emergency}")
        finally:
            self.clients.discard(ws)

    def _start_ws(self):
        async def main():
            async with websockets.serve(self._ws_handler,"0.0.0.0",WS_PORT):
                while self.running:
                    if self.clients:
                        msg=json.dumps(self.current_data)
                        await asyncio.gather(*[c.send(msg) for c in self.clients],return_exceptions=True)
                    await asyncio.sleep(0.05)
        asyncio.run(main())

    def start(self):
        threading.Thread(target=self._sensor_thread,daemon=True).start()
        threading.Thread(target=self._motor_thread,daemon=True).start()
        threading.Thread(target=self._start_ws,daemon=True).start()
        threading.Thread(target=lambda:HTTPServer(("0.0.0.0",HTTP_PORT),RobotHandler).serve_forever(),daemon=True).start()

        ip=socket.gethostbyname(socket.gethostname())
        print(f"\n🌐 http://{ip}:{HTTP_PORT}")
        print("␣ ESPACIO = EMERGENCIA\n")

        fd=sys.stdin.fileno()
        old=termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while True:
                if select.select([sys.stdin],[],[],0.1)[0]:
                    if sys.stdin.read(1)==" ":
                        with self.lock:
                            self.emergency=not self.emergency
                            print(f"🚨 EMERGENCIA TECLADO: {self.emergency}")
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,old)

# ==================== MAIN ====================
if __name__=="__main__":
    try:
        RobotSystem().start()
    except KeyboardInterrupt:
        Ordinary_Car().set_motor_model(0,0,0,0)
        print("\n🛑 STOP")

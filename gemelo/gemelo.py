"""
Actividad 7 — Gemelo digital del enjambre ACO en PyBullet.

Recibe por UDP el estado de los 3 nodos ESP32 (feromona, mejor ruta y ruta
preferida de cada colonia) y lo replica con 3 carritos virtuales en el mismo
laberinto. La vista se publica como video en el navegador: http://localhost:8080

Uso:
  python gemelo.py              # con los ESP32 (el PC debe estar en la red ACO_ENJAMBRE)
  python gemelo.py --simular    # sin hardware: simula las 3 colonias aquí mismo
  python gemelo.py --gui        # además abre la ventana de PyBullet (solo fuera de Docker)

Variables de entorno: AP_IP (por defecto 192.168.4.1), PUERTO_WEB (por defecto 8080)
"""

import argparse
import json
import math
import os
import random
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pybullet as p
import pybullet_data

# ============================================================
# CONFIGURACIÓN (debe coincidir con aco.h)
# ============================================================
AP_IP = os.environ.get("AP_IP", "192.168.4.1")
PUERTO_UDP = 4210
PUERTO_WEB = int(os.environ.get("PUERTO_WEB", "8080"))

LABERINTO = [
    "#############",
    "#A....#.....#",
    "#.###.#.###.#",
    "#.#.......#.#",
    "#.#.###.#.#.#",
    "#...#...#...#",
    "###.#.#####.#",
    "#.....#....M#",
    "#############",
]
FILAS, COLS = len(LABERINTO), len(LABERINTO[0])
TAU_INI, TAU_MIN, TAU_MAX = 1.0, 0.05, 5.0
MAX_ARISTAS, MAX_RUTA = 64, 80
FORMATO = "<2sBBBBHHBBBBB64H80s80s"           # struct Paquete de aco.h
TAM_PAQUETE = struct.calcsize(FORMATO)        # 303 bytes
T_SIN_SENAL = 5.0                             # s sin paquetes -> nodo desconectado

TAM_CELDA = 0.5                               # m por celda en PyBullet
V_CARRITO = 1.5                               # m/s
COLOR_NODO = {1: (0.85, 0.12, 0.12), 2: (0.15, 0.40, 0.95), 3: (0.10, 0.70, 0.25)}
NOMBRE_NODO = {1: "N1 ESP32 (AP)", 2: "N2 ESP32", 3: "N3 ESP32-C3"}

ABIERTAS = [(f, c) for f in range(FILAS) for c in range(COLS) if LABERINTO[f][c] != "#"]
INICIO = next(f * COLS + c for f, c in ABIERTAS if LABERINTO[f][c] == "A")
META = next(f * COLS + c for f, c in ABIERTAS if LABERINTO[f][c] == "M")

# Aristas en el MISMO orden que aco.h: por filas, primero derecha y luego abajo
ARISTAS = []
for f in range(FILAS):
    for c in range(COLS):
        if LABERINTO[f][c] == "#":
            continue
        if c + 1 < COLS and LABERINTO[f][c + 1] != "#":
            ARISTAS.append((f * COLS + c, f * COLS + c + 1))
        if f + 1 < FILAS and LABERINTO[f + 1][c] != "#":
            ARISTAS.append((f * COLS + c, (f + 1) * COLS + c))
VECINOS = {}
for e, (a, b) in enumerate(ARISTAS):
    VECINOS.setdefault(a, []).append((b, e))
    VECINOS.setdefault(b, []).append((a, e))


def celda_xy(celda):
    f, c = divmod(celda, COLS)
    return c * TAM_CELDA, -f * TAM_CELDA


def parsear(datos):
    if len(datos) != TAM_PAQUETE:
        return None
    v = struct.unpack(FORMATO, datos)
    magia, version, tipo, nodo, conv, it, mejor_largo, exito, en_mejor, n_ar, l_mejor, l_pref = v[:12]
    tau = v[12:12 + MAX_ARISTAS]
    mejor, preferida = v[12 + MAX_ARISTAS], v[13 + MAX_ARISTAS]
    if magia != b"AC" or version != 1 or tipo != 1 or n_ar != len(ARISTAS) or not 1 <= nodo <= 3:
        return None
    return {
        "nodo": nodo, "convergido": bool(conv), "iteracion": it,
        "mejor_largo": None if mejor_largo == 0xFFFF else mejor_largo,
        "exito": exito, "en_mejor": en_mejor,
        "tau": [t / 1000.0 for t in tau[:n_ar]],
        "mejor": list(mejor[:l_mejor]), "preferida": list(preferida[:l_pref]),
    }


# ============================================================
# ESTADO COMPARTIDO DEL ENJAMBRE
# ============================================================
class Enjambre:
    def __init__(self):
        self.lock = threading.Lock()
        self.nodos = {}           # nodo -> (paquete, instante)

    def actualizar(self, paquete):
        with self.lock:
            self.nodos[paquete["nodo"]] = (paquete, time.time())

    def foto(self):
        with self.lock:
            return dict(self.nodos)

    def limpiar(self):
        with self.lock:
            self.nodos.clear()


class RedUDP(threading.Thread):
    """Se suscribe al nodo 1 (envía 'SUB' cada segundo) y recibe los paquetes de
    los 3 nodos. Como la conexión la inicia el gemelo, funciona también dentro de
    Docker (las respuestas vuelven por la misma ruta NAT)."""

    def __init__(self, enjambre):
        super().__init__(daemon=True)
        self.enjambre = enjambre
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.3)
        self.paquetes = 0

    def reset(self):
        try:
            self.sock.sendto(b"RESET", (AP_IP, PUERTO_UDP))
        except OSError:
            pass
        self.enjambre.limpiar()

    def run(self):
        t_sub = 0.0
        while True:
            if time.time() - t_sub > 1.0:
                t_sub = time.time()
                try:
                    self.sock.sendto(b"SUB", (AP_IP, PUERTO_UDP))
                except OSError:
                    pass              # aún no hay red: se reintenta
            try:
                datos, _ = self.sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue
            paquete = parsear(datos)
            if paquete:
                self.paquetes += 1
                self.enjambre.actualizar(paquete)


# ============================================================
# SIMULADOR (mismo algoritmo que aco.h) para probar sin hardware
# ============================================================
ALFA, BETA, RHO, Q, ELITE = 1.0, 0.5, 0.15, 4.0, 2.0
N_HORMIGAS, W_MEZCLA, ITER_ESTABLE, FRAC_CONVERGE = 6, 0.25, 10, 0.75
_FM, _CM = divmod(META, COLS)
HEUR = {f * COLS + c: 1.0 / (1 + abs(f - _FM) + abs(c - _CM)) for f, c in ABIERTAS}


class ColoniaSim:
    def __init__(self, nodo, semilla):
        self.nodo, self.rnd = nodo, random.Random(semilla)
        self.reiniciar()

    def reiniciar(self):
        self.tau = [TAU_INI] * len(ARISTAS)
        self.mejor, self.preferida = [], []
        self.iteracion = self.sin_mejora = 0
        self.exito = self.en_mejor = 0.0
        self.convergido = False

    def caminar(self):
        pos, ruta, vis = INICIO, [INICIO], {INICIO}
        while pos != META and len(ruta) < MAX_RUTA:
            op = [(v, e) for v, e in VECINOS[pos] if v not in vis]
            if not op:
                return ruta, False
            pesos = [self.tau[e] ** ALFA * HEUR[v] ** BETA for v, e in op]
            pos = self.rnd.choices([v for v, _ in op], weights=pesos)[0]
            ruta.append(pos)
            vis.add(pos)
        return ruta, pos == META

    def depositar(self, ruta, cantidad):
        for a, b in zip(ruta, ruta[1:]):
            e = next(e for v, e in VECINOS[a] if v == b)
            self.tau[e] += cantidad

    def iterar(self):
        self.iteracion += 1
        rutas = [self.caminar() for _ in range(N_HORMIGAS)]
        mejoro = False
        for r, ok in rutas:
            if ok and (not self.mejor or len(r) < len(self.mejor)):
                self.mejor, mejoro = r, True
        self.tau = [t * (1 - RHO) for t in self.tau]
        for r, ok in rutas:
            if ok:
                self.depositar(r, Q / (len(r) - 1))
        if self.mejor:
            self.depositar(self.mejor, ELITE * Q / (len(self.mejor) - 1))
        self.tau = [min(max(t, TAU_MIN), TAU_MAX) for t in self.tau]
        exitos = [r for r, ok in rutas if ok]
        self.exito = len(exitos) / N_HORMIGAS
        self.en_mejor = sum(len(r) == len(self.mejor) for r in exitos) / N_HORMIGAS
        self.sin_mejora = 0 if mejoro else self.sin_mejora + 1
        self.calcular_preferida()
        if mejoro:
            self.convergido = False
        if (self.mejor and self.sin_mejora >= ITER_ESTABLE and self.en_mejor >= FRAC_CONVERGE
                and len(self.preferida) == len(self.mejor)):
            self.convergido = True

    def calcular_preferida(self):
        pos, ruta, vis = INICIO, [INICIO], {INICIO}
        while pos != META and len(ruta) < MAX_RUTA:
            op = [(self.tau[e], v) for v, e in VECINOS[pos] if v not in vis]
            if not op:
                break
            pos = max(op)[1]
            ruta.append(pos)
            vis.add(pos)
        self.preferida = ruta

    def mezclar(self, otro):
        self.tau = [min(max((1 - W_MEZCLA) * a + W_MEZCLA * b, TAU_MIN), TAU_MAX)
                    for a, b in zip(self.tau, otro["tau"])]
        if otro["mejor"] and (not self.mejor or len(otro["mejor"]) < len(self.mejor)):
            self.mejor, self.sin_mejora = list(otro["mejor"]), 0

    def paquete(self):
        return {"nodo": self.nodo, "convergido": self.convergido, "iteracion": self.iteracion,
                "mejor_largo": len(self.mejor) - 1 if self.mejor else None,
                "exito": round(self.exito * 100), "en_mejor": round(self.en_mejor * 100),
                "tau": list(self.tau), "mejor": list(self.mejor), "preferida": list(self.preferida)}


class Simulador(threading.Thread):
    def __init__(self, enjambre, periodo=0.6):
        super().__init__(daemon=True)
        self.enjambre, self.periodo = enjambre, periodo
        self.colonias = [ColoniaSim(n, random.randrange(1 << 30)) for n in (1, 2, 3)]
        self.paquetes = 0

    def reset(self):
        for c in self.colonias:
            c.reiniciar()
        self.enjambre.limpiar()

    def run(self):
        while True:
            for c in self.colonias:
                c.iterar()
            paquetes = [c.paquete() for c in self.colonias]
            for c in self.colonias:
                for pq in paquetes:
                    if pq["nodo"] != c.nodo:
                        c.mezclar(pq)
            for pq in paquetes:
                self.enjambre.actualizar(pq)
                self.paquetes += 1
            time.sleep(self.periodo)


# ============================================================
# ESCENA PYBULLET
# ============================================================
def caja(medias, color, pos, orn=(0, 0, 0, 1)):
    vs = p.createVisualShape(p.GEOM_BOX, halfExtents=medias, rgbaColor=list(color) + [1])
    return p.createMultiBody(0, -1, vs, pos, orn)


def color_calor(v):
    """0 -> azul oscuro, 0.5 -> amarillo, 1 -> rojo (feromona baja -> alta)."""
    v = min(max(v, 0.0), 1.0)
    if v < 0.5:
        k = v / 0.5
        return (0.12 + 0.75 * k, 0.15 + 0.65 * k, 0.35 - 0.25 * k)
    k = (v - 0.5) / 0.5
    return (0.87 + 0.1 * k, 0.8 - 0.6 * k, 0.1)


class Escena:
    def __init__(self, gui):
        p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.loadURDF("plane.urdf", [0, 0, -0.001])

        h = TAM_CELDA / 2
        paredes = [celda_xy(f * COLS + c) for f in range(FILAS) for c in range(COLS) if LABERINTO[f][c] == "#"]
        vs = p.createVisualShape(p.GEOM_BOX, halfExtents=[h, h, 0.12], rgbaColor=[0.55, 0.57, 0.62, 1])
        p.createMultiBody(0, -1, vs, batchPositions=[[x, y, 0.12] for x, y in paredes])

        self.baldosas = {}
        for f, c in ABIERTAS:
            celda = f * COLS + c
            x, y = celda_xy(celda)
            color = (0.15, 0.85, 0.95) if celda == INICIO else (0.95, 0.95, 0.95) if celda == META else (0.12, 0.15, 0.35)
            self.baldosas[celda] = caja([h * 0.94, h * 0.94, 0.006], color, [x, y, 0.006])
        self._bandera(*celda_xy(META))

        self.carros = {n: self._carrito(COLOR_NODO[n]) for n in (1, 2, 3)}
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.resetDebugVisualizerCamera(6.5, 0, -55, [3.0, -2.0, 0])
        cx, cy = (COLS - 1) * TAM_CELDA / 2, -(FILAS - 1) * TAM_CELDA / 2
        self.vista = p.computeViewMatrix([cx, cy - 2.2, 6.2], [cx, cy - 0.15, 0], [0, 0, 1])
        self.proyeccion = p.computeProjectionMatrixFOV(46, 640 / 480, 0.1, 30)

    def _bandera(self, x, y):
        for dy in (-0.2, 0.2):
            caja([0.02, 0.02, 0.35], (0.85, 0.15, 0.15), [x, y + dy, 0.35])
        for i in range(8):
            for j in range(2):
                negro = (i + j) % 2 == 0
                caja([0.002, 0.025, 0.025], (0.05, 0.05, 0.05) if negro else (1, 1, 1),
                     [x, y - 0.175 + i * 0.05, 0.62 + j * 0.05])

    def _carrito(self, color):
        cuerpo = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.17, 0.08, 0.045], rgbaColor=list(color) + [1])
        cabina = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.07, 0.075, 0.035], rgbaColor=[0.85, 0.9, 1.0, 1])
        rueda = p.createVisualShape(p.GEOM_CYLINDER, radius=0.04, length=0.03, rgbaColor=[0.05, 0.05, 0.05, 1])
        giro = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
        posiciones = [[-0.02, 0, 0.07]] + [[sx * 0.1, sy * 0.1, -0.01] for sx in (-1, 1) for sy in (-1, 1)]
        return p.createMultiBody(
            0, -1, cuerpo, [0, 0, 0.06],
            linkMasses=[0] * 5, linkCollisionShapeIndices=[-1] * 5,
            linkVisualShapeIndices=[cabina] + [rueda] * 4,
            linkPositions=posiciones, linkOrientations=[[0, 0, 0, 1]] + [giro] * 4,
            linkInertialFramePositions=[[0, 0, 0]] * 5, linkInertialFrameOrientations=[[0, 0, 0, 1]] * 5,
            linkParentIndices=[0] * 5, linkJointTypes=[p.JOINT_FIXED] * 5, linkJointAxis=[[0, 0, 1]] * 5)

    def pintar_feromona(self, tau_medio):
        calor = {celda: 0.0 for celda in self.baldosas}
        for e, (a, b) in enumerate(ARISTAS):
            v = (tau_medio[e] - TAU_MIN) / (TAU_MAX - TAU_MIN)
            calor[a], calor[b] = max(calor[a], v), max(calor[b], v)
        for celda, cuerpo in self.baldosas.items():
            if celda not in (INICIO, META):
                p.changeVisualShape(cuerpo, -1, rgbaColor=list(color_calor(calor[celda])) + [1])

    def render(self):
        _, _, rgb, _, _ = p.getCameraImage(640, 480, self.vista, self.proyeccion)
        return cv2.cvtColor(np.reshape(rgb, (480, 640, 4))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)


class Carrito:
    """Recorre la ruta preferida de su nodo desde A. Al terminar espera y vuelve a A
    con la ruta más reciente: así se ve cómo cada colonia va aprendiendo."""

    def __init__(self, nodo, cuerpo, retraso):
        self.nodo, self.cuerpo = nodo, cuerpo
        self.lateral = {1: -0.14, 2: 0.0, 3: 0.14}[nodo]
        self.puntos, self.s, self.largo = [celda_xy(INICIO)], 0.0, 0.0
        self.espera = retraso
        self.yaw = 0.0
        self.viajes = 0

    def nuevo_viaje(self, ruta):
        self.puntos = [celda_xy(c) for c in ruta] if len(ruta) >= 2 else [celda_xy(INICIO)]
        self.largo = TAM_CELDA * (len(self.puntos) - 1)
        self.s = 0.0
        self.viajes += 1

    def avanzar(self, dt, ruta):
        if self.espera > 0:
            self.espera -= dt
            if self.espera <= 0:
                self.nuevo_viaje(ruta)
        elif self.s < self.largo:
            self.s = min(self.largo, self.s + V_CARRITO * dt)
            if self.s >= self.largo:
                self.espera = 0.8
        i = min(int(self.s / TAM_CELDA), len(self.puntos) - 2) if len(self.puntos) > 1 else 0
        if len(self.puntos) > 1:
            (x0, y0), (x1, y1) = self.puntos[i], self.puntos[i + 1]
            k = self.s / TAM_CELDA - i
            x, y = x0 + (x1 - x0) * k, y0 + (y1 - y0) * k
            objetivo = math.atan2(y1 - y0, x1 - x0)
            dif = (objetivo - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.yaw += max(-10 * dt, min(10 * dt, dif))         # giro suave en las esquinas
        else:
            x, y = self.puntos[0]
        x -= math.sin(self.yaw) * self.lateral
        y += math.cos(self.yaw) * self.lateral
        p.resetBasePositionAndOrientation(self.cuerpo, [x, y, 0.06], p.getQuaternionFromEuler([0, 0, self.yaw]))


# ============================================================
# PANEL 2D (OpenCV)
# ============================================================
PX = 36
ORIGEN = (86, 52)
COLOR_BGR = {n: tuple(int(255 * v) for v in reversed(c)) for n, c in COLOR_NODO.items()}


def centro_px(celda, desplaz=0):
    f, c = divmod(celda, COLS)
    return ORIGEN[0] + c * PX + PX // 2 + desplaz, ORIGEN[1] + f * PX + PX // 2 + desplaz


def panel(foto, tau_medio, modo, paquetes):
    img = np.full((480, 640, 3), 24, np.uint8)
    ahora = time.time()
    for f in range(FILAS):
        for c in range(COLS):
            x0, y0 = ORIGEN[0] + c * PX, ORIGEN[1] + f * PX
            color = (70, 70, 70) if LABERINTO[f][c] == "#" else (45, 35, 30)
            cv2.rectangle(img, (x0, y0), (x0 + PX - 1, y0 + PX - 1), color, -1)
    if tau_medio:
        for e, (a, b) in enumerate(ARISTAS):
            v = (tau_medio[e] - TAU_MIN) / (TAU_MAX - TAU_MIN)
            r, g, bl = color_calor(v)
            cv2.line(img, centro_px(a), centro_px(b), (int(bl * 255), int(g * 255), int(r * 255)), 2 + int(10 * v))
    for n, desplaz in ((1, -6), (2, 0), (3, 6)):
        if n in foto and len(foto[n][0]["preferida"]) > 1:
            pts = np.array([centro_px(cel, desplaz) for cel in foto[n][0]["preferida"]], np.int32)
            cv2.polylines(img, [pts], False, COLOR_BGR[n], 2, cv2.LINE_AA)
    for cel, txt in ((INICIO, "A"), (META, "M")):
        cx, cy = centro_px(cel)
        cv2.putText(img, txt, (cx - 9, cy + 9), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(img, f"Enjambre ACO - {modo}", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    y = ORIGEN[1] + FILAS * PX + 22
    activos = []
    for n in (1, 2, 3):
        if n in foto and ahora - foto[n][1] < T_SIN_SENAL:
            pq = foto[n][0]
            activos.append(pq)
            mejor = f"{pq['mejor_largo']} pasos" if pq["mejor_largo"] is not None else "-"
            texto = (f"{NOMBRE_NODO[n]}: it {pq['iteracion']:3d} | mejor {mejor} | "
                     f"en la mejor {pq['en_mejor']}% | {'CONVERGIDO' if pq['convergido'] else 'buscando'}")
        else:
            texto = f"{NOMBRE_NODO[n]}: sin senal"
        cv2.circle(img, (20, y - 5), 6, COLOR_BGR[n], -1)
        cv2.putText(img, texto, (32, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        y += 20
    if (len(activos) == 3 and len({q["mejor_largo"] for q in activos}) == 1
            and all(q["mejor_largo"] is not None and len(q["preferida"]) - 1 == q["mejor_largo"] for q in activos)):
        estado, color = f"ENJAMBRE DE ACUERDO: ruta optima de {activos[0]['mejor_largo']} pasos", (80, 220, 80)
    elif activos:
        estado, color = f"Buscando... ({len(activos)}/3 nodos en linea, {paquetes} paquetes)", (0, 200, 255)
    else:
        estado, color = f"Esperando nodos en {AP_IP}:{PUERTO_UDP}...", (0, 140, 255)
    cv2.putText(img, estado, (12, 472), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return img


# ============================================================
# SERVIDOR WEB (video MJPEG en http://localhost:8080)
# ============================================================
class Web:
    jpeg = b""
    estado = {}
    lock = threading.Lock()
    al_reiniciar = None


PAGINA = """<!doctype html><html><head><meta charset="utf-8"><title>Gemelo digital ACO</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:100%;border:1px solid #444}button{font-size:16px;padding:8px 18px;margin:10px}</style></head>
<body><h2>Enjambre de 3 carritos con ACO &mdash; gemelo digital en PyBullet</h2>
<img src="/stream"><br><button onclick="fetch('/reset')">Reiniciar enjambre</button></body></html>"""


class Manejador(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            datos = PAGINA.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(datos)))
            self.end_headers()
            self.wfile.write(datos)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=cuadro")
            self.end_headers()
            try:
                while True:
                    with Web.lock:
                        jpeg = Web.jpeg
                    if jpeg:
                        self.wfile.write(b"--cuadro\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif self.path == "/reset":
            if Web.al_reiniciar:
                Web.al_reiniciar()
            self.send_response(204)
            self.end_headers()
        elif self.path == "/estado":
            with Web.lock:
                datos = json.dumps(Web.estado).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(datos)
        else:
            self.send_error(404)


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================
def main():
    ap = argparse.ArgumentParser(description="Gemelo digital del enjambre ACO")
    ap.add_argument("--simular", action="store_true", help="simula las 3 colonias (sin ESP32)")
    ap.add_argument("--gui", action="store_true", help="abre la ventana de PyBullet (fuera de Docker)")
    args = ap.parse_args()

    enjambre = Enjambre()
    fuente = Simulador(enjambre) if args.simular else RedUDP(enjambre)
    fuente.start()
    Web.al_reiniciar = fuente.reset
    modo = "SIMULACION" if args.simular else f"ESP32 reales (AP {AP_IP})"

    escena = Escena(args.gui)
    carros = {n: Carrito(n, escena.carros[n], retraso=0.5 + 1.0 * (n - 1)) for n in (1, 2, 3)}

    servidor = ThreadingHTTPServer(("0.0.0.0", PUERTO_WEB), Manejador)
    threading.Thread(target=servidor.serve_forever, daemon=True).start()
    print(f"Gemelo digital listo ({modo}). Abre http://localhost:{PUERTO_WEB} en el navegador.")

    t_prev = time.time()
    t_calor = 0.0
    try:
        while True:
            ahora = time.time()
            dt, t_prev = min(ahora - t_prev, 0.2), ahora
            foto = {n: v for n, v in enjambre.foto().items() if ahora - v[1] < T_SIN_SENAL}

            tau_medio = None
            if foto:
                taus = [v[0]["tau"] for v in foto.values()]
                tau_medio = [sum(t) / len(taus) for t in zip(*taus)]
                if ahora - t_calor > 0.3:
                    escena.pintar_feromona(tau_medio)
                    t_calor = ahora

            for n, carro in carros.items():
                ruta = foto[n][0]["preferida"] if n in foto else [INICIO]
                carro.avanzar(dt, ruta)

            vista3d = escena.render()
            vista2d = panel(foto, tau_medio, modo, fuente.paquetes)
            cuadro = np.hstack([vista3d, vista2d])
            ok, jpeg = cv2.imencode(".jpg", cuadro, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with Web.lock:
                Web.jpeg = jpeg.tobytes() if ok else Web.jpeg
                Web.estado = {str(n): {k: v[0][k] for k in ("iteracion", "mejor_largo", "en_mejor", "convergido")}
                              for n, v in foto.items()}
            time.sleep(max(0.0, 0.066 - (time.time() - ahora)))
    except KeyboardInterrupt:
        pass
    finally:
        servidor.shutdown()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
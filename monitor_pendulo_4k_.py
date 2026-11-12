"""
Monitor del carro péndulo   ·   STM32 -> ESP32-C3 -> WiFi -> PC

Interfaz (opción A):
  - Animación en vivo del carro y el péndulo vistos de lado, a escala
  - Gráfica del ángulo θ (sensor AS5600) y de la señal de control (PWM)
  - Valores actuales, estado de la STM32 y parámetros del sistema
  - "Más variables": ω, x, v y RPM en otra ventana
  - Grabación de todas las tramas en CSV

Uso:
  1. Conecta la PC a la red WiFi "Carro-Pendulo" (clave carro1234).
  2. Ejecuta:  python monitor_pendulo.py
  3. Pulsa "Conectar" (IP 192.168.4.1, puerto 3333).
  Con "Simulación" puedes probar la interfaz sin el carro.

Requisitos:
    pip install matplotlib
"""

import base64
import binascii
import csv
import io
import logging
import math
import queue
import random
import socket
import struct
import sys
import threading
import time
from collections import deque, namedtuple

import tkinter as tk
from tkinter import messagebox, filedialog

import numpy as np
import matplotlib
from matplotlib import patheffects
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgba
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Ellipse, FancyArrowPatch, FancyBboxPatch, Rectangle, Wedge
from matplotlib.ticker import FuncFormatter, MaxNLocator
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:                                    # viene instalado con matplotlib
    from PIL import Image, ImageTk
except ImportError:
    Image = ImageTk = None


# =========================== CONFIGURACIÓN ===========================
TITULO = ""                    # título grande del encabezado (vacío = no se muestra)
SUBTITULO = "Telemetría en vivo · STM32F401 → ESP32-C3 → WiFi"

IP_DEFECTO = "192.168.4.1"
PUERTO_DEFECTO = 3333

# Encoders
CPR_IZQ = 1974.0               # cuentas por vuelta de la rueda izquierda
CPR_DER = 1969.6               # cuentas por vuelta de la rueda derecha
SIGNO_RPM_DERECHO = -1         # convención de signo del carro

# Carro y péndulo (para el dibujo y el panel de parámetros)
MASA_PENDULO_G = 27.3
MASA_CARRO_G = 846
LARGO_VARILLA_CM = 35.0        # estimado de la captura de Blender: medir y corregir
DIAMETRO_RUEDA_CM = 6.5        # estimado: medir y corregir
ALTURA_PIVOTE_CM = 16.0        # del piso al eje del péndulo (estimado)
SEPARACION_RUEDAS_CM = 12.0    # entre ejes, vista lateral (estimado)
MEDIDAS_CONFIRMADAS = False    # pon True cuando las 4 medidas de arriba sean las reales

# Gráficas
VENTANA_GRAFICA_S = 10         # segundos visibles
ZONA_EQUILIBRIO_DEG = 2.0      # banda verde en la gráfica de θ
SIN_DATOS_S = 2.0              # aviso si no llegan tramas en este tiempo
REFRESCO_DATOS_MS = 50         # números y animación
REFRESCO_GRAFICA_MS = 100      # gráficas
MAX_PUNTOS = 3000              # muestras guardadas (150 s a 20 tramas/s)
# =====================================================================


# ----------------------------- colores -----------------------------
FONDO = "#eef1f4"
PANEL = "#ffffff"
FONDO_ANIM = "#fbfcfd"
BORDE = "#dfe3e8"
BORDE_FUERTE = "#c9d0d7"
TEXTO = "#1d2329"
TENUE = "#5b6672"
ROJO_U = "#b5121b"             # rojo Unillanos: ángulo θ y franja del encabezado
AZUL = "#1f6fb5"               # motor izquierdo / flecha del PWM
NARANJA = "#e07b00"            # motor derecho (líneas)
NARANJA_TXT = "#a85b00"        # motor derecho (texto)
VERDE_FONDO, VERDE_TXT = "#e8f5ec", "#14612a"
AMBAR_FONDO, AMBAR_TXT = "#fff4e0", "#8a4b00"
ROJO_FONDO, ROJO_TXT = "#fdecea", "#b5121b"
GRIS_FONDO = "#f1f3f5"

FUENTE = "Segoe UI"
FUENTE_NUM = "Consolas"
FUENTES_MONO = ["Consolas", "DejaVu Sans Mono", "monospace"]

# Colores del dibujo (iguales al modelo de Blender)
ORO, ORO_OSCURO = "#d9a521", "#a87d12"     # varilla del péndulo
CUERPO, CUERPO_BORDE = "#2f6f62", "#24574d"
PLACA = "#1d3557"              # BlackPill y AS5600
CHASIS = "#3b4650"
LLANTA = "#2b2f33"
RIN = "#2e86c1"
RAYOS = "#d6eaf8"

# Letra de las gráficas igual a la de la ventana
matplotlib.rcParams.update({"font.family": "sans-serif",
                            "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"]})
logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

# Escala de la pantalla (1.0 = 100 %, 1.25 = 125 %…); se mide al abrir la ventana
ESCALA = 1.0


def px(valor):
    """Píxeles escalados según la configuración de pantalla de Windows."""
    return int(round(valor * ESCALA))


def activar_alta_resolucion():
    """
    Con la pantalla de Windows al 125 % o 150 %, sin esto Windows estira la
    ventana como si fuera una foto y todo se ve borroso.
    """
    if sys.platform != "win32":
        return
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass


# -----------------------------------------------------------------
#  Protocolo de la STM32 (trama binaria v1 de 50 bytes)
# -----------------------------------------------------------------
SYNC = b"\xA5\x5A"
VERSION = 1
LARGO_PAYLOAD = 44
TAMANO_TRAMA = 50
_CUERPO = struct.Struct("<BBBBIIiihhHBBffffI")    # bytes 0..47; luego CRC16
assert _CUERPO.size == TAMANO_TRAMA - 2

Trama = namedtuple("Trama", "seq t_us enc_izq enc_der pwm_izq pwm_der as_crudo "
                            "as_estado flags x_m v_m_s theta_rad omega_rad_s missed")

# bit de flags, texto normal, texto con el flag activo, gravedad
FLAGS = (
    (0x01, "AS5600 válido", "AS5600 inválido", "rojo"),
    (0x02, "Cero calibrado", "Sin cero", "ambar"),
    (0x04, "Signos verificados", "Signos por verificar", "ambar"),
    (0x08, "Seguridad OK", "Fallo de seguridad", "rojo"),
)


def crc16(datos):
    """CRC16 CCITT-FALSE (polinomio 0x1021, inicio 0xFFFF), igual que la STM32."""
    return binascii.crc_hqx(bytes(datos), 0xFFFF)


def a_int32(valor):
    """Interpreta un entero como int32 con signo (para contadores que dan la vuelta)."""
    return ((int(valor) + 2**31) % 2**32) - 2**31


def decodificar(trama):
    return Trama(*_CUERPO.unpack_from(trama)[4:])


def construir_trama(seq, t_us, enc_izq, enc_der, pwm_izq, pwm_der, as_crudo, as_estado,
                    flags, x_m, v_m_s, theta_rad, omega_rad_s, missed):
    """Arma una trama igual a la de la STM32 (para la simulación y las pruebas)."""
    cuerpo = _CUERPO.pack(0xA5, 0x5A, VERSION, LARGO_PAYLOAD,
                          seq & 0xFFFFFFFF, t_us & 0xFFFFFFFF,
                          a_int32(enc_izq), a_int32(enc_der), pwm_izq, pwm_der,
                          as_crudo, as_estado, flags,
                          x_m, v_m_s, theta_rad, omega_rad_s, missed & 0xFFFFFFFF)
    return cuerpo + struct.pack("<H", crc16(cuerpo))


class ReceptorTramas:
    """
    Separa las tramas del flujo de bytes. Si una trama llega dañada busca la
    siguiente cabecera A5 5A, así no se pierde la trama que viene detrás.
    """

    def __init__(self):
        self.buf = bytearray()
        self.validas = 0
        self.errores_crc = 0
        self.errores_cabecera = 0
        self.bytes_descartados = 0

    def alimentar(self, datos):
        buf = self.buf
        buf.extend(datos)
        tramas = []
        while True:
            i = buf.find(SYNC)
            if i < 0:
                guardar = 1 if buf.endswith(b"\xA5") else 0   # cabecera partida
                self.bytes_descartados += len(buf) - guardar
                del buf[:len(buf) - guardar]
                break
            if i:
                self.bytes_descartados += i
                del buf[:i]
            if len(buf) < 4:
                break
            if buf[2] != VERSION or buf[3] != LARGO_PAYLOAD:
                self.errores_cabecera += 1
                del buf[:1]
                continue
            if len(buf) < TAMANO_TRAMA:
                break
            if crc16(buf[:TAMANO_TRAMA - 2]) == int.from_bytes(buf[TAMANO_TRAMA - 2:TAMANO_TRAMA], "little"):
                tramas.append(decodificar(bytes(buf[:TAMANO_TRAMA])))
                self.validas += 1
                del buf[:TAMANO_TRAMA]
            else:
                self.errores_crc += 1
                del buf[:1]
        return tramas


# -----------------------------------------------------------------
#  Recepción en segundo plano
# -----------------------------------------------------------------
class Lector(threading.Thread):
    """Recibe las tramas en segundo plano y las deja en una cola para la interfaz."""

    def __init__(self, modo, ip, puerto, cola):
        super().__init__(daemon=True)
        self.modo = modo            # "wifi" o "sim"
        self.ip = ip
        self.puerto = puerto
        self.cola = cola
        self.receptor = ReceptorTramas()
        self._parar = threading.Event()
        self._empujon = threading.Event()

    def detener(self):
        self._parar.set()

    def empujar(self):
        """Solo en simulación: da un empujón al péndulo."""
        self._empujon.set()

    def _avisar(self, tipo, valor=None):
        self.cola.put((self, tipo, valor))

    def _entregar(self, datos):
        for trama in self.receptor.alimentar(datos):
            self._avisar("dato", trama)

    def run(self):
        try:
            if self.modo == "wifi":
                self._leer_wifi()
            else:
                self._simular()
        except Exception as error:   # cualquier fallo se muestra en la interfaz
            self._avisar("error", str(error))
        else:
            self._avisar("fin")

    def _leer_wifi(self):
        try:
            conexion = socket.create_connection((self.ip, self.puerto), timeout=5)
        except socket.timeout:
            raise ConnectionError(f"sin respuesta de {self.ip}:{self.puerto}. "
                                  "¿La PC está en la red Carro-Pendulo?") from None
        except OSError as error:
            raise ConnectionError(f"no se pudo conectar ({error}). "
                                  "¿La PC está en la red Carro-Pendulo?") from None
        with conexion:
            conexion.settimeout(0.5)
            self._avisar("estado", f"{self.ip}:{self.puerto}")
            while not self._parar.is_set():
                try:
                    datos = conexion.recv(4096)
                except socket.timeout:
                    continue
                if not datos:
                    raise ConnectionError("la ESP32-C3 cerró la conexión")
                self._entregar(datos)

    def _simular(self):
        """Genera tramas parecidas a las del carro (20 por segundo) y las pasa por el receptor."""
        self._avisar("estado", "simulación")
        periodo = 0.05
        radio = DIAMETRO_RUEDA_CM / 200
        seq = 0
        t_golpe, amplitud = -10.0, 0.0
        proximo = time.monotonic()
        while not self._parar.is_set():
            t = seq * periodo
            if t - t_golpe >= 6.0 or self._empujon.is_set():   # empujón cada 6 s o al pedirlo
                self._empujon.clear()
                t_golpe = t
                amplitud = math.radians(random.uniform(5, 9)) * random.choice((-1, 1))
            tau = t - t_golpe
            a, w = 1.1, 2 * math.pi * 1.25
            amort = amplitud * math.exp(-a * tau)
            theta = amort * math.sin(w * tau)
            omega = amort * (w * math.cos(w * tau) - a * math.sin(w * tau))
            x = 0.08 * math.sin(0.45 * t) + 0.3 * theta
            v = 0.036 * math.cos(0.45 * t) + 0.3 * omega
            u = 350 * theta + 30 * omega + 80 * x + 40 * v
            pwm_izq = max(-100, min(100, round(u)))
            pwm_der = max(-100, min(100, round(u) + random.choice((-1, 0, 1))))
            vueltas = x / (2 * math.pi * radio)
            trama = construir_trama(
                seq, int(t * 1e6),
                round(vueltas * CPR_IZQ), -round(vueltas * CPR_DER),
                pwm_izq, pwm_der,
                round(2048 + theta / (2 * math.pi) * 4096) % 4096, 0x20, 0,
                x, v, theta, omega, 0)
            corte = random.randint(1, TAMANO_TRAMA - 1)   # llega partida, como por TCP
            self._entregar(trama[:corte])
            self._entregar(trama[corte:])
            seq += 1
            proximo += periodo
            espera = proximo - time.monotonic()
            if espera > 0:
                self._parar.wait(espera)
            else:
                proximo = time.monotonic()


# -----------------------------------------------------------------
#  Utilidades
# -----------------------------------------------------------------
def con_signo(valor, decimales):
    """'+3.4', '−2.0', '0.0' o '—' si no hay dato."""
    if valor is None or not math.isfinite(valor):
        return "—"
    texto = f"{abs(valor):.{decimales}f}"
    if float(texto) == 0:
        return texto
    return ("+" if valor > 0 else "−") + texto


def formato_eje(valor, _pos):
    if abs(valor) < 1e-9:
        return "0"
    return ("+" if valor > 0 else "−") + f"{abs(valor):g}"


def cargar_logo(datos_b64, alto):
    """Logo incrustado en este archivo, escalado a 'alto' píxeles con buen filtrado."""
    try:
        if Image is not None:
            img = Image.open(io.BytesIO(base64.b64decode(datos_b64))).convert("RGB")
            ancho = round(img.width * alto / img.height)
            return ImageTk.PhotoImage(img.resize((ancho, alto), Image.LANCZOS))
        foto = tk.PhotoImage(data=datos_b64)
        paso = max(1, round(foto.height() / alto))
        return foto.subsample(paso, paso)
    except Exception:           # si algo falla, la interfaz funciona sin logos
        return None


def crear_panel(padre):
    return tk.Frame(padre, bg=PANEL, highlightbackground=BORDE, highlightcolor=BORDE,
                    highlightthickness=1, bd=0)


def crear_boton(padre, texto, comando, primario=False):
    """Botón plano con borde de 1 px. Devuelve (marco para ubicarlo, botón)."""
    marco = tk.Frame(padre, bg=TEXTO if primario else BORDE_FUERTE, padx=1, pady=1)
    boton = tk.Button(marco, text=texto, command=comando, font=(FUENTE, 10, "bold"),
                      bg=TEXTO if primario else PANEL, fg="#ffffff" if primario else TEXTO,
                      activebackground="#3a434c" if primario else GRIS_FONDO,
                      activeforeground="#ffffff" if primario else TEXTO,
                      disabledforeground="#9aa5b1", relief=tk.FLAT, bd=0,
                      padx=px(14), pady=px(6), cursor="hand2", highlightthickness=0)
    boton.pack()
    return marco, boton


# -----------------------------------------------------------------
#  Gráfica de los últimos segundos (solo se redibujan las curvas)
# -----------------------------------------------------------------
class GraficaTiempo:
    """
    Eje de tiempo fijo (-10 s a 0 = ahora). La escala vertical solo cambia
    cuando los datos se salen o quedan muy chicos; el resto del tiempo se
    redibujan solo las curvas sobre el fondo guardado, que es muy rápido.
    """

    def __init__(self, padre, curvas, rango_min, centrado=False, banda=None,
                 guias=(), alto=2.4):
        # curvas: [(clave de la serie, color, estilo de línea, grosor)]
        self.rango_min = rango_min
        self.centrado = centrado
        self.fig = Figure(figsize=(5, alto), dpi=100, facecolor=PANEL)
        self.fig.subplots_adjust(left=0.105, right=0.965, top=0.965, bottom=0.135)
        self.ax = ax = self.fig.add_subplot()
        ax.set_facecolor(PANEL)
        if banda:
            ax.axhspan(-banda, banda, color=VERDE_FONDO, lw=0, zorder=0)
        ax.axhline(0, color="#7b8794", lw=1.1, ls=(0, (5, 4)), zorder=1)
        for y in guias:
            ax.axhline(y, color="#aab3bc", lw=1, ls=(0, (2, 3)), zorder=1)
        self.lineas = []
        for clave, color, estilo, grosor in curvas:
            linea, = ax.plot([], [], color=color, ls=estilo, lw=grosor, animated=True,
                             solid_joinstyle="round", solid_capstyle="round")
            punto, = ax.plot([], [], ls="none", marker="o", ms=5.5, color=color, mec=PANEL,
                             mew=1.3, animated=True)
            self.lineas.append((clave, linea, punto))

        marcas = list(range(-VENTANA_GRAFICA_S, 1, 2))
        ax.set_xlim(-VENTANA_GRAFICA_S, 0.25)          # un poco de aire para el punto de "ahora"
        ax.set_xticks(marcas)
        ax.set_xticklabels([f"−{-m} s" if m == marcas[0] else ("ahora" if m == 0 else f"−{-m}")
                            for m in marcas])
        ax.yaxis.set_major_locator(MaxNLocator(nbins=5, steps=[1, 2, 5, 10]))
        ax.yaxis.set_major_formatter(FuncFormatter(formato_eje))
        ax.grid(True, color="#e6e9ed", lw=0.9)
        ax.tick_params(labelsize=8.5, colors=TENUE, length=0, pad=4)
        for lado in ax.spines.values():
            lado.set_color(BORDE_FUERTE)
            lado.set_linewidth(0.9)
        mitad = rango_min / 2
        ax.set_ylim(-mitad * 1.05, mitad * 1.05)

        self.fondo = None
        self.lienzo = FigureCanvasTkAgg(self.fig, master=padre)
        self.widget = self.lienzo.get_tk_widget()
        self.widget.configure(background=PANEL, highlightthickness=0)
        self.lienzo.mpl_connect("draw_event", self._al_dibujar_todo)

    def actualizar(self, t_rel, series):
        datos = []
        for clave, linea, punto in self.lineas:
            valores = series[clave]
            linea.set_data(t_rel, valores)
            if len(valores) and math.isfinite(valores[-1]):
                punto.set_data([t_rel[-1]], [valores[-1]])
            else:
                punto.set_data([], [])
            datos.append(valores)
        cambio = self._ajustar_escala(np.concatenate(datos) if datos else np.array([]))
        if cambio or self.fondo is None:
            self.lienzo.draw()           # ejes, textos y curvas
        else:
            self._dibujar_curvas()       # solo las curvas

    def _ajustar_escala(self, datos):
        validos = datos[np.isfinite(datos)]
        if validos.size == 0:
            return False
        lo, hi = self.ax.get_ylim()
        if self.centrado:
            m = max(self.rango_min / 2, float(np.max(np.abs(validos))))
            if m <= hi and m >= 0.4 * hi:
                return False
            limite = m * (1.05 if m <= self.rango_min / 2 else 1.15)
            self.ax.set_ylim(-limite, limite)
            return True
        d_min, d_max = float(validos.min()), float(validos.max())
        if d_max - d_min < self.rango_min:
            centro = (d_min + d_max) / 2
            d_min, d_max = centro - self.rango_min / 2, centro + self.rango_min / 2
        if d_min >= lo and d_max <= hi and (d_max - d_min) >= 0.4 * (hi - lo):
            return False
        margen = 0.15 * (d_max - d_min)
        self.ax.set_ylim(d_min - margen, d_max + margen)
        return True

    def _al_dibujar_todo(self, _evento):
        """Tras cada dibujo completo (también al cambiar el tamaño) guarda el fondo."""
        self.fondo = self.lienzo.copy_from_bbox(self.ax.bbox)
        self._dibujar_curvas()

    def _dibujar_curvas(self):
        self.lienzo.restore_region(self.fondo)
        for _, linea, punto in self.lineas:
            self.ax.draw_artist(linea)
            self.ax.draw_artist(punto)
        self.lienzo.blit(self.ax.bbox)


# -----------------------------------------------------------------
#  Animación del carro y el péndulo (vista lateral, a escala)
# -----------------------------------------------------------------
class Animacion:
    """
    Dibujada con matplotlib para que los bordes salgan suaves. Las medidas
    están en centímetros reales. El fondo (cuadrícula, piso y regla) se guarda
    y en cada cuadro solo se redibujan las piezas que se mueven.
    """
    ABAJO_IN = 0.78              # espacio bajo el piso para la regla y la flecha (pulgadas)

    def __init__(self, padre):
        self.centro_cm = 0           # posición de la pista en el centro de la vista
        self.x_cm = 0.0
        self.theta = 0.0
        self.theta_ok = True
        self.pwm = 0.0
        self.hay_datos = False
        self.estela = deque()        # (t_s, x_cm, θ°) del último segundo
        self.estaticos = []
        self.fondo = None

        self.fig = Figure(figsize=(8, 4.4), dpi=100, facecolor=FONDO_ANIM)
        self.ax = ax = self.fig.add_axes([0, 0, 1, 1])
        ax.set_axis_off()

        def parche(p):
            p.set_animated(True)
            ax.add_patch(p)
            return p

        def linea(**opciones):
            return ax.plot([], [], animated=True, **opciones)[0]

        self.i_estela = LineCollection([], capstyle="round", animated=True)
        ax.add_collection(self.i_estela)
        self.i_ref = linea(color="#9aa5b1", lw=1.1, ls=(0, (5, 4)))
        self.i_sombra = parche(Ellipse((0, 0), 1, 1, facecolor="#000000", alpha=0.10, edgecolor="none"))
        self.i_chasis = parche(FancyBboxPatch((0, 0), 1, 1, boxstyle="round,pad=0,rounding_size=0.5",
                                              facecolor=CHASIS, edgecolor="none"))
        self.i_caja = parche(FancyBboxPatch((0, 0), 1, 1, boxstyle="round,pad=0,rounding_size=0.8",
                                            facecolor=CUERPO, edgecolor=CUERPO_BORDE, lw=0.8))
        self.i_placa = parche(FancyBboxPatch((0, 0), 1, 1, boxstyle="round,pad=0,rounding_size=0.3",
                                             facecolor=PLACA, edgecolor="none"))
        self.i_mastil = parche(Rectangle((0, 0), 1, 1, facecolor=CUERPO, edgecolor="none"))
        self.ruedas = []
        for _ in range(2):
            llanta = parche(Circle((0, 0), 1, facecolor=LLANTA, edgecolor="none"))
            rin = parche(Circle((0, 0), 1, facecolor=RIN, edgecolor="none"))
            rayos = linea(color=RAYOS, solid_capstyle="round")
            buje = parche(Circle((0, 0), 1, facecolor="#e8eef3", edgecolor="none"))
            self.ruedas.append((llanta, rin, rayos, buje))
        self.i_as5600 = parche(FancyBboxPatch((0, 0), 1, 1, boxstyle="round,pad=0,rounding_size=0.3",
                                              facecolor=PLACA, edgecolor="none"))
        self.i_cuna = parche(Wedge((0, 0), 1, 89, 90, facecolor=ROJO_U, alpha=0.10, edgecolor="none"))
        self.i_arco = linea(color=ROJO_U, lw=1.6, solid_capstyle="round")
        self.i_varilla = linea(color=ORO, solid_capstyle="round")
        self.i_pivote = parche(Circle((0, 0), 1, facecolor="#f4f5f7", edgecolor=PLACA, lw=1.5))
        self.i_txt_theta = ax.annotate("", xy=(0, 0), xytext=(0, 0), textcoords="offset points",
                                       ha="right", va="center", fontsize=12, fontweight="bold",
                                       family=FUENTES_MONO, color=ROJO_U, animated=True,
                                       bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="none", alpha=0.85))
        self.i_flecha = parche(FancyArrowPatch((0, 0), (1, 0), arrowstyle="-|>,head_length=0.55,head_width=0.32",
                                               mutation_scale=20, lw=3.2, color=AZUL,
                                               shrinkA=0, shrinkB=0, joinstyle="miter"))
        self.i_txt_pwm = ax.annotate("", xy=(0, 0), xytext=(10, 0), textcoords="offset points",
                                     ha="left", va="center", fontsize=10.5, fontweight="bold",
                                     family=FUENTES_MONO, color=AZUL, animated=True)
        self.i_aviso = ax.text(0.5, 0.93, "Sin datos · pulsa Conectar", transform=ax.transAxes,
                               ha="center", va="center", fontsize=11, color=TENUE, animated=True)

        # orden de dibujo, de atrás hacia adelante
        self.piezas = [self.i_estela, self.i_ref, self.i_sombra, self.i_chasis, self.i_caja,
                       self.i_placa, self.i_mastil]
        for rueda in self.ruedas:
            self.piezas.extend(rueda)
        self.piezas += [self.i_as5600, self.i_cuna, self.i_arco, self.i_varilla, self.i_pivote,
                        self.i_txt_theta, self.i_flecha, self.i_txt_pwm, self.i_aviso]

        self.lienzo = FigureCanvasTkAgg(self.fig, master=padre)
        self.widget = self.lienzo.get_tk_widget()
        self.widget.configure(background=FONDO_ANIM, highlightthickness=0)
        self.lienzo.mpl_connect("resize_event", self._al_redimensionar)
        self.lienzo.mpl_connect("draw_event", self._al_dibujar_todo)
        self._ajustar_vista()
        self._rehacer_fondo()
        self._colocar_piezas()

    # --- escala y fondo ---
    def _ajustar_vista(self):
        """Elige la escala (pulgadas por cm) para que el carro y la varilla quepan."""
        ancho_in, alto_in = self.fig.get_size_inches()
        alto_cm = ALTURA_PIVOTE_CM + LARGO_VARILLA_CM + 5
        self.k = max(1e-3, min((alto_in - self.ABAJO_IN - 0.05) / alto_cm, (ancho_in - 0.4) / 60.0))
        self.pt_cm = self.k * 72                          # puntos por cm, para grosores
        self.medio_cm = (ancho_in / 2) / self.k
        self.ax.set_xlim(self.centro_cm - self.medio_cm, self.centro_cm + self.medio_cm)
        self.ax.set_ylim(-self.ABAJO_IN / self.k, (alto_in - self.ABAJO_IN) / self.k)

    def _rehacer_fondo(self):
        """Cuadrícula de 10 cm, piso, regla y textos fijos."""
        for artista in self.estaticos:
            artista.remove()
        ax = self.ax
        x0, x1 = ax.get_xlim()
        y0 = ax.get_ylim()[0]
        alto_cm = ALTURA_PIVOTE_CM + LARGO_VARILLA_CM + 4
        e = []
        lineas = [[(x, 0), (x, alto_cm)] for x in range(math.ceil(x0 / 10) * 10, math.floor(x1) + 1, 10)]
        lineas += [[(x0, y), (x1, y)] for y in range(10, int(alto_cm) + 1, 10)]
        e.append(ax.add_collection(LineCollection(lineas, colors="#e8ecf0", linewidths=0.8, zorder=0)))
        e.append(ax.add_patch(Rectangle((x0, y0), x1 - x0, -y0, facecolor=GRIS_FONDO,
                                        edgecolor="none", zorder=0)))
        e.append(ax.plot([x0, x1], [0, 0], color="#7b8794", lw=1.6, zorder=1)[0])
        marcas = []
        for x in range(math.ceil(x0 / 5) * 5, math.floor(x1) + 1, 5):
            mayor = x % 10 == 0
            marcas.append([(x, 0), (x, -(7 if mayor else 3.5) / 72 / self.k)])
            if mayor:
                e.append(ax.annotate(str(x).replace("-", "−"), xy=(x, 0), xytext=(0, -10),
                                     textcoords="offset points", ha="center", va="top",
                                     fontsize=8.5, color=TENUE))
        e.append(ax.add_collection(LineCollection(marcas, colors="#7b8794", linewidths=1.0, zorder=1)))
        e.append(ax.annotate("x (cm)", xy=(0, 0), xycoords="axes fraction", xytext=(12, 10),
                             textcoords="offset points", ha="left", va="bottom", fontsize=8.5, color=TENUE))
        nota = "" if MEDIDAS_CONFIRMADAS else "  (medidas por confirmar)"
        e.append(ax.annotate(f"Péndulo {MASA_PENDULO_G:g} g  ·  Carro {MASA_CARRO_G:g} g\n"
                             f"Varilla {LARGO_VARILLA_CM:g} cm  ·  Rueda Ø {DIAMETRO_RUEDA_CM:g} cm{nota}",
                             xy=(0, 1), xycoords="axes fraction", xytext=(12, -10),
                             textcoords="offset points", ha="left", va="top", fontsize=8.5,
                             color=TENUE, linespacing=1.6))
        self.estaticos = e

    # --- piezas que se mueven ---
    def _colocar_piezas(self):
        X0, Yp, L, pt = self.x_cm, ALTURA_PIVOTE_CM, LARGO_VARILLA_CM, self.pt_cm
        tope = min(11.4, ALTURA_PIVOTE_CM - 1.5)

        self.i_sombra.set_center((X0, 0.1))
        self.i_sombra.set_width(SEPARACION_RUEDAS_CM + DIAMETRO_RUEDA_CM + 6)
        self.i_sombra.set_height(1.3)
        self.i_chasis.set_bounds(X0 - 9.0, 3.6, 18.0, 1.4)
        self.i_caja.set_bounds(X0 - 5.3, 5.0, 10.6, tope - 5.0)
        self.i_placa.set_bounds(X0 - 3.6, 0.55 * tope + 1.8, 6.1, 2.0)
        self.i_mastil.set_bounds(X0 - 0.7, tope, 1.4, Yp - tope)
        self.i_as5600.set_bounds(X0 - 1.5, Yp - 1.5, 3.0, 3.0)

        r = DIAMETRO_RUEDA_CM / 2
        giro = -self.x_cm / r                        # avanzar a la derecha = giro horario
        for (llanta, rin, rayos, buje), dx in zip(self.ruedas, (-SEPARACION_RUEDAS_CM / 2,
                                                                SEPARACION_RUEDAS_CM / 2)):
            cx = X0 + dx
            llanta.set_center((cx, r))
            llanta.set_radius(r)
            rin.set_center((cx, r))
            rin.set_radius(0.62 * r)
            xs, ys = [], []
            for j in range(3):
                a = giro + j * 2 * math.pi / 3
                xs += [cx, cx + 0.56 * r * math.cos(a), np.nan]
                ys += [r, r + 0.56 * r * math.sin(a), np.nan]
            rayos.set_data(xs, ys)
            rayos.set_linewidth(max(1.0, 0.32 * pt))
            buje.set_center((cx, r))
            buje.set_radius(0.17 * r)

        # Péndulo
        ok = self.theta_ok
        th = math.radians(self.theta)
        punta = (X0 + L * math.sin(th), Yp + L * math.cos(th))
        grosor = max(3.0, 0.9 * pt)
        self.i_varilla.set_data([X0, punta[0]], [Yp, punta[1]])
        self.i_varilla.set_linewidth(grosor)
        self.i_varilla.set_color(ORO if ok else "#b9c0c8")
        self.i_varilla.set_path_effects([patheffects.withStroke(
            linewidth=grosor + 1.6, foreground=ORO_OSCURO if ok else "#9aa3ad")])
        self.i_pivote.set_center((X0, Yp))
        self.i_pivote.set_radius(0.8)
        self.i_ref.set_data([X0, X0], [Yp, Yp + L + 2])

        R = 0.4 * L
        visible = ok and abs(self.theta) >= 0.05
        a1, a2 = sorted((90.0, 90.0 - self.theta))
        self.i_cuna.set_center((X0, Yp))
        self.i_cuna.set_radius(R)
        self.i_cuna.set_theta1(a1)
        self.i_cuna.set_theta2(a2 if a2 > a1 else a1 + 0.01)
        self.i_cuna.set_visible(visible)
        angulos = np.radians(np.linspace(a1, a2, 24))
        self.i_arco.set_data(X0 + R * np.cos(angulos), Yp + R * np.sin(angulos))
        self.i_arco.set_visible(visible)
        lado = 1 if self.theta >= 0 else -1
        self.i_txt_theta.xy = (X0, Yp + 0.55 * R)
        self.i_txt_theta.set_position((-lado * 12, 0))
        self.i_txt_theta.set_horizontalalignment("right" if lado > 0 else "left")
        self.i_txt_theta.set_text(f"θ {con_signo(self.theta, 1)}°" if ok else "θ —")
        self.i_txt_theta.set_color(ROJO_U if ok else TENUE)

        # Estela de la punta: se desvanece hacia atrás
        puntos = [(x + L * math.sin(math.radians(a)), Yp + L * math.cos(math.radians(a)))
                  for _, x, a in self.estela]
        if len(puntos) >= 2:
            n = len(puntos) - 1
            rgb = to_rgba(ORO)[:3]
            self.i_estela.set_segments([[puntos[i], puntos[i + 1]] for i in range(n)])
            self.i_estela.set_color([(*rgb, 0.05 + 0.5 * (i + 1) / n) for i in range(n)])
            self.i_estela.set_linewidth(max(2.0, 0.55 * pt))
            self.i_estela.set_visible(True)
        else:
            self.i_estela.set_visible(False)

        # Flecha del PWM promedio (hacia dónde y con cuánta fuerza empuja el control)
        y = -0.50 / self.k
        hay_flecha = self.hay_datos and abs(self.pwm) > 3
        if hay_flecha:
            d = 1 if self.pwm > 0 else -1
            largo = min(abs(self.pwm), 120) / 100 * 0.36 * self.medio_cm
            self.i_flecha.set_positions((X0, y), (X0 + d * largo, y))
            self.i_txt_pwm.xy = (X0 + d * largo, y)
            self.i_txt_pwm.set_position((d * 10, 0))
            self.i_txt_pwm.set_horizontalalignment("left" if d > 0 else "right")
            self.i_txt_pwm.set_text(f"PWM {con_signo(self.pwm, 0)} %")
        self.i_flecha.set_visible(hay_flecha)
        self.i_txt_pwm.set_visible(hay_flecha)
        self.i_aviso.set_visible(not self.hay_datos)

    # --- dibujo ---
    def _al_redimensionar(self, _evento):
        self._ajustar_vista()
        self._rehacer_fondo()
        self._colocar_piezas()        # luego matplotlib redibuja todo solo

    def _al_dibujar_todo(self, _evento):
        """Tras cada dibujo completo guarda el fondo y pinta las piezas encima."""
        self.fondo = self.lienzo.copy_from_bbox(self.fig.bbox)
        self._dibujar_piezas()

    def _dibujar_piezas(self):
        self.lienzo.restore_region(self.fondo)
        for pieza in self.piezas:
            self.ax.draw_artist(pieza)
        self.lienzo.blit(self.fig.bbox)

    def _redibujar(self):
        if self.fondo is None:
            self.lienzo.draw()
        else:
            self._dibujar_piezas()

    def reiniciar(self):
        self.estela.clear()
        self.hay_datos = False
        self.x_cm = self.theta = self.pwm = 0.0
        self.theta_ok = True
        if self.centro_cm != 0:
            self.centro_cm = 0
            self._ajustar_vista()
            self._rehacer_fondo()
            self._colocar_piezas()
            self.lienzo.draw()
            return
        self._colocar_piezas()
        self._redibujar()

    def mostrar(self, x_cm, theta_deg, theta_ok, pwm, t_s):
        if math.isfinite(x_cm):
            if abs(x_cm - self.x_cm) > 5:      # salto brusco (reinicio): la estela no aplica
                self.estela.clear()
            self.x_cm = x_cm
        if theta_ok:
            self.theta = theta_deg
            self.estela.append((t_s, self.x_cm, theta_deg))
        while self.estela and t_s - self.estela[0][0] > 1.0:
            self.estela.popleft()
        self.theta_ok = theta_ok
        self.pwm = pwm if math.isfinite(pwm) else 0.0
        self.hay_datos = True
        # Si el carro se acerca al borde, la vista se recentra (pasos de 10 cm)
        if abs(self.x_cm - self.centro_cm) > self.medio_cm - 12:
            self.centro_cm = int(round(self.x_cm / 10) * 10)
            self._ajustar_vista()
            self._rehacer_fondo()
            self._colocar_piezas()
            self.lienzo.draw()
            return
        self._colocar_piezas()
        self._redibujar()


# -----------------------------------------------------------------
#  Ventana principal
# -----------------------------------------------------------------
class Aplicacion:
    def __init__(self, raiz):
        self.raiz = raiz
        raiz.title("Péndulo invertido sobre carro · Telemetría")
        raiz.configure(bg=FONDO)
        raiz.geometry(f"{px(1440)}x{px(860)}")
        raiz.minsize(px(1100), px(700))
        try:
            raiz.state("zoomed")          # maximizada en Windows
        except tk.TclError:
            pass

        self.cola = queue.Queue()
        self.lector = None
        self.archivo_csv = None
        self.escritor_csv = None
        self.ultima = None
        self.llegadas = deque()          # instantes de llegada, para medir tramas/s
        self.ultima_llegada = None
        self.conectado_desde = None
        self.perdidas = 0
        self.pausado = False
        self.ultimo_dibujo = 0.0
        self.pendiente_vista = False
        self.ventana_mas = None
        self.graficas_mas = []
        self._reiniciar_series()

        self._crear_encabezado()
        self._crear_cuerpo()
        self._cambiar_modo()
        self._poner_estado("Desconectado", "rojo")
        self._actualizar_chips(None)
        self._actualizar_contadores()

        raiz.protocol("WM_DELETE_WINDOW", self._cerrar)
        raiz.after(REFRESCO_DATOS_MS, self._ciclo)

    # ---------------- construcción de la interfaz ----------------
    def _crear_encabezado(self):
        cab = tk.Frame(self.raiz, bg=PANEL)
        cab.pack(side=tk.TOP, fill=tk.X)
        tk.Frame(self.raiz, bg=ROJO_U, height=px(3)).pack(side=tk.TOP, fill=tk.X)
        fila = tk.Frame(cab, bg=PANEL)
        fila.pack(fill=tk.X, padx=px(20), pady=px(8))

        # Logos
        self.logos = [cargar_logo(LOGO_UNILLANOS, px(46)), cargar_logo(LOGO_FACULTAD, px(50))]
        if self.logos[0]:
            tk.Label(fila, image=self.logos[0], bg=PANEL, bd=0).pack(side=tk.LEFT)
        if self.logos[0] and self.logos[1]:
            tk.Frame(fila, bg=BORDE, width=1, height=px(40)).pack(side=tk.LEFT, padx=px(14))
        if self.logos[1]:
            tk.Label(fila, image=self.logos[1], bg=PANEL, bd=0).pack(side=tk.LEFT)

        textos = tk.Frame(fila, bg=PANEL)
        textos.pack(side=tk.LEFT, padx=(px(20), 0))
        if TITULO:
            tk.Label(textos, text=TITULO, bg=PANEL, fg=TEXTO,
                     font=(FUENTE, 15, "bold")).pack(anchor="w")
        tk.Label(textos, text=SUBTITULO, bg=PANEL, fg=TENUE, font=(FUENTE, 10)).pack(anchor="w")

        # Controles de conexión
        controles = tk.Frame(fila, bg=PANEL)
        controles.pack(side=tk.RIGHT)
        selector = tk.Frame(controles, bg=BORDE_FUERTE, padx=1, pady=1)
        selector.pack(side=tk.LEFT)
        self.modo = tk.StringVar(value="wifi")
        self.radios = {}
        for valor, texto in (("wifi", "WiFi"), ("sim", "Simulación")):
            boton = tk.Radiobutton(selector, text=texto, value=valor, variable=self.modo,
                                   indicatoron=0, command=self._cambiar_modo,
                                   font=(FUENTE, 10, "bold"), bd=0, relief=tk.FLAT,
                                   offrelief=tk.FLAT, padx=px(12), pady=px(6), cursor="hand2",
                                   highlightthickness=0)
            boton.pack(side=tk.LEFT)
            self.radios[valor] = boton

        tk.Label(controles, text="IP", bg=PANEL, fg=TENUE, font=(FUENTE, 9)).pack(
            side=tk.LEFT, padx=(px(14), px(4)))
        self.ent_ip = tk.Entry(controles, width=13, font=(FUENTE_NUM, 10), relief=tk.FLAT, bd=0,
                               highlightthickness=1, highlightbackground=BORDE_FUERTE,
                               highlightcolor=AZUL, disabledbackground=GRIS_FONDO)
        self.ent_ip.insert(0, IP_DEFECTO)
        self.ent_ip.pack(side=tk.LEFT, ipady=px(6), ipadx=px(4))
        tk.Label(controles, text="Puerto", bg=PANEL, fg=TENUE, font=(FUENTE, 9)).pack(
            side=tk.LEFT, padx=(px(10), px(4)))
        self.ent_puerto = tk.Entry(controles, width=6, font=(FUENTE_NUM, 10), relief=tk.FLAT, bd=0,
                                   highlightthickness=1, highlightbackground=BORDE_FUERTE,
                                   highlightcolor=AZUL, disabledbackground=GRIS_FONDO)
        self.ent_puerto.insert(0, str(PUERTO_DEFECTO))
        self.ent_puerto.pack(side=tk.LEFT, ipady=px(6), ipadx=px(4))

        marco, self.btn_conectar = crear_boton(controles, "Conectar", self._alternar_conexion,
                                               primario=True)
        marco.pack(side=tk.LEFT, padx=(px(12), px(8)))
        self.lbl_estado = tk.Label(controles, text="", font=(FUENTE, 10, "bold"),
                                   padx=px(12), pady=px(7))
        self.lbl_estado.pack(side=tk.LEFT)

    def _crear_cuerpo(self):
        cuerpo = tk.Frame(self.raiz, bg=FONDO)
        cuerpo.pack(fill=tk.BOTH, expand=True, padx=px(16), pady=px(12))
        cuerpo.columnconfigure(0, weight=58, uniform="columnas")
        cuerpo.columnconfigure(1, weight=42, uniform="columnas")
        cuerpo.rowconfigure(0, weight=1)

        izquierda = tk.Frame(cuerpo, bg=FONDO)
        izquierda.grid(row=0, column=0, sticky="nsew", padx=(0, px(8)))
        derecha = tk.Frame(cuerpo, bg=FONDO)
        derecha.grid(row=0, column=1, sticky="nsew", padx=(px(8), 0))

        # --- Animación ---
        panel = crear_panel(izquierda)
        panel.pack(fill=tk.BOTH, expand=True)
        enc = tk.Frame(panel, bg=PANEL)
        enc.pack(fill=tk.X, padx=px(14), pady=px(8))
        textos = tk.Frame(enc, bg=PANEL)
        textos.pack(side=tk.LEFT)
        tk.Label(textos, text="Vista lateral en vivo", bg=PANEL, fg=TEXTO,
                 font=(FUENTE, 11, "bold")).pack(anchor="w")
        tk.Label(textos, text="Escala real · cuadrícula de 10 cm · la flecha azul es el PWM promedio",
                 bg=PANEL, fg=TENUE, font=(FUENTE, 9)).pack(anchor="w")
        marco, self.btn_pausa = crear_boton(enc, "Pausar", self._alternar_pausa)
        marco.pack(side=tk.RIGHT)
        marco, self.btn_empujon = crear_boton(enc, "Simular empujón", self._empujon, primario=True)
        marco.pack(side=tk.RIGHT, padx=(0, px(8)))
        tk.Frame(panel, bg=FONDO, height=1).pack(fill=tk.X)
        self.anim = Animacion(panel)
        self.anim.widget.pack(fill=tk.BOTH, expand=True)

        # --- Lecturas ---
        lecturas = tk.Frame(izquierda, bg=FONDO)
        lecturas.pack(fill=tk.X, pady=(px(12), 0))
        self.lbl = {}
        definiciones = (("theta", "Ángulo θ", "°", ROJO_U), ("omega", "Vel. angular ω", "°/s", TEXTO),
                        ("x", "Posición x", "cm", TEXTO), ("v", "Velocidad v", "cm/s", TEXTO),
                        ("rpm_izq", "RPM izquierda", "", AZUL), ("rpm_der", "RPM derecha", "", NARANJA_TXT))
        for i, (clave, titulo, unidad, color) in enumerate(definiciones):
            tarjeta = crear_panel(lecturas)
            tarjeta.grid(row=0, column=i, sticky="ew", padx=(0 if i == 0 else px(8), 0))
            lecturas.columnconfigure(i, weight=1, uniform="lecturas")
            tk.Label(tarjeta, text=titulo, bg=PANEL, fg=TENUE, font=(FUENTE, 9)).pack(
                anchor="w", padx=px(12), pady=(px(7), 0))
            fila = tk.Frame(tarjeta, bg=PANEL)
            fila.pack(anchor="w", padx=px(12), pady=(0, px(7)))
            valor = tk.Label(fila, text="—", bg=PANEL, fg=color, font=(FUENTE_NUM, 17, "bold"))
            valor.pack(side=tk.LEFT)
            if unidad:
                tk.Label(fila, text=" " + unidad, bg=PANEL, fg=TENUE, font=(FUENTE, 11)).pack(
                    side=tk.LEFT, anchor="s", pady=(0, px(3)))
            self.lbl[clave] = valor

        # --- Gráfica de θ ---
        panel = crear_panel(derecha)
        panel.pack(fill=tk.BOTH, expand=True)
        enc = tk.Frame(panel, bg=PANEL)
        enc.pack(fill=tk.X, padx=px(14), pady=(px(8), 0))
        tk.Label(enc, text="Ángulo del péndulo θ · sensor AS5600", bg=PANEL, fg=TEXTO,
                 font=(FUENTE, 11, "bold")).pack(side=tk.LEFT)
        self.lbl_theta_graf = tk.Label(enc, text="—", bg=PANEL, fg=ROJO_U,
                                       font=(FUENTE_NUM, 15, "bold"))
        self.lbl_theta_graf.pack(side=tk.RIGHT)
        self.graf_theta = GraficaTiempo(panel, [("theta", ROJO_U, "-", 2.2)], rango_min=20,
                                        centrado=True, banda=ZONA_EQUILIBRIO_DEG)
        self.graf_theta.widget.pack(fill=tk.BOTH, expand=True, padx=px(6))
        leyenda = tk.Frame(panel, bg=PANEL)
        leyenda.pack(fill=tk.X, padx=px(14), pady=(0, px(8)))
        tk.Label(leyenda, text="■", bg=PANEL, fg="#b7dfc3", font=(FUENTE, 10)).pack(side=tk.LEFT)
        tk.Label(leyenda, text=f"zona de equilibrio ±{ZONA_EQUILIBRIO_DEG:g}°", bg=PANEL, fg=TENUE,
                 font=(FUENTE, 9)).pack(side=tk.LEFT, padx=(px(2), px(14)))
        tk.Label(leyenda, text="╌╌", bg=PANEL, fg="#7b8794", font=(FUENTE, 9)).pack(side=tk.LEFT)
        tk.Label(leyenda, text="referencia 0°", bg=PANEL, fg=TENUE, font=(FUENTE, 9)).pack(
            side=tk.LEFT, padx=(px(2), 0))

        # --- Gráfica del PWM ---
        panel = crear_panel(derecha)
        panel.pack(fill=tk.BOTH, expand=True, pady=(px(12), 0))
        enc = tk.Frame(panel, bg=PANEL)
        enc.pack(fill=tk.X, padx=px(14), pady=(px(8), 0))
        tk.Label(enc, text="Señal de control · PWM de los motores", bg=PANEL, fg=TEXTO,
                 font=(FUENTE, 11, "bold")).pack(side=tk.LEFT)
        valores = tk.Frame(enc, bg=PANEL)
        valores.pack(side=tk.RIGHT)
        self.lbl_pwm_izq = tk.Label(valores, text="—", bg=PANEL, fg=AZUL, font=(FUENTE_NUM, 15, "bold"))
        self.lbl_pwm_izq.pack(side=tk.LEFT)
        tk.Label(valores, text=" / ", bg=PANEL, fg=TENUE, font=(FUENTE_NUM, 15, "bold")).pack(side=tk.LEFT)
        self.lbl_pwm_der = tk.Label(valores, text="—", bg=PANEL, fg=NARANJA_TXT,
                                    font=(FUENTE_NUM, 15, "bold"))
        self.lbl_pwm_der.pack(side=tk.LEFT)
        tk.Label(valores, text=" %", bg=PANEL, fg=TENUE, font=(FUENTE, 11)).pack(side=tk.LEFT)
        self.graf_pwm = GraficaTiempo(panel, [("pwm_izq", AZUL, "-", 2.0),
                                              ("pwm_der", NARANJA, (0, (7, 3)), 2.0)],
                                      rango_min=200, centrado=True, guias=(100, -100))
        self.graf_pwm.widget.pack(fill=tk.BOTH, expand=True, padx=px(6))
        leyenda = tk.Frame(panel, bg=PANEL)
        leyenda.pack(fill=tk.X, padx=px(14), pady=(0, px(8)))
        for simbolo, color, texto in (("━━", AZUL, "motor izquierdo"), ("╍╍", NARANJA, "motor derecho"),
                                      ("┄┄", "#aab3bc", "±100 % = saturación")):
            tk.Label(leyenda, text=simbolo, bg=PANEL, fg=color, font=(FUENTE, 9, "bold")).pack(side=tk.LEFT)
            tk.Label(leyenda, text=texto, bg=PANEL, fg=TENUE, font=(FUENTE, 9)).pack(
                side=tk.LEFT, padx=(px(3), px(14)))

        # --- Franja inferior: estado y botones ---
        franja = crear_panel(cuerpo)
        franja.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(px(12), 0))
        interior = tk.Frame(franja, bg=PANEL)
        interior.pack(fill=tk.X, padx=px(14), pady=px(8))
        tk.Label(interior, text="ESTADO STM32", bg=PANEL, fg=TENUE,
                 font=(FUENTE, 9, "bold")).pack(side=tk.LEFT, padx=(0, px(8)))
        self.chips = []
        for _ in FLAGS:
            chip = tk.Label(interior, text="", font=(FUENTE, 9, "bold"), padx=px(10), pady=px(4))
            chip.pack(side=tk.LEFT, padx=(0, px(6)))
            self.chips.append(chip)
        self.lbl_contadores = tk.Label(interior, text="", bg=PANEL, fg=TENUE, font=(FUENTE_NUM, 10))
        self.lbl_contadores.pack(side=tk.LEFT, padx=(px(16), 0))
        marco, _ = crear_boton(interior, "Más variables", self._abrir_mas_variables)
        marco.pack(side=tk.RIGHT)
        marco, self.btn_csv = crear_boton(interior, "Grabar CSV", self._alternar_csv)
        marco.pack(side=tk.RIGHT, padx=(0, px(8)))

    # ---------------- estado de la conexión ----------------
    def _poner_estado(self, texto, tono):
        fondo, color = {"verde": (VERDE_FONDO, VERDE_TXT), "ambar": (AMBAR_FONDO, AMBAR_TXT),
                        "rojo": (ROJO_FONDO, ROJO_TXT)}[tono]
        self.lbl_estado.config(text="● " + texto, bg=fondo, fg=color)

    def _cambiar_modo(self):
        conectado = self.lector is not None
        modo = self.modo.get()
        for valor, boton in self.radios.items():
            elegido = valor == modo
            boton.config(bg=TEXTO if elegido else PANEL, fg="#ffffff" if elegido else TEXTO,
                         selectcolor=TEXTO, activebackground=TEXTO if elegido else GRIS_FONDO,
                         activeforeground="#ffffff" if elegido else TEXTO,
                         state=tk.DISABLED if conectado and not elegido else tk.NORMAL,
                         disabledforeground="#9aa5b1")
        estado_campos = tk.NORMAL if modo == "wifi" and not conectado else tk.DISABLED
        self.ent_ip.config(state=estado_campos)
        self.ent_puerto.config(state=estado_campos)
        self.btn_empujon.config(state=tk.NORMAL if conectado and modo == "sim" else tk.DISABLED)

    def _alternar_conexion(self):
        if self.lector:
            self._desconectar("Desconectado", "rojo")
            return
        modo = self.modo.get()
        ip, puerto = "", 0
        if modo == "wifi":
            ip = self.ent_ip.get().strip()
            try:
                puerto = int(self.ent_puerto.get())
                if not 0 < puerto < 65536:
                    raise ValueError
            except ValueError:
                messagebox.showwarning("Conexión", "El puerto no es válido.")
                return
            if not ip:
                messagebox.showwarning("Conexión", "Escribe la IP de la ESP32-C3.")
                return

        self._reiniciar_series()
        self.anim.reiniciar()
        self.perdidas = 0
        self.ultima = None
        self.ultima_llegada = None
        self.conectado_desde = None
        self.lector = Lector(modo, ip, puerto, self.cola)
        self.lector.start()
        self.btn_conectar.config(text="Desconectar")
        self._poner_estado("Conectando…", "ambar")
        self._cambiar_modo()

    def _desconectar(self, mensaje, tono):
        if self.lector:
            self.lector.detener()
            self.lector = None
        self.conectado_desde = None
        self.btn_conectar.config(text="Conectar")
        self._poner_estado(mensaje, tono)
        self._cambiar_modo()

    # ---------------- botones ----------------
    def _empujon(self):
        if self.lector and self.lector.modo == "sim":
            self.lector.empujar()

    def _alternar_pausa(self):
        self.pausado = not self.pausado
        self.btn_pausa.config(text="Reanudar" if self.pausado else "Pausar")
        if not self.pausado:
            self.pendiente_vista = True

    # ---------------- datos ----------------
    def _reiniciar_series(self):
        self.t = deque(maxlen=MAX_PUNTOS)
        self.series = {nombre: deque(maxlen=MAX_PUNTOS) for nombre in
                       ("theta", "omega", "x", "v", "pwm_izq", "pwm_der", "rpm_izq", "rpm_der")}
        self.t_s = 0.0
        self.seq_ant = None
        self.anterior = None           # (t_us, enc_izq, enc_der) de la trama anterior
        self.pendiente_dibujo = True

    def _nueva_trama(self, tr):
        ahora = time.monotonic()
        self.llegadas.append(ahora)
        self.ultima_llegada = ahora

        # Número de trama: detecta pérdidas y reinicios de la STM32
        if self.seq_ant is not None:
            if tr.seq == self.seq_ant:
                return                          # trama repetida
            if tr.seq < self.seq_ant:
                self._reiniciar_series()        # la STM32 se reinició
            elif tr.seq - self.seq_ant > 1:
                self.perdidas += tr.seq - self.seq_ant - 1

        # Tiempo y RPM (las cuentas y t_us dan la vuelta a 32 bits)
        rpm_izq = rpm_der = math.nan
        if self.anterior is not None:
            t_ant, izq_ant, der_ant = self.anterior
            dt = ((tr.t_us - t_ant) % 2**32) * 1e-6
            self.t_s += dt
            if 0.015 <= dt <= 0.3:
                rpm_izq = a_int32(tr.enc_izq - izq_ant) * 60.0 / (CPR_IZQ * dt)
                rpm_der = SIGNO_RPM_DERECHO * a_int32(tr.enc_der - der_ant) * 60.0 / (CPR_DER * dt)
        self.anterior = (tr.t_us, tr.enc_izq, tr.enc_der)
        self.seq_ant = tr.seq

        # El ángulo solo vale si el AS5600 leyó bien
        as_ok = (tr.flags & 0x01) == 0 and tr.as_crudo != 0xFFFF
        theta = math.degrees(tr.theta_rad) if as_ok and math.isfinite(tr.theta_rad) else math.nan
        omega = math.degrees(tr.omega_rad_s) if as_ok and math.isfinite(tr.omega_rad_s) else math.nan
        x = tr.x_m * 100.0 if math.isfinite(tr.x_m) else math.nan
        v = tr.v_m_s * 100.0 if math.isfinite(tr.v_m_s) else math.nan

        self.t.append(self.t_s)
        for nombre, valor in (("theta", theta), ("omega", omega), ("x", x), ("v", v),
                              ("pwm_izq", tr.pwm_izq), ("pwm_der", tr.pwm_der),
                              ("rpm_izq", rpm_izq), ("rpm_der", rpm_der)):
            self.series[nombre].append(valor)
        self.pendiente_dibujo = True

        self.ultima = {"tr": tr, "theta": theta, "omega": omega, "x": x, "v": v,
                       "rpm_izq": rpm_izq, "rpm_der": rpm_der, "as_ok": as_ok, "t_s": self.t_s}

        if self.escritor_csv:
            def num(valor, decimales):
                return "" if not math.isfinite(valor) else f"{valor:.{decimales}f}"
            self.escritor_csv.writerow([
                f"{time.time():.3f}", tr.seq, tr.t_us, f"{self.t_s:.4f}",
                num(theta, 3), num(omega, 2), num(x, 3), num(v, 3),
                tr.enc_izq, tr.enc_der, num(rpm_izq, 2), num(rpm_der, 2),
                tr.pwm_izq, tr.pwm_der, tr.as_crudo, tr.as_estado, int(as_ok),
                tr.flags, tr.missed,
            ])

    # ---------------- CSV ----------------
    def _alternar_csv(self):
        if self.archivo_csv:
            self.archivo_csv.close()
            self.archivo_csv = self.escritor_csv = None
            self.btn_csv.config(text="Grabar CSV")
            return
        ruta = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=time.strftime("pendulo_%Y%m%d_%H%M%S.csv"))
        if not ruta:
            return
        try:
            self.archivo_csv = open(ruta, "w", newline="", encoding="utf-8")
        except OSError as error:
            messagebox.showerror("CSV", f"No se pudo crear el archivo:\n{error}")
            return
        self.escritor_csv = csv.writer(self.archivo_csv)
        self.escritor_csv.writerow([
            "hora_pc", "seq", "t_stm32_us", "t_s", "theta_deg", "omega_deg_s", "x_cm", "v_cm_s",
            "enc_izq", "enc_der", "rpm_izq", "rpm_der", "pwm_izq", "pwm_der",
            "as5600_crudo", "as5600_estado", "as5600_valido", "flags", "missed_stm32",
        ])
        self.btn_csv.config(text="Detener CSV")

    # ---------------- ventana "Más variables" ----------------
    def _abrir_mas_variables(self):
        if self.ventana_mas is not None:
            self.ventana_mas.lift()
            return
        ventana = tk.Toplevel(self.raiz)
        ventana.title("Más variables · carro péndulo")
        ventana.configure(bg=FONDO)
        ventana.geometry(f"{px(1100)}x{px(680)}")
        definiciones = (
            ("Velocidad angular ω (°/s)", [("omega", "#6f42a8", "-", 2.0)], 40),
            ("Posición del carro x (cm)", [("x", "#2a8f4a", "-", 2.0)], 4),
            ("Velocidad del carro v (cm/s)", [("v", "#7a6a00", "-", 2.0)], 10),
            ("Velocidad de las ruedas (RPM) · izquierda / derecha",
             [("rpm_izq", AZUL, "-", 2.0), ("rpm_der", NARANJA, (0, (7, 3)), 2.0)], 20),
        )
        self.graficas_mas = []
        for i, (titulo, curvas, rango_min) in enumerate(definiciones):
            panel = crear_panel(ventana)
            panel.grid(row=i // 2, column=i % 2, sticky="nsew", padx=px(8), pady=px(8))
            tk.Label(panel, text=titulo, bg=PANEL, fg=TEXTO, font=(FUENTE, 11, "bold")).pack(
                anchor="w", padx=px(14), pady=(px(8), 0))
            grafica = GraficaTiempo(panel, curvas, rango_min, alto=2.6)
            grafica.widget.pack(fill=tk.BOTH, expand=True, padx=px(6), pady=(0, px(6)))
            self.graficas_mas.append(grafica)
        ventana.columnconfigure((0, 1), weight=1)
        ventana.rowconfigure((0, 1), weight=1)
        self.lbl_encoders = tk.Label(ventana, text="", bg=FONDO, fg=TENUE, font=(FUENTE_NUM, 10))
        self.lbl_encoders.grid(row=2, column=0, columnspan=2, sticky="w", padx=px(14), pady=(0, px(8)))
        ventana.protocol("WM_DELETE_WINDOW", self._cerrar_mas_variables)
        self.ventana_mas = ventana
        self.pendiente_dibujo = True
        if self.ultima:
            self._actualizar_numeros()

    def _cerrar_mas_variables(self):
        if self.ventana_mas is not None:
            self.ventana_mas.destroy()
        self.ventana_mas = None
        self.graficas_mas = []

    # ---------------- actualización de la vista ----------------
    def _ciclo(self):
        hubo_datos = False
        for _ in range(2000):   # límite para que la ventana nunca se congele
            try:
                origen, tipo, valor = self.cola.get_nowait()
            except queue.Empty:
                break
            if origen is not self.lector:
                continue        # mensajes de una conexión anterior
            if tipo == "dato":
                self._nueva_trama(valor)
                hubo_datos = True
            elif tipo == "estado":
                self.conectado_desde = time.monotonic()
                self._poner_estado(f"Conectado · {valor}", "verde")
            elif tipo == "error":
                self._desconectar(f"Error: {valor}"[:90], "rojo")
            elif tipo == "fin":
                self._desconectar("Conexión cerrada", "rojo")

        if (hubo_datos or self.pendiente_vista) and self.ultima and not self.pausado:
            self._actualizar_numeros()
            u = self.ultima
            self.anim.mostrar(u["x"], u["theta"], u["as_ok"],
                              (u["tr"].pwm_izq + u["tr"].pwm_der) / 2, u["t_s"])
            self.pendiente_vista = False
        self._actualizar_estado()

        ahora = time.monotonic()
        if (self.pendiente_dibujo and not self.pausado
                and (ahora - self.ultimo_dibujo) * 1000 >= REFRESCO_GRAFICA_MS):
            self._actualizar_graficas()
            self.ultimo_dibujo = ahora
            self.pendiente_dibujo = False

        self.raiz.after(REFRESCO_DATOS_MS, self._ciclo)

    def _actualizar_numeros(self):
        u = self.ultima
        tr = u["tr"]
        self.lbl["theta"].config(text=con_signo(u["theta"], 1))
        self.lbl["omega"].config(text=con_signo(u["omega"], 0))
        self.lbl["x"].config(text=con_signo(u["x"], 1))
        self.lbl["v"].config(text=con_signo(u["v"], 1))
        self.lbl["rpm_izq"].config(text=con_signo(u["rpm_izq"], 0))
        self.lbl["rpm_der"].config(text=con_signo(u["rpm_der"], 0))
        theta_txt = con_signo(u["theta"], 1)
        self.lbl_theta_graf.config(text=theta_txt if theta_txt == "—" else theta_txt + "°")
        self.lbl_pwm_izq.config(text=con_signo(tr.pwm_izq, 0))
        self.lbl_pwm_der.config(text=con_signo(tr.pwm_der, 0))
        self._actualizar_chips(tr)
        if self.ventana_mas is not None:
            self.lbl_encoders.config(text=f"Encoders: izquierdo {tr.enc_izq} · derecho {tr.enc_der} "
                                          f"cuentas   ·   AS5600 crudo {tr.as_crudo} · STATUS 0x{tr.as_estado:02X}")

    def _actualizar_chips(self, tr):
        for chip, (bit, normal, activo, gravedad) in zip(self.chips, FLAGS):
            if tr is None:
                chip.config(text=normal.split()[0], bg=GRIS_FONDO, fg=TENUE)
                continue
            falla = bool(tr.flags & bit) or (bit == 0x01 and tr.as_crudo == 0xFFFF)
            if not falla:
                chip.config(text="● " + normal, bg=VERDE_FONDO, fg=VERDE_TXT)
            elif gravedad == "rojo":
                chip.config(text="● " + activo, bg=ROJO_FONDO, fg=ROJO_TXT)
            else:
                chip.config(text="● " + activo, bg=AMBAR_FONDO, fg=AMBAR_TXT)

    def _actualizar_contadores(self):
        receptor = self.lector.receptor if self.lector else None
        seq = self.ultima["tr"].seq if self.ultima else "—"
        crc = receptor.errores_crc if receptor else 0
        self.lbl_contadores.config(text=f"Trama {seq} · perdidas {self.perdidas} · CRC {crc}")

    def _actualizar_estado(self):
        # Promedio de 2 s: por WiFi las tramas llegan en ráfagas
        ahora = time.monotonic()
        while self.llegadas and ahora - self.llegadas[0] > 2.0:
            self.llegadas.popleft()
        tramas_s = round(len(self.llegadas) / 2)
        self._actualizar_contadores()

        if self.lector and self.conectado_desde is not None:
            referencia = self.ultima_llegada or self.conectado_desde
            sin = ahora - referencia
            fuente = "Simulación" if self.lector.modo == "sim" else "Conectado"
            pausa = " · en pausa" if self.pausado else ""
            if sin > SIN_DATOS_S:
                aviso = ("esperando datos de la STM32" if self.ultima_llegada is None
                         else f"sin datos hace {sin:.0f} s")
                self._poner_estado(f"{fuente} · {aviso}", "ambar")
            else:
                self._poner_estado(f"{fuente} · {tramas_s} tramas/s{pausa}", "verde")

    def _actualizar_graficas(self):
        if self.t:
            tiempos = np.fromiter(self.t, float)
            t_fin = tiempos[-1]
            i0 = int(np.searchsorted(tiempos, t_fin - VENTANA_GRAFICA_S))
            t_rel = tiempos[i0:] - t_fin
            series = {nombre: np.fromiter(valores, float)[i0:]
                      for nombre, valores in self.series.items()}
        else:
            t_rel = np.array([])
            series = {nombre: np.array([]) for nombre in self.series}
        self.graf_theta.actualizar(t_rel, series)
        self.graf_pwm.actualizar(t_rel, series)
        for grafica in self.graficas_mas:
            grafica.actualizar(t_rel, series)

    # ---------------- cierre ----------------
    def _cerrar(self):
        if self.lector:
            self.lector.detener()
        if self.archivo_csv:
            self.archivo_csv.close()
        self.raiz.destroy()


# -----------------------------------------------------------------
#  Logos incrustados (PNG en base64) para que el programa sea un solo archivo
# -----------------------------------------------------------------
LOGO_UNILLANOS = """
iVBORw0KGgoAAAANSUhEUgAAAgAAAACgCAMAAABNJSQPAAADAFBMVEX///////7+/////v///v7/
/v3+/v7//f7//f3//fz+/f7+/f3+/fz//Pz+/P3+/Pz+/Pv/+/z++/z++/v++/r9+/v++vz++vv+
+vr9+vr++vn++fr9+fn9+Pj99/j99/f8+Pj99vf99fb89fb89PX88/T78/P88vP78fL78PH77/D7
7u/67e757Oz66+z56er55+n55uf45eb44+T44uP34eL33+H33uD23N722tz12dv12Nr119n01Nbz
1Nb00tXz0NPyzdDyzM/yys3xyczxx8rwxMfvwsXvv8LuvcDtur7tuLztt7rstLjrsrbrsLTqrrLq
q7Dpqq7pqK3oparnoqfnoKXmnaLlmqDlmJ3klpzklJnjkZbijpTijJLhipDgh43ghIrfgonfgIfe
fYTdeoHdeH/cdXzbc3rbcHjabXXZa3LYaG/YZW3XYmvXYGjWXmbVW2PVWWLUVl/TVFzTUVrST1jS
TlfRTFXRSlPQSFHQRk/PRE7PQUvOP0jNPEfNO0XMOUPMN0HMNkDLND7LMj3KMTvKLzrKLjjJKzbJ
KTTIKDPIJzLHJTDHIy/HIi3GISvGHyvGHirFHSjFGyfEGyXEGSTEGCPEFiLDFSHDFR7DEx/DEh7C
ERzCEBzCDhvBDhjBDRnBDBfBCxjBChfAChfACxXAChbAChS+CxDBCRbACRfACRbBCRXACRXBCBbA
CBbACBXACRTACBTACBO/CBHBBxXABxXABxTABxPABhTABhO/BxS/BhS/BhO/BxLABhK/BhK/Bg/A
BRK/BRO/BRLABRG/BRHABBK/BBK/BBG/BRC/BBDAAxG/AxG/AxC/Aw+/AhC/Ag+/Ag6/AQ6+BA++
Aw+9BAq9Awq+AhC+Ag++Ag69Agm+AQ++AQ6+AQ29AQq7AQS+AA6+AA29AA2+AAy9AAy+AAu9AAu+
AAq9AAq9AAm9AAi9AAe8AAm8AAi8AAe8AAa7AAa8AAW7AAW8AAS8AAO7AAS7AAO8AAK7AAK7AAG7
AAC6AAG6AAC5AADXsY+gAABl4UlEQVR42u29C1wTWbYvvOdYo7mTI5lDzlzpo+hHq+OLg+8HrVxB
BQXFJ4iKCCiIvBQFXwgIigg+G0HRVrHx0U47XMYM08fCOJlBrcsx7WdaRtEObbDNiaST2CQ3jWRA
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
SUAAAAAASUVORK5CYII=
"""

LOGO_FACULTAD = """
iVBORw0KGgoAAAANSUhEUgAAAikAAACqCAMAAABf9i9zAAADAFBMVEX///////7+///+//79///9
//79//37///5///6//33///2//71///x///s/////v/+/v/+/v79/v7+/v3//f79/f79/f39/fv9
/Pz8/v38/f38/P78/Pz7/Pz8/Pr7+/z8+/v7+/v7+/r7+fT5/f33/f75+/v5+/r6+vr5+/b5+fn4
+Pj39/T0/v30+/nw/Pzo/Pz09fXz8+7t9PPx8fDv7u/r7e7s7Ozx7dnp6+vp6ejn5+fl5eXt5Mjv
1aff5+jf4N/b2trS1dbRzs3KysnGyMizydDDxMPRwpXquUvytyDyuBvxthzytRvyuRTztxXythTx
tRXxsQq/wMC8vr27u7q3t7W0s7GstLarrKypqaihpaeynp2goJ+ampmAmKKDjpK1fX1/f351dXVs
bGtbiJ1Dano3ZHg8Y3QdYoIcYoIuYXkbYYUbYYMbYYAqYHsbYIQZYoQYYYQYYYIaYYEYYYEZYX8a
YIYaYIQYYIkYYIgYYIYaYIMaYIIZYIMZYIAXYYYXYIQXYIEVYIHVOjjTIyzVIirSIizYISvWISrX
ISfVISrSISrYICnXICrYICfUICrTICXaHyjZHynWHynZHibTHiPcHSfUHCLXFhnVDhC/PDdiW1pZ
WVlRUVFKSklCQ0Q7OzoyMjIsLCzPIy3QICnMJCzPGiCvFBWHHSKKEBRsHyMnJyckJCQhISAbX4ka
X4kaX4gaX4cYX4cXX4gXX4caX4QZX4UZX4MZX4IZX4EXX4QZX38bXogZXogZXocYXocZXoQYXoIa
Xn8YXoAXXoAaXYYYXYIYXX0cLjcZGRkWX4MWXn8WXH0TXH0OWXkOVXMHVHYLSWYSEhIODg4NDAwL
CwoKCgkKCQgJCQkJCQcJCAcHCggICAgICAYGCAYHBwcHBwUFBwUGBgcGBgYGBgUGBQUFBgYFBQYF
BQUFBQQFBAMEBgUEBAQEBAMCBQQDAwQDAwMDAwIDAgICAwMCAgMCAgICAgEBAgECAQABAQEBAQAB
AAAAAQEAAAEAAAByQdhdAABZtUlEQVR42u29C1xU57X3v0Gt1oB1y3GMOOgZjxfCVSJKRKJcQzQx
JuEWoEwQxuSjTU2bGPVojEdjYi6aahJtYmC4xXjaD5Cq1XhLjBETPgQvlFvgHDjQA+8ptEP/w7/v
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
"""


def main():
    global ESCALA
    activar_alta_resolucion()            # antes de crear la ventana
    raiz = tk.Tk()
    try:
        ESCALA = max(1.0, raiz.winfo_fpixels("1i") / 96.0)
    except Exception:
        ESCALA = 1.0
    Aplicacion(raiz)
    raiz.mainloop()


if __name__ == "__main__":
    main()

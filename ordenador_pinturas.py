# -*- coding: utf-8 -*-
"""
Ordenador de Pinturas v2 - GUI para el firmware C del STM32 Black Pill (USB CDC)
Motores: Base (columna), Hombro, Brazo.

Protocolo (texto, líneas terminadas en '\\n'):
  PC -> STM32 : R,a0,a1,a2 | P,i,kp,ki,kd | F,tau | Z | X | E,1 | E,0
  STM32 -> PC : D,ms,r0,y0,r1,y1,r2,y2 (cada 100 ms) | #mensaje
"""
import csv
import json
import os
import queue
import sys
import threading
import time
from collections import deque

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ------------------------------------------------------------------
# CONFIGURACIÓN (mantener los límites iguales a los del firmware)
# ------------------------------------------------------------------
NAMES = ["Base", "Hombro", "Brazo"]
N = 3
ANG_MIN = [-180.0, -90.0, -120.0]
ANG_MAX = [180.0, 90.0, 120.0]
BAUDS = ["9600", "57600", "115200", "250000"]
WINDOWS = ["10", "30", "60", "120"]
HIST = 1200                      # puntos guardados (10 Hz = 120 s)
STM32_VID, STM32_PID = 0x0483, 0x5740   # "STM32 Virtual ComPort"
TELEM_TIMEOUT = 1.5              # s sin telemetría = alerta


def app_dir():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


CONFIG_PATH = os.path.join(app_dir(), "config_pinturas.json")

DEFAULT_CFG = {
    "port": "",
    "baud": "115200",
    "pid": [[3.0, 0.5, 0.0] for _ in range(N)],
    "tau": 0.3,
    "tol": 1.0,
    "timeout": 20.0,
    "repeat": False,
    "window": "30",
    "poses": [],
}


def load_cfg():
    cfg = json.loads(json.dumps(DEFAULT_CFG))
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg.update(json.load(f))
    except Exception:
        pass
    return cfg


def to_float(text):
    return float(str(text).strip().replace(",", "."))


# ------------------------------------------------------------------
# ENLACE SERIAL
# ------------------------------------------------------------------
class SerialLink:
    def __init__(self, rx_queue):
        self.ser = None
        self.q = rx_queue
        self.running = False
        self.lock = threading.Lock()

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baud):
        self.ser = serial.Serial(port, int(baud), timeout=0.1)
        self.running = True
        threading.Thread(target=self._reader, args=(self.ser,), daemon=True).start()

    def close(self):
        self.running = False
        time.sleep(0.15)
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _reader(self, ser):
        while self.running:
            try:
                line = ser.readline()
                if line:
                    self.q.put(line.decode("ascii", "ignore").strip())
            except Exception as e:
                if self.running:
                    self.q.put("#ERR_SERIAL " + str(e))
                self.running = False
                break

    def send(self, text):
        if not self.connected:
            return False
        with self.lock:
            try:
                self.ser.write((text + "\n").encode("ascii"))
                return True
            except Exception as e:
                self.q.put("#ERR_SERIAL " + str(e))
                return False


# ------------------------------------------------------------------
# APLICACIÓN
# ------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Ordenador de Pinturas v2 - Control 3 GDL (STM32)")
        self.geometry("1420x880")
        self.minsize(1180, 740)

        self.cfg = load_cfg()
        self.rx = queue.Queue()
        self.link = SerialLink(self.rx)

        self.poses = list(self.cfg.get("poses", []))
        self.y = [None] * N          # posiciones medidas (°)
        self.r = [0.0] * N           # objetivos reportados por el firmware (°)
        self.hist_t = deque(maxlen=HIST)
        self.hist_r = [deque(maxlen=HIST) for _ in range(N)]
        self.hist_y = [deque(maxlen=HIST) for _ in range(N)]
        self.motors_enabled = True
        self.last_telem = 0.0
        self.sync_pending = False

        self.csv_file = None
        self.csv_writer = None

        self.seq_running = False
        self.seq_idx = 0
        self.seq_phase = "go"
        self.seq_t0 = 0.0

        self._build_ui()
        self.refresh_ports(auto_select=True)
        self.refresh_tree()

        self.bind("<Escape>", lambda ev: self.stop())
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self.process_queue)
        self.after(200, self.update_plot)
        self.after(300, self.check_health)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def _build_ui(self):
        # ---- Barra superior: conexión ----
        top = ttk.Frame(self, padding=6)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Puerto:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=self.cfg.get("port", ""))
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=42,
                                    postcommand=self.refresh_ports)
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Actualizar", command=lambda: self.refresh_ports(True)).pack(side=tk.LEFT)

        ttk.Label(top, text="  Baudios (USB: se ignora):").pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(self.cfg.get("baud", "115200")))
        ttk.Combobox(top, textvariable=self.baud_var, values=BAUDS, width=8).pack(side=tk.LEFT, padx=4)

        self.btn_conn = ttk.Button(top, text="Conectar", command=self.toggle_connect)
        self.btn_conn.pack(side=tk.LEFT, padx=8)

        self.led = tk.Canvas(top, width=16, height=16, highlightthickness=0)
        self.led_id = self.led.create_oval(2, 2, 14, 14, fill="#999999", outline="")
        self.led.pack(side=tk.LEFT)
        self.status_var = tk.StringVar(value="Desconectado")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=6)

        # ---- Cuerpo ----
        body = ttk.Frame(self)
        body.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(body, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(body, padding=6)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_joints(left)
        self._build_poses(left)
        self._build_log(left)
        self._build_plot(right)

    def _build_joints(self, parent):
        lf = ttk.LabelFrame(parent, text="Articulaciones", padding=6)
        lf.pack(fill=tk.X)

        heads = ["", "Objetivo (°)", "", "Jog (°)", "Actual (°)", "Error (°)", "Kp", "Ki", "Kd", ""]
        for c, h in enumerate(heads):
            ttk.Label(lf, text=h, font=("Segoe UI", 9, "bold")).grid(row=0, column=c, padx=3)

        self.ref_vars, self.scale_vars, self.act_vars, self.err_vars = [], [], [], []
        self.pid_vars = []
        for i in range(N):
            r = i + 1
            ttk.Label(lf, text=NAMES[i], width=8).grid(row=r, column=0, sticky="w")

            rv = tk.StringVar(value="0.0")
            sv = tk.DoubleVar(value=0.0)
            av = tk.StringVar(value="--")
            ev = tk.StringVar(value="--")
            self.ref_vars.append(rv)
            self.scale_vars.append(sv)
            self.act_vars.append(av)
            self.err_vars.append(ev)

            e = ttk.Entry(lf, textvariable=rv, width=8, justify="center")
            e.grid(row=r, column=1, padx=3, pady=4)
            e.bind("<Return>", lambda ev_: self.move_all())

            sc = ttk.Scale(lf, from_=ANG_MIN[i], to=ANG_MAX[i], variable=sv, length=150,
                           command=lambda v, k=i: self.ref_vars[k].set("%.1f" % float(v)))
            sc.grid(row=r, column=2, padx=3)

            jf = ttk.Frame(lf)
            jf.grid(row=r, column=3, padx=3)
            for d in (-5, -1, 1, 5):
                ttk.Button(jf, text=("%+d" % d), width=3,
                           command=lambda k=i, dd=d: self.jog(k, dd)).pack(side=tk.LEFT)

            ttk.Label(lf, textvariable=av, width=8, anchor="center",
                      font=("Consolas", 10, "bold")).grid(row=r, column=4)
            ttk.Label(lf, textvariable=ev, width=7, anchor="center",
                      font=("Consolas", 10)).grid(row=r, column=5)

            pv = []
            for j in range(3):
                v = tk.StringVar(value=str(self.cfg["pid"][i][j]))
                ttk.Entry(lf, textvariable=v, width=6, justify="center").grid(row=r, column=6 + j, padx=2)
                pv.append(v)
            self.pid_vars.append(pv)
            ttk.Button(lf, text="Enviar PID", width=10,
                       command=lambda k=i: self.send_pid(k)).grid(row=r, column=9, padx=3)

        # Botones de acción
        act = ttk.Frame(lf)
        act.grid(row=N + 1, column=0, columnspan=10, pady=(8, 2), sticky="w")
        ttk.Button(act, text="Mover todas", command=self.move_all).pack(side=tk.LEFT, padx=2)
        ttk.Button(act, text="Ir a 0°", command=self.go_zero).pack(side=tk.LEFT, padx=2)
        ttk.Button(act, text="Poner a cero", command=self.zero).pack(side=tk.LEFT, padx=2)
        tk.Button(act, text="PARAR (Esc)", command=self.stop, bg="#cc2222", fg="white",
                  activebackground="#aa1111", activeforeground="white",
                  font=("Segoe UI", 9, "bold"), relief=tk.FLAT, padx=10).pack(side=tk.LEFT, padx=6)
        self.btn_en = ttk.Button(act, text="Deshabilitar motores", command=self.toggle_enable)
        self.btn_en.pack(side=tk.LEFT, padx=2)
        ttk.Button(act, text="Enviar todos los PID", command=self.send_all_pid).pack(side=tk.LEFT, padx=2)

        flt = ttk.Frame(lf)
        flt.grid(row=N + 2, column=0, columnspan=10, pady=2, sticky="w")
        ttk.Label(flt, text="Filtro referencia τ (s):").pack(side=tk.LEFT)
        self.tau_var = tk.StringVar(value=str(self.cfg.get("tau", 0.3)))
        ttk.Entry(flt, textvariable=self.tau_var, width=6, justify="center").pack(side=tk.LEFT, padx=4)
        ttk.Button(flt, text="Enviar", command=self.send_tau).pack(side=tk.LEFT)

    def _build_poses(self, parent):
        lf = ttk.LabelFrame(parent, text="Posiciones guardadas / secuencia", padding=6)
        lf.pack(fill=tk.BOTH, expand=True, pady=6)

        cols = ("nombre", "base", "hombro", "brazo", "espera")
        self.tree = ttk.Treeview(lf, columns=cols, show="headings", height=7, selectmode="browse")
        for c, t, w in zip(cols, ["Nombre", "Base", "Hombro", "Brazo", "Espera (s)"],
                           [170, 80, 80, 80, 90]):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.bind("<Double-1>", lambda ev: self.go_selected())
        self.tree.bind("<<TreeviewSelect>>", self.on_select_pose)

        f1 = ttk.Frame(lf)
        f1.pack(fill=tk.X, pady=4)
        ttk.Label(f1, text="Nombre:").pack(side=tk.LEFT)
        self.name_var = tk.StringVar()
        ttk.Entry(f1, textvariable=self.name_var, width=18).pack(side=tk.LEFT, padx=3)
        ttk.Label(f1, text="Espera (s):").pack(side=tk.LEFT)
        self.wait_var = tk.StringVar(value="1.0")
        ttk.Entry(f1, textvariable=self.wait_var, width=5, justify="center").pack(side=tk.LEFT, padx=3)

        f2 = ttk.Frame(lf)
        f2.pack(fill=tk.X)
        ttk.Button(f2, text="Guardar actual", command=lambda: self.save_pose("actual")).pack(side=tk.LEFT, padx=1)
        ttk.Button(f2, text="Guardar de campos", command=lambda: self.save_pose("campos")).pack(side=tk.LEFT, padx=1)
        ttk.Button(f2, text="Actualizar sel.", command=self.update_pose).pack(side=tk.LEFT, padx=1)
        ttk.Button(f2, text="Ir a", command=self.go_selected).pack(side=tk.LEFT, padx=1)

        f3 = ttk.Frame(lf)
        f3.pack(fill=tk.X, pady=2)
        ttk.Button(f3, text="Subir", width=7, command=lambda: self.move_pose(-1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(f3, text="Bajar", width=7, command=lambda: self.move_pose(1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(f3, text="Eliminar", width=8, command=self.delete_pose).pack(side=tk.LEFT, padx=1)
        ttk.Button(f3, text="Exportar…", command=self.export_poses).pack(side=tk.LEFT, padx=1)
        ttk.Button(f3, text="Importar…", command=self.import_poses).pack(side=tk.LEFT, padx=1)

        f4 = ttk.Frame(lf)
        f4.pack(fill=tk.X, pady=(6, 0))
        self.btn_seq = ttk.Button(f4, text="▶ Ejecutar secuencia", command=self.toggle_sequence)
        self.btn_seq.pack(side=tk.LEFT, padx=1)
        self.repeat_var = tk.BooleanVar(value=bool(self.cfg.get("repeat", False)))
        ttk.Checkbutton(f4, text="Repetir", variable=self.repeat_var).pack(side=tk.LEFT, padx=6)
        ttk.Label(f4, text="Tol. (°):").pack(side=tk.LEFT)
        self.tol_var = tk.StringVar(value=str(self.cfg.get("tol", 1.0)))
        ttk.Entry(f4, textvariable=self.tol_var, width=5, justify="center").pack(side=tk.LEFT, padx=2)
        ttk.Label(f4, text="Timeout (s):").pack(side=tk.LEFT)
        self.to_var = tk.StringVar(value=str(self.cfg.get("timeout", 20.0)))
        ttk.Entry(f4, textvariable=self.to_var, width=5, justify="center").pack(side=tk.LEFT, padx=2)

        self.seq_var = tk.StringVar(value="Secuencia: detenida")
        ttk.Label(lf, textvariable=self.seq_var, foreground="#444").pack(anchor="w", pady=(4, 0))

    def _build_log(self, parent):
        lf = ttk.LabelFrame(parent, text="Registro", padding=4)
        lf.pack(fill=tk.X)
        self.log = tk.Text(lf, height=6, state="disabled", font=("Consolas", 9))
        self.log.pack(fill=tk.X)
        f = ttk.Frame(lf)
        f.pack(fill=tk.X, pady=(3, 0))
        self.btn_rec = ttk.Button(f, text="● Grabar datos (CSV)", command=self.toggle_record)
        self.btn_rec.pack(side=tk.LEFT)
        self.show_tx = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="Mostrar comandos enviados", variable=self.show_tx).pack(side=tk.LEFT, padx=10)

    def _build_plot(self, parent):
        ctl = ttk.Frame(parent)
        ctl.pack(fill=tk.X)
        ttk.Label(ctl, text="Ventana (s):").pack(side=tk.LEFT)
        self.win_var = tk.StringVar(value=str(self.cfg.get("window", "30")))
        ttk.Combobox(ctl, textvariable=self.win_var, values=WINDOWS, width=5,
                     state="readonly").pack(side=tk.LEFT, padx=4)
        self.pause_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Pausar gráfica", variable=self.pause_var).pack(side=tk.LEFT, padx=10)
        ttk.Button(ctl, text="Limpiar", command=self.clear_hist).pack(side=tk.LEFT)

        self.fig = Figure(figsize=(7, 6), dpi=100)
        self.axes, self.lines_r, self.lines_y = [], [], []
        for i in range(N):
            ax = self.fig.add_subplot(N, 1, i + 1)
            lr, = ax.plot([], [], "--", color="tab:red", label="Objetivo")
            ly, = ax.plot([], [], "-", color="tab:blue", label="Actual")
            ax.set_ylabel(NAMES[i] + " (°)")
            ax.set_xlim(-30, 0)
            ax.set_ylim(ANG_MIN[i] - 5, ANG_MAX[i] + 5)
            ax.grid(True, alpha=0.3)
            if i == 0:
                ax.set_title("Ángulos de los eslabones")
                ax.legend(loc="upper left", fontsize=8)
            if i < N - 1:
                ax.tick_params(labelbottom=False)
            self.axes.append(ax)
            self.lines_r.append(lr)
            self.lines_y.append(ly)
        self.axes[-1].set_xlabel("Tiempo (s)")
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------
    # UTILIDADES
    # ------------------------------------------------------------------
    def log_msg(self, text):
        self.log.configure(state="normal")
        self.log.insert(tk.END, time.strftime("%H:%M:%S ") + text + "\n")
        self.log.see(tk.END)
        self.log.configure(state="disabled")

    def tx(self, text):
        ok = self.link.send(text)
        if ok and self.show_tx.get():
            self.log_msg("> " + text)
        return ok

    def save_cfg(self):
        try:
            self.cfg["port"] = self.port_var.get()
            self.cfg["baud"] = self.baud_var.get()
            self.cfg["pid"] = [[to_float(v.get()) for v in self.pid_vars[i]] for i in range(N)]
            self.cfg["tau"] = to_float(self.tau_var.get())
            self.cfg["tol"] = to_float(self.tol_var.get())
            self.cfg["timeout"] = to_float(self.to_var.get())
            self.cfg["repeat"] = bool(self.repeat_var.get())
            self.cfg["window"] = self.win_var.get()
        except ValueError:
            pass
        self.cfg["poses"] = self.poses
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(self.cfg, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.log_msg("No se pudo guardar la configuración: %s" % e)

    def need_conn(self):
        if not self.link.connected:
            messagebox.showwarning("Sin conexión", "Conecta primero el puerto serial.")
            return False
        return True

    def read_refs(self):
        q = []
        for i in range(N):
            try:
                v = to_float(self.ref_vars[i].get())
            except ValueError:
                raise ValueError("Valor inválido en %s" % NAMES[i])
            q.append(min(max(v, ANG_MIN[i]), ANG_MAX[i]))
        return q

    def set_ref_fields(self, q):
        for i in range(N):
            self.ref_vars[i].set("%.1f" % q[i])
            self.scale_vars[i].set(q[i])

    def set_led(self, color):
        self.led.itemconfigure(self.led_id, fill=color)

    # ------------------------------------------------------------------
    # CONEXIÓN
    # ------------------------------------------------------------------
    def list_ports(self):
        res = []
        for p in serial.tools.list_ports.comports():
            stm = (p.vid == STM32_VID and p.pid == STM32_PID)
            res.append(("%s - %s%s" % (p.device, p.description, "  [STM32]" if stm else ""), stm))
        return res

    def refresh_ports(self, auto_select=False):
        ports = self.list_ports()
        self.port_cb["values"] = [p for p, _ in ports]
        if not auto_select:
            return
        saved = self.port_var.get().split(" - ")[0].strip()
        for p, _ in ports:
            if saved and p.startswith(saved + " "):
                self.port_var.set(p)
                return
        for p, stm in ports:
            if stm:
                self.port_var.set(p)
                return
        if ports:
            self.port_var.set(ports[0][0])

    def toggle_connect(self):
        if self.link.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().split(" - ")[0].strip()
        if not port:
            messagebox.showwarning("Puerto", "Selecciona un puerto serial.")
            return
        try:
            self.link.open(port, self.baud_var.get())
        except Exception as e:
            messagebox.showerror("Error al abrir el puerto", str(e))
            return
        self.btn_conn.configure(text="Desconectar")
        self.last_telem = 0.0
        self.sync_pending = True
        self.log_msg("Conectado a %s" % port)
        self.save_cfg()
        self.after(600, self.post_connect)

    def post_connect(self):
        if not self.link.connected:
            return
        self.send_tau()
        self.send_all_pid()

    def disconnect(self):
        if self.seq_running:
            self.seq_stop("Secuencia detenida (desconexión)")
        self.link.close()
        self.btn_conn.configure(text="Conectar")
        self.log_msg("Desconectado")

    def check_health(self):
        if self.link.connected:
            if time.time() - self.last_telem < TELEM_TIMEOUT:
                self.set_led("#22aa22")
                self.status_var.set("Conectado - telemetría OK")
            else:
                self.set_led("#dd2222")
                self.status_var.set("Conectado - SIN telemetría (¿firmware cargado?)")
        else:
            self.set_led("#999999")
            self.status_var.set("Desconectado")
        self.after(300, self.check_health)

    # ------------------------------------------------------------------
    # COMANDOS
    # ------------------------------------------------------------------
    def send_pid(self, i):
        if not self.need_conn():
            return
        try:
            kp, ki, kd = [to_float(v.get()) for v in self.pid_vars[i]]
        except ValueError:
            messagebox.showerror("PID", "Valores PID inválidos en %s" % NAMES[i])
            return
        self.tx("P,%d,%.5f,%.5f,%.5f" % (i, kp, ki, kd))
        self.save_cfg()

    def send_all_pid(self):
        if not self.link.connected:
            return
        for i in range(N):
            self.send_pid(i)

    def send_tau(self):
        if not self.link.connected:
            return
        try:
            tau = to_float(self.tau_var.get())
        except ValueError:
            messagebox.showerror("Filtro", "τ inválido")
            return
        self.tx("F,%.4f" % tau)
        self.save_cfg()

    def send_refs(self, q):
        q = [min(max(q[i], ANG_MIN[i]), ANG_MAX[i]) for i in range(N)]
        self.tx("R,%.2f,%.2f,%.2f" % tuple(q))

    def move_all(self):
        if not self.need_conn():
            return
        try:
            q = self.read_refs()
        except ValueError as e:
            messagebox.showerror("Referencia", str(e))
            return
        self.set_ref_fields(q)
        self.send_refs(q)

    def jog(self, i, delta):
        if not self.need_conn():
            return
        try:
            q = self.read_refs()
        except ValueError as e:
            messagebox.showerror("Referencia", str(e))
            return
        q[i] = min(max(q[i] + delta, ANG_MIN[i]), ANG_MAX[i])
        self.set_ref_fields(q)
        self.send_refs(q)

    def go_zero(self):
        if not self.need_conn():
            return
        q = [0.0] * N
        self.set_ref_fields(q)
        self.send_refs(q)

    def zero(self):
        if not self.need_conn():
            return
        if messagebox.askyesno("Poner a cero",
                               "Esto define la posición actual como 0° en las tres articulaciones.\n¿Continuar?"):
            if self.seq_running:
                self.seq_stop("Secuencia detenida (cero)")
            self.tx("Z")
            self.set_ref_fields([0.0] * N)
            self.clear_hist()

    def stop(self):
        if self.seq_running:
            self.seq_stop("Secuencia detenida (PARAR)")
        if self.link.connected:
            self.tx("X")
            if all(v is not None for v in self.y):
                self.set_ref_fields(self.y)

    def toggle_enable(self):
        if not self.need_conn():
            return
        if self.motors_enabled:
            if self.seq_running:
                self.seq_stop("Secuencia detenida (motores deshabilitados)")
            self.tx("E,0")
        else:
            self.tx("E,1")

    def set_enabled_ui(self, enabled):
        self.motors_enabled = enabled
        self.btn_en.configure(text="Deshabilitar motores" if enabled else "Habilitar motores")

    # ------------------------------------------------------------------
    # RECEPCIÓN
    # ------------------------------------------------------------------
    def process_queue(self):
        try:
            while True:
                self.handle_line(self.rx.get_nowait())
        except queue.Empty:
            pass
        self.after(30, self.process_queue)

    def handle_line(self, line):
        if not line:
            return
        if line.startswith("D,"):
            p = line.split(",")
            if len(p) != 2 + 2 * N:
                return
            try:
                t = float(p[1]) / 1000.0
                vals = [float(x) for x in p[2:]]
            except ValueError:
                return
            self.last_telem = time.time()
            self.hist_t.append(t)
            for i in range(N):
                self.r[i] = vals[2 * i]
                self.y[i] = vals[2 * i + 1]
                self.hist_r[i].append(self.r[i])
                self.hist_y[i].append(self.y[i])
                self.act_vars[i].set("%.1f" % self.y[i])
                self.err_vars[i].set("%+.1f" % (self.r[i] - self.y[i]))
            if self.sync_pending:
                self.set_ref_fields(self.r)
                self.sync_pending = False
            if self.csv_writer:
                self.csv_writer.writerow([t] + vals)
        elif line.startswith("#ERR_SERIAL"):
            self.log_msg(line)
            if self.btn_conn["text"] == "Desconectar":
                self.disconnect()
                messagebox.showerror("Serial", "Se perdió la conexión con el STM32.")
        elif line.startswith("#"):
            msg = line[1:]
            if msg.startswith("HABILITADO"):
                self.set_enabled_ui(True)
            elif msg.startswith("DESHABILITADO"):
                self.set_enabled_ui(False)
            elif msg.startswith("CERO"):
                self.clear_hist()
            self.log_msg(msg)

    # ------------------------------------------------------------------
    # GRÁFICA
    # ------------------------------------------------------------------
    def clear_hist(self):
        for d in [self.hist_t] + self.hist_r + self.hist_y:
            d.clear()

    def update_plot(self):
        if self.hist_t and not self.pause_var.get():
            try:
                win = float(self.win_var.get())
            except ValueError:
                win = 30.0
            t_end = self.hist_t[-1]
            xs = [t - t_end for t in self.hist_t]
            for i in range(N):
                self.lines_r[i].set_data(xs, list(self.hist_r[i]))
                self.lines_y[i].set_data(xs, list(self.hist_y[i]))
                self.axes[i].set_xlim(-win, 0)
            self.canvas.draw_idle()
        self.after(200, self.update_plot)

    # ------------------------------------------------------------------
    # POSICIONES
    # ------------------------------------------------------------------
    def refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        for p in self.poses:
            self.tree.insert("", tk.END, values=(
                p["name"], "%.1f" % p["q"][0], "%.1f" % p["q"][1], "%.1f" % p["q"][2], "%.1f" % p["wait"]))

    def selected_index(self):
        sel = self.tree.selection()
        if not sel:
            return None
        return self.tree.index(sel[0])

    def on_select_pose(self, ev=None):
        if self.seq_running:
            return
        i = self.selected_index()
        if i is None:
            return
        self.name_var.set(self.poses[i]["name"])
        self.wait_var.set("%.1f" % self.poses[i]["wait"])

    def pose_from_inputs(self, source):
        if source == "actual":
            if any(v is None for v in self.y):
                messagebox.showwarning("Posición", "Aún no hay lecturas del STM32.")
                return None
            q = [round(v, 1) for v in self.y]
        else:
            try:
                q = [round(v, 1) for v in self.read_refs()]
            except ValueError as e:
                messagebox.showerror("Referencia", str(e))
                return None
        try:
            wait = max(0.0, to_float(self.wait_var.get()))
        except ValueError:
            messagebox.showerror("Espera", "Tiempo de espera inválido")
            return None
        name = self.name_var.get().strip() or "Pose %d" % (len(self.poses) + 1)
        return {"name": name, "q": q, "wait": wait}

    def save_pose(self, source):
        p = self.pose_from_inputs(source)
        if p is None:
            return
        self.poses.append(p)
        self.refresh_tree()
        self.save_cfg()
        self.name_var.set("")

    def update_pose(self):
        i = self.selected_index()
        if i is None:
            messagebox.showinfo("Posiciones", "Selecciona una posición de la lista.")
            return
        p = self.pose_from_inputs("actual" if all(v is not None for v in self.y) else "campos")
        if p is None:
            return
        self.poses[i] = p
        self.refresh_tree()
        self.save_cfg()

    def go_selected(self):
        i = self.selected_index()
        if i is None:
            messagebox.showinfo("Posiciones", "Selecciona una posición de la lista.")
            return
        if not self.need_conn():
            return
        q = self.poses[i]["q"]
        self.set_ref_fields(q)
        self.send_refs(q)

    def move_pose(self, d):
        i = self.selected_index()
        if i is None:
            return
        j = i + d
        if 0 <= j < len(self.poses):
            self.poses[i], self.poses[j] = self.poses[j], self.poses[i]
            self.refresh_tree()
            self.tree.selection_set(self.tree.get_children()[j])
            self.save_cfg()

    def delete_pose(self):
        i = self.selected_index()
        if i is None:
            return
        del self.poses[i]
        self.refresh_tree()
        self.save_cfg()

    def export_poses(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")],
                                            initialfile="posiciones.json")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.poses, f, indent=2, ensure_ascii=False)

    def import_poses(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            poses = []
            for p in data:
                poses.append({"name": str(p["name"]), "q": [float(x) for x in p["q"][:N]],
                              "wait": float(p.get("wait", 0.0))})
            self.poses = poses
            self.refresh_tree()
            self.save_cfg()
        except Exception as e:
            messagebox.showerror("Importar", "Archivo inválido: %s" % e)

    # ------------------------------------------------------------------
    # SECUENCIA
    # ------------------------------------------------------------------
    def toggle_sequence(self):
        if self.seq_running:
            self.seq_stop("Secuencia detenida")
            return
        if not self.need_conn():
            return
        if not self.poses:
            messagebox.showinfo("Secuencia", "No hay posiciones guardadas.")
            return
        try:
            to_float(self.tol_var.get())
            to_float(self.to_var.get())
        except ValueError:
            messagebox.showerror("Secuencia", "Tolerancia o timeout inválidos.")
            return
        self.save_cfg()
        self.seq_running = True
        self.seq_idx = 0
        self.seq_phase = "go"
        self.btn_seq.configure(text="■ Detener secuencia")
        self.log_msg("Secuencia iniciada")
        self.seq_tick()

    def seq_stop(self, msg):
        self.seq_running = False
        self.btn_seq.configure(text="▶ Ejecutar secuencia")
        self.seq_var.set("Secuencia: detenida")
        self.log_msg(msg)
        if self.link.connected and all(v is not None for v in self.y):
            self.tx("X")

    def seq_tick(self):
        if not self.seq_running:
            return
        now = time.time()
        if now - self.last_telem > 2.0:
            self.seq_stop("Secuencia abortada: sin telemetría del STM32")
            return

        pose = self.poses[self.seq_idx]
        total = len(self.poses)
        tol = to_float(self.tol_var.get())
        timeout = to_float(self.to_var.get())

        if self.seq_phase == "go":
            kids = self.tree.get_children()
            self.tree.selection_set(kids[self.seq_idx])
            self.tree.see(kids[self.seq_idx])
            self.set_ref_fields(pose["q"])
            self.send_refs(pose["q"])
            self.seq_t0 = now
            self.seq_phase = "move"

        elif self.seq_phase == "move":
            self.seq_var.set("Secuencia: paso %d/%d '%s' - moviendo" % (self.seq_idx + 1, total, pose["name"]))
            reached = all(self.y[i] is not None and abs(self.y[i] - pose["q"][i]) <= tol for i in range(N))
            if reached:
                self.seq_phase = "wait"
                self.seq_t0 = now
            elif now - self.seq_t0 > timeout:
                self.seq_stop("Secuencia abortada: no llegó a '%s' en %.0f s" % (pose["name"], timeout))
                return

        elif self.seq_phase == "wait":
            resto = max(0.0, pose["wait"] - (now - self.seq_t0))
            self.seq_var.set("Secuencia: paso %d/%d '%s' - esperando %.1f s" % (
                self.seq_idx + 1, total, pose["name"], resto))
            if now - self.seq_t0 >= pose["wait"]:
                self.seq_idx += 1
                if self.seq_idx >= total:
                    if self.repeat_var.get():
                        self.seq_idx = 0
                    else:
                        self.seq_stop("Secuencia terminada")
                        return
                self.seq_phase = "go"

        self.after(50, self.seq_tick)

    # ------------------------------------------------------------------
    # GRABACIÓN CSV
    # ------------------------------------------------------------------
    def toggle_record(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.btn_rec.configure(text="● Grabar datos (CSV)")
            self.log_msg("Grabación detenida")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=time.strftime("datos_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        hdr = ["t_s"]
        for n in NAMES:
            hdr += ["ref_%s_deg" % n, "pos_%s_deg" % n]
        self.csv_writer.writerow(hdr)
        self.btn_rec.configure(text="■ Detener grabación")
        self.log_msg("Grabando en %s" % path)

    # ------------------------------------------------------------------
    def on_close(self):
        self.save_cfg()
        if self.csv_file:
            self.csv_file.close()
        if self.link.connected:
            self.link.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()

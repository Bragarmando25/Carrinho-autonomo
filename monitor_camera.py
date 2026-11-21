#!/usr/bin/env python3
"""Monitor USB da OV2640. Python 3.9+ com Tkinter; pySerial acompanha o pacote.

Execute este arquivo ou AbrirCamera.cmd. Não precisa de OpenCV nem navegador.
Protocolo: comando g, PGM binário 160x120, marcadores FRAME_PGM_BEGIN/END.
Somente bancada, COMPETICAO=false; não envia comandos de movimento.
"""
from pathlib import Path
import sys
import time

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE / "_vendor"))
WIDTH, HEIGHT = 160, 120
PIXEL_COUNT = WIDTH * HEIGHT
PGM_HEADER = b"P5\n160 120\n255\n"
FOOTER = b"\nFRAME_PGM_END\n"


class FrameParser:
    """Não interpreta bytes de pixels como texto e só entrega quadros completos."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.buffer = bytearray()
        self.stage = "line"
        self.header_index = 0
        self.pixels = None
        self.last_byte = 0.0

    def expire(self, now):
        if self.stage != "line" and now - self.last_byte > 4:
            self.reset()
            return [("error", "Imagem incompleta. Capture novamente.")]
        return []

    def feed(self, data, now=None):
        if not data:
            return []
        self.last_byte = time.monotonic() if now is None else now
        self.buffer.extend(data)
        events = []
        while self.buffer:
            if self.stage == "pixels":
                if len(self.buffer) < PIXEL_COUNT:
                    break
                self.pixels = bytes(self.buffer[:PIXEL_COUNT])
                del self.buffer[:PIXEL_COUNT]
                self.stage = "footer"
            elif self.stage == "footer":
                if len(self.buffer) < len(FOOTER):
                    break
                if self.buffer[:len(FOOTER)] == FOOTER:
                    del self.buffer[:len(FOOTER)]
                    events.append(("frame", self.pixels))
                else:
                    events.append(("error", "Fim de imagem inválido; quadro descartado."))
                self.pixels = None
                self.stage = "line"
            else:
                newline = self.buffer.find(b"\n")
                if newline < 0:
                    if len(self.buffer) > 65536:
                        self.reset()
                        events.append(("error", "Dados sem formato reconhecido; verifique o firmware."))
                    break
                line = bytes(self.buffer[:newline]).rstrip(b"\r")
                del self.buffer[:newline + 1]
                if line == b"FRAME_PGM_BEGIN":
                    self.stage, self.header_index = "header", 0
                elif self.stage == "header":
                    expected = (b"P5", b"160 120", b"255")
                    if line != expected[self.header_index]:
                        self.stage = "line"
                        events.append(("error", "Formato inesperado. Use o firmware deste projeto."))
                    else:
                        self.header_index += 1
                        if self.header_index == 3:
                            self.stage = "pixels"
                elif line:
                    events.append(("log", line.decode("utf-8", errors="replace")[:500]))
        return events


class CameraMonitor:
    def __init__(self, root, serial_module=None, list_ports=None):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk = tk, ttk
        self.root, self.serial_module, self.list_ports = root, serial_module, list_ports
        self.port = None
        self.parser = FrameParser()
        self.pending_since = None
        self.next_capture = 0.0
        self.raw = None
        self.photo = None
        self.closed = False
        self.status = tk.StringVar(value="Conecte a base MB por USB e selecione a porta COM.")
        self.port_name = tk.StringVar()
        self.auto = tk.BooleanVar(value=True)
        self.binary = tk.BooleanVar(value=False)
        self.bands = tk.BooleanVar(value=True)
        self.threshold = tk.IntVar(value=110)
        root.title("Câmera do carrinho — monitor Python USB")
        root.geometry("960x740")
        root.minsize(800, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)
        outer = ttk.Frame(root, padding=14)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Imagem da ESP32-CAM", font=("Segoe UI", 18, "bold")).pack(anchor="w")
        ttk.Label(outer, text="Bancada: COMPETICAO=false. Feche o Monitor Serial do Arduino antes de conectar.").pack(anchor="w", pady=(3, 12))
        ports = ttk.Frame(outer)
        ports.pack(fill="x")
        ttk.Label(ports, text="Porta:").pack(side="left")
        self.combo = ttk.Combobox(ports, textvariable=self.port_name, width=32)
        self.combo.pack(side="left", padx=6)
        self.refresh_button = ttk.Button(ports, text="Atualizar portas", command=self.refresh_ports)
        self.refresh_button.pack(side="left", padx=4)
        self.connect_button = ttk.Button(ports, text="Conectar", command=self.connect)
        self.connect_button.pack(side="left", padx=4)
        self.disconnect_button = ttk.Button(ports, text="Desconectar", command=self.disconnect, state="disabled")
        self.disconnect_button.pack(side="left", padx=4)
        actions = ttk.Frame(outer)
        actions.pack(fill="x", pady=10)
        self.capture_button = ttk.Button(actions, text="Capturar imagem", command=self.capture, state="disabled")
        self.capture_button.pack(side="left")
        self.stop_button = ttk.Button(actions, text="Parar motores", command=self.stop, state="disabled")
        self.stop_button.pack(side="left", padx=6)
        self.save_button = ttk.Button(actions, text="Salvar imagem PNG", command=self.save, state="disabled")
        self.save_button.pack(side="left", padx=6)
        ttk.Checkbutton(actions, text="Atualizar automaticamente (parado)", variable=self.auto).pack(side="left", padx=6)
        body = ttk.Frame(outer)
        body.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(body, width=640, height=480, bg="#17232f", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda event: self.draw())
        self.canvas.create_text(300, 230, text="Aguardando imagem da câmera", fill="white", font=("Segoe UI", 14), tags="placeholder")
        logs = ttk.Frame(body, padding=(10, 0, 0, 0))
        logs.pack(side="right", fill="y")
        ttk.Label(logs, text="Mensagens da placa").pack(anchor="w")
        self.log_widget = tk.Text(logs, width=28, height=20, state="disabled", wrap="word", font=("Consolas", 9))
        self.log_widget.pack(fill="both", expand=True)
        controls = ttk.Frame(outer)
        controls.pack(fill="x", pady=8)
        ttk.Checkbutton(controls, text="Faixas de leitura", variable=self.bands, command=self.draw).pack(side="left")
        ttk.Checkbutton(controls, text="Prévia preto/branco", variable=self.binary, command=self.draw).pack(side="left", padx=10)
        ttk.Label(controls, text="Limiar:").pack(side="left")
        tk.Scale(controls, from_=0, to=255, orient="horizontal", variable=self.threshold, length=190,
                 command=lambda value: self.draw()).pack(side="left")
        ttk.Label(outer, textvariable=self.status, wraplength=900).pack(fill="x", anchor="w")
        ttk.Label(outer, text="O limiar acima muda apenas a prévia. As imagens por USB chegam aproximadamente a cada 2 segundos.",
                  wraplength=900).pack(fill="x", anchor="w", pady=(5, 0))
        self.refresh_ports()
        self.timer = root.after(30, self.poll)

    def log(self, text):
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", text + "\n")
        if int(self.log_widget.index("end-1c").split(".")[0]) > 180:
            self.log_widget.delete("1.0", "60.0")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def refresh_ports(self):
        if self.list_ports is None:
            return
        try:
            values = [f"{p.device} — {p.description}" for p in self.list_ports.comports()]
            self.combo["values"] = values
            if not self.port_name.get() and values:
                self.port_name.set(values[0])
            if not values:
                self.status.set("Nenhuma porta encontrada. Confira o cabo USB de dados e clique Atualizar portas.")
        except Exception as exc:
            self.status.set(f"Não foi possível listar as portas: {exc}")

    def connected_widgets(self, connected):
        for button in (self.capture_button, self.stop_button, self.disconnect_button):
            button.configure(state="normal" if connected else "disabled")
        self.connect_button.configure(state="disabled" if connected else "normal")
        self.refresh_button.configure(state="disabled" if connected else "normal")
        self.combo.configure(state="disabled" if connected else "normal")

    def connect(self):
        if self.port is not None:
            return
        selected = self.port_name.get().split(" — ")[0].strip()
        if not selected:
            self.status.set("Selecione a porta COM da base MB.")
            return
        candidate = None
        try:
            # Aplica DTR/RTS antes de abrir para reduzir resets de algumas bases.
            candidate = self.serial_module.Serial(port=None, baudrate=115200, timeout=0,
                                                   write_timeout=0.5, rtscts=False, dsrdtr=False)
            candidate.dtr = False
            candidate.rts = False
            candidate.port = selected
            candidate.open()
            self.port = candidate
            self.parser.reset()
            self.pending_since = None
            self.next_capture = time.monotonic() + 3
            self.connected_widgets(True)
            self.auto.set(True)
            self.send(b"x\n")
            self.status.set("Conectado. Aguardando a placa inicializar; captura automática em alguns segundos.")
        except Exception as exc:
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
            self.port = None
            self.connected_widgets(False)
            self.status.set(f"Falha ao abrir {selected}. Feche Arduino/monitor HTML e confira o cabo. Detalhe: {exc}")

    def send(self, data):
        if self.port is None:
            return
        if self.port.write(data) != len(data):
            raise OSError("O comando USB não foi transmitido por completo.")

    def capture(self):
        if self.port is None or self.pending_since is not None:
            return
        try:
            # x não move o robô: cancela testes/partida e pede parada em bancada.
            self.send(b"x\ng\n")
            self.pending_since = time.monotonic()
            self.capture_button.configure(state="disabled")
            self.status.set("Recebendo imagem da OV2640…")
        except Exception as exc:
            self.connection_error(exc)

    def stop(self):
        self.auto.set(False)
        try:
            self.send(b"x\n")
            self.status.set("Parada solicitada em bancada. No modo prova, os comandos USB são ignorados.")
        except Exception as exc:
            self.connection_error(exc)

    def handle_events(self, events):
        for kind, value in events:
            if kind == "frame":
                self.raw = value
                self.pending_since = None
                self.next_capture = time.monotonic() + 0.3
                self.capture_button.configure(state="normal" if self.port is not None else "disabled")
                self.save_button.configure(state="normal")
                self.draw()
                self.status.set(f"Imagem recebida às {time.strftime('%H:%M:%S')} — 160 × 120, escala de cinza.")
            elif kind == "error":
                self.pending_since = None
                self.auto.set(False)
                self.capture_button.configure(state="normal" if self.port is not None else "disabled")
                self.status.set(value)
                self.log(value)
            else:
                self.log(value)
                if "modo=PROVA" in value or "PROVA:" in value:
                    self.auto.set(False)
                    self.status.set("Placa em modo PROVA. Grave COMPETICAO=false para visualizar a câmera.")

    def poll(self):
        if self.closed:
            return
        try:
            if self.port is not None:
                available = min(self.port.in_waiting, 16384)
                if available:
                    self.handle_events(self.parser.feed(self.port.read(available)))
                now = time.monotonic()
                self.handle_events(self.parser.expire(now))
                if self.pending_since is not None and now - self.pending_since > 10:
                    self.pending_since = None
                    self.parser.reset()
                    self.auto.set(False)
                    self.capture_button.configure(state="normal")
                    self.status.set("Sem imagem após 10 s. Confira COMPETICAO=false e as mensagens da placa; depois capture novamente.")
                if self.auto.get() and self.pending_since is None and now >= self.next_capture:
                    self.capture()
        except Exception as exc:
            self.connection_error(exc)
        self.timer = self.root.after(30, self.poll)

    def connection_error(self, exc):
        self.disconnect(send_stop=False)
        self.status.set(f"Conexão interrompida: {exc}. Confira o cabo e conecte novamente.")

    def disconnect(self, send_stop=True):
        if self.port is not None:
            try:
                if send_stop:
                    self.send(b"x\n")
            except Exception:
                pass
            finally:
                try:
                    self.port.close()
                except Exception:
                    pass
        self.port = None
        self.pending_since = None
        self.parser.reset()
        self.auto.set(False)
        self.connected_widgets(False)
        self.status.set("Desconectado. A última imagem permanece disponível para salvar.")

    def draw(self):
        if self.raw is None:
            return
        pixels = self.raw
        if self.binary.get():
            threshold = self.threshold.get()
            pixels = pixels.translate(bytes(0 if i < threshold else 255 for i in range(256)))
        self.source_photo = self.tk.PhotoImage(data=PGM_HEADER + pixels, format="PPM", master=self.root)
        zoom = max(1, min(5, self.canvas.winfo_width() // WIDTH, self.canvas.winfo_height() // HEIGHT))
        self.photo = self.source_photo.zoom(zoom, zoom)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
        if self.bands.get():
            for y in (112, 104, 96, 88, 80, 72, 64, 56, 48, 40, 32, 24):
                self.canvas.create_line(0, y * zoom, WIDTH * zoom, y * zoom, fill="#25bca6")
            self.canvas.create_line(79.5 * zoom, 0, 79.5 * zoom, HEIGHT * zoom, fill="#ffb84d")

    def save(self):
        if self.raw is None:
            return
        from tkinter import filedialog, messagebox
        filename = filedialog.asksaveasfilename(parent=self.root, title="Salvar imagem original da câmera",
                                               initialfile="camera_carrinho.png", defaultextension=".png",
                                               filetypes=[("Imagem PNG", "*.png"), ("Imagem PGM", "*.pgm")])
        if not filename:
            return
        try:
            if Path(filename).suffix.lower() == ".pgm":
                Path(filename).write_bytes(PGM_HEADER + self.raw)
            else:
                photo = self.tk.PhotoImage(data=PGM_HEADER + self.raw, format="PPM", master=self.root)
                photo.write(filename, format="png")
            self.status.set("Imagem original salva: " + filename)
        except Exception as exc:
            messagebox.showerror("Não foi possível salvar", str(exc), parent=self.root)

    def close(self):
        self.closed = True
        self.root.after_cancel(self.timer)
        self.disconnect()
        self.root.destroy()


def main():
    import tkinter as tk
    from tkinter import messagebox
    root = tk.Tk()
    try:
        import serial
        from serial.tools import list_ports
        if not hasattr(serial, "Serial"):
            raise ImportError("O pacote serial encontrado não é pySerial.")
    except ImportError:
        root.withdraw()
        messagebox.showerror("Biblioteca USB ausente",
                             "Extraia o ZIP completo, incluindo _vendor.\n\n"
                             "Se estiver usando apenas o arquivo Python, instale:\n"
                             'python -m pip install pyserial==3.5', parent=root)
        root.destroy()
        return 1
    CameraMonitor(root, serial, list_ports)
    root.mainloop()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ImportError as exc:
        print("Este monitor precisa de Python 3.9+ com Tkinter. Use uma instalação completa do Python.")
        print(exc)
        raise SystemExit(1)

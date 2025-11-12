# index.py — Oscilloscope 3‑CH (manual trigger) + Đồng hồ công suất (độc lập)
# BẢN FULL KHÔNG DÙNG DEMO CHO SCOPE. CHỈ NHẬN DỮ LIỆU TỪ STM32 QUA UART.
# - Scope: đọc block CSV từ UART (/dev/serial0 @ 921600) theo khuôn:
#     sr=10000\n
#     a,b,c\n  (dòng tiêu đề tùy chọn)
#     <1024 dòng> mỗi dòng: v1,v2,v3  (float Volt hoặc raw 12‑bit)
# - Nếu phát hiện "raw" (>10), tự đổi về Volt quanh 0 V: (raw-2048)*(Vref/4096), rồi nhân hệ số SCALE_CHx.
# - Power Meter vẫn demo (độc lập). Có thể thay provider thật sau.

import sys, importlib.util, threading, time, os, re
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

# ============================ Scope Provider (UART) ============================

class ScopeProviderBase(QtCore.QObject):
    # emit block: {'sr':float, 't':np.ndarray(float32), 'ch1':nd, 'ch2':nd, 'ch3':nd}
    block = QtCore.pyqtSignal(dict)
    def start(self): ...
    def stop(self): ...

# Cấu hình UART/ADC/SCALE qua env
PORT  = os.environ.get("SCOPE_UART_PORT", "/dev/serial0")
BAUD  = int(os.environ.get("SCOPE_UART_BAUD", "921600"))
TIMEOUT = 0.2
BLOCK_ROWS = int(os.environ.get("SCOPE_BLOCK_ROWS", "1024"))
VREF = float(os.environ.get("SCOPE_ADC_VREF", "3.3"))
SCALE_CH = [
    float(os.environ.get("SCOPE_SCALE_CH1", "1.0")),
    float(os.environ.get("SCOPE_SCALE_CH2", "1.0")),
    float(os.environ.get("SCOPE_SCALE_CH3", "1.0")),
]
MAX_BLOCK_TIME = float(os.environ.get("SCOPE_MAX_BLOCK_TIME", "1.5"))

# Regex parser cho dòng dữ liệu và sr=
_num = re.compile(rb'^\s*([+-]?\d+(?:\.\d*)?)\s*,\s*([+-]?\d+(?:\.\d*)?)\s*,\s*([+-]?\d+(?:\.\d*)?)\s*$')
_sr   = re.compile(rb'^\s*sr\s*=\s*([0-9]+(?:\.[0-9]*)?)', re.I)
_float = np.float32

class UARTScopeProvider(ScopeProviderBase):
    def __init__(self, port=PORT, baud=BAUD):
        super().__init__()
        self.port = port; self.baud = baud
        self._run = False; self._fs = 10_000.0; self._idx = 0
        self._ser = None

    def start(self):
        # Kiểm tra pyserial ngay từ đầu, nếu lỗi thì báo và thoát
        try:
            import serial  # noqa: F401
        except Exception:
            QtWidgets.QMessageBox.critical(None, "Thiếu pyserial",
                "Hãy cài:  python3 -m pip install pyserial")
            sys.exit(2)
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._run = False
        try:
            if self._ser: self._ser.close()
        except Exception:
            pass

    def _open(self):
        import serial
        if self._ser and self._ser.is_open: return
        self._ser = serial.Serial(self.port, self.baud, timeout=TIMEOUT)

    def _readline(self):
        return self._ser.readline()

    def _loop(self):
        buf_a = np.zeros(BLOCK_ROWS, dtype=_float)
        buf_b = np.zeros(BLOCK_ROWS, dtype=_float)
        buf_c = np.zeros(BLOCK_ROWS, dtype=_float)
        ADC_FULLSCALE = float(1 << 12)
        ADC_MID = ADC_FULLSCALE/2.0
        while self._run:
            try:
                self._open()

                # 1) Tìm dòng sr=
                t0 = time.time()
                while self._run:
                    line = self._readline()
                    if not line:
                        if time.time()-t0 > MAX_BLOCK_TIME: t0 = time.time()
                        continue
                    m = _sr.match(line)
                    if m:
                        try: self._fs = float(m.group(1).decode('ascii'))
                        except Exception: self._fs = 10_000.0
                        break

                # 2) Bỏ qua header "a,b,c" nếu có; nếu là số, dùng luôn
                line = self._readline()
                if not line: continue
                rows = 0
                if _num.match(line):
                    a,b,c = [float(x) for x in line.decode('ascii').split(',')]
                    buf_a[rows]=a; buf_b[rows]=b; buf_c[rows]=c; rows+=1

                # 3) Đọc đủ BLOCK_ROWS
                t_block = time.time()
                while rows < BLOCK_ROWS and self._run:
                    line = self._readline()
                    if not line:
                        if time.time()-t_block > MAX_BLOCK_TIME: break
                        continue
                    m = _num.match(line)
                    if not m: break
                    a = float(m.group(1)); b = float(m.group(2)); c = float(m.group(3))
                    buf_a[rows]=a; buf_b[rows]=b; buf_c[rows]=c; rows+=1

                if rows < BLOCK_ROWS:  # block lỗi -> chờ block mới
                    continue

                # 4) raw->Volt nếu cần, rồi SCALE
                a = buf_a.copy(); b = buf_b.copy(); c = buf_c.copy()
                if np.max(np.abs([a.max(), b.max(), c.max()])) > 10.0:
                    k = (VREF/ADC_FULLSCALE)
                    a = (a-ADC_MID)*k; b = (b-ADC_MID)*k; c = (c-ADC_MID)*k
                a *= SCALE_CH[0]; b *= SCALE_CH[1]; c *= SCALE_CH[2]

                # 5) t tuyệt đối
                n = a.size
                t = (np.arange(self._idx, self._idx+n, dtype=np.float64)/self._fs).astype(_float)
                self._idx += n

                self.block.emit({'sr': float(self._fs), 't': t,
                                 'ch1': a.astype(_float, copy=False),
                                 'ch2': b.astype(_float, copy=False),
                                 'ch3': c.astype(_float, copy=False)})
            except Exception:
                try:
                    if self._ser: self._ser.close()
                except Exception: pass
                time.sleep(0.5)

# =========================== Power (demo, độc lập) ===========================

class PowerProviderBase(QtCore.QObject):
    block = QtCore.pyqtSignal(dict)
    def start(self): ...
    def stop(self): ...

class DemoPowerProvider(PowerProviderBase):
    """Lưới 50 Hz demo: Vrms~220V, Irms~0.8A, PF~0.85 (độc lập với scope)."""
    def __init__(self, fs=4000.0, freq=50.0, vrms=220.0, irms=0.8, pf=0.85, block=512):
        super().__init__()
        self.fs=float(fs); self.freq=float(freq); self.vrms=float(vrms); self.irms=float(irms); self.pf=float(pf)
        self.block_n=int(block); self._run=False; self._i=0
    def start(self):
        self._run=True
        threading.Thread(target=self._loop, daemon=True).start()
    def stop(self): self._run=False
    def _loop(self):
        w = 2*np.pi*self.freq
        phi = float(np.arccos(np.clip(self.pf, -1.0, 1.0)))
        Av = self.vrms*np.sqrt(2.0); Ai = self.irms*np.sqrt(2.0)
        while self._run:
            n = self.block_n
            idx = np.arange(self._i, self._i+n, dtype=np.float64); self._i += n
            t = idx / self.fs
            v = (Av*np.sin(w*t)).astype(np.float32)
            i = (Ai*np.sin(w*t - phi)).astype(np.float32)
            self.block.emit({'fs': self.fs, 'v': v, 'i': i})
            time.sleep(max(0.0, n/self.fs*0.7))

# ================================ Tiện ích ===================================

def stats_vpp_freq(x: np.ndarray, fs_hint: float):
    if x.size < 4: return 0.0, 0.0, 0.0
    vpp = float(x.max() - x.min())
    xm = x - x.mean()
    s = np.signbit(xm)
    cross = np.where((s[:-1] == True) & (s[1:] == False))[0]
    freq = 0.0
    if cross.size >= 2:
        T = np.median(np.diff(cross)) / fs_hint
        if T > 0: freq = 1.0/T
    duty = 0.0
    if cross.size >= 2:
        a, b = cross[0], cross[1]
        seg = xm[a:b]
        if seg.size>0: duty = 100.0*np.count_nonzero(seg>0)/seg.size
    return vpp, freq, duty

def rms(x):
    if x.size==0: return 0.0
    return float(np.sqrt(np.mean(x*x)))

# ============================= Power Meter Panel =============================

class PowerMeterPanel(QtWidgets.QFrame):
    def __init__(self, provider: PowerProviderBase, parent=None):
        super().__init__(parent)
        self.provider = provider
        self.setStyleSheet("background:#1A1A1A; border-radius:10px;")
        self.setMinimumHeight(130)
        self.fs = 4000.0; self.win_s = 1.0; self.N = int(self.fs*4)
        self.v = np.zeros(self.N, dtype=np.float32)
        self.i = np.zeros(self.N, dtype=np.float32)
        self.iw = 0
        grid = QtWidgets.QGridLayout(self); grid.setContentsMargins(12,10,12,10); grid.setSpacing(6)
        font_big = QtGui.QFont("Consolas", 16, QtGui.QFont.Bold)
        font_lbl = QtGui.QFont("Consolas", 14, QtGui.QFont.Bold)
        def title(txt):
            L=QtWidgets.QLabel(txt); L.setFont(font_lbl); L.setStyleSheet("color:#9ad;"); return L
        def val(txt):
            L=QtWidgets.QLabel(txt); L.setFont(font_big); L.setStyleSheet("color:#fff;"); return L
        grid.addWidget(title("ĐỒNG HỒ CÔNG SUẤT (độc lập)"), 0, 0, 1, 2)
        self.lab_U = val("U: ---.- V (rms)")
        self.lab_I = val("I: ---.- A (rms)")
        self.lab_P = val("P: ---.- W")
        self.lab_S = val("S: ---.- VA    PF: ---.--")
        grid.addWidget(self.lab_U, 1,0); grid.addWidget(self.lab_I,1,1)
        grid.addWidget(self.lab_P, 2,0); grid.addWidget(self.lab_S,2,1)
        self.provider.block.connect(self.on_block)
        self.provider.start()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start(200)
    def on_block(self, blk: dict):
        fs = blk.get('fs', None)
        if fs: self.fs = float(fs)
        v = blk['v']; i = blk['i']; n = v.size
        i0 = self.iw; i1 = (i0 + n) % self.N
        if i0 < i1:
            self.v[i0:i1] = v[:i1-i0]; self.i[i0:i1] = i[:i1-i0]
        else:
            k = self.N - i0
            self.v[i0:] = v[:k]; self.i[i0:] = i[:k]
            self.v[:i1] = v[k:]; self.i[:i1] = i[k:]
        self.iw = i1
    def refresh_ui(self):
        n = min(int(self.fs*self.win_s), self.N-1)
        i1 = self.iw; i0 = (i1 - n) % self.N
        if i0 < i1:
            v = self.v[i0:i1]; i = self.i[i0:i1]
        else:
            v = np.concatenate((self.v[i0:], self.v[:i1])); i = np.concatenate((self.i[i0:], self.i[:i1]))
        if v.size < 8: return
        Vrms = rms(v); Irms = rms(i)
        Pavg = float(np.mean(v*i))
        S = Vrms*Irms
        PF = (Pavg/S) if S>1e-12 else 0.0
        self.lab_U.setText(f"U: {Vrms:6.1f} V (rms)")
        self.lab_I.setText(f"I: {Irms:6.3f} A (rms)")
        self.lab_P.setText(f"P: {Pavg:7.2f} W")
        self.lab_S.setText(f"S: {S:7.2f} VA    PF: {PF:5.3f}")

# ================================ Scope UI ===================================

class ScopePanelUI(QtWidgets.QWidget):
    def __init__(self, scope_provider: ScopeProviderBase, power_provider: PowerProviderBase):
        super().__init__()
        self.scope_provider = scope_provider
        self.power_provider = power_provider
        self.setWindowTitle("Oscilloscope 3-CH + Đồng hồ công suất")
        self.resize(1280, 750)
        self.setStyleSheet("background:#1A1A1A;")
        self.N = 400000
        self.t  = np.zeros(self.N, dtype=np.float32)
        self.ch = [np.zeros(self.N, dtype=np.float32),
                   np.zeros(self.N, dtype=np.float32),
                   np.zeros(self.N, dtype=np.float32)]
        self.iw = 0; self.sr = 50_000.0
        self.time_div  = 0.001
        self.volts_div = [0.5,0.5,0.5]
        self.offset    = [0.0,0.0,0.0]
        self.trigger_src = 0
        self.trigger_lvl = 0.0
        self._build_ui()
        self.scope_provider.block.connect(self.on_block)
        self.scope_provider.start()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)

    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10,10,10,10); root.setSpacing(10)
        top_widget = QtWidgets.QWidget(); top_widget.setStyleSheet("background:#2D2D2D; color:#DDD;")
        top = QtWidgets.QHBoxLayout(top_widget); top.setContentsMargins(15,5,15,5); top.setSpacing(10)
        self.lbl_sr = QtWidgets.QLabel("SR: -- kS/s"); self.lbl_pts = QtWidgets.QLabel(f"PTS: {self.N}")
        self.lbl_sr.setFont(QtGui.QFont("Consolas", 14, QtGui.QFont.Bold))
        self.lbl_pts.setFont(QtGui.QFont("Consolas", 14, QtGui.QFont.Bold))
        self.lbl_sr.setMinimumWidth(100); self.lbl_pts.setMinimumWidth(100)
        top.addStretch(1); top.addWidget(self.lbl_sr); top.addSpacing(12); top.addWidget(self.lbl_pts)
        root.addWidget(top_widget)
        center_widget = QtWidgets.QWidget(); center = QtWidgets.QHBoxLayout(center_widget)
        center.setContentsMargins(10,10,10,10); center.setSpacing(10)
        pg.setConfigOptions(antialias=False)
        self.plot = pg.PlotWidget(); self.plot.setBackground((26,26,26))
        grid = pg.GridItem(); grid.setPen(pg.mkPen(color=(100,100,100), style=QtCore.Qt.DashLine)); grid.setOpacity(0.3)
        self.plot.addItem(grid); self.plot.setMenuEnabled(False); self.plot.setMouseEnabled(x=False, y=False)
        self.cur1 = self.plot.plot(pen=pg.mkPen((255,215,0), width=3), clipToView=True)
        self.cur2 = self.plot.plot(pen=pg.mkPen((0,255,255), width=3), clipToView=True)
        self.cur3 = self.plot.plot(pen=pg.mkPen((255,20,147), width=3), clipToView=True)
        self.txt1 = pg.TextItem(color=(255,215,0)); self.txt2 = pg.TextItem(color=(0,255,255)); self.txt3 = pg.TextItem(color=(255,20,147))
        fnt = QtGui.QFont("Consolas", 12, QtGui.QFont.Bold)
        self.txt1.setFont(fnt); self.txt2.setFont(fnt); self.txt3.setFont(fnt)
        self.txt1.setAnchor((1,0)); self.txt2.setAnchor((1,0)); self.txt3.setAnchor((1,0))
        self.plot.addItem(self.txt1); self.plot.addItem(self.txt2); self.plot.addItem(self.txt3)
        center.addWidget(self.plot, 1)
        right = QtWidgets.QVBoxLayout(); right.setContentsMargins(10,10,10,10); right.setSpacing(10)
        right.addWidget(self._group_horizontal()); right.addWidget(self._group_all_vertical())
        right.addWidget(self._per_channel(0,"Vertical CH1"))
        right.addWidget(self._per_channel(1,"Vertical CH2"))
        right.addWidget(self._per_channel(2,"Vertical CH3"))
        right.addWidget(self._group_trigger())
        self.power_panel = PowerMeterPanel(DemoPowerProvider())
        right.addWidget(self.power_panel); right.addStretch(1)
        center.addLayout(right, 0)
        root.addWidget(center_widget)
        badges_widget = QtWidgets.QWidget(); badges_widget.setStyleSheet("background:#2D2D2D;")
        badges = QtWidgets.QHBoxLayout(badges_widget); badges.setContentsMargins(10,5,10,5); badges.setSpacing(10)
        self.bad1 = self._badge('CH1 0.5V/div +0.00V','rgb(255,215,0)')
        self.bad2 = self._badge('CH2 0.5V/div +0.00V','rgb(0,255,255)')
        self.bad3 = self._badge('CH3 0.5V/div +0.00V','rgb(255,20,147)')
        self.bad1.setFont(QtGui.QFont("Consolas", 16, QtGui.QFont.Bold))
        self.bad2.setFont(QtGui.QFont("Consolas", 16, QtGui.QFont.Bold))
        self.bad3.setFont(QtGui.QFont("Consolas", 16, QtGui.QFont.Bold))
        self.bad1.setMinimumWidth(150); self.bad2.setMinimumWidth(150); self.bad3.setMinimumWidth(150)
        self.badT = self._badge('TRIG CH1 +0.00V','#ddd'); self.badT.setFont(QtGui.QFont("Consolas", 16, QtGui.QFont.Bold)); self.badT.setMinimumWidth(150)
        for b in (self.bad1,self.bad2,self.bad3,self.badT): badges.addWidget(b)
        badges.addStretch(1); root.addWidget(badges_widget)
        self._update_axes()

    def _badge(self, text, color):
        L = QtWidgets.QLabel(text)
        L.setStyleSheet(f"padding:6px 12px;border-radius:12px;background:#2b2b2b;color:{color};")
        return L

    def _group_horizontal(self):
        box = QtWidgets.QGroupBox("Horizontal"); box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444;")
        g = QtWidgets.QGridLayout(box); g.setContentsMargins(10,10,10,10); g.setSpacing(10)
        g.addWidget(QtWidgets.QLabel("TIME/DIV"), 0, 0)
        m = QtWidgets.QPushButton("-"); p = QtWidgets.QPushButton("+")
        for b in (m,p): b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 5px;")
        m.clicked.connect(lambda: self._time_div_scale(1/1.2))
        p.clicked.connect(lambda: self._time_div_scale(1.2))
        g.addWidget(m,0,1); g.addWidget(p,0,2)
        return box

    def _group_all_vertical(self):
        box = QtWidgets.QGroupBox("Vertical All Channels"); box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444;")
        g = QtWidgets.QGridLayout(box); g.setContentsMargins(10,10,10,10); g.setSpacing(10)
        g.addWidget(QtWidgets.QLabel("POSITION (All)"), 0, 0)
        d = QtWidgets.QPushButton("↓"); u = QtWidgets.QPushButton("↑")
        for b in (d,u): b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 5px;")
        d.clicked.connect(lambda: self._move_all_off(-0.2))
        u.clicked.connect(lambda: self._move_all_off(+0.2))
        g.addWidget(d,0,1); g.addWidget(u,0,2)
        return box

    def _per_channel(self, idx, title):
        box = QtWidgets.QGroupBox(title); box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444;")
        g = QtWidgets.QGridLayout(box); g.setContentsMargins(10,10,10,10); g.setSpacing(10)
        g.addWidget(QtWidgets.QLabel("VOLTS/DIV (SYNC)"), 0, 0)
        m = QtWidgets.QPushButton("-"); p = QtWidgets.QPushButton("+")
        for b in (m,p): b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 5px;")
        m.clicked.connect(lambda: self._vdiv(idx, 1/1.2))
        p.clicked.connect(lambda: self._vdiv(idx, 1.2))
        g.addWidget(m,0,1); g.addWidget(p,0,2)
        g.addWidget(QtWidgets.QLabel("POSITION"), 1, 0)
        d = QtWidgets.QPushButton("↓"); u = QtWidgets.QPushButton("↑")
        for b in (d,u): b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 5px;")
        d.clicked.connect(lambda: self._move_off(idx, -0.2))
        u.clicked.connect(lambda: self._move_off(idx, +0.2))
        g.addWidget(d,1,1); g.addWidget(u,1,2)
        return box

    def _group_trigger(self):
        box = QtWidgets.QGroupBox("Trigger"); box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444;")
        g = QtWidgets.QGridLayout(box); g.setContentsMargins(10,10,10,10); g.setSpacing(10)
        g.addWidget(QtWidgets.QLabel("Source"), 0, 0)
        b1 = QtWidgets.QPushButton("CH1"); b2 = QtWidgets.QPushButton("CH2"); b3 = QtWidgets.QPushButton("CH3")
        for b in (b1,b2,b3): b.setCheckable(True); b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 5px;")
        trg_group = QtWidgets.QButtonGroup(self); trg_group.addButton(b1,0); trg_group.addButton(b2,1); trg_group.addButton(b3,2); trg_group.setExclusive(True)
        b1.setChecked(True); trg_group.idClicked.connect(self._set_trg)
        g.addWidget(b1,0,1); g.addWidget(b2,0,2); g.addWidget(b3,0,3)
        g.addWidget(QtWidgets.QLabel("LEVEL"), 1, 0)
        m = QtWidgets.QPushButton('-'); p = QtWidgets.QPushButton('+')
        for b in (m,p): b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 5px;")
        m.clicked.connect(lambda: self._set_lvl(-0.1)); p.clicked.connect(lambda: self._set_lvl(+0.1))
        g.addWidget(m,1,1); g.addWidget(p,1,2)
        return box

    @QtCore.pyqtSlot(dict)
    def on_block(self, blk: dict):
        sr = blk.get('sr', None)
        if sr: self.sr = float(sr)
        t = blk['t']; a = blk['ch1']; b = blk['ch2']; c = blk['ch3']; n = t.size
        i0 = self.iw; i1 = (i0 + n) % self.N
        if i0 < i1:
            self.t[i0:i1] = t; self.ch[0][i0:i1] = a; self.ch[1][i0:i1] = b; self.ch[2][i0:i1] = c
        else:
            k = self.N - i0
            self.t[i0:] = t[:k]; self.t[:i1] = t[k:]
            self.ch[0][i0:] = a[:k]; self.ch[0][:i1] = a[k:]
            self.ch[1][i0:] = b[:k]; self.ch[1][:i1] = b[k:]
            self.ch[2][i0:] = c[:k]; self.ch[2][:i1] = c[k:]
        self.iw = i1

    def update_ui(self):
        self.lbl_sr.setText(f"SR: {self.sr/1000:.1f} kS/s")
        win  = self.time_div*10.0
        grab = win*5.0
        if self.iw == 0 and self.t[0] == 0: return
        t_right = self.t[(self.iw-1)%self.N]
        t_left  = t_right - grab
        n_need = int(self.sr*grab)+4
        i1 = self.iw; i0 = (i1 - n_need) % self.N
        if i0 < i1:
            tx = self.t[i0:i1]; a = self.ch[0][i0:i1]; b = self.ch[1][i0:i1]; c = self.ch[2][i0:i1]
        else:
            tx = np.concatenate((self.t[i0:], self.t[:i1]))
            a  = np.concatenate((self.ch[0][i0:], self.ch[0][:i1]))
            b  = np.concatenate((self.ch[1][i0:], self.ch[1][:i1]))
            c  = np.concatenate((self.ch[2][i0:], self.ch[2][:i1]))
        m = (tx >= t_left) & (tx <= t_right)
        tx, a, b, c = tx[m], a[m], b[m], c[m]
        if tx.size < 8: return
        tx_rel = tx - t_right
        src = [a,b,c][self.trigger_src]
        sgn = np.signbit(src - self.trigger_lvl)
        cross = np.where((sgn[:-1]==True) & (sgn[1:]==False))[0]
        if cross.size > 0:
            right_idx_start = int(tx.size * 0.5)
            cand = cross[cross >= right_idx_start]
            if cand.size > 0:
                k = int(cand[0]); trig_pos = -5 * self.time_div
                tx_rel = tx_rel - (tx_rel[k] - trig_pos)
            else:
                cand = cross[cross >= int(tx.size * 0.4)]
                if cand.size > 0:
                    k = int(cand[0]); trig_pos = -5 * self.time_div
                    tx_rel = tx_rel - (tx_rel[k] - trig_pos)
        pre_trigger_time = 2.0 * self.time_div
        mm = (tx_rel >= -win - pre_trigger_time) & (tx_rel <= 0.0)
        tx_rel, a, b, c = tx_rel[mm], a[mm], b[mm], c[mm]
        if tx_rel.size < 8: return
        v1, f1, _ = stats_vpp_freq(a, self.sr)
        v2, f2, _ = stats_vpp_freq(b, self.sr)
        v3, f3, _ = stats_vpp_freq(c, self.sr)
        a = a + self.offset[0]; b = b + self.offset[1]; c = c + self.offset[2]
        n = tx_rel.size; mpts = min(2500, n)
        if n > mpts:
            sel = np.linspace(0, n-1, mpts).astype(int)
            txd, ad, bd, cd = tx_rel[sel], a[sel], b[sel], c[sel]
        else:
            txd, ad, bd, cd = tx_rel, a, b, c
        self.cur1.setData(txd, ad); self.cur2.setData(txd, bd); self.cur3.setData(txd, cd)
        view_range = self.plot.getViewBox().viewRange(); x_max, y_max = view_range[0][1], view_range[1][1]
        offset_x = 20; offset_y = -10
        self.txt1.setPos(x_max - offset_x, y_max + offset_y)
        self.txt2.setPos(x_max - offset_x, y_max + offset_y - 20)
        self.txt3.setPos(x_max - offset_x, y_max + offset_y - 40)
        self.txt1.setText(f"CH1 Vpp={v1:.1f}V f={f1:.0f}Hz")
        self.txt2.setText(f"CH2 Vpp={v2:.1f}V f={f2:.0f}Hz")
        self.txt3.setText(f"CH3 Vpp={v3:.1f}V f={f3:.0f}Hz")
        self._update_axes()
        self.bad1.setText(f"CH1 {self.volts_div[0]:.2f}V/div {self.offset[0]:+,.2f}V")
        self.bad2.setText(f"CH2 {self.volts_div[1]:.2f}V/div {self.offset[1]:+,.2f}V")
        self.bad3.setText(f"CH3 {self.volts_div[2]:.2f}V/div {self.offset[2]:+,.2f}V")
        src = "CH1" if self.trigger_src == 0 else ("CH2" if self.trigger_src == 1 else "CH3")
        self.badT.setText(f"TRIG {src} {self.trigger_lvl:+.2f}V")

    def _update_axes(self):
        vdiv = self.volts_div[0]; ymax = vdiv*4.0
        self.plot.setYRange(-ymax, ymax, padding=0.05)
        win = self.time_div*10.0
        self.plot.setXRange(-win, 0.0, padding=0)

    def _time_div_scale(self, k):
        self.time_div = float(np.clip(self.time_div*k, 5e-5, 5.0)); self._update_axes()
    def _vdiv(self, idx, k):
        base = self.volts_div[idx]; nv = float(np.clip(base*k, 1e-3, 1000.0))
        self.volts_div = [nv, nv, nv]; self._update_axes()
    def _move_off(self, idx, k): self.offset[idx] += k*self.volts_div[idx]
    def _move_all_off(self, k):
        delta = k * self.volts_div[0]
        for i in range(3): self.offset[i] += delta
    def _set_trg(self, s): self.trigger_src = int(s)
    def _set_lvl(self, dv): self.trigger_lvl += dv

# =================================== MAIN ====================================

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(26, 26, 26))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(35, 35, 35))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.ToolTipText, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.BrightText, QtGui.QColor(255, 0, 0))
    palette.setColor(QtGui.QPalette.Link, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(0, 0, 0))
    app.setPalette(palette)

    # KHÔNG CÓ DEMO CHO SCOPE — chỉ UART thật
    scope_p = UARTScopeProvider()
    power_p = DemoPowerProvider(fs=4000.0, freq=50.0, vrms=220.0, irms=0.8, pf=0.85, block=512)
    ui = ScopePanelUI(scope_p, power_p)
    ui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

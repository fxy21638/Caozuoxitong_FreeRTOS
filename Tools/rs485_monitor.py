"""
RS485 实时监控台
浅色主题 · 可点击历史 · 一键导出
"""

import sys, os, time, threading, struct
from collections import deque
from datetime import datetime

try: import serial, serial.tools.list_ports
except ImportError: print("pip install pyserial"); sys.exit(1)
try:
    from PyQt5.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,
        QTextEdit,QLineEdit,QPushButton,QLabel,QSplitter,QComboBox,QStatusBar,
        QFrame,QGridLayout,QFileDialog,QTreeWidget,QTreeWidgetItem,QHeaderView,
        QAbstractItemView)
    from PyQt5.QtCore import Qt,QTimer
    from PyQt5.QtGui import QFont,QColor,QBrush
except ImportError: print("pip install PyQt5"); sys.exit(1)


# =====================================================
# 协议常量
# =====================================================
FRAME_HEAD,FRAME_TAIL = 0xAA,0x55
MSG_META = {
    0x01:("温湿度 DHT","🌡️","#2563EB"),
    0x02:("姿态角 MPU","📐","#059669"),
    0x03:("LED 控制","💡","#D97706"),
}
ADDR_NAME = {0x01:"管理端",0x02:"前端1(DHT)",0x03:"前端2(MPU)"}
CRC8_TABLE=[
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3,
]
def crc8(data): c=0; [c:=CRC8_TABLE[c^b] for b in data]; return c

def parse_frame(buf:bytes):
    if len(buf)<5 or buf[0]!=FRAME_HEAD or buf[-1]!=FRAME_TAIL:
        return False,{"raw":buf.hex(" "),"err":"帧头/尾错误"}
    addr=buf[1];length=buf[2];typ=buf[3];payload=buf[4:4+length-1]
    crc_byte=buf[4+length-1];expected=crc8(buf[:4+length-1])
    name,icon,color=MSG_META.get(typ,("未知","❓","#6B7280"))
    return (crc_byte==expected),{
        "addr":f"0x{addr:02X}","addr_name":ADDR_NAME.get(addr,"?"),
        "type":f"0x{typ:02X}","type_name":name,"type_icon":icon,"type_color":color,
        "length":length,"payload":payload.hex(" "),
        "crc_recv":f"0x{crc_byte:02X}","crc_calc":f"0x{expected:02X}",
        "crc_ok_flag":(crc_byte==expected),
        "raw":buf.hex(" ")
    }


# =====================================================
# 串口后台
# =====================================================
class SerialReader:
    def __init__(self,port,baud,cb):
        self.port=port;self.baudrate=baud;self.on_frame=cb
        self.ser=None;self.thr=None;self.running=False
    def start(self):
        try:self.ser=serial.Serial(self.port,self.baudrate,timeout=0.05)
        except:return False
        self.running=True
        self.thr=threading.Thread(target=self._loop,daemon=True);self.thr.start()
        return True
    def stop(self):
        self.running=False
        if self.ser:
            try:self.ser.close()
            except:pass
        if self.thr:self.thr.join(timeout=1)
    def _loop(self):
        st="WAIT_HEAD";buf=bytearray();exp=0;got=0
        while self.running:
            try:
                d=self.ser.read(1)
                if not d:continue
                b=d[0]
                if st=="WAIT_HEAD":
                    if b==FRAME_HEAD:buf=bytearray([b]);st="WAIT_LEN"
                elif st=="WAIT_LEN":
                    buf.append(b)
                    if 1<=b<=32:exp=b;got=0;st="WAIT_DATA"
                    else:st="WAIT_HEAD"
                elif st=="WAIT_DATA":
                    buf.append(b);got+=1
                    if got>=exp:st="WAIT_CRC"
                elif st=="WAIT_CRC":
                    buf.append(b);st="WAIT_TAIL"
                elif st=="WAIT_TAIL":
                    buf.append(b)
                    if b==FRAME_TAIL:self.on_frame(bytes(buf))
                    st="WAIT_HEAD"
            except:break
    def send(self,d):
        if self.ser and self.ser.is_open:self.ser.write(d);self.ser.flush();return True
        return False


# =====================================================
# 浅色主题
# =====================================================
LIGHT = """
*{font-family:"Microsoft YaHei","Segoe UI",sans-serif}
QMainWindow{background:#F8FAFC}
QFrame#Card{background:#FFFFFF;border:1px solid #E2E8F0;border-radius:12px}
QLabel#CardTitle{color:#475569;font-size:11pt;font-weight:600;padding:2px 4px}
QLabel#BigNumber{font-size:34pt;font-weight:bold}
QLabel#Unit{font-size:13pt;color:#94A3B8}
QLabel#Caption{color:#64748B;font-size:9pt;padding:2px 6px}
QPushButton{background:#3B82F6;color:white;border:none;border-radius:8px;
    padding:8px 18px;font-weight:600}
QPushButton:hover{background:#2563EB}
QPushButton#ConnectBtn{background:#10B981}
QPushButton#ConnectBtn:hover{background:#059669}
QPushButton#DisconnectBtn{background:#EF4444}
QPushButton#DisconnectBtn:hover{background:#DC2626}
QPushButton#PresetBtn{background:#FFFFFF;border:1.5px solid #CBD5E1;color:#1E293B;
    text-align:left;padding:10px 14px;font-size:10pt}
QPushButton#PresetBtn:hover{background:#EFF6FF;border-color:#3B82F6}
QPushButton#ExportBtn{background:#F1F5F9;color:#475569;border:1px solid #CBD5E1}
QPushButton#ExportBtn:hover{background:#E2E8F0}
QComboBox,QLineEdit{background:#FFFFFF;border:1px solid #CBD5E1;border-radius:6px;
    padding:6px 10px;color:#1E293B}
QTreeWidget{background:#FFFFFF;border:1px solid #E2E8F0;border-radius:8px;
    font-size:9pt;color:#334155;alternate-background-color:#F8FAFC}
QTreeWidget::item{padding:3px 8px;border-bottom:1px solid #F1F5F9}
QTreeWidget::item:selected{background:#DBEAFE;color:#1E40AF}
QTreeWidget::item:hover{background:#EFF6FF}
QHeaderView::section{background:#F8FAFC;color:#475569;font-weight:600;font-size:9pt;
    padding:6px 8px;border:none;border-bottom:2px solid #E2E8F0}
QTextEdit{background:#FFFFFF;border:1px solid #E2E8F0;border-radius:8px;
    font-family:"Consolas","Courier New",monospace;font-size:9.5pt;color:#334155}
QTextEdit#ExampleText{background:#FFFBEB;border:1px solid #FDE68A;
    font-family:"Consolas",monospace;font-size:8pt;color:#92400E}
QStatusBar{background:#F1F5F9;color:#64748B;border-top:1px solid #E2E8F0}
"""


# =====================================================
# 主窗口
# =====================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RS485 实时监控台 — 野火 挑战者 F429")
        self.resize(1360,880)
        self.setStyleSheet(LIGHT)
        self.reader=None
        self.history=[];self.frame_count=0
        self._build()
        self._wd=QTimer(self);self._wd.timeout.connect(self._wd_tick);self._wd.start(1500)

    # ============= 布局 =============
    def _build(self):
        c=QWidget();self.setCentralWidget(c)
        root=QVBoxLayout(c);root.setContentsMargins(14,10,14,10);root.setSpacing(8)

        # ---- 标题栏 ----
        tb=QHBoxLayout()
        title=QLabel("⚡ RS485 总线实时监控")
        title.setStyleSheet("font-size:15pt;font-weight:bold;color:#1E293B")
        tb.addWidget(title);tb.addStretch()
        self.st=QLabel("● 未连接")
        self.st.setStyleSheet("color:#EF4444;font-weight:bold;font-size:10pt;"
                              "background:#FEF2F2;padding:6px 14px;border-radius:8px")
        tb.addWidget(self.st)
        self.cnt=QLabel("帧: 0")
        self.cnt.setStyleSheet("color:#64748B;font-size:10pt");tb.addWidget(self.cnt)
        root.addLayout(tb)

        # ---- 工具栏 ----
        bar=QFrame();bar.setObjectName("Card")
        bl=QHBoxLayout(bar);bl.setContentsMargins(12,7,12,7);bl.setSpacing(8)
        bl.addWidget(QLabel("串口:"))
        self.port_cb=QComboBox();self.port_cb.setMinimumWidth(120)
        self._refresh_ports();bl.addWidget(self.port_cb)
        sc=QPushButton("🔄 扫描");sc.clicked.connect(self._refresh_ports);bl.addWidget(sc)
        bl.addSpacing(10)
        bl.addWidget(QLabel("波特率:"))
        self.baud=QComboBox();self.baud.addItems(["9600","19200","38400","57600","115200"])
        self.baud.setCurrentText("115200");bl.addWidget(self.baud)
        bl.addSpacing(10)
        self.cn=QPushButton("🔌 连接");self.cn.setObjectName("ConnectBtn")
        self.cn.clicked.connect(self._toggle_connect);bl.addWidget(self.cn)
        bl.addStretch()
        root.addWidget(bar)

        # ---- 主体: 三栏 ----
        split=QSplitter(Qt.Horizontal)

        # 左: 数据卡片 (窄)
        split.addWidget(self._left_panel())

        # 中: 帧流 + 历史表格 (宽)
        split.addWidget(self._center_panel())

        # 右: 示例 + 详情
        split.addWidget(self._right_panel())

        split.setSizes([320,560,460])
        root.addWidget(split,stretch=1)

        # ---- 底部发送条 ----
        root.addWidget(self._bottom_bar())

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("就绪 · 插入 USB-485 后点扫描 · 选串口连接")

    # ============= 左栏: 数据卡片 =============
    def _left_panel(self):
        f=QFrame();f.setObjectName("Card")
        v=QVBoxLayout(f);v.setContentsMargins(12,8,12,8);v.setSpacing(8)
        v.addWidget(QLabel("📊 实时数据"));v.itemAt(v.count()-1).widget().setObjectName("CardTitle")

        # DHT
        d=QFrame();d.setObjectName("Card");d.setStyleSheet("border:1px solid #DBEAFE")
        dv=QVBoxLayout(d);dv.setSpacing(2);dv.setContentsMargins(10,6,10,6)
        dv.addWidget(QLabel("🌡️ 温湿度 · 前端 0x02"))
        dv.itemAt(dv.count()-1).widget().setObjectName("CardTitle")
        g=QGridLayout();g.setSpacing(4)
        self.dht_t=QLabel("--.-");self.dht_t.setObjectName("BigNumber")
        self.dht_t.setStyleSheet("color:#2563EB");self.dht_t.setAlignment(Qt.AlignCenter)
        g.addWidget(self.dht_t,0,0,Qt.AlignCenter)
        g.addWidget(self._u("°C"),1,0,Qt.AlignCenter)
        self.dht_h=QLabel("--.-");self.dht_h.setObjectName("BigNumber")
        self.dht_h.setStyleSheet("color:#0891B2");self.dht_h.setAlignment(Qt.AlignCenter)
        g.addWidget(self.dht_h,0,1,Qt.AlignCenter)
        g.addWidget(self._u("%RH"),1,1,Qt.AlignCenter)
        dv.addLayout(g)
        self.dht_ts=QLabel("⏱ 等待数据...");self.dht_ts.setObjectName("Caption")
        self.dht_ts.setAlignment(Qt.AlignCenter);dv.addWidget(self.dht_ts)
        v.addWidget(d)

        # MPU
        d2=QFrame();d2.setObjectName("Card");d2.setStyleSheet("border:1px solid #D1FAE5")
        dv2=QVBoxLayout(d2);dv2.setSpacing(2);dv2.setContentsMargins(10,6,10,6)
        dv2.addWidget(QLabel("📐 姿态角 · 前端 0x03"))
        dv2.itemAt(dv2.count()-1).widget().setObjectName("CardTitle")
        g2=QGridLayout();g2.setSpacing(4)
        for col,key,color in [(0,"Pitch","#10B981"),(1,"Roll","#F59E0B"),(2,"Yaw","#EF4444")]:
            g2.addWidget(self._u(key),0,col,Qt.AlignCenter)
            lbl=QLabel("--.-");lbl.setStyleSheet(f"color:{color};font-size:20pt;font-weight:bold")
            lbl.setAlignment(Qt.AlignCenter);g2.addWidget(lbl,1,col,Qt.AlignCenter)
            g2.addWidget(self._u("°"),2,col,Qt.AlignCenter)
            setattr(self,f"mpu_{key.lower()}",lbl)
        dv2.addLayout(g2)
        self.mpu_ts=QLabel("⏱ 等待数据...");self.mpu_ts.setObjectName("Caption")
        self.mpu_ts.setAlignment(Qt.AlignCenter);dv2.addWidget(self.mpu_ts)
        v.addWidget(d2)
        v.addStretch()
        return f

    def _u(self,t): w=QLabel(t);w.setObjectName("Unit");return w

    # ============= 中栏: 帧流 + 历史表格 =============
    def _center_panel(self):
        f=QFrame();f.setObjectName("Card")
        v=QVBoxLayout(f);v.setContentsMargins(10,6,10,6);v.setSpacing(6)

        hdr=QHBoxLayout()
        hdr.addWidget(QLabel("📡 实时帧流"));hdr.itemAt(hdr.count()-1).widget().setObjectName("CardTitle")
        hdr.addStretch()
        self.frame_cnt_label=QLabel()
        self.frame_cnt_label.setStyleSheet("color:#64748B;font-size:9pt")
        hdr.addWidget(self.frame_cnt_label)
        v.addLayout(hdr)
        self.stream=QTextEdit();self.stream.setReadOnly(True);self.stream.setMaximumHeight(140)
        v.addWidget(self.stream)

        # 历史表格
        hdr2=QHBoxLayout()
        hdr2.addWidget(QLabel("📋 历史记录 (点击查看解析详情)"))
        hdr2.itemAt(hdr2.count()-1).widget().setObjectName("CardTitle")
        hdr2.addStretch()
        exp=QPushButton("📄 导出 CSV");exp.setObjectName("ExportBtn")
        exp.clicked.connect(self._export_history);hdr2.addWidget(exp)
        clr=QPushButton("🗑 清空");clr.setObjectName("ExportBtn")
        clr.clicked.connect(self._clear_history);hdr2.addWidget(clr)
        v.addLayout(hdr2)

        self.hist_tree=QTreeWidget()
        self.hist_tree.setRootIsDecorated(False)
        self.hist_tree.setAlternatingRowColors(True)
        self.hist_tree.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.hist_tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.hist_tree.setColumnCount(5)
        self.hist_tree.setHeaderLabels(["时间","方向","类型","源地址","原始 HEX"])
        self.hist_tree.header().setStretchLastSection(True)
        self.hist_tree.header().resizeSection(0,80)
        self.hist_tree.header().resizeSection(1,50)
        self.hist_tree.header().resizeSection(2,100)
        self.hist_tree.header().resizeSection(3,100)
        self.hist_tree.itemClicked.connect(self._on_history_clicked)
        v.addWidget(self.hist_tree,stretch=1)

        return f

    # ============= 右栏: 示例 + 详情 =============
    def _right_panel(self):
        f=QFrame();f.setObjectName("Card")
        v=QVBoxLayout(f);v.setContentsMargins(12,8,12,8);v.setSpacing(6)

        v.addWidget(QLabel("💡 使用说明"));v.itemAt(v.count()-1).widget().setObjectName("CardTitle")
        steps=[
            "1️⃣ 扫描串口 → 选择 → 连接",
            "2️⃣ 点示例按钮发送预设帧",
            "3️⃣ 观察左侧大数字刷新",
            "4️⃣ 点击历史记录查看解析",
            "5️⃣ 导出 CSV 保存数据",
        ]
        for s in steps:
            v.addWidget(QLabel(f"<span style='color:#64748B;font-size:9pt'>{s}</span>"))

        v.addSpacing(6)
        v.addWidget(QLabel("📋 示例帧 (可复制 HEX)"));v.itemAt(v.count()-1).widget().setObjectName("CardTitle")

        examples=[
            ("🌡️ 温湿度应答","AA 01 05 01 01 2C 01 F4 27 55",
             "温度 30.0°C (0x012C) · 湿度 50.0% (0x01F4)\n效果: LCD 左侧卡片更新"),
            ("📐 姿态角应答","AA 01 0D 02 00 00 70 41 00 00 F0 C1 00 00 34 42 7E 55",
             "Pitch=15° · Roll=-30° · Yaw=45°\n效果: LCD 右侧卡片更新"),
            ("💡 LED ON 应答","AA 01 02 03 01 1C 55",
             "LED 状态=1 (点亮)\n效果: 板载 LED1 点亮"),
        ]
        for label,hex_text,explain in examples:
            box=QFrame()
            box.setStyleSheet("background:#FFF;border:1px solid #E2E8F0;border-radius:8px;padding:6px")
            bv=QVBoxLayout(box);bv.setSpacing(3);bv.setContentsMargins(6,4,6,4)
            hdr=QHBoxLayout();hdr.addWidget(QLabel(f"<b>{label}</b>"))
            hdr.addStretch()
            btn=QPushButton("发送 →");btn.setObjectName("PresetBtn")
            btn.clicked.connect(lambda _,h=hex_text:self._send(h));hdr.addWidget(btn)
            bv.addLayout(hdr)
            hex_edit=QTextEdit();hex_edit.setObjectName("ExampleText")
            hex_edit.setPlainText(hex_text);hex_edit.setMaximumHeight(36)
            hex_edit.setToolTip("选中后 Ctrl+C 复制");bv.addWidget(hex_edit)
            expl=QLabel(explain);expl.setStyleSheet("color:#64748B;font-size:8pt;padding:2px 4px")
            expl.setWordWrap(True);bv.addWidget(expl)
            v.addWidget(box)

        v.addSpacing(6)
        v.addWidget(QLabel("✏️ 手动发送"));v.itemAt(v.count()-1).widget().setObjectName("CardTitle")
        r2=QHBoxLayout()
        self.send_edit=QLineEdit()
        self.send_edit.setPlaceholderText("AA 01 05 01 01 2C 01 F4 27 55")
        self.send_edit.returnPressed.connect(self._do_send);r2.addWidget(self.send_edit)
        sb=QPushButton("发送");sb.clicked.connect(self._do_send);r2.addWidget(sb)
        v.addLayout(r2)

        # 详情面板
        v.addSpacing(6)
        v.addWidget(QLabel("🔍 解析详情 (点击历史记录查看)"))
        v.itemAt(v.count()-1).widget().setObjectName("CardTitle")
        self.detail=QTextEdit();self.detail.setReadOnly(True)
        self.detail.setHtml('<div style="color:#94A3B8;text-align:center;padding:20px">'
                            '← 点击左侧历史记录查看解析</div>')
        v.addWidget(self.detail,stretch=1)
        return f

    def _bottom_bar(self):
        f=QFrame();f.setObjectName("Card")
        v=QHBoxLayout(f);v.setContentsMargins(12,5,12,5);v.setSpacing(6)
        v.addWidget(QLabel("快捷:"))
        for label,hex_text in [
            ("🌡️ DHT 30/50","AA 01 05 01 01 2C 01 F4 27 55"),
            ("📐 MPU 15/-30/45","AA 01 0D 02 00 00 70 41 00 00 F0 C1 00 00 34 42 7E 55"),
            ("💡 LED ON","AA 01 02 03 01 1C 55"),
        ]:
            b=QPushButton(label);b.setObjectName("PresetBtn")
            b.clicked.connect(lambda _,h=hex_text:self._send(h));v.addWidget(b)
        v.addStretch()
        return f

    # ============= 串口管理 =============
    def _refresh_ports(self):
        self.port_cb.blockSignals(True);cur=self.port_cb.currentText()
        self.port_cb.clear()
        avail=self._scan_ports()
        self.port_cb.addItems(avail if avail else["(无)"])
        if cur in avail:self.port_cb.setCurrentText(cur)
        self.port_cb.blockSignals(False)

    def _scan_ports(self):
        raw=[p.device for p in serial.tools.list_ports.comports()]
        if sys.platform=="win32":
            try:
                import winreg
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                    r"HARDWARE\DEVICEMAP\SERIALCOMM")as k:
                    i=0
                    while True:
                        try:_,v,_=winreg.EnumValue(k,i);raw.append(v);i+=1
                        except OSError:break
            except:pass
        raw=sorted(set(raw),key=lambda s:(not s.startswith("COM"),s))
        ok=[]
        try:baud=int(self.baud.currentText())
        except:baud=115200
        for p in raw:
            try:t=serial.Serial(p,baud,timeout=0.05);ok.append(p);t.close()
            except:pass
        return ok

    def _toggle_connect(self):
        if self.reader and self.reader.running:
            self.reader.stop();self.reader=None
            self._set_ui(False);return
        port=self.port_cb.currentText()
        if not port or port=="(无)":return
        baud=int(self.baud.currentText())
        self.reader=SerialReader(port,baud,self._on_frame)
        if self.reader.start():self._set_ui(True,port)
        else:self.reader=None

    def _set_ui(self,on,port=""):
        if on:
            self.st.setText(f"● 已连接 {port}")
            self.st.setStyleSheet("color:#059669;font-weight:bold;font-size:10pt;"
                                  "background:#ECFDF5;padding:6px 14px;border-radius:8px")
            self.cn.setText("🔌 断开");self.cn.setObjectName("DisconnectBtn")
        else:
            self.st.setText("● 未连接")
            self.st.setStyleSheet("color:#EF4444;font-weight:bold;font-size:10pt;"
                                  "background:#FEF2F2;padding:6px 14px;border-radius:8px")
            self.cn.setText("🔌 连接");self.cn.setObjectName("ConnectBtn")

    def _wd_tick(self):
        if self.reader and self.reader.running:
            port=self.port_cb.currentText()
            try:
                t=serial.Serial(port,int(self.baud.currentText()),timeout=0.05)
                t.close()
                self.reader.stop();self.reader=None
                self._set_ui(False);self.statusBar().showMessage(f"⚠ 串口 {port} 已断开",3000)
            except:pass
            return
        avail=self._scan_ports()
        cur=[self.port_cb.itemText(i)for i in range(self.port_cb.count())]
        if avail!=cur:
            self.port_cb.blockSignals(True);keep=self.port_cb.currentText()
            self.port_cb.clear()
            self.port_cb.addItems(avail if avail else["(无)"])
            if keep in avail:self.port_cb.setCurrentText(keep)
            self.port_cb.blockSignals(False)

    # ============= 数据处理 =============
    def _on_frame(self,frame):
        self.frame_count+=1
        ok,info=parse_frame(frame)
        ts=datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str=info["raw"]

        # 帧流
        if ok:
            line=(f'<span style="color:#94A3B8">{ts}</span> '
                  f'<span style="color:{info["type_color"]}"><b>{info["type_icon"]} {info["type_name"]}</b></span> '
                  f'<span style="color:#64748B">[{info["addr_name"]}]</span> '
                  f'<span style="color:#334155;font-family:Consolas;font-size:8.5pt">{hex_str}</span><br>')
        else:
            line=(f'<span style="color:#94A3B8">{ts}</span> '
                  f'<span style="color:#EF4444">⚠ {info["err"]}</span> '
                  f'<span style="color:#64748B;font-family:Consolas;font-size:8.5pt">{hex_str}</span><br>')
        self.stream.append(line);sb=self.stream.verticalScrollBar();sb.setValue(sb.maximum())
        self.cnt.setText(f"帧: {self.frame_count}")

        # 历史表格
        self._add_history_row(ts,"⬇ RX",info)

        # 大数字
        self._update_big_display(ok,info,ts)

    def _add_history_row(self,ts,direction,info):
        ok=info.get("crc_ok_flag",False)
        icon=info.get("type_icon","❓")
        name=info.get("type_name","未知")
        addr=info.get("addr_name","?")
        color=info["type_color"] if ok else "#EF4444"
        raw=info.get("raw","")
        item=QTreeWidgetItem()
        item.setText(0,ts)
        item.setText(1,direction)
        item.setText(2,f"{icon} {name}")
        item.setText(3,addr)
        item.setText(4,raw)
        item.setForeground(1,QBrush(QColor("#3B82F6") if "TX" in direction else QColor("#059669")))
        item.setForeground(2,QBrush(QColor(color)))
        if not ok:
            item.setForeground(4,QBrush(QColor("#EF4444")))
            item.setText(2,"⚠ 错误")
        item.setData(0,Qt.UserRole,(ok,info))  # 存解析数据
        self.hist_tree.insertTopLevelItem(0,item)
        # 限制 200 行
        while self.hist_tree.topLevelItemCount()>200:
            self.hist_tree.takeTopLevelItem(self.hist_tree.topLevelItemCount()-1)

        self.history.append((ts,raw,info))
        if len(self.history)>200:self.history=self.history[-200:]

    def _on_history_clicked(self,item):
        data=item.data(0,Qt.UserRole)
        if not data:return
        ok,info=data
        if not ok:
            self.detail.setHtml(f'<div style="color:#EF4444;padding:12px">'
                                f'<b>帧错误</b><br>'
                                f'<span style="font-family:Consolas;font-size:9pt">{info.get("raw","")}</span>'
                                f'</div>')
            return
        crc_color="#059669" if info.get("crc_ok_flag") else "#EF4444"
        self.detail.setHtml(f"""
        <div style="font-size:10pt;line-height:1.8;color:#1E293B;">
          <div style="font-size:13pt;color:{info['type_color']};margin-bottom:6px;">
            {info['type_icon']} {info['type_name']}
          </div>
          <table style="border-collapse:collapse;width:100%">
            <tr><td style="color:#64748B">源地址</td>
                <td><b>{info['addr_name']}</b> ({info['addr']})</td></tr>
            <tr><td style="color:#64748B">消息类型</td>
                <td><b>{info['type_name']}</b> ({info['type']})</td></tr>
            <tr><td style="color:#64748B">数据长度</td>
                <td>{info['length']} 字节</td></tr>
            <tr><td style="color:#64748B">数据载荷</td>
                <td style="font-family:Consolas;word-break:break-all">{info['payload']}</td></tr>
            <tr><td style="color:#64748B">CRC 接收值</td>
                <td style="font-family:Consolas">{info['crc_recv']}</td></tr>
            <tr><td style="color:#64748B">CRC 计算值</td>
                <td style="font-family:Consolas">{info['crc_calc']}</td></tr>
            <tr><td style="color:#64748B">CRC 校验</td>
                <td style="color:{crc_color};font-weight:bold">{'✓ 校验通过' if info.get('crc_ok_flag') else '✗ 校验失败'}</td></tr>
          </table>
        </div>""")

    def _update_big_display(self,ok,info,ts):
        if not ok:return
        typ=int(info["type"],16)
        if typ==0x01:
            try:
                b=bytes.fromhex(info["payload"].replace(" ",""))
                if len(b)>=4:
                    t=((b[0]<<8)|b[1])/10.0;h=((b[2]<<8)|b[3])/10.0
                    self.dht_t.setText(f"{t:.1f}");self.dht_h.setText(f"{h:.1f}")
                    self.dht_ts.setText(f"⏱ 更新于 {ts}")
            except:pass
        elif typ==0x02:
            try:
                b=bytes.fromhex(info["payload"].replace(" ",""))
                if len(b)>=12:
                    p,r,y=struct.unpack('<fff',b[:12])
                    self.mpu_pitch.setText(f"{p:.1f}")
                    self.mpu_roll.setText(f"{r:.1f}")
                    self.mpu_yaw.setText(f"{y:.1f}")
                    self.mpu_ts.setText(f"⏱ 更新于 {ts}")
            except:pass

    def _export_history(self):
        if not self.history:
            self.statusBar().showMessage("暂无历史数据",2000);return
        path,_=QFileDialog.getSaveFileName(self,"导出历史","rs485_history.csv","CSV (*.csv)")
        if not path:return
        try:
            with open(path,'w',encoding='utf-8-sig')as f:
                f.write("时间,方向,类型,源地址,数据载荷,CRC状态,原始HEX\n")
                for ts,raw,info in self.history:
                    ok="通过" if info.get("crc_ok_flag") else "失败"
                    name=info.get("type_name","未知")
                    addr=info.get("addr_name","?")
                    direction="TX" if info.get("type_name")=="TX 发送" else "RX"
                    payload=info.get("payload","")
                    f.write(f"{ts},{direction},{name},{addr},{payload},{ok},{raw}\n")
            self.statusBar().showMessage(f"已导出 {len(self.history)} 条到 {path}",3000)
        except Exception as e:
            self.statusBar().showMessage(f"导出失败: {e}",3000)

    def _clear_history(self):
        self.history.clear();self.frame_count=0
        self.stream.clear();self.hist_tree.clear();self.cnt.setText("帧: 0")
        self.dht_t.setText("--.-");self.dht_h.setText("--.-");self.dht_ts.setText("⏱ 等待数据...")
        self.mpu_pitch.setText("--.-");self.mpu_roll.setText("--.-");self.mpu_yaw.setText("--.-")
        self.mpu_ts.setText("⏱ 等待数据...")
        self.detail.setHtml('<div style="color:#94A3B8;text-align:center;padding:20px">'
                            '← 点击左侧历史记录查看解析</div>')
        self.statusBar().showMessage("已清空",2000)

    # ============= 发送 =============
    def _do_send(self):self._send(self.send_edit.text().strip())

    def _send(self,hex_text):
        if not self.reader or not self.reader.running:
            self.statusBar().showMessage("⚠ 请先连接串口",2000);return
        try:
            d=bytes.fromhex(hex_text.replace(" ",""))
            if self.reader.send(d):
                ts=datetime.now().strftime("%H:%M:%S.%f")[:-3]
                hex_str=" ".join(f"{b:02X}"for b in d)
                self.stream.append(
                    f'<span style="color:#3B82F6;font-weight:bold">⬆ [TX] {hex_str}</span><br>')
                sb=self.stream.verticalScrollBar();sb.setValue(sb.maximum())
                tx_info={"crc_ok_flag":True,"type_name":"TX 发送","type_icon":"⬆",
                         "type_color":"#3B82F6","addr_name":"本机","raw":hex_str,
                         "addr":"—","type":"—","length":"—","payload":"—",
                         "crc_recv":"—","crc_calc":"—"}
                self._add_history_row(ts,"⬆ TX",tx_info)
                self.history.append((ts,hex_str,tx_info))
                if len(self.history)>200:self.history=self.history[-200:]
                self.statusBar().showMessage(f"已发送 {len(d)} 字节",1500)
        except ValueError:
            self.statusBar().showMessage("HEX 格式错误",3000)

    def closeEvent(self,e):
        if self.reader:self.reader.stop()
        super().closeEvent(e)


def main():
    app=QApplication(sys.argv);app.setStyle("Fusion")
    win=MainWindow();win.show();sys.exit(app.exec_())

if __name__=="__main__":main()

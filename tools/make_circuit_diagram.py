"""Regenerate the README wiring diagram docs/img/circuit.svg. Pins match config.h; edit both together.
Usage: python tools/make_circuit_diagram.py"""
import sys, os
ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUT = os.path.join(ROOT, "docs", "img", "circuit.svg")
W, H = 920, 640
o = []
A = o.append
A(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" '
  'aria-label="Wiring diagram of the ESP32, RSLK motor driver board, encoders, HC-SR04 and BNO055">')
A("""<style>
.bg{fill:#fcfcfb}.t{fill:#0b0b0b}.s{fill:#52514e}.m{fill:#898781}
.box{fill:#f3f2ee;stroke:#898781}.mcu{fill:#e8f0fb;stroke:#2a78d6}
.sig{stroke:#52514e}.v5{stroke:#eb6834}.v3{stroke:#2a78d6}.gnd{stroke:#898781}
.f5{fill:#eb6834}.f3{fill:#2a78d6}.fg{fill:#898781}.res{fill:#fcfcfb;stroke:#52514e}
@media (prefers-color-scheme:dark){
.bg{fill:#1a1a19}.t{fill:#f0efec}.s{fill:#c3c2b7}.box{fill:#262624;stroke:#6b6a65}.mcu{fill:#1d2a3b;stroke:#3987e5}
.sig{stroke:#c3c2b7}.v5{stroke:#d95926}.v3{stroke:#3987e5}.f5{fill:#d95926}.f3{fill:#3987e5}.res{fill:#1a1a19;stroke:#c3c2b7}}
text{font-family:system-ui,-apple-system,"Segoe UI",sans-serif}
</style>""")
A(f'<rect class="bg" width="{W}" height="{H}" rx="8"/>')
A('<text class="t" x="24" y="30" font-size="15" font-weight="600">Wiring diagram</text>')
A('<text class="s" x="24" y="49" font-size="12">Signal pins as defined in config.h. Power: 6×AA → RSLK board → 5 V regulator → ESP32; the ESP32 3.3 V rail feeds the encoders and the BNO055</text>')

def box(x0, y0, x1, y1, title, sub=None, cls="box"):
    A(f'<rect class="{cls}" x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" rx="6" stroke-width="1.2"/>')
    A(f'<text class="t" x="{(x0+x1)/2}" y="{y0-20 if sub else y0-8}" font-size="12" font-weight="600" text-anchor="middle">{title}</text>')
    if sub: A(f'<text class="m" x="{(x0+x1)/2}" y="{y0-7}" font-size="10.5" text-anchor="middle">{sub}</text>')

def wire(pts, cls="sig", w=1.6):
    A(f'<polyline class="{cls}" fill="none" stroke-width="{w}" points="{" ".join(f"{x},{y}" for x, y in pts)}"/>')

def dot(x, y, cls="s"):
    A(f'<circle class="{cls}" cx="{x}" cy="{y}" r="3"/>')

def lab(x, y, txt, anchor="start", cls="s", size=11):
    A(f'<text class="{cls}" x="{x}" y="{y+4}" font-size="{size}" text-anchor="{anchor}">{txt}</text>')

def ground(x, y):
    wire([(x, y), (x, y+8)], "gnd")
    for i, hw in enumerate((8, 5, 2)):
        wire([(x-hw, y+8+i*3), (x+hw, y+8+i*3)], "gnd", 1.4)

def tag(x, y, txt, cls, fcls, right=True):
    """net flag: short stub + label, meaning 'connects to the net of this name'"""
    x1 = x + 18 if right else x - 18
    wire([(x, y), (x1, y)], cls)
    lab(x1 + (4 if right else -4), y, txt, "start" if right else "end", fcls.replace("f", "f"), 11)

def resistor_h(x0, x1, y, txt):
    wire([(x0, y), (x1, y)])
    A(f'<rect class="res" x="{(x0+x1)/2-13}" y="{y-5}" width="26" height="10" stroke-width="1.4"/>')
    lab((x0+x1)/2, y-14, txt, "middle", "m", 10)

def resistor_v(x, y0, y1, txt):
    wire([(x, y0), (x, y1)])
    A(f'<rect class="res" x="{x-5}" y="{(y0+y1)/2-13}" width="10" height="26" stroke-width="1.4"/>')
    lab(x+9, (y0+y1)/2, txt, "start", "m", 10)

# ---------------- ESP32 ----------------
EX0, EX1 = 440, 600
box(EX0, 90, EX1, 590, "Freenove ESP32-WROVER-E", "camera unplugged", "mcu")

# ---------------- left: battery, motors, RSLK board ----------------
BX0, BX1 = 215, 345
box(BX0, 100, BX1, 330, "RSLK MAX motor driver", "and power distribution board")
box(30, 108, 135, 160, "Battery")
lab(82, 128, "6 × AA", "middle", "s")
lab(82, 145, "power switch", "middle", "m", 10)
wire([(135, 122), (BX0, 122)], "v5"); lab(BX0+6, 122, "VBAT", cls="s")
wire([(135, 146), (BX0, 146)], "gnd"); lab(BX0+6, 146, "GND", cls="s")
box(30, 205, 135, 240, "")
lab(82, 222, "Left motor", "middle", "s")
box(30, 262, 135, 297, "")
lab(82, 279, "Right motor", "middle", "s")
wire([(135, 222), (BX0, 222)], "sig", 2.4); lab(BX0+6, 222, "M L", cls="s")
wire([(135, 279), (BX0, 279)], "sig", 2.4); lab(BX0+6, 279, "M R", cls="s")
lab(BX0+65, 315, "2 × DRV8838", "middle", "m", 10)

rows = [("PWML", "GPIO25", "sig"), ("DIRL", "GPIO26", "sig"), ("nSLPL", "GPIO27", "sig"),
        ("PWMR", "GPIO32", "sig"), ("DIRR", "GPIO33", "sig"), ("nSLPR", "GPIO13", "sig"),
        ("VREG 5V", "5V", "v5"), ("GND", "GND", "gnd")]
y = 122
for a, b, c in rows:
    wire([(BX1, y), (EX0, y)], c)
    lab(BX1-6, y, a, "end"); lab(EX0+8, y, b)
    y += 26

# ---------------- left: encoders ----------------
box(BX0, 380, BX1, 560, "Wheel encoders", "on the motor shafts, 12 CPR")
rows = [("L  A", "GPIO18", "sig"), ("L  B", "GPIO19", "sig"), ("R  A", "GPIO23", "sig"),
        ("R  B", "GPIO4", "sig"), ("VCC", "3V3", "v3"), ("GND", "GND", "gnd")]
y = 402
for a, b, c in rows:
    wire([(BX1, y), (EX0, y)], c)
    lab(BX1-6, y, a, "end"); lab(EX0+8, y, b)
    y += 26

# ---------------- right: HC-SR04 ----------------
HX0, HX1 = 730, 860
box(HX0, 110, HX1, 230, "HC-SR04 sonar", "front, 5 V")
lab(HX0+8, 132, "TRIG"); lab(HX0+8, 158, "ECHO")
lab(HX1-8, 184, "VCC", "end"); lab(HX1-8, 210, "GND", "end")
wire([(EX1, 132), (HX0, 132)]); lab(EX1-8, 132, "GPIO14", "end")
lab(EX1-8, 158, "GPIO34", "end")
wire([(EX1, 158), (645, 158)])
dot(645, 158)
resistor_h(645, HX0, 158, "1 kΩ")
resistor_v(645, 158, 215, "2 kΩ")
ground(645, 215)
tag(HX1, 184, "5V", "v5", "f5")
wire([(HX1, 210), (HX1+18, 210)], "gnd"); ground(HX1+18, 210)
lab(615, 262, "GPIO34 is input-only and not 5 V tolerant:", "start", "m", 10)
lab(615, 276, "the divider brings ECHO down to 3.3 V", "start", "m", 10)

# ---------------- right: BNO055 ----------------
IX0, IX1 = 730, 860
box(IX0, 340, IX1, 500, "BNO055 IMU", "I2C 0x28 / 0x29, 3.3 V")
rows = [("SDA", "GPIO21", "sig"), ("SCL", "GPIO22", "sig"), ("VIN", "3V3", "v3"), ("GND", "GND", "gnd")]
y = 362
for a, b, c in rows:
    wire([(EX1, y), (IX0, y)], c)
    lab(IX0+8, y, a); lab(EX1-8, y, b, "end")
    y += 26
lab(IX1-8, 440, "PS0", "end"); lab(IX1-8, 466, "PS1", "end")
wire([(IX1, 440), (IX1+22, 440), (IX1+22, 466)], "gnd"); wire([(IX1, 466), (IX1+22, 466)], "gnd")
ground(IX1+22, 466)
lab(IX0+65, 520, "PS0 and PS1 to GND select I2C mode", "middle", "m", 10)

# ---------------- legend ----------------
lx, ly = 24, 615
for txt, cls, wd in (("signal", "sig", 1.6), ("battery / 5 V", "v5", 1.6), ("3.3 V", "v3", 1.6),
                     ("ground", "gnd", 1.6), ("motor power", "sig", 2.4)):
    wire([(lx, ly), (lx+22, ly)], cls, wd + 0.8); lab(lx+28, ly, txt, size=12)
    lx += 28 + 7.4 * len(txt) + 26
A("</svg>")
open(OUT, "w", encoding="utf-8").write("\n".join(o))
print("wrote", OUT)

"""Regenerate the README system architecture and data flow diagrams in docs/img/.
They describe the module structure and control loop in the source; edit them together with the code.
Usage: python tools/make_architecture_diagrams.py"""
import os, sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUT = os.path.join(ROOT, "docs", "img")

STYLE = """<style>
.bg{fill:#fcfcfb}.t{fill:#0b0b0b}.s{fill:#52514e}.m{fill:#898781}
.box{fill:#f3f2ee;stroke:#898781}.hw{fill:#fbe9e1;stroke:#eb6834}.ui{fill:#e8f0fb;stroke:#2a78d6}
.lay{fill:none;stroke:#c3c2b7;stroke-dasharray:4 4}
.ar{stroke:#52514e}.ah{fill:#52514e}.ph{stroke:#eb6834}.phh{fill:#eb6834}.rc{stroke:#898781}.rch{fill:#898781}
@media (prefers-color-scheme:dark){
.bg{fill:#1a1a19}.t{fill:#f0efec}.s{fill:#c3c2b7}.box{fill:#262624;stroke:#6b6a65}
.hw{fill:#35211a;stroke:#d95926}.ui{fill:#1d2a3b;stroke:#3987e5}.lay{stroke:#383835}
.ar{stroke:#c3c2b7}.ah{fill:#c3c2b7}.ph{stroke:#d95926}.phh{fill:#d95926}}
text{font-family:system-ui,-apple-system,"Segoe UI",sans-serif}
</style>"""


class Svg:
    def __init__(self, w, h, title, subtitle, label):
        self.w, self.h = w, h
        self.o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" role="img" aria-label="{label}">',
                  STYLE, f'<rect class="bg" width="{w}" height="{h}" rx="8"/>',
                  f'<text class="t" x="24" y="30" font-size="15" font-weight="600">{title}</text>',
                  f'<text class="s" x="24" y="49" font-size="12">{subtitle}</text>']

    def text(self, x, y, txt, cls="s", size=11, anchor="start", weight=None):
        w = f' font-weight="{weight}"' if weight else ""
        self.o.append(f'<text class="{cls}" x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}"{w}>{txt}</text>')

    def box(self, x0, y0, x1, y1, title, lines=(), cls="box"):
        self.o.append(f'<rect class="{cls}" x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" rx="6" stroke-width="1.2"/>')
        n = 1 + len(lines)
        y = (y0 + y1) / 2 - (n - 1) * 7.5 + 4
        self.text((x0 + x1) / 2, y, title, "t", 12, "middle", 600)
        for ln in lines:
            y += 15
            self.text((x0 + x1) / 2, y, ln, "s", 10.5, "middle")

    def arrow(self, pts, label=None, at=None, cls="ar", dashed=False, anchor="middle"):
        head = {"ar": "ah", "ph": "phh", "rc": "rch"}[cls]
        d = ' stroke-dasharray="5 4"' if dashed else ""
        p = " ".join(f"{x},{y}" for x, y in pts)
        self.o.append(f'<polyline class="{cls}" fill="none" stroke-width="1.6"{d} points="{p}"/>')
        (xa, ya), (xb, yb) = pts[-2], pts[-1]
        if xa == xb:   # vertical end
            s = 1 if yb > ya else -1
            tri = f"{xb},{yb} {xb-4.5},{yb-8*s} {xb+4.5},{yb-8*s}"
        else:
            s = 1 if xb > xa else -1
            tri = f"{xb},{yb} {xb-8*s},{yb-4.5} {xb-8*s},{yb+4.5}"
        self.o.append(f'<polygon class="{head}" points="{tri}"/>')
        if label:
            self.text(at[0], at[1], label, "m", 10.5, anchor)

    def legend(self, y, items):
        x = 24
        for txt, cls, dashed in items:
            d = ' stroke-dasharray="5 4"' if dashed else ""
            self.o.append(f'<line class="{cls}" x1="{x}" x2="{x+24}" y1="{y}" y2="{y}" stroke-width="2"{d}/>')
            self.text(x + 30, y + 4, txt, "s", 12)
            x += 30 + 6.8 * len(txt) + 28

    def save(self, name):
        self.o.append("</svg>")
        open(os.path.join(OUT, name), "w", encoding="utf-8").write("\n".join(self.o))
        print("wrote", name)


# ============================ system architecture ============================
S = Svg(1010, 760, "System architecture",
        "Firmware modules on the ESP32, grouped in layers. Arrows show which module calls or feeds which",
        "Layered firmware architecture: user interfaces, communication, behaviour, control, drivers and hardware")

LX0, LX1 = 150, 860
layers = [("Users", 70, 130), ("Communication", 150, 220), ("Behaviour", 260, 345),
          ("Control", 385, 455), ("Drivers", 500, 550), ("Hardware", 590, 640)]
for name, y0, y1 in layers:
    S.o.append(f'<rect class="lay" x="24" y="{y0-8}" width="{LX1-14}" height="{y1-y0+16}" rx="6"/>')
    S.text(36, (y0 + y1) / 2 + 4, name, "m", 11.5, weight=600)


cA, cB, cC, cD = (160, 310), (330, 480), (500, 650), (670, 820)
# users
S.box(160, 70, 440, 130, "Web browser", ["phone or PC: control page"], "ui")
S.box(540, 70, 670, 130, "Serial monitor", ["USB, 115200 baud"], "ui")
S.box(690, 70, 820, 130, "PID tuner", ["tools/pid_tuner.py"], "ui")
# communication
S.box(160, 150, 440, 220, "web.*  +  web_page.h",
      ["WiFi (hotspot or own AP), mDNS rslk.local", "/  /drive  /speed  /status  /cmd  /rec"])
S.box(540, 150, 820, 220, "console.*", ["command interpreter (serial and web)", "telemetry, rec dump"])
# behaviour
S.box(cA[0], 260, cD[1], 345, "behavior.*",
      ["modes IDLE / AUTO / MANUAL  ·  overtake state machine CRUISE → PASS_OUT → PASS_BACK",
       "odometry (x, lane offset y, heading ψ)  ·  steering  ·  manual heading hold  ·  flight recorder"])
# control
S.box(cA[0], 385, cB[1], 455, "control.*  +  pid.h",
      ["setpoint ramps  ·  wheel-sync PID", "wheel speed PIDs + feedforward  ·  cal"])
# drivers
S.box(cA[0], 500, cA[1], 550, "motors.*", ["DRV8838, 20 kHz PWM"])
S.box(cB[0], 500, cB[1], 550, "encoders.*", ["PCNT, 4× quadrature"])
S.box(cC[0], 500, cC[1], 550, "sonar.*", ["HC-SR04, median of 3"])
S.box(cD[0], 500, cD[1], 550, "imu.*", ["BNO055, IMUPLUS"])
# hardware
S.box(cA[0], 590, cA[1], 640, "2 × DRV8838", ["left and right motors"], "hw")
S.box(cB[0], 590, cB[1], 640, "Wheel encoders", ["A/B per wheel"], "hw")
S.box(cC[0], 590, cC[1], 640, "HC-SR04", ["front sonar"], "hw")
S.box(cD[0], 590, cD[1], 640, "BNO055", ["I2C 21/22"], "hw")
# side: settings + config
S.box(885, 150, 995, 220, "settings.*", ["NVS flash", "Preferences"])
S.box(885, 385, 995, 455, "config.h", ["pins, defaults,", "geometry"])

# users -> communication
S.arrow([(300, 130), (300, 150)], "HTTP", (306, 144), anchor="start")
S.arrow([(605, 130), (605, 150)], "USB serial", (611, 144), anchor="start")
S.arrow([(755, 130), (755, 150)], "USB serial", (761, 144), anchor="start")
S.arrow([(440, 185), (540, 185)], "/cmd", (490, 179))
S.arrow([(820, 185), (885, 185)], "save / load", (852, 179))
# communication -> behaviour
S.arrow([(240, 220), (240, 260)], "drive pad, cruise speed", (246, 244), anchor="start")
S.arrow([(660, 220), (660, 260)], "go, stop, m S T", (666, 244), anchor="start")
# console -> control (gains)
S.arrow([(800, 220), (800, 238), (845, 238), (845, 472), (420, 472), (420, 455)], "gains, cal", (630, 467))
# behaviour <-> control
S.arrow([(210, 345), (210, 385)], "drive(speed, turn)", (216, 369), anchor="start")
S.arrow([(380, 385), (380, 345)], "wheel travel", (386, 369), anchor="start")
# control -> drivers, drivers -> control / behaviour
S.arrow([(235, 455), (235, 500)], "PWM L, R", (241, 485), anchor="start")
S.arrow([(380, 500), (380, 455)], "tick counts", (386, 485), anchor="start")
S.arrow([(575, 500), (575, 345)], "distance", (581, 490), anchor="start")
S.arrow([(745, 500), (745, 345)], "heading", (751, 490), anchor="start")
# drivers <-> hardware
S.arrow([(235, 550), (235, 590)], cls="ph")
S.arrow([(405, 590), (405, 550)], cls="ph")
S.arrow([(560, 550), (560, 590)], cls="ph")
S.arrow([(590, 590), (590, 550)], cls="ph")
S.arrow([(745, 550), (745, 590)], cls="ph")
S.arrow([(775, 590), (775, 550)], cls="ph")

S.text(24, 682, "Main loop (rslk_esp32_pid_avoid.ino), no RTOS tasks of its own:", "t", 12, weight=600)
S.text(24, 702, "consoleStep() → webStep() → sonarUpdate() every 60 ms → imuUpdate() every 10 ms → behaviorStep() → controlUpdate() every 10 ms (100 Hz)", "s", 12)
S.legend(738, [("call or data", "ar", False), ("electrical signal", "ph", False)])
S.save("architecture.svg")

# ================================ data flow ================================
D = Svg(970, 660, "Data flow: from sensors to motors",
        "One pass of the control loop. The bottom row closes the loop: wheel motion is measured by the encoders",
        "Data flow from sonar, gyro and encoders through odometry, steering, ramps and PIDs to the motors")

c = [(24, 184), (254, 424), (494, 674), (774, 944)]
A, B, C, Dr, E = (80, 140), (190, 250), (295, 350), (405, 465), (515, 570)
mid = lambda r: (r[0] + r[1]) // 2
xm = [(a + b) // 2 for a, b in c]
g01, g12, g23 = 219, 459, 734

D.box(c[0][0], A[0], c[0][1], A[1], "HC-SR04 sonar", ["echo time"], "hw")
D.box(c[1][0], A[0], c[1][1], A[1], "Sonar filter", ["median of 3, every 60 ms"])
D.box(c[2][0], A[0], c[2][1], A[1], "Overtake state machine", ["CRUISE → PASS_OUT → BACK", "lane reference y_ref, ẏ_ref"])
D.box(c[0][0], B[0], c[0][1], B[1], "BNO055 gyro", ["fused heading, 100 Hz"], "hw")
D.box(c[1][0], B[0], c[1][1], B[1], "Heading filter", ["unwrap, reject jumps > 20°"])
D.box(c[2][0], B[0], c[2][1], B[1], "Odometry", ["x, lane offset y, heading ψ", "(encoder ψ if no IMU)"])
D.box(c[3][0], A[0], c[3][1], B[1], "Steering", ["ψ_ref = atan2(ẏ_ref, v)", "  + atan2(y_ref − y, 20 cm)",
                                                "turn = v · k · (ψ_ref − ψ)", "limited to 0.6 · v"])
D.box(c[0][0], C[0], c[0][1], C[1], "Web page / console", ["user commands"], "ui")
D.box(c[3][0], C[0], c[3][1], C[1], "Mode select", ["AUTO · MANUAL · IDLE"])
D.box(c[3][0], Dr[0], c[3][1], Dr[1], "Setpoint ramps", ["3000 / 20000 ticks/s²", "→ targets tgtL, tgtR"])
D.box(c[2][0], Dr[0], c[2][1], Dr[1], "Wheel-sync PID", ["L − R travel vs ∫ commanded turn", "→ correction ± corr"])
D.box(c[1][0], Dr[0], c[1][1], Dr[1], "Wheel speed PIDs", ["feedforward + PID", "per wheel, 100 Hz"])
D.box(c[0][0], Dr[0], c[0][1], Dr[1], "DRV8838 + motors", ["20 kHz PWM"], "hw")
D.box(c[0][0], E[0], c[0][1], E[1], "Wheel encoders", ["A/B quadrature"], "hw")
D.box(c[1][0], E[0], c[1][1], E[1], "Speed estimate", ["Δticks / 10 ms, EMA α 0.3"])
D.box(c[2][0], E[0], c[2][1], E[1], "PCNT counters", ["4× quadrature, in hardware"])
D.box(c[3][0], E[0], c[3][1], E[1], "Flight recorder", ["20 Hz, 60 s → /rec CSV"])

# rows A / B (left to right)
D.arrow([(c[0][1], mid(A)), (c[1][0], mid(A))])
D.arrow([(c[1][1], mid(A)), (c[2][0], mid(A))], "distance", (g12, mid(A) - 8))
D.arrow([(c[2][1], mid(A)), (c[3][0], mid(A))], "y_ref, ẏ_ref", (g23, mid(A) - 8))
D.arrow([(c[0][1], mid(B)), (c[1][0], mid(B))])
D.arrow([(c[1][1], mid(B)), (c[2][0], mid(B))], "ψ", (g12, mid(B) - 8))
D.arrow([(c[2][1], mid(B)), (c[3][0], mid(B))], "y, ψ", (g23, mid(B) - 8))
D.arrow([(xm[2], B[0]), (xm[2], A[1])], "x (passed yet?)", (xm[2] + 6, 170), anchor="start")
# row C
D.arrow([(xm[3], B[1]), (xm[3], C[0])], "turn (AUTO)", (xm[3] + 6, 278), anchor="start")
D.arrow([(c[0][1], mid(C)), (c[3][0], mid(C))], "go / stop, cruise speed, manual speed and turn", ((c[0][1] + c[3][0]) // 2, mid(C) - 8))
# row D (right to left)
D.arrow([(xm[3], C[1]), (xm[3], Dr[0])], "drive(speed, turn)", (xm[3] + 6, 383), anchor="start")
D.arrow([(c[3][0], mid(Dr)), (c[2][1], mid(Dr))], "tgtL, tgtR", (g23 + 6, mid(Dr) - 8))
D.arrow([(c[2][0], mid(Dr)), (c[1][1], mid(Dr))], "tgt ∓ corr", (g12, mid(Dr) - 8))
D.arrow([(c[1][0], mid(Dr)), (c[0][1], mid(Dr))], "PWM", (g01, mid(Dr) - 8), cls="ph")
D.arrow([(xm[3], Dr[1]), (xm[3], E[0])], "every signal shown", (xm[3] + 6, 492), cls="rc", dashed=True, anchor="start")
# feedback (bottom row)
D.arrow([(xm[0], Dr[1]), (xm[0], E[0])], "wheels turn", (xm[0] + 6, 492), cls="ph", anchor="start")
D.arrow([(xm[0], E[1]), (xm[0], 600), (xm[2], 600), (xm[2], E[1])], "A/B pulses", ((xm[0] + xm[2]) // 2, 594), cls="ph")
D.arrow([(c[2][0], mid(E)), (c[1][1], mid(E))], "posL, posR", (g12, mid(E) - 8))
D.arrow([(xm[1], E[0]), (xm[1], Dr[1])], "velL, velR", (xm[1] + 6, 492), anchor="start")
D.arrow([(xm[2], E[0]), (xm[2], Dr[1])], "posL − posR", (xm[2] + 6, 492), anchor="start")
D.arrow([(c[2][1], mid(E)), (694, mid(E)), (694, 272), (640, 272), (640, B[1])], "wheel travel", (700, 500), anchor="start")

D.legend(638, [("data in firmware", "ar", False), ("physical / electrical", "ph", False), ("recorded", "rc", True)])
D.save("data_flow.svg")

"""Regenerate the README charts in docs/img/ from docs/data/ (tuner step tests) and logs/rslk_run_2.csv (floor run).
Usage: python tools/make_readme_plots.py"""
import csv, math, os, sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUT = os.path.join(ROOT, "docs", "img")
os.makedirs(OUT, exist_ok=True)

STYLE = """<style>
.bg{fill:#fcfcfb}.t{fill:#0b0b0b}.s{fill:#52514e}.m{fill:#898781}
.g{stroke:#e1e0d9}.a{stroke:#c3c2b7}.c1{stroke:#2a78d6}.c2{stroke:#eb6834}.tg{stroke:#52514e}
.f1{fill:#2a78d6}.f2{fill:#eb6834}.fo{fill:#898781;fill-opacity:.25;stroke:#898781}.band{fill:#898781;fill-opacity:.08}
@media (prefers-color-scheme:dark){
.bg{fill:#1a1a19}.t{fill:#f0efec}.s{fill:#c3c2b7}.g{stroke:#2c2c2a}.a{stroke:#383835}
.c1{stroke:#3987e5}.c2{stroke:#d95926}.tg{stroke:#c3c2b7}.f1{fill:#3987e5}.f2{fill:#d95926}}
text{font-family:system-ui,-apple-system,"Segoe UI",sans-serif}
</style>"""

def nice_ticks(lo, hi, n=5):
    span = hi - lo
    raw = span / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((s * mag for s in (1, 2, 2.5, 5, 10) if s * mag >= raw), default=10 * mag)
    start = math.ceil(lo / step) * step
    ticks, v = [], start
    while v <= hi + 1e-9:
        ticks.append(round(v, 10)); v += step
    return ticks

def fmt(v):
    return f"{v:g}" if abs(v) < 1e4 else f"{v:.0f}"

def chart(name, title, subtitle, xlabel, ylabel, series, xr, yr, bands=(), boxes=(), notes=(), W=760, H=340):
    """series: (label, xs, ys, cls, dashed). bands: (x0,x1,label). boxes: (x0,y0,x1,y1,label). notes: (x,y,text)."""
    L, R, T, B = 64, 20, 78, 46
    pw, ph = W - L - R, H - T - B
    X = lambda x: L + (x - xr[0]) / (xr[1] - xr[0]) * pw
    Y = lambda y: T + ph - (y - yr[0]) / (yr[1] - yr[0]) * ph
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-label="{title}">',
         STYLE, f'<rect class="bg" width="{W}" height="{H}" rx="8"/>',
         f'<text class="t" x="{L}" y="24" font-size="15" font-weight="600">{title}</text>',
         f'<text class="s" x="{L}" y="43" font-size="12">{subtitle}</text>']
    for x0, x1, lab in bands:
        o.append(f'<rect class="band" x="{X(x0):.1f}" y="{T}" width="{X(x1)-X(x0):.1f}" height="{ph}"/>')
        o.append(f'<text class="m" x="{(X(x0)+X(x1))/2:.1f}" y="{T+14}" font-size="11" text-anchor="middle">{lab}</text>')
    for v in nice_ticks(*yr):
        o.append(f'<line class="g" x1="{L}" x2="{L+pw}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" stroke-width="1"/>')
        o.append(f'<text class="m" x="{L-8}" y="{Y(v)+4:.1f}" font-size="11" text-anchor="end">{fmt(v)}</text>')
    for v in nice_ticks(*xr, n=8):
        o.append(f'<text class="m" x="{X(v):.1f}" y="{T+ph+18}" font-size="11" text-anchor="middle">{fmt(v)}</text>')
    o.append(f'<line class="a" x1="{L}" x2="{L+pw}" y1="{T+ph}" y2="{T+ph}" stroke-width="1"/>')
    o.append(f'<text class="m" x="{L+pw/2}" y="{H-8}" font-size="11" text-anchor="middle">{xlabel}</text>')
    o.append(f'<text class="m" x="14" y="{T+ph/2}" font-size="11" text-anchor="middle" transform="rotate(-90 14 {T+ph/2})">{ylabel}</text>')
    for x0, y0, x1, y1, lab in boxes:
        o.append(f'<rect class="fo" x="{X(x0):.1f}" y="{Y(y1):.1f}" width="{X(x1)-X(x0):.1f}" height="{Y(y0)-Y(y1):.1f}" rx="3"/>')
        o.append(f'<text class="s" x="{(X(x0)+X(x1))/2:.1f}" y="{Y(y0)+14:.1f}" font-size="11" text-anchor="middle">{lab}</text>')
    o.append(f'<clipPath id="p"><rect x="{L}" y="{T}" width="{pw}" height="{ph}"/></clipPath>')
    for lab, xs, ys, cls, dashed in series:
        pts = " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in zip(xs, ys))
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        width = 1.5 if dashed else 2
        o.append(f'<polyline clip-path="url(#p)" class="{cls}" fill="none" stroke-width="{width}" stroke-linejoin="round"{dash} points="{pts}"/>')
    for x, y, txt in notes:
        o.append(f'<text class="s" x="{X(x):.1f}" y="{Y(y):.1f}" font-size="11">{txt}</text>')
    # legend (one entry per distinct label)
    lx, seen = L, set()
    for lab, xs, ys, cls, dashed in series:
        if lab in seen: continue
        seen.add(lab)
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        o.append(f'<line class="{cls}" x1="{lx}" x2="{lx+20}" y1="60" y2="60" stroke-width="{1.5 if dashed else 2.5}"{dash}/>')
        o.append(f'<text class="s" x="{lx+26}" y="64" font-size="12">{lab}</text>')
        lx += 26 + 7.2 * len(lab) + 22
    o.append("</svg>")
    open(os.path.join(OUT, name), "w", encoding="utf-8").write("\n".join(o))
    print("wrote", name)

def load(path):
    rows = list(csv.DictReader(open(path)))
    return [{k: (v if k == "mode" else float(v)) for k, v in r.items()} for r in rows]

def step(run, col_t="tgtL", col_v="velL", span=800):
    R = load(os.path.join(ROOT, "docs", "data", f"tune_run{run:03d}.csv"))
    i0 = next(i for i, r in enumerate(R) if r[col_t] != 0)
    t0 = R[i0]["t"] - 20
    sel = [r for r in R[max(i0 - 2, 0):] if r["t"] - t0 <= span]
    return [r["t"] - t0 for r in sel], [r[col_v] for r in sel], [r[col_t] for r in sel]

# 1. Wheel PID: baseline vs tuned step
xa, va, ta = step(1); xb, vb, tb = step(18)
chart("pid_step_response.svg", "Wheel speed PID: step response before and after tuning",
      "Left wheel, step to 1662 ticks/s, wheels off the ground. Baseline: 12.8% overshoot, settles in 760 ms. Tuned: 3.4%, 100 ms",
      "time since step (ms)", "wheel speed (ticks/s)",
      [("target", xb, tb, "tg", True),
       ("baseline kp 0.04 ki 0.25", xa, va, "c1", False),
       ("tuned kp 0.15 ki 1.0 kd 0.002 iz 250", xb, vb, "c2", False)],
      (0, 800), (0, 2000))

# 2. Setpoint ramp: spin overshoot
xa, va, ta = step(35, span=600); xb, vb, tb = step(45, span=600)
chart("spin_ramp.svg", "Setpoint ramp removes the spin overshoot",
      "Left wheel during a spin in place (target 1219 ticks/s), sync PID on, wheels off the ground",
      "time since command (ms)", "wheel speed (ticks/s)",
      [("ramped target", xb, tb, "tg", True),
       ("no ramp: 43% overshoot", xa, va, "c1", False),
       ("ramp 20000 ticks/s²: 2.8%", xb, vb, "c2", False)],
      (0, 600), (0, 1800))

# Floor run
F = load(os.path.join(ROOT, "logs", "rslk_run_2.csv"))
t = [r["t_s"] for r in F]
def phase(m):
    s = [r["t_s"] for r in F if r["mode"] == m]
    return s[0], s[-1]
o0, o1 = phase("overtake"); m0, m1 = phase("merge")
bands = [(o0, o1, "overtake"), (m0, m1, "merge")]

# 3. Speed tracking on the floor
chart("floor_speed.svg", "Wheel speed tracking on the floor (auto mode, one overtake)",
      "Dashed = target per wheel, solid = measured. Speed ramps at 3000 ticks/s², wheels split while steering",
      "time (s)", "wheel speed (ticks/s)",
      [("targets", t, [r["tgtL"] for r in F], "tg", True), ("targets", t, [r["tgtR"] for r in F], "tg", True),
       ("left measured", t, [r["velL"] for r in F], "c1", False),
       ("right measured", t, [r["velR"] for r in F], "c2", False)],
      (0, 16), (0, 2600), bands=bands)

# 4. Heading: gyro vs encoders
chart("heading_gyro_vs_encoder.svg", "Heading: BNO055 gyro vs encoder estimate (same floor run)",
      "The robot steers on the gyro. The encoder estimate drifts 3° by the end, slip the encoders can't see",
      "time (s)", "heading (degrees, + = right)",
      [("gyro (used for steering)", t, [r["heading_deg"] for r in F], "c1", False),
       ("encoders only", t, [r["enc_heading_deg"] for r in F], "c2", False)],
      (0, 16), (-35, 55), bands=bands)

# 5. Path (top-down)
detX = next(r for r in F if r["mode"] == "overtake")
obsX = detX["x_cm"] - 0.6 + detX["dist_cm"]
chart("overtake_path.svg", "Overtake path from odometry (top-down)",
      f"Obstacle seen at {detX['dist_cm']:.0f} cm. Lane offset 30 cm. Measured on the floor: ended back on the start line",
      "distance along the lane (cm)", "sideways offset (cm, + = right)",
      [("robot path", [r["x_cm"] for r in F], [r["y_cm"] for r in F], "c1", False),
       ("original lane", [0, 400], [0, 0], "tg", True)],
      (0, 400), (-15, 40), boxes=[(obsX, -10, obsX + 20, 10, "obstacle (assumed 20 cm)")])

# 6. Sync error
chart("sync_error.svg", "Wheel-sync error on the floor (same run)",
      "Left minus right wheel travel vs commanded, in encoder ticks (65.5 ticks = 1 cm). Stays within ±11 ticks",
      "time (s)", "sync error (ticks)",
      [("sync error", t, [r["syncErr"] for r in F], "c1", False)],
      (0, 16), (-15, 15), bands=bands)

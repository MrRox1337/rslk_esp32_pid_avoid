"""
PID tuning bridge for the RSLK ESP32 firmware.

Owns the serial port, records telemetry, runs step tests, computes metrics,
saves each run as CSV + SVG, and serves a live dashboard.

  python tools/pid_tuner.py --port COM3          # dashboard at http://localhost:8765

HTTP API (used by the dashboard and by scripts):
  /cmd?c=kp%200.05                 send a console command, returns its text reply
  /test?kind=step&speed=150&hold=1500&sync=0&label=...
  /test?kind=sync&speed=150&turn=0&hold=2500&label=...
  /stop                            stop the robot
  /runs                            all run summaries (JSON)
Standard library only (no numpy / matplotlib).
"""
import argparse, re, json, os, queue, statistics, threading, time, atexit, csv
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import serial

HERE = os.path.dirname(os.path.abspath(__file__))
RUN_DIR = os.path.join(HERE, "tune_runs")
FIELDS = ["t", "tgtL", "velL", "tgtR", "velR", "outL", "outR", "syncErr", "dist"]


class Bridge:
    def __init__(self, port, tele_ms):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.tele_ms = tele_ms
        self.wlock = threading.Lock()
        self.test_lock = threading.Lock()
        self.text = queue.Queue()           # non-telemetry lines, for /cmd replies
        self.subs = []                       # SSE subscriber queues
        self.recording = None                # list of samples while a test runs
        self.latest = {}
        self.gains = {}
        self.runs = []
        self.busy = ""
        os.makedirs(RUN_DIR, exist_ok=True)
        self._load_runs()
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._pusher, daemon=True).start()
        self._tele_buf = []
        time.sleep(2.5)                      # opening the port resets the ESP32
        self.cmd("stop")
        self.cmd(f"tele {tele_ms}")
        self.refresh_gains()

    # ---------- serial ----------
    def _reader(self):
        buf = b""
        while True:
            try:
                buf += self.ser.read(4096)
            except Exception as e:
                self.broadcast("log", f"[serial error] {e}\n"); time.sleep(1); continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode(errors="replace").strip()
                if not s:
                    continue
                if s.startswith("t:"):
                    try:
                        d = {k: float(v) for k, v in (p.split(":") for p in s.split(","))}
                    except ValueError:
                        continue
                    self.latest = d
                    self._tele_buf.append(d)
                    if self.recording is not None:
                        self.recording.append(d)
                else:
                    self.text.put(s)
                    self.broadcast("log", s + "\n")

    def _pusher(self):                       # batch telemetry to the browser at 10 Hz
        while True:
            time.sleep(0.1)
            if self._tele_buf:
                b, self._tele_buf = self._tele_buf, []
                self.broadcast("tele", b)

    def write(self, c):
        with self.wlock:
            self.ser.write((c + "\n").encode())

    def cmd(self, c, quiet_s=0.35, max_s=6.0):
        while not self.text.empty():
            self.text.get_nowait()
        self.broadcast("log", f"> {c}\n")
        self.write(c)
        out, t0, last = [], time.time(), time.time()
        while time.time() - t0 < max_s:
            try:
                out.append(self.text.get(timeout=0.05)); last = time.time()
            except queue.Empty:
                if time.time() - last > quiet_s and (out or time.time() - t0 > 1.0):
                    break
        return "\n".join(out)

    def refresh_gains(self):
        txt = self.cmd("gains")
        g = {}
        for line in txt.splitlines():
            try:
                if line.startswith("wheel"):
                    nums = re.findall(r"-?\d+\.?\d*", line.split("=", 1)[1])
                    g["kp"], g["ki"], g["kd"] = [float(v) for v in nums[:3]]
                    g["iz"] = float(nums[3]) if len(nums) > 3 else 0.0
                elif line.startswith("sync"):
                    nums = line.split("=")[1].split("(")[0]
                    g["sp"], g["si"], g["sd"] = [float(v) for v in nums.split(",")]
                    g["syncOn"] = "(on)" in line
                elif line.startswith("MAX_TPS"):
                    g["maxTps"] = float(line.split("=")[1].split()[0])
                    a = re.search(r"accel = (-?[\d.]+)", line)
                    g["acc"] = float(a.group(1)) if a else 0.0
                    g["dirty"] = False
                elif "unsaved" in line:
                    g["dirty"] = True
            except ValueError:
                pass
        self.gains = g
        self.broadcast("gains", g)
        return g

    # ---------- SSE ----------
    def broadcast(self, ev, data):
        msg = f"event: {ev}\ndata: {json.dumps(data)}\n\n".encode()
        for q in list(self.subs):
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass

    # ---------- tests ----------
    def run_test(self, kind, speed, hold_ms, sync, turn, label):
        if not self.test_lock.acquire(blocking=False):
            return {"error": "a test is already running"}
        try:
            self.busy = f"{kind} {speed}"
            self.broadcast("busy", self.busy)
            g = self.refresh_gains()
            sync_was = g.get("syncOn", True)
            if kind == "step":
                self.cmd(f"sync {1 if sync else 0}", quiet_s=0.1)
            else:
                self.cmd("sync 1", quiet_s=0.1)
            self.write("stop"); time.sleep(0.4)
            self.recording = []
            time.sleep(0.25)                               # baseline
            if kind == "step":
                self.write(f"m {speed} 0"); time.sleep(hold_ms / 1000)
                self.write(f"m {speed // 2} 0"); time.sleep(hold_ms / 2000)
            else:
                self.write(f"m {speed} {turn}"); time.sleep(hold_ms / 1000)
            self.write("stop"); time.sleep(0.5)
            samples, self.recording = self.recording, None
            if bool(sync_was) != bool(sync if kind == "step" else 1):
                self.cmd(f"sync {1 if sync_was else 0}", quiet_s=0.1)
            g = self.refresh_gains()
            run = self._summarize(kind, speed, hold_ms, sync, turn, label, g, samples)
            self.runs.append(run)
            self._save(run)
            self.broadcast("run", run)
            return run
        finally:
            try:
                self.write("stop")
            except Exception:
                pass
            self.recording = None
            self.busy = ""
            self.broadcast("busy", "")
            self.test_lock.release()

    def _summarize(self, kind, speed, hold_ms, sync, turn, label, g, S):
        rid = len(self.runs) + 1
        run = {"id": rid, "kind": kind, "speed": speed, "hold": hold_ms, "sync": sync,
               "turn": turn, "label": label, "gains": g, "time": time.strftime("%H:%M:%S"),
               "samples": [[d.get(f, 0) for f in FIELDS] for d in S], "metrics": {}}
        if not S:
            run["metrics"] = {"error": "no telemetry"}; return run
        t0 = S[0]["t"]
        onset = next((d["t"] for d in S if abs(d["tgtL"]) > 0 or abs(d["tgtR"]) > 0), None)
        if onset is None:
            run["metrics"] = {"error": "robot never moved"}; return run
        run["onset"] = onset - t0
        # first step only; stop 60 ms early so the next command (or its ramp) is never included
        win = [d for d in S if onset <= d["t"] < onset + hold_ms - 60]
        m = {}
        for w in ("L", "R"):
            T = win[-1]["tgt" + w] if win else 0  # plateau target (targets may ramp up)
            if T == 0:
                continue
            m[w] = wheel_metrics(win, onset, T, "vel" + w, "out" + w)
        if kind == "sync" or sync:
            se = [d["syncErr"] for d in win]
            if se:
                m["sync"] = {"peak": max(se, key=abs), "rms": (sum(x * x for x in se) / len(se)) ** .5,
                             "final": se[-1]}
        run["metrics"] = m
        return run

    def _save(self, run):
        base = os.path.join(RUN_DIR, f"run{run['id']:03d}")
        with open(base + ".csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(FIELDS); w.writerows(run["samples"])
        with open(base + ".json", "w") as f:
            json.dump(run, f)
        with open(base + ".svg", "w", encoding="utf-8") as f:
            f.write(run_svg(run))

    def _load_runs(self):
        for fn in sorted(os.listdir(RUN_DIR)):
            if fn.endswith(".json"):
                try:
                    with open(os.path.join(RUN_DIR, fn)) as f:
                        self.runs.append(json.load(f))
                except Exception:
                    pass


def wheel_metrics(win, onset, T, vk, ok):
    t = [(d["t"] - onset) for d in win]
    v = [d[vk] for d in win]
    o = [d[ok] for d in win]
    s = 1 if T > 0 else -1
    rel = [s * x / abs(T) for x in v]                     # 1.0 = on target
    r10 = next((t[i] for i, x in enumerate(rel) if x >= 0.1), None)
    r90 = next((t[i] for i, x in enumerate(rel) if x >= 0.9), None)
    settle = 0
    for i in range(len(rel) - 1, -1, -1):
        if abs(rel[i] - 1) > 0.05:
            settle = t[i + 1] if i + 1 < len(t) else None
            break
    tail = rel[int(len(rel) * 0.6):] or rel
    return {
        "target": T,
        "rise_ms": (r90 - r10) if r10 is not None and r90 is not None else None,
        "t90_ms": r90,
        "overshoot_pct": max(0.0, (max(rel) - 1) * 100),
        "settle_ms": settle,
        "sse_pct": (statistics.mean(tail) - 1) * 100,
        "ripple_pct": (statistics.pstdev(tail) * 100) if len(tail) > 1 else 0,
        "sat_pct": 100 * sum(1 for x in o if abs(x) >= 255) / max(1, len(o)),
        "pwm_mean": statistics.mean(abs(x) for x in o[int(len(o) * 0.6):] or o),
    }


def run_svg(run):
    """Static SVG of one run: speeds on top, PWM in the middle, sync error at the bottom."""
    W, H, P = 900, 560, 50
    S = run["samples"]
    if not S:
        return f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="80"><text x="10" y="40">no data</text></svg>'
    idx = {f: i for i, f in enumerate(FIELDS)}
    t0 = S[0][0]; tmax = max(S[-1][0] - t0, 1)
    panels = [("ticks/s", ["tgtL", "velL", "tgtR", "velR"], 40, 230),
              ("PWM", ["outL", "outR"], 290, 120),
              ("sync err (ticks)", ["syncErr"], 430, 100)]
    col = {"tgtL": "#93c5fd", "velL": "#2563eb", "tgtR": "#fca5a5", "velR": "#dc2626",
           "outL": "#2563eb", "outR": "#dc2626", "syncErr": "#7c3aed"}
    dash = {"tgtL": "6 4", "tgtR": "6 4"}
    g = run["gains"]
    title = (f"Run {run['id']} · {run['kind']} speed {run['speed']} · kp {g.get('kp')} ki {g.get('ki')} "
             f"kd {g.get('kd')} · sync {'on' if (run['kind']=='sync' or run['sync']) else 'off'} "
             f"sp {g.get('sp')} si {g.get('si')} {run.get('label') or ''}")
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" font-family="sans-serif" font-size="11">',
           f'<rect width="{W}" height="{H}" fill="#fff"/>',
           f'<text x="{P}" y="22" font-size="13" font-weight="600">{title}</text>']
    for name, keys, y0, h in panels:
        vals = [r[idx[k]] for r in S for k in keys]
        lo, hi = min(vals + [0]), max(vals + [0])
        if hi - lo < 1e-6: hi = lo + 1
        pad = (hi - lo) * 0.08; lo -= pad; hi += pad
        X = lambda tt: P + (tt - t0) / tmax * (W - 2 * P)
        Y = lambda v: y0 + h - (v - lo) / (hi - lo) * h
        out.append(f'<rect x="{P}" y="{y0}" width="{W-2*P}" height="{h}" fill="none" stroke="#ccc"/>')
        out.append(f'<line x1="{P}" x2="{W-P}" y1="{Y(0):.1f}" y2="{Y(0):.1f}" stroke="#bbb" stroke-dasharray="2 3"/>')
        out.append(f'<text x="{P-4}" y="{y0+10}" text-anchor="end">{hi:.0f}</text>')
        out.append(f'<text x="{P-4}" y="{y0+h}" text-anchor="end">{lo:.0f}</text>')
        out.append(f'<text x="{P+4}" y="{y0-4}" fill="#555">{name}</text>')
        lx = W - P - 6
        for k in reversed(keys):
            pts = " ".join(f"{X(r[0]):.1f},{Y(r[idx[k]]):.1f}" for r in S)
            out.append(f'<polyline points="{pts}" fill="none" stroke="{col[k]}" stroke-width="1.6" '
                       f'stroke-dasharray="{dash.get(k, "")}"/>')
            out.append(f'<text x="{lx}" y="{y0-4}" text-anchor="end" fill="{col[k]}">{k}</text>')
            lx -= 48
    for sec in range(0, int(tmax / 1000) + 1):
        x = P + sec * 1000 / tmax * (W - 2 * P)
        out.append(f'<text x="{x:.0f}" y="{H-12}" text-anchor="middle" fill="#555">{sec}s</text>')
    out.append("</svg>")
    return "\n".join(out)


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RSLK PID Tuner</title><style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1d2330;--mut:#6b7280;--line:#d9dce3;--L:#2563eb;--Lt:#93c5fd;--R:#dc2626;--Rt:#fca5a5;--S:#7c3aed;--ok:#16a34a}
@media(prefers-color-scheme:dark){:root{--bg:#111318;--card:#1b1e25;--fg:#e6e8ee;--mut:#9aa1ad;--line:#2c313b;--L:#60a5fa;--Lt:#1e3a8a;--R:#f87171;--Rt:#7f1d1d;--S:#a78bfa}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
main{max-width:1200px;margin:0 auto;padding:16px;display:grid;gap:12px;grid-template-columns:minmax(0,2fr) minmax(0,1fr)}
@media(max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
h1{font-size:18px;margin:0}h2{font-size:13px;margin:0 0 8px;color:var(--mut);font-weight:600;text-transform:uppercase;letter-spacing:.04em}
canvas{width:100%;display:block}.leg{font-size:12px;color:var(--mut);display:flex;gap:12px;flex-wrap:wrap}.leg i{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:4px}
.g{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}.g div{font-size:11px;color:var(--mut)}.g b{display:block;font-size:16px;color:var(--fg);font-variant-numeric:tabular-nums}
pre{margin:0;height:260px;overflow:auto;font:12px/1.4 ui-monospace,monospace;background:var(--bg);border-radius:8px;padding:8px;white-space:pre-wrap}
table{width:100%;border-collapse:collapse;font-size:12px;font-variant-numeric:tabular-nums}td,th{padding:4px 6px;border-bottom:1px solid var(--line);text-align:right}th{color:var(--mut);font-weight:600}td:first-child,th:first-child{text-align:left}
.runs{grid-column:1/-1}.run{display:grid;grid-template-columns:minmax(0,3fr) minmax(0,2fr);gap:12px;border-top:1px solid var(--line);padding-top:10px;margin-top:10px}
@media(max-width:900px){.run{grid-template-columns:1fr}}
.badge{display:inline-block;padding:2px 8px;border-radius:99px;font-size:12px;background:var(--bg);border:1px solid var(--line)}
.busy{background:#f59e0b;color:#111;border:0}button{font:inherit;font-weight:600;padding:8px 14px;border-radius:8px;border:0;background:#dc2626;color:#fff;cursor:pointer}
</style></head><body><main>
<div class="card" style="grid-column:1/-1;display:flex;align-items:center;gap:12px;flex-wrap:wrap">
<h1 style="flex:1">RSLK PID Tuner</h1><span id="busy" class="badge">idle</span><span id="link" class="badge">connecting…</span><button onclick="fetch('/stop')">STOP</button></div>
<div style="display:grid;gap:12px">
<div class="card"><h2>Wheel speed (ticks/s), last 6 s</h2><canvas id="cv" height="220"></canvas>
<div class="leg"><span><i style="background:var(--Lt)"></i>target L</span><span><i style="background:var(--L)"></i>velocity L</span><span><i style="background:var(--Rt)"></i>target R</span><span><i style="background:var(--R)"></i>velocity R</span></div></div>
<div class="card"><h2>PWM</h2><canvas id="cp" height="110"></canvas></div>
<div class="card"><h2>Sync error (ticks)</h2><canvas id="cs" height="90"></canvas></div>
</div>
<div style="display:grid;gap:12px;align-content:start">
<div class="card"><h2>Gains on robot</h2><div class="g">
<div>kp<b id="kp">–</b></div><div>ki<b id="ki">–</b></div><div>kd<b id="kd">–</b></div><div>i-zone<b id="iz">–</b></div><div>accel<b id="acc">–</b></div><div>MAX_TPS<b id="maxTps">–</b></div>
<div>sp<b id="sp">–</b></div><div>si<b id="si">–</b></div><div>sd<b id="sd">–</b></div><div>sync<b id="syncOn">–</b></div></div>
<div id="dirty" style="font-size:12px;color:var(--mut);margin-top:6px"></div></div>
<div class="card"><h2>Console</h2><pre id="log"></pre></div>
</div>
<div class="card runs"><h2>Test runs (newest first)</h2><div id="runs"></div></div>
</main><script>
const $=id=>document.getElementById(id),css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const buf=[];const SPAN=6000;
function log(t){const l=$('log');l.textContent+=t;if(l.textContent.length>20000)l.textContent=l.textContent.slice(-15000);l.scrollTop=l.scrollHeight}
function chart(cv,data,series,opts={}){const dpr=devicePixelRatio||1,w=cv.clientWidth,h=+(cv.dataset.h||(cv.dataset.h=cv.getAttribute('height')));cv.width=w*dpr;cv.height=h*dpr;cv.style.height=h+'px';
 const c=cv.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);if(!data.length)return;
 const t0=opts.t0??data[0].t,t1=opts.t1??data[data.length-1].t;let lo=0,hi=0;
 data.forEach(d=>series.forEach(s=>{lo=Math.min(lo,d[s.k]);hi=Math.max(hi,d[s.k])}));if(opts.sym){const m=Math.max(-lo,hi,1);lo=-m;hi=m}if(hi-lo<1)hi=lo+1;
 const pad=(hi-lo)*.08;lo-=pad;hi+=pad;const L=44,X=t=>L+(t-t0)/Math.max(1,t1-t0)*(w-L-4),Y=v=>4+(hi-v)/(hi-lo)*(h-20);
 c.font='11px system-ui';c.fillStyle=css('--mut');c.strokeStyle=css('--line');c.lineWidth=1;
 [hi,0,lo].forEach(v=>{c.beginPath();c.moveTo(L,Y(v));c.lineTo(w-4,Y(v));c.stroke();c.fillText(Math.round(v),2,Y(v)+4)});
 if(opts.secs)for(let s=0;s<=(t1-t0)/1000;s++){c.fillText(s+'s',X(t0+s*1000)-6,h-3)}
 series.forEach(s=>{c.strokeStyle=css(s.c);c.lineWidth=s.w||1.8;c.setLineDash(s.d||[]);c.beginPath();data.forEach((d,i)=>{const x=X(d.t),y=Y(d[s.k]);i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()});c.setLineDash([])}
const SV=[{k:'tgtL',c:'--Lt',d:[6,4],w:2.5},{k:'tgtR',c:'--Rt',d:[6,4],w:2.5},{k:'velL',c:'--L'},{k:'velR',c:'--R'}],SP=[{k:'outL',c:'--L'},{k:'outR',c:'--R'}],SS=[{k:'syncErr',c:'--S'}];
function draw(){if(!buf.length)return;const t1=buf[buf.length-1].t,t0=t1-SPAN;while(buf.length&&buf[0].t<t0-500)buf.shift();
 chart($('cv'),buf,SV,{t0,t1});chart($('cp'),buf,SP,{t0,t1,sym:1});chart($('cs'),buf,SS,{t0,t1,sym:1})}
setInterval(draw,100);
const F=['t','tgtL','velL','tgtR','velR','outL','outR','syncErr','dist'];
const fmt=(v,d=0)=>v==null?'–':(+v).toFixed(d);
function addRun(r){const el=document.createElement('div');el.className='run';const g=r.gains||{},m=r.metrics||{};
 const data=r.samples.map(a=>Object.fromEntries(F.map((f,i)=>[f,a[i]])));
 const rows=['L','R'].filter(w=>m[w]).map(w=>{const x=m[w];return `<tr><td>${w}</td><td>${fmt(x.target)}</td><td>${fmt(x.rise_ms)}</td><td>${fmt(x.overshoot_pct,1)}</td><td>${fmt(x.settle_ms)}</td><td>${fmt(x.sse_pct,1)}</td><td>${fmt(x.ripple_pct,1)}</td><td>${fmt(x.sat_pct)}</td></tr>`}).join('');
 const sy=m.sync?`<div style="margin-top:6px;font-size:12px">Sync error: peak <b>${fmt(m.sync.peak)}</b>, rms <b>${fmt(m.sync.rms,1)}</b>, final <b>${fmt(m.sync.final)}</b> ticks</div>`:'';
 el.innerHTML=`<div><canvas height="170"></canvas><canvas height="70"></canvas></div><div>
 <div style="font-weight:600">#${r.id} ${r.kind} · speed ${r.speed}${r.turn?' turn '+r.turn:''} <span class="badge">${r.time}</span></div>
 <div style="font-size:12px;color:var(--mut);margin:4px 0 8px">${r.label||''}<br>kp ${g.kp} · ki ${g.ki} · kd ${g.kd} · iz ${g.iz??0} · acc ${g.acc??0} · sync ${(r.kind=='sync'||r.sync)?'on':'off'} (sp ${g.sp} si ${g.si} sd ${g.sd})</div>
 <table><tr><th>wheel</th><th>target</th><th>rise ms</th><th>overshoot %</th><th>settle ms</th><th>SS err %</th><th>ripple %</th><th>sat %</th></tr>${rows}</table>${sy}
 ${m.error?'<div style="color:var(--R)">'+m.error+'</div>':''}</div>`;
 $('runs').prepend(el);const cs=el.querySelectorAll('canvas');requestAnimationFrame(()=>{chart(cs[0],data,SV,{secs:1});chart(cs[1],data,(r.kind=='sync'||r.sync)?SS:SP,{sym:1})})}
function gains(g){["kp","ki","kd","iz","acc","sp","si","sd"].forEach(k=>$(k).textContent=g[k]??'–');$('maxTps').textContent=fmt(g.maxTps);$('syncOn').textContent=g.syncOn?'on':'off';$('dirty').textContent=g.dirty?'Unsaved changes on robot':''}
fetch('/runs').then(r=>r.json()).then(rs=>rs.forEach(addRun));fetch('/gains').then(r=>r.json()).then(gains);
const es=new EventSource('/events');es.onopen=()=>$('link').textContent='live';es.onerror=()=>$('link').textContent='no link';
es.addEventListener('tele',e=>JSON.parse(e.data).forEach(d=>buf.push(d)));es.addEventListener('log',e=>log(JSON.parse(e.data)));
es.addEventListener('run',e=>addRun(JSON.parse(e.data)));es.addEventListener('gains',e=>gains(JSON.parse(e.data)));
es.addEventListener('busy',e=>{const b=JSON.parse(e.data);$('busy').textContent=b?'running: '+b:'idle';$('busy').className='badge'+(b?' busy':'')});
addEventListener('resize',()=>{});
</script></body></html>"""


def make_handler(br):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def _send(self, code, body, ctype="application/json"):
            b = body if isinstance(body, bytes) else body.encode()
            self.send_response(code); self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

        def do_GET(self):
            u = urlparse(self.path); q = {k: v[0] for k, v in parse_qs(u.query).items()}
            if u.path == "/":
                return self._send(200, PAGE, "text/html; charset=utf-8")
            if u.path == "/events":
                self.send_response(200); self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache"); self.end_headers()
                sq = queue.Queue(maxsize=500); br.subs.append(sq)
                try:
                    while True:
                        try:
                            self.wfile.write(sq.get(timeout=15)); self.wfile.flush()
                        except queue.Empty:
                            self.wfile.write(b": ping\n\n"); self.wfile.flush()
                except Exception:
                    pass
                finally:
                    br.subs.remove(sq)
                return
            if u.path == "/cmd":
                c = q.get("c", "")
                txt = br.cmd(c, max_s=8.0 if c.startswith("cal") else 4.0)
                if c.split(" ")[0] in ("kp", "ki", "kd", "iz", "acc", "sp", "si", "sd", "sync", "cal", "save", "load", "cruise"):
                    br.refresh_gains()
                return self._send(200, txt + "\n", "text/plain; charset=utf-8")
            if u.path == "/stop":
                br.write("stop"); return self._send(200, "ok\n", "text/plain")
            if u.path == "/gains":
                return self._send(200, json.dumps(br.gains))
            if u.path == "/runs":
                return self._send(200, json.dumps(br.runs))
            if u.path == "/test":
                r = br.run_test(q.get("kind", "step"), int(q.get("speed", 150)), int(q.get("hold", 1500)),
                                int(q.get("sync", 0)), int(q.get("turn", 0)), q.get("label", ""))
                slim = {k: v for k, v in r.items() if k != "samples"}
                return self._send(200, json.dumps(slim, indent=1))
            self._send(404, "not found")
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--http", type=int, default=8765)
    ap.add_argument("--tele", type=int, default=20, help="telemetry period ms")
    a = ap.parse_args()
    br = Bridge(a.port, a.tele)
    atexit.register(lambda: (br.write("stop"), br.write("tele 1")))
    srv = ThreadingHTTPServer(("127.0.0.1", a.http), make_handler(br))
    srv.daemon_threads = True
    print(f"PID tuner on http://localhost:{a.http}  (serial {a.port})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

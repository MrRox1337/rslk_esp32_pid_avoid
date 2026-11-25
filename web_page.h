#pragma once
// The control page served at "/". Plain HTML/CSS/JS, no external resources
// (the phone has no internet while joined to the robot's WiFi).
#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RSLK Control</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1d2330;--mut:#6b7280;--line:#d9dce3;--acc:#2563eb;--go:#16a34a;--stop:#dc2626}
@media(prefers-color-scheme:dark){:root{--bg:#111318;--card:#1b1e25;--fg:#e6e8ee;--mut:#9aa1ad;--line:#2c313b;--acc:#60a5fa;--go:#22c55e;--stop:#ef4444}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
main{max-width:560px;margin:0 auto;padding:16px;display:grid;gap:12px}
h1{font-size:18px;margin:0}.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.stat{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.stat div{font-size:12px;color:var(--mut)}
.stat b{display:block;font-size:17px;color:var(--fg);font-variant-numeric:tabular-nums}
.row{display:flex;gap:8px;flex-wrap:wrap}button{flex:1;min-width:64px;padding:12px 8px;font:inherit;font-weight:600;
border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--fg);cursor:pointer;touch-action:none;user-select:none}
button:active,button.on{background:var(--acc);color:#fff;border-color:var(--acc)}
.go{background:var(--go);color:#fff;border:0}.stop{background:var(--stop);color:#fff;border:0}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.pad button{padding:20px 0;font-size:20px}
label{font-size:13px;color:var(--mut);display:flex;align-items:center;gap:8px}input[type=range]{flex:1}
form{display:flex;gap:8px}input[type=text]{flex:1;min-width:0;padding:10px;font:14px ui-monospace,monospace;
border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--fg)}form button{flex:0 0 auto}
pre{margin:8px 0 0;height:180px;overflow:auto;font:12px/1.4 ui-monospace,monospace;background:var(--bg);
border-radius:8px;padding:8px;white-space:pre-wrap}#link{font-size:12px;color:var(--mut)}
</style></head><body><main>
<div class="row" style="align-items:center"><h1 style="flex:1">RSLK Control</h1><span id="link">connecting…</span></div>
<div class="card stat">
<div>Mode<b id="mode">–</b></div><div>Distance<b id="dist">–</b></div><div>Sync err<b id="sync">–</b></div>
<div>L tgt / vel<b id="l">–</b></div><div>R tgt / vel<b id="r">–</b></div><div>PWM L / R<b id="pwm">–</b></div>
<div>Lane offset<b id="lane">–</b></div><div>IMU heading<b id="hdg">–</b></div>
</div>
<div class="card row"><button class="go" data-c="go">GO (auto)</button><button class="stop" data-c="stop">STOP</button>
<button data-c="cal">Calibrate</button><button data-c="gains">Gains</button><button data-c="save">Save</button>
<a href="/rec" download="rslk_run.csv" style="flex:1;min-width:64px"><button style="width:100%">Log</button></a></div>
<div class="card"><div class="pad" id="pad">
<button data-s="1" data-t="-.5">↖</button><button data-s="1" data-t="0">↑</button><button data-s="1" data-t=".5">↗</button>
<button data-s="0" data-t="-1">⟲</button><button class="stop" data-c="stop">■</button><button data-s="0" data-t="1">⟳</button>
<button data-s="-1" data-t="-.5">↙</button><button data-s="-1" data-t="0">↓</button><button data-s="-1" data-t=".5">↘</button>
</div><label style="margin-top:10px">Speed <input type="range" id="spd" min="60" max="230" value="150"><span id="spdv">150</span></label>
<div style="font-size:12px;color:var(--mut);margin-top:4px">Hold a button to drive; release to stop. The speed is also the GO cruise speed and is saved.</div></div>
<div class="card"><form id="f"><input type="text" id="c" placeholder="command, e.g. kp 0.05  |  m 120 0  |  help" autocomplete="off" autocapitalize="off"><button>Send</button></form>
<pre id="log"></pre></div>
</main><script>
const $=id=>document.getElementById(id),log=$('log');
function out(t){log.textContent+=t;log.scrollTop=log.scrollHeight}
async function cmd(c){out('> '+c+'\n');try{const r=await fetch('/cmd?c='+encodeURIComponent(c));out(await r.text())}catch(e){out('(no reply)\n')}}
document.querySelectorAll('[data-c]').forEach(b=>b.addEventListener('click',()=>{stopPad();cmd(b.dataset.c)}));
$('f').onsubmit=e=>{e.preventDefault();const c=$('c').value.trim();if(c)cmd(c);$('c').value=''};
const spd=$('spd');let spdSynced=false;spd.oninput=()=>{spdSynced=true;$('spdv').textContent=spd.value};
spd.onchange=()=>fetch('/speed?v='+spd.value).catch(()=>{});
let timer=null,held=null;
function send(){const v=+spd.value;fetch('/drive?s='+Math.round(held.dataset.s*v)+'&t='+Math.round(held.dataset.t*v)).catch(()=>{})}
function stopPad(){if(!held)return;clearInterval(timer);held.classList.remove('on');held=null;fetch('/cmd?c=stop').catch(()=>{})}
document.querySelectorAll('#pad [data-s]').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();stopPad();held=b;b.classList.add('on');send();timer=setInterval(send,150)});
 ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,stopPad))});
document.addEventListener('visibilitychange',stopPad);
let busy=false;setInterval(async()=>{if(busy)return;busy=true;try{const s=await (await fetch('/status')).json();
$('mode').textContent=s.mode;$('dist').textContent=s.dist.toFixed(0)+' cm';$('sync').textContent=s.sync.toFixed(0);
$('l').textContent=s.tgtL.toFixed(0)+' / '+s.velL.toFixed(0);$('r').textContent=s.tgtR.toFixed(0)+' / '+s.velR.toFixed(0);
$('pwm').textContent=s.outL+' / '+s.outR;$('lane').textContent=s.lane.toFixed(0)+' cm';$('hdg').textContent=s.imu?s.hdg.toFixed(1)+'°':'off';
if(!spdSynced){spdSynced=true;spd.value=s.cruise;$('spdv').textContent=spd.value}$('link').textContent='connected'}catch(e){$('link').textContent='no link'}busy=false},300);
</script></body></html>)HTML";

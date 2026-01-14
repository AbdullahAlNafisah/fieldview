
import uasyncio as asyncio
import json, time
import config as C
from i2c_nodes import make_i2c_nodes

INDEX_HTML = r"""
HTTP/1.1 200 OK
Content-Type: text/html; charset=utf-8
Cache-Control: no-store
Connection: close

<!doctype html><html><head><meta charset=\"utf-8\"/><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"/>
<title>ESP32‑S2 Live Plots</title>
<style>:root{--fg:#0f172a;--muted:#64748b;--bg:#f8fafc}html,body{margin:0;padding:0;background:var(--bg);color:var(--fg);font-family:ui-sans-serif,system-ui,Segoe UI,Roboto,Arial}header{padding:16px 20px;border-bottom:1px solid #e2e8f0;background:#fff;position:sticky;top:0}h1{font-size:18px;margin:0}main{padding:16px;max-width:1100px;margin:0 auto}.card{background:#fff;border:1px solid #e2e8f0;border-radius:14px;padding:14px;margin:12px 0;box-shadow:0 1px 2px rgba(0,0,0,.04)}.row{display:grid;grid-template-columns:1fr 1fr;gap:12px}@media(max-width:900px){.row{grid-template-columns:1fr}}.muted{color:var(--muted)}canvas{width:100%;height:300px}.pill{display:inline-block;padding:2px 8px;border-radius:999px;border:1px solid #e2e8f0;background:#f1f5f9;color:#334155;font-size:12px}</style>
<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script></head>
<body><header><h1>ESP32‑S2 live plots <span id=\"status\" class=\"pill\">connecting…</span></h1></header><main><div class=\"card\"><div class=\"muted\">Raw (latest): <code id=\"latest\"></code></div></div><div id=\"charts\" class=\"row\"></div></main>
<script>
const MAX_POINTS=200, N=6; const sharedOpts={responsive:true,animation:false,scales:{x:{type:'linear',ticks:{display:false},grid:{display:false}},y:{beginAtZero:false}},plugins:{legend:{display:true,position:'bottom'},decimation:{enabled:true,algorithm:'lttb',samples:200}},elements:{point:{radius:0}}};
function makeXYZ(){return[{label:'x',data:[],fill:false,tension:.15,borderWidth:2},{label:'y',data:[],fill:false,tension:.15,borderWidth:2},{label:'z',data:[],fill:false,tension:.15,borderWidth:2}]}
const charts=[],wrap=document.getElementById('charts');
for(let i=0;i<N;i++){const card=document.createElement('div');card.className='card';card.innerHTML=`<div class=\"muted\">Sensor ${i} — x,y,z</div><canvas id=\"c${i}\"></canvas>`;wrap.appendChild(card);const ctx=document.getElementById(`c${i}`);charts[i]=new Chart(ctx,{type:'line',data:{datasets:makeXYZ()},options:JSON.parse(JSON.stringify(sharedOpts))});}
function pushXYZ(chart,t,x,y,z){const d=chart.data.datasets;d[0].data.push({x:t,y:x});d[1].data.push({x:t,y:y});d[2].data.push({x:t,y:z});for(let k=0;k<3;k++){const a=d[k].data;if(a.length>MAX_POINTS)a.shift()}chart.update()}
let t=0; const latest=document.getElementById('latest'),status=document.getElementById('status');
const es=new EventSource('/sse'); es.onopen=()=>status.textContent='live'; es.onerror=()=>status.textContent='reconnecting…'; es.onmessage=(ev)=>{try{const arr=JSON.parse(ev.data); latest.textContent=arr.map(o=>`${o.idx}:{x:${o.x.toFixed(2)}, y:${o.y.toFixed(2)}, z:${o.z.toFixed(2)}, T:${o.temp.toFixed(2)}}`).join('  '); for(let i=0;i<N;i++){const o=arr[i]||arr[arr.length-1]; pushXYZ(charts[i],t,o.x,o.y,o.z);} t+=1;}catch(e){console.error('bad JSON',e)}};
</script></body></html>
"""

async def handle_http(reader, writer):
    try:
        req_line = await reader.readline()
        if not req_line:
            await writer.aclose(); return
        # Drain headers
        while True:
            line = await reader.readline()
            if not line or line == b"\r\n":
                break
        if req_line.startswith(b"GET /sse"):
            await writer.awrite("HTTP/1.1 200 OK\r\n")
            await writer.awrite("Content-Type: text/event-stream\r\n")
            await writer.awrite("Cache-Control: no-store\r\n")
            await writer.awrite("Connection: keep-alive\r\n\r\n")

            nodes = make_i2c_nodes()
            last = [(0.0,0.0,0.0,25.0) for _ in range(len(nodes))]
            period_ms = max(1, int(1000 / C.HZ))

            while True:
                t0 = time.ticks_ms()
                arr = []
                for idx, node in enumerate(nodes):
                    out = node.read()
                    if out is not None:
                        last[idx] = out
                    x,y,z,tc = last[idx]
                    # Example: flip even indices for quick visual separation
                    if (idx % 2)==0:
                        arr.append({"idx":idx,"x":x,"y":y,"z":z,"temp":tc})
                    else:
                        arr.append({"idx":idx,"x":-x,"y":y,"z":-z,"temp":tc})
                if C.DEBUG:
                    print("  ".join(f"{o['idx']}:{{x:{o['x']:.2f},y:{o['y']:.2f},z:{o['z']:.2f},T:{o['temp']:.2f}}}" for o in arr))
                try:
                    await writer.awrite("data: "); await writer.awrite(json.dumps(arr)); await writer.awrite("\n\n"); await writer.drain()
                except Exception:
                    break
                dt = time.ticks_diff(time.ticks_ms(), t0)
                sleep_ms = period_ms - dt
                await asyncio.sleep_ms(sleep_ms if sleep_ms>0 else 1)
            try: await writer.aclose()
            except Exception: pass
            return
        # Otherwise: index page
        await writer.awrite(INDEX_HTML)
        try: await writer.aclose()
        except Exception: pass
    except Exception:
        try: await writer.aclose()
        except Exception: pass

async def serve_forever():
    srv = await asyncio.start_server(handle_http, "0.0.0.0", 80)
    # Build copy‑paste friendly URLs
    try:
        import network
        sta, ap = network.WLAN(network.STA_IF), network.WLAN(network.AP_IF)
        ip = None
        if sta.active() and sta.isconnected():
            ip = sta.ifconfig()[0]
        elif ap.active():
            ip = ap.ifconfig()[0]
        else:
            ip = "0.0.0.0"
    except Exception:
        ip = "0.0.0.0"

    from app import config as C
    urls = [f"http://{ip}/"] if ip and ip != "0.0.0.0" else []
    host = getattr(C, "HOSTNAME", None)
    if host:
        urls += [f"http://{host}/", f"http://{host}.local/"]
    msg = "HTTP server: " + ("  ".join(urls) if urls else "http://0.0.0.0/")
    print(msg)

    while True:
        await asyncio.sleep(1)

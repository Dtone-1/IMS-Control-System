// IMS_RemoteWeb.cpp
#include "IMS_RemoteWeb.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// ===== 离线网页：开始/暂停 + Canvas 谱图 + WebSocket（坐标轴标注清晰）=====
// 说明：
// - 已取消“保存”按钮与 /api/save 路由
// - 远程端坐标轴：X=0..24ms（200点），Y=0..512（与你本地一致）
// - 绘图使用固定轴映射，不使用 min/max 自动缩放，保证与本地一致
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>IMS Remote</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:16px;}
    .row{display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-bottom:12px;}
    button{padding:10px 14px;border:0;border-radius:10px;background:#1b6fff;color:#fff;font-weight:600;}
    button.secondary{background:#666;}
    .card{border:1px solid #ddd;border-radius:14px;padding:12px;max-width:820px;}
    #status{font-size:14px;line-height:1.5;}
    #c{width:100%;max-width:780px;height:280px;border:1px solid #ddd;border-radius:12px;}
    .mono{font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;}
    .pill{display:inline-block;padding:2px 8px;border-radius:999px;background:#eee;margin-left:8px;}
  </style>
</head>
<body>
  <h2>IMS Remote Control</h2>

  <div class="card">
    <div class="row">
      <button onclick="postCmd('/api/start')">开始</button>
      <button class="secondary" onclick="postCmd('/api/pause')">暂停</button>
      <span id="wsState" class="pill mono">WS: --</span>
      <span id="axisState" class="pill mono">X:0..24ms Y:0..512</span>
    </div>

    <div id="status" class="mono">
      Scan: --&nbsp;&nbsp;|&nbsp;&nbsp;PeakTime: -- ms&nbsp;&nbsp;|&nbsp;&nbsp;PeakAmp: -- V
    </div>

    <div style="height:10px"></div>
    <canvas id="c" width="780" height="280"></canvas>

    <div style="height:8px"></div>
    <div class="mono" style="font-size:12px;color:#666;">
      提示：连接到 ESP32 AP 后访问 <span class="mono">http://192.168.4.1/</span>
    </div>
  </div>

<script>
let ws = null;
let lastY = null;

// 默认轴：与你本地一致（可被设备端 pushWaveform 的 xms/ymin/ymax 覆盖）
let axis = { xms:24, ymin:0, ymax:512 };

async function postCmd(path){
  try{ await fetch(path, {method:'POST'}); }
  catch(e){ console.log(e); }
}

function setWsState(s){
  document.getElementById('wsState').innerText = 'WS: ' + s;
}

function setAxisState(){
  const xms = (axis.xms !== undefined) ? axis.xms : 24;
  const ymin = (axis.ymin !== undefined) ? axis.ymin : 0;
  const ymax = (axis.ymax !== undefined) ? axis.ymax : 512;
  document.getElementById('axisState').innerText = `X:0..${xms}ms Y:${ymin}..${ymax}`;
}

function connectWS(){
  const proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
  const url = proto + location.host + '/ws';
  ws = new WebSocket(url);

  ws.onopen = ()=> setWsState('OPEN');
  ws.onclose = ()=>{
    setWsState('CLOSED');
    setTimeout(connectWS, 800);
  };
  ws.onerror = ()=> setWsState('ERROR');

  ws.onmessage = (ev)=>{
    try{
      const o = JSON.parse(ev.data);

      // 更新轴参数（关键：保证与本地坐标轴一致）
      if (o.xms !== undefined) axis.xms = o.xms;
      if (o.ymin !== undefined) axis.ymin = o.ymin;
      if (o.ymax !== undefined) axis.ymax = o.ymax;
      setAxisState();

      // 状态显示
      const scan = o.scan ? 'RUN' : 'STOP';
      const pt = (o.pt !== undefined) ? o.pt.toFixed(2) : NaN;
      const pa = (o.pa !== undefined) ? o.pa.toFixed(3) : NaN;
      document.getElementById('status').innerHTML =
        'Scan: ' + scan + ' | PeakTime: ' + (isNaN(pt)?'--':pt) + ' ms | PeakAmp: ' + (isNaN(pa)?'--':pa) + ' V';

      // 谱图
      if (o.y && o.y.length){
        lastY = o.y;
        drawWave(o.y);
      } else if (lastY) {
        drawWave(lastY);
      }

    }catch(e){
      console.log('parse err', e);
    }
  };
}

// 固定坐标轴绘图：X=0..axis.xms(ms), Y=axis.ymin..axis.ymax（默认 0..512）
// 带刻度与数值标注，避免与本地显示不一致
function drawWave(y){
  const canvas = document.getElementById('c');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;

  ctx.clearRect(0,0,W,H);

  // 坐标轴范围
  const xmin = 0;
  const xmax = (axis.xms !== undefined) ? axis.xms : 24;  // ms
  const ymin = (axis.ymin !== undefined) ? axis.ymin : 0;
  const ymax = (axis.ymax !== undefined) ? axis.ymax : 512;
  const yrng = Math.max(1, ymax - ymin);

  // 留出边距给刻度文字
  const padL = 52, padR = 14, padT = 14, padB = 34;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;

  // 坐标系原点在 (padL, padT)，绘图区左上角
  ctx.save();
  ctx.translate(padL, padT);

  // 样式
  ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace";
  ctx.fillStyle = "#111";
  ctx.strokeStyle = "#111";

  // 网格
  const xTicks = 6; // 0..24ms 分成6段
  const yTicks = 4; // 0..512 分成4段（每128）
  ctx.globalAlpha = 0.18;
  ctx.beginPath();
  for(let i=0;i<=xTicks;i++){
    const x = i*plotW/xTicks;
    ctx.moveTo(x,0); ctx.lineTo(x,plotH);
  }
  for(let i=0;i<=yTicks;i++){
    const yy = i*plotH/yTicks;
    ctx.moveTo(0,yy); ctx.lineTo(plotW,yy);
  }
  ctx.stroke();
  ctx.globalAlpha = 1.0;

  // 坐标轴线
  ctx.beginPath();
  ctx.moveTo(0,0); ctx.lineTo(0,plotH);
  ctx.moveTo(0,plotH); ctx.lineTo(plotW,plotH);
  ctx.stroke();

  // 刻度文字透明度
  ctx.globalAlpha = 0.85;

  // X 轴刻度与数值（单位 ms）
  for(let i=0;i<=xTicks;i++){
    const x = i*plotW/xTicks;
    const v = xmin + (xmax - xmin)*i/xTicks;

    ctx.beginPath();
    ctx.moveTo(x,plotH); ctx.lineTo(x,plotH+5);
    ctx.stroke();

    const txt = (Math.round(v*10)/10).toString(); // 1位小数
    const tw = ctx.measureText(txt).width;
    ctx.fillText(txt, x - tw/2, plotH + 18);
  }
  ctx.globalAlpha = 0.7;
  ctx.fillText("ms", plotW + 4, plotH + 18);
  ctx.globalAlpha = 0.85;

  // Y 轴刻度与数值（0..512）
  for(let i=0;i<=yTicks;i++){
    const yy = i*plotH/yTicks;
    const v = ymax - (ymax - ymin)*i/yTicks;

    ctx.beginPath();
    ctx.moveTo(-5,yy); ctx.lineTo(0,yy);
    ctx.stroke();

    const txt = Math.round(v).toString();
    const tw = ctx.measureText(txt).width;
    ctx.fillText(txt, -8 - tw, yy + 4);
  }

  // 轴标题（简短）
  ctx.globalAlpha = 0.6;
  ctx.fillText("Y", 6, 12);
  ctx.fillText("X", plotW - 10, plotH - 6);
  ctx.globalAlpha = 1.0;

  // 绘制谱线（固定轴映射，不做 min/max 自动缩放）
  ctx.beginPath();
  for(let i=0;i<y.length;i++){
    const x = i*(plotW)/(y.length-1);

    let v = y[i];
    if (v < ymin) v = ymin;
    if (v > ymax) v = ymax;

    const yy = (1 - (v - ymin)/yrng) * (plotH);

    if(i===0) ctx.moveTo(x,yy);
    else ctx.lineTo(x,yy);
  }
  ctx.stroke();

  ctx.restore();
}

connectWS();
</script>
</body>
</html>
)HTML";

bool IMS_RemoteWeb::beginAP(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(ssid, pass);
  if (!ok) return false;

  Serial.print("[AP] SSID: "); Serial.println(ssid);
  Serial.print("[AP] IP: ");   Serial.println(WiFi.softAPIP());

  setupRoutes_();

  // WebSocket 事件：用于串口确认连接
  ws.onEvent([](AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)serverPtr; (void)arg; (void)data; (void)len;
    if (type == WS_EVT_CONNECT) {
      Serial.printf("[WS] Client %u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("[WS] Client %u disconnected\n", client->id());
    }
  });
  server.addHandler(&ws);

  server.begin();
  Serial.println("[HTTP] Server started");
  return true;
}

void IMS_RemoteWeb::setupRoutes_() {
  // 首页
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", INDEX_HTML);
  });

  // Start
  server.on("/api/start", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (cb_start_) cb_start_();
    req->send(200, "text/plain", "START OK");
  });

  // Pause
  server.on("/api/pause", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (cb_pause_) cb_pause_();
    req->send(200, "text/plain", "PAUSE OK");
  });

  // 已取消 /api/save
}

void IMS_RemoteWeb::pushStatus(float peakTimeMs, float peakAmp, bool scanning) {
  if (ws.count() == 0) return;

  String s;
  s.reserve(256);
  s += "{\"scan\":";
  s += (scanning ? "1" : "0");
  s += ",\"pt\":";
  s += String(peakTimeMs, 2);
  s += ",\"pa\":";
  s += String(peakAmp, 3);
  s += "}";

  ws.textAll(s);
  ws.cleanupClients();
}

void IMS_RemoteWeb::pushWaveform(const int16_t* y, int n,
                                float peakTimeMs, float peakAmp,
                                bool scanning) {
  if (ws.count() == 0) return;

  // 与本地一致：X=0~24ms，Y=0~512
  const int X_MS  = 24;
  const int Y_MIN = 0;
  const int Y_MAX = 512;

  String s;
  // 200 点 JSON 通常 2~4KB
  s.reserve(4096);

  s += "{\"scan\":";
  s += (scanning ? "1" : "0");
  s += ",\"pt\":";
  s += String(peakTimeMs, 2);
  s += ",\"pa\":";
  s += String(peakAmp, 3);

  // 坐标轴参数：网页按这个固定轴绘制
  s += ",\"xms\":";
  s += X_MS;
  s += ",\"ymin\":";
  s += Y_MIN;
  s += ",\"ymax\":";
  s += Y_MAX;

  if (y && n > 0) {
    s += ",\"y\":[";
    for (int i = 0; i < n; i++) {
      s += String((int)y[i]);
      if (i != n - 1) s += ",";
    }
    s += "]";
  }

  s += "}";

  ws.textAll(s);
  ws.cleanupClients();
}

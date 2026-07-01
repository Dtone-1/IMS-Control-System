#include "IMS_RemoteWeb.h"

#include <ESPAsyncWebServer.h>
#include <WiFi.h>

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>IMS Remote Monitor</title>
  <style>
    :root{--ok:#188038;--low:#b06000;--bad:#b3261e;--muted:#5f6368;--line:#d9dee7;--ink:#17202a;--bg:#f6f8fb;}
    *{box-sizing:border-box}
    body{font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:0;background:var(--bg);color:var(--ink);}
    .page{max-width:920px;margin:0 auto;padding:18px;}
    h1{font-size:24px;margin:0 0 14px;font-weight:720;}
    .panel{background:#fff;border:1px solid var(--line);border-radius:8px;padding:14px;margin-bottom:12px;box-shadow:0 1px 2px rgba(20,30,40,.04);}
    .id-card{border-left:7px solid var(--muted);padding:16px 18px;}
    .id-title{font-size:13px;font-weight:700;letter-spacing:.04em;text-transform:uppercase;color:#647080;margin-bottom:8px;}
    .id-main{display:flex;gap:18px;align-items:flex-end;justify-content:space-between;flex-wrap:wrap;}
    .id-result{font-size:34px;line-height:1.05;font-weight:780;word-break:break-word;}
    .status-matched{border-left-color:var(--ok)} .status-matched .id-result{color:var(--ok)}
    .status-low{border-left-color:var(--low)} .status-low .id-result{color:var(--low)}
    .status-unknown{border-left-color:var(--bad)} .status-unknown .id-result{color:var(--bad)}
    .status-empty,.status-nosignal{border-left-color:var(--muted)} .status-empty .id-result,.status-nosignal .id-result{color:var(--muted)}
    .controls{display:flex;gap:10px;flex-wrap:wrap;align-items:center;}
    button{padding:10px 16px;border:0;border-radius:6px;background:#1769e0;color:#fff;font-weight:700;cursor:pointer;}
    button.secondary{background:#5f6368;}
    .pill{display:inline-block;padding:4px 9px;border-radius:999px;background:#eef1f6;color:#344054;font-size:12px;}
    .feature-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;}
    .feature{border:1px solid #e2e6ee;border-radius:6px;padding:10px;background:#fbfcfe;}
    .feature .k{font-size:12px;color:#667085;margin-bottom:3px;font-weight:700;}
    .feature .v{font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,"Liberation Mono","Courier New",monospace;font-size:18px;font-weight:650;}
    #c{width:100%;height:300px;border:1px solid var(--line);border-radius:8px;background:#fff;display:block;}
    .chart-title{display:flex;justify-content:space-between;gap:12px;align-items:center;margin-bottom:8px;color:#344054;font-weight:700;}
    .hint{font-size:12px;color:#667085;margin-top:8px;}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,"Liberation Mono","Courier New",monospace;}
    @media(max-width:700px){.feature-grid{grid-template-columns:1fr;}.id-result{font-size:28px}}
  </style>
</head>
<body>
  <main class="page">
    <h1>IMS Remote Monitor</h1>

    <section id="idCard" class="panel id-card status-nosignal">
      <div class="id-title">IMS Identification Result</div>
      <div class="id-main">
        <div>
          <div class="id-result" id="idResult">No Signal</div>
        </div>
      </div>
    </section>

    <section class="panel">
      <div class="controls">
        <button onclick="startScan()">Start</button>
        <button class="secondary" onclick="pauseScan()">Pause</button>
        <button id="btnExportSummary" onclick="exportSummaryCSV()" disabled>Export Summary CSV</button>
        <button id="btnExportSpectrum" onclick="exportSpectrumCSV()" disabled>Export Spectrum CSV</button>
        <button id="btnExportPNG" onclick="exportPNG()" disabled>Export PNG</button>
        <span id="wsState" class="pill mono">WS: --</span>
        <span id="scanState" class="pill mono">Scan: --</span>
        <span id="axisState" class="pill mono">X:0..24ms Y:0.0000..0.0010 V</span>
      </div>
    </section>

    <section class="panel">
      <div class="feature-grid">
        <div class="feature"><div class="k">Main</div><div class="v" id="mainPeak">--.-- ms</div></div>
        <div class="feature"><div class="k">Quality</div><div class="v" id="quality">Poor</div></div>
      </div>
    </section>

    <section class="panel">
      <div class="chart-title">
        <span>IMS Spectrum</span>
        <span class="mono">Drift Time / ms · Voltage / V</span>
      </div>
      <canvas id="c" width="860" height="300"></canvas>
      <div class="hint">Connect to the ESP32 AP and open <span class="mono">http://192.168.4.1/</span>.</div>
    </section>
  </main>

<script>
let ws = null;
let lastY = null;
let lastFrame = null;
let exportFrame = null;
let scanRunning = false;
let axis = { xms:24, ymin:0, ymax:0.001, yUnit:'V' };

async function postCmd(path){
  try{ await fetch(path, {method:'POST'}); }
  catch(e){ console.log(e); }
}

function $(id){ return document.getElementById(id); }

function setWsState(s){ $('wsState').innerText = 'WS: ' + s; }

function cloneFrame(f){
  return f ? JSON.parse(JSON.stringify(f)) : null;
}

function updateExportButtons(){
  const enabled = !scanRunning && exportFrame && exportFrame.y && exportFrame.y.length;
  $('btnExportSummary').disabled = !enabled;
  $('btnExportSpectrum').disabled = !enabled;
  $('btnExportPNG').disabled = !enabled;
}

async function startScan(){
  scanRunning = true;
  lastFrame = null;
  exportFrame = null;
  updateExportButtons();
  await postCmd('/api/start');
}

async function pauseScan(){
  await postCmd('/api/pause');
  scanRunning = false;
  exportFrame = null;
  updateExportButtons();
}

function formatMs(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) return '--';
  return n.toFixed(1).replace(/\.0$/, '');
}

function formatVoltage(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) return '--';
  return n.toFixed(4);
}

function frameYMax(f, y) {
  const fromFrame = Number((f && f.ymax !== undefined) ? f.ymax : axis.ymax);
  if (Number.isFinite(fromFrame) && fromFrame > 0) return fromFrame;
  let maxY = 0;
  for (const value of y) {
    const n = Number(value);
    if (Number.isFinite(n) && n > maxY) maxY = n;
  }
  return maxY > 0 ? maxY : 0.001;
}

function setAxisState(){
  const xms = Number(axis.xms ?? 24);
  const ymin = Number(axis.ymin ?? 0);
  const ymax = Number(axis.ymax ?? 0.001);
  const unit = axis.yUnit || 'V';
  $('axisState').innerText = `X:0..${formatMs(xms)}ms Y:${formatVoltage(ymin)}..${formatVoltage(ymax)} ${unit}`;
}
function qualityText(q){
  if (q >= 0.8) return 'Good';
  if (q >= 0.5) return 'Normal';
  return 'Poor';
}

function statusClass(status){
  if (status === 4) return 'status-matched';
  if (status === 3) return 'status-low';
  if (status === 2) return 'status-unknown';
  if (status === 1) return 'status-empty';
  return 'status-nosignal';
}

function statusText(status){
  if (status === 4) return 'Matched';
  if (status === 3) return 'Maybe Match';
  if (status === 2) return 'Unknown';
  if (status === 1) return 'No Library';
  return 'No Signal';
}

function displayResult(status, name){
  const fallback = statusText(status);
  const cleanName = (name && name.length) ? name : fallback;
  if (status === 4) return cleanName;
  if (status === 3) return 'Maybe ' + cleanName;
  if (status === 2) return 'Unknown';
  if (status === 1) return 'No Library';
  return 'No Signal';
}

function updateIdentification(o){
  const status = Number(o.idStatus ?? 0);
  const card = $('idCard');
  card.className = 'panel id-card ' + statusClass(status);

  $('idResult').innerText = displayResult(status, o.idName);

}

function updateFeatures(o){
  scanRunning = !!(o.running ?? o.scanning ?? o.scan);
  const scan = scanRunning ? 'RUN' : 'STOP';
  $('scanState').innerText = 'Scan: ' + scan;
  updateExportButtons();

  const main = Number(o.pt ?? o.peakTimeMs ?? 0);
  $('mainPeak').innerText = (main > 0 ? main.toFixed(2) : '--.--') + ' ms';
  const q = Number(o.qualityScore ?? 0);
  $('quality').innerText = o.qualityText || qualityText(q);

}

function connectWS(){
  const proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
  ws = new WebSocket(proto + location.host + '/ws');

  ws.onopen = ()=> setWsState('OPEN');
  ws.onclose = ()=>{ setWsState('CLOSED'); setTimeout(connectWS, 800); };
  ws.onerror = ()=> setWsState('ERROR');

  ws.onmessage = (ev)=>{
    try{
      const o = JSON.parse(ev.data);
      if (o.xms !== undefined) axis.xms = o.xms;
      if (o.ymin !== undefined) axis.ymin = o.ymin;
      if (o.ymax !== undefined) axis.ymax = o.ymax;
      if (o.yUnit !== undefined) axis.yUnit = o.yUnit;
      setAxisState();

      updateIdentification(o);
      updateFeatures(o);

      if (o.y && o.y.length){
        lastFrame = cloneFrame(o);
        lastY = o.y;
        if (!scanRunning) exportFrame = cloneFrame(lastFrame);
        updateExportButtons();
        drawWave(o.y);
      }
      else if (lastY){ drawWave(lastY); }
    }catch(e){ console.log('parse err', e); }
  };
}

function requireExportFrame(){
  if (scanRunning) {
    alert("Please pause acquisition before export.");
    return null;
  }
  if (!exportFrame || !exportFrame.y || !exportFrame.y.length) {
    alert("No frozen frame available.");
    return null;
  }
  return exportFrame;
}

function downloadText(filename, text, mime) {
  const blob = new Blob([text], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

function timestampName() {
  const d = new Date();
  const pad = n => String(n).padStart(2, "0");
  return d.getFullYear()
    + pad(d.getMonth() + 1)
    + pad(d.getDate())
    + "_"
    + pad(d.getHours())
    + pad(d.getMinutes())
    + pad(d.getSeconds());
}

function safeCsvValue(v) {
  if (v === null || v === undefined) return "";
  const n = Number(v);
  return Number.isFinite(n) ? String(n) : "";
}

function safeCsvFixed(v, digits) {
  if (v === null || v === undefined) return "";
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(digits) : "";
}

function exportSummaryCSV() {
  const f = requireExportFrame();
  if (!f) return;

  const header = "main_peak_time_ms,main_peak_amp_v,fwhm_ms,main_peak_area_v_ms,total_frame_response_v_ms,main_second_height_ratio,main_second_time_diff_ms";
  const row = [
    safeCsvValue(f.mainPeakTimeMs),
    safeCsvFixed(f.mainPeakAmpV ?? f.peakAmp ?? f.pa, 4),
    safeCsvValue(f.fwhmMs),
    safeCsvFixed(f.mainPeakAreaVMs, 4),
    safeCsvFixed(f.totalFrameResponseVMs, 4),
    safeCsvValue(f.mainSecondHeightRatio),
    safeCsvValue(f.mainSecondTimeDiffMs)
  ].join(",");

  downloadText("ims_summary_" + timestampName() + ".csv",
               header + "\n" + row + "\n",
               "text/csv;charset=utf-8");
}
function exportSpectrumCSV() {
  const f = requireExportFrame();
  if (!f) return;

  const y = f.y.map(Number);
  const xRangeMs = Number(f.xRangeMs || f.xms || 24.0);
  let lines = [];
  lines.push("index,time_ms,voltage_v");
  for (let i = 0; i < y.length; i++) {
    const t = y.length > 1 ? xRangeMs * i / (y.length - 1) : 0;
    const v = Number.isFinite(y[i]) ? y[i].toFixed(4) : "";
    lines.push(i + "," + t.toFixed(6) + "," + v);
  }

  downloadText("ims_spectrum_" + timestampName() + ".csv",
               lines.join("\n") + "\n",
               "text/csv;charset=utf-8");
}
function exportPNG() {
  const f = requireExportFrame();
  if (!f) return;

  const y = f.y.map(Number);
  const xRangeMs = Number(f.xRangeMs || f.xms || 24.0);
  const canvas = document.createElement("canvas");
  canvas.width = 2400;
  canvas.height = 1400;
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  const marginLeft = 240;
  const marginRight = 80;
  const marginTop = 100;
  const marginBottom = 190;
  const plotW = width - marginLeft - marginRight;
  const plotH = height - marginTop - marginBottom;

  let maxY = frameYMax(f, y);
  if (!Number.isFinite(maxY) || maxY <= 0) maxY = 0.001;

  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, width, height);

  ctx.strokeStyle = "#d8dce3";
  ctx.lineWidth = 2;
  const xTicks = 6;
  const yTicks = 5;
  ctx.beginPath();
  for (let k = 0; k <= xTicks; k++) {
    const x = marginLeft + plotW * k / xTicks;
    ctx.moveTo(x, marginTop);
    ctx.lineTo(x, marginTop + plotH);
  }
  for (let k = 0; k <= yTicks; k++) {
    const yy = marginTop + plotH * k / yTicks;
    ctx.moveTo(marginLeft, yy);
    ctx.lineTo(marginLeft + plotW, yy);
  }
  ctx.stroke();

  ctx.strokeStyle = "#000000";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(marginLeft, marginTop + plotH);
  ctx.lineTo(marginLeft + plotW, marginTop + plotH);
  ctx.moveTo(marginLeft, marginTop);
  ctx.lineTo(marginLeft, marginTop + plotH);
  ctx.stroke();

  ctx.fillStyle = "#000000";
  ctx.font = "28px Arial, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let k = 0; k <= xTicks; k++) {
    const x = marginLeft + plotW * k / xTicks;
    const t = xRangeMs * k / xTicks;
    ctx.beginPath();
    ctx.moveTo(x, marginTop + plotH);
    ctx.lineTo(x, marginTop + plotH + 14);
    ctx.stroke();
    ctx.fillText(t.toFixed(1), x, marginTop + plotH + 24);
  }

  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let k = 0; k <= yTicks; k++) {
    const yy = marginTop + plotH * (1 - k / yTicks);
    const val = maxY * k / yTicks;
    ctx.beginPath();
    ctx.moveTo(marginLeft - 14, yy);
    ctx.lineTo(marginLeft, yy);
    ctx.stroke();
    ctx.fillText(formatVoltage(val), marginLeft - 24, yy);
  }

  ctx.strokeStyle = "#000000";
  ctx.lineWidth = 4;
  ctx.beginPath();
  for (let i = 0; i < y.length; i++) {
    const x = marginLeft + plotW * i / Math.max(1, y.length - 1);
    let v = Number(y[i]);
    if (!Number.isFinite(v) || v < 0) v = 0;
    if (v > maxY) v = maxY;
    const yy = marginTop + plotH * (1 - v / maxY);
    if (i === 0) ctx.moveTo(x, yy);
    else ctx.lineTo(x, yy);
  }
  ctx.stroke();

  ctx.fillStyle = "#000000";
  ctx.font = "34px Arial, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("Drift time / ms", marginLeft + plotW / 2, height - 65);

  ctx.save();
  ctx.translate(75, marginTop + plotH / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.fillText("Voltage / V", 0, 0);
  ctx.restore();

  canvas.toBlob((blob) => {
    if (!blob) {
      alert("PNG export failed.");
      return;
    }
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "ims_spectrum_" + timestampName() + ".png";
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }, "image/png");
}
function drawWave(y){
  const canvas = $('c');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0,0,W,H);

  const xmax = Number(axis.xms ?? 24);
  const ymin = Number(axis.ymin ?? 0);
  const ymax = Number(axis.ymax ?? 0.001);
  const yrng = Math.max(0.000001, ymax - ymin);
  const padL = 72, padR = 16, padT = 16, padB = 38;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;

  ctx.save();
  ctx.translate(padL, padT);
  ctx.font = "12px ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,'Liberation Mono','Courier New',monospace";
  ctx.fillStyle = "#111827";
  ctx.strokeStyle = "#111827";

  const xTicks = 6, yTicks = 4;
  ctx.globalAlpha = 0.16;
  ctx.beginPath();
  for(let i=0;i<=xTicks;i++){ const x=i*plotW/xTicks; ctx.moveTo(x,0); ctx.lineTo(x,plotH); }
  for(let i=0;i<=yTicks;i++){ const yy=i*plotH/yTicks; ctx.moveTo(0,yy); ctx.lineTo(plotW,yy); }
  ctx.stroke();
  ctx.globalAlpha = 1;

  ctx.beginPath();
  ctx.moveTo(0,0); ctx.lineTo(0,plotH);
  ctx.moveTo(0,plotH); ctx.lineTo(plotW,plotH);
  ctx.stroke();

  ctx.globalAlpha = 0.85;
  for(let i=0;i<=xTicks;i++){
    const x=i*plotW/xTicks;
    const v=xmax*i/xTicks;
    ctx.beginPath(); ctx.moveTo(x,plotH); ctx.lineTo(x,plotH+5); ctx.stroke();
    const txt=formatMs(v);
    ctx.fillText(txt, x-ctx.measureText(txt).width/2, plotH+20);
  }
  ctx.fillText("Drift Time / ms", plotW-108, plotH+34);

  for(let i=0;i<=yTicks;i++){
    const yy=i*plotH/yTicks;
    const v=ymax-(ymax-ymin)*i/yTicks;
    ctx.beginPath(); ctx.moveTo(-5,yy); ctx.lineTo(0,yy); ctx.stroke();
    const txt=formatVoltage(v);
    ctx.fillText(txt, -10-ctx.measureText(txt).width, yy+4);
  }
  ctx.save();
  ctx.rotate(-Math.PI/2);
  ctx.fillText("Voltage / V", -70, -56);
  ctx.restore();

  ctx.strokeStyle = "#1769e0";
  ctx.lineWidth = 2;
  ctx.beginPath();
  const denom = Math.max(1, y.length - 1);
  for(let i=0;i<y.length;i++){
    const x=i*plotW/denom;
    let v=Number(y[i]);
    if (!Number.isFinite(v)) v = ymin;
    if (v < ymin) v = ymin;
    if (v > ymax) v = ymax;
    const yy=(1-(v-ymin)/yrng)*plotH;
    if(i===0) ctx.moveTo(x,yy); else ctx.lineTo(x,yy);
  }
  ctx.stroke();
  ctx.restore();
}
connectWS();
</script>
</body>
</html>
)HTML";

static const char* statusTextFromId(int idStatus) {
  switch (idStatus) {
    case 4: return "Matched";
    case 3: return "Maybe Match";
    case 2: return "Unknown";
    case 1: return "No Library";
    default: return "No Signal";
  }
}

static const char* qualityTextFromScore(float q) {
  if (q >= 0.8f) return "Good";
  if (q >= 0.5f) return "Normal";
  return "Poor";
}

static void appendJsonString(String &s, const char *value) {
  s += '"';
  if (value != nullptr) {
    for (const char *p = value; *p != '\0'; ++p) {
      char c = *p;
      if (c == '"' || c == '\\') {
        s += '\\';
        s += c;
      } else if ((uint8_t)c >= 0x20) {
        s += c;
      }
    }
  }
  s += '"';
}

static float roundUpToStep(float value, float step) {
  if (step <= 0.0f) return value;
  int steps = (int)(value / step);
  if ((float)steps * step < value) steps++;
  if (steps < 1) steps = 1;
  return (float)steps * step;
}

static float chooseVoltageAxisMax(const float *yV, int n) {
  float maxV = 0.0f;
  if (yV && n > 0) {
    for (int i = 0; i < n; i++) {
      if (yV[i] > maxV) maxV = yV[i];
    }
  }

  float target = maxV * 1.10f;
  if (target <= 0.001f) return 0.001f;

  float step = 0.001f;
  if (target > 0.010f) step = 0.002f;
  if (target > 0.020f) step = 0.005f;
  if (target > 0.050f) step = 0.010f;
  if (target > 0.100f) step = 0.020f;
  if (target > 0.200f) step = 0.050f;
  if (target > 0.500f) step = 0.100f;
  if (target > 1.000f) step = 0.200f;
  if (target > 2.000f) step = 0.500f;
  return roundUpToStep(target, step);
}

bool IMS_RemoteWeb::beginAP(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(ssid, pass);
  if (!ok) return false;

  Serial.print("[AP] SSID: "); Serial.println(ssid);
  Serial.print("[AP] IP: ");   Serial.println(WiFi.softAPIP());

  setupRoutes_();

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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", INDEX_HTML);
  });

  server.on("/api/start", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (cb_start_) cb_start_();
    req->send(200, "text/plain", "START OK");
  });

  server.on("/api/pause", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (cb_pause_) cb_pause_();
    req->send(200, "text/plain", "PAUSE OK");
  });
}

void IMS_RemoteWeb::pushStatus(float peakTimeMs, float peakAmp, bool scanning) {
  if (ws.count() == 0) return;

  String s;
  s.reserve(320);
  s += "{\"type\":\"status\",\"scan\":";
  s += (scanning ? "1" : "0");
  s += ",\"scanning\":";
  s += (scanning ? "true" : "false");
  s += ",\"pt\":";
  s += String(peakTimeMs, 2);
  s += ",\"peakTimeMs\":";
  s += String(peakTimeMs, 2);
  s += ",\"pa\":";
  s += String(peakAmp, 3);
  s += ",\"peakAmp\":";
  s += String(peakAmp, 3);
  s += "}";

  ws.textAll(s);
  ws.cleanupClients();
}

size_t IMS_RemoteWeb::clientCount() const {
  return ws.count();
}

void IMS_RemoteWeb::pushWaveform(const float* yV, int n,
                                float peakTimeMs, float peakAmp,
                                bool scanning,
                                const char* idName,
                                int idStatus,
                                float featureQuality,
                                uint32_t frameId,
                                float xRangeMs,
                                float fwhmMs,
                                float mainPeakAreaVMs,
                                float totalFrameResponseVMs,
                                bool hasSecondPeak,
                                float mainSecondHeightRatio,
                                float mainSecondTimeDiffMs) {
  if (ws.count() == 0) return;
  if (scanning) {
    pushStatus(peakTimeMs, peakAmp, true);
    return;
  }

  const float yMin = 0.0f;
  const float yMax = chooseVoltageAxisMax(yV, n);
  String s;
  s.reserve(7800);

  s += "{\"type\":\"waveform\",\"scan\":";
  s += (scanning ? "1" : "0");
  s += ",\"running\":";
  s += (scanning ? "true" : "false");
  s += ",\"scanning\":";
  s += (scanning ? "true" : "false");
  s += ",\"frameId\":";
  s += frameId;
  s += ",\"pt\":";
  s += String(peakTimeMs, 2);
  s += ",\"peakTimeMs\":";
  s += String(peakTimeMs, 2);
  s += ",\"mainPeakTimeMs\":";
  s += String(peakTimeMs, 3);
  s += ",\"pa\":";
  s += String(peakAmp, 6);
  s += ",\"peakAmp\":";
  s += String(peakAmp, 6);
  s += ",\"mainPeakAmpV\":";
  s += String(peakAmp, 6);
  s += ",\"fwhmMs\":";
  s += String(fwhmMs, 3);
  s += ",\"mainPeakAreaVMs\":";
  s += String(mainPeakAreaVMs, 6);
  s += ",\"totalFrameResponseVMs\":";
  s += String(totalFrameResponseVMs, 6);
  s += ",\"mainSecondHeightRatio\":";
  if (hasSecondPeak) s += String(mainSecondHeightRatio, 3);
  else s += "null";
  s += ",\"mainSecondTimeDiffMs\":";
  if (hasSecondPeak) s += String(mainSecondTimeDiffMs, 3);
  else s += "null";
  s += ",\"qualityScore\":";
  s += String(featureQuality, 2);
  s += ",\"qualityText\":";
  appendJsonString(s, qualityTextFromScore(featureQuality));
  s += ",\"idName\":";
  appendJsonString(s, idName);
  s += ",\"idStatus\":";
  s += idStatus;
  s += ",\"idStatusText\":";
  appendJsonString(s, statusTextFromId(idStatus));
  s += ",\"xms\":";
  s += String(xRangeMs, 3);
  s += ",\"xRangeMs\":";
  s += String(xRangeMs, 3);
  s += ",\"ymin\":";
  s += String(yMin, 4);
  s += ",\"ymax\":";
  s += String(yMax, 4);
  s += ",\"yUnit\":\"V\"";

  if (yV && n > 0) {
    s += ",\"y\":[";
    for (int i = 0; i < n; i++) {
      s += String(yV[i], 6);
      if (i != n - 1) s += ",";
    }
    s += "]";
  }

  s += "}";

  ws.textAll(s);
  ws.cleanupClients();
}

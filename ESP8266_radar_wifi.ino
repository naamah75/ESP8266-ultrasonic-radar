#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <math.h>

#include "secrets.h"

// ================= DEBUG =================
// 0 = OFF
// 1 = BASIC
// 2 = VERBOSE
#define DEBUG_SERIAL 1

#if DEBUG_SERIAL >= 1
  #define DBG1(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG1(...)
#endif

#if DEBUG_SERIAL >= 2
  #define DBG2(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG2(...)
#endif

// ================= UI toggles =================
// 0 = no sweep line  1 = sweep line enabled
#define UI_SHOW_BEAM 1

ESP8266WebServer server(80);
Servo myServo;

// ================= PIN =================
static const int PIN_SERVO = D5;
static const int PIN_TRIG  = D6;
static const int PIN_ECHO  = D7;

// ================= RADAR =================
static const int ANGLE_MIN  = 10;
static const int ANGLE_MAX  = 170;
static const int ANGLE_STEP = 1;
static const uint32_t LOOP_PERIOD_MS = 35;

// ====== SINGLE SOURCE OF TRUTH ======
static const uint16_t MAX_CM = 200;
static const uint16_t TIMEOUT_MARGIN_CM = 5;
static const uint32_t ECHO_TIMEOUT_US =
  (uint32_t)(MAX_CM + TIMEOUT_MARGIN_CM) * 58U;

// ====== STABILITY ======
static const uint16_t SERVO_SETTLE_MS = 18;

// ====== SONAR PROCESSING (CONFIG) ======
enum AggregationMode : uint8_t { AGG_MEDIAN = 0, AGG_MEAN = 1 };

static const uint8_t SONAR_SAMPLES = 3;              // 1..9
static const AggregationMode AGGREGATION_MODE = AGG_MEDIAN;

// filtro anti-spike (per angolo) - opzionale
static const bool  ENABLE_SPIKE_FILTER = false;
static const float SPIKE_MAX_JUMP_CM   = 40.0f;

// ================= STATE =================
volatile int   g_angle = ANGLE_MIN;
volatile float g_dist_cm = NAN;

int dir = +1;
uint32_t nextTick = 0;

static float lastGoodByAngle[181];

// ================= UTILS =================
static inline float usToCm(uint32_t us){ return us/58.0f; }

float readDistanceRaw(){
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  uint32_t dur = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if(dur==0) return NAN;
  return usToCm(dur);
}

static float median3(float a,float b,float c){
  if(isnan(a)) return isnan(b)?c:b;
  if(isnan(b)) return isnan(a)?c:a;
  if(isnan(c)) return isnan(a)?b:a;
  if(a>b){float t=a;a=b;b=t;}
  if(b>c){float t=b;b=c;c=t;}
  if(a>b){float t=a;a=b;b=t;}
  return b;
}

static void sortFloat(float* v, int n){
  for(int i=0;i<n-1;i++){
    for(int j=i+1;j<n;j++){
      if(v[j] < v[i]){
        float t=v[i]; v[i]=v[j]; v[j]=t;
      }
    }
  }
}

float aggregateSamples(const float* samples, uint8_t n, AggregationMode mode){
  float vals[9];
  int m = 0;
  for(uint8_t i=0;i<n && i<9;i++){
    if(!isnan(samples[i])) vals[m++] = samples[i];
  }
  if(m == 0) return NAN;
  if(m == 1) return vals[0];

  if(mode == AGG_MEAN){
    float sum = 0.0f;
    for(int i=0;i<m;i++) sum += vals[i];
    return sum / (float)m;
  }

  // median
  if(m == 3) return median3(vals[0], vals[1], vals[2]);
  sortFloat(vals, m);
  if(m & 1) return vals[m/2];
  return 0.5f * (vals[m/2 - 1] + vals[m/2]);
}

float readDistanceProcessedForAngle(int angleDeg){
  float s[9];
  uint8_t n = SONAR_SAMPLES;
  if(n < 1) n = 1;
  if(n > 9) n = 9;

  for(uint8_t i=0;i<n;i++){
    s[i] = readDistanceRaw();
  }

  float cm = aggregateSamples(s, n, AGGREGATION_MODE);

#if DEBUG_SERIAL >= 2
  DBG2("a=%d samples:", angleDeg);
  for(uint8_t i=0;i<n;i++){
    if(isnan(s[i])) DBG2(" --");
    else DBG2(" %.1f", s[i]);
  }
  if(isnan(cm)) DBG2(" -> N/A\n");
  else DBG2(" -> %.1f\n", cm);
#endif

  // spike filter per-angolo
  if(!isnan(cm)){
    if(angleDeg < 0) angleDeg = 0;
    if(angleDeg > 180) angleDeg = 180;

    if(ENABLE_SPIKE_FILTER){
      float prev = lastGoodByAngle[angleDeg];
      if(!isnan(prev) && fabsf(cm - prev) > SPIKE_MAX_JUMP_CM){
        return NAN;
      }
    }
    lastGoodByAngle[angleDeg] = cm;
  }

  return cm;
}

// ================= WIFI INFO =================
String getModeString(){
  if(WiFi.status()==WL_CONNECTED) return "STA";
  WiFiMode_t m=WiFi.getMode();
  if(m==WIFI_AP||m==WIFI_AP_STA) return "AP";
  return "OFF";
}

String getIpString(){
  if(WiFi.status()==WL_CONNECTED)
    return WiFi.localIP().toString();
  return WiFi.softAPIP().toString();
}

// ================= WEB UI =================
// Placeholder {{SHOW_BEAM}} sostituito in handleRoot()
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>ESP8266 Radar</title>
<style>
  :root{
    --bg:#071018;
    --panel:rgba(10,18,28,.45);
    --fg:#d7e3ff;
  }
  body{
    margin:0;
    background:var(--bg);
    color:var(--fg);
    font-family:system-ui;
    overflow:hidden;
  }
  header{
    height:56px;
    padding:10px 14px;
    display:flex;
    justify-content:space-between;
    align-items:center;
    border-bottom:1px solid rgba(42,58,85,.6);
  }
  .left{display:flex;gap:10px; align-items:center;}
  .pill{
    padding:4px 10px;
    border:1px solid rgba(42,58,85,.9);
    border-radius:999px;
    font-size:12px;
    background:var(--panel);
    backdrop-filter: blur(2px);
  }
  #wrap{
    position:relative;
    width:100vw;
    height:calc(100vh - 56px);
  }
  canvas{
    width:100%;
    height:100%;
    display:block;
  }

  /* readout stile DOS/CRT */
  #readout{
    position:absolute;
    left:50%;
    top:38%;
    transform:translate(-50%,-50%);
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace;
    font-size: 34px;
    letter-spacing: 1px;
    color: rgba(160,255,200,.95);
    text-shadow:
      0 0 10px rgba(0,255,170,.35),
      0 0 22px rgba(0,255,170,.22);
    padding:10px 14px;
    border-radius:12px;
    background: rgba(4,10,14,.18);
    border: 1px solid rgba(0,255,170,.08);
    user-select:none;
    pointer-events:none;

    width: 320px;
    text-align:center;
    box-sizing:border-box;

    min-height: 86px;
    display:flex;
    flex-direction:column;
    justify-content:center;
  }
  #readout small{
    display:block;
    font-size:14px;
    letter-spacing:0.5px;
    opacity:.75;
    margin-top:6px;
    color: rgba(200,230,255,.65);
    text-shadow:none;
  }
</style>
</head>
<body>

<header>
  <div class="left">
    <div class="pill" id="cfg">MAX=— TO=—</div>
    <div class="pill" id="stat">a=— d=—</div>
  </div>
  <div class="pill" id="net">—</div>
</header>

<div id="wrap">
  <canvas id="c"></canvas>
  <div id="readout">—<small>—</small></div>
</div>

<script>
const SHOW_BEAM = {{SHOW_BEAM}};  // injected by firmware (0/1)

let MAX_CM=200;
let ECHO_TO=0;

const POLL_MS=70;

// Punti: decadono con age
const FADE_TICKS = 260;
const SWEEP_FADE_BOOST = 10;

// Fosforo (persistenza globale)
const PHOSPHOR_FADE=0.10;

// Sweep line (solo linea)
const SWEEP_LINE_ALPHA = 0.95;

// Bloom + “bombatura”
const BLOOM_ALPHA = 0.18;
const CURVE_ZOOM  = 1.03;
const CURVE_ALPHA = 0.18;
const VIGNETTE_ALPHA = 0.55;
const SCANLINE_ALPHA = 0.28;

const c=document.getElementById('c');
const ctx=c.getContext('2d');
const stat=document.getElementById('stat');
const net=document.getElementById('net');
const cfg=document.getElementById('cfg');
const readout=document.getElementById('readout');

function resize(){
  const dpr=window.devicePixelRatio||1;
  c.width=Math.floor(c.clientWidth*dpr);
  c.height=Math.floor(c.clientHeight*dpr);
}
window.addEventListener('resize',resize);
resize();

let dist=new Array(181).fill(null);
let age =new Array(181).fill(999999);
let last={a:0,d:-1,mode:"?",ip:"?",max_cm:MAX_CM,echo_to:0};

// phosphor buffer
const p=document.createElement('canvas');
const pctx=p.getContext('2d');

function ensureSize(){
  if(p.width!==c.width||p.height!==c.height){
    p.width=c.width;
    p.height=c.height;
    pctx.clearRect(0,0,p.width,p.height);
  }
}

function drawGrid(cx,cy,r){
  ctx.globalAlpha=.22;
  ctx.strokeStyle='rgba(106,166,255,1)';
  ctx.lineWidth=Math.max(1.0, c.width/1200);

  for(let i=.25;i<=1;i+=.25){
    ctx.beginPath();
    ctx.arc(cx,cy,r*i,Math.PI,2*Math.PI);
    ctx.stroke();
  }
  for(let deg=0;deg<=180;deg+=30){
    const a = Math.PI - (Math.PI*deg/180);
    ctx.beginPath();
    ctx.moveTo(cx,cy);
    ctx.lineTo(cx + r*Math.cos(a), cy - r*Math.sin(a));
    ctx.stroke();
  }
  ctx.globalAlpha=1;
}

// linea sweep disegnata sul CANVAS finale (sempre visibile)
function drawSweepLineOn(ctx3, cx, cy, r, angleDeg){
  const a = Math.PI - (Math.PI*angleDeg/180);

  ctx3.save();
  ctx3.globalCompositeOperation = 'screen';
  ctx3.globalAlpha = SWEEP_LINE_ALPHA;
  ctx3.strokeStyle = 'rgba(0,224,255,1)';
  ctx3.lineWidth = Math.max(2.0, c.width/1100);

  ctx3.beginPath();
  ctx3.moveTo(cx, cy);
  ctx3.lineTo(cx + r*Math.cos(a), cy - r*Math.sin(a));
  ctx3.stroke();

  // micro glow solo sulla linea
  ctx3.globalAlpha = SWEEP_LINE_ALPHA * 0.25;
  ctx3.filter = 'blur(2px)';
  ctx3.stroke();
  ctx3.filter = 'none';

  ctx3.restore();
}

function drawScanlines(w,h){
  ctx.save();
  ctx.globalCompositeOperation='overlay';
  ctx.globalAlpha=SCANLINE_ALPHA;
  ctx.fillStyle='rgba(255,255,255,1)';
  for(let y=0;y<h;y+=3){
    ctx.fillRect(0,y,w,1);
  }

  ctx.globalCompositeOperation='multiply';
  ctx.globalAlpha=SCANLINE_ALPHA*0.55;
  ctx.fillStyle='rgba(0,0,0,1)';
  for(let y=1;y<h;y+=6){
    ctx.fillRect(0,y,w,1);
  }
  ctx.restore();
}

function drawVignette(w,h){
  ctx.save();
  const g = ctx.createRadialGradient(w*0.5,h*0.55,Math.min(w,h)*0.12, w*0.5,h*0.55, Math.max(w,h)*0.65);
  g.addColorStop(0,'rgba(0,0,0,0)');
  g.addColorStop(1,`rgba(0,0,0,${VIGNETTE_ALPHA})`);
  ctx.fillStyle=g;
  ctx.fillRect(0,0,w,h);
  ctx.restore();
}

function drawCurvatureGlow(w,h){
  ctx.save();
  ctx.globalCompositeOperation='screen';
  ctx.globalAlpha=CURVE_ALPHA;
  ctx.filter='blur(10px)';
  ctx.translate(w*0.5,h*0.5);
  ctx.scale(CURVE_ZOOM,CURVE_ZOOM);
  ctx.translate(-w*0.5,-h*0.5);
  ctx.drawImage(p,0,0);
  ctx.restore();
  ctx.filter='none';
}

function drawBloom(w,h){
  ctx.save();
  ctx.globalCompositeOperation='screen';
  ctx.globalAlpha=BLOOM_ALPHA;
  ctx.filter='blur(6px)';
  ctx.drawImage(p,0,0);
  ctx.restore();
  ctx.filter='none';
}

function draw(){
  ensureSize();

  const w=c.width,h=c.height;
  const cx=w/2,cy=h*0.95;
  const r=Math.min(w*0.45,h*0.85);

  // fade fosforo (ghosting)
  pctx.fillStyle=`rgba(0,0,0,${PHOSPHOR_FADE})`;
  pctx.fillRect(0,0,w,h);

  // punti con decadimento
  for(let a=0;a<=180;a++){
    if(dist[a]==null) continue;
    const t=age[a];
    if(t>FADE_TICKS) continue;

    const alpha=1-(t/FADE_TICKS);
    const rr=(dist[a]/MAX_CM)*r;
    const ang=Math.PI-(Math.PI*a/180);
    const x=cx+rr*Math.cos(ang);
    const y=cy-rr*Math.sin(ang);

    pctx.globalAlpha = 0.10 + 0.90*alpha;
    pctx.fillStyle='rgba(0,255,140,1)';
    pctx.fillRect(x-2,y-2,4,4);

    pctx.globalAlpha = 0.06 + 0.12*alpha;
    pctx.fillStyle='rgba(0,255,140,1)';
    pctx.fillRect(x-6,y-6,12,12);

    age[a] = t + 1;
  }
  pctx.globalAlpha = 1;

  // frame base
  ctx.clearRect(0,0,w,h);

  const bg = ctx.createRadialGradient(w*0.5,h*0.65,Math.min(w,h)*0.05, w*0.5,h*0.65, Math.max(w,h)*0.9);
  bg.addColorStop(0,'rgba(10,24,32,1)');
  bg.addColorStop(1,'rgba(5,10,16,1)');
  ctx.fillStyle=bg;
  ctx.fillRect(0,0,w,h);

  drawGrid(cx,cy,r);

  // fosforo
  ctx.globalCompositeOperation='screen';
  ctx.drawImage(p,0,0);
  ctx.globalCompositeOperation='source-over';

  // bloom + bombatura
  drawBloom(w,h);
  drawCurvatureGlow(w,h);

  // sweep line SEMPRE SOPRA (se abilitata)
  if(SHOW_BEAM){
    drawSweepLineOn(ctx, cx, cy, r, last.a);
  }

  // scanlines + vignette per ultime
  drawScanlines(w,h);
  drawVignette(w,h);

  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

function setReadout(a, d){
  const angleStr = (a|0).toString().padStart(3,'0');   // 000..180

  let distCore;
  if(d < 0){
    distCore = "---.-";
  }else{
    distCore = d.toFixed(1).padStart(5,'0');           // 000.0..200.0
  }

  readout.innerHTML = `
    ${angleStr}°  ${distCore} cm
    <small>RADAR • ${MAX_CM}cm FS</small>
  `;
}

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const j=await r.json();

    const prevA = last.a|0;
    const nowA  = j.a|0;
    if(prevA > 160 && nowA < 20){
      if(SWEEP_FADE_BOOST > 0){
        for(let i=0;i<=180;i++){
          if(age[i] < 999999) age[i] += SWEEP_FADE_BOOST;
        }
      }
    }

    last=j;

    MAX_CM=j.max_cm;
    ECHO_TO=j.echo_to;

    const a=Math.max(0,Math.min(180,j.a|0));
    if(j.d>=0 && j.d<=MAX_CM){
      dist[a]=j.d;
      age[a]=0;
    }

    cfg.textContent=`MAX=${MAX_CM}cm TO=${ECHO_TO}us`;
    stat.textContent=`a=${j.a}° d=${j.d<0?'—':j.d.toFixed(1)+'cm'}`;
    net.textContent=`${j.mode} ${j.ip}`;
    setReadout(j.a|0, j.d);

  }catch(e){}
  setTimeout(tick,POLL_MS);
}
tick();
</script>

</body>
</html>
)HTML";

// ================= HTTP =================
void handleRoot(){
  String html = FPSTR(INDEX_HTML);
  html.replace("{{SHOW_BEAM}}", String((int)UI_SHOW_BEAM));
  server.send(200, "text/html", html);
}

void handleData(){
  String out="{";
  out+="\"a\":"+String(g_angle)+",";
  out+="\"d\":"+(isnan(g_dist_cm)?String(-1):String(g_dist_cm,1))+",";
  out+="\"mode\":\""+getModeString()+"\",";
  out+="\"ip\":\""+getIpString()+"\",";
  out+="\"max_cm\":"+String(MAX_CM)+",";
  out+="\"echo_to\":"+String(ECHO_TIMEOUT_US);
  out+="}";
  server.send(200,"application/json",out);
}

void startWeb(){
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
}

// ================= WIFI =================
bool connectSTA(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<12000){
    delay(250);
  }
  return WiFi.status()==WL_CONNECTED;
}

// ================= SETUP =================
void setup(){
  Serial.begin(115200);

  for(int i=0;i<=180;i++) lastGoodByAngle[i] = NAN;

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  myServo.attach(PIN_SERVO);
  myServo.write(g_angle);

  if(!connectSTA()){
    WiFi.mode(WIFI_AP);
    WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASS);
  }

  startWeb();
  nextTick = millis();

#if DEBUG_SERIAL >= 1
  DBG1("MAX_CM=%u TIMEOUT=%luus settle=%ums samples=%u mode=%s spike=%s jump=%.1f UI_SWEEP=%d\n",
       MAX_CM, (unsigned long)ECHO_TIMEOUT_US, (unsigned)SERVO_SETTLE_MS,
       (unsigned)SONAR_SAMPLES,
       (AGGREGATION_MODE==AGG_MEAN ? "MEAN" : "MEDIAN"),
       (ENABLE_SPIKE_FILTER ? "ON" : "OFF"),
       (double)SPIKE_MAX_JUMP_CM,
       (int)UI_SHOW_BEAM);
#endif
}

// ================= LOOP =================
void loop(){
  server.handleClient();

  uint32_t now=millis();
  if((int32_t)(now-nextTick)<0) return;
  nextTick=now+LOOP_PERIOD_MS;

  g_angle += dir*ANGLE_STEP;

  if(g_angle>=ANGLE_MAX){ g_angle=ANGLE_MAX; dir=-1; }
  if(g_angle<=ANGLE_MIN){ g_angle=ANGLE_MIN; dir=+1; }

  myServo.write(g_angle);
  delay(SERVO_SETTLE_MS);

  int a = g_angle;
  if(a < 0) a = 0;
  if(a > 180) a = 180;

  g_dist_cm = readDistanceProcessedForAngle(a);
}

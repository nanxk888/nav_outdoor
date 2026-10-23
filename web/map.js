// Offline OSM vector preview. Coordinates are longitude, latitude (WGS84).
const mapCanvas = document.getElementById('osm'), paint = mapCanvas.getContext('2d');
let features = [], bounds = null, gps = null, center = null, zoom = 1;
let online = false;
let pickMode=null, goal=null, heading=null, down=null, projection=null;
async function submit(kind,value){try{const r=await fetch('/api/operator/'+kind,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(value),signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error(r.status);await r.json();document.getElementById('operatorResult').textContent='已保存预览，未执行：'+JSON.stringify(value)+'。未接定位/导航；目标尚未审核道路可达性。'}catch(e){document.getElementById('operatorResult').textContent='保存失败或结果未知，请勿视为已执行：'+e.message}}
document.getElementById('pickGoal').onclick=()=>{pickMode='goal';document.getElementById('operatorResult').textContent='点击地图设置目标（仅预览）'};
document.getElementById('pickHeading').onclick=()=>{pickMode='heading';document.getElementById('operatorResult').textContent='从机头起点向朝向拖拽（仅取方向，不设置起始位置）'};
document.getElementById('saveHeading').onclick=()=>{const v=Number(document.getElementById('headingValue').value);if(Number.isFinite(v)&&v>=0&&v<360){heading=v;submit('heading',{heading_deg:v});draw()}};
function eventPoint(e){const r=mapCanvas.getBoundingClientRect();return [(e.clientX-r.left)*850/r.width,(e.clientY-r.top)*480/r.height]}
mapCanvas.addEventListener('pointerdown',e=>{down=eventPoint(e);mapCanvas.setPointerCapture(e.pointerId)});
mapCanvas.addEventListener('pointerup',e=>{if(!down||!projection)return;const end=eventPoint(e);
if(pickMode==='goal'){const [ox,oy,s]=projection;const lon=ox+(end[0]-425)/s;const my=oy-(end[1]-240)/s;const lat=(2*Math.atan(Math.exp(my*Math.PI/180))-Math.PI/2)*180/Math.PI;goal=[lon,lat];submit('goal',{longitude:lon,latitude:lat})}
if(pickMode==='heading'&&Math.hypot(end[0]-down[0],end[1]-down[1])>10){heading=(Math.atan2(end[0]-down[0],down[1]-end[1])*180/Math.PI+360)%360;document.getElementById('headingValue').value=heading.toFixed(1);submit('heading',{heading_deg:heading})}down=null;pickMode=null;draw()});
function xy(p) { return [p[0], Math.log(Math.tan(Math.PI / 4 + Math.max(-85, Math.min(85, p[1])) * Math.PI / 360)) * 180 / Math.PI]; }
function draw() {
  paint.clearRect(0, 0, 850, 480);
  if (!bounds) return;
  const origin = center || [(bounds[0]+bounds[2])/2,(bounds[1]+bounds[3])/2];
  const scale = Math.min(790 / Math.max(bounds[2]-bounds[0],1e-8), 420 / Math.max(bounds[3]-bounds[1],1e-8))*zoom;
  projection=[...origin,scale];
  const pixel = p => { const v=xy(p);return [425+(v[0]-origin[0])*scale,240-(v[1]-origin[1])*scale]; };
  for (const f of features) {
    paint.strokeStyle=f.properties.highway?'#a4cbd5':'#506978';paint.lineWidth=f.properties.highway?2:1;
    paint.beginPath();f.geometry.coordinates.forEach((p,i)=>{const v=pixel(p);i?paint.lineTo(...v):paint.moveTo(...v)});paint.stroke();
  }
  const current = online && gps && gps.receive_age_sec < 2 && gps.value.valid;
  if(goal){const p=pixel(goal);paint.strokeStyle='#ffbe55';paint.beginPath();paint.arc(...p,9,0,Math.PI*2);paint.stroke();paint.fillStyle='#ffbe55';paint.fillText('目标预览',p[0]+12,p[1])}
  if(heading!==null){const a=heading*Math.PI/180;paint.strokeStyle='#ffbe55';paint.beginPath();paint.moveTo(50,60);paint.lineTo(50+30*Math.sin(a),60-30*Math.cos(a));paint.stroke();paint.fillStyle='#ffbe55';paint.fillText('初始方向预览 '+heading.toFixed(1)+'°',15,110)}
  if(current){const p=pixel([gps.value.longitude,gps.value.latitude]);paint.fillStyle='#76ef78';paint.beginPath();paint.arc(...p,7,0,Math.PI*2);paint.fill();
    if(p[0]<0||p[0]>850||p[1]<0||p[1]>480)document.getElementById('mapState').textContent='GPS在当前视野外，点击定位GPS；本地OSM可能不覆盖该位置。';
  }
  if(!current)document.getElementById('mapState').textContent='无新鲜有效GPS定位；地图仅显示本地OSM，不绘制当前位置。';
}
window.updateGpsMap = (sample, connected=true) => {
  gps=sample;online=connected;
  document.getElementById('mapState').textContent=connected&&gps?.value.valid&&gps.receive_age_sec<2?'原始GPS天线位置（非融合位置）；道路不代表已审核通行许可。':'等待有效GPS';draw();
};
document.getElementById('mapHome').onclick=()=>{center=null;zoom=1;draw()};
document.getElementById('gpsHome').onclick=()=>{if(online&&gps?.value.valid&&gps.receive_age_sec<2){center=xy([gps.value.longitude,gps.value.latitude]);draw()}};
mapCanvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.25,Math.min(64,zoom*(e.deltaY<0?1.2:1/1.2)));draw()},{passive:false});
fetch('/api/osm').then(r=>{if(!r.ok)throw Error(r.status);return r.json()}).then(data=>{
  features=data.features;for(const f of features)for(const p of f.geometry.coordinates){const v=xy(p);if(!bounds)bounds=[...v,...v];else{bounds[0]=Math.min(bounds[0],v[0]);bounds[1]=Math.min(bounds[1],v[1]);bounds[2]=Math.max(bounds[2],v[0]);bounds[3]=Math.max(bounds[3],v[1])}}draw();
}).catch(e=>document.getElementById('mapState').textContent='OSM加载失败：'+e.message);
setInterval(()=>{if(gps)gps.receive_age_sec+=.5;draw()},500);

/* Offline OSM with explicit, display-only geographic alignment. */
(() => {
 const {Camera,project,unproject}=NavViewport;
 const $=id=>document.getElementById(id);
 const gpsCanvas=$('osm'),lioCanvas=$('plot');
 const cameras={gps:new Camera(.1),lio:new Camera(45)};
 let features=[],bounds=null,loadError='',gps=null,lio=null,navCloud=null,cloudStatus=null,online=false,receivedAt=0;
 let active='lio',mode='pan',goal=null,heading=null,follow=true,trajectory=[],lastStamp=null;
 let alignment=null,lastEpoch=null,lastFrame=null,geoFollow=true;
 function cloudReady(){const v=navCloud?.value;return goodLio()&&v?.valid&&age(navCloud)<1.5&&v.measurement_age_sec+age(navCloud)<1.5&&v.frame===lio.value.frame&&cloudStatus&&age(cloudStatus)<1.5&&cloudStatus.value?.ready===true}
 function geoReady(){return alignment&&goodLio()&&alignment.frame===lio.value.frame&&alignment.epoch===lio.value.epoch}
 const notify=(message,tone)=>window.NavUI?.notify(message,tone);
 const finitePoint=p=>Array.isArray(p)&&p.length>=2&&p.slice(0,2).every(Number.isFinite);
 function age(sample){return sample?sample.receive_age_sec+(performance.now()-receivedAt)/1000:Infinity}
 function goodGps(){return online&&age(gps)<2&&gps?.value.valid&&Number.isFinite(gps.value.longitude)&&Number.isFinite(gps.value.latitude)}
 function goodLio(){return online&&age(lio)<2&&finitePoint(lio?.value.position)}
 function context(canvas,camera){
  const r=canvas.getBoundingClientRect(),dpr=Math.min(2,window.devicePixelRatio||1);
  if(r.width<1||r.height<1)return null;
  if(camera.width>1&&(camera.width!==r.width||camera.height!==r.height))camera.scale*=Math.min(r.width/camera.width,r.height/camera.height);
  camera.width=r.width;camera.height=r.height;
  if(canvas.width!==Math.round(r.width*dpr)||canvas.height!==Math.round(r.height*dpr)){canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr)}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,r.width,r.height);ctx.lineJoin='round';return ctx;
 }
 function marker(ctx,p,color,r=5){ctx.fillStyle=color;ctx.beginPath();ctx.arc(p[0],p[1],r,0,Math.PI*2);ctx.fill();ctx.strokeStyle='#0b2027';ctx.lineWidth=2;ctx.stroke()}
 function drawGps(){
  const camera=cameras.gps,ctx=context(gpsCanvas,camera);if(!ctx)return;
  if(bounds&&!camera.fitted){camera.fit(bounds);camera.fitted=true}
  if(geoReady()&&geoFollow){const p=alignment.point(lio.value.position);camera.x=p[0];camera.y=p[1]}
  for(const f of features){ctx.lineWidth=f.road?1.6:1;ctx.strokeStyle=f.road?'#91b9c4':'#345463';ctx.beginPath();f.points.forEach((p,i)=>{const v=camera.screen(p);i?ctx.lineTo(...v):ctx.moveTo(...v)});ctx.stroke()}
  if(goal){const p=camera.screen(project(...goal));ctx.strokeStyle='#ffcb79';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(...p,5,0,Math.PI*2);ctx.moveTo(p[0]-9,p[1]);ctx.lineTo(p[0]+9,p[1]);ctx.moveTo(p[0],p[1]-9);ctx.lineTo(p[0],p[1]+9);ctx.stroke();if(active==='gps'){ctx.fillStyle='#ffcb79';ctx.font='12px system-ui';ctx.fillText('目标预览',p[0]+12,p[1]-10)}}
  if(goodGps())marker(ctx,camera.screen(project(gps.value.longitude,gps.value.latitude)),'#79ed96',6);
  if(geoReady()){
   ctx.strokeStyle='#6bdcc6';ctx.lineWidth=2;ctx.beginPath();trajectory.forEach((p,i)=>{const v=camera.screen(alignment.point(p));i?ctx.lineTo(...v):ctx.moveTo(...v)});ctx.stroke();
   if(cloudReady()){ctx.fillStyle='#ffb454';const size=Math.max(2,Math.min(5,.05*camera.scale/alignment.factor));for(const p of navCloud.value.points.slice(0,2000)){if(!finitePoint(p))continue;const v=camera.screen(alignment.point(p));ctx.fillRect(v[0]-size/2,v[1]-size/2,size,size)}}
   const p=camera.screen(alignment.point(lio.value.position));marker(ctx,p,'#5edee7',5);
   const yaw=alignment.yaw(lio.value.orientation);ctx.strokeStyle='#5edee7';ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(p[0]+25*Math.cos(yaw),p[1]-25*Math.sin(yaw));ctx.stroke();
  }
  $('alignmentState').textContent=geoReady()?`已地理配准（非持续融合）· 青色LIO / 绿色原始GPS / 橙色障碍 · ${goodGps()?'GPS有效':'GPS无效，仅相对推算'} · 锚点GPS轴向标准差 ${alignment.sigma===null?'未知':alignment.sigma.toFixed(1)+'m（上游估计）'} · 安装外参及OSM误差影响匹配精度`:'未对齐：有效GPS＋新鲜LIO＋实际初始方向，静止后点击对齐';
  $('mapState').textContent=loadError||(!online?'数据网关未连接':!gps?'尚未收到GPS数据':age(gps)>=2?'GPS数据已过期':!gps.value.valid?'当前无GPS信号，等待有效定位':'原始GPS天线位置 · 非融合位置');
 }
 function drawLio(){
  const camera=cameras.lio,ctx=context(lioCanvas,camera);if(!ctx)return;
  const position=finitePoint(lio?.value.position)?lio.value.position:null;
  if(follow&&position){camera.x=position[0];camera.y=position[1]}
  const target=70/camera.scale,power=10**Math.floor(Math.log10(target));
  const step=[1,2,5,10].map(n=>n*power).find(n=>n>=target)||power*10;
  const lo=camera.world([0,camera.height]),hi=camera.world([camera.width,0]);
  ctx.strokeStyle='#203944';ctx.lineWidth=1;
  for(let x=Math.ceil(lo[0]/step)*step;x<=hi[0];x+=step){const p=camera.screen([x,0]);ctx.beginPath();ctx.moveTo(p[0],0);ctx.lineTo(p[0],camera.height);ctx.stroke()}
  for(let y=Math.ceil(lo[1]/step)*step;y<=hi[1];y+=step){const p=camera.screen([0,y]);ctx.beginPath();ctx.moveTo(0,p[1]);ctx.lineTo(camera.width,p[1]);ctx.stroke()}
  const origin=camera.screen([0,0]);ctx.fillStyle='#688a99';ctx.font='11px system-ui';ctx.fillText('LIO起点',origin[0]+8,origin[1]-8);
  const cv=navCloud?.value;
  let reason='等待导航障碍点云';
  if(!online)reason='网关断开，障碍预览已隐藏';
  else if(cv){
   if(!cv.valid)reason='点云无效，已隐藏';
   else if(age(navCloud)>=1.5||cv.measurement_age_sec+age(navCloud)>=1.5)reason='点云已过期，已隐藏';
   else if(!goodLio())reason='LIO无新鲜位置，已隐藏障碍';
   else if(cv.frame!==lio.value.frame)reason='点云与LIO坐标系不一致，拒绝叠加';
   else if(!cloudStatus||age(cloudStatus)>=1.5||cloudStatus.value?.ready!==true)reason='地面过滤状态无效，已隐藏';
   else{
    ctx.fillStyle='#ffb454';const size=Math.max(2,Math.min(5,.05*camera.scale));
    for(const xyz of cv.points.slice(0,2000)){if(!finitePoint(xyz))continue;const p=camera.screen(xyz);ctx.fillRect(p[0]-size/2,p[1]-size/2,size,size)}
    reason=`橙色：实时障碍点 ${cv.points.length} / ${cv.input_points}（非Nav2代价地图）`;
   }
  }
  $('cloudHint').textContent=reason;
  ctx.strokeStyle=goodLio()?'#6bdcc6':'#637e86';ctx.lineWidth=2;ctx.beginPath();trajectory.forEach((p,i)=>{const v=camera.screen(p);i?ctx.lineTo(...v):ctx.moveTo(...v)});ctx.stroke();
  if(position){const p=camera.screen(position);marker(ctx,p,goodLio()?'#80eea4':'#718690',6);
   const q=lio.value.orientation;
   if(Array.isArray(q)&&q.length===4&&q.every(Number.isFinite)){const [x,y,z,w]=q,yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));ctx.strokeStyle='#a0eed8';ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(p[0]+24*Math.cos(yaw),p[1]-24*Math.sin(yaw));ctx.stroke()}}
  $('lioHome').textContent=follow?'LIO 跟随中':'跟随 LIO';
 }
 function draw(){drawGps();drawLio();const c=cameras[active],factor=active==='gps'?Math.cos(unproject(c.x,c.y)[1]*Math.PI/180):1;const m=100/c.scale*factor;$('scaleLabel').textContent=`100像素 ≈ ${m>=1?m.toFixed(1)+' m':(m*100).toFixed(1)+' cm'}${active==='gps'?' · 缩放不代表定位精度':''}`}
 function setMode(value){mode=value;$('gpsView').classList.toggle('selecting',value!=='pan');$('pickGoal').textContent=value==='goal'?'取消选点':'＋ 选择目标'}
 function show(view){active=view;for(const v of ['gps','lio']){$(v+'View').classList.toggle('is-main',v===view);$(v+'View').classList.toggle('is-mini',v!==view)}setMode('pan');$('swapView').textContent=view==='lio'?'切换全局地图':'切换LIO视图';requestAnimationFrame(draw)}
 async function submit(kind,value){try{const r=await fetch('/api/operator/'+kind,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(value),signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error(r.status);await r.json();$('operatorResult').textContent=kind==='goal'?'目标预览已保存，未下发导航；道路可达性尚未审核。':'方向预览已保存，未修改融合定位。';notify(kind==='goal'?'目标预览已保存，未启动导航':'初始方向预览已保存，未执行');return true}catch(e){$('operatorResult').textContent='保存失败或结果未知：'+e.message;notify('保存失败或结果未知，请重试','error');return false}}
 function point(canvas,e){const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top]}
 for(const [view,canvas] of [['gps',gpsCanvas],['lio',lioCanvas]]){
  let down=null,last=null,moved=false,mini=false;
  canvas.addEventListener('pointerdown',e=>{if(e.button!==0)return;mini=active!==view;down=point(canvas,e);last=down;moved=false;canvas.setPointerCapture(e.pointerId)});
  canvas.addEventListener('pointermove',e=>{if(!down)return;const p=point(canvas,e);if(Math.hypot(p[0]-down[0],p[1]-down[1])>5)moved=true;if(!mini&&moved&&!(view==='gps'&&mode==='heading')){cameras[view].pan(p[0]-last[0],p[1]-last[1]);if(view==='lio')follow=false;else geoFollow=false;draw()}last=p});
  canvas.addEventListener('pointerup',async e=>{if(!down)return;const p=point(canvas,e),start=down;down=null;
   if(mini){if(!moved)show(view);return}
   if(view==='gps'&&mode==='goal'&&!moved){const v=unproject(...cameras.gps.world(p));if(!bounds){notify('地图尚未就绪','error');return}const world=project(...v);if(world[0]<bounds[0]||world[0]>bounds[2]||world[1]<bounds[1]||world[1]>bounds[3]){notify('目标超出本地地图覆盖范围','error');return}goal=v;$('targetSummary').textContent='保存目标中…';draw();setMode('pan');if(await submit('goal',{longitude:v[0],latitude:v[1]}))$('targetSummary').textContent=`${v[1].toFixed(7)}° N, ${v[0].toFixed(7)}° E`;else $('targetSummary').textContent='目标未确认保存';}
   else if(view==='gps'&&mode==='heading'&&moved){heading=(Math.atan2(p[0]-start[0],start[1]-p[1])*180/Math.PI+360)%360;$('headingValue').value=heading.toFixed(1);$('headingSummary').textContent=`方向预览 ${heading.toFixed(1)}°`;setMode('pan');await submit('heading',{heading_deg:heading})}
  });
  canvas.addEventListener('pointercancel',()=>{down=null;last=null});
  canvas.addEventListener('wheel',e=>{e.preventDefault();if(active!==view)return;cameras[view].zoom(Math.exp(-Math.max(-200,Math.min(200,e.deltaY))*.003),point(canvas,e));draw()},{passive:false});
 }
 document.querySelectorAll('[data-view]').forEach(button=>button.onclick=()=>show(button.dataset.view));
 $('swapView').onclick=()=>show(active==='lio'?'gps':'lio');
 $('pickGoal').onclick=()=>{const cancel=active==='gps'&&mode==='goal';if(active!=='gps')show('gps');setMode(cancel?'pan':'goal');notify(cancel?'已取消选点':'放大后点击目标；拖动仍可平移地图')};
 $('pickHeading').onclick=()=>{show('gps');setMode('heading');notify('从机头位置向前拖拽，设置初始方向预览（不是位置初始化）')};
 $('saveHeading').onclick=async()=>{const raw=$('headingValue').value,v=Number(raw);if(!raw.trim()||!Number.isFinite(v)||v<0||v>=360){notify('方向请输入0到小于360的角度','error');return}if(await submit('heading',{heading_deg:v})){heading=v;$('headingSummary').textContent=`方向预览 ${v.toFixed(1)}°`}};
 $('zoomIn').onclick=()=>{cameras[active].zoom(1.4);draw()};$('zoomOut').onclick=()=>{cameras[active].zoom(1/1.4);draw()};
 $('alignGeo').onclick=()=>{try{
  if(!goodGps()||!goodLio())throw Error('需要新鲜有效GPS和LIO，当前不能对齐');
  const ga=Date.now()/1000-gps.value.stamp,la=Date.now()/1000-lio.value.stamp;
  if(!Number.isFinite(ga)||!Number.isFinite(la)||ga<-.1||la<-.1||ga>2||la>1)throw Error('测量时间已过期或电脑与Orin时钟不一致');
  const ids=['headingValue','imuHeadingOffset','antennaX','antennaY','antennaZ'];
  if(ids.some(id=>!$(id).value.trim()))throw Error('请填写实际初始方向与安装参数');
  const [h,o,x,y,z]=ids.map(id=>Number($(id).value));
  alignment=NavGeoAlignment.create(gps.value,lio.value,h,o,[x,y,z]);geoFollow=true;
  cameras.gps.scale=8*alignment.factor;show('gps');notify('已静止对齐，仅显示配准；未启用GPS持续融合或导航');
 }catch(e){notify(e.message,'error')}};
 $('clearAlignment').onclick=()=>{alignment=null;draw();notify('已清除地理对齐，不影响LIO或机器人控制')};
 $('followGeo').onclick=()=>{show('gps');geoFollow=true;draw();if(!geoReady())notify('请先完成有效的GPS＋LIO地理对齐','error')};
 for(const id of ['headingValue','imuHeadingOffset','antennaX','antennaY','antennaZ'])$(id).addEventListener('input',()=>{if(alignment){alignment=null;draw();notify('方向或安装参数已更改，请重新静止对齐')}});
 $('mapHome').onclick=()=>{geoFollow=false;show('gps');if(bounds){cameras.gps.fit(bounds);draw()}else notify('地图尚未加载','error')};
 $('gpsHome').onclick=()=>{geoFollow=false;show('gps');if(!online){notify('网关未连接，无法定位GPS','error');return}if(!gps){notify('尚未收到GPS数据，请检查驱动和桥接','error');return}if(age(gps)>=2){notify('GPS数据已过期，无法定位','error');return}if(!goodGps()){notify('当前无GPS信号，尚未获得有效经纬度','error');return}
  const p=project(gps.value.longitude,gps.value.latitude);cameras.gps.x=p[0];cameras.gps.y=p[1];cameras.gps.scale=Math.max(cameras.gps.scale,1.5);draw();notify('定位成功：已居中显示原始GPS位置（非融合定位）')};
 $('lioHome').onclick=()=>{show('lio');follow=true;draw();notify(goodLio()?'已跟随LIO相对位置':'LIO暂无新鲜数据，等待输入',goodLio()?'info':'error')};
 window.NavMaps={show,draw,update(response,connected){online=connected;gps=response?.samples?.gps||null;lio=response?.samples?.lio||null;navCloud=response?.samples?.nav_cloud||null;cloudStatus=response?.samples?.cloud_status||null;
   const reset=!connected||!lio||lio.value.epoch!==lastEpoch||lio.value.frame!==lastFrame||lio.value.stamp<lastStamp||(lastStamp!==null&&lio.value.stamp-lastStamp>1)||lio.receive_age_sec>=1;
   if(reset){trajectory=[];if(alignment){alignment=null;notify('LIO中断或坐标重置，旧地理对齐已取消，请静止重新对齐','error')}}
   lastEpoch=lio?.value.epoch;lastFrame=lio?.value.frame;
   if(cloudStatus&&typeof cloudStatus.value==='string'){try{cloudStatus={...cloudStatus,value:JSON.parse(cloudStatus.value)}}catch(e){cloudStatus=null}}
   receivedAt=performance.now();
   if(goodLio()&&lio.value.stamp!==lastStamp){if(lastStamp!==null&&lio.value.stamp<lastStamp){trajectory=[];follow=true;notify('LIO时间轴重置，已清除旧显示轨迹')}lastStamp=lio.value.stamp;trajectory.push(lio.value.position.slice(0,2));if(trajectory.length>2000)trajectory.shift()}draw()}};
 new ResizeObserver(draw).observe($('workspace'));new ResizeObserver(draw).observe($('gpsView'));new ResizeObserver(draw).observe($('lioView'));
 fetch('/api/osm').then(r=>{if(!r.ok)throw Error(r.status);return r.json()}).then(data=>{features=data.features.map(f=>({road:!!f.properties.highway,points:f.geometry.coordinates.map(p=>project(...p))}));for(const f of features)for(const p of f.points){if(!bounds)bounds=[...p,...p];else{bounds[0]=Math.min(bounds[0],p[0]);bounds[1]=Math.min(bounds[1],p[1]);bounds[2]=Math.max(bounds[2],p[0]);bounds[3]=Math.max(bounds[3],p[1])}}draw()}).catch(e=>{loadError='地图加载失败：'+e.message;notify(loadError,'error');draw()});
 setInterval(draw,500);
})();

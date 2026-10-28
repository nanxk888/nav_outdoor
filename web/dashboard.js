(() => {
 const $=id=>document.getElementById(id);let timer;
 function notify(message,tone='info'){log(message);const toast=$('toast');toast.textContent=message;toast.dataset.tone=tone;toast.hidden=false;clearTimeout(timer);timer=setTimeout(()=>toast.hidden=true,5000)}
 window.NavUI={notify};
 const logGroup=document.createElement('details');logGroup.className='parameter-group';logGroup.name='parameters';
 logGroup.innerHTML='<summary>导航日志</summary><section><button id="clearNavLog">清除日志</button><p>记录本页面操作与状态变化，最多保留2000条；不含页面关闭期间的事件。</p><pre id="navLog" style="white-space:pre-wrap;max-height:55vh;overflow:auto"></pre></section>';
 $('drawer').appendChild(logGroup);
 let logs=[];try{logs=JSON.parse(localStorage.getItem('nxkNavigationLog')||'[]');if(!Array.isArray(logs))logs=[]}catch(e){}
 function log(message){logs.push(new Date().toLocaleTimeString()+' '+message);logs=logs.slice(-2000);$('navLog').textContent=logs.join('\n');try{localStorage.setItem('nxkNavigationLog',JSON.stringify(logs))}catch(e){}}
 $('navLog').textContent=logs.join('\n');$('clearNavLog').onclick=()=>{logs=[];$('navLog').textContent='';try{localStorage.removeItem('nxkNavigationLog')}catch(e){}};
 document.addEventListener('click',e=>{const b=e.target.closest('button');if(b&&b.id!=='clearNavLog')log('操作：'+b.textContent.trim())},true);
 const seen={};
 function changed(key,value){if(value&&seen[key]!==value){seen[key]=value;log(key+'：'+value)}}
 function reason(text){const t=String(text||'');const rules=[[/LIO stale/,'里程计数据过期'],[/Ground.*stale/,'地面滤波数据过期'],[/Ground.*reject|Ground filtering unavailable/,'地面滤波不可用'],[/Observed.*stale/,'观察地图过期'],[/lease.*expired/,'执行心跳中断'],[/goal reached|SUCCEEDED/i,'到达目标'],[/Operator stop|operator stop/,'人工取消'],[/heartbeat timeout/,'连接或网页心跳超时'],[/SDK|FSM|Receiver fault/,'机器人模式或接收器异常'],[/discontinuity|reset|goal changed/i,'定位重置或目标改变'],[/speed parameters/,'速度参数未就绪']];for(const [r,c] of rules)if(r.test(t))return c+'（'+t+'）';return /[\u4e00-\u9fff]/.test(t)?t:'状态详情：'+t}
 window.NavUI.reason=reason;
 log('导航页面已打开');
 function drawer(open){$('drawer').hidden=!open;$('parameters').setAttribute('aria-expanded',String(open));if(open)$('closeDrawer').focus()}
 $('parameters').onclick=()=>drawer($('drawer').hidden);$('closeDrawer').onclick=()=>drawer(false);
 document.addEventListener('keydown',e=>{if(e.key==='Escape'){drawer(false);$('parameters').focus()}});
 $('fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await $('workspace').requestFullscreen()}catch(e){notify('浏览器不支持当前全屏操作','error')}};
 function pill(id,text,good){$(id).textContent=text;$(id).className='status-pill '+(good?'good':'warn')}
 async function poll(){try{const r=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error(r.status);const data=await r.json(),samples=data.samples||{};
  pill('connection','● 网关已连接',true);
  const g=samples.gps,l=samples.lio;
  pill('gpsBadge',!g?'GPS 未收到':g.receive_age_sec>=2?'GPS 已过期':g.value.valid?'GPS 有效定位':'GPS 无信号',!!g&&g.receive_age_sec<2&&g.value.valid);
  pill('lioBadge',!l?'LIO 未收到':l.receive_age_sec>=2?'LIO 已过期':'LIO 跟踪中',!!l&&l.receive_age_sec<2);
  for(const key of ['gps','gps_status','lio','lio_status']){const s=samples[key];let value=s?.value;if(typeof value==='string'){try{value=JSON.parse(value)}catch(e){/* retain raw string */}}$(key).textContent=s?`接收年龄 ${s.receive_age_sec.toFixed(2)} 秒\n`+JSON.stringify(value,null,2):'尚未收到该话题'}
  changed('连接','网页已连接');
  if(data.control)changed('控制',data.control.phase+' · '+reason(data.control.reason));
  try{let v=samples.local_execution?.value;if(typeof v==='string')v=JSON.parse(v);changed('导航输入',v?.input_reason?reason(v.input_reason):'正常')}catch(e){}
  for(const name of ['local_execution','cloud_status','lio_status','reference']){let v=samples[name]?.value;if(typeof v==='string'){try{v=JSON.parse(v)}catch(e){continue}}if(v)changed(name,(v.state||'')+' · '+reason(String(v.reason||'').replace(/nav=\d+ clear=\d+/g,'点云有效')));}
  window.updateControl(data.control);window.NavMaps.update(data,true);
 }catch(e){changed('连接','网页连接中断');pill('connection','● 网关已断开',false);pill('gpsBadge','GPS 状态未知',false);pill('lioBadge','LIO 状态未知',false);window.updateControl(null);window.NavMaps.update(null,false)}finally{setTimeout(poll,500)}}
 poll();
})();

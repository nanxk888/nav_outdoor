(() => {
 const $=id=>document.getElementById(id);let timer;
 function notify(message,tone='info'){const toast=$('toast');toast.textContent=message;toast.dataset.tone=tone;toast.hidden=false;clearTimeout(timer);timer=setTimeout(()=>toast.hidden=true,5000)}
 window.NavUI={notify};
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
  window.updateControl(data.control);window.NavMaps.update(data,true);
 }catch(e){pill('connection','● 网关已断开',false);pill('gpsBadge','GPS 状态未知',false);pill('lioBadge','LIO 状态未知',false);window.updateControl(null);window.NavMaps.update(null,false)}finally{setTimeout(poll,500)}}
 poll();
})();

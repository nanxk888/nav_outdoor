let heartbeatEnabled=false, pendingHeartbeat=false;
const field=id=>document.getElementById(id);
async function command(kind){const token=field('controlToken').value;
 const r=await fetch('/api/control/'+kind,{method:'POST',headers:{'Content-Type':'application/json',Authorization:'Bearer '+token},body:'{}',signal:AbortSignal.timeout(1500)});
 if(!r.ok)throw Error(await r.text());return r.json();}
for(const [id,kind] of [['robotStart','start'],['robotNavigate','navigate'],['robotStop','stop']]){
 field(id).onclick=async()=>{if(kind!=='stop'&&!confirm(kind==='start'?'确认现场安全、急停可用、SDK模式和接收端已授权？将使机器人站立。':'确认目标预览并请求真实导航？'))return;
 try{const r=await command(kind);heartbeatEnabled=kind!=='stop';field('controlResult').textContent='请求已排队，不代表执行成功：'+JSON.stringify(r)}catch(e){field('controlResult').textContent='请求失败或结果未知：'+e.message}};
}
window.updateControl=s=>{field('controlState').textContent=s?JSON.stringify(s,null,2):'网关断开，机器人停止状态未知，请确认硬件安全';
 field('robotStart').disabled=!s?.enabled||!['IDLE','STOPPED'].includes(s.phase);
 field('robotNavigate').disabled=!s?.enabled||s.phase!=='READY'||!s.navigation_safe;
 if(s&&['IDLE','STOPPED','DISABLED'].includes(s.phase))heartbeatEnabled=false;};
setInterval(async()=>{if(!heartbeatEnabled||pendingHeartbeat)return;pendingHeartbeat=true;try{await command('heartbeat')}catch(e){field('controlResult').textContent='心跳失败，请检查机器人是否停止'}finally{pendingHeartbeat=false}},500);

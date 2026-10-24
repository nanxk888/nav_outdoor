let heartbeatEnabled=false,pendingHeartbeat=false;
const field=id=>document.getElementById(id);
const controlButtons=['robotStart','robotLoco','robotLower','robotStop','robotPose','robotNavigate'];
async function command(kind,body={}){
 const r=await fetch('/api/control/'+kind,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:AbortSignal.timeout(1500)});
 if(!r.ok)throw Error(await r.text());return r.json();
}
for(const [id,kind] of [['robotStart','start'],['robotLoco','loco'],['robotLower','lower'],['robotStop','stop'],['robotNavigate','navigate']]){
 field(id).onclick=async()=>{
  try{
   const body=['loco','lower'].includes(kind)?{confirm_stationary:true}:{};
   const r=await command(kind,body);heartbeatEnabled=kind!=='stop';
   field('controlResult').textContent='请求已排队，非执行成功：'+JSON.stringify(r);
  }catch(e){field('controlResult').textContent='请求被拒绝或结果未知：'+e.message}
 };
}
field('robotPose').onclick=async()=>{
 const roll=field('rollValue').value,pitch=field('pitchValue').value,r=Number(roll),p=Number(pitch);
 if(!roll.trim()||!pitch.trim()||!Number.isFinite(r)||!Number.isFinite(p)||Math.abs(r)>11.4||Math.abs(p)>22.9){field('controlResult').textContent='姿态角为空或越界';return}
 try{await command('pose',{roll_rad:r*Math.PI/180,pitch_rad:p*Math.PI/180});heartbeatEnabled=true;field('controlResult').textContent='姿态请求已排队，不代表实际姿态已到位。'}
 catch(e){field('controlResult').textContent='请求被拒绝或结果未知：'+e.message}
};
window.updateControl=s=>{
 field('controlState').textContent=s?JSON.stringify(s,null,2):'网关断开，实际机器人状态未知';
 const batteryAge=s?.battery_receive_age_sec;
 field('robotBattery').textContent=['battery1','battery2'].map((name,i)=>{
  const b=s?.batteries?.[name];
  if(!s)return `电池${i+1}：连接中断`;
  if(!b)return `电池${i+1}：未收到`;
  const stale=b.stale||batteryAge==null||batteryAge>90||(b.age_sec||0)+batteryAge>90||(b.source_age_sec!=null&&b.source_age_sec+(b.age_sec||0)+batteryAge>90);
  return `电池${i+1}：${b.percent==null?'电量未知':b.percent+'%'}${b.voltage==null?'':' / '+b.voltage+'V'}${stale?'（过期）':''}`;
 }).join(' · ')+'（每60秒获取）';
 const robot=s?.robot,fsm=robot?.fsm||'未知';
 const valid=!!s&&s.robot_receive_age_sec<.3&&robot?.fsm_valid===true;
 field('robotFsm').textContent=`机器人FSM：${fsm}（${valid?'状态校验有效':'未验证/已过期，请勿视作当前状态'}）`;
 field('controlFlow').textContent=s?`控制流程：${s.phase}；${s.reason||''}`:'网关断开；软件停止状态未知，请检查硬件';
 field('controlBrief').textContent=`FSM：${fsm} · ${s?.phase||'连接中断'}`;
 // Operator can always submit an intent. Hardware authorization, session state,
 // standstill, numeric limits and navigation safety remain server-side checks.
 for(const id of controlButtons)field(id).disabled=false;
 if(s&&['IDLE','STOPPED','DISABLED'].includes(s.phase))heartbeatEnabled=false;
};
for(const id of controlButtons)field(id).disabled=false;
setInterval(async()=>{
 if(!heartbeatEnabled||pendingHeartbeat)return;pendingHeartbeat=true;
 try{await command('heartbeat')}catch(e){field('controlResult').textContent='心跳失败，请检查实际机器人和硬件急停'}finally{pendingHeartbeat=false}
},500);

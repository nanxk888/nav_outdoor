// Fake DOM and HTTP only. Never contacts a gateway or a robot.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements=new Map(),posts=[];
function el(id){if(!elements.has(id))elements.set(id,{value:'0',disabled:true,textContent:''});return elements.get(id)}
const context={document:{getElementById:el},window:{},setInterval:()=>{},
 AbortSignal:{timeout:()=>({})},confirm:()=>{throw Error('Confirmation popup forbidden')},
 fetch:async(url,options)=>{posts.push([url,JSON.parse(options.body)]);return {ok:true,json:async()=>({queued:true})}}};
vm.createContext(context);vm.runInContext(fs.readFileSync('web/control.js','utf8'),context);
(async()=>{
 const ids=['robotStart','robotLoco','robotLower','robotStop','robotPose','robotNavigate','robotLocalNavigate'];
 for(const phase of ['DISABLED','IDLE','STANDING','STAND_READY','READY','NAVIGATING','STOPPED']){
  context.window.updateControl({phase,enabled:false,reason:'fixture',robot_receive_age_sec:0,robot:{fsm:'loco',fsm_valid:true}});
  for(const id of ids)assert.equal(el(id).disabled,false,phase+': '+id);
 }
 assert(el('robotFsm').textContent.includes('loco'));
 context.window.updateControl({phase:'IDLE',battery_receive_age_sec:1,batteries:{battery1:{percent:72,voltage:48,age_sec:0,stale:false}},robot:{}});
 assert(el('robotBattery').textContent.includes('72%'));
 assert(el('robotBattery').textContent.includes('电池2：未收到'));
 context.window.updateControl({phase:'IDLE',battery_receive_age_sec:100,batteries:{battery1:{percent:72,age_sec:0}},robot:{}});
 assert(el('robotBattery').textContent.includes('过期'));
 for(const id of ids)await el(id).onclick();
 assert.deepEqual(posts.map(p=>p[0]),['start','loco','lower','stop','pose','navigate','lio_navigate'].map(k=>'/api/control/'+k));
 assert.equal(posts.length,7,'one request per click');
 el('navLinearSpeed').value='0.08';el('navAngularSpeed').value='0.2';
 await el('applyNavSpeed').onclick();
 assert.equal(posts[7][0],'/api/control/speed_limits');
 assert.deepEqual(posts[7][1],{linear_mps:.08,angular_rps:.2});
 el('navLinearSpeed').value='0.5';await el('applyNavSpeed').onclick();
 assert.equal(posts.length,8,'invalid speed must not be submitted');
 context.window.updateControl({phase:'IDLE',speed_limits:{linear_mps:.08,angular_rps:.2}});
 assert(el('navSpeedState').textContent.includes('0.08'));
 context.window.updateControl(null);assert(el('robotFsm').textContent.includes('未知'));
 context.window.updateControl({phase:'STOPPED',robot_receive_age_sec:3,robot:{fsm:'idle',fsm_valid:true}});
 assert(el('robotFsm').textContent.includes('已过期'));
 console.log('PASS: all control buttons clickable, no confirm popup, one POST per click, FSM and stale-state display');
})().catch(e=>{console.error(e);process.exitCode=1});

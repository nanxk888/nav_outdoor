// Fake DOM and HTTP only. Never contacts a gateway or a robot.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements=new Map(),posts=[];
function el(id){if(!elements.has(id))elements.set(id,{value:'0',disabled:true,textContent:''});return elements.get(id)}
const context={document:{getElementById:el},window:{},setInterval:()=>{},
 AbortSignal:{timeout:()=>({})},confirm:()=>{throw Error('Confirmation popup forbidden')},
 fetch:async(url,options)=>{posts.push([url,JSON.parse(options.body)]);return {ok:true,json:async()=>({queued:true})}}};
vm.createContext(context);vm.runInContext(fs.readFileSync('web/control.js','utf8'),context);
(async()=>{
 const ids=['robotStart','robotLoco','robotLower','robotStop','robotPose','robotNavigate'];
 for(const phase of ['DISABLED','IDLE','STANDING','STAND_READY','READY','NAVIGATING','STOPPED']){
  context.window.updateControl({phase,enabled:false,reason:'fixture',robot_receive_age_sec:0,robot:{fsm:'loco',fsm_valid:true}});
  for(const id of ids)assert.equal(el(id).disabled,false,phase+': '+id);
 }
 assert(el('robotFsm').textContent.includes('loco'));
 for(const id of ids)await el(id).onclick();
 assert.deepEqual(posts.map(p=>p[0]),['start','loco','lower','stop','pose','navigate'].map(k=>'/api/control/'+k));
 assert.equal(posts.length,6,'one request per click');
 context.window.updateControl(null);assert(el('robotFsm').textContent.includes('未知'));
 context.window.updateControl({phase:'STOPPED',robot_receive_age_sec:3,robot:{fsm:'idle',fsm_valid:true}});
 assert(el('robotFsm').textContent.includes('已过期'));
 console.log('PASS: all control buttons clickable, no confirm popup, one POST per click, FSM and stale-state display');
})().catch(e=>{console.error(e);process.exitCode=1});

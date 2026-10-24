// Application interaction unit test in a fake DOM; not browser visual QA.
const assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs'),path=require('node:path');
const messages=[],posts=[],els=new Map();
class Element{
 constructor(id){this.id=id;this.textContent='';this.value='';this.handlers={};this.flags=new Set(id==='lioView'?['is-main']:id==='gpsView'?['is-mini']:[]);this.classList={toggle:(key,on)=>on?this.flags.add(key):this.flags.delete(key)}}
 addEventListener(name,fn){this.handlers[name]=fn}
 setPointerCapture(){}
 getBoundingClientRect(){const main=els.get(this.id==='osm'?'gpsView':'lioView')?.flags.has('is-main');return {left:0,top:0,width:main?1200:300,height:main?800:205}}
 getContext(){return new Proxy({}, {get:(_,key)=>(...args)=>{if(['arc','moveTo','lineTo'].includes(key))assert(args.every(Number.isFinite))}})}
}
const $=id=>{if(!els.has(id))els.set(id,new Element(id));return els.get(id)};
$('lioView');$('gpsView');
const context={console,NavViewport:require('../web/viewport.js'),document:{getElementById:$,querySelectorAll:()=>[]},performance:{now:()=>0},ResizeObserver:class{observe(){}},requestAnimationFrame:fn=>fn(),setInterval:()=>0,AbortSignal:{timeout:()=>({})},Math,Number,Array,Promise};
context.window=context;context.NavUI={notify:(text,tone)=>messages.push([text,tone])};
context.fetch=async(url,options)=>{if(options?.method==='POST'){posts.push([url,JSON.parse(options.body)]);return {ok:true,json:async()=>({saved:true})}}return {ok:true,json:async()=>({features:[{properties:{highway:'footway'},geometry:{coordinates:[[108.82,34.12],[108.84,34.14]]}}]})}};
vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/map.js'),'utf8'),context);
const event=(x,y)=>({button:0,pointerId:1,clientX:x,clientY:y,preventDefault(){}});
async function click(x,y){$('osm').handlers.pointerdown(event(x,y));await $('osm').handlers.pointerup(event(x,y))}
(async()=>{
 await new Promise(r=>setImmediate(r));
 context.NavMaps.update({samples:{gps:{receive_age_sec:0,value:{valid:false}}}},true);
 $('gpsHome').onclick();assert(messages.at(-1)[0].includes('无GPS信号'));
 context.NavMaps.update({samples:{gps:{receive_age_sec:3,value:{valid:true,longitude:108.83,latitude:34.13}}}},true);
 $('gpsHome').onclick();assert(messages.at(-1)[0].includes('过期'));
 context.NavMaps.update({samples:{gps:{receive_age_sec:0,value:{valid:true,longitude:108.83,latitude:34.13}}}},true);
 $('gpsHome').onclick();assert(messages.at(-1)[0].includes('定位成功'));
 $('mapHome').onclick();$('pickGoal').onclick();await click(600,400);assert.equal(posts.length,1);assert.equal(posts[0][0],'/api/operator/goal');
 const first=posts[0][1];$('pickGoal').onclick();$('osm').handlers.pointerdown(event(600,400));$('osm').handlers.pointermove(event(800,400));await $('osm').handlers.pointerup(event(800,400));assert.equal(posts.length,1,'drag must not submit a target');
 await click(600,400);assert.equal(posts.length,2);assert(Math.abs(posts[1][1].longitude-first.longitude)>1e-8,'pan must change clicked world coordinates');
 $('headingValue').value='';await $('saveHeading').onclick();assert.equal(posts.length,2,'blank heading must not silently become north');
 $('headingValue').value='90';await $('saveHeading').onclick();assert.equal(posts.at(-1)[1].heading_deg,90);
 $('swapView').onclick();assert($('lioView').flags.has('is-main'));assert($('gpsView').flags.has('is-mini'));
 context.NavMaps.update(null,false);$('gpsHome').onclick();assert(messages.at(-1)[0].includes('未连接'));
 assert(!posts.some(p=>p[0].startsWith('/api/control')),'map actions must never start motion');
 console.log('PASS: GPS success/no-fix/stale/disconnect feedback; target selection; drag suppression; heading validation; main/mini swap; no motion requests');
})().catch(e=>{console.error(e);process.exitCode=1});

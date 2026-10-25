// Application interaction unit test in a fake DOM; not browser visual QA.
const assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs'),path=require('node:path');
const messages=[],posts=[],els=new Map();let obstacleDraws=0;
class Element{
 constructor(id){this.id=id;this.textContent='';this.value='';this.handlers={};this.flags=new Set(id==='lioView'?['is-main']:id==='gpsView'?['is-mini']:[]);this.classList={toggle:(key,on)=>on?this.flags.add(key):this.flags.delete(key)}}
 addEventListener(name,fn){this.handlers[name]=fn}
 setPointerCapture(){}
 getBoundingClientRect(){const main=els.get(this.id==='osm'?'gpsView':'lioView')?.flags.has('is-main');return {left:0,top:0,width:main?1200:300,height:main?800:205}}
 getContext(){return new Proxy({}, {get:(_,key)=>(...args)=>{if(key==='fillRect')obstacleDraws++;if(['arc','moveTo','lineTo','fillRect'].includes(key))assert(args.every(Number.isFinite))}})}
}
const $=id=>{if(!els.has(id))els.set(id,new Element(id));return els.get(id)};
$('lioView');$('gpsView');
const context={console,NavViewport:require('../web/viewport.js'),NavGeoAlignment:require('../web/geo_alignment.js'),document:{getElementById:$,querySelectorAll:()=>[]},performance:{now:()=>0},ResizeObserver:class{observe(){}},requestAnimationFrame:fn=>fn(),setInterval:()=>0,AbortSignal:{timeout:()=>({})},Math,Number,Array,Promise};
context.window=context;context.NavUI={notify:(text,tone)=>messages.push([text,tone])};
context.fetch=async(url,options)=>{if(options?.method==='POST'){posts.push([url,JSON.parse(options.body)]);return {ok:true,json:async()=>({saved:true})}}return {ok:true,json:async()=>({features:[{properties:{highway:'footway'},geometry:{coordinates:[[108.82,34.12],[108.84,34.14]]}}]})}};
vm.createContext(context);vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/map.js'),'utf8'),context);
const event=(x,y)=>({button:0,pointerId:1,clientX:x,clientY:y,preventDefault(){}});
async function click(x,y){$('osm').handlers.pointerdown(event(x,y));await $('osm').handlers.pointerup(event(x,y))}
(async()=>{
 await new Promise(r=>setImmediate(r));
 const scan={lio:{receive_age_sec:0,value:{position:[0,0,0],frame:'lio_odom',stamp:1}},
  nav_cloud:{receive_age_sec:0,value:{valid:true,frame:'lio_odom',measurement_age_sec:0,points:[[1,2,.1]],input_points:1}},
  cloud_status:{receive_age_sec:0,value:JSON.stringify({ready:true})}};
 context.NavMaps.update({samples:scan},true);assert.equal(obstacleDraws,1);
 scan.nav_cloud.value.frame='wrong';context.NavMaps.update({samples:scan},true);assert.equal(obstacleDraws,1);assert($('cloudHint').textContent.includes('坐标系'));
 scan.nav_cloud.value.frame='lio_odom';scan.nav_cloud.receive_age_sec=3;context.NavMaps.update({samples:scan},true);assert.equal(obstacleDraws,1);assert($('cloudHint').textContent.includes('过期'));
 scan.nav_cloud.receive_age_sec=0;scan.cloud_status.value=JSON.stringify({ready:false});context.NavMaps.update({samples:scan},true);assert.equal(obstacleDraws,1);
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
 const now=Date.now()/1000;
 scan.lio.value={position:[0,0,0],orientation:[0,0,0,1],frame:'lio_odom',epoch:'one',stamp:now};
 scan.gps={receive_age_sec:0,value:{valid:true,longitude:108.83,latitude:34.13,stamp:now}};
 scan.cloud_status.value=JSON.stringify({ready:true});
 context.NavMaps.update({samples:scan},true);
 for(const id of ['imuHeadingOffset','antennaX','antennaY','antennaZ'])$(id).value='0';
 $('headingValue').value='90';await $('alignGeo').onclick();
 assert.equal(posts.at(-1)[0],'/api/operator/register');assert.equal(posts.at(-1)[1].heading_deg,90);
 assert($('gpsView').flags.has('is-main'));assert($('alignmentState').textContent.includes('已地理配准'));
 const draws=obstacleDraws;context.NavMaps.draw();assert(obstacleDraws>=draws+2,'both geographic and local view draw the same cloud');
 scan.gps.value.valid=false;context.NavMaps.update({samples:scan},true);
 assert($('alignmentState').textContent.includes('仅相对推算'),'GPS loss must be labelled, not silently fused');
 const beforeStale=obstacleDraws;scan.nav_cloud.receive_age_sec=3;context.NavMaps.update({samples:scan},true);
 assert.equal(obstacleDraws,beforeStale,'stale obstacles hidden in both views');
 scan.lio.value.epoch='two';context.NavMaps.update({samples:scan},true);
 assert($('alignmentState').textContent.includes('未对齐'),'new LIO epoch cancels alignment');
 scan.memory={receive_age_sec:0,value:JSON.stringify({ready:true,frame:'lio_odom',epoch:'memory-test',source_stamp:now,
  resolution_m:.25,cells:[[0,0,0],[2,0,100]],cell_count:2,retention_sec:120,route:{points:[[0,0],[1,0]],partial:true},reason:'test'})};
 context.NavMaps.update({samples:scan},true);assert($('memoryHint').textContent.includes('观察记忆'));
 $('pickLioGoal').onclick();$('plot').handlers.pointerdown(event(600,400));await $('plot').handlers.pointerup(event(600,400));
 assert.equal(posts.at(-1)[0],'/api/operator/lio_goal');assert.equal(posts.at(-1)[1].epoch,'memory-test');
 assert.equal(scan.gps.value.valid,false,'GPS-free local goal must work without valid GPS');
 await $('resetMemory').onclick();assert.equal(posts.at(-1)[0],'/api/operator/memory_reset');
 assert(messages.at(-1)[0].includes('里程计不重启'));
 assert(!posts.some(p=>p[0].startsWith('/api/control')),'map actions must never start motion');
 console.log('PASS: GPS success/no-fix/stale/disconnect feedback; target selection; drag suppression; heading validation; main/mini swap; no motion requests');
})().catch(e=>{console.error(e);process.exitCode=1});

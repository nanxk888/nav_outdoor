const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const elements=new Map(),store=new Map(),handlers={};
const el=id=>{if(!elements.has(id))elements.set(id,{textContent:'',dataset:{},setAttribute(){},focus(){},appendChild(){}});return elements.get(id)};
const context={console,Date,JSON,String,Array,clearTimeout(){},setTimeout(){},AbortSignal:{timeout(){}},
 document:{getElementById:el,createElement:()=>({}),addEventListener:(k,f)=>handlers[k]=f},
 localStorage:{getItem:k=>store.get(k),setItem:(k,v)=>store.set(k,v),removeItem:k=>store.delete(k)},
 fetch:async()=>({ok:true,json:async()=>({samples:{},control:{phase:'STOPPED',reason:'LIO stale'}})})};
context.window=context;context.updateControl=()=>{};context.NavMaps={update(){}};
vm.createContext(context);vm.runInContext(fs.readFileSync('web/dashboard.js','utf8'),context);
setImmediate(()=>{
 assert(el('navLog').textContent.includes('里程计数据过期'));
 handlers.click({target:{closest:()=>({id:'swapView',textContent:'切换全局地图'})}});
 assert(el('navLog').textContent.includes('切换全局地图'));
 context.NavUI.notify('导航启动请求');assert(el('navLog').textContent.includes('导航启动请求'));
 assert(store.has('nxkNavigationLog'));el('clearNavLog').onclick();
 assert.equal(el('navLog').textContent,'');assert(!store.has('nxkNavigationLog'));
 console.log('PASS: Chinese navigation reasons, map actions, notifications, persistence and manual clear');
});

// Pure camera/projection tests. No browser, ROS or robot connection.
const assert=require('node:assert/strict');
const {Camera,project,unproject}=require('../web/viewport.js');
for(const p of [[108.829,34.125],[-73.9,40.7],[0,0],[120,-35]]){const q=unproject(...project(...p));assert(Math.abs(p[0]-q[0])<1e-8);assert(Math.abs(p[1]-q[1])<1e-8)}
const c=new Camera(20);c.width=1200;c.height=800;c.x=10;c.y=-20;
const anchor=[120,190],before=c.world(anchor);c.zoom(5,anchor);assert(Math.hypot(...c.world(anchor).map((n,i)=>n-before[i]))<1e-9);
const p=c.screen([12,-18]);c.pan(100,-20);const q=c.screen([12,-18]);assert(Math.abs(q[0]-p[0]-100)<1e-8);assert(Math.abs(q[1]-p[1]+20)<1e-8);
for(let i=0;i<100;i++)c.zoom(10);assert(c.scale<=512);for(let i=0;i<100;i++)c.zoom(.1);assert(c.scale>=.00001);
c.fit([0,0,100,100]);assert(c.screen([0,0])[0]>=0);assert(c.screen([100,100])[0]<=1200);
console.log('PASS: WGS84 projection, anchored zoom, drag direction, zoom limits, fit bounds');
for(const yaw of [0,Math.PI,-Math.PI/2,.7]){
 const r=new Camera(30);r.width=900;r.height=600;r.rotation=Math.PI/2-yaw;
 const tip=r.screen([Math.cos(yaw),Math.sin(yaw)]);
 assert(Math.abs(tip[0]-450)<1e-8&&tip[1]<300,'initial nose must point up');
 const point=[-2,3],pixel=r.screen(point),back=r.world(pixel);
 assert(Math.hypot(back[0]-point[0],back[1]-point[1])<1e-9);
 r.pan(50,-20);const moved=r.screen(point);assert(Math.abs(moved[0]-pixel[0]-50)<1e-8&&Math.abs(moved[1]-pixel[1]+20)<1e-8);
 const cursor=[130,170],world=r.world(cursor);r.zoom(2,cursor);assert(Math.hypot(...r.world(cursor).map((v,i)=>v-world[i]))<1e-9);
}
console.log('PASS: initial-heading-up view, rotated inverse target picking, pan and cursor zoom');

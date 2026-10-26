/* Display-only rigid LIO -> local ENU -> Web Mercator. No navigation writes. */
(function(root){
 const {project}=root.NavViewport||(typeof require==='function'?require('./viewport.js'):{}),rad=Math.PI/180;
 function horizontalHeading(q){
  if(!Array.isArray(q)||q.length!==4||!q.every(Number.isFinite))return null;
  const norm=Math.hypot(...q);if(Math.abs(norm-1)>.01)return null;
  const [x,y,z,w]=q.map(v=>v/norm),a=1-2*(y*y+z*z),b=2*(x*y+w*z);
  return Math.hypot(a,b)<.3?null:Math.atan2(b,a);
 }
 function bodyHeading(q){const a=horizontalHeading(q);return a===null?null:Math.atan2(-Math.sin(a),-Math.cos(a))}
 function create(gps,lio,heading,offset=0,lever=[0,0,0]){
  if(!gps.valid||![gps.longitude,gps.latitude,gps.stamp,lio.stamp,heading,offset,...lever,...lio.position,...lio.orientation].every(Number.isFinite))throw Error('需要有效GPS、LIO位姿和数值参数');
  if(Math.abs(gps.latitude)>80||Math.abs(gps.longitude)>180||heading<0||heading>=360||Math.abs(offset)>360||lever.some(v=>Math.abs(v)>5))throw Error('对齐参数超出范围');
  if(Math.abs(gps.stamp-lio.stamp)>.3)throw Error('GPS与LIO测量时间差超过0.3秒，请静止等待');
  let [x,y,z,w]=lio.orientation;const norm=Math.hypot(x,y,z,w);
  if(Math.abs(norm-1)>.01)throw Error('LIO四元数无效');[x,y,z,w]=[x,y,z,w].map(v=>v/norm);
  const a=1-2*(y*y+z*z),b=2*(x*y+w*z);
  if(Math.hypot(a,b)<.3)throw Error('IMU +X接近竖直，无法可靠设置平面航向');
  const rotation=Math.PI/2-(heading+offset)*rad-bodyHeading(lio.orientation),c=Math.cos(rotation),s=Math.sin(rotation);
  // Antenna lever is expressed in IMU axes; apply current full attitude before planar alignment.
  const lx=a*lever[0]+2*(x*y-w*z)*lever[1]+2*(x*z+w*y)*(lever[2]||0),ly=b*lever[0]+(1-2*(x*x+z*z))*lever[1]+2*(y*z-w*x)*(lever[2]||0);
  const base=project(gps.longitude,gps.latitude),factor=Math.cos(gps.latitude*rad);
  const origin=[lio.position[0]+lx,lio.position[1]+ly];
  const cv=gps.covariance;
  const sigma=gps.covariance_type>0&&Array.isArray(cv)&&[cv[0],cv[4]].every(v=>Number.isFinite(v)&&v>=0)?Math.sqrt(Math.max(cv[0],cv[4])):null;
  return {frame:lio.frame,epoch:lio.epoch,stamp:lio.stamp,factor,rotation,sigma,
   point(p){const dx=p[0]-origin[0],dy=p[1]-origin[1];return [base[0]+(c*dx-s*dy)/factor,base[1]+(s*dx+c*dy)/factor]},
   unpoint(p){const dx=(p[0]-base[0])*factor,dy=(p[1]-base[1])*factor;return [origin[0]+c*dx+s*dy,origin[1]-s*dx+c*dy]},
   yaw(q){return bodyHeading(q)+rotation}
  };
 }
 const api={create,horizontalHeading,bodyHeading};root.NavGeoAlignment=api;if(typeof module!=='undefined')module.exports=api;
})(globalThis);

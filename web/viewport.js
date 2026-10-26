/* Shared projection and cursor-anchored camera. No ROS dependency. */
(function(root){
 const R=6378137;
 function project(lon,lat){return [R*lon*Math.PI/180,R*Math.log(Math.tan(Math.PI/4+Math.max(-85,Math.min(85,lat))*Math.PI/360))]}
 function unproject(x,y){return [x/R*180/Math.PI,(2*Math.atan(Math.exp(y/R))-Math.PI/2)*180/Math.PI]}
 class Camera{
  constructor(scale=30){this.x=0;this.y=0;this.scale=scale;this.width=1;this.height=1;this.rotation=0}
  screen(p){const x=p[0]-this.x,y=p[1]-this.y,c=Math.cos(this.rotation),s=Math.sin(this.rotation);return [this.width/2+(c*x-s*y)*this.scale,this.height/2-(s*x+c*y)*this.scale]}
  world(p){const x=(p[0]-this.width/2)/this.scale,y=-(p[1]-this.height/2)/this.scale,c=Math.cos(this.rotation),s=Math.sin(this.rotation);return [this.x+c*x+s*y,this.y-s*x+c*y]}
  pan(dx,dy){const before=this.world([this.width/2,this.height/2]),after=this.world([this.width/2+dx,this.height/2+dy]);this.x-=after[0]-before[0];this.y-=after[1]-before[1]}
  zoom(factor,p=[this.width/2,this.height/2]){const before=this.world(p);this.scale=Math.max(.00001,Math.min(512,this.scale*factor));const after=this.world(p);this.x+=before[0]-after[0];this.y+=before[1]-after[1]}
  fit(b){this.x=(b[0]+b[2])/2;this.y=(b[1]+b[3])/2;this.scale=Math.max(.00001,Math.min(512,Math.min(this.width*.82/Math.max(1,b[2]-b[0]),this.height*.78/Math.max(1,b[3]-b[1]))))}
 }
 const api={Camera,project,unproject};root.NavViewport=api;if(typeof module!=='undefined')module.exports=api;
})(globalThis);

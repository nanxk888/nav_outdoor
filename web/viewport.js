/* Shared projection and cursor-anchored camera. No ROS dependency. */
(function(root){
 const R=6378137;
 function project(lon,lat){return [R*lon*Math.PI/180,R*Math.log(Math.tan(Math.PI/4+Math.max(-85,Math.min(85,lat))*Math.PI/360))]}
 function unproject(x,y){return [x/R*180/Math.PI,(2*Math.atan(Math.exp(y/R))-Math.PI/2)*180/Math.PI]}
 class Camera{
  constructor(scale=30){this.x=0;this.y=0;this.scale=scale;this.width=1;this.height=1}
  screen(p){return [this.width/2+(p[0]-this.x)*this.scale,this.height/2-(p[1]-this.y)*this.scale]}
  world(p){return [this.x+(p[0]-this.width/2)/this.scale,this.y-(p[1]-this.height/2)/this.scale]}
  pan(dx,dy){this.x-=dx/this.scale;this.y+=dy/this.scale}
  zoom(factor,p=[this.width/2,this.height/2]){const before=this.world(p);this.scale=Math.max(.00001,Math.min(512,this.scale*factor));const after=this.world(p);this.x+=before[0]-after[0];this.y+=before[1]-after[1]}
  fit(b){this.x=(b[0]+b[2])/2;this.y=(b[1]+b[3])/2;this.scale=Math.max(.00001,Math.min(512,Math.min(this.width*.82/Math.max(1,b[2]-b[0]),this.height*.78/Math.max(1,b[3]-b[1]))))}
 }
 const api={Camera,project,unproject};root.NavViewport=api;if(typeof module!=='undefined')module.exports=api;
})(globalThis);

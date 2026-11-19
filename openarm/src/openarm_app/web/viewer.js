import * as T from './lib/three.module.js';
import {OrbitControls} from './lib/OrbitControls.js';
export class RobotView{
 constructor(host,status){
  this.status=status;this.objects=[];this.id=null;this.loading=false;this.latest=null;this.retryAt=0;
  this.scene=new T.Scene();this.scene.background=new T.Color('#f5f9fe');
  this.camera=new T.PerspectiveCamera(38,1,.01,30);this.camera.up.set(0,0,1);this.camera.position.set(.95,-1.15,.95);
  this.renderer=new T.WebGLRenderer({antialias:true});this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.append(this.renderer.domElement);
  this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.target.set(.15,0,.45);this.controls.enableDamping=true;
  this.scene.add(new T.HemisphereLight(0xffffff,0x607080,2.5));const light=new T.DirectionalLight(0xffffff,3);light.position.set(2,-3,4);this.scene.add(light);
  const grid=new T.GridHelper(3,30,0xc8d8eb,0xe1e9f4);grid.rotation.x=Math.PI/2;grid.position.z=-.002;this.scene.add(grid);
  this.scene.add(new T.AxesHelper(.25));
  new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;this.renderer.setSize(w,h);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}).observe(host);
  const animate=()=>{requestAnimationFrame(animate);this.controls.update();this.renderer.render(this.scene,this.camera);};animate();
 }
 async update(visual){
  if(!visual){this.status.textContent='请重启仿真服务以加载 3D 模型';return;}
  this.latest=visual;
  if(this.id!==visual.id){
   if(this.loading||Date.now()<this.retryAt)return;this.loading=true;this.status.textContent='正在加载实际场景模型…';
   try{const response=await fetch('/api/model');if(!response.ok)throw Error('模型加载失败');const model=await response.json();
    this.clear();const meshes={};const decode=(s,Type)=>new Type(Uint8Array.from(atob(s),c=>c.charCodeAt(0)).buffer);
    for(const [key,value] of Object.entries(model.meshes)){const g=new T.BufferGeometry();g.setAttribute('position',new T.BufferAttribute(decode(value.vertices,Float32Array),3));g.setIndex(new T.BufferAttribute(decode(value.faces,Uint32Array),1));g.computeVertexNormals();meshes[key]=g;}
    this.geometries=new Set(Object.values(meshes));
    this.objects=model.geoms.map(spec=>{
     const [x,y,z]=spec.size;let g=meshes[String(spec.mesh)];
     if(!g){switch(spec.type){case 0:g=new T.PlaneGeometry(x?2*x:3,y?2*y:3);break;case 2:g=new T.SphereGeometry(x,20,12);break;case 3:g=new T.CapsuleGeometry(x,2*y,6,12);g.rotateX(Math.PI/2);break;case 4:g=new T.SphereGeometry(1,20,12);g.scale(x,y,z);break;case 5:g=new T.CylinderGeometry(x,x,2*y,20);g.rotateX(Math.PI/2);break;case 6:g=new T.BoxGeometry(2*x,2*y,2*z);break;default:g=new T.BufferGeometry();}this.geometries.add(g);}
     const [r,b,c,a]=spec.rgba,mat=new T.MeshStandardMaterial({color:new T.Color(r,b,c),roughness:.65,metalness:.12,opacity:a,transparent:a<1,side:T.DoubleSide});
     const object=new T.Mesh(g,mat);object.matrixAutoUpdate=false;this.scene.add(object);return object;
    });this.id=model.id;this.status.textContent='实时模型 · 拖动旋转 / 滚轮缩放';
   }catch(e){this.status.textContent=e.message+'；稍后自动重试';this.retryAt=Date.now()+5000;}finally{this.loading=false;}
  }
  const v=this.latest;if(v?.id===this.id){this.objects.forEach((o,i)=>{const r=v.rotations[i],p=v.positions[i];if(!r||!p)return;o.matrix.set(r[0],r[1],r[2],p[0],r[3],r[4],r[5],p[1],r[6],r[7],r[8],p[2],0,0,0,1);});}
 }
 clear(){for(const o of this.objects){this.scene.remove(o);o.material.dispose();}this.geometries?.forEach(g=>g.dispose());this.objects=[];}
 reset(){this.camera.position.set(.95,-1.15,.95);this.controls.target.set(.15,0,.45);}
}

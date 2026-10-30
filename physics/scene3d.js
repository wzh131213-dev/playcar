import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';

const EngineAPI=window.PlayCarPhysicsEngine;
const status=document.getElementById('scene-status');
const host=document.getElementById('scene3d');
if(!EngineAPI||!host){if(status)status.textContent='3D 场景初始化失败：缺少物理引擎或容器';throw new Error('PlayCar 3D scene prerequisites missing');}
const {PlayCarPhysicsEngine,TriangleMesh,Vector3:V,Quaternion:Q}=EngineAPI;
const scene=new THREE.Scene();
scene.background=new THREE.Color(0x91b7ce);
scene.fog=new THREE.Fog(0x91b7ce,90,260);
const camera=new THREE.PerspectiveCamera(56,1,.1,500);
camera.position.set(12,9,16);
const renderer=new THREE.WebGLRenderer({antialias:true,alpha:false});
renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.7));
renderer.shadowMap.enabled=true;
renderer.shadowMap.type=THREE.PCFSoftShadowMap;
renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.toneMapping=THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure=1.12;
host.appendChild(renderer.domElement);
renderer.domElement.setAttribute('aria-label','可交互三维赛车物理模拟场景');renderer.domElement.style.touchAction='none';

scene.add(new THREE.HemisphereLight(0xdceeff,0x344455,2.1));
const sun=new THREE.DirectionalLight(0xfff1d7,3.2);sun.position.set(-35,55,25);sun.castShadow=true;
sun.shadow.mapSize.set(2048,2048);sun.shadow.camera.left=-85;sun.shadow.camera.right=85;sun.shadow.camera.top=85;sun.shadow.camera.bottom=-85;scene.add(sun);
const fill=new THREE.DirectionalLight(0x75cfff,.8);fill.position.set(35,18,-45);scene.add(fill);

const mats={
 road:new THREE.MeshStandardMaterial({color:0x394b59,roughness:.92}),
 grass:new THREE.MeshStandardMaterial({color:0x567a58,roughness:1}),
 line:new THREE.MeshStandardMaterial({color:0xd7e4e8,roughness:.8}),
 barrier:new THREE.MeshStandardMaterial({color:0xe5edf1,roughness:.65}),
 orange:new THREE.MeshStandardMaterial({color:0xff713d,roughness:.38,metalness:.2}),
 glass:new THREE.MeshStandardMaterial({color:0x183d52,roughness:.2,metalness:.35}),
 tire:new THREE.MeshStandardMaterial({color:0x111820,roughness:.9}),
 rim:new THREE.MeshStandardMaterial({color:0x9eb4c4,metalness:.8,roughness:.25}),
};
const world=new THREE.Group();scene.add(world);
const physicsTriangles=[],colliderTriangles=[];
function addTriangle(target,a,b,c){target.push([a,b,c]);}
function addQuad(target,a,b,c,d){addTriangle(target,a,b,c);addTriangle(target,a,c,d);}
function meshFromTriangles(tris,material){const positions=[];for(const t of tris)for(const p of t)positions.push(p.x,p.y,p.z);const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));g.computeVertexNormals();const m=new THREE.Mesh(g,material);m.receiveShadow=true;m.castShadow=true;world.add(m);return m;}
function createBarrier(x1,z1,x2,z2,height=1.25){
 const a={x:x1,y:0,z:z1},b={x:x2,y:0,z:z2},c={x:x2,y:height,z:z2},d={x:x1,y:height,z:z1};
 const v={x:x2-x1,y:0,z:z2-z1},len=Math.hypot(v.x,v.z)||1,n={x:-v.z/len*.45,y:0,z:v.x/len*.45};
 addQuad(colliderTriangles,a,b,c,d);
 addQuad(colliderTriangles,{x:a.x+n.x,y:0,z:a.z+n.z},{x:b.x+n.x,y:0,z:b.z+n.z},{x:c.x+n.x,y:height,z:c.z+n.z},{x:d.x+n.x,y:height,z:d.z+n.z});
 const shape=new THREE.BoxGeometry(Math.hypot(x2-x1,z2-z1),height,.55);
 const obj=new THREE.Mesh(shape,mats.barrier);obj.position.set((x1+x2)/2,height/2,(z1+z2)/2);obj.rotation.y=-Math.atan2(z2-z1,x2-x1);obj.castShadow=true;obj.receiveShadow=true;world.add(obj);
}
function createRamp(){
 const a={x:-5,y:0,z:-18},b={x:5,y:0,z:-18},c={x:5,y:3.2,z:-28},d={x:-5,y:3.2,z:-28};
 addQuad(physicsTriangles,a,b,c,d);
 const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute([a.x,a.y,a.z,b.x,b.y,b.z,c.x,c.y,c.z,a.x,a.y,a.z,c.x,c.y,c.z,d.x,d.y,d.z],3));geo.computeVertexNormals();
 const ramp=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color:0x657d8e,roughness:.7,side:THREE.DoubleSide}));ramp.receiveShadow=true;ramp.castShadow=true;world.add(ramp);
 for(let i=0;i<5;i++){const stripe=new THREE.Mesh(new THREE.BoxGeometry(9.7,.06,.12),mats.line);stripe.position.set(0,.15+i*.58,-19-i*1.7);stripe.rotation.x=-Math.atan2(3.2,10);world.add(stripe);}
}
const groundGeo=new THREE.PlaneGeometry(240,240,1,1);groundGeo.rotateX(-Math.PI/2);
const ground=new THREE.Mesh(groundGeo,mats.grass);ground.position.y=-.18;ground.receiveShadow=true;world.add(ground);
const roadGeo=new THREE.PlaneGeometry(18,150,1,1);roadGeo.rotateX(-Math.PI/2);
const road=new THREE.Mesh(roadGeo,mats.road);road.position.set(0,-.06,-12);road.receiveShadow=true;road.castShadow=true;world.add(road);
addQuad(physicsTriangles,{x:-9,y:0,z:63},{x:-9,y:0,z:-87},{x:9,y:0,z:-87},{x:9,y:0,z:63});
for(const x of [-9.25,9.25])createBarrier(x,62,x,-86,1.15);
for(let z=55;z>-82;z-=8){const stripe=new THREE.Mesh(new THREE.BoxGeometry(.22,.025,3.6),mats.line);stripe.position.set(0,-.025,z);world.add(stripe);}
createRamp();
const surface=TriangleMesh.fromTriangles(physicsTriangles,6);
const walls=TriangleMesh.fromTriangles(colliderTriangles,5);
const vehicle=new PlayCarPhysicsEngine({surfaceMesh:surface,colliderMeshes:[walls],position:{x:0,y:.08,z:34},preset:'simcade',config:{gravity:9.81,maxSpeed:62,bodyHalfExtents:{x:.82,y:.42,z:1.55},wheelbase:2.55}});
const carRoot=new THREE.Group();world.add(carRoot);
const chassis=new THREE.Mesh(new THREE.BoxGeometry(1.65,.55,3.05),mats.orange);chassis.position.y=.55;chassis.castShadow=true;chassis.receiveShadow=true;carRoot.add(chassis);
const cabin=new THREE.Mesh(new THREE.BoxGeometry(1.25,.58,1.35),mats.glass);cabin.position.set(0,1.08,-.2);cabin.castShadow=true;carRoot.add(cabin);
const nose=new THREE.Mesh(new THREE.BoxGeometry(1.35,.16,.5),mats.orange);nose.position.set(0,.83,-1.18);carRoot.add(nose);
const wheels=[];
for(const x of [-.88,.88])for(const z of [-1.02,1.02]){
 const pivot=new THREE.Group();pivot.position.set(x,.36,z);carRoot.add(pivot);
 const tire=new THREE.Mesh(new THREE.CylinderGeometry(.37,.37,.25,20),mats.tire);tire.rotation.z=Math.PI/2;tire.castShadow=true;pivot.add(tire);
 const hub=new THREE.Mesh(new THREE.CylinderGeometry(.18,.18,.27,16),mats.rim);hub.rotation.z=Math.PI/2;pivot.add(hub);wheels.push({pivot,tire,z});
}
const headlightMat=new THREE.MeshStandardMaterial({color:0xfff4c4,emissive:0xffd98b,emissiveIntensity:1.8});
for(const x of [-.48,.48]){const lamp=new THREE.Mesh(new THREE.BoxGeometry(.28,.12,.08),headlightMat);lamp.position.set(x,.58,-1.55);carRoot.add(lamp);}
const shadow=new THREE.Mesh(new THREE.CircleGeometry(1.6,32),new THREE.MeshBasicMaterial({color:0x15212b,transparent:true,opacity:.23}));shadow.rotation.x=-Math.PI/2;shadow.position.y=-.12;world.add(shadow);

const input={gas:false,brake:false,left:false,right:false};
function release(){for(const k of Object.keys(input))input[k]=false;}
function bindHold(button,key){
 const el=document.getElementById(button);if(!el)return;
 el.addEventListener('pointerdown',e=>{e.preventDefault();input[key]=true;try{el.setPointerCapture(e.pointerId);}catch(_){}});
 ['pointerup','pointercancel','lostpointercapture'].forEach(type=>el.addEventListener(type,e=>{if(e)e.preventDefault();input[key]=false;}));
}
[['scene-gas','gas'],['scene-brake','brake'],['scene-left','left'],['scene-right','right']].forEach(x=>bindHold(...x));
window.addEventListener('keydown',e=>{if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(e.key))e.preventDefault();if(e.key==='ArrowUp'||e.key.toLowerCase()==='w')input.gas=true;if(e.key==='ArrowDown'||e.key.toLowerCase()==='s')input.brake=true;if(e.key==='ArrowLeft'||e.key.toLowerCase()==='a')input.left=true;if(e.key==='ArrowRight'||e.key.toLowerCase()==='d')input.right=true;},{passive:false});
window.addEventListener('keyup',e=>{if(e.key==='ArrowUp'||e.key.toLowerCase()==='w')input.gas=false;if(e.key==='ArrowDown'||e.key.toLowerCase()==='s')input.brake=false;if(e.key==='ArrowLeft'||e.key.toLowerCase()==='a')input.left=false;if(e.key==='ArrowRight'||e.key.toLowerCase()==='d')input.right=false;});
window.addEventListener('blur',release);
function resetVehicle(){vehicle.reset();vehicle.state.position={x:0,y:.08,z:34};vehicle.state.velocity={x:0,y:0,z:0};vehicle.state.orientation=Q.identity();vehicle.previousState={...vehicle.state,position:{...vehicle.state.position},velocity:{...vehicle.state.velocity},orientation:{...vehicle.state.orientation}};release();}
document.getElementById('scene-reset')?.addEventListener('click',resetVehicle);
let follow=true;
document.getElementById('scene-follow')?.addEventListener('click',e=>{follow=!follow;e.currentTarget.textContent=follow?'跟随视角：开':'跟随视角：关';});
const speedEl=document.getElementById('scene-speed'),gripEl=document.getElementById('scene-grip'),heightEl=document.getElementById('scene-height'),slipEl=document.getElementById('scene-slip');
const clock=new THREE.Clock();let frame=0,lastStatus=0;
function resize(){const w=Math.max(1,host.clientWidth),h=Math.max(260,host.clientHeight);renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
const ro=new ResizeObserver(resize);ro.observe(host);resize();
function render(){
 requestAnimationFrame(render);const dt=Math.min(clock.getDelta(),.05);vehicle.advance(dt,{gas:input.gas,brake:input.brake,left:input.left,right:input.right});
 const pose=vehicle.interpolatedState();carRoot.position.set(pose.position.x,pose.position.y,pose.position.z);carRoot.quaternion.set(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
 for(const w of wheels)w.tire.rotation.x-=dt*V.len(vehicle.state.velocity)*.9;
 shadow.position.x=vehicle.state.position.x;shadow.position.z=vehicle.state.position.z;
 if(follow){const p=carRoot.position.clone();const back=new THREE.Vector3(0,5.2,11).applyQuaternion(carRoot.quaternion);camera.position.lerp(p.clone().add(back),.075);camera.lookAt(p.x,p.y+.65,p.z-1.2);}
 renderer.render(scene,camera);
 if(++frame%6===0){const t=vehicle.getTelemetry();if(speedEl)speedEl.textContent=(t.speed*3.6).toFixed(0)+' km/h';if(gripEl)gripEl.textContent=vehicle.state.gripState;if(heightEl)heightEl.textContent=vehicle.state.position.y.toFixed(2)+' m';if(slipEl)slipEl.textContent=(t.slipAngle*180/Math.PI).toFixed(1)+'°';if(status)status.textContent='3D 场景运行中 · '+vehicle.state.tick+' ticks';}
}
status.textContent='3D 场景已就绪';
render();

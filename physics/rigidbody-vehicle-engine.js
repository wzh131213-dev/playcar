/* PlayCar rigid-body vehicle physics 2.5.0
 * Fixed-step 120 Hz, four independent suspension/tire contacts, rigid-body integration.
 * SI-like units: metres, seconds, kilograms, newtons, radians.
 */
(function(root){
'use strict';
const V={
 add:(a,b)=>({x:a.x+b.x,y:a.y+b.y,z:a.z+b.z}), sub:(a,b)=>({x:a.x-b.x,y:a.y-b.y,z:a.z-b.z}),
 mul:(a,s)=>({x:a.x*s,y:a.y*s,z:a.z*s}), dot:(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z,
 cross:(a,b)=>({x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x}),
 len:a=>Math.hypot(a.x,a.y,a.z), norm:a=>{const n=Math.hypot(a.x,a.y,a.z)||1;return{x:a.x/n,y:a.y/n,z:a.z/n};},
 clamp:(x,a,b)=>Math.max(a,Math.min(b,x)), zero:()=>({x:0,y:0,z:0})
};
const Q={
 id:()=>({x:0,y:0,z:0,w:1}), mul:(a,b)=>({x:a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,y:a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,z:a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,w:a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z}),
 norm:q=>{const n=Math.hypot(q.x,q.y,q.z,q.w)||1;return{x:q.x/n,y:q.y/n,z:q.z/n,w:q.w/n};},
 rotate:(q,v)=>{const u={x:q.x,y:q.y,z:q.z},t=V.mul(V.cross(u,v),2);return V.add(v,V.add(V.mul(t,q.w),V.cross(u,t)));},
 axis:(a,t)=>{const n=V.norm(a),s=Math.sin(t/2);return{x:n.x*s,y:n.y*s,z:n.z*s,w:Math.cos(t/2)};},
 slerp:(a,b,t)=>{let d=a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;if(d<0){b={x:-b.x,y:-b.y,z:-b.z,w:-b.w};d=-d;}if(d>.9995)return Q.norm({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,z:a.z+(b.z-a.z)*t,w:a.w+(b.w-a.w)*t});const th=Math.acos(V.clamp(d,-1,1)),s=Math.sin(th),u=Math.sin((1-t)*th)/s,v=Math.sin(t*th)/s;return Q.norm({x:a.x*u+b.x*v,y:a.y*u+b.y*v,z:a.z*u+b.z*v,w:a.w*u+b.w*v});}
};
const DEFAULTS={
 fixedDt:1/120,maxSubsteps:24,mass:1250,gravity:9.81,
 inertia:{x:520,y:1450,z:1650},com:{x:0,y:.52,z:0},
 wheelbase:2.65,track:1.56,wheelRadius:.32,wheelMass:18,
 suspension:{rest:.32,min:.08,max:.48,stiffness:32000,damping:4200,bump:90000,bumpStopRange:.12},
 tire:{mu:1.08,corneringStiffness:78000,longitudinalStiffness:95000,relaxation:.04,relaxationAngle:.08,loadSensitivity:.12,referenceLoad:3000,aligningTrail:.08,pacejka:{enabled:false,B:0,C:1.3,E:.97,peak:1},frictionEllipse:{longitudinal:1,lateral:1}},
 engine:{peakTorque:280,idleRpm:900,redline:6500,finalDrive:3.9,gears:[3.2,2.1,1.5,1.18,.94,.78],efficiency:.88},
 antiRoll:{front:8500,rear:6500},body:{rollStiffness:9000,rollDamping:2200,pitchStiffness:14000,pitchDamping:2800},differential:{type:'open',limitedSlip:2.2},aids:{abs:true,tcs:true,absSlip:.18,tcsSlip:.22},steering:{maxAngle:.48,highSpeedScale:.42},
 brakeTorque:4200,handbrakeTorque:6500,dragArea:0.68,airDensity:1.225,downforceArea:0.0,
 restitution:.08,collisionFriction:.55,bodyHalf:{x:.88,y:.48,z:2.05},ccd:true
};
function clone(o){return JSON.parse(JSON.stringify(o));}
function calculateTireForces(FxRaw,slipAngle,normalLoad,frictionCoefficient,tire={}){
 const cap=Math.max(0,frictionCoefficient*normalLoad),pacejka=tire.pacejka||{};
 let FyRaw;
 if(pacejka.enabled){
  const C=Math.max(.1,Number(pacejka.C)||1.3),E=Number.isFinite(Number(pacejka.E))?Number(pacejka.E):.97,peak=Math.max(0,Number(pacejka.peak??1));
  const D=cap*peak,B=Number(pacejka.B)>0?Number(pacejka.B):Math.max(.001,(Number(tire.corneringStiffness)||78000)/(C*Math.max(D,1)));
  const x=B*slipAngle;
  FyRaw=-D*Math.sin(C*Math.atan(x-E*(x-Math.atan(x))));
 }else FyRaw=-(Number(tire.corneringStiffness)||78000)*slipAngle;
 const ellipse=tire.frictionEllipse||{},longitudinal=Math.max(.01,Number(ellipse.longitudinal)||1),lateral=Math.max(.01,Number(ellipse.lateral)||1);
 const nx=cap*longitudinal,ny=cap*lateral;
 const utilization=cap>0?Math.hypot(FxRaw/nx,FyRaw/ny):Infinity,scale=utilization>1?1/utilization:1;
 const Fx=cap>0?FxRaw*scale:0,Fy=cap>0?FyRaw*scale:0;
 return{Fx,Fy,FyRaw,utilization:cap>0?Math.hypot(Fx/nx,Fy/ny):0,scale};
}
class RigidBodyVehicle{
 constructor(options={}){
  this.c=Object.assign({},clone(DEFAULTS),options.config||{});this.c.inertia=Object.assign({},DEFAULTS.inertia,options.config?.inertia||{});this.c.com=Object.assign({},DEFAULTS.com,options.config?.com||{});this.c.suspension=Object.assign({},DEFAULTS.suspension,options.config?.suspension||{});this.c.tire=Object.assign({},DEFAULTS.tire,options.config?.tire||{});this.c.tire.pacejka=Object.assign({},DEFAULTS.tire.pacejka,options.config?.tire?.pacejka||{});this.c.tire.frictionEllipse=Object.assign({},DEFAULTS.tire.frictionEllipse,options.config?.tire?.frictionEllipse||{});this.c.engine=Object.assign({},DEFAULTS.engine,options.config?.engine||{});this.c.antiRoll=Object.assign({},DEFAULTS.antiRoll,options.config?.antiRoll||{});this.c.body=Object.assign({},DEFAULTS.body,options.config?.body||{});this.c.differential=Object.assign({},DEFAULTS.differential,options.config?.differential||{});this.c.aids=Object.assign({},DEFAULTS.aids,options.config?.aids||{});this.c.steering=Object.assign({},DEFAULTS.steering,options.config?.steering||{});
  this.mesh=options.surfaceMesh||null;this.colliders=options.colliderMeshes||[];this.groundY=options.groundPlaneY??null;
  this.accumulator=0;this.gear=1;this.engineRpm=this.c.engine.idleRpm;this.tick=0;
  this.state={position:{...(options.position||{x:0,y:.65,z:0})},velocity:V.zero(),orientation:Q.id(),angularVelocity:V.zero(),wheelOmega:[0,0,0,0],grounded:false};
  this.previous=clone(this.state);this.lastContacts=[];this.telemetry={};this._slipState=[0,0,0,0];this._slipAngleState=[0,0,0,0];this.aidState={abs:false,tcs:false};
 }
 setSurfaceMesh(mesh){this.mesh=mesh;return this;}
 reset(position={x:0,y:.65,z:0}){this.state={position:{...position},velocity:V.zero(),orientation:Q.id(),angularVelocity:V.zero(),wheelOmega:[0,0,0,0],grounded:false};this.previous=clone(this.state);this.accumulator=0;this.tick=0;this._slipState=[0,0,0,0];this._slipAngleState=[0,0,0,0];this.aidState={abs:false,tcs:false};this.lastContacts=[];this.telemetry={};}
 _invInertiaWorld(v){const q=this.state.orientation,local=Q.rotate({x:-q.x,y:-q.y,z:-q.z,w:q.w},v),i=this.c.inertia;return Q.rotate(q,{x:local.x/i.x,y:local.y/i.y,z:local.z/i.z});}
 _pointVelocity(r){return V.add(this.state.velocity,V.cross(this.state.angularVelocity,r));}
 _applyForce(F,r,dt){const s=this.state;s.velocity=V.add(s.velocity,V.mul(F,dt/this.c.mass));s.angularVelocity=V.add(s.angularVelocity,V.mul(this._invInertiaWorld(V.cross(r,F)),dt));}
 _applyTorque(torque,dt){this.state.angularVelocity=V.add(this.state.angularVelocity,V.mul(this._invInertiaWorld(torque),dt));}
 _contacts(){
  const c=this.c,s=this.state,out=[],q=s.orientation,half=c.wheelbase/2,track=c.track/2;
  const mounts=[[-track,-half],[track,-half],[-track,half],[track,half]];
  for(let i=0;i<4;i++){
   const local={x:mounts[i][0],y:0,z:mounts[i][1]},r=Q.rotate(q,local),origin=V.add(s.position,r);
   const down=Q.rotate(q,{x:0,y:-1,z:0}),reach=c.suspension.max+c.wheelRadius;
   let hit=null;
   if(this.mesh?.closestPoint){hit=this.mesh.closestPoint(origin,reach+c.wheelRadius,{x:0,y:1,z:0});}
   if(!hit&&Number.isFinite(this.groundY)){const dist=origin.y-this.groundY;if(dist<reach+c.wheelRadius)hit={point:{x:origin.x,y:this.groundY,z:origin.z},normal:{x:0,y:1,z:0},distance:Math.max(0,dist-c.wheelRadius),signedDistance:dist-c.wheelRadius};}
   if(!hit)continue;
   let n=V.norm(hit.normal||{x:0,y:1,z:0});if(V.dot(n,{x:0,y:1,z:0})<0)n=V.mul(n,-1);
   const rawLength=hit.distance-c.wheelRadius,length=V.clamp(rawLength, c.suspension.min,c.suspension.max),compression=c.suspension.rest-length;
   const point=hit.point||V.sub(origin,V.mul(n,hit.distance)),arm=V.sub(point,s.position),pv=this._pointVelocity(arm),vn=V.dot(pv,n);
   out.push({index:i,point,normal:n,arm,compression,length,rawLength,normalVelocity:vn,velocity:pv,local,friction:Number.isFinite(hit.friction)?Math.max(0,hit.friction):1});
  }
  return out;
 }
 _bodyTilt(){
  const q=this.state.orientation,up=Q.rotate(q,{x:0,y:1,z:0}),inverse={x:-q.x,y:-q.y,z:-q.z,w:q.w};
  const localAngularVelocity=Q.rotate(inverse,this.state.angularVelocity);
  return{roll:Math.atan2(-up.x,Math.max(1e-9,up.y)),pitch:Math.atan2(up.z,Math.max(1e-9,up.y)),rollRate:localAngularVelocity.z,pitchRate:localAngularVelocity.x};
 }
 _applyBodyStabilization(dt){
  const b=this.c.body||{},tilt=this._bodyTilt();
  const localTorque={x:-Math.max(0,b.pitchStiffness||0)*tilt.pitch-Math.max(0,b.pitchDamping||0)*tilt.pitchRate,y:0,z:-Math.max(0,b.rollStiffness||0)*tilt.roll-Math.max(0,b.rollDamping||0)*tilt.rollRate};
  if(V.len(localTorque)>0)this._applyTorque(Q.rotate(this.state.orientation,localTorque),dt);
 }
 _fixed(dt,input){
  const c=this.c,s=this.state;this.previous=clone(s);this.aidState={abs:false,tcs:false};
  const contacts=this._contacts();this.lastContacts=contacts;
  const normalLoad=[0,0,0,0],forces=[],suspensionTelemetry=Array.from({length:4},(_,wheel)=>({wheel,compression:0,rawLength:null,bumpStopCompression:0,bumpStopForce:0,normalLoad:0}));
  // 1-2: wheel contact and spring/damper/bump-stop forces
  for(const h of contacts){
   const susp=c.suspension,bumpStopCompression=Math.max(0,susp.min-h.rawLength),bumpStopForce=Math.max(0,susp.bump||0)*bumpStopCompression*(1+bumpStopCompression/Math.max(.001,susp.bumpStopRange||.12)),spring=susp.stiffness*h.compression-susp.damping*h.normalVelocity+bumpStopForce;
   const load=Math.max(0,spring);normalLoad[h.index]=load;suspensionTelemetry[h.index]={wheel:h.index,compression:h.compression,rawLength:h.rawLength,bumpStopCompression,bumpStopForce,normalLoad:load};
   const F=V.mul(h.normal,load);this._applyForce(F,h.arm,dt);
  }
  for(const [left,right,rate] of [[0,1,c.antiRoll.front],[2,3,c.antiRoll.rear]]){const a=contacts.find(h=>h.index===left),b=contacts.find(h=>h.index===right);if(a&&b&&rate>0){const transfer=(a.compression-b.compression)*rate,fa=Math.max(-normalLoad[left],transfer),fb=Math.max(-normalLoad[right],-transfer);normalLoad[left]+=fa;normalLoad[right]+=fb;this._applyForce(V.mul(a.normal,fa),a.arm,dt);this._applyForce(V.mul(b.normal,fb),b.arm,dt);}}
  const forward=Q.rotate(s.orientation,{x:0,y:0,z:-1}),right=Q.rotate(s.orientation,{x:1,y:0,z:0});
  const speed=V.len(s.velocity),airDrag=.5*c.airDensity*c.dragArea*speed*speed;
  if(speed>1e-5)this._applyForce(V.mul(s.velocity,-airDrag/speed),V.zero(),dt);
  const downforce=.5*c.airDensity*c.downforceArea*speed*speed;
  if(downforce)this._applyForce(V.mul(Q.rotate(s.orientation,{x:0,y:-1,z:0}),downforce),V.zero(),dt);
  // 3-4: engine torque -> gearbox -> wheel torque; brakes and combined-slip tire forces
  const throttle=V.clamp(input.throttle||0,0,1),brake=V.clamp(input.brake||0,0,1),steer=V.clamp(input.steer||0,-1,1);
  const e=c.engine,avgWheel=s.wheelOmega.reduce((a,b)=>a+Math.abs(b),0)/4;
  this.engineRpm=V.clamp(avgWheel*c.wheelRadius*e.finalDrive*(e.gears[this.gear-1]||1)*60/(2*Math.PI),e.idleRpm,e.redline);
  if(this.engineRpm>e.redline*.96&&this.gear<e.gears.length)this.gear++;
  if(this.engineRpm<e.redline*.55&&this.gear>1)this.gear--;
  const torque=e.peakTorque*throttle*V.clamp(1-Math.abs(this.engineRpm-3800)/6500,.25,1);
  const totalDriveTorque=torque*e.gears[this.gear-1]*e.finalDrive*e.efficiency;
  // Predict longitudinal tire-force demand, then redistribute the same support load
  // between axles before tire forces are solved. Positive transfer adds load to the rear.
  const longitudinalSpeed=V.dot(s.velocity,forward);
  const driveForce=totalDriveTorque/Math.max(.05,c.wheelRadius);
  const brakeForce=longitudinalSpeed===0?0:brake*c.brakeTorque*4/Math.max(.05,c.wheelRadius)*Math.sign(longitudinalSpeed);
  const requestedLongitudinalForce=driveForce-brakeForce;
  const requestedAcceleration=V.clamp(requestedLongitudinalForce/c.mass,-c.tire.mu*c.gravity,c.tire.mu*c.gravity);
  const requestedTransfer=c.mass*requestedAcceleration*Math.max(0,c.com.y)/Math.max(.1,c.wheelbase);
  const frontContacts=contacts.filter(h=>h.index<2),rearContacts=contacts.filter(h=>h.index>=2);
  const frontAvailable=frontContacts.reduce((sum,h)=>sum+normalLoad[h.index],0);
  const rearAvailable=rearContacts.reduce((sum,h)=>sum+normalLoad[h.index],0);
  const longitudinalTransfer=frontContacts.length&&rearContacts.length?V.clamp(requestedTransfer,-rearAvailable,frontAvailable):0;
  for(const [axle,delta] of [[frontContacts,-longitudinalTransfer],[rearContacts,longitudinalTransfer]]){
   if(!axle.length)continue;
   const perWheel=delta/axle.length;
   for(const h of axle){
    normalLoad[h.index]=Math.max(0,normalLoad[h.index]+perWheel);
    const applied=normalLoad[h.index]-suspensionTelemetry[h.index].normalLoad;
    if(applied)this._applyForce(V.mul(h.normal,applied),h.arm,dt);
    suspensionTelemetry[h.index].normalLoad=normalLoad[h.index];
   }
  }
  const steeringForTransfer=steer*c.steering.maxAngle*(1-(1-c.steering.highSpeedScale)*V.clamp(speed/55,0,1));
  const requestedLateralAcceleration=longitudinalSpeed*Math.abs(longitudinalSpeed)*Math.tan(steeringForTransfer)/Math.max(.1,c.wheelbase);
  // Positive lateral acceleration (positive steering while moving forward) loads the left/outside wheels.
  const requestedLateralTransfer=c.mass*requestedLateralAcceleration*Math.max(0,c.com.y)/Math.max(.1,c.track);
  const leftContacts=contacts.filter(h=>h.index===0||h.index===2),rightContacts=contacts.filter(h=>h.index===1||h.index===3);
  const leftAvailable=leftContacts.reduce((sum,h)=>sum+normalLoad[h.index],0);
  const rightAvailable=rightContacts.reduce((sum,h)=>sum+normalLoad[h.index],0);
  const lateralTransfer=leftContacts.length&&rightContacts.length?V.clamp(requestedLateralTransfer,-leftAvailable,rightAvailable):0;
  for(const [side,delta] of [[leftContacts,lateralTransfer],[rightContacts,-lateralTransfer]]){
   if(!side.length)continue;
   const perWheel=delta/side.length;
   for(const h of side){
    normalLoad[h.index]=Math.max(0,normalLoad[h.index]+perWheel);
    const applied=normalLoad[h.index]-suspensionTelemetry[h.index].normalLoad;
    if(applied)this._applyForce(V.mul(h.normal,applied),h.arm,dt);
    suspensionTelemetry[h.index].normalLoad=normalLoad[h.index];
   }
  }
  const loadTransferTelemetry={longitudinal:-longitudinalTransfer,lateral:lateralTransfer};
  const rawDiffType=c.differential.type||'open',driveType=c.differential.layout||(['fwd','rwd','awd'].includes(rawDiffType)?rawDiffType:'rwd'),diffType=['lsd','limited-slip'].includes(rawDiffType)?'lsd':'open';const driven=[0,1,2,3].filter(i=>driveType==='awd'||(driveType==='fwd'&&i<2)||(driveType!=='fwd'&&driveType!=='awd'&&i>=2));
  const wheelDriveTorque=[0,0,0,0];
  for(const axle of [[0,1],[2,3]]){const pair=axle.filter(i=>driven.includes(i));if(!pair.length)continue;const base=totalDriveTorque/driven.length;
   if(pair.length===2&&diffType==='lsd'){const a=pair[0],b=pair[1],wa=Math.abs(s.wheelOmega[a]),wb=Math.abs(s.wheelOmega[b]),tbr=Math.max(1,Number(c.differential.limitedSlip)||1);if(Math.abs(wa-wb)<.25){wheelDriveTorque[a]=base;wheelDriveTorque[b]=base;}else{const slow=wa<wb?a:b,fast=slow===a?b:a;wheelDriveTorque[slow]=base*2*tbr/(1+tbr);wheelDriveTorque[fast]=base*2/(1+tbr);}}
   else for(const i of pair)wheelDriveTorque[i]=base;
  }
  const steerAngle=steer*c.steering.maxAngle*(1-(1-c.steering.highSpeedScale)*V.clamp(speed/55,0,1));
  for(const h of contacts){
   const i=h.index,rear=i>=2,front=!rear;
   let wheelForward=forward,wheelRight=right;
   if(front){wheelForward=V.norm(V.add(V.mul(forward,Math.cos(steerAngle)),V.mul(right,Math.sin(steerAngle))));wheelRight=V.norm(V.cross(wheelForward,h.normal));}
   const vLong=V.dot(h.velocity,wheelForward),vLat=V.dot(h.velocity,wheelRight),omega=s.wheelOmega[i];
   const rawSlip=(omega*c.wheelRadius-vLong)/Math.max(2,Math.abs(vLong)),relax=V.clamp(dt/Math.max(.001,c.tire.relaxation),0,1);this._slipState[i]+=(rawSlip-this._slipState[i])*relax;
   let slipRatio=this._slipState[i],rawSlipAngle=Math.atan2(vLat,Math.max(1,Math.abs(vLong))),angleRelax=V.clamp(dt/Math.max(.001,c.tire.relaxationAngle||c.tire.relaxation),0,1);this._slipAngleState[i]+=(rawSlipAngle-this._slipAngleState[i])*angleRelax;let slipAngle=this._slipAngleState[i];
   let wheelDrive=wheelDriveTorque[i];
   if(c.aids.tcs&&wheelDrive>0&&slipRatio>c.aids.tcsSlip){wheelDrive*=V.clamp(c.aids.tcsSlip/Math.max(.001,slipRatio),.15,1);this.aidState.tcs=true;}
   const FxRaw=c.tire.longitudinalStiffness*slipRatio+wheelDrive/c.wheelRadius;
   const loadSensitivityFactor=V.clamp(Math.pow(Math.max(1,normalLoad[i])/Math.max(1,c.tire.referenceLoad||3000),-(c.tire.loadSensitivity||0)),.55,1.25),frictionCoefficient=c.tire.mu*h.friction*loadSensitivityFactor,tireForce=calculateTireForces(FxRaw,slipAngle,normalLoad[i],frictionCoefficient,c.tire),Fx=tireForce.Fx,Fy=tireForce.Fy,FyRaw=tireForce.FyRaw,aligningMoment=-Fy*(c.tire.aligningTrail||0)*V.clamp(1-Math.abs(slipAngle)/.6,0,1);
   this._applyForce(V.add(V.mul(wheelForward,Fx),V.mul(wheelRight,Fy)),h.arm,dt);if(aligningMoment)this._applyTorque(V.mul(h.normal,aligningMoment),dt);
   let brakeTorque=brake*c.brakeTorque+(input.handbrake&&rear?c.handbrakeTorque:0);
   if(c.aids.abs&&brakeTorque>0&&Math.abs(slipRatio)>c.aids.absSlip){brakeTorque*=.35;this.aidState.abs=true;}
   const wheelAccel=(wheelDrive-Fx*c.wheelRadius-Math.sign(omega||vLong)*brakeTorque)/(c.wheelMass*c.wheelRadius*c.wheelRadius);
   s.wheelOmega[i]+=wheelAccel*dt;
   if(Math.abs(s.wheelOmega[i])<.5&&brake>.8)s.wheelOmega[i]=0;
   forces.push({wheel:i,Fx,Fy,normalLoad:normalLoad[i],slipRatio,slipAngle,rawSlipAngle,aligningMoment,loadSensitivityFactor,frictionCoefficient,driveTorque:wheelDrive,brakeTorque});
  }
  // 5-6: gravity, chassis roll/pitch suspension, semi-implicit Euler and quaternion integration
  this._applyForce({x:0,y:-c.mass*c.gravity,z:0},V.zero(),dt);
  this._applyBodyStabilization(dt);
  s.position=V.add(s.position,V.mul(s.velocity,dt));
  const w=s.angularVelocity,omegaQ={x:w.x,y:w.y,z:w.z,w:0};
  const dq=Q.mul(omegaQ,s.orientation);s.orientation=Q.norm({x:s.orientation.x+.5*dq.x*dt,y:s.orientation.y+.5*dq.y*dt,z:s.orientation.z+.5*dq.z*dt,w:s.orientation.w+.5*dq.w*dt});
  // 7: positional contact recovery, normal impulse and approximate swept-body CCD
  this._solveCollisions(dt);
  const after=this._contacts();s.grounded=after.length>=2;
  this.tick++;
  this.telemetry={tick:this.tick,contacts:after.length,normalLoads:normalLoad,forces,suspension:suspensionTelemetry,bodyTilt:this._bodyTilt(),bodyStabilization:{...c.body},gear:this.gear,rpm:this.engineRpm,speed:V.len(s.velocity),downforce,absActive:this.aidState.abs,tcsActive:this.aidState.tcs,antiRoll:{front:c.antiRoll.front,rear:c.antiRoll.rear},loadTransfer:loadTransferTelemetry,differential:diffType,driveLayout:driveType,drivenWheels:[...driven]};
 }
 _solveCollisions(dt){
  const s=this.state,c=this.c;
  // Wheel-supported ground plus chassis floor clamp; mesh closest-point acts as a two-sided contact query.
  if(Number.isFinite(this.groundY)){
   const bottom=s.position.y-c.bodyHalf.y;
   if(bottom<this.groundY){s.position.y+=this.groundY-bottom;if(s.velocity.y<0)s.velocity.y=0;}
  }
  if(!this.mesh?.closestPoint)return;
  const radius=Math.hypot(c.bodyHalf.x,c.bodyHalf.y,c.bodyHalf.z);
  const q=s.orientation,axisX=Q.rotate(q,{x:1,y:0,z:0}),axisY=Q.rotate(q,{x:0,y:1,z:0}),axisZ=Q.rotate(q,{x:0,y:0,z:1});
  // Projected OBB support radius avoids treating the whole chassis as a sphere on flat ground.
  const previous=this.previous.position,travel=V.sub(s.position,previous),distance=V.len(travel),samples=c.ccd?Math.min(32,Math.max(1,Math.ceil(distance/Math.max(.08,c.bodyHalf.y*.5)))):1;
  for(let k=0;k<=samples;k++){
   const t=k/samples,center=V.add(previous,V.mul(travel,t)),hit=this.mesh.closestPoint(center,radius,{x:0,y:1,z:0});
   if(!hit)continue;
   const n=V.norm(hit.normal||{x:0,y:1,z:0}),signed=hit.signedDistance??V.dot(V.sub(center,hit.point),n);
   const support=Math.abs(V.dot(n,axisX))*c.bodyHalf.x+Math.abs(V.dot(n,axisY))*c.bodyHalf.y+Math.abs(V.dot(n,axisZ))*c.bodyHalf.z;
   const penetration=support-signed;
   if(penetration>0&&V.dot(n,{x:0,y:1,z:0})>.15){
    s.position=V.add(s.position,V.mul(n,penetration));
    const vn=V.dot(s.velocity,n);if(vn<0)s.velocity=V.sub(s.velocity,V.mul(n,(1+c.restitution)*vn));
    const tangent=V.sub(s.velocity,V.mul(n,V.dot(s.velocity,n)));s.velocity=V.sub(s.velocity,V.mul(tangent,c.collisionFriction));
   }
  }
 }
 advance(frameDelta,input={}){
  const dt=Math.max(0,Math.min(.25,Number(frameDelta)||0));this.accumulator+=dt;let steps=0;
  while(this.accumulator+1e-10>=this.c.fixedDt&&steps<this.c.maxSubsteps){this._fixed(this.c.fixedDt,input);this.accumulator-=this.c.fixedDt;steps++;}
  if(this.accumulator>=this.c.fixedDt)this.accumulator%=this.c.fixedDt;
  return{steps,alpha:V.clamp(this.accumulator/this.c.fixedDt,0,1)};
 }
 interpolatedState(){const a=this.previous,b=this.state,t=V.clamp(this.accumulator/this.c.fixedDt,0,1);return{position:V.add(a.position,V.mul(V.sub(b.position,a.position),t)),velocity:V.add(a.velocity,V.mul(V.sub(b.velocity,a.velocity),t)),orientation:Q.slerp(a.orientation,b.orientation,t),grounded:b.grounded};}
}
root.PlayCarRigidBody={RigidBodyVehicle,Vector3:V,Quaternion:Q,calculateTireForces,defaults:clone(DEFAULTS),version:'2.5.0'};
if(typeof module!=='undefined'&&module.exports)module.exports=root.PlayCarRigidBody;
})(typeof globalThis!=='undefined'?globalThis:this);

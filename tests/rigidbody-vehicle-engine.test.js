'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const {RigidBodyVehicle,Vector3:V}=require('../physics/rigidbody-vehicle-engine.js');
function plane(){
 const a={x:-100,y:0,z:-100},b={x:-100,y:0,z:100},c={x:100,y:0,z:100},d={x:100,y:0,z:-100};
 const tris=[[a,b,c],[a,c,d]];
 return {closestPoint(p,max=Infinity){let best=null;for(const tri of tris){const y=0,point={x:V.clamp(p.x,-100,100),y,z:V.clamp(p.z,-100,100)},distance=Math.abs(p.y-y);if(distance<=max&&(!best||distance<best.distance))best={point,normal:{x:0,y:1,z:0},distance,signedDistance:p.y-y};}return best;}};
}
function make(){return new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0}});}
test('fixed simulation step is exactly 1/120 second',()=>{const e=make();assert.equal(e.c.fixedDt,1/120);const r=e.advance(1/60,{throttle:1});assert.equal(r.steps,2);});
test('four independent wheel contacts create suspension loads',()=>{const e=make();e.advance(1/120,{});assert.equal(e.telemetry.contacts,4);assert.equal(e.telemetry.normalLoads.length,4);assert.ok(e.telemetry.normalLoads.every(Number.isFinite));assert.ok(e.telemetry.normalLoads.some(x=>x>0),'compressed springs should generate support load');});
test('engine torque, gears and wheel angular velocity respond to throttle',()=>{const e=make();for(let i=0;i<120;i++)e.advance(1/120,{throttle:1});assert.ok(e.state.wheelOmega.some(x=>x>0));assert.ok(e.telemetry.rpm>=e.c.engine.idleRpm);});
test('combined tire force is clamped by mu times normal load',()=>{const e=make();e.advance(1/120,{throttle:1,steer:1});for(const f of e.telemetry.forces){assert.ok(Math.hypot(f.Fx,f.Fy)<=e.c.tire.mu*f.normalLoad+1e-6);}});
test('semi-implicit integration keeps quaternion normalized and state finite',()=>{const e=make();for(let i=0;i<240;i++)e.advance(1/120,{throttle:.5,steer:.2});const q=e.state.orientation;assert.ok(Math.abs(Math.hypot(q.x,q.y,q.z,q.w)-1)<1e-8);for(const v of Object.values(e.state.position))assert.ok(Number.isFinite(v));});
test('render interpolation returns a normalized orientation',()=>{const e=make();e.advance(.01,{throttle:1});const p=e.interpolatedState();assert.ok(Math.abs(Math.hypot(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)-1)<1e-8);});

test('front-wheel-drive differential layout drives only the front axle',()=>{const e=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0},config:{differential:{type:'fwd',limitedSlip:2.2}}});for(let i=0;i<120;i++)e.advance(1/120,{throttle:1});assert.ok(e.state.wheelOmega[0]>0&&e.state.wheelOmega[1]>0);assert.deepEqual(e.telemetry.drivenWheels,[0,1]);assert.equal(e.telemetry.differential,'open');assert.equal(e.telemetry.driveLayout,'fwd');});
test('anti-roll and driver-aid telemetry remains finite and exposed',()=>{const e=make();e.advance(1/120,{throttle:.5,steer:.3});assert.deepEqual(e.telemetry.antiRoll,{front:e.c.antiRoll.front,rear:e.c.antiRoll.rear});assert.equal(e.telemetry.differential,'open');assert.equal(typeof e.telemetry.absActive,'boolean');assert.equal(typeof e.telemetry.tcsActive,'boolean');for(const n of e.telemetry.normalLoads)assert.ok(Number.isFinite(n));});

test('limited-slip differential biases torque toward the slower wheel',()=>{const e=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0},config:{differential:{type:'lsd',limitedSlip:3},aids:{abs:false,tcs:false,absSlip:.18,tcsSlip:.22}}});e.state.wheelOmega[2]=0;e.state.wheelOmega[3]=24;e.advance(1/120,{throttle:1});const rear=e.telemetry.forces.filter(f=>f.wheel>=2);assert.equal(rear.length,2);assert.ok(rear[0].driveTorque>rear[1].driveTorque);assert.ok(Math.abs(rear[0].driveTorque/rear[1].driveTorque-3)<1e-9,'torque bias ratio must match the configured TBR');assert.ok(Math.abs(rear[0].driveTorque+rear[1].driveTorque)>0);});

test('tire slip angle relaxes over time instead of applying an instantaneous lateral force',()=>{const e=make();e.state.velocity={x:5,y:0,z:0};e.advance(1/120,{});const f=e.telemetry.forces[0];assert.ok(Math.abs(f.slipAngle)<Math.abs(f.rawSlipAngle));assert.ok(Math.abs(f.slipAngle)>0);});
test('tire load sensitivity reduces normalized grip as vertical load rises',()=>{const low=make(),high=make();low.c.tire.loadSensitivity=.2;high.c.tire.loadSensitivity=.2;low.c.tire.referenceLoad=3000;high.c.tire.referenceLoad=3000;low.state.position.y=.3;high.state.position.y=.3;high.c.suspension.stiffness*=2;low.advance(1/120,{steer:.5});high.advance(1/120,{steer:.5});const lf=low.telemetry.forces[0],hf=high.telemetry.forces[0];assert.ok(hf.loadSensitivityFactor<lf.loadSensitivityFactor);});
test('pneumatic trail produces a finite non-zero self-aligning moment',()=>{const e=make();e.state.velocity={x:5,y:0,z:0};e.advance(1/120,{});assert.ok(e.telemetry.forces.some(f=>Number.isFinite(f.aligningMoment)&&Math.abs(f.aligningMoment)>0));assert.ok(Number.isFinite(e.state.angularVelocity.y));});
test('reset clears transient tire slip-angle memory',()=>{const e=make();e.state.velocity={x:5,y:0,z:0};e.advance(1/120,{});e.reset({x:0,y:.3,z:0});assert.ok(e._slipAngleState.every(x=>x===0));});

test('per-surface friction coefficient changes the actual tire-force limit',()=>{const build=friction=>{const base=plane();const mesh={closestPoint(...args){const hit=base.closestPoint(...args);return hit?{...hit,friction}:null;}};const e=new RigidBodyVehicle({surfaceMesh:mesh,groundPlaneY:0,position:{x:0,y:.3,z:0}});e.state.velocity={x:5,y:0,z:0};e.advance(1/120,{});return e.telemetry.forces[0];};const dry=build(1),ice=build(.2);assert.ok(ice.frictionCoefficient<dry.frictionCoefficient);assert.ok(Math.hypot(ice.Fx,ice.Fy)<Math.hypot(dry.Fx,dry.Fy));});


test('Pacejka tire curve is nonlinear and approaches a finite lateral-force peak',()=>{
 const {calculateTireForces}=require('../physics/rigidbody-vehicle-engine.js');
 const tire={mu:1,corneringStiffness:78000,pacejka:{enabled:true,B:0,C:1.3,E:.97,peak:1},frictionEllipse:{longitudinal:1,lateral:1}};
 const small=calculateTireForces(0,.025,3000,1,tire);
 const medium=calculateTireForces(0,.18,3000,1,tire);
 const large=calculateTireForces(0,1.1,3000,1,tire);
 assert.ok(Math.abs(medium.FyRaw/large.FyRaw)>0.1&&Math.abs(medium.FyRaw/large.FyRaw)<0.95,'lateral force must curve toward saturation');
 assert.ok(Math.abs(large.FyRaw)<=3000+1e-6,'Pacejka peak must respect the configured tire load limit');
 assert.ok(Math.abs(small.FyRaw/.025)>Math.abs(large.FyRaw/1.1),'effective cornering stiffness must fall near saturation');
});

test('Pacejka B/C/E and peak parameters change the tire force curve',()=>{
 const {calculateTireForces}=require('../physics/rigidbody-vehicle-engine.js');
 const base={mu:1,corneringStiffness:78000,pacejka:{enabled:true,B:0,C:1.3,E:.97,peak:1},frictionEllipse:{longitudinal:1,lateral:1}};
 const tuned={...base,pacejka:{enabled:true,B:7,C:1.1,E:.2,peak:.65}};
 const a=calculateTireForces(0,.35,3000,1,base),b=calculateTireForces(0,.35,3000,1,tuned);
 assert.notEqual(a.FyRaw,b.FyRaw);
 assert.ok(Math.abs(b.FyRaw)<=1950+1e-6);
});

test('friction ellipse couples longitudinal and lateral tire-force limits',()=>{
 const {calculateTireForces}=require('../physics/rigidbody-vehicle-engine.js');
 const tire={mu:1,corneringStiffness:78000,pacejka:{enabled:false},frictionEllipse:{longitudinal:1,lateral:1}};
 const lateralOnly=calculateTireForces(0,.12,3000,1,tire);
 const combined=calculateTireForces(2400,.12,3000,1,tire);
 assert.ok(Math.abs(combined.Fy)<Math.abs(lateralOnly.Fy),'longitudinal force must consume lateral grip');
 assert.ok(combined.utilization<=1+1e-9);
 assert.ok(lateralOnly.utilization<=1+1e-9);
});

test('friction ellipse honors separate longitudinal and lateral axes',()=>{
 const {calculateTireForces}=require('../physics/rigidbody-vehicle-engine.js');
 const tire={mu:1,corneringStiffness:78000,pacejka:{enabled:false},frictionEllipse:{longitudinal:.7,lateral:.9}};
 const f=calculateTireForces(5000,.1,3000,1,tire);
 const ellipse=Math.hypot(f.Fx/(3000*.7),f.Fy/(3000*.9));
 assert.ok(ellipse<=1+1e-9);
});


test('progressive bump stops engage before the suspension reaches its hard travel limit',()=>{
 const makeAt=(y)=>new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y,z:0}});
 const shallow=makeAt(.3),deep=makeAt(.2);
 shallow.advance(1/120,{});deep.advance(1/120,{});
 const a=shallow.telemetry.suspension[0],b=deep.telemetry.suspension[0];
 assert.ok(a.bumpStopForce>0,'bump stop should engage when raw suspension length enters the stop zone');
 assert.ok(b.bumpStopForce>a.bumpStopForce,'deeper compression should create a stronger progressive bump-stop force');
 assert.ok(Number.isFinite(b.bumpStopCompression));
});

test('suspension bump-stop settings can disable and tune end-stop force',()=>{
 const disabled=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0},config:{suspension:{bump:0}}});
 const tuned=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0},config:{suspension:{bump:180000}}});
 disabled.advance(1/120,{});tuned.advance(1/120,{});
 assert.equal(disabled.telemetry.suspension[0].bumpStopForce,0);
 assert.ok(tuned.telemetry.suspension[0].bumpStopForce>disabled.telemetry.suspension[0].bumpStopForce);
});


function axleLoads(engine){const n=engine.telemetry.normalLoads;return{front:n[0]+n[1],rear:n[2]+n[3],total:n.reduce((a,b)=>a+b,0)};}
test('braking transfers tire normal load from rear axle to front axle',()=>{
 const baseline=make(),braking=make();
 baseline.state.velocity={x:0,y:0,z:-12};braking.state.velocity={x:0,y:0,z:-12};
 baseline.advance(1/120,{});braking.advance(1/120,{brake:1});
 const a=axleLoads(baseline),b=axleLoads(braking);
 assert.ok(b.front>a.front,'braking must increase front-axle normal load');
 assert.ok(b.rear<a.rear,'braking must reduce rear-axle normal load');
 assert.ok(Math.abs(b.total-a.total)<1e-6,'load transfer must conserve total normal load');
 assert.ok(braking.telemetry.loadTransfer.longitudinal>0);
});
test('acceleration transfers tire normal load from front axle to rear axle',()=>{
 const baseline=make(),accelerating=make();
 baseline.advance(1/120,{});accelerating.advance(1/120,{throttle:1});
 const a=axleLoads(baseline),b=axleLoads(accelerating);
 assert.ok(b.rear>a.rear,'acceleration must increase rear-axle normal load');
 assert.ok(b.front<a.front,'acceleration must reduce front-axle normal load');
 assert.ok(Math.abs(b.total-a.total)<1e-6,'load transfer must conserve total normal load');
 assert.ok(accelerating.telemetry.loadTransfer.longitudinal<0);
});
test('longitudinal load transfer remains finite and cannot create negative wheel loads',()=>{
 const e=make();e.state.velocity={x:0,y:0,z:-40};e.advance(1/120,{brake:1});
 assert.ok(e.telemetry.normalLoads.every(n=>Number.isFinite(n)&&n>=0));
 assert.ok(Number.isFinite(e.telemetry.loadTransfer.longitudinal));
});


test('cornering transfers normal load to the outside wheels and conserves total load',()=>{
 const straight=make(),turning=make();
 straight.state.velocity={x:0,y:0,z:-20};turning.state.velocity={x:0,y:0,z:-20};
 straight.advance(1/120,{});turning.advance(1/120,{steer:1});
 const n=axleLoads(straight).total,m=turning.telemetry.normalLoads;
 const left=m[0]+m[2],right=m[1]+m[3];
 assert.ok(left>right,'positive steering must load the outside (left) wheels');
 assert.ok(Math.abs(m.reduce((a,b)=>a+b,0)-n)<1e-6,'lateral transfer must conserve total normal load');
 assert.ok(turning.telemetry.loadTransfer.lateral>0);
});
test('opposite steering reverses lateral load transfer direction',()=>{
 const left=make(),right=make();
 for(const e of [left,right])e.state.velocity={x:0,y:0,z:-20};
 left.advance(1/120,{steer:1});right.advance(1/120,{steer:-1});
 assert.ok(left.telemetry.loadTransfer.lateral>0);
 assert.ok(right.telemetry.loadTransfer.lateral<0);
 const l=left.telemetry.normalLoads,r=right.telemetry.normalLoads;
 assert.ok(l[0]+l[2]>l[1]+l[3]);
 assert.ok(r[1]+r[3]>r[0]+r[2]);
});
test('lateral load transfer is zero without cornering demand',()=>{
 const e=make();e.state.velocity={x:0,y:0,z:-20};e.advance(1/120,{});
 assert.equal(e.telemetry.loadTransfer.lateral,0);
});


function tiltAngle(q,axis){
 const up=require('../physics/rigidbody-vehicle-engine.js').Quaternion.rotate(q,{x:0,y:1,z:0});
 return axis==='roll'?Math.atan2(-up.x,up.y):Math.atan2(up.z,up.y);
}
test('body roll spring restores a tilted chassis toward upright',()=>{
 const {Quaternion:Q}=require('../physics/rigidbody-vehicle-engine.js');
 const e=new RigidBodyVehicle({position:{x:0,y:10,z:0}});e.state.orientation=Q.axis({x:0,y:0,z:1},.12);
 const before=Math.abs(tiltAngle(e.state.orientation,'roll'));
 for(let i=0;i<30;i++)e.advance(1/120,{});
 assert.ok(Math.abs(tiltAngle(e.state.orientation,'roll'))<before,'roll restoring torque must reduce chassis roll angle');
});
test('body pitch spring restores a pitched chassis toward level',()=>{
 const {Quaternion:Q}=require('../physics/rigidbody-vehicle-engine.js');
 const e=new RigidBodyVehicle({position:{x:0,y:10,z:0}});e.state.orientation=Q.axis({x:1,y:0,z:0},.12);
 const before=Math.abs(tiltAngle(e.state.orientation,'pitch'));
 for(let i=0;i<30;i++)e.advance(1/120,{});
 assert.ok(Math.abs(tiltAngle(e.state.orientation,'pitch'))<before,'pitch restoring torque must reduce chassis pitch angle');
});
test('body roll damping dissipates roll angular velocity',()=>{
 const e=new RigidBodyVehicle({position:{x:0,y:10,z:0}});e.state.angularVelocity={x:0,y:0,z:.8};
 for(let i=0;i<30;i++)e.advance(1/120,{});
 assert.ok(Math.abs(e.state.angularVelocity.z)<.8,'roll damping must reduce angular velocity');
});
test('body pitch damping dissipates pitch angular velocity',()=>{
 const e=new RigidBodyVehicle({position:{x:0,y:10,z:0}});e.state.angularVelocity={x:.8,y:0,z:0};
 for(let i=0;i<30;i++)e.advance(1/120,{});
 assert.ok(Math.abs(e.state.angularVelocity.x)<.8,'pitch damping must reduce angular velocity');
});
test('body roll and pitch stabilization can be independently tuned or disabled',()=>{
 const {Quaternion:Q}=require('../physics/rigidbody-vehicle-engine.js');
 const soft=new RigidBodyVehicle({position:{x:0,y:10,z:0},config:{body:{rollStiffness:1000,rollDamping:100,pitchStiffness:0,pitchDamping:0}}});
 const stiff=new RigidBodyVehicle({groundPlaneY:0,position:{x:0,y:.3,z:0},config:{body:{rollStiffness:12000,rollDamping:1000,pitchStiffness:0,pitchDamping:0}}});
 for(const e of [soft,stiff]){e.state.orientation=Q.axis({x:0,y:0,z:1},.1);e.advance(1/120,{});}
 assert.ok(Math.abs(tiltAngle(stiff.state.orientation,'roll'))<Math.abs(tiltAngle(soft.state.orientation,'roll')));
 assert.equal(soft.c.body.pitchStiffness,0);
});


test('front and rear suspension geometry use independent alignment settings',()=>{
 const e=make();e.advance(1/120,{});
 const wheels=e.telemetry.wheelGeometry;
 assert.equal(wheels.length,4);
 assert.equal(wheels[0].axle,'front');
 assert.equal(wheels[2].axle,'rear');
 assert.notEqual(wheels[0].camberDeg,wheels[2].camberDeg);
 assert.ok(wheels.every(w=>Number.isFinite(w.rollCenterHeight)&&Number.isFinite(w.toeDeg)));
});
test('camber and toe change continuously with suspension compression',()=>{
 const config={suspensionGeometry:{front:{camberStatic:-.5,camberGain:-30,toeStatic:0,toeGain:1}}};
 const low=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.52,z:0},config});
 const compressed=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.32,z:0},config});
 const a=low._contacts({})[0],b=compressed._contacts({})[0];
 assert.ok(b.compression>a.compression);
 assert.ok(b.geometry.camberDeg<a.geometry.camberDeg);
 assert.ok(b.geometry.toeDeg>a.geometry.toeDeg);
 assert.ok(Math.abs(b.geometry.camberDeg-a.geometry.camberDeg)<10,'alignment must vary smoothly, not jump');
});
test('wheel tire frame follows sloped-road tangent and remains orthonormal',()=>{
 const normal=V.norm({x:-.25,y:1,z:.12});
 const mesh={closestPoint(p,max=Infinity){const d=p.x*.25-p.y-p.z*.12;const distance=Math.abs(d)/Math.hypot(.25,1,.12);if(distance>max)return null;const point={x:p.x-normal.x*d/Math.hypot(.25,1,.12),y:p.y-normal.y*d/Math.hypot(.25,1,.12),z:p.z-normal.z*d/Math.hypot(.25,1,.12)};return{point,normal,distance,signedDistance:d/Math.hypot(.25,1,.12)};}};
 const e=new RigidBodyVehicle({surfaceMesh:mesh,position:{x:0,y:.5,z:0}});
 const h=e._contacts({steer:.2})[0];
 assert.ok(h.tireForward.x>0,'positive steering must rotate the wheel forward vector toward vehicle-right');
 assert.ok(Math.abs(V.dot(h.tireForward,h.normal))<1e-6);
 assert.ok(Math.abs(V.dot(h.tireRight,h.normal))<1e-6);
 assert.ok(Math.abs(V.dot(h.tireForward,h.tireRight))<1e-6);
 assert.ok(Math.abs(V.len(h.tireForward)-1)<1e-6);
 assert.ok(Math.abs(V.len(h.tireRight)-1)<1e-6);
});
test('roll-centre height changes the lateral load-transfer moment arm',()=>{
 const build=height=>new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.3,z:0},config:{suspensionGeometry:{front:{rollCenterHeight:height},rear:{rollCenterHeight:height}}}});
 const low=build(.05),high=build(.35);
 for(const e of [low,high]){e.state.velocity={x:0,y:0,z:-20};e.advance(1/120,{steer:.7});}
 assert.ok(high.telemetry.rollCenter.average>low.telemetry.rollCenter.average);
 assert.ok(high.telemetry.rollCenter.lateralMomentArm<low.telemetry.rollCenter.lateralMomentArm);
 assert.ok(Math.abs(high.telemetry.loadTransfer.lateral)<Math.abs(low.telemetry.loadTransfer.lateral));
});


test('hardpoint suspension geometry solves finite instant centres and roll centres per axle',()=>{
 const e=make();e.advance(1/120,{});
 const wheels=e.telemetry.wheelGeometry;
 assert.equal(wheels.length,4);
 assert.ok(wheels.every(w=>w.hardpointValid,'default hardpoint geometry should solve'));
 assert.ok(wheels.every(w=>Number.isFinite(w.rollCenterHeight)));
 assert.ok(wheels.every(w=>w.instantCenter&&Number.isFinite(w.instantCenter.x)&&Number.isFinite(w.instantCenter.y)));
 assert.ok(wheels.every(w=>w.upperOuter&&w.lowerOuter));
 assert.notEqual(wheels[0].rollCenterHeight,wheels[2].rollCenterHeight);
});
test('hardpoint motion changes upright camber and wheel lateral position with compression',()=>{
 const config={suspensionGeometry:{front:{hardpoints:{enabled:true}}}};
 const low=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.52,z:0},config});
 const compressed=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.32,z:0},config});
 const a=low._contacts({})[0],b=compressed._contacts({})[0];
 assert.ok(b.compression>a.compression);
 assert.ok(b.geometry.hardpointValid&&a.geometry.hardpointValid);
 assert.notEqual(b.geometry.camberDeg,a.geometry.camberDeg);
 assert.notEqual(b.geometry.wheelOffsetX,a.geometry.wheelOffsetX);
 assert.notEqual(b.point.x,a.point.x,'hardpoint lateral motion must move the wheel contact lever arm');
});
test('changing control-arm pickup points changes solved instant centre and roll centre',()=>{
 const build=upperX=>new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.32,z:0},config:{suspensionGeometry:{front:{hardpoints:{upperInner:{x:upperX,y:.82}}}}}});
 const a=build(.28),b=build(.38),wa=a._contacts({})[0].geometry,wb=b._contacts({})[0].geometry;
 assert.ok(wa.hardpointValid&&wb.hardpointValid);
 assert.ok(Math.abs(wa.instantCenter.x-wb.instantCenter.x)>1e-4||Math.abs(wa.instantCenter.y-wb.instantCenter.y)>1e-4);
 assert.notEqual(wa.rollCenterHeight,wb.rollCenterHeight);
});
test('hardpoint solver safely falls back when requested travel exceeds arm reach',()=>{
 const e=new RigidBodyVehicle({surfaceMesh:plane(),groundPlaneY:0,position:{x:0,y:.01,z:0}});
 const g=e._wheelKinematics(0,2,{x:0,y:1,z:0},{});
 assert.equal(g.hardpointValid,false);
 assert.ok(Number.isFinite(g.camberDeg)&&Number.isFinite(g.rollCenterHeight));
});


test('3D steering axis uses caster and kingpin inclination to change wheel heading and camber under steer',()=>{
 const make=config=>new RigidBodyVehicle({groundPlaneY:0,config});
 const base=make({suspensionGeometry:{front:{casterDeg:0,kingpinInclinationDeg:0}}});
 const angled=make({suspensionGeometry:{front:{casterDeg:7,kingpinInclinationDeg:10}}});
 const a=base._wheelKinematics(0,0,{x:0,y:1,z:0},{steer:.6});
 const b=angled._wheelKinematics(0,0,{x:0,y:1,z:0},{steer:.6});
 assert.ok(Math.abs(a.forward.y)<1e-8);
 assert.ok(Math.abs(b.camberDeg-a.camberDeg)>1e-4,'steering-axis inclination should create steering camber');
 assert.ok(Math.abs(a.forward.x-b.forward.x)>1e-4||Math.abs(a.forward.z-b.forward.z)>1e-4);
 assert.ok(Number.isFinite(b.jackingHeight)&&b.jackingHeight>0);
});
test('3D steering axis reverses symmetrically with steering input',()=>{
 const e=new RigidBodyVehicle({groundPlaneY:0,config:{suspensionGeometry:{front:{casterDeg:6,kingpinInclinationDeg:9}}}});
 const left=e._wheelKinematics(0,0,{x:0,y:1,z:0},{steer:.5});
 const right=e._wheelKinematics(0,0,{x:0,y:1,z:0},{steer:-.5});
 assert.ok(Math.abs(left.jackingHeight+right.jackingHeight)<1e-8);
 assert.ok(Math.abs(left.forward.x+right.forward.x)<.02);
 assert.ok(Math.abs(left.forward.z-right.forward.z)<.02);
});
test('scrub radius and mechanical trail are exposed as finite steering geometry telemetry',()=>{
 const e=new RigidBodyVehicle({groundPlaneY:0,config:{suspensionGeometry:{front:{scrubRadius:.045,mechanicalTrail:.065}}}});
 const g=e._wheelKinematics(0,0,{x:0,y:1,z:0},{steer:.4});
 assert.equal(g.scrubRadius,.045);
 assert.equal(g.mechanicalTrail,.065);
 assert.ok(Number.isFinite(g.steeringAxis.x)&&Number.isFinite(g.steeringAxis.y)&&Number.isFinite(g.steeringAxis.z));
});

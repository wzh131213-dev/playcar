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

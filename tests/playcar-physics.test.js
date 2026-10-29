'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const {PlayCarPhysicsEngine: Engine, TriangleMesh, Vector3: V, Quaternion: Q} =
  require('../physics/playcar-physics-engine.js');

function rectangleSurface({xMin=-15,xMax=15,zMin=-25,zMax=25,yAt=()=>0}={}){
  const a={x:xMin,y:yAt(xMin,zMin),z:zMin};
  const b={x:xMin,y:yAt(xMin,zMax),z:zMax};
  const c={x:xMax,y:yAt(xMax,zMax),z:zMax};
  const d={x:xMax,y:yAt(xMax,zMin),z:zMin};
  return TriangleMesh.fromTriangles([[a,b,c],[a,c,d]],8);
}
function makeEngine(options={}){
  return new Engine({surfaceMesh:rectangleSurface(),position:{x:0,y:-.03,z:0},...options});
}
function runFor(engine,seconds,input,fps=60){
  const frame=1/fps;let elapsed=0;
  while(elapsed+1e-9<seconds){
    const dt=Math.min(frame,seconds-elapsed);
    engine.advance(dt,input);elapsed+=dt;
  }
}
function finiteState(engine){
  const s=engine.state;
  for(const key of ['x','y','z'])assert.ok(Number.isFinite(s.position[key]),'position '+key+' must be finite');
  for(const key of ['x','y','z'])assert.ok(Number.isFinite(s.velocity[key]),'velocity '+key+' must be finite');
  for(const key of ['x','y','z','w'])assert.ok(Number.isFinite(s.orientation[key]),'orientation '+key+' must be finite');
}

test('engine accelerates under throttle on a flat 3D surface',()=>{
  const e=makeEngine();runFor(e,1,{throttle:1},60);
  assert.ok(V.len(e.state.velocity)>12,'expected meaningful acceleration');finiteState(e);
});
test('braking reduces forward speed',()=>{
  const e=makeEngine();e.state.velocity={x:0,y:0,z:-22};runFor(e,.45,{brake:1},60);
  assert.ok(V.len(e.state.velocity)<8,'braking should substantially reduce speed');finiteState(e);
});
test('left and right steering turn in the matching direction',()=>{
  const right=makeEngine();right.state.velocity={x:0,y:0,z:-20};runFor(right,.8,{right:true},60);
  const rightForward=Q.rotate(right.state.orientation,{x:0,y:0,z:-1});
  assert.ok(rightForward.x>.12,'right input should turn the nose toward +X/right');
  const left=makeEngine();left.state.velocity={x:0,y:0,z:-20};runFor(left,.8,{left:true},60);
  const leftForward=Q.rotate(left.state.orientation,{x:0,y:0,z:-1});
  assert.ok(leftForward.x<-.12,'left input should turn the nose toward -X/left');
  finiteState(right);finiteState(left);
});
test('tire grip damps lateral velocity; handbrake has a weaker grip response',()=>{
  const normal=makeEngine();normal.state.velocity={x:8,y:0,z:-12};runFor(normal,.5,{},60);
  const drift=makeEngine();drift.state.velocity={x:8,y:0,z:-12};runFor(drift,.5,{handbrake:1},60);
  assert.ok(Math.abs(normal.state.velocity.x)<8,'normal grip must oppose side slip');
  assert.ok(Math.abs(drift.state.velocity.x)>Math.abs(normal.state.velocity.x),'handbrake should retain more lateral slip');
});
test('fixed-step simulation is near-identical at 30 and 120 render frames per second',()=>{
  const a=makeEngine(),b=makeEngine();
  runFor(a,1,{throttle:1,right:true},30);runFor(b,1,{throttle:1,right:true},120);
  assert.ok(V.distance(a.state.position,b.state.position)<1e-4,'position should depend on fixed steps, not render cadence');
  assert.ok(V.distance(a.state.velocity,b.state.velocity)<1e-4,'velocity should depend on fixed steps, not render cadence');
});
test('leaving the end of a finite track produces airborne gravity and falling vertical speed',()=>{
  const surface=rectangleSurface({xMin:-10,xMax:10,zMin:-5,zMax:25});
  const e=new Engine({surfaceMesh:surface,position:{x:0,y:-.03,z:0}});
  e.state.velocity={x:0,y:0,z:-24};runFor(e,.9,{},60);
  assert.equal(e.state.grounded,false,'wheels should lose the finite road surface');
  assert.ok(e.state.velocity.y < -2,'gravity should accelerate the car downward');
  finiteState(e);
});
test('road is solid through wheel support without false body-wall collisions',()=>{
  const road=rectangleSurface();
  const e=new Engine({surfaceMesh:road,colliderMeshes:[],position:{x:0,y:.12,z:0}});
  runFor(e,.3,{},60);
  assert.equal(e.state.grounded,true,'wheel/suspension contacts should keep the car on the road');
  assert.equal(e.state.totalCollisionEvents,0,'the road must not be misclassified as a wall impact');
  assert.ok(e.state.position.y>-.12,'wheel support must prevent the car from falling through the road');
  finiteState(e);
});
test('solid chassis collision prevents the body from passing through the road',()=>{
  const e=new Engine({surfaceMesh:rectangleSurface(),colliderMeshes:[],position:{x:0,y:-.4,z:0}});
  const before=e.state.position.y;
  assert.equal(e._resolveSurfacePenetration(),true,'the chassis should detect road penetration');
  assert.ok(e.state.position.y>before+.3,'surface collision should push the chassis back above the road');
  assert.ok(e.state.position.y>-.05,'the chassis should not remain below the road surface');
  assert.equal(e.state.totalCollisionEvents,0,'road contact should not be reported as a wall collision');
  finiteState(e);
});
test('the world ground plane catches a car that falls off the road',()=>{
  const e=new Engine({surfaceMesh:null,colliderMeshes:[],groundPlaneY:-2,position:{x:0,y:-2.9,z:0}});
  e.state.velocity={x:0,y:-8,z:0};
  runFor(e,.5,{});
  assert.ok(e.state.position.y>-2.2,'the solid ground plane should keep the chassis above ground');
  assert.equal(e.state.velocity.y,0,'vertical velocity should be stopped by the ground plane');
  assert.equal(e.state.grounded,true,'the car should be supported after landing on the world ground');
  finiteState(e);
});
test('a falling car contacts the road and recovers from a landing',()=>{
  const e=makeEngine();e.state.position.y=2;e.state.velocity={x:0,y:-12,z:0};
  runFor(e,.75,{},60);
  assert.equal(e.state.grounded,true,'car should reconnect with the road');
  assert.ok(e.state.position.y>-.2,'contact correction should prevent continued tunnelling through the road');
  finiteState(e);
});
test('a car that penetrates below the road is pushed back onto the solid surface',()=>{
  const e=makeEngine();e.state.position.y=-.62;e.state.velocity={x:0,y:-2,z:0};
  runFor(e,.25,{},60);
  assert.ok(e.state.position.y>-.25,'signed-distance contact should recover the car from below the road');
  assert.equal(e.state.grounded,true,'recovered wheels should regain road contact');
  finiteState(e);
});

test('uphill ramp redirects forward motion into vertical lift and takeoff',()=>{
  const triangles=[];
  const fa={x:-10,y:0,z:0},fb={x:-10,y:0,z:20},fc={x:10,y:0,z:20},fd={x:10,y:0,z:0};
  triangles.push([fa,fb,fc],[fa,fc,fd]);
  const ra={x:-10,y:0,z:0},rb={x:-10,y:3,z:-6},rc={x:10,y:3,z:-6},rd={x:10,y:0,z:0};
  triangles.push([ra,rd,rc],[ra,rc,rb]);
  const surface=TriangleMesh.fromTriangles(triangles,4);
  const e=new Engine({surfaceMesh:surface,position:{x:0,y:-.03,z:4}});
  e.state.velocity={x:0,y:0,z:-20};
  let lifted=false;
  for(let i=0;i<90;i++){
    e.advance(1/120,{});
    if(!e.state.grounded&&e.state.velocity.y>1)lifted=true;
  }
  assert.ok(lifted,'vehicle should retain upward momentum after leaving the rising ramp');
  finiteState(e);
});
test('crossing a checkpoint plane updates the recoverable checkpoint',()=>{
  const e=makeEngine();
  e.addCheckpoint({id:'start',position:{x:0,y:0,z:0},forward:{x:0,y:0,z:-1},radius:6});
  e.addCheckpoint({id:'gate-1',position:{x:0,y:0,z:-3},forward:{x:0,y:0,z:-1},radius:6});
  e.state.position={x:0,y:-.03,z:0};e.state.velocity={x:0,y:0,z:-20};
  runFor(e,.3,{},60);
  assert.equal(e.state.checkpointId,'gate-1');
  assert.equal(e.resetToCheckpoint(),true);
  assert.equal(e.state.checkpointId,'gate-1');
  assert.ok(Math.abs(e.state.position.z+3)<1e-9);
});
test('3D triangle collision detects and resolves a vertical wall',()=>{
  const ground=rectangleSurface();
  const a={x:-10,y:0,z:-3},b={x:-10,y:5,z:-3},c={x:10,y:5,z:-3},d={x:10,y:0,z:-3};
  const wall=TriangleMesh.fromTriangles([[a,b,c],[a,c,d]],4);
  const e=makeEngine({colliderMeshes:[wall]});e.state.velocity={x:0,y:0,z:-16};
  runFor(e,.45,{},60);
  assert.ok(e.state.totalCollisionEvents>0,'wall triangles should generate a persistent collision event');
  assert.ok(e.state.position.z>-5,'vehicle should be separated from wall instead of passing through');
  finiteState(e);
});
test('3D loop mesh supplies continuous collision-surface samples around the full loop',()=>{
  const triangles=[],segments=48,radius=7,width=3,centerY=8;
  const at=(x,t)=>({x,y:centerY+radius*Math.cos(t),z:radius*Math.sin(t)});
  for(let i=0;i<segments;i++){
    const t0=i/segments*Math.PI*2,t1=(i+1)/segments*Math.PI*2;
    const a=at(-width,t0),b=at(width,t0),c=at(width,t1),d=at(-width,t1);
    triangles.push([a,b,c],[a,c,d]);
  }
  const loop=TriangleMesh.fromTriangles(triangles,3);
  for(let i=0;i<segments;i++){
    const t=(i+.5)/segments*Math.PI*2,p={x:0,y:centerY+radius*Math.cos(t),z:radius*Math.sin(t)};
    const hit=loop.closestPoint(p,.18);
    assert.ok(hit&&hit.distance<.18,'loop surface should remain queryable at segment '+i);
    assert.ok(Number.isFinite(hit.normal.x+hit.normal.y+hit.normal.z),'loop surface normal should be finite');
  }
});
test('reset restores position, velocity, orientation and checkpoint identity',()=>{
  const e=makeEngine();
  e.addCheckpoint({id:'start',position:{x:0,y:0,z:0},forward:{x:0,y:0,z:-1},radius:5});
  e.addCheckpoint({id:'gate-1',position:{x:0,y:0,z:-10},forward:{x:0,y:0,z:-1},radius:5});
  e.state.velocity={x:5,y:4,z:-9};e.setCheckpoint('gate-1');
  assert.equal(e.resetToCheckpoint(),true);
  assert.equal(e.state.checkpointId,'gate-1');assert.deepEqual(e.state.velocity,{x:0,y:0,z:0});
  assert.ok(Math.abs(e.state.position.z+10)<1e-9);finiteState(e);
});
test('large render delta is clamped and cannot create an unstable physics step',()=>{
  const e=makeEngine();const r=e.advance(1.2,{throttle:1});
  assert.ok(r.steps<=e.config.maxSubsteps,'substep budget should be enforced');
  assert.ok(r.droppedTime>=0,'dropped backlog should be reported');finiteState(e);
});


test('weather setter clamps wetness and wind to safe finite ranges',()=>{
  const e=makeEngine();const w=e.setWeather({wetness:4,wind:{x:100,z:-100}});
  assert.equal(w.wetness,1);assert.equal(w.wind.x,20);assert.equal(w.wind.z,-20);
  const clean=e.setWeather({wetness:NaN,wind:{x:Infinity,z:NaN}});
  assert.equal(clean.wetness,0);assert.deepEqual(clean.wind,{x:0,z:0});
});
test('crosswind changes vehicle velocity in the wind direction',()=>{
  const calm=makeEngine(),windy=makeEngine();calm.state.position.y=15;windy.state.position.y=15;calm.state.velocity={x:0,y:0,z:-12};windy.state.velocity={x:0,y:0,z:-12};
  windy.setWeather({wind:{x:10,z:0},wetness:0});runFor(calm,.8,{},60);runFor(windy,.8,{});
  assert.ok(windy.state.velocity.x>calm.state.velocity.x+.5,'crosswind should apply measurable lateral force while airborne');finiteState(windy);
});
test('wet road reduces tire grip and increases retained side slip',()=>{
  const dry=makeEngine(),wet=makeEngine();dry.state.velocity={x:8,y:0,z:-12};wet.state.velocity={x:8,y:0,z:-12};wet.setWeather({wetness:1});
  runFor(dry,.5,{},60);runFor(wet,.5,{});assert.ok(Math.abs(wet.state.velocity.x)>Math.abs(dry.state.velocity.x),'wet tires should retain more lateral slip');finiteState(wet);
});


test('game HTML never updates removed debug panel elements in the render loop',()=>{
  const fs=require('node:fs');
  const html=fs.readFileSync(require('node:path').join(__dirname,'..','index.html'),'utf8');
  for(const id of ['debugSpeed','debugVertical','debugGrip','debugGrounded','debugCollision','debugMs','debugTick','debugEvents','debugDropped']){
    assert.equal(html.includes("$('"+id+"')"),false,'render loop must not access removed #'+id);
  }
  assert.match(html,/function animate\(now\)[\s\S]*?update\(dt\);renderer\.render\(scene,camera\)/);
  assert.match(html,/function updateWeather\(dt\)/);
});

test('grounded braking is limited by tire friction instead of raw brake strength',()=>{
  const e=makeEngine();
  e.state.velocity={x:0,y:0,z:-20};
  const before=V.dot(e.state.velocity,Q.rotate(e.state.orientation,{x:0,y:0,z:-1}));
  e.advance(1/120,{brake:1});
  const forward=Q.rotate(e.state.orientation,{x:0,y:0,z:-1});
  const after=V.dot(e.state.velocity,forward);
  const deceleration=(before-after)/(1/120);
  const nonTireDrag=(e.config.rollingResistance+e.config.aerodynamicDrag*before)*before;
  const load=e.config.gravity+Math.min(e.config.maxDownforce,before*before*e.config.downforceCoefficient);
  const tireOnlyDeceleration=deceleration-nonTireDrag;
  assert.ok(tireOnlyDeceleration <= e.config.tireGrip*load+1,
    'tire braking force should respect grip after accounting for aerodynamic and rolling drag');
  finiteState(e);
});
test('holding brake from rest engages reverse like an arcade racer',()=>{
  const e=makeEngine();
  runFor(e,.65,{brake:1},60);
  const forward=Q.rotate(e.state.orientation,{x:0,y:0,z:-1});
  const longitudinal=V.dot(e.state.velocity,forward);
  assert.ok(longitudinal < -2,'brake held from rest should move the car backward');
  finiteState(e);
});
test('speed-dependent downforce increases available braking grip at speed',()=>{
  function initialDeceleration(speed){
    const e=makeEngine();
    e.state.velocity={x:0,y:0,z:-speed};
    const before=V.len(e.state.velocity);
    e.advance(1/120,{brake:1});
    return (before-V.len(e.state.velocity))/(1/120);
  }
  const low=initialDeceleration(10), high=initialDeceleration(45);
  assert.ok(high>low+1.5,'higher speed should add downforce and increase tire load');
});

test('game HTML requests the current physics-engine cache version',()=>{
  const fs=require('node:fs');
  const path=require('node:path');
  const html=fs.readFileSync(path.join(__dirname,'../index.html'),'utf8');
  const api=require('../physics/playcar-physics-engine.js');
  assert.equal(api.version,'1.2.0','behavior changes must bump the engine version');
  assert.ok(html.includes(`./physics/playcar-physics-engine.js?v=${api.version}`),
    'HTML cache-bust version must match the engine version');
});

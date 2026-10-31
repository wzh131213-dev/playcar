/* PlayCar Physics Engine + Three.js Scene - single-file build 3.0.0.
 * GENERATED FILE - edit physics/*.js and run "npm run build" instead.
 *
 * Sections:
 *   1. Legacy kinematic compatibility engine (PlayCarPhysicsEngine 1.8 API)
 *   2. Rigid-body core (broadphase, manifolds, sequential-impulse solver)
 *   3. Rigid-body vehicle (suspension, combined-slip tires, drivetrain, aids)
 *   4. Unified runtime facade
 *   5. Weather model + Three.js weather visuals
 *   6. Three.js driving lab
 */

/* PlayCar Physics Engine 1.8.0
 * Original, dependency-free vehicle dynamics and triangle collision solver.
 * Units are game-world units; callers own rendering and track geometry.
 * No PolyTrack code, WASM, assets, or formulas are used.
 */
(function (root) {
  'use strict';

  const EPS = 1e-8;
  const V = {
    make: (x=0,y=0,z=0) => ({x,y,z}),
    add: (a,b) => ({x:a.x+b.x,y:a.y+b.y,z:a.z+b.z}),
    sub: (a,b) => ({x:a.x-b.x,y:a.y-b.y,z:a.z-b.z}),
    mul: (a,s) => ({x:a.x*s,y:a.y*s,z:a.z*s}),
    dot: (a,b) => a.x*b.x+a.y*b.y+a.z*b.z,
    cross: (a,b) => ({x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x}),
    len2: a => a.x*a.x+a.y*a.y+a.z*a.z,
    len: a => Math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z),
    norm: a => { const l=Math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z); return l>EPS?{x:a.x/l,y:a.y/l,z:a.z/l}:{x:0,y:0,z:0}; },
    distance: (a,b) => Math.sqrt((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2),
    clamp: (v,lo,hi) => Math.max(lo,Math.min(hi,v))
  };

  const Q = {
    identity: () => ({x:0,y:0,z:0,w:1}),
    normalize: q => { const n=Math.hypot(q.x,q.y,q.z,q.w)||1; return {x:q.x/n,y:q.y/n,z:q.z/n,w:q.w/n}; },
    mul: (a,b) => ({
      x:a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
      y:a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
      z:a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,
      w:a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z
    }),
    axis: (axis,angle) => { const n=V.norm(axis), s=Math.sin(angle*.5); return {x:n.x*s,y:n.y*s,z:n.z*s,w:Math.cos(angle*.5)}; },
    rotate: (q,v) => {
      const u={x:q.x,y:q.y,z:q.z}, uv=V.cross(u,v), uuv=V.cross(u,uv);
      return V.add(v,V.add(V.mul(uv,2*q.w),V.mul(uuv,2)));
    },
    inverse: q => ({x:-q.x,y:-q.y,z:-q.z,w:q.w}),
    fromForwardUp: (forward,up) => {
      let u=V.norm(up); if(V.len2(u)<EPS) u=V.make(0,1,0);
      let f=V.sub(forward,V.mul(u,V.dot(forward,u))); f=V.norm(f);
      if(V.len2(f)<EPS) f=V.norm(Math.abs(u.y)<.9?V.cross(u,V.make(0,1,0)):V.cross(u,V.make(1,0,0)));
      let r=V.norm(V.cross(f,u)); u=V.norm(V.cross(r,f));
      const m00=r.x,m01=u.x,m02=-f.x,m10=r.y,m11=u.y,m12=-f.y,m20=r.z,m21=u.z,m22=-f.z;
      const tr=m00+m11+m22; let x,y,z,w;
      if(tr>0){const s=Math.sqrt(tr+1)*2;w=.25*s;x=(m21-m12)/s;y=(m02-m20)/s;z=(m10-m01)/s;}
      else if(m00>m11&&m00>m22){const s=Math.sqrt(1+m00-m11-m22)*2;w=(m21-m12)/s;x=.25*s;y=(m01+m10)/s;z=(m02+m20)/s;}
      else if(m11>m22){const s=Math.sqrt(1+m11-m00-m22)*2;w=(m02-m20)/s;x=(m01+m10)/s;y=.25*s;z=(m12+m21)/s;}
      else {const s=Math.sqrt(1+m22-m00-m11)*2;w=(m10-m01)/s;x=(m02+m20)/s;y=(m12+m21)/s;z=.25*s;}
      return Q.normalize({x,y,z,w});
    },
    slerp: (a,b,t) => {
      let cos=a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w, end=b;
      if(cos<0){cos=-cos;end={x:-b.x,y:-b.y,z:-b.z,w:-b.w};}
      if(cos>.9995) return Q.normalize({x:a.x+(end.x-a.x)*t,y:a.y+(end.y-a.y)*t,z:a.z+(end.z-a.z)*t,w:a.w+(end.w-a.w)*t});
      const theta=Math.acos(V.clamp(cos,-1,1)),s=Math.sin(theta),wa=Math.sin((1-t)*theta)/s,wb=Math.sin(t*theta)/s;
      return Q.normalize({x:a.x*wa+end.x*wb,y:a.y*wa+end.y*wb,z:a.z*wa+end.z*wb,w:a.w*wa+end.w*wb});
    }
  };

  function closestPointOnTriangle(p,a,b,c){
    const ab=V.sub(b,a),ac=V.sub(c,a),ap=V.sub(p,a),d1=V.dot(ab,ap),d2=V.dot(ac,ap);
    if(d1<=0&&d2<=0)return a;
    const bp=V.sub(p,b),d3=V.dot(ab,bp),d4=V.dot(ac,bp);
    if(d3>=0&&d4<=d3)return b;
    const vc=d1*d4-d3*d2;
    if(vc<=0&&d1>=0&&d3<=0)return V.add(a,V.mul(ab,d1/(d1-d3)));
    const cp=V.sub(p,c),d5=V.dot(ab,cp),d6=V.dot(ac,cp);
    if(d6>=0&&d5<=d6)return c;
    const vb=d5*d2-d1*d6;
    if(vb<=0&&d2>=0&&d6<=0)return V.add(a,V.mul(ac,d2/(d2-d6)));
    const va=d3*d6-d5*d4;
    if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0)return V.add(b,V.mul(V.sub(c,b),(d4-d3)/((d4-d3)+(d5-d6))));
    const den=1/(va+vb+vc),v=vb*den,w=vc*den;
    return V.add(a,V.add(V.mul(ab,v),V.mul(ac,w)));
  }

  class TriangleMesh {
    constructor(triangles=[],cellSize=8){
      this.cellSize=cellSize;this.triangles=[];this.cells=new Map();
      for(const tri of triangles)this.addTriangle(tri[0],tri[1],tri[2],typeof tri[3]==='number'?tri[3]:(tri[3]?.friction??tri.friction??1));
    }
    static fromTriangles(triangles,cellSize=8){return new TriangleMesh(triangles,cellSize);}
    static fromHeightfield(options={},cellSize=8){
      const width=Math.max(.001,Number(options.width)||100),depth=Math.max(.001,Number(options.depth)||100);
      const segmentsX=Math.max(1,Math.floor(options.segmentsX||32)),segmentsZ=Math.max(1,Math.floor(options.segmentsZ||32));
      const sourceOrigin=options.origin||{},origin={x:Number.isFinite(sourceOrigin.x)?sourceOrigin.x:0,
        y:Number.isFinite(sourceOrigin.y)?sourceOrigin.y:0,z:Number.isFinite(sourceOrigin.z)?sourceOrigin.z:0};
      const heightAt=typeof options.heightAt==='function'?options.heightAt:()=>0;
      const triangles=[],dx=width/segmentsX,dz=depth/segmentsZ;
      const point=(ix,iz)=>{
        const x=origin.x-width/2+ix*dx,z=origin.z-depth/2+iz*dz;
        const y=Number(heightAt(x,z,ix,iz));
        if(!Number.isFinite(y))throw new TypeError('TriangleMesh.fromHeightfield: heightAt must return a finite number');
        return {x,y:y+origin.y,z};
      };
      for(let iz=0;iz<segmentsZ;iz++)for(let ix=0;ix<segmentsX;ix++){
        const a=point(ix,iz),b=point(ix,iz+1),c=point(ix+1,iz+1),d=point(ix+1,iz);
        triangles.push([a,b,c],[a,c,d]);
      }
      return new TriangleMesh(triangles,cellSize);
    }
    static fromBufferGeometry(geometry,matrixElements=null,cellSize=8){
      const attr=geometry&&geometry.attributes&&geometry.attributes.position;
      if(!attr)throw new Error('TriangleMesh: geometry has no position attribute');
      const a=attr.array,indices=geometry.index?geometry.index.array:null;
      const count=indices?indices.length:attr.count;
      const out=[];
      const point=i=>{
        const j=i*attr.itemSize;let p={x:a[j],y:a[j+1],z:a[j+2]};
        if(matrixElements){
          const e=matrixElements,x=p.x,y=p.y,z=p.z;
          const w=e[3]*x+e[7]*y+e[11]*z+e[15]||1;
          p={x:(e[0]*x+e[4]*y+e[8]*z+e[12])/w,y:(e[1]*x+e[5]*y+e[9]*z+e[13])/w,z:(e[2]*x+e[6]*y+e[10]*z+e[14])/w};
        }
        return p;
      };
      for(let i=0;i+2<count;i+=3)out.push([point(indices?indices[i]:i),point(indices?indices[i+1]:i+1),point(indices?indices[i+2]:i+2)]);
      return new TriangleMesh(out,cellSize);
    }
    _key(x,y,z){return x+','+y+','+z;}
    _cellRange(min,max){
      const s=this.cellSize;
      return [Math.floor(min.x/s),Math.floor(max.x/s),Math.floor(min.y/s),Math.floor(max.y/s),Math.floor(min.z/s),Math.floor(max.z/s)];
    }
    _insert(tri){
      const r=this._cellRange(tri.min,tri.max);
      for(let x=r[0];x<=r[1];x++)for(let y=r[2];y<=r[3];y++)for(let z=r[4];z<=r[5];z++){
        const key=this._key(x,y,z);let bucket=this.cells.get(key);if(!bucket){bucket=[];this.cells.set(key,bucket);}bucket.push(tri.id);
      }
    }
    addTriangle(a,b,c,friction=1){
      friction=Number.isFinite(Number(friction))?Math.max(0,Number(friction)):1;
      const face=V.norm(V.cross(V.sub(b,a),V.sub(c,a)));
      const tri={id:this.triangles.length,a:{...a},b:{...b},c:{...c},friction,rawNormal:face,
        centroid:V.mul(V.add(V.add(a,b),c),1/3),
        min:{x:Math.min(a.x,b.x,c.x),y:Math.min(a.y,b.y,c.y),z:Math.min(a.z,b.z,c.z)},
        max:{x:Math.max(a.x,b.x,c.x),y:Math.max(a.y,b.y,c.y),z:Math.max(a.z,b.z,c.z)}};
      this.triangles.push(tri);this._insert(tri);return tri.id;
    }
    queryAABB(min,max){
      const r=this._cellRange(min,max),seen=new Set(),out=[];
      for(let x=r[0];x<=r[1];x++)for(let y=r[2];y<=r[3];y++)for(let z=r[4];z<=r[5];z++){
        const bucket=this.cells.get(this._key(x,y,z));if(!bucket)continue;
        for(const id of bucket)if(!seen.has(id)){seen.add(id);out.push(this.triangles[id]);}
      }
      return out;
    }
    closestPoint(p,maxDistance=Infinity,preferredNormal=null){
      let candidates;
      if(!Number.isFinite(maxDistance))candidates=this.triangles;
      else{
        const r=Math.ceil(maxDistance/this.cellSize)+1,s=this.cellSize,cx=Math.floor(p.x/s),cy=Math.floor(p.y/s),cz=Math.floor(p.z/s),seen=new Set();candidates=[];
        for(let x=cx-r;x<=cx+r;x++)for(let y=cy-r;y<=cy+r;y++)for(let z=cz-r;z<=cz+r;z++){
          const bucket=this.cells.get(this._key(x,y,z));if(!bucket)continue;
          for(const id of bucket)if(!seen.has(id)){seen.add(id);candidates.push(this.triangles[id]);}
        }
      }
      let best=null,best2=maxDistance*maxDistance;
      for(const tri of candidates){
        const q=closestPointOnTriangle(p,tri.a,tri.b,tri.c),d2=V.len2(V.sub(p,q));
        if(d2<=best2){
          // Keep the closest triangle, not merely the last triangle within range.
          // Without updating best2, wheel support can jump between arbitrary road
          // triangles and appear to lose the solid ground on long/curved sections.
          best2=d2;
          // Orient the contact normal consistently with the previous support normal.
          // Road triangles may be wound either way; trusting rawNormal can make the
          // suspension push the car through the road and invert steering on banked sections.
          let n=tri.rawNormal;
          if(preferredNormal&&V.len2(preferredNormal)>EPS&&V.dot(n,preferredNormal)<0)n=V.mul(n,-1);
          best={point:q,normal:n,distance:Math.sqrt(d2),signedDistance:V.dot(V.sub(p,q),n),triangleIndex:tri.id,friction:tri.friction,face:tri};
        }
      }
      return best;
    }
  }

  function cloneState(s){
    return {position:{...s.position},velocity:{...s.velocity},orientation:{...s.orientation},
      angularVelocity:{...s.angularVelocity},grounded:!!s.grounded,contactNormal:{...s.contactNormal},
      collisionState:s.collisionState,collisionCount:s.collisionCount,collisionTimer:s.collisionTimer||0,totalCollisionEvents:s.totalCollisionEvents||0,checkpointId:s.checkpointId,
      gripState:s.gripState,tick:s.tick};
  }

  function triangleBoxOverlap(tri,half){
    const vertices=[tri.a,tri.b,tri.c],edges=[V.sub(tri.b,tri.a),V.sub(tri.c,tri.b),V.sub(tri.a,tri.c)];
    const axes=[V.make(1,0,0),V.make(0,1,0),V.make(0,0,1),V.cross(edges[0],edges[1])];
    const basis=[V.make(1,0,0),V.make(0,1,0),V.make(0,0,1)];
    for(const edge of edges)for(const axis of basis)axes.push(V.cross(edge,axis));
    let minOverlap=Infinity,minAxis=null;
    for(let axis of axes){
      const length=V.len(axis);if(length<EPS)continue;axis=V.mul(axis,1/length);
      const p0=V.dot(vertices[0],axis),p1=V.dot(vertices[1],axis),p2=V.dot(vertices[2],axis);
      const lo=Math.min(p0,p1,p2),hi=Math.max(p0,p1,p2),radius=half.x*Math.abs(axis.x)+half.y*Math.abs(axis.y)+half.z*Math.abs(axis.z);
      const overlap=Math.min(radius,hi)-Math.max(-radius,lo);
      if(overlap<0)return null;
      if(overlap<minOverlap){minOverlap=overlap;minAxis=axis;}
    }
    const centroid=V.mul(V.add(V.add(tri.a,tri.b),tri.c),1/3);
    if(V.dot(centroid,minAxis)>0)minAxis=V.mul(minAxis,-1);
    return {axis:minAxis,penetration:minOverlap};
  }

  const DEFAULTS={
    fixedTimeStep:1/120,maxSubsteps:16,maxFrameDelta:.12,
    gravity:9.81,maxSpeed:60,engineAcceleration:5.2,brakeAcceleration:9.5,
    rollingResistance:.12,aerodynamicDrag:.0022,windInfluence:.16,
    wheelbase:2.7,maxSteerAngle:.48,highSpeedSteerScale:.48,
    tireGrip:1.05,handbrakeGrip:.72,handbrakeYawMultiplier:1.35,
    reverseAcceleration:2.2,downforceCoefficient:.0014,maxDownforce:10,
    airYawRate:.62,airPitchRate:.55,airRollRate:.82,airPitchStability:1.8,
    wheelbaseHalf:1.35,trackHalfWidth:.78,wheelProbeY:.07,
    suspensionRestLength:.09,suspensionReach:.9,contactPositionGain:1,
    collisionRestitution:.16,collisionFriction:.42,
    surfaceCollisionReach:.9,surfaceSeparation:.012,surfaceCollisionRestitution:.02,maxSurfaceCollisionIterations:4,
    bodyHalfExtents:{x:1.05,y:.8,z:2.1},bodyCenterY:.84,
    maxCollisionIterations:3
  };

  // Profiles tune handling feel; all units remain game-world units unless the host
  // deliberately chooses a metre-based scale and configures the matching values.
  const PRESETS={
    arcade:{engineAcceleration:7.2,brakeAcceleration:12,tireGrip:1.15,handbrakeGrip:.78,
      maxSteerAngle:.55,highSpeedSteerScale:.58,rollingResistance:.18,aerodynamicDrag:.0018,
      airYawRate:.72,airPitchStability:2.2},
    simcade:{},
    simulation:{gravity:9.81,maxSpeed:55,engineAcceleration:4.6,brakeAcceleration:8.5,
      rollingResistance:.1,aerodynamicDrag:.0028,tireGrip:1.0,handbrakeGrip:.68,
      maxSteerAngle:.42,highSpeedSteerScale:.32,downforceCoefficient:.0009,maxDownforce:8,
      airYawRate:.3,airPitchRate:.28,airRollRate:.38,airPitchStability:1.1,
      collisionRestitution:.08,collisionFriction:.55}
  };

  class PlayCarPhysicsEngine {
    constructor(options={}){
      const presetName=typeof options.preset==='string'&&PRESETS[options.preset]?options.preset:'simcade';
      this.preset=presetName;
      this.config=Object.assign({},DEFAULTS,PRESETS[presetName],options.config||{});
      this.config.bodyHalfExtents=Object.assign({},DEFAULTS.bodyHalfExtents,PRESETS[presetName].bodyHalfExtents||{},(options.config||{}).bodyHalfExtents||{});
      this.surfaceMesh=options.surfaceMesh||null;
      this.groundPlaneY=Number.isFinite(options.groundPlaneY)?options.groundPlaneY:null;
      this.colliderMeshes=Array.isArray(options.colliderMeshes)?options.colliderMeshes.slice():[];
      this.frameAt=typeof options.frameAt==='function'?options.frameAt:null;
      this.weather={wind:{x:0,z:0},wetness:0};
      this.accumulator=0;this.tick=0;this.droppedTime=0;this.lastStepMs=0;this.totalStepMs=0;
      this.lastInput=this._normalizeInput({});
      this.telemetry={longitudinalSpeed:0,lateralSpeed:0,slipAngle:0,downforce:0,airborne:false,wheelContacts:0};
      this.checkpoints=[];this.currentCheckpointIndex=0;
      this.state={position:V.make(0,0,0),velocity:V.make(0,0,0),orientation:Q.identity(),
        angularVelocity:V.make(0,0,0),grounded:false,contactNormal:V.make(0,1,0),collisionState:'none',
        collisionCount:0,collisionTimer:0,totalCollisionEvents:0,checkpointId:'start',gripState:'AIR',tick:0};
      this.previousState=cloneState(this.state);
      if(options.position)this.reset(options.position,options.yaw||0);
    }
    setWeather(weather={}){
      const wind=weather.wind||{};
      const prev=this.weather||{};
      const num=(value,fallback,min,max)=>{
        const n=Number(value);
        return Number.isFinite(n)?V.clamp(n,min,max):fallback;
      };
      this.weather={
        wind:{x:num(wind.x,0,-20,20),z:num(wind.z,0,-20,20)},
        wetness:num(weather.wetness,0,0,1),
        rain:num(weather.rain,prev.rain||0,0,1),
        snow:num(weather.snow,prev.snow||0,0,1),
        fog:num(weather.fog,prev.fog||0,0,1),
        temperature:num(weather.temperature,Number.isFinite(prev.temperature)?prev.temperature:20,-40,60)
      };
      return {...this.weather,wind:{...this.weather.wind}};
    }
    _normalizeInput(input={}){
      const finite=(value,fallback=0)=>Number.isFinite(Number(value))?Number(value):fallback;
      const bool=(value)=>value===true||value===1;
      const throttle=V.clamp(finite(input.throttle,finite(input.accelerate,bool(input.gas)?1:0)),0,1);
      const brake=V.clamp(finite(input.brake,0),0,1);
      const steerAxis=finite(input.steer,finite(input.steering,NaN));
      const steer=Number.isFinite(steerAxis)?V.clamp(steerAxis,-1,1):V.clamp((bool(input.right)?1:0)-(bool(input.left)?1:0),-1,1);
      return {throttle,gas:throttle>0,brake,handbrake:V.clamp(finite(input.handbrake,finite(input.hand,bool(input.hand)?1:0)),0,1),
        steer,left:steer<0,right:steer>0,pitch:V.clamp(finite(input.pitch,0),-1,1),
        roll:V.clamp(finite(input.roll,0),-1,1),yaw:V.clamp(finite(input.yaw,0),-1,1)};
    }
    setInput(input={}){
      this.lastInput=this._normalizeInput(input);
      return {...this.lastInput};
    }
    update(frameDelta,input){
      return this.advance(frameDelta,input);
    }
    step(frameDelta,input){
      return this.advance(frameDelta,input);
    }
    setSurfaceMesh(mesh){this.surfaceMesh=mesh||null;return this;}
    setColliderMeshes(meshes=[]){this.colliderMeshes=Array.isArray(meshes)?meshes.slice():[];return this;}
    addColliderMesh(mesh){if(mesh&&typeof mesh.queryAABB==='function')this.colliderMeshes.push(mesh);return this;}
    applyImpulse(impulse,worldPoint=null){
      if(!impulse||![impulse.x,impulse.y,impulse.z].every(Number.isFinite))return false;
      this.state.velocity=V.add(this.state.velocity,impulse);
      if(worldPoint&&[worldPoint.x,worldPoint.y,worldPoint.z].every(Number.isFinite)){
        const center=V.add(this.state.position,Q.rotate(this.state.orientation,V.make(0,this.config.bodyCenterY,0)));
        const torque=V.cross(V.sub(worldPoint,center),impulse);
        const mass=Math.max(.01,this.config.mass||1);
        this.state.angularVelocity=V.add(this.state.angularVelocity,V.mul(torque,1/mass));
      }
      return true;
    }
    setVelocity(velocity){
      if(!velocity||![velocity.x,velocity.y,velocity.z].every(Number.isFinite))return false;
      this.state.velocity={x:velocity.x,y:velocity.y,z:velocity.z};return true;
    }
    setConfig(patch={}) {
      if(!patch || typeof patch!=='object' || Array.isArray(patch)) throw new TypeError('setConfig expects an object');
      const next={...this.config};
      for(const [key,value] of Object.entries(patch)){
        if(key==='bodyHalfExtents'){
          if(!value || typeof value!=='object') throw new TypeError('bodyHalfExtents must be an object');
          const body={...next.bodyHalfExtents};
          for(const axis of ['x','y','z']) if(value[axis]!==undefined){
            const n=Number(value[axis]); if(!Number.isFinite(n)||n<=0) throw new RangeError('bodyHalfExtents.'+axis+' must be > 0'); body[axis]=n;
          }
          next.bodyHalfExtents=body; continue;
        }
        if(!(key in DEFAULTS)) throw new RangeError('Unknown physics parameter: '+key);
        const n=Number(value); if(!Number.isFinite(n)) throw new TypeError(key+' must be finite');
        if(['fixedTimeStep','maxFrameDelta','gravity','maxSpeed','wheelbase','suspensionReach','bodyCenterY'].includes(key) && n<=0) throw new RangeError(key+' must be > 0');
        if(['maxSubsteps','maxCollisionIterations','maxSurfaceCollisionIterations'].includes(key) && (!Number.isInteger(n)||n<1)) throw new RangeError(key+' must be a positive integer');
        next[key]=n;
      }
      this.config=next; return {...this.config,bodyHalfExtents:{...this.config.bodyHalfExtents}};
    }
    getSnapshot(){
      return {version:'1.8.0',preset:this.preset,config:{...this.config,bodyHalfExtents:{...this.config.bodyHalfExtents}},
        weather:{wind:{...this.weather.wind},wetness:this.weather.wetness,
          rain:this.weather.rain||0,snow:this.weather.snow||0,fog:this.weather.fog||0,
          temperature:Number.isFinite(this.weather.temperature)?this.weather.temperature:20},
        input:{...this.lastInput},
        state:this.getState(),telemetry:this.getTelemetry(),debug:this.debugSnapshot(),
        timing:{accumulator:this.accumulator,droppedTime:this.droppedTime,lastStepMs:this.lastStepMs,totalStepMs:this.totalStepMs},
        checkpoints:this.checkpoints.map(c=>({...c,position:{...c.position},forward:{...c.forward}})),
        currentCheckpointIndex:this.currentCheckpointIndex,colliderMeshCount:this.colliderMeshes.length,
        surfaceTriangleCount:this.surfaceMesh?.triangles?.length||0};
    }
    getState(){return cloneState(this.state);}
    getTelemetry(){
      const s=this.state,forward=Q.rotate(s.orientation,V.make(0,0,-1)),right=Q.rotate(s.orientation,V.make(1,0,0));
      const longitudinalSpeed=V.dot(s.velocity,forward),lateralSpeed=V.dot(s.velocity,right);
      return {...this.telemetry,speed:V.len(s.velocity),longitudinalSpeed,lateralSpeed,
        slipAngle:Math.atan2(lateralSpeed,Math.max(1,Math.abs(longitudinalSpeed))),
        airborne:!s.grounded,grounded:s.grounded,verticalSpeed:s.velocity.y,
        contactNormal:{...s.contactNormal},position:{...s.position},velocity:{...s.velocity}};
    }
    addCheckpoint(cp){
      const forward=V.norm(cp.forward||V.make(0,0,-1));
      this.checkpoints.push({id:cp.id===undefined?'checkpoint-'+this.checkpoints.length:cp.id,
        position:{...cp.position},forward,radius:cp.radius||12});
      if(this.checkpoints.length===1){this.currentCheckpointIndex=0;this.state.checkpointId=this.checkpoints[0].id;}
    }
    setCheckpoint(id){
      const i=this.checkpoints.findIndex(c=>c.id===id);if(i>=0){this.currentCheckpointIndex=i;this.state.checkpointId=this.checkpoints[i].id;return true;}return false;
    }
    reset(position={x:0,y:0,z:0},yaw=0,up={x:0,y:1,z:0}){
      const forward={x:-Math.sin(yaw),y:0,z:-Math.cos(yaw)};
      this.accumulator=0;this.lastInput=this._normalizeInput({});this.telemetry={longitudinalSpeed:0,lateralSpeed:0,slipAngle:0,downforce:0,airborne:true,wheelContacts:0};this.state={position:{...position},velocity:V.make(0,0,0),
        orientation:Q.fromForwardUp(forward,up),angularVelocity:V.make(0,0,0),grounded:false,
        contactNormal:{...up},collisionState:'none',collisionCount:0,collisionTimer:0,totalCollisionEvents:0,checkpointId:this.checkpoints[this.currentCheckpointIndex]?.id||'start',
        gripState:'AIR',tick:this.tick};
      this.previousState=cloneState(this.state);
    }
    resetToCheckpoint(){
      const cp=this.checkpoints[this.currentCheckpointIndex];if(!cp)return false;
      this.reset({x:cp.position.x,y:cp.position.y,z:cp.position.z},
        Math.atan2(-cp.forward.x,-cp.forward.z),V.make(0,1,0));
      this.state.checkpointId=cp.id;this.previousState=cloneState(this.state);return true;
    }
    advance(frameDelta,input){
      const normalized=input===undefined?{...this.lastInput}:this.setInput(input);
      const dt=V.clamp(Number.isFinite(frameDelta)?frameDelta:0,0,this.config.maxFrameDelta);
      this.accumulator+=dt;let steps=0;
      while(this.accumulator+EPS>=this.config.fixedTimeStep&&steps<this.config.maxSubsteps){
        this._fixedStep(this.config.fixedTimeStep,normalized);this.accumulator-=this.config.fixedTimeStep;steps++;
      }
      if(this.accumulator>=this.config.fixedTimeStep){
        const keep=this.accumulator%this.config.fixedTimeStep;this.droppedTime+=this.accumulator-keep;this.accumulator=keep;
      }
      return {steps,alpha:V.clamp(this.accumulator/this.config.fixedTimeStep,0,1),droppedTime:this.droppedTime};
    }
    _wheelContacts(){
      const c=this.config,s=this.state,q=s.orientation,local=[
        V.make(-c.trackHalfWidth,c.wheelProbeY,-c.wheelbaseHalf),
        V.make(c.trackHalfWidth,c.wheelProbeY,-c.wheelbaseHalf),
        V.make(-c.trackHalfWidth,c.wheelProbeY,c.wheelbaseHalf),
        V.make(c.trackHalfWidth,c.wheelProbeY,c.wheelbaseHalf)
      ],hits=[];
      if(!this.surfaceMesh)return hits;
      for(const offset of local){
        const probe=V.add(s.position,Q.rotate(q,offset));
        const hit=this.surfaceMesh.closestPoint(probe,c.suspensionReach,s.contactNormal);
        if(hit)hits.push({...hit,probe});
      }
      return hits;
    }
    _resolveGroundPlane(){
      const c=this.config,s=this.state;
      if(!Number.isFinite(this.groundPlaneY))return false;
      const half=c.bodyHalfExtents,q=s.orientation;
      const center=V.add(s.position,Q.rotate(q,V.make(0,c.bodyCenterY,0)));
      const axisX=Q.rotate(q,V.make(1,0,0)),axisY=Q.rotate(q,V.make(0,1,0)),axisZ=Q.rotate(q,V.make(0,0,1));
      const verticalRadius=Math.abs(axisX.y)*half.x+Math.abs(axisY.y)*half.y+Math.abs(axisZ.y)*half.z;
      const bottom=center.y-verticalRadius;
      if(bottom<this.groundPlaneY+c.surfaceSeparation){
        s.position.y+=this.groundPlaneY+c.surfaceSeparation-bottom;
        if(s.velocity.y<0)s.velocity.y=0;
        if(s.collisionState==='none')s.collisionState='ground';
        return true;
      }
      return bottom<=this.groundPlaneY+c.surfaceSeparation*2;
    }
    _resolveSurfacePenetration(){
      const c=this.config,s=this.state;
      if(!this.surfaceMesh)return false;
      const half=c.bodyHalfExtents;
      let resolved=false;
      // Wheel contacts provide suspension, but they alone do not make the chassis
      // solid: a fast fall or a missed wheel sample can let the body cross the road.
      // Resolve the deepest chassis-corner penetration against the same road mesh,
      // using the local support normal so banked sections and loops remain two-sided.
      for(let iter=0;iter<c.maxSurfaceCollisionIterations;iter++){
        const q=s.orientation;
        const center=V.add(s.position,Q.rotate(q,V.make(0,c.bodyCenterY,0)));
        let deepest=null;
        for(const x of [-half.x,half.x])for(const y of [-half.y,half.y])for(const z of [-half.z,half.z]){
          const corner=V.add(center,Q.rotate(q,V.make(x,y,z)));
          const hit=this.surfaceMesh.closestPoint(corner,c.surfaceCollisionReach,s.contactNormal);
          if(!hit)continue;
          const penetration=-hit.signedDistance;
          if(penetration>c.surfaceSeparation&&(!deepest||penetration>deepest.penetration)){
            deepest={normal:hit.normal,penetration};
          }
        }
        if(!deepest)break;
        s.position=V.add(s.position,V.mul(deepest.normal,deepest.penetration+c.surfaceSeparation));
        const vn=V.dot(s.velocity,deepest.normal);
        if(vn<0)s.velocity=V.sub(s.velocity,V.mul(deepest.normal,(1+c.surfaceCollisionRestitution)*vn));
        s.contactNormal={...deepest.normal};
        resolved=true;
      }
      if(resolved&&s.collisionState==='none')s.collisionState='track';
      return resolved;
    }
    _applyStaticCollisions(){
      const c=this.config,s=this.state,half=c.bodyHalfExtents;
      for(let iter=0;iter<c.maxCollisionIterations;iter++){
        const q=s.orientation,center=V.add(s.position,Q.rotate(q,V.make(0,c.bodyCenterY,0)));
        const corners=[];
        for(const x of [-half.x,half.x])for(const y of [-half.y,half.y])for(const z of [-half.z,half.z])
          corners.push(V.add(center,Q.rotate(q,V.make(x,y,z))));
        const min={x:Math.min(...corners.map(p=>p.x)),y:Math.min(...corners.map(p=>p.y)),z:Math.min(...corners.map(p=>p.z))};
        const max={x:Math.max(...corners.map(p=>p.x)),y:Math.max(...corners.map(p=>p.y)),z:Math.max(...corners.map(p=>p.z))};
        let hitAny=false;
        for(const mesh of this.colliderMeshes){
          const tris=mesh.queryAABB(min,max);
          for(const tri of tris){
            const local={a:Q.rotate(Q.inverse(q),V.sub(tri.a,center)),b:Q.rotate(Q.inverse(q),V.sub(tri.b,center)),c:Q.rotate(Q.inverse(q),V.sub(tri.c,center))};
            const hit=triangleBoxOverlap(local,half);if(!hit)continue;
            const normal=V.norm(Q.rotate(q,hit.axis));
            s.position=V.add(s.position,V.mul(normal,hit.penetration+.002));
            const vn=V.dot(s.velocity,normal);
            if(vn<0){
              s.velocity=V.sub(s.velocity,V.mul(normal,(1+c.collisionRestitution)*vn));
              const tangent=V.sub(s.velocity,V.mul(normal,V.dot(s.velocity,normal)));
              s.velocity=V.sub(s.velocity,V.mul(tangent,c.collisionFriction));
            }
            s.collisionState='wall';s.collisionTimer=.35;s.collisionCount++;s.totalCollisionEvents=(s.totalCollisionEvents||0)+1;hitAny=true;break;
          }
          if(hitAny)break;
        }
        if(!hitAny)break;
      }
    }
    _fixedStep(dt,input){
      input=this._normalizeInput(input);
      const c=this.config,s=this.state,t0=(typeof performance!=='undefined'&&performance.now)?performance.now():Date.now();
      this.previousState=cloneState(s);
      s.collisionCount=0;
      if(s.collisionTimer>0){s.collisionTimer=Math.max(0,s.collisionTimer-dt);s.collisionState=s.collisionTimer>0?'wall':'none';}
      else s.collisionState='none';
      const contactsBefore=this._wheelContacts();
      const contactThreshold=c.suspensionRestLength+.12;
      const supportedBefore=contactsBefore.filter(h=>h.distance<=contactThreshold || h.signedDistance<c.suspensionRestLength);
      const wasGrounded=supportedBefore.length>=2;
      let normal=V.make(0,1,0);
      if(wasGrounded){
        const sum=supportedBefore.reduce((v,h)=>V.add(v,h.normal),V.make(0,0,0));normal=V.norm(sum);
        if(V.len2(normal)>EPS)s.contactNormal=normal;
      }
      // Ground steering includes signed forward speed in yawRate below. With forward = -Z,
      // left must use a negative steer value while moving forward; air yaw uses the opposite sign.
      const steer=input.steer;
      const airSteer=-steer;
      const throttle=input.throttle;
      const brake=input.brake;
      const handbrake=input.handbrake;
      const fwd=Q.rotate(s.orientation,V.make(0,0,-1)),right=Q.rotate(s.orientation,V.make(1,0,0));
      const longitudinal=V.dot(s.velocity,fwd),speed=V.len(s.velocity);
      const speedRatio=V.clamp(Math.abs(longitudinal)/c.maxSpeed,0,1);
      const wetGripScale=1-.42*this.weather.wetness;
      // tireGrip is a dimensionless friction coefficient (mu), not a force multiplier.
      // Longitudinal drive/brake and lateral cornering share mu * normalLoad.
      const baseMu=(handbrake>0?c.handbrakeGrip:c.tireGrip)*wetGripScale;
      // Speed-squared aerodynamic load follows the road support normal. It is
      // capped so an arcade car stays controllable and does not gain infinite grip.
      const downforce=wasGrounded?Math.min(c.maxDownforce,speed*speed*c.downforceCoefficient):0;
      const tireCapacity=Math.max(.001,baseMu*(c.gravity+downforce));
      if(downforce>0&&V.len2(s.contactNormal)>EPS){
        s.velocity=V.sub(s.velocity,V.mul(s.contactNormal,downforce*dt));
      }
      const longitudinalBeforeDrive=V.dot(s.velocity,fwd);
      const driveDemand=throttle*c.engineAcceleration*(1-speedRatio*.68);
      if(throttle>0){
        const drive=wasGrounded?Math.min(driveDemand,tireCapacity):driveDemand;
        s.velocity=V.add(s.velocity,V.mul(fwd,drive*dt));
      }
      let longNow=V.dot(s.velocity,fwd);
      if(brake>0){
        if(longNow>.5){
          // Braking opposes forward travel; below rolling speed the same control selects reverse.
          const requested=brake*c.brakeAcceleration;
          const available=wasGrounded?tireCapacity:requested;
          const braking=Math.min(Math.abs(longNow)/dt,requested,available);
          s.velocity=V.sub(s.velocity,V.mul(fwd,Math.sign(longNow)*braking*dt));
        }else if(wasGrounded){
          // Like PolyTrack's arcade controls, holding brake after stopping selects reverse.
          const reverse=Math.min(brake*c.reverseAcceleration,tireCapacity);
          s.velocity=V.sub(s.velocity,V.mul(fwd,reverse*dt));
        }
      }
      const longitudinalAccel=(V.dot(s.velocity,fwd)-longitudinalBeforeDrive)/dt;
      const postBrakeSpeed=V.len(s.velocity);
      s.velocity=V.mul(s.velocity,Math.exp(-(c.rollingResistance+c.aerodynamicDrag*postBrakeSpeed)*dt));
      if(wasGrounded){
        // Original simcade tire model (not copied from PolyTrack): a simplified
        // Pacejka-style slip-angle curve plus a combined-grip friction ellipse.
        // Tire forces rise progressively, saturate at mu * normal load, then
        // fall away after peak slip so the car can slide instead of snapping.
        const sideSpeed=V.dot(s.velocity,right);
        const forwardSpeed=V.dot(s.velocity,fwd);
        const slipAngle=Math.atan2(sideSpeed,Math.max(1,Math.abs(forwardSpeed)));
        const tireB=7.2,tireC=1.3,tireE=.88;
        const bx=tireB*slipAngle;
        const pacejkaShape=bx-tireE*(bx-Math.atan(bx));
        const lateralDemand=-baseMu*c.gravity*Math.sin(tireC*Math.atan(pacejkaShape));
        // Longitudinal acceleration consumes the same tire grip budget as cornering.
        const longitudinalCapacity=tireCapacity;
        const longitudinalUse=V.clamp(Math.abs(longitudinalAccel)/longitudinalCapacity,0,1);
        const lateralCapacity=longitudinalCapacity*Math.sqrt(Math.max(0,1-longitudinalUse*longitudinalUse));
        // Strong road bite without oscillatory side-to-side velocity flips:
        // tire force may cancel slip in one step, but cannot reverse it instantly.
        const sideForceLimit=Math.min(lateralCapacity,Math.abs(sideSpeed)/dt);
        const sideAccel=V.clamp(lateralDemand,-sideForceLimit,sideForceLimit);
        s.velocity=V.add(s.velocity,V.mul(right,sideAccel*dt));
        // Rolling tire drag and lateral scrub: more slip costs more speed, while
        // handbrake reduces rear grip to permit a controllable low-poly drift.
        const scrub=V.clamp(Math.abs(sideSpeed)*.012*(handbrake>0?1.35:1),0,.24);
        if(scrub>0)s.velocity=V.sub(s.velocity,V.mul(fwd,Math.sign(forwardSpeed)*Math.min(Math.abs(forwardSpeed),scrub*dt*10)));
        const speedForSteer=Math.max(1,Math.abs(V.dot(s.velocity,fwd)));
        const steerAngle=c.maxSteerAngle*(1-speedRatio*(1-c.highSpeedSteerScale))*steer;
        let yawRate=-V.dot(s.velocity,fwd)/c.wheelbase*Math.tan(steerAngle); // Forward is -Z: invert yaw so left input turns left and right input turns right.
        if(handbrake>0)yawRate*=c.handbrakeYawMultiplier;
        const maxYaw=Math.max(.08,longitudinalCapacity/speedForSteer);
        yawRate=V.clamp(yawRate,-maxYaw,maxYaw);
        s.orientation=Q.normalize(Q.mul(Q.axis(normal,yawRate*dt),s.orientation));
      }else{
        const airYaw=Q.axis(V.make(0,1,0),(airSteer+input.yaw)*c.airYawRate*dt);
        s.orientation=Q.normalize(Q.mul(airYaw,s.orientation));
        const localRight=Q.rotate(s.orientation,V.make(1,0,0)),localForward=Q.rotate(s.orientation,V.make(0,0,-1));
        const pitchInput=(input.brake>0?input.brake:0)-(input.throttle>0?input.throttle:0)+input.pitch;
        const pitchError=Math.asin(V.clamp(localForward.y,-1,1));
        // With no deliberate pitch input, gently bring the nose back toward level.
        // Manual gas/brake pitch control remains fully available while airborne.
        const pitchRecovery=pitchInput===0?-pitchError*c.airPitchStability*dt:0;
        s.orientation=Q.normalize(Q.mul(Q.axis(localRight,pitchInput*c.airPitchRate*dt+pitchRecovery),s.orientation));
        s.orientation=Q.normalize(Q.mul(Q.axis(localForward,(airSteer+input.roll)*c.airRollRate*dt),s.orientation));
      }
      // Wind is an external horizontal force; wetness reduces tire grip above.
      s.velocity.x+=this.weather.wind.x*c.windInfluence*dt;
      s.velocity.z+=this.weather.wind.z*c.windInfluence*dt;
      s.velocity.y-=c.gravity*dt;
      const velocityMagnitude=V.len(s.velocity);
      if(velocityMagnitude>c.maxSpeed*1.35)s.velocity=V.mul(s.velocity,(c.maxSpeed*1.35)/velocityMagnitude);
      s.position=V.add(s.position,V.mul(s.velocity,dt));
      this._resolveSurfacePenetration();
      this._applyStaticCollisions();
      this._resolveSurfacePenetration();
      const onGroundPlane=this._resolveGroundPlane();
      const contacts=this._wheelContacts().filter(h=>h.distance<=contactThreshold || h.signedDistance<c.suspensionRestLength);
      if(contacts.length>=2){
        let nsum=V.make(0,0,0),correction=V.make(0,0,0),correctionCount=0;
        let strongestImpact=0;
        for(const h of contacts){
          nsum=V.add(nsum,h.normal);
          if(h.signedDistance<c.suspensionRestLength){
            // Signed distance recovers probes that have crossed through the road triangle.
            // Add a small separation margin to prevent residual road penetration.
            const depth=c.suspensionRestLength-h.signedDistance+.012;
            correction=V.add(correction,V.mul(h.normal,depth));correctionCount++;
          }
          const vn=V.dot(s.velocity,h.normal);
          if(vn<0)strongestImpact=Math.max(strongestImpact,-vn);
        }
        const avg=V.norm(nsum);
        if(correctionCount){
          s.position=V.add(s.position,V.mul(V.mul(correction,1/correctionCount),c.contactPositionGain));
        }
        if(V.len2(avg)>EPS){
          s.contactNormal=avg;
          const normalVelocity=V.dot(s.velocity,avg);
          if(normalVelocity<0){
            const restitution=strongestImpact>7?c.collisionRestitution*.72:.02;
            s.velocity=V.sub(s.velocity,V.mul(avg,(1+restitution)*normalVelocity));
          }
          const f=Q.rotate(s.orientation,V.make(0,0,-1));
          const tangent=V.norm(V.sub(f,V.mul(avg,V.dot(f,avg))));
          if(V.len2(tangent)>EPS){
            const target=Q.fromForwardUp(tangent,avg);
            s.orientation=Q.slerp(s.orientation,target,1-Math.exp(-7.5*dt));
          }
        }
        s.grounded=true;s.gripState=handbrake>0?'DRIFT':'GRIP';
        if(s.collisionState==='none')s.collisionState='track';
      }else{
        s.grounded=false;s.gripState='AIR';
      }
      if(onGroundPlane){
        s.grounded=true;s.contactNormal=V.make(0,1,0);s.gripState='GRIP';
        s.velocity.y=Math.max(0,s.velocity.y);
        s.velocity.x*=Math.exp(-1.8*dt);s.velocity.z*=Math.exp(-1.8*dt);
      }
      this._updateCheckpoints(this.previousState.position,s.position);
      s.tick=++this.tick;
      const forwardNow=Q.rotate(s.orientation,V.make(0,0,-1)),rightNow=Q.rotate(s.orientation,V.make(1,0,0));
      const longitudinalSpeed=V.dot(s.velocity,forwardNow),lateralSpeed=V.dot(s.velocity,rightNow);
      this.telemetry={longitudinalSpeed,lateralSpeed,slipAngle:Math.atan2(lateralSpeed,Math.max(1,Math.abs(longitudinalSpeed))),
        downforce,airborne:!s.grounded,wheelContacts:contacts.length};
      const t1=(typeof performance!=='undefined'&&performance.now)?performance.now():Date.now();
      this.lastStepMs=Math.max(0,t1-t0);this.totalStepMs+=this.lastStepMs;
    }
    _updateCheckpoints(previous,current){
      if(this.checkpoints.length<2)return;
      const next=(this.currentCheckpointIndex+1)%this.checkpoints.length,cp=this.checkpoints[next];
      const d0=V.dot(V.sub(previous,cp.position),cp.forward),d1=V.dot(V.sub(current,cp.position),cp.forward);
      if(d0<0&&d1>=0){
        const cross=V.add(previous,V.mul(V.sub(current,previous),d0/(d0-d1)));
        if(V.distance(cross,cp.position)<=cp.radius){this.currentCheckpointIndex=next;this.state.checkpointId=cp.id;}
      }
    }
    interpolatedState(){
      const alpha=V.clamp(this.accumulator/this.config.fixedTimeStep,0,1),a=this.previousState,b=this.state;
      return {position:V.add(a.position,V.mul(V.sub(b.position,a.position),alpha)),
        velocity:V.add(a.velocity,V.mul(V.sub(b.velocity,a.velocity),alpha)),
        orientation:Q.slerp(a.orientation,b.orientation,alpha),grounded:b.grounded,gripState:b.gripState,
        collisionState:b.collisionState,checkpointId:b.checkpointId,tick:b.tick};
    }
    debugSnapshot(){
      const s=this.state;
      return {speed:V.len(s.velocity),verticalSpeed:s.velocity.y,grounded:s.grounded,
        grip:s.gripState,collision:s.collisionState,collisionCount:s.collisionCount,
        physicsMs:this.lastStepMs,tick:s.tick,droppedTime:this.droppedTime,collisionEvents:s.totalCollisionEvents||0,checkpointId:s.checkpointId};
    }
  }

  function createRenderAdapter(engine,renderObject){
    if(!engine||typeof engine.interpolatedState!=='function'||typeof engine.update!=='function')
      throw new TypeError('createRenderAdapter: expected a PlayCarPhysicsEngine-compatible instance');
    if(!renderObject||typeof renderObject!=='object')
      throw new TypeError('createRenderAdapter: expected a mutable render object');
    const write=(target,source,keys)=>{
      if(!target)return;
      if(typeof target.set==='function')target.set(...keys.map(key=>source[key]));
      else for(const key of keys)target[key]=source[key];
    };
    const sync=()=>{
      const pose=engine.interpolatedState();
      write(renderObject.position,pose.position,['x','y','z']);
      write(renderObject.quaternion,pose.orientation,['x','y','z','w']);
      return pose;
    };
    return {
      engine,
      sync,
      update(deltaSeconds,input){
        const step=engine.update(deltaSeconds,input),pose=sync();
        return {step,pose,telemetry:engine.getTelemetry()};
      },
      reset(position,yaw=0,up=V.make(0,1,0)){
        engine.reset(position,yaw,up);return sync();
      }
    };
  }

  const api={PlayCarPhysicsEngine,TriangleMesh,Vector3:V,Quaternion:Q,closestPointOnTriangle,createRenderAdapter,defaults:{...DEFAULTS,bodyHalfExtents:{...DEFAULTS.bodyHalfExtents}},presets:Object.fromEntries(Object.entries(PRESETS).map(([name,config])=>[name,{...config}])),version:'1.8.0'};
  root.PlayCarPhysicsEngine=api;
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);

/* PlayCar Rigid Body Core 3.0.0
 * Original, dependency-free 3D rigid-body collision and constraint solver.
 *
 * Features
 *  - Convex shapes: oriented box, sphere, infinite plane, triangle mesh (BVH).
 *  - Broadphase: body AABB pairs + BVH triangle queries for static meshes.
 *  - Narrowphase: SAT for box/box, triangle/box manifolds, sphere contacts,
 *    speculative contacts to stop fast bodies tunnelling through thin geometry.
 *  - Sequential-impulse solver: warm-started normal + Coulomb friction impulses,
 *    restitution, Baumgarte penetration recovery and non-linear position pass.
 *  - Sleeping, damping, ray casting, contact events and deterministic stepping.
 *
 * Units are metres, kilograms, seconds and radians. The module has no runtime
 * dependency and exports both a browser global and a CommonJS namespace.
 *
 * This is an original implementation written for PlayCar. It is not a copy of
 * Bullet, Box2D, PhysX, Rapier or any other engine.
 */
(function (root) {
  'use strict';

  const EPS = 1e-9;
  const SLEEP_LINEAR = 0.06;
  const SLEEP_ANGULAR = 0.12;
  const SLEEP_TIME = 0.6;

  /* ------------------------------------------------------------------ */
  /* Math                                                               */
  /* ------------------------------------------------------------------ */

  const V = {
    make: (x = 0, y = 0, z = 0) => ({ x, y, z }),
    zero: () => ({ x: 0, y: 0, z: 0 }),
    clone: a => ({ x: a.x, y: a.y, z: a.z }),
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
    mul: (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s }),
    neg: a => ({ x: -a.x, y: -a.y, z: -a.z }),
    addScaled: (a, b, s) => ({ x: a.x + b.x * s, y: a.y + b.y * s, z: a.z + b.z * s }),
    dot: (a, b) => a.x * b.x + a.y * b.y + a.z * b.z,
    cross: (a, b) => ({
      x: a.y * b.z - a.z * b.y,
      y: a.z * b.x - a.x * b.z,
      z: a.x * b.y - a.y * b.x
    }),
    lengthSq: a => a.x * a.x + a.y * a.y + a.z * a.z,
    length: a => Math.hypot(a.x, a.y, a.z),
    distance: (a, b) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z),
    normalize: a => {
      const l = Math.hypot(a.x, a.y, a.z);
      return l > EPS ? { x: a.x / l, y: a.y / l, z: a.z / l } : { x: 0, y: 0, z: 0 };
    },
    abs: a => ({ x: Math.abs(a.x), y: Math.abs(a.y), z: Math.abs(a.z) }),
    min: (a, b) => ({ x: Math.min(a.x, b.x), y: Math.min(a.y, b.y), z: Math.min(a.z, b.z) }),
    max: (a, b) => ({ x: Math.max(a.x, b.x), y: Math.max(a.y, b.y), z: Math.max(a.z, b.z) }),
    clamp: (v, lo, hi) => Math.max(lo, Math.min(hi, v)),
    isFinite: a => Number.isFinite(a.x) && Number.isFinite(a.y) && Number.isFinite(a.z),
    lerp: (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: a.z + (b.z - a.z) * t })
  };

  const Q = {
    identity: () => ({ x: 0, y: 0, z: 0, w: 1 }),
    clone: q => ({ x: q.x, y: q.y, z: q.z, w: q.w }),
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z, w: a.w + b.w }),
    mul: (a, b) => ({
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    }),
    conjugate: q => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w }),
    normalize: q => {
      const n = Math.hypot(q.x, q.y, q.z, q.w) || 1;
      return { x: q.x / n, y: q.y / n, z: q.z / n, w: q.w / n };
    },
    axisAngle: (axis, angle) => {
      const n = V.normalize(axis);
      const s = Math.sin(angle * 0.5);
      return { x: n.x * s, y: n.y * s, z: n.z * s, w: Math.cos(angle * 0.5) };
    },
    // Rotate v by q using the (1 + 2w u× + 2u×u×) expansion.
    rotate: (q, v) => {
      const u = { x: q.x, y: q.y, z: q.z };
      const uv = V.cross(u, v);
      const uuv = V.cross(u, uv);
      return {
        x: v.x + 2 * (q.w * uv.x + uuv.x),
        y: v.y + 2 * (q.w * uv.y + uuv.y),
        z: v.z + 2 * (q.w * uv.z + uuv.z)
      };
    },
    rotateInverse: (q, v) => Q.rotate(Q.conjugate(q), v),
    fromForwardUp: (forward, up) => {
      let u = V.normalize(up);
      if (V.lengthSq(u) < EPS) u = { x: 0, y: 1, z: 0 };
      let f = V.normalize(V.sub(forward, V.mul(u, V.dot(forward, u))));
      if (V.lengthSq(f) < EPS) {
        f = V.normalize(Math.abs(u.y) < 0.9 ? V.cross(u, { x: 0, y: 1, z: 0 }) : V.cross(u, { x: 1, y: 0, z: 0 }));
      }
      const r = V.normalize(V.cross(f, u));
      u = V.normalize(V.cross(r, f));
      const m00 = r.x, m01 = u.x, m02 = -f.x;
      const m10 = r.y, m11 = u.y, m12 = -f.y;
      const m20 = r.z, m21 = u.z, m22 = -f.z;
      const tr = m00 + m11 + m22;
      let x, y, z, w;
      if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
      else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s; }
      else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s; }
      else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s; }
      return Q.normalize({ x, y, z, w });
    },
    slerp: (a, b, t) => {
      let cos = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
      let end = b;
      if (cos < 0) { cos = -cos; end = { x: -b.x, y: -b.y, z: -b.z, w: -b.w }; }
      if (cos > 0.9995) {
        return Q.normalize({
          x: a.x + (end.x - a.x) * t,
          y: a.y + (end.y - a.y) * t,
          z: a.z + (end.z - a.z) * t,
          w: a.w + (end.w - a.w) * t
        });
      }
      const theta = Math.acos(V.clamp(cos, -1, 1));
      const s = Math.sin(theta);
      const wa = Math.sin((1 - t) * theta) / s;
      const wb = Math.sin(t * theta) / s;
      return Q.normalize({
        x: a.x * wa + end.x * wb,
        y: a.y * wa + end.y * wb,
        z: a.z * wa + end.z * wb,
        w: a.w * wa + end.w * wb
      });
    },
    // q' = q + 0.5 * (omega as quaternion) * q * dt
    integrate: (q, omega, dt) => {
      const dq = Q.mul({ x: omega.x, y: omega.y, z: omega.z, w: 0 }, q);
      return Q.normalize({
        x: q.x + dq.x * 0.5 * dt,
        y: q.y + dq.y * 0.5 * dt,
        z: q.z + dq.z * 0.5 * dt,
        w: q.w + dq.w * 0.5 * dt
      });
    }
  };

  function rotationMatrix(q) {
    const x = q.x, y = q.y, z = q.z, w = q.w;
    const xx = x * x, yy = y * y, zz = z * z;
    const xy = x * y, xz = x * z, yz = y * z;
    const wx = w * x, wy = w * y, wz = w * z;
    return [
      1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
      2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
      2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)
    ];
  }

  /* ------------------------------------------------------------------ */
  /* Axis-aligned bounding box                                          */
  /* ------------------------------------------------------------------ */

  const AABB = {
    create: (min, max) => ({
      min: min ? V.clone(min) : { x: Infinity, y: Infinity, z: Infinity },
      max: max ? V.clone(max) : { x: -Infinity, y: -Infinity, z: -Infinity }
    }),
    expand: (box, p) => {
      box.min.x = Math.min(box.min.x, p.x); box.min.y = Math.min(box.min.y, p.y); box.min.z = Math.min(box.min.z, p.z);
      box.max.x = Math.max(box.max.x, p.x); box.max.y = Math.max(box.max.y, p.y); box.max.z = Math.max(box.max.z, p.z);
      return box;
    },
    grow: (box, amount) => ({
      min: { x: box.min.x - amount, y: box.min.y - amount, z: box.min.z - amount },
      max: { x: box.max.x + amount, y: box.max.y + amount, z: box.max.z + amount }
    }),
    overlaps: (a, b) =>
      a.min.x <= b.max.x && a.max.x >= b.min.x &&
      a.min.y <= b.max.y && a.max.y >= b.min.y &&
      a.min.z <= b.max.z && a.max.z >= b.min.z,
    containsPoint: (box, p) =>
      p.x >= box.min.x && p.x <= box.max.x &&
      p.y >= box.min.y && p.y <= box.max.y &&
      p.z >= box.min.z && p.z <= box.max.z
  };

  /* ------------------------------------------------------------------ */
  /* Closest-point helpers                                              */
  /* ------------------------------------------------------------------ */

  function closestPointOnTriangle(p, a, b, c) {
    const ab = V.sub(b, a), ac = V.sub(c, a), ap = V.sub(p, a);
    const d1 = V.dot(ab, ap), d2 = V.dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const bp = V.sub(p, b), d3 = V.dot(ab, bp), d4 = V.dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return V.add(a, V.mul(ab, d1 / (d1 - d3)));
    const cp = V.sub(p, c), d5 = V.dot(ab, cp), d6 = V.dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return V.add(a, V.mul(ac, d2 / (d2 - d6)));
    const va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
      return V.add(b, V.mul(V.sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }
    const den = 1 / (va + vb + vc);
    return V.add(a, V.add(V.mul(ab, vb * den), V.mul(ac, vc * den)));
  }

  function barycentricInside(p, a, b, c, tolerance) {
    // Barycentric coordinates of p projected on the triangle plane.
    const v0 = V.sub(b, a), v1 = V.sub(c, a), v2 = V.sub(p, a);
    const d00 = V.dot(v0, v0), d01 = V.dot(v0, v1), d11 = V.dot(v1, v1);
    const d20 = V.dot(v2, v0), d21 = V.dot(v2, v1);
    const den = d00 * d11 - d01 * d01;
    if (Math.abs(den) < 1e-12) return false;
    const v = (d11 * d20 - d01 * d21) / den;
    const w = (d00 * d21 - d01 * d20) / den;
    const u = 1 - v - w;
    return u >= -tolerance && v >= -tolerance && w >= -tolerance;
  }

  function closestPointsSegmentSegment(p1, q1, p2, q2) {
    const d1 = V.sub(q1, p1), d2 = V.sub(q2, p2), r = V.sub(p1, p2);
    const a = V.dot(d1, d1), e = V.dot(d2, d2), f = V.dot(d2, r);
    let s, t;
    if (a <= EPS && e <= EPS) return { p1, p2, distance: V.distance(p1, p2) };
    if (a <= EPS) {
      s = 0; t = V.clamp(f / e, 0, 1);
    } else {
      const c = V.dot(d1, r);
      if (e <= EPS) {
        t = 0; s = V.clamp(-c / a, 0, 1);
      } else {
        const b = V.dot(d1, d2);
        const denom = a * e - b * b;
        s = denom !== 0 ? V.clamp((b * f - c * e) / denom, 0, 1) : 0;
        t = (b * s + f) / e;
        if (t < 0) { t = 0; s = V.clamp(-c / a, 0, 1); }
        else if (t > 1) { t = 1; s = V.clamp((b - c) / a, 0, 1); }
      }
    }
    const c1 = V.add(p1, V.mul(d1, s));
    const c2 = V.add(p2, V.mul(d2, t));
    return { p1: c1, p2: c2, distance: V.distance(c1, c2) };
  }

  /* ------------------------------------------------------------------ */
  /* Shapes                                                             */
  /* ------------------------------------------------------------------ */

  let NEXT_SHAPE_ID = 1;

  class BoxShape {
    constructor(halfExtents = { x: 0.5, y: 0.5, z: 0.5 }) {
      this.type = 'box';
      this.id = NEXT_SHAPE_ID++;
      this.halfExtents = {
        x: Math.max(1e-4, Number(halfExtents.x) || 0.5),
        y: Math.max(1e-4, Number(halfExtents.y) || 0.5),
        z: Math.max(1e-4, Number(halfExtents.z) || 0.5)
      };
      this.boundingRadius = V.length(this.halfExtents);
    }
    computeAABB(position, orientation) {
      const h = this.halfExtents;
      const ax = Q.rotate(orientation, { x: h.x, y: 0, z: 0 });
      const ay = Q.rotate(orientation, { x: 0, y: h.y, z: 0 });
      const az = Q.rotate(orientation, { x: 0, y: 0, z: h.z });
      const e = {
        x: Math.abs(ax.x) + Math.abs(ay.x) + Math.abs(az.x),
        y: Math.abs(ax.y) + Math.abs(ay.y) + Math.abs(az.y),
        z: Math.abs(ax.z) + Math.abs(ay.z) + Math.abs(az.z)
      };
      return { min: V.sub(position, e), max: V.add(position, e) };
    }
    volume() { const h = this.halfExtents; return 8 * h.x * h.y * h.z; }
  }

  class SphereShape {
    constructor(radius = 0.5) {
      this.type = 'sphere';
      this.id = NEXT_SHAPE_ID++;
      this.radius = Math.max(1e-4, Number(radius) || 0.5);
      this.boundingRadius = this.radius;
    }
    computeAABB(position) {
      const r = this.radius;
      return {
        min: { x: position.x - r, y: position.y - r, z: position.z - r },
        max: { x: position.x + r, y: position.y + r, z: position.z + r }
      };
    }
    volume() { return (4 / 3) * Math.PI * this.radius ** 3; }
  }

  class PlaneShape {
    constructor(normal = { x: 0, y: 1, z: 0 }, offset = 0) {
      this.type = 'plane';
      this.id = NEXT_SHAPE_ID++;
      this.normal = V.normalize(normal);
      this.offset = Number(offset) || 0;
      this.boundingRadius = Infinity;
    }
    distanceToPoint(p) { return V.dot(p, this.normal) - this.offset; }
    computeAABB() {
      return {
        min: { x: -Infinity, y: -Infinity, z: -Infinity },
        max: { x: Infinity, y: Infinity, z: Infinity }
      };
    }
  }

  class MeshShape {
    constructor(triangles = [], options = {}) {
      this.type = 'mesh';
      this.id = NEXT_SHAPE_ID++;
      this.triangles = [];
      this.friction = Number.isFinite(options.friction) ? options.friction : 1;
      this.restitution = Number.isFinite(options.restitution) ? options.restitution : 0.05;
      this.leafSize = Math.max(2, Math.min(32, options.leafSize || 8));
      this.order = [];
      this.nodes = [];
      this.aabb = AABB.create();
      for (const tri of triangles) this._add(tri);
      this._build();
    }

    static fromTriangles(triangles, options = {}) {
      return new MeshShape(triangles, options);
    }

    static fromLegacyMesh(mesh, options = {}) {
      if (!mesh || !Array.isArray(mesh.triangles)) return null;
      const tris = mesh.triangles.map(t => {
        if (Array.isArray(t)) return t;
        return [t.a, t.b, t.c, t.friction];
      });
      return new MeshShape(tris, options);
    }

    static fromBufferGeometry(geometry, matrixElements = null, options = {}) {
      const attr = geometry && geometry.attributes && geometry.attributes.position;
      if (!attr) throw new Error('MeshShape.fromBufferGeometry: geometry has no position attribute');
      const array = attr.array;
      const indices = geometry.index ? geometry.index.array : null;
      const count = indices ? indices.length : attr.count;
      const point = i => {
        const j = i * attr.itemSize;
        let p = { x: array[j], y: array[j + 1], z: array[j + 2] };
        if (matrixElements) {
          const e = matrixElements, x = p.x, y = p.y, z = p.z;
          const w = e[3] * x + e[7] * y + e[11] * z + e[15] || 1;
          p = {
            x: (e[0] * x + e[4] * y + e[8] * z + e[12]) / w,
            y: (e[1] * x + e[5] * y + e[9] * z + e[13]) / w,
            z: (e[2] * x + e[6] * y + e[10] * z + e[14]) / w
          };
        }
        return p;
      };
      const out = [];
      for (let i = 0; i + 2 < count; i += 3) {
        out.push([point(indices ? indices[i] : i), point(indices ? indices[i + 1] : i + 1), point(indices ? indices[i + 2] : i + 2)]);
      }
      return new MeshShape(out, options);
    }

    static fromHeightfield(options = {}, meshOptions = {}) {
      const width = Math.max(0.001, Number(options.width) || 100);
      const depth = Math.max(0.001, Number(options.depth) || 100);
      const segmentsX = Math.max(1, Math.floor(options.segmentsX || 32));
      const segmentsZ = Math.max(1, Math.floor(options.segmentsZ || 32));
      const src = options.origin || {};
      const origin = {
        x: Number.isFinite(src.x) ? src.x : 0,
        y: Number.isFinite(src.y) ? src.y : 0,
        z: Number.isFinite(src.z) ? src.z : 0
      };
      const heightAt = typeof options.heightAt === 'function' ? options.heightAt : () => 0;
      const triangles = [], dx = width / segmentsX, dz = depth / segmentsZ;
      const point = (ix, iz) => {
        const x = origin.x - width / 2 + ix * dx;
        const z = origin.z - depth / 2 + iz * dz;
        const y = Number(heightAt(x, z, ix, iz));
        if (!Number.isFinite(y)) throw new TypeError('MeshShape.fromHeightfield: heightAt must return a finite number');
        return { x, y: y + origin.y, z };
      };
      for (let iz = 0; iz < segmentsZ; iz++) {
        for (let ix = 0; ix < segmentsX; ix++) {
          const a = point(ix, iz), b = point(ix, iz + 1), c = point(ix + 1, iz + 1), d = point(ix + 1, iz);
          triangles.push([a, b, c], [a, c, d]);
        }
      }
      return new MeshShape(triangles, meshOptions);
    }

    _add(source) {
      const a = source.a || source[0];
      const b = source.b || source[1];
      const c = source.c || source[2];
      if (!a || !b || !c) return;
      const ab = V.sub(b, a), ac = V.sub(c, a);
      const raw = V.cross(ab, ac);
      const len = V.length(raw);
      if (len < 1e-12) return; // degenerate triangle
      const normal = V.mul(raw, 1 / len);
      const min = V.min(a, V.min(b, c));
      const max = V.max(a, V.max(b, c));
      const tri = {
        id: this.triangles.length,
        a: { x: a.x, y: a.y, z: a.z },
        b: { x: b.x, y: b.y, z: b.z },
        c: { x: c.x, y: c.y, z: c.z },
        normal,
        min,
        max,
        centroid: V.mul(V.add(a, V.add(b, c)), 1 / 3),
        friction: Number.isFinite(source.friction) ? source.friction : (Number.isFinite(source[3]) ? source[3] : this.friction),
        restitution: Number.isFinite(source.restitution) ? source.restitution : this.restitution
      };
      AABB.expand(this.aabb, min);
      AABB.expand(this.aabb, max);
      this.triangles.push(tri);
    }

    _build() {
      const n = this.triangles.length;
      this.order = new Array(n);
      for (let i = 0; i < n; i++) this.order[i] = i;
      this.nodes = [];
      if (n === 0) { this.root = -1; return; }
      this.root = this._buildNode(0, n, 0);
    }

    _buildNode(start, count, depth) {
      const node = {
        min: { x: Infinity, y: Infinity, z: Infinity },
        max: { x: -Infinity, y: -Infinity, z: -Infinity },
        left: -1, right: -1, start, count: 0
      };
      for (let i = start; i < start + count; i++) {
        const tri = this.triangles[this.order[i]];
        AABB.expand(node, tri.min);
        AABB.expand(node, tri.max);
      }
      const index = this.nodes.length;
      this.nodes.push(node);
      if (count <= this.leafSize || depth > 40) { node.count = count; return index; }

      const extent = V.sub(node.max, node.min);
      const axis = extent.x > extent.y ? (extent.x > extent.z ? 'x' : 'z') : (extent.y > extent.z ? 'y' : 'z');
      const slice = this.order.slice(start, start + count);
      slice.sort((i, j) => this.triangles[i].centroid[axis] - this.triangles[j].centroid[axis]);
      for (let i = 0; i < count; i++) this.order[start + i] = slice[i];

      const half = count >> 1;
      node.left = this._buildNode(start, half, depth + 1);
      node.right = this._buildNode(start + half, count - half, depth + 1);
      return index;
    }

    queryAABB(min, max, out = []) {
      if (this.root < 0) return out;
      const stack = [this.root];
      while (stack.length) {
        const node = this.nodes[stack.pop()];
        if (node.max.x < min.x || node.min.x > max.x ||
            node.max.y < min.y || node.min.y > max.y ||
            node.max.z < min.z || node.min.z > max.z) continue;
        if (node.left === -1) {
          for (let i = node.start; i < node.start + node.count; i++) {
            const tri = this.triangles[this.order[i]];
            if (tri.max.x < min.x || tri.min.x > max.x ||
                tri.max.y < min.y || tri.min.y > max.y ||
                tri.max.z < min.z || tri.min.z > max.z) continue;
            out.push(tri);
          }
        } else {
          stack.push(node.left, node.right);
        }
      }
      return out;
    }

    raycast(origin, direction, maxDistance = Infinity) {
      if (this.root < 0) return null;
      const dir = V.normalize(direction);
      let best = null;
      const stack = [this.root];
      while (stack.length) {
        const node = this.nodes[stack.pop()];
        if (!rayHitsAABB(origin, dir, maxDistance, node)) continue;
        if (node.left === -1) {
          for (let i = node.start; i < node.start + node.count; i++) {
            const tri = this.triangles[this.order[i]];
            const t = rayTriangle(origin, dir, tri.a, tri.b, tri.c);
            if (t !== null && t >= 0 && t <= (best ? best.distance : maxDistance)) {
              const point = V.add(origin, V.mul(dir, t));
              const normal = V.dot(dir, tri.normal) > 0 ? V.neg(tri.normal) : tri.normal;
              best = {
                hit: true, distance: t, point, normal,
                triangleIndex: tri.id, friction: tri.friction, restitution: tri.restitution
              };
            }
          }
        } else {
          stack.push(node.left, node.right);
        }
      }
      return best;
    }

    closestPoint(p, maxDistance = Infinity, preferredNormal = null) {
      let candidates;
      if (Number.isFinite(maxDistance) && maxDistance < Infinity) {
        const box = { min: { x: p.x - maxDistance, y: p.y - maxDistance, z: p.z - maxDistance },
          max: { x: p.x + maxDistance, y: p.y + maxDistance, z: p.z + maxDistance } };
        candidates = this.queryAABB(box.min, box.max);
      } else {
        candidates = this.triangles;
      }
      let best = null, bestDist = maxDistance;
      for (const tri of candidates) {
        const q = closestPointOnTriangle(p, tri.a, tri.b, tri.c);
        const distance = V.distance(p, q);
        if (distance <= bestDist) {
          bestDist = distance;
          let normal = tri.normal;
          if (preferredNormal && V.lengthSq(preferredNormal) > EPS && V.dot(normal, preferredNormal) < 0) {
            normal = V.neg(normal);
          }
          best = {
            point: q, normal, distance,
            signedDistance: V.dot(V.sub(p, q), normal),
            triangleIndex: tri.id, friction: tri.friction, restitution: tri.restitution
          };
        }
      }
      return best;
    }

    getTriangle(index) { return this.triangles[index] || null; }

    computeAABB() { return { min: this.aabb.min, max: this.aabb.max }; }
  }

  /* Ray helpers ------------------------------------------------------- */

  function rayHitsAABB(origin, dir, maxDistance, box) {
    let tmin = 0, tmax = maxDistance;
    for (const axis of ['x', 'y', 'z']) {
      const o = origin[axis], d = dir[axis];
      if (Math.abs(d) < EPS) {
        if (o < box.min[axis] || o > box.max[axis]) return false;
      } else {
        const inv = 1 / d;
        let t1 = (box.min[axis] - o) * inv;
        let t2 = (box.max[axis] - o) * inv;
        if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
        tmin = Math.max(tmin, t1);
        tmax = Math.min(tmax, t2);
        if (tmin > tmax) return false;
      }
    }
    return true;
  }

  function rayTriangle(origin, dir, a, b, c) {
    const e1 = V.sub(b, a), e2 = V.sub(c, a);
    const p = V.cross(dir, e2);
    const det = V.dot(e1, p);
    if (Math.abs(det) < 1e-10) return null;
    const invDet = 1 / det;
    const tvec = V.sub(origin, a);
    const u = V.dot(tvec, p) * invDet;
    if (u < -1e-6 || u > 1 + 1e-6) return null;
    const q = V.cross(tvec, e1);
    const v = V.dot(dir, q) * invDet;
    if (v < -1e-6 || u + v > 1 + 1e-6) return null;
    const t = V.dot(e2, q) * invDet;
    return t > 1e-7 ? t : null;
  }

  /* ------------------------------------------------------------------ */
  /* Body                                                               */
  /* ------------------------------------------------------------------ */

  let NEXT_BODY_ID = 1;

  function computeInertia(shape, mass) {
    if (!shape || mass <= 0) return { x: 0, y: 0, z: 0 };
    if (shape.type === 'box') {
      const h = shape.halfExtents;
      return {
        x: mass / 3 * (h.y * h.y + h.z * h.z),
        y: mass / 3 * (h.x * h.x + h.z * h.z),
        z: mass / 3 * (h.x * h.x + h.y * h.y)
      };
    }
    if (shape.type === 'sphere') {
      const i = 0.4 * mass * shape.radius * shape.radius;
      return { x: i, y: i, z: i };
    }
    return { x: mass, y: mass, z: mass };
  }

  class Body {
    constructor(options = {}) {
      this.id = NEXT_BODY_ID++;
      this.name = options.name || '';
      this.type = options.type || (Number(options.mass) > 0 ? 'dynamic' : 'static');
      if (!['dynamic', 'static', 'kinematic'].includes(this.type)) this.type = 'static';
      this.mass = this.type === 'dynamic' ? Math.max(1e-4, Number(options.mass) || 1) : 0;
      this.invMass = this.mass > 0 ? 1 / this.mass : 0;
      const inertia = options.inertia || computeInertia(options.shape, this.mass);
      this.inertia = {
        x: Math.max(1e-6, Number(inertia.x) || (this.mass || 1)),
        y: Math.max(1e-6, Number(inertia.y) || (this.mass || 1)),
        z: Math.max(1e-6, Number(inertia.z) || (this.mass || 1))
      };
      this.invInertia = this.type === 'dynamic'
        ? { x: 1 / this.inertia.x, y: 1 / this.inertia.y, z: 1 / this.inertia.z }
        : { x: 0, y: 0, z: 0 };
      this.shape = options.shape || null;
      this.position = options.position ? V.clone(options.position) : { x: 0, y: 0, z: 0 };
      this.orientation = options.orientation ? Q.normalize(Q.clone(options.orientation)) : Q.identity();
      this.linearVelocity = options.velocity ? V.clone(options.velocity) : { x: 0, y: 0, z: 0 };
      this.angularVelocity = options.angularVelocity ? V.clone(options.angularVelocity) : { x: 0, y: 0, z: 0 };
      this.force = { x: 0, y: 0, z: 0 };
      this.torque = { x: 0, y: 0, z: 0 };
      this.friction = Number.isFinite(options.friction) ? options.friction : 0.65;
      this.restitution = Number.isFinite(options.restitution) ? options.restitution : 0.1;
      this.linearDamping = Number.isFinite(options.linearDamping) ? options.linearDamping : 0.01;
      this.angularDamping = Number.isFinite(options.angularDamping) ? options.angularDamping : 0.02;
      this.gravityScale = Number.isFinite(options.gravityScale) ? options.gravityScale : 1;
      this.allowSleep = options.allowSleep !== false;
      this.ccd = options.ccd !== false;
      this.sleeping = false;
      this.sleepTimer = 0;
      this.aabb = { min: { x: 0, y: 0, z: 0 }, max: { x: 0, y: 0, z: 0 } };
      this.userData = options.userData || null;
      this.islandId = -1;
      this.updateAABB();
    }

    setShape(shape) {
      this.shape = shape;
      if (this.type === 'dynamic') {
        const inertia = computeInertia(shape, this.mass);
        this.inertia = inertia;
        this.invInertia = { x: 1 / inertia.x, y: 1 / inertia.y, z: 1 / inertia.z };
      }
      this.updateAABB();
      return this;
    }

    wake() {
      if (this.type === 'dynamic') {
        this.sleeping = false;
        this.sleepTimer = 0;
      }
      return this;
    }

    sleep() {
      if (this.type !== 'dynamic') return this;
      this.sleeping = true;
      this.linearVelocity = { x: 0, y: 0, z: 0 };
      this.angularVelocity = { x: 0, y: 0, z: 0 };
      this.force = { x: 0, y: 0, z: 0 };
      this.torque = { x: 0, y: 0, z: 0 };
      return this;
    }

    localToWorld(point) {
      return V.add(this.position, Q.rotate(this.orientation, point));
    }

    worldToLocal(point) {
      return Q.rotateInverse(this.orientation, V.sub(point, this.position));
    }

    pointVelocity(point) {
      return V.add(this.linearVelocity, V.cross(this.angularVelocity, V.sub(point, this.position)));
    }

    applyForce(force, worldPoint = this.position) {
      if (this.type !== 'dynamic') return this;
      this.wake();
      this.force = V.add(this.force, force);
      this.torque = V.add(this.torque, V.cross(V.sub(worldPoint, this.position), force));
      return this;
    }

    applyTorque(torque) {
      if (this.type !== 'dynamic') return this;
      this.wake();
      this.torque = V.add(this.torque, torque);
      return this;
    }

    applyImpulse(impulse, worldPoint = this.position) {
      if (this.type !== 'dynamic') return this;
      this.wake();
      return this.applyImpulseRaw(impulse, worldPoint);
    }

    // Solver-internal impulses must not reset the sleep timer every step.
    applyImpulseRaw(impulse, worldPoint = this.position) {
      if (this.type !== 'dynamic') return this;
      this.linearVelocity = V.add(this.linearVelocity, V.mul(impulse, this.invMass));
      const r = V.sub(worldPoint, this.position);
      this.angularVelocity = V.add(this.angularVelocity, this.applyInvInertia(V.cross(r, impulse)));
      return this;
    }

    applyAngularImpulse(impulse) {
      if (this.type !== 'dynamic') return this;
      this.wake();
      this.angularVelocity = V.add(this.angularVelocity, this.applyInvInertia(impulse));
      return this;
    }

    applyInvInertia(v) {
      if (this.type !== 'dynamic') return { x: 0, y: 0, z: 0 };
      const local = Q.rotateInverse(this.orientation, v);
      local.x *= this.invInertia.x;
      local.y *= this.invInertia.y;
      local.z *= this.invInertia.z;
      return Q.rotate(this.orientation, local);
    }

    integrateVelocity(dt, gravity) {
      if (this.type !== 'dynamic' || this.sleeping) return;
      const accel = {
        x: this.force.x * this.invMass + gravity.x * this.gravityScale,
        y: this.force.y * this.invMass + gravity.y * this.gravityScale,
        z: this.force.z * this.invMass + gravity.z * this.gravityScale
      };
      this.linearVelocity = V.add(this.linearVelocity, V.mul(accel, dt));
      this.angularVelocity = V.add(this.angularVelocity, this.applyInvInertia(V.mul(this.torque, dt)));
      const ld = Math.max(0, this.linearDamping);
      const ad = Math.max(0, this.angularDamping);
      if (ld > 0) this.linearVelocity = V.mul(this.linearVelocity, 1 / (1 + ld * dt));
      if (ad > 0) this.angularVelocity = V.mul(this.angularVelocity, 1 / (1 + ad * dt));
      // Clamp to sane magnitudes so a bad force cannot poison the whole world.
      const speed = V.length(this.linearVelocity);
      if (!Number.isFinite(speed) || speed > 1e5) this.linearVelocity = { x: 0, y: 0, z: 0 };
      const spin = V.length(this.angularVelocity);
      if (!Number.isFinite(spin) || spin > 1e5) this.angularVelocity = { x: 0, y: 0, z: 0 };
      this.force = { x: 0, y: 0, z: 0 };
      this.torque = { x: 0, y: 0, z: 0 };
    }

    integratePosition(dt) {
      if (this.type !== 'dynamic' || this.sleeping) return;
      if (this.type === 'kinematic') return;
      this.position = V.add(this.position, V.mul(this.linearVelocity, dt));
      this.orientation = Q.integrate(this.orientation, this.angularVelocity, dt);
      this.updateAABB();
    }

    updateAABB() {
      if (!this.shape) return;
      const box = this.shape.computeAABB(this.position, this.orientation);
      this.aabb.min = box.min;
      this.aabb.max = box.max;
      return this.aabb;
    }

    kineticEnergy() {
      const v2 = V.lengthSq(this.linearVelocity);
      const w = this.angularVelocity;
      return 0.5 * this.mass * v2 +
        0.5 * (this.inertia.x * w.x * w.x + this.inertia.y * w.y * w.y + this.inertia.z * w.z * w.z);
    }
  }

  /* ------------------------------------------------------------------ */
  /* Narrowphase                                                        */
  /* ------------------------------------------------------------------ */

  function makeManifold(normal) {
    return { normal, points: [] };
  }

  function addPoint(manifold, point, separation, id, feature) {
    for (const existing of manifold.points) {
      if (existing.id === id) return;
      if (V.distance(existing.point, point) < 1e-4 && Math.abs(existing.separation - separation) < 1e-4) return;
    }
    manifold.points.push({ point: V.clone(point), separation, id, feature: feature || '' });
  }

  function reduceManifold(manifold, maxPoints = 4) {
    if (manifold.points.length <= maxPoints) return manifold;
    manifold.points.sort((a, b) => a.separation - b.separation);
    manifold.points.length = maxPoints;
    return manifold;
  }

  const BOX_SIGNS = [
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
  ];

  const BOX_EDGES = [
    [0, 1], [1, 2], [2, 3], [3, 0],
    [4, 5], [5, 6], [6, 7], [7, 4],
    [0, 4], [1, 5], [2, 6], [3, 7]
  ];

  function boxVertices(position, orientation, half) {
    const out = [];
    for (let i = 0; i < 8; i++) {
      const s = BOX_SIGNS[i];
      out.push(V.add(position, Q.rotate(orientation, {
        x: s[0] * half.x, y: s[1] * half.y, z: s[2] * half.z
      })));
    }
    return out;
  }

  function pointInsideBox(point, position, orientation, half, margin = 0) {
    const local = Q.rotateInverse(orientation, V.sub(point, position));
    return Math.abs(local.x) <= half.x + margin &&
      Math.abs(local.y) <= half.y + margin &&
      Math.abs(local.z) <= half.z + margin;
  }

  function boxSupportRadius(normal, orientation, half) {
    const ax = Q.rotate(orientation, { x: half.x, y: 0, z: 0 });
    const ay = Q.rotate(orientation, { x: 0, y: half.y, z: 0 });
    const az = Q.rotate(orientation, { x: 0, y: 0, z: half.z });
    return Math.abs(V.dot(normal, ax)) + Math.abs(V.dot(normal, ay)) + Math.abs(V.dot(normal, az));
  }

  /* Sphere vs sphere -------------------------------------------------- */
  function collideSphereSphere(bodyA, shapeA, bodyB, shapeB) {
    const delta = V.sub(bodyB.position, bodyA.position);
    const dist = V.length(delta);
    const radius = shapeA.radius + shapeB.radius;
    if (dist > radius) return null;
    const normal = dist > EPS ? V.mul(delta, 1 / dist) : { x: 1, y: 0, z: 0 };
    const point = V.add(bodyA.position, V.mul(normal, shapeA.radius - (radius - dist) * 0.5));
    const manifold = makeManifold(normal);
    addPoint(manifold, point, dist - radius, 'sphere', 'sphere');
    return manifold;
  }

  /* Sphere vs plane --------------------------------------------------- */
  function collideSpherePlane(planeBody, planeShape, sphereBody, sphereShape, lookahead) {
    const d = planeShape.distanceToPoint(sphereBody.position);
    const sign = d >= 0 ? 1 : -1;
    const normal = V.mul(planeShape.normal, sign); // plane -> sphere
    const separation = Math.abs(d) - sphereShape.radius;
    if (separation > lookahead) return null;
    const point = V.sub(sphereBody.position, V.mul(normal, sphereShape.radius + separation * 0.5));
    const manifold = makeManifold(normal);
    addPoint(manifold, point, separation, 'sphere', 'sphere-plane');
    return manifold;
  }

  /* Sphere vs box ------------------------------------------------------ */
  function collideSphereBox(boxBody, boxShape, sphereBody, sphereShape, lookahead) {
    const local = Q.rotateInverse(boxBody.orientation, V.sub(sphereBody.position, boxBody.position));
    const half = boxShape.halfExtents;
    const closest = {
      x: V.clamp(local.x, -half.x, half.x),
      y: V.clamp(local.y, -half.y, half.y),
      z: V.clamp(local.z, -half.z, half.z)
    };
    const deltaLocal = V.sub(local, closest);
    const dist = V.length(deltaLocal);
    const inside = dist < EPS;
    const normalLocal = inside
      ? (() => {
        const dx = half.x - Math.abs(local.x);
        const dy = half.y - Math.abs(local.y);
        const dz = half.z - Math.abs(local.z);
        if (dx < dy && dx < dz) return { x: Math.sign(local.x) || 1, y: 0, z: 0 };
        if (dy < dz) return { x: 0, y: Math.sign(local.y) || 1, z: 0 };
        return { x: 0, y: 0, z: Math.sign(local.z) || 1 };
      })()
      : V.mul(deltaLocal, 1 / dist);
    const separation = inside
      ? -(Math.min(half.x - Math.abs(local.x), half.y - Math.abs(local.y), half.z - Math.abs(local.z)) + sphereShape.radius)
      : dist - sphereShape.radius;
    if (separation > lookahead) return null;
    const normal = Q.rotate(boxBody.orientation, normalLocal);
    const point = V.add(boxBody.position, Q.rotate(boxBody.orientation, closest));
    const manifold = makeManifold(normal); // box -> sphere
    addPoint(manifold, point, separation, 'sphere', 'sphere-box');
    return manifold;
  }

  /* Box vs plane ------------------------------------------------------- */
  function collideBoxPlane(planeBody, planeShape, boxBody, boxShape, lookahead) {
    const half = boxShape.halfExtents;
    const centerD = planeShape.distanceToPoint(boxBody.position);
    const sign = centerD >= 0 ? 1 : -1;
    const normal = V.mul(planeShape.normal, sign);
    const support = boxSupportRadius(normal, boxBody.orientation, half);
    const separation = Math.abs(centerD) - support;
    if (separation > lookahead) return null;
    const verts = boxVertices(boxBody.position, boxBody.orientation, half);
    const manifold = makeManifold(normal);
    for (let i = 0; i < 8; i++) {
      const dist = V.dot(verts[i], planeShape.normal) - planeShape.offset;
      const signed = sign * dist;
      if (signed <= lookahead) {
        const p = V.sub(verts[i], V.mul(planeShape.normal, dist));
        addPoint(manifold, p, signed, 'v' + i, 'box-plane');
      }
    }
    if (!manifold.points.length) {
      // Flat face contact: project the centre onto the plane.
      const point = V.sub(boxBody.position, V.mul(planeShape.normal, centerD));
      addPoint(manifold, point, separation, 'face', 'box-plane');
    }
    return reduceManifold(manifold);
  }

  /* Triangle vs box ---------------------------------------------------- */
  function collideTriangleBox(tri, boxBody, boxShape, lookahead) {
    const position = boxBody.position, orientation = boxBody.orientation, half = boxShape.halfExtents;
    let normal = tri.normal;
    if (V.dot(V.sub(position, tri.a), normal) < 0) normal = V.neg(normal); // triangle -> box
    const verts = boxVertices(position, orientation, half);
    const manifold = makeManifold(normal);
    const maxPenetration = 10;
    for (let i = 0; i < 8; i++) {
      const d = V.dot(V.sub(verts[i], tri.a), normal);
      if (d > lookahead || d < -maxPenetration) continue;
      const projected = V.sub(verts[i], V.mul(normal, d));
      if (!barycentricInside(projected, tri.a, tri.b, tri.c, 1e-4)) continue;
      addPoint(manifold, projected, d, 'v' + i, 'vertex-face');
    }

    // The triangle may be small and poking into a box face. If no box vertex
    // was behind the triangle plane, test triangle vertices against the box.
    if (!manifold.points.length) {
      const triVerts = [tri.a, tri.b, tri.c];
      for (let i = 0; i < 3; i++) {
        if (!pointInsideBox(triVerts[i], position, orientation, half, lookahead)) continue;
        const local = Q.rotateInverse(orientation, V.sub(triVerts[i], position));
        const px = half.x - Math.abs(local.x);
        const py = half.y - Math.abs(local.y);
        const pz = half.z - Math.abs(local.z);
        let axis, sign, depth;
        if (px < py && px < pz) { axis = { x: 1, y: 0, z: 0 }; sign = Math.sign(local.x) || 1; depth = px; }
        else if (py < pz) { axis = { x: 0, y: 1, z: 0 }; sign = Math.sign(local.y) || 1; depth = py; }
        else { axis = { x: 0, y: 0, z: 1 }; sign = Math.sign(local.z) || 1; depth = pz; }
        const faceNormal = Q.rotate(orientation, V.mul(axis, sign));
        // Push the box away from the intruding triangle vertex.
        manifold.normal = V.neg(faceNormal);
        addPoint(manifold, triVerts[i], -Math.min(depth, lookahead), 't' + i, 'vertex-box');
      }
    }

    // Edge/edge crossing fallback (triangle edge through box, no vertex inside).
    if (!manifold.points.length) {
      const triEdges = [[tri.a, tri.b], [tri.b, tri.c], [tri.c, tri.a]];
      for (let e = 0; e < 12; e++) {
        const be = BOX_EDGES[e];
        for (let t = 0; t < 3; t++) {
          const c = closestPointsSegmentSegment(verts[be[0]], verts[be[1]], triEdges[t][0], triEdges[t][1]);
          if (c.distance < lookahead) {
            addPoint(manifold, c.p1, c.distance, 'e' + e + '_' + t, 'edge-edge');
          }
        }
      }
    }
    return manifold.points.length ? reduceManifold(manifold) : null;
  }

  /* Box vs box (SAT + vertex manifold) --------------------------------- */
  function collideBoxBox(bodyA, shapeA, bodyB, shapeB, lookahead) {
    const hA = shapeA.halfExtents, hB = shapeB.halfExtents;
    const d = V.sub(bodyB.position, bodyA.position);
    const axesA = [
      Q.rotate(bodyA.orientation, { x: 1, y: 0, z: 0 }),
      Q.rotate(bodyA.orientation, { x: 0, y: 1, z: 0 }),
      Q.rotate(bodyA.orientation, { x: 0, y: 0, z: 1 })
    ];
    const axesB = [
      Q.rotate(bodyB.orientation, { x: 1, y: 0, z: 0 }),
      Q.rotate(bodyB.orientation, { x: 0, y: 1, z: 0 }),
      Q.rotate(bodyB.orientation, { x: 0, y: 0, z: 1 })
    ];
    const axes = [...axesA, ...axesB];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        const cross = V.cross(axesA[i], axesB[j]);
        const len = V.length(cross);
        if (len > 1e-6) axes.push(V.mul(cross, 1 / len));
      }
    }

    let minOverlap = Infinity, bestNormal = null;
    for (const axis of axes) {
      const rA = boxSupportRadius(axis, bodyA.orientation, hA);
      const rB = boxSupportRadius(axis, bodyB.orientation, hB);
      const dist = V.dot(d, axis);
      const overlap = rA + rB - Math.abs(dist);
      if (overlap < -1e-9) return null;
      if (overlap < minOverlap) {
        minOverlap = overlap;
        bestNormal = dist < 0 ? V.neg(axis) : axis;
      }
    }
    if (!bestNormal) return null;
    if (minOverlap > lookahead) return null;

    const normal = bestNormal;
    const rA = boxSupportRadius(normal, bodyA.orientation, hA);
    const centerDist = V.dot(d, normal);
    const manifold = makeManifold(normal);
    const vertsB = boxVertices(bodyB.position, bodyB.orientation, hB);
    const vertsA = boxVertices(bodyA.position, bodyA.orientation, hA);
    const margin = Math.max(0.005, lookahead);

    for (let i = 0; i < 8; i++) {
      if (!pointInsideBox(vertsB[i], bodyA.position, bodyA.orientation, hA, margin)) continue;
      const separation = V.dot(V.sub(vertsB[i], bodyA.position), normal) - rA;
      if (separation > lookahead) continue;
      addPoint(manifold, vertsB[i], separation, 'b' + i, 'box-box');
    }
    for (let i = 0; i < 8; i++) {
      if (!pointInsideBox(vertsA[i], bodyB.position, bodyB.orientation, hB, margin)) continue;
      const along = V.dot(V.sub(vertsA[i], bodyA.position), normal);
      const separation = along - centerDist + boxSupportRadius(normal, bodyB.orientation, hB);
      if (separation > lookahead) continue;
      addPoint(manifold, vertsA[i], separation, 'a' + i, 'box-box');
    }
    if (!manifold.points.length) {
      // Edge/edge contact: closest points between the two boxes, projected.
      let best = null;
      for (let i = 0; i < 8; i++) {
        for (let j = 0; j < 8; j++) {
          const dist = V.distance(vertsA[i], vertsB[j]);
          if (!best || dist < best.dist) best = { i, j, dist };
        }
      }
      if (!best) return null;
      const p = V.mul(V.add(vertsA[best.i], vertsB[best.j]), 0.5);
      addPoint(manifold, p, -Math.max(0, minOverlap), 'edge', 'box-box-edge');
    }
    // Prefer the deepest points for stability; keep four.
    manifold.points.sort((a, b) => a.separation - b.separation);
    if (manifold.points.length > 4) manifold.points.length = 4;
    return manifold;
  }

  /* Sphere vs mesh ----------------------------------------------------- */
  function collideSphereMesh(meshShape, sphereBody, sphereShape, lookahead) {
    const box = sphereShape.computeAABB(sphereBody.position);
    const candidates = meshShape.queryAABB(
      { x: box.min.x - lookahead, y: box.min.y - lookahead, z: box.min.z - lookahead },
      { x: box.max.x + lookahead, y: box.max.y + lookahead, z: box.max.z + lookahead }
    );
    let manifold = null;
    for (const tri of candidates) {
      const q = closestPointOnTriangle(sphereBody.position, tri.a, tri.b, tri.c);
      const delta = V.sub(sphereBody.position, q);
      const distance = V.length(delta);
      const separation = distance - sphereShape.radius;
      if (separation > lookahead) continue;
      let normal = distance > EPS ? V.mul(delta, 1 / distance) : tri.normal;
      if (V.lengthSq(normal) < EPS) normal = { x: 0, y: 1, z: 0 };
      if (!manifold) {
        manifold = makeManifold(normal);
        manifold.friction = tri.friction;
        manifold.restitution = tri.restitution;
      }
      addPoint(manifold, q, separation, 't' + tri.id, 'sphere-mesh');
    }
    return manifold ? reduceManifold(manifold) : null;
  }

  /* Box vs mesh -------------------------------------------------------- */
  function collideBoxMesh(meshShape, boxBody, boxShape, lookahead) {
    const box = boxShape.computeAABB(boxBody.position, boxBody.orientation);
    const candidates = meshShape.queryAABB(
      { x: box.min.x - lookahead, y: box.min.y - lookahead, z: box.min.z - lookahead },
      { x: box.max.x + lookahead, y: box.max.y + lookahead, z: box.max.z + lookahead }
    );
    let manifold = null;
    for (const tri of candidates) {
      const m = collideTriangleBox(tri, boxBody, boxShape, lookahead);
      if (!m || !m.points.length) continue;
      if (!manifold) {
        manifold = makeManifold(m.normal);
        manifold.friction = m.friction;
        manifold.restitution = m.restitution;
      }
      // Keep the normal consistently oriented towards the box (triangle -> box).
      if (V.dot(m.normal, manifold.normal) < 0.5) {
        if (!manifold.points.length) manifold.normal = m.normal;
      }
      for (const p of m.points) addPoint(manifold, p.point, p.separation, 't' + tri.id + p.id, p.feature);
    }
    return manifold ? reduceManifold(manifold) : null;
  }

  /* Dispatch ----------------------------------------------------------- */
  function collideShapes(bodyA, shapeA, bodyB, shapeB, lookahead) {
    const a = shapeA.type, b = shapeB.type;
    if (a === 'sphere' && b === 'sphere') return collideSphereSphere(bodyA, shapeA, bodyB, shapeB);
    if (a === 'plane' && b === 'sphere') return collideSpherePlane(bodyA, shapeA, bodyB, shapeB, lookahead);
    if (a === 'sphere' && b === 'plane') {
      const m = collideSpherePlane(bodyB, shapeB, bodyA, shapeA, lookahead);
      if (m) m.normal = V.neg(m.normal);
      return m;
    }
    if (a === 'box' && b === 'sphere') return collideSphereBox(bodyA, shapeA, bodyB, shapeB, lookahead);
    if (a === 'sphere' && b === 'box') {
      const m = collideSphereBox(bodyB, shapeB, bodyA, shapeA, lookahead);
      if (m) m.normal = V.neg(m.normal);
      return m;
    }
    if (a === 'plane' && b === 'box') return collideBoxPlane(bodyA, shapeA, bodyB, shapeB, lookahead);
    if (a === 'box' && b === 'plane') {
      const m = collideBoxPlane(bodyB, shapeB, bodyA, shapeA, lookahead);
      if (m) m.normal = V.neg(m.normal);
      return m;
    }
    if (a === 'box' && b === 'box') return collideBoxBox(bodyA, shapeA, bodyB, shapeB, lookahead);
    if (a === 'mesh' && b === 'box') return collideBoxMesh(shapeA, bodyB, shapeB, lookahead);
    if (a === 'box' && b === 'mesh') {
      const m = collideBoxMesh(shapeB, bodyA, shapeA, lookahead);
      if (m) {
        m.normal = V.neg(m.normal);
        for (const p of m.points) p.separation = p.separation; // separation is symmetric
      }
      return m;
    }
    if (a === 'mesh' && b === 'sphere') return collideSphereMesh(shapeA, bodyB, shapeB, lookahead);
    if (a === 'sphere' && b === 'mesh') {
      const m = collideSphereMesh(shapeB, bodyA, shapeA, lookahead);
      if (m) m.normal = V.neg(m.normal);
      return m;
    }
    return null;
  }

  /* ------------------------------------------------------------------ */
  /* Contact constraint                                                 */
  /* ------------------------------------------------------------------ */

  function contactKey(bodyA, bodyB) {
    return bodyA.id < bodyB.id ? bodyA.id + ':' + bodyB.id : bodyB.id + ':' + bodyA.id;
  }

  function tangentBasis(normal) {
    const ref = Math.abs(normal.y) < 0.9 ? { x: 0, y: 1, z: 0 } : { x: 1, y: 0, z: 0 };
    const t1 = V.normalize(V.cross(normal, ref));
    const t2 = V.normalize(V.cross(normal, t1));
    return [t1, t2];
  }

  class ContactConstraint {
    constructor(bodyA, bodyB, manifold, settings) {
      this.bodyA = bodyA;
      this.bodyB = bodyB;
      this.manifold = manifold;
      this.normal = manifold.normal;
      this.tangents = tangentBasis(this.normal);
      this.points = manifold.points.map(p => ({
        point: p.point,
        separation: p.separation,
        id: p.id,
        normalMass: 0,
        tangentMass: [0, 0],
        normalImpulse: 0,
        tangentImpulse: [0, 0],
        bias: 0,
        targetNormalVelocity: 0,
        rA: { x: 0, y: 0, z: 0 },
        rB: { x: 0, y: 0, z: 0 },
        localA: { x: 0, y: 0, z: 0 },
        localB: { x: 0, y: 0, z: 0 }
      }));
      const A = bodyA, B = bodyB;
      const surfaceFriction = Number.isFinite(manifold.friction) ? manifold.friction : A.friction;
      this.friction = Math.sqrt(Math.max(0, surfaceFriction) * Math.max(0, B.friction));
      const surfaceRestitution = Number.isFinite(manifold.restitution) ? manifold.restitution : 0;
      this.restitution = Math.max(surfaceRestitution, A.restitution, B.restitution);
      this.settings = settings;
    }

    preStep(dt, warmCache) {
      const A = this.bodyA, B = this.bodyB, n = this.normal;
      const slop = this.settings.slop;
      const beta = this.settings.baumgarte;
      const threshold = this.settings.restitutionThreshold;
      for (const p of this.points) {
        p.rA = V.sub(p.point, A.position);
        p.rB = V.sub(p.point, B.position);
        p.localA = Q.rotateInverse(A.orientation, p.rA);
        p.localB = Q.rotateInverse(B.orientation, p.rB);

        const rnA = V.cross(p.rA, n);
        const rnB = V.cross(p.rB, n);
        const angularA = V.dot(V.cross(A.applyInvInertia(rnA), p.rA), n);
        const angularB = V.dot(V.cross(B.applyInvInertia(rnB), p.rB), n);
        const k = A.invMass + B.invMass + angularA + angularB;
        p.normalMass = k > EPS ? 1 / k : 0;

        for (let t = 0; t < 2; t++) {
          const tan = this.tangents[t];
          const rtA = V.cross(p.rA, tan);
          const rtB = V.cross(p.rB, tan);
          const kt = A.invMass + B.invMass +
            V.dot(V.cross(A.applyInvInertia(rtA), p.rA), tan) +
            V.dot(V.cross(B.applyInvInertia(rtB), p.rB), tan);
          p.tangentMass[t] = kt > EPS ? 1 / kt : 0;
        }

        const relativeVelocity = V.sub(B.pointVelocity(p.point), A.pointVelocity(p.point));
        const vn = V.dot(relativeVelocity, n);
        let bias;
        if (p.separation > 0) {
          // Speculative contact: allow closing the gap in exactly one step but no more.
          bias = -p.separation / dt;
        } else {
          // Penetration is mainly removed by the position pass; a small clamped
          // Baumgarte term avoids injecting energy into resting contacts.
          bias = Math.min(this.settings.maxBias, beta * Math.max(0, -p.separation - slop) / dt);
        }
        if (p.separation <= 0.03 && vn < -threshold) {
          bias = Math.max(bias, -this.restitution * vn);
        }
        p.bias = bias;

        const cached = warmCache && warmCache.get(p.id);
        p.normalImpulse = cached ? cached.n : 0;
        p.tangentImpulse[0] = cached ? cached.t1 : 0;
        p.tangentImpulse[1] = cached ? cached.t2 : 0;
      }
    }

    warmStart() {
      const A = this.bodyA, B = this.bodyB;
      for (const p of this.points) {
        const P = V.add(
          V.mul(this.normal, p.normalImpulse),
          V.add(V.mul(this.tangents[0], p.tangentImpulse[0]), V.mul(this.tangents[1], p.tangentImpulse[1]))
        );
        if (!P.x && !P.y && !P.z) continue;
        A.applyImpulseRaw(V.neg(P), p.point);
        B.applyImpulseRaw(P, p.point);
      }
    }

    solveVelocity() {
      const A = this.bodyA, B = this.bodyB, n = this.normal;
      for (const p of this.points) {
        let relativeVelocity = V.sub(B.pointVelocity(p.point), A.pointVelocity(p.point));
        let vn = V.dot(relativeVelocity, n);
        let dPn = -p.normalMass * (vn - p.bias);
        const oldNormal = p.normalImpulse;
        p.normalImpulse = Math.max(0, oldNormal + dPn);
        dPn = p.normalImpulse - oldNormal;
        if (dPn !== 0) {
          const impulse = V.mul(n, dPn);
          A.applyImpulseRaw(V.neg(impulse), p.point);
          B.applyImpulseRaw(impulse, p.point);
        }

        const maxFriction = this.friction * p.normalImpulse;
        for (let t = 0; t < 2; t++) {
          const tan = this.tangents[t];
          relativeVelocity = V.sub(B.pointVelocity(p.point), A.pointVelocity(p.point));
          const vt = V.dot(relativeVelocity, tan);
          let dPt = -p.tangentMass[t] * vt;
          const oldTangent = p.tangentImpulse[t];
          const next = V.clamp(oldTangent + dPt, -maxFriction, maxFriction);
          dPt = next - oldTangent;
          p.tangentImpulse[t] = next;
          if (dPt !== 0) {
            const impulse = V.mul(tan, dPt);
            A.applyImpulseRaw(V.neg(impulse), p.point);
            B.applyImpulseRaw(impulse, p.point);
          }
        }
      }
    }

    solvePosition() {
      const A = this.bodyA, B = this.bodyB, n = this.normal;
      const slop = this.settings.slop;
      const maxCorrection = this.settings.maxCorrection;
      for (const p of this.points) {
        if (p.separation > 0) continue;
        const pointA = V.add(A.position, Q.rotate(A.orientation, p.localA));
        const pointB = V.add(B.position, Q.rotate(B.orientation, p.localB));
        const separation = p.separation + V.dot(V.sub(pointB, pointA), n);
        const correction = V.clamp(separation + slop, -maxCorrection, 0);
        if (correction >= 0) continue;
        const rA = V.sub(pointA, A.position);
        const rB = V.sub(pointB, B.position);
        const rnA = V.cross(rA, n);
        const rnB = V.cross(rB, n);
        const k = A.invMass + B.invMass +
          V.dot(V.cross(A.applyInvInertia(rnA), rA), n) +
          V.dot(V.cross(B.applyInvInertia(rnB), rB), n);
        if (k < EPS) continue;
        const impulse = -correction * this.settings.positionBeta / k;
        const P = V.mul(n, impulse);
        if (A.invMass) {
          A.position = V.sub(A.position, V.mul(P, A.invMass));
          A.orientation = Q.normalize(Q.integrate(A.orientation, A.applyInvInertia(V.neg(V.cross(rA, P))), 1));
        }
        if (B.invMass) {
          B.position = V.add(B.position, V.mul(P, B.invMass));
          B.orientation = Q.normalize(Q.integrate(B.orientation, B.applyInvInertia(V.cross(rB, P)), 1));
        }
      }
    }

    storeImpulses(cache) {
      for (const p of this.points) {
        cache.set(p.id, { n: p.normalImpulse, t1: p.tangentImpulse[0], t2: p.tangentImpulse[1] });
      }
    }
  }

  /* ------------------------------------------------------------------ */
  /* World                                                              */
  /* ------------------------------------------------------------------ */

  const WORLD_DEFAULTS = {
    gravity: { x: 0, y: -9.81, z: 0 },
    fixedDt: 1 / 120,
    maxSubsteps: 12,
    velocityIterations: 10,
    positionIterations: 4,
    slop: 0.005,
    baumgarte: 0.05,
    maxBias: 0.4,
    positionBeta: 0.45,
    maxCorrection: 0.08,
    restitutionThreshold: 0.9,
    allowSleep: true,
    maxCcdSubsteps: 4,
    ccdThreshold: 0.12
  };

  class World {
    constructor(options = {}) {
      this.settings = { ...WORLD_DEFAULTS, ...options };
      this.settings.gravity = options.gravity ? V.clone(options.gravity) : { ...WORLD_DEFAULTS.gravity };
      this.bodies = [];          // dynamic + kinematic
      this.staticBodies = [];    // static, includes meshes and planes
      this.manifolds = [];
      this.constraints = [];
      this._warmCache = new Map();
      this._nextWarmCache = new Map();
      this.stats = { contacts: 0, bodies: 0, broadphaseQueries: 0, solverIterations: 0, ccdSplits: 0 };
      this.events = { beginContact: [], endContact: [] };
      this._contactTouching = new Set();
      this.time = 0;
      this.stepCount = 0;
      this.accumulator = 0;
    }

    addBody(body) {
      if (body.type === 'static') this.staticBodies.push(body);
      else this.bodies.push(body);
      return body;
    }

    removeBody(body) {
      this.bodies = this.bodies.filter(b => b !== body);
      this.staticBodies = this.staticBodies.filter(b => b !== body);
    }

    addStaticMesh(meshShape, materials = {}) {
      const body = new Body({
        type: 'static',
        shape: meshShape,
        friction: Number.isFinite(materials.friction) ? materials.friction : 1,
        restitution: Number.isFinite(materials.restitution) ? materials.restitution : 0.05
      });
      this.addBody(body);
      return body;
    }

    addGroundPlane(y = 0, options = {}) {
      return this.addBody(new Body({
        type: 'static',
        shape: new PlaneShape(options.normal || { x: 0, y: 1, z: 0 }, y),
        friction: Number.isFinite(options.friction) ? options.friction : 1,
        restitution: Number.isFinite(options.restitution) ? options.restitution : 0.05
      }));
    }

    getGravity() { return this.settings.gravity; }

    setGravity(g) {
      this.settings.gravity = V.clone(g);
      for (const body of this.bodies) body.wake();
    }

    _lookahead(body, dt) {
      const speed = V.length(body.linearVelocity);
      // Speculative contacts: awake bodies at speed are tested against geometry
      // they can reach this step. The ccd flag widens the window; disabling it
      // falls back to penetration recovery only.
      const scale = body.ccd ? 1.25 : 0.2;
      return Math.max(0.01, speed * dt * scale + this.settings.slop * 2);
    }

    collectManifolds(dt) {
      const manifolds = [];
      const statics = this.staticBodies;
      const dynamics = this.bodies;
      for (const body of dynamics) {
        if (!body.shape || body.sleeping) continue;
        const lookahead = this._lookahead(body, dt);
        // Dynamic vs dynamic
        for (const other of dynamics) {
          if (other === body) continue;
          // Awake bodies own the pair to avoid double processing; a sleeping body
          // must still be tested from the awake side regardless of id ordering.
          if (!other.sleeping && other.id <= body.id) continue;
          if (!other.shape || (body.sleeping && other.sleeping)) continue;
          if (!AABB.overlaps(AABB.grow(body.aabb, lookahead), AABB.grow(other.aabb, lookahead))) continue;
          if (other.sleeping) other.wake();
          if (body.sleeping) body.wake();
          const m = collideShapes(body, body.shape, other, other.shape, lookahead);
          if (m && m.points.length) manifolds.push({ bodyA: body, bodyB: other, manifold: m });
        }
        // Dynamic vs static
        for (const other of statics) {
          if (!other.shape) continue;
          this.stats.broadphaseQueries++;
          if (other.shape.type === 'mesh') {
            if (!AABB.overlaps(AABB.grow(body.aabb, lookahead), other.aabb)) continue;
          } else if (other.shape.type === 'plane') {
            // planes always considered
          } else if (!AABB.overlaps(AABB.grow(body.aabb, lookahead), AABB.grow(other.aabb, lookahead))) {
            continue;
          }
          const m = collideShapes(other, other.shape, body, body.shape, lookahead);
          if (m && m.points.length) manifolds.push({ bodyA: other, bodyB: body, manifold: m });
        }
      }
      return manifolds;
    }

    stepOnce(dt) {
      const settings = this.settings;
      for (const body of this.bodies) body.integrateVelocity(dt, settings.gravity);
      for (const body of this.bodies) body.updateAABB();
      this._nextWarmCache = new Map();
      const manifolds = this.collectManifolds(dt);
      this.manifolds = manifolds;
      this.stats.contacts = 0;
      this.constraints = [];
      const touching = new Set();
      for (const entry of manifolds) {
        const { bodyA, bodyB, manifold } = entry;
        if (bodyA.type !== 'dynamic' && bodyB.type !== 'dynamic') continue;
        const key = contactKey(bodyA, bodyB);
        const cache = this._warmCache.get(key);
        const constraint = new ContactConstraint(bodyA, bodyB, manifold, settings);
        constraint.preStep(dt, cache);
        constraint.warmStart();
        this.constraints.push({ constraint, key });
        this.stats.contacts += manifold.points.length;
        touching.add(key);
      }
      // Deterministic solve order.
      this.constraints.sort((a, b) => a.key < b.key ? -1 : a.key > b.key ? 1 : 0);
      for (let i = 0; i < settings.velocityIterations; i++) {
        for (const { constraint } of this.constraints) constraint.solveVelocity();
        this.stats.solverIterations++;
      }
      for (const body of this.bodies) body.integratePosition(dt);
      for (let i = 0; i < settings.positionIterations; i++) {
        for (const { constraint } of this.constraints) constraint.solvePosition();
      }
      for (const body of this.bodies) body.updateAABB();
      for (const { constraint, key } of this.constraints) {
        constraint.storeImpulses(this._nextWarmCache.get(key) || (() => {
          const map = new Map();
          this._nextWarmCache.set(key, map);
          return map;
        })());
      }
      this._warmCache = this._nextWarmCache;

      // Contact begin/end events.
      for (const key of touching) {
        if (!this._contactTouching.has(key)) {
          const entry = manifolds.find(m => contactKey(m.bodyA, m.bodyB) === key);
          if (entry) this.events.beginContact.push({ bodyA: entry.bodyA, bodyB: entry.bodyB, manifold: entry.manifold });
        }
      }
      for (const key of this._contactTouching) {
        if (!touching.has(key)) {
          this.events.endContact.push({ key });
        }
      }
      this._contactTouching = touching;

      // Sleeping.
      if (settings.allowSleep) {
        for (const body of this.bodies) {
          if (body.type !== 'dynamic' || !body.allowSleep) continue;
          const linear = V.length(body.linearVelocity);
          const angular = V.length(body.angularVelocity);
          if (linear < SLEEP_LINEAR && angular < SLEEP_ANGULAR) {
            body.sleepTimer += dt;
            if (body.sleepTimer > SLEEP_TIME) body.sleep();
          } else {
            body.sleepTimer = 0;
            body.sleeping = false;
          }
        }
      }
      this.time += dt;
      this.stepCount++;
    }

    step(dt) {
      const fixed = this.settings.fixedDt;
      if (!Number.isFinite(dt) || dt <= 0) return 0;
      this.accumulator += Math.min(dt, 0.25);
      let steps = 0;
      while (this.accumulator + 1e-10 >= fixed && steps < this.settings.maxSubsteps) {
        this.stepOnce(fixed);
        this.accumulator -= fixed;
        steps++;
      }
      if (this.accumulator >= fixed) this.accumulator %= fixed;
      return steps;
    }

    // Advance exactly one fixed sub-step (used by the vehicle at 120 Hz).
    fixedStep(dt) {
      this.stepOnce(dt);
      return 1;
    }

    queryAABB(min, max) {
      const out = [];
      for (const body of this.bodies.concat(this.staticBodies)) {
        if (!body.shape) continue;
        if (body.shape.type === 'plane') { out.push(body); continue; }
        if (body.shape.type === 'mesh') {
          const tris = body.shape.queryAABB(min, max);
          if (tris.length) out.push(body);
        } else if (AABB.overlaps(body.aabb, { min, max })) {
          out.push(body);
        }
      }
      return out;
    }

    raycast(origin, direction, maxDistance = Infinity, options = {}) {
      let best = null;
      const test = hit => {
        if (hit && hit.hit && (!best || hit.distance < best.distance)) best = hit;
      };
      for (const body of this.staticBodies.concat(this.bodies)) {
        if (!body.shape) continue;
        if (options.ignore === body) continue;
        if (options.mask && !(options.mask & 1)) continue;
        const shape = body.shape;
        if (shape.type === 'mesh') {
          const hit = shape.raycast(origin, direction, best ? best.distance : maxDistance);
          if (hit) {
            test({ ...hit, body, shape });
          }
        } else if (shape.type === 'plane') {
          const denom = V.dot(direction, shape.normal);
          if (Math.abs(denom) < EPS) continue;
          const t = (shape.offset - V.dot(origin, shape.normal)) / denom;
          if (t >= 0 && t <= (best ? best.distance : maxDistance)) {
            test({
              hit: true, distance: t, point: V.add(origin, V.mul(V.normalize(direction), t)),
              normal: denom < 0 ? shape.normal : V.neg(shape.normal),
              body, shape, friction: body.friction, restitution: body.restitution
            });
          }
        } else if (shape.type === 'sphere') {
          const oc = V.sub(origin, body.position);
          const d = V.normalize(direction);
          const b = V.dot(oc, d);
          const c = V.lengthSq(oc) - shape.radius * shape.radius;
          const disc = b * b - c;
          if (disc >= 0) {
            const t = -b - Math.sqrt(disc);
            if (t >= 0 && t <= (best ? best.distance : maxDistance)) {
              const point = V.add(origin, V.mul(d, t));
              test({
                hit: true, distance: t, point, normal: V.normalize(V.sub(point, body.position)),
                body, shape, friction: body.friction, restitution: body.restitution
              });
            }
          }
        } else if (shape.type === 'box') {
          const localOrigin = Q.rotateInverse(body.orientation, V.sub(origin, body.position));
          const localDir = Q.rotateInverse(body.orientation, V.normalize(direction));
          const h = shape.halfExtents;
          let tmin = 0, tmax = best ? best.distance : maxDistance;
          let hitAxis = null;
          let hitSign = 1;
          let ok = true;
          for (const axis of ['x', 'y', 'z']) {
            if (Math.abs(localDir[axis]) < EPS) {
              if (Math.abs(localOrigin[axis]) > h[axis]) { ok = false; break; }
            } else {
              const inv = 1 / localDir[axis];
              let t1 = (-h[axis] - localOrigin[axis]) * inv;
              let t2 = (h[axis] - localOrigin[axis]) * inv;
              let sign = -1;
              if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; sign = 1; }
              if (t1 > tmin) { tmin = t1; hitAxis = axis; hitSign = sign; }
              tmax = Math.min(tmax, t2);
              if (tmin > tmax) { ok = false; break; }
            }
          }
          if (ok && hitAxis) {
            const localPoint = V.add(localOrigin, V.mul(localDir, tmin));
            const normalLocal = { x: 0, y: 0, z: 0 };
            normalLocal[hitAxis] = hitSign;
            test({
              hit: true, distance: tmin,
              point: V.add(body.position, Q.rotate(body.orientation, localPoint)),
              normal: Q.rotate(body.orientation, normalLocal),
              body, shape, friction: body.friction, restitution: body.restitution
            });
          }
        }
      }
      return best || { hit: false };
    }
  }

  /* ------------------------------------------------------------------ */
  /* Exports                                                            */
  /* ------------------------------------------------------------------ */

  const Core = {
    version: '3.0.0',
    Vec3: V,
    Quaternion: Q,
    AABB,
    closestPointOnTriangle,
    closestPointsSegmentSegment,
    BoxShape,
    SphereShape,
    PlaneShape,
    MeshShape,
    Body,
    ContactConstraint,
    World,
    collideShapes,
    collideBoxBox,
    collideTriangleBox,
    tangentBasis,
    defaults: { ...WORLD_DEFAULTS, gravity: { ...WORLD_DEFAULTS.gravity } }
  };

  root.PlayCarRigidBodyCore = Core;
  if (typeof module !== 'undefined' && module.exports) module.exports = Core;
})(typeof globalThis !== 'undefined' ? globalThis : this);

/* PlayCar rigid-body vehicle physics 3.0.0
 * Fixed-step 120 Hz vehicle dynamics on top of the PlayCar rigid-body core.
 * SI-like units: metres, seconds, kilograms, newtons, radians.
 *
 * Layers
 *  1. Rigid-body chassis (box) integrated and collision-solved by rigidbody-core.
 *  2. Four independent ray-cast suspension units with progressive springs,
 *     asymmetric bump/rebound damping, bump stops and anti-roll bars.
 *  3. Combined-slip tire model (normalised-slip similarity method, Pacejka
 *     Magic Formula shape) with relaxation length, load sensitivity, camber
 *     thrust, rolling resistance and per-surface friction.
 *  4. Engine torque curve, gearbox with shift time and engine braking,
 *     open / limited-slip / locked differentials and driven-wheel layouts.
 *  5. ABS, TCS and optional ESC driver aids, aerodynamic drag/downforce.
 *
 * The module is dependency-free except for ./rigidbody-core.js. It exports a
 * browser global (PlayCarRigidBody) and a CommonJS namespace.
 */
(function (root) {
  'use strict';

  const V = {
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
    mul: (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s }),
    dot: (a, b) => a.x * b.x + a.y * b.y + a.z * b.z,
    cross: (a, b) => ({ x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }),
    len: a => Math.hypot(a.x, a.y, a.z),
    len2: a => a.x * a.x + a.y * a.y + a.z * a.z,
    norm: a => {
      const n = Math.hypot(a.x, a.y, a.z) || 1;
      return { x: a.x / n, y: a.y / n, z: a.z / n };
    },
    clamp: (x, a, b) => Math.max(a, Math.min(b, x)),
    zero: () => ({ x: 0, y: 0, z: 0 })
  };

  const Q = {
    id: () => ({ x: 0, y: 0, z: 0, w: 1 }),
    mul: (a, b) => ({
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    }),
    norm: q => {
      const n = Math.hypot(q.x, q.y, q.z, q.w) || 1;
      return { x: q.x / n, y: q.y / n, z: q.z / n, w: q.w / n };
    },
    rotate: (q, v) => {
      const u = { x: q.x, y: q.y, z: q.z };
      const t = V.mul(V.cross(u, v), 2);
      return V.add(v, V.add(V.mul(t, q.w), V.cross(u, t)));
    },
    rotateInverse: (q, v) => Q.rotate({ x: -q.x, y: -q.y, z: -q.z, w: q.w }, v),
    axis: (axis, angle) => {
      const n = V.norm(axis), s = Math.sin(angle / 2);
      return { x: n.x * s, y: n.y * s, z: n.z * s, w: Math.cos(angle / 2) };
    },
    slerp: (a, b, t) => {
      let d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
      if (d < 0) { b = { x: -b.x, y: -b.y, z: -b.z, w: -b.w }; d = -d; }
      if (d > 0.9995) {
        return Q.norm({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: a.z + (b.z - a.z) * t, w: a.w + (b.w - a.w) * t });
      }
      const th = Math.acos(V.clamp(d, -1, 1)), s = Math.sin(th);
      const u = Math.sin((1 - t) * th) / s, v = Math.sin(t * th) / s;
      return Q.norm({ x: a.x * u + b.x * v, y: a.y * u + b.y * v, z: a.z * u + b.z * v, w: a.w * u + b.w * v });
    }
  };

  /* ------------------------------------------------------------------ */
  /* Defaults                                                           */
  /* ------------------------------------------------------------------ */

  const DEFAULTS = {
    fixedDt: 1 / 120,
    maxSubsteps: 24,
    mass: 1250,
    gravity: 9.81,
    inertia: { x: 520, y: 1450, z: 1650 },
    com: { x: 0, y: 0.52, z: 0 },
    wheelbase: 2.65,
    track: 1.56,
    wheelRadius: 0.32,
    wheelMass: 18,
    suspension: {
      rest: 0.32, min: 0.08, max: 0.48,
      stiffness: 32000, damping: 4200,
      dampingBump: 4200, dampingRebound: 5600,
      progressive: 0.35, bump: 90000, bumpStopRange: 0.12
    },
    suspensionGeometry: {
      front: {
        camberStatic: -0.5, camberGain: -22, toeStatic: 0, toeGain: 0.35,
        rollCenterHeight: 0.12, rollCenterTravelGain: 0.10, rollCenterLateralGain: 0.08,
        rollCamberGain: 0.28, casterDeg: 5.5, kingpinInclinationDeg: 8.5,
        scrubRadius: 0.035, mechanicalTrail: 0.045,
        hardpoints: {
          enabled: true,
          upperInner: { x: 0.28, y: 0.82 }, upperOuter: { x: 0.68, y: 0.72 },
          lowerInner: { x: 0.30, y: 0.34 }, lowerOuter: { x: 0.76, y: 0.30 },
          wheelCenterX: 0.88
        }
      },
      rear: {
        camberStatic: -1.0, camberGain: -15, toeStatic: 0.08, toeGain: -0.22,
        rollCenterHeight: 0.17, rollCenterTravelGain: 0.07, rollCenterLateralGain: 0.06,
        rollCamberGain: 0.18, casterDeg: 0, kingpinInclinationDeg: 0,
        scrubRadius: 0, mechanicalTrail: 0,
        hardpoints: {
          enabled: true,
          upperInner: { x: 0.30, y: 0.80 }, upperOuter: { x: 0.69, y: 0.70 },
          lowerInner: { x: 0.32, y: 0.33 }, lowerOuter: { x: 0.77, y: 0.29 },
          wheelCenterX: 0.89
        }
      }
    },
    tire: {
      mu: 1.08,
      corneringStiffness: 78000,
      longitudinalStiffness: 95000,
      relaxation: 0.04,
      relaxationAngle: 0.08,
      relaxationLength: 0.35,
      loadSensitivity: 0.12,
      referenceLoad: 3000,
      aligningTrail: 0.08,
      camberThrustGain: 0.08,
      camberStiffness: 0.55,
      rollingResistance: 0.012,
      pacejka: { enabled: false, B: 0, C: 1.3, E: 0.97, peak: 1 },
      frictionEllipse: { longitudinal: 1, lateral: 1 },
      combined: { enabled: true, C: 1.35, E: 0.92, peak: 1.0 }
    },
    engine: {
      peakTorque: 280, idleRpm: 900, redline: 6500,
      finalDrive: 3.9, gears: [3.2, 2.1, 1.5, 1.18, 0.94, 0.78],
      efficiency: 0.88,
      engineInertia: 0.22,
      shiftTime: 0.16,
      engineBraking: 46,
      revLimiter: 6650,
      torqueCurve: null
    },
    airControl: {
      yawRate: 0.9, pitchRate: 0.8, rollRate: 1.0,
      yawGain: 2200, pitchGain: 3000, rollGain: 3000
    },
    antiRoll: { front: 8500, rear: 6500 },
    body: { rollStiffness: 9000, rollDamping: 2200, pitchStiffness: 14000, pitchDamping: 2800 },
    differential: { type: 'open', limitedSlip: 2.2, preload: 0, ramp: 0 },
    aids: { abs: true, tcs: true, esc: false, absSlip: 0.18, tcsSlip: 0.22, escYawThreshold: 0.14, escBrakeTorque: 1600 },
    steering: { maxAngle: 0.48, highSpeedScale: 0.42, speedReference: 55 },
    brakeTorque: 4200,
    handbrakeTorque: 6500,
    dragArea: 0.68,
    airDensity: 1.225,
    downforceArea: 0.0,
    downforceFront: 0.45,
    // Weather / environment. `wind` is the air velocity in world space (m/s);
    // drag, downforce and the crosswind side force all use relative airspeed.
    wind: { x: 0, y: 0, z: 0 },
    sideArea: 1.15,
    aeroYawArm: 0.35,
    restitution: 0.08,
    collisionFriction: 0.55,
    bodyHalf: { x: 0.88, y: 0.48, z: 2.05 },
    ccd: true,
    solver: { velocityIterations: 10, positionIterations: 4, slop: 0.005 },
    collisionSurface: true
  };

  const Core = (typeof module !== 'undefined' && module.exports)
    ? require('./rigidbody-core.js')
    : root.PlayCarRigidBodyCore;

  if (!Core || !Core.World) {
    throw new Error('PlayCarRigidBody: rigidbody-core.js must be loaded before rigidbody-vehicle-engine.js');
  }

  /* ------------------------------------------------------------------ */
  /* Utilities                                                          */
  /* ------------------------------------------------------------------ */

  function clone(o) { return JSON.parse(JSON.stringify(o)); }

  function isPlainObject(value) {
    return value !== null && typeof value === 'object' && !Array.isArray(value);
  }

  function mergeConfig(base, patch) {
    if (!isPlainObject(patch)) return patch === undefined ? base : patch;
    const out = isPlainObject(base) ? { ...base } : {};
    for (const key of Object.keys(patch)) {
      if (isPlainObject(patch[key]) && isPlainObject(out[key])) out[key] = mergeConfig(out[key], patch[key]);
      else out[key] = patch[key];
    }
    return out;
  }

  function circleIntersections(a, ra, b, rb) {
    const dx = b.x - a.x, dy = b.y - a.y, d = Math.hypot(dx, dy);
    if (d < 1e-8 || d > ra + rb + 1e-8 || d < Math.abs(ra - rb) - 1e-8) return [];
    const along = (ra * ra - rb * rb + d * d) / (2 * d);
    const height = Math.sqrt(Math.max(0, ra * ra - along * along));
    const x = a.x + along * dx / d, y = a.y + along * dy / d;
    const px = -dy * height / d, py = dx * height / d;
    return [{ x: x + px, y: y + py }, { x: x - px, y: y - py }];
  }

  function lineIntersection(a, b, c, d) {
    const rx = b.x - a.x, ry = b.y - a.y, sx = d.x - c.x, sy = d.y - c.y;
    const den = rx * sy - ry * sx;
    if (Math.abs(den) < 1e-8) return null;
    const t = ((c.x - a.x) * sy - (c.y - a.y) * sx) / den;
    return { x: a.x + t * rx, y: a.y + t * ry };
  }

  function rotateVectorAroundAxis(vector, axis, angle) {
    const a = V.norm(axis), c = Math.cos(angle), sn = Math.sin(angle);
    return V.add(V.add(V.mul(vector, c), V.mul(V.cross(a, vector), sn)), V.mul(a, V.dot(a, vector) * (1 - c)));
  }

  function solveHardpointKinematics(g, compression, fallbackHeight) {
    const hp = g.hardpoints;
    if (!hp || !hp.enabled) return null;
    const ui = hp.upperInner, li = hp.lowerInner, uo = hp.upperOuter, lo = hp.lowerOuter;
    if (![ui, li, uo, lo].every(p => Number.isFinite(p && p.x) && Number.isFinite(p && p.y))) {
      return { valid: false, rollCenterHeight: fallbackHeight, wheelOffsetX: 0, camberDeltaDeg: 0 };
    }
    const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
    const lowerLength = distance(li, lo), upperLength = distance(ui, uo), uprightLength = distance(uo, lo);
    const solveAt = travel => {
      const y = lo.y + travel;
      const dy = y - li.y;
      const remain = lowerLength * lowerLength - dy * dy;
      if (remain < 0) return null;
      const dx = Math.sqrt(Math.max(0, remain));
      const lower = { x: li.x + (lo.x >= li.x ? dx : -dx), y };
      const candidates = circleIntersections(ui, upperLength, lower, uprightLength);
      if (!candidates.length) return null;
      candidates.sort((a, b) => distance(a, uo) - distance(b, uo));
      return { lower, upper: candidates[0] };
    };
    const current = solveAt(compression), baseline = solveAt(0);
    if (!current || !baseline) {
      return { valid: false, rollCenterHeight: fallbackHeight, wheelOffsetX: 0, camberDeltaDeg: 0, instantCenter: null };
    }
    const angle = p => Math.atan2(p.upper.x - p.lower.x, p.upper.y - p.lower.y);
    const camberDeltaDeg = (angle(current) - angle(baseline)) * 180 / Math.PI;
    const ic = lineIntersection(ui, current.upper, li, current.lower);
    const contactX = Number.isFinite(hp.contactPatchX) ? hp.contactPatchX : (hp.wheelCenterX ?? 0.88);
    let rollCenterHeight = fallbackHeight;
    if (ic && Math.abs(ic.x - contactX) > 1e-5) {
      const candidate = ic.y * (0 - contactX) / (ic.x - contactX);
      if (Number.isFinite(candidate)) rollCenterHeight = V.clamp(candidate, -2, 2);
    }
    return {
      valid: true, rollCenterHeight, wheelOffsetX: current.lower.x - lo.x,
      camberDeltaDeg, instantCenter: ic, upperOuter: current.upper, lowerOuter: current.lower
    };
  }

  /* ------------------------------------------------------------------ */
  /* Tire models                                                        */
  /* ------------------------------------------------------------------ */

  // Backwards-compatible solver used by existing integrations and tests.
  function calculateTireForces(FxRaw, slipAngle, normalLoad, frictionCoefficient, tire = {}) {
    const cap = Math.max(0, frictionCoefficient * normalLoad);
    const pacejka = tire.pacejka || {};
    let FyRaw;
    if (pacejka.enabled) {
      const C = Math.max(0.1, Number(pacejka.C) || 1.3);
      const E = Number.isFinite(Number(pacejka.E)) ? Number(pacejka.E) : 0.97;
      const peak = Math.max(0, Number(pacejka.peak ?? 1));
      const D = cap * peak;
      const B = Number(pacejka.B) > 0
        ? Number(pacejka.B)
        : Math.max(0.001, (Number(tire.corneringStiffness) || 78000) / (C * Math.max(D, 1)));
      const x = B * slipAngle;
      FyRaw = -D * Math.sin(C * Math.atan(x - E * (x - Math.atan(x))));
    } else {
      FyRaw = -(Number(tire.corneringStiffness) || 78000) * slipAngle;
    }
    const ellipse = tire.frictionEllipse || {};
    const longitudinal = Math.max(0.01, Number(ellipse.longitudinal) || 1);
    const lateral = Math.max(0.01, Number(ellipse.lateral) || 1);
    const nx = cap * longitudinal, ny = cap * lateral;
    const utilization = cap > 0 ? Math.hypot(FxRaw / nx, FyRaw / ny) : Infinity;
    const scale = utilization > 1 ? 1 / utilization : 1;
    const Fx = cap > 0 ? FxRaw * scale : 0;
    const Fy = cap > 0 ? FyRaw * scale : 0;
    return { Fx, Fy, FyRaw, utilization: cap > 0 ? Math.hypot(Fx / nx, Fy / ny) : 0, scale };
  }

  function magicFormula(x, B, C, D, E) {
    return D * Math.sin(C * Math.atan(B * x - E * (B * x - Math.atan(B * x))));
  }

  /**
   * Combined-slip tire force using the normalised-slip similarity method.
   * Longitudinal and lateral slips are combined into one slip vector before
   * the Magic Formula is evaluated, so the resulting force always lies inside
   * a single friction circle and turn-in / power-oversteer stay coupled.
   */
  function calculateCombinedTireForces(options = {}) {
    const normalLoad = Math.max(0, Number(options.normalLoad) || 0);
    const frictionCoefficient = Math.max(0, Number(options.frictionCoefficient) || 0);
    const cap = normalLoad * frictionCoefficient;
    if (cap <= 1e-9) {
      return { Fx: 0, Fy: 0, FyRaw: 0, util: 0, utilization: 0, scale: 1, combinedSlip: 0 };
    }
    const tire = options.tire || {};
    const combined = tire.combined || {};
    const slipRatio = Number(options.slipRatio) || 0;
    const slipAngle = Number(options.slipAngle) || 0;
    const C = Math.max(0.1, Number(combined.C) || 1.35);
    const E = V.clamp(Number.isFinite(Number(combined.E)) ? Number(combined.E) : 0.92, -1, 1);
    const peak = Math.max(0.05, Number(combined.peak) || 1);
    const D = cap * peak;
    const corneringStiffness = Number(tire.corneringStiffness) || 78000;
    const B = Number(combined.B) > 0 ? Number(combined.B) : Math.max(0.5, corneringStiffness / (C * Math.max(D, 1)));

    // Normalised slips keep the combined force continuous as either slip grows.
    const denom = 1 + Math.abs(slipRatio);
    const sx = slipRatio / denom;
    const sy = Math.tan(slipAngle) / denom;
    const s = Math.hypot(sx, sy);
    const F = s > 1e-9 ? magicFormula(s, B, C, D, E) : 0;
    const ux = s > 1e-9 ? sx / s : 0;
    const uy = s > 1e-9 ? sy / s : 0;

    let Fx = F * ux;
    let Fy = -F * uy;

    // Camber thrust: a cambered tire produces lateral force even at zero slip.
    const camber = Number(options.camber) || 0;
    if (camber && tire.camberStiffness) Fy -= camber * cap * Math.max(0, Number(tire.camberStiffness));

    // Rolling resistance opposes the wheel's rolling direction.
    const rollingResistance = Math.max(0, Number(tire.rollingResistance) || 0);
    if (rollingResistance) {
      const rollingSign = Math.sign(options.wheelSurfaceSpeed ?? options.longitudinalSpeed ?? 0);
      if (rollingSign) Fx -= rollingSign * rollingResistance * normalLoad;
    }

    let magnitude = Math.hypot(Fx, Fy);
    let scale = 1;
    if (magnitude > cap) {
      scale = cap / magnitude;
      Fx *= scale;
      Fy *= scale;
      magnitude = cap;
    }
    return {
      Fx, Fy, FyRaw: -F, utilization: magnitude / cap, util: magnitude / cap, scale,
      combinedSlip: s, directionX: ux, directionY: uy
    };
  }

  /* ------------------------------------------------------------------ */
  /* Vehicle                                                            */
  /* ------------------------------------------------------------------ */

  const MOUNTS = [[-1, -1], [1, -1], [-1, 1], [1, 1]]; // [side, axle]

  class RigidBodyVehicle {
    constructor(options = {}) {
      const config = mergeConfig(clone(DEFAULTS), options.config || {});
      this.c = config;
      // Keep arrays and nested special values exact.
      this.c.inertia = mergeConfig(clone(DEFAULTS.inertia), options.config?.inertia || {});
      this.c.com = mergeConfig(clone(DEFAULTS.com), options.config?.com || {});
      this.c.suspension = mergeConfig(clone(DEFAULTS.suspension), options.config?.suspension || {});
      this.c.suspensionGeometry = {
        front: mergeConfig(clone(DEFAULTS.suspensionGeometry.front), options.config?.suspensionGeometry?.front || {}),
        rear: mergeConfig(clone(DEFAULTS.suspensionGeometry.rear), options.config?.suspensionGeometry?.rear || {})
      };
      this.c.tire = mergeConfig(clone(DEFAULTS.tire), options.config?.tire || {});
      this.c.tire.pacejka = mergeConfig(clone(DEFAULTS.tire.pacejka), options.config?.tire?.pacejka || {});
      this.c.tire.combined = mergeConfig(clone(DEFAULTS.tire.combined), options.config?.tire?.combined || {});
      this.c.tire.frictionEllipse = mergeConfig(clone(DEFAULTS.tire.frictionEllipse), options.config?.tire?.frictionEllipse || {});
      this.c.engine = mergeConfig(clone(DEFAULTS.engine), options.config?.engine || {});
      this.c.antiRoll = mergeConfig(clone(DEFAULTS.antiRoll), options.config?.antiRoll || {});
      this.c.body = mergeConfig(clone(DEFAULTS.body), options.config?.body || {});
      this.c.differential = mergeConfig(clone(DEFAULTS.differential), options.config?.differential || {});
      this.c.aids = mergeConfig(clone(DEFAULTS.aids), options.config?.aids || {});
      this.c.steering = mergeConfig(clone(DEFAULTS.steering), options.config?.steering || {});
      this.c.solver = mergeConfig(clone(DEFAULTS.solver), options.config?.solver || {});
      this.c.bodyHalf = mergeConfig(clone(DEFAULTS.bodyHalf), options.config?.bodyHalf || {});

      this.mesh = options.surfaceMesh || null;
      this.colliders = options.colliderMeshes || [];
      this.groundY = options.groundPlaneY ?? null;

      this.accumulator = 0;
      this.gear = 1;
      this.engineRpm = this.c.engine.idleRpm;
      this.shiftTimer = 0;
      this.tick = 0;

      const start = options.position || { x: 0, y: 0.65, z: 0 };
      this.state = {
        position: { ...start },
        velocity: V.zero(),
        orientation: options.orientation ? { ...options.orientation } : Q.id(),
        angularVelocity: V.zero(),
        wheelOmega: [0, 0, 0, 0],
        wheelSpin: [0, 0, 0, 0],
        grounded: false
      };
      if (options.velocity) this.state.velocity = { ...options.velocity };
      this.previous = clone(this.state);
      this.lastContacts = [];
      this.telemetry = {};
      this._slipState = [0, 0, 0, 0];
      this._slipAngleState = [0, 0, 0, 0];
      this.aidState = { abs: false, tcs: false, esc: false };
      this.lastInput = {};
      this.world = null;
      this.chassis = null;
      this._staticBodies = { surface: null, colliders: [] };
      this._buildWorld();
    }

    /* ---------------- collision world ---------------- */

    _buildWorld() {
      const c = this.c;
      this.world = new Core.World({
        gravity: { x: 0, y: -c.gravity, z: 0 },
        fixedDt: c.fixedDt,
        velocityIterations: c.solver.velocityIterations,
        positionIterations: c.solver.positionIterations,
        slop: c.solver.slop,
        allowSleep: true
      });
      this.chassis = new Core.Body({
        type: 'dynamic',
        mass: c.mass,
        inertia: { ...c.inertia },
        shape: new Core.BoxShape(c.bodyHalf),
        position: this.state.position,
        orientation: this.state.orientation,
        velocity: this.state.velocity,
        angularVelocity: this.state.angularVelocity,
        friction: c.collisionFriction,
        restitution: c.restitution,
        linearDamping: 0,
        angularDamping: 0,
        allowSleep: false,
        ccd: c.ccd
      });
      this.world.addBody(this.chassis);
      if (Number.isFinite(this.groundY)) {
        this.world.addGroundPlane(this.groundY, { friction: 1, restitution: 0.02 });
      }
      this._refreshStaticBodies();
    }

    _toCoreMesh(mesh) {
      if (!mesh) return null;
      if (mesh instanceof Core.MeshShape) return mesh;
      if (Array.isArray(mesh.triangles) && mesh.triangles.length) {
        if (mesh._playcarCoreMesh && mesh._playcarCoreMeshSource === mesh.triangles.length) return mesh._playcarCoreMesh;
        const coreMesh = Core.MeshShape.fromLegacyMesh(mesh);
        mesh._playcarCoreMesh = coreMesh;
        mesh._playcarCoreMeshSource = mesh.triangles.length;
        return coreMesh;
      }
      if (typeof mesh.raycast === 'function' && typeof mesh.queryAABB === 'function') return mesh;
      return null;
    }

    _refreshStaticBodies() {
      if (!this.world) return;
      for (const body of Object.values(this._staticBodies).flat()) {
        if (body) this.world.removeBody(body);
      }
      this._staticBodies = { surface: null, colliders: [] };
      if (this.c.collisionSurface) {
        const mesh = this._toCoreMesh(this.mesh);
        if (mesh) this._staticBodies.surface = this.world.addStaticMesh(mesh, {
          friction: 1, restitution: 0.02
        });
      }
      for (const collider of this.colliders || []) {
        const mesh = this._toCoreMesh(collider);
        if (mesh) {
          this._staticBodies.colliders.push(this.world.addStaticMesh(mesh, {
            friction: this.c.collisionFriction, restitution: this.c.restitution
          }));
        }
      }
    }

    setSurfaceMesh(mesh) {
      this.mesh = mesh;
      this._refreshStaticBodies();
      return this;
    }

    setColliderMeshes(meshes) {
      this.colliders = meshes || [];
      this._refreshStaticBodies();
      return this;
    }

    /** Add a dynamic rigid body (cone, barrel, crate) to the vehicle's world. */
    addProp(options = {}) {
      const shape = options.shape === 'sphere'
        ? new Core.SphereShape(options.radius ?? 0.35)
        : new Core.BoxShape(options.halfExtents || { x: 0.25, y: 0.35, z: 0.25 });
      const body = new Core.Body({
        mass: options.mass ?? 12,
        shape,
        position: options.position || { x: 0, y: 1, z: 0 },
        orientation: options.orientation,
        friction: options.friction ?? 0.7,
        restitution: options.restitution ?? 0.15,
        linearDamping: options.linearDamping ?? 0.08,
        angularDamping: options.angularDamping ?? 0.12
      });
      this.world.addBody(body);
      return body;
    }

    addStaticBox(halfExtents, position, options = {}) {
      const body = new Core.Body({
        type: 'static',
        shape: new Core.BoxShape(halfExtents),
        position,
        friction: options.friction ?? 0.8,
        restitution: options.restitution ?? 0.1
      });
      this.world.addBody(body);
      return body;
    }

    /* ---------------- state helpers ---------------- */

    _syncBodyFromState() {
      const body = this.chassis, s = this.state;
      if (!body) return;
      body.position = { ...s.position };
      body.orientation = { ...s.orientation };
      body.linearVelocity = { ...s.velocity };
      body.angularVelocity = { ...s.angularVelocity };
      body.updateAABB();
    }

    _syncStateFromBody() {
      const body = this.chassis, s = this.state;
      s.position = { ...body.position };
      s.orientation = { ...body.orientation };
      s.velocity = { ...body.linearVelocity };
      s.angularVelocity = { ...body.angularVelocity };
    }

    _invInertiaWorld(v) {
      const q = this.state.orientation;
      const local = Q.rotateInverse(q, v);
      const i = this.c.inertia;
      return Q.rotate(q, { x: local.x / i.x, y: local.y / i.y, z: local.z / i.z });
    }

    _pointVelocity(r) {
      return V.add(this.state.velocity, V.cross(this.state.angularVelocity, r));
    }

    _applyForce(F, r) {
      if (!this.chassis) {
        this.state.velocity = V.add(this.state.velocity, V.mul(F, this.c.fixedDt / this.c.mass));
        this.state.angularVelocity = V.add(this.state.angularVelocity,
          V.mul(this._invInertiaWorld(V.cross(r, F)), this.c.fixedDt));
        return;
      }
      this.chassis.applyForce(F, V.add(this.state.position, r || V.zero()));
    }

    _applyTorque(torque) {
      if (!this.chassis) {
        this.state.angularVelocity = V.add(this.state.angularVelocity,
          V.mul(this._invInertiaWorld(torque), this.c.fixedDt));
        return;
      }
      this.chassis.applyTorque(torque);
    }

    reset(position = { x: 0, y: 0.65, z: 0 }) {
      this.state = {
        position: { ...position },
        velocity: V.zero(),
        orientation: Q.id(),
        angularVelocity: V.zero(),
        wheelOmega: [0, 0, 0, 0],
        wheelSpin: [0, 0, 0, 0],
        grounded: false
      };
      this.previous = clone(this.state);
      this.accumulator = 0;
      this.tick = 0;
      this.gear = 1;
      this.engineRpm = this.c.engine.idleRpm;
      this.shiftTimer = 0;
      this._slipState = [0, 0, 0, 0];
      this._slipAngleState = [0, 0, 0, 0];
      this.aidState = { abs: false, tcs: false, esc: false };
      this.lastContacts = [];
      this.telemetry = {};
      if (this.chassis) {
        this.chassis.position = { ...position };
        this.chassis.orientation = Q.id();
        this.chassis.linearVelocity = V.zero();
        this.chassis.angularVelocity = V.zero();
        this.chassis.sleeping = false;
        this.chassis.sleepTimer = 0;
        this.chassis.updateAABB();
      }
      if (this.world) {
        this.world.time = 0;
        this.world.stepCount = 0;
        this.world.accumulator = 0;
        this.world._warmCache = new Map();
      }
      return this;
    }

    /* ---------------- wheel geometry / contacts ---------------- */

    _wheelKinematics(index, compression, normal, input = {}) {
      const c = this.c, s = this.state, front = index < 2, side = (index === 0 || index === 2) ? -1 : 1;
      const g = front ? c.suspensionGeometry.front : c.suspensionGeometry.rear;
      const tilt = this._bodyTilt();
      const travel = V.clamp(compression / Math.max(0.001, c.suspension.max - c.suspension.min), -1, 1);
      const fallbackRollCenter = Math.max(0, g.rollCenterHeight + g.rollCenterTravelGain * compression + g.rollCenterLateralGain * side * compression);
      const hardpoint = solveHardpointKinematics(g, compression, fallbackRollCenter);
      // Static alignment (camber, toe) is mirrored per side: both wheels of an
      // axle lean/point inward. Applying the same signed angle to both sides
      // would steer the whole axle and make the car drift at speed.
      const alignmentCamberDeg = g.camberStatic + g.camberGain * compression +
        (hardpoint && hardpoint.valid ? hardpoint.camberDeltaDeg : 0);
      const camberDeg = side * alignmentCamberDeg + g.rollCamberGain * tilt.roll * 180 / Math.PI;
      const toeDeg = side * (g.toeStatic + g.toeGain * compression);
      const speed = V.len(s.velocity);
      const steer = front
        ? V.clamp(input.steer || 0, -1, 1) * c.steering.maxAngle *
          (1 - (1 - c.steering.highSpeedScale) * V.clamp(speed / (c.steering.speedReference || 55), 0, 1))
        : 0;
      const n = V.norm(normal);
      const bodyForward = Q.rotate(s.orientation, { x: 0, y: 0, z: -1 });
      let forward = V.norm(V.sub(bodyForward, V.mul(n, V.dot(bodyForward, n))));
      if (V.len(forward) < 1e-8) forward = V.norm(V.cross({ x: 1, y: 0, z: 0 }, n));
      const right0 = V.norm(V.cross(forward, n));
      const angle = steer + toeDeg * Math.PI / 180;
      const caster = V.clamp(Number(g.casterDeg) || 0, -25, 25) * Math.PI / 180;
      const kpi = V.clamp(Number(g.kingpinInclinationDeg) || 0, 0, 30) * Math.PI / 180;
      const steeringAxis = V.mul(V.norm(V.add(
        V.mul(n, Math.cos(kpi) * Math.cos(caster)),
        V.add(V.mul(right0, -side * Math.sin(kpi) * Math.cos(caster)), V.mul(forward, -Math.sin(caster)))
      )), -1);
      const steered = rotateVectorAroundAxis(forward, steeringAxis, angle);
      forward = V.norm(V.sub(steered, V.mul(n, V.dot(steered, n))));
      const right = V.norm(V.cross(forward, n));
      const steeringNormal = rotateVectorAroundAxis(n, steeringAxis, angle);
      const steeringCamberDeg = Math.atan2(V.dot(steeringNormal, right0), Math.max(1e-8, V.dot(steeringNormal, n))) * 180 / Math.PI;
      const effectiveCamberDeg = camberDeg + steeringCamberDeg;
      const scrubRadius = Math.max(0, Number(g.scrubRadius) || 0);
      const mechanicalTrail = Math.max(0, Number(g.mechanicalTrail) || 0);
      const jackingHeight = scrubRadius * Math.sin(angle) * Math.sin(kpi);
      const camberedNormal = Q.rotate(Q.axis(forward, effectiveCamberDeg * Math.PI / 180), n);
      const camberedRight = V.norm(V.cross(forward, camberedNormal));
      const nominalRollCenter = front ? DEFAULTS.suspensionGeometry.front.rollCenterHeight : DEFAULTS.suspensionGeometry.rear.rollCenterHeight;
      const rollCenterHeight = hardpoint && hardpoint.valid
        ? hardpoint.rollCenterHeight + (fallbackRollCenter - nominalRollCenter)
        : fallbackRollCenter;
      return {
        camber: effectiveCamberDeg * Math.PI / 180, camberDeg: effectiveCamberDeg, steeringCamberDeg,
        toe: toeDeg * Math.PI / 180, toeDeg, steer, forward, right, steeringAxis,
        casterDeg: caster * 180 / Math.PI, kingpinInclinationDeg: kpi * 180 / Math.PI,
        scrubRadius, mechanicalTrail, jackingHeight, camberedNormal, camberedRight,
        rollCenterHeight, travel, side, axle: front ? 'front' : 'rear',
        hardpointValid: !!(hardpoint && hardpoint.valid),
        instantCenter: (hardpoint && hardpoint.instantCenter) || null,
        upperOuter: (hardpoint && hardpoint.upperOuter) || null,
        lowerOuter: (hardpoint && hardpoint.lowerOuter) || null,
        wheelOffsetX: (hardpoint && hardpoint.wheelOffsetX) || 0
      };
    }

    _contacts(input = {}) {
      const c = this.c, s = this.state, out = [], q = s.orientation;
      const halfWheelbase = c.wheelbase / 2, halfTrack = c.track / 2;
      for (let i = 0; i < 4; i++) {
        const mount = MOUNTS[i];
        const local = { x: mount[0] * halfTrack, y: 0, z: mount[1] * halfWheelbase };
        const r = Q.rotate(q, local);
        const origin = V.add(s.position, r);
        const reach = c.suspension.max + c.wheelRadius;
        let hit = null;
        if (this.mesh && typeof this.mesh.closestPoint === 'function') {
          hit = this.mesh.closestPoint(origin, reach + c.wheelRadius, { x: 0, y: 1, z: 0 });
        }
        if (!hit && Number.isFinite(this.groundY)) {
          const dist = origin.y - this.groundY;
          if (dist < reach + c.wheelRadius) {
            hit = {
              point: { x: origin.x, y: this.groundY, z: origin.z },
              normal: { x: 0, y: 1, z: 0 },
              distance: Math.max(0, dist),
              signedDistance: dist - c.wheelRadius
            };
          }
        }
        if (!hit) continue;
        let n = V.norm(hit.normal || { x: 0, y: 1, z: 0 });
        if (V.dot(n, { x: 0, y: 1, z: 0 }) < 0) n = V.mul(n, -1);
        const rawLength = hit.distance - c.wheelRadius;
        const length = V.clamp(rawLength, c.suspension.min, c.suspension.max);
        const compression = c.suspension.rest - length;
        const geometry = this._wheelKinematics(i, compression, n, input);
        const rawPoint = hit.point || V.sub(origin, V.mul(n, hit.distance));
        const bodyRight = Q.rotate(q, { x: 1, y: 0, z: 0 });
        const tangentRight = V.norm(V.sub(bodyRight, V.mul(n, V.dot(bodyRight, n))));
        const point = V.add(rawPoint, V.mul(tangentRight, geometry.wheelOffsetX * geometry.side));
        const arm = V.sub(point, s.position);
        const pv = this._pointVelocity(arm);
        const vn = V.dot(pv, n);
        const wheelCenter = V.add(point, V.mul(n, c.wheelRadius + geometry.jackingHeight));
        out.push({
          index: i, point, normal: n, arm, compression, length, rawLength,
          normalVelocity: vn, velocity: pv, local, wheelCenter, geometry,
          tireForward: geometry.forward, tireRight: geometry.right,
          friction: Number.isFinite(hit.friction) ? Math.max(0, hit.friction) : 1,
          restitution: Number.isFinite(hit.restitution) ? hit.restitution : 0.05
        });
      }
      return out;
    }

    _bodyTilt() {
      const q = this.state.orientation;
      const up = Q.rotate(q, { x: 0, y: 1, z: 0 });
      const localAngularVelocity = Q.rotateInverse(q, this.state.angularVelocity);
      return {
        roll: Math.atan2(-up.x, Math.max(1e-9, up.y)),
        pitch: Math.atan2(up.z, Math.max(1e-9, up.y)),
        rollRate: localAngularVelocity.z,
        pitchRate: localAngularVelocity.x
      };
    }

    _applyBodyStabilization(input = {}) {
      // On the ground the spring keeps the body planted. In the air it only
      // levels the car passively; active pitch/roll input disables it so flips
      // and aerials stay possible.
      if (!this.state.grounded && ((Number(input.pitch) || 0) !== 0 || (Number(input.roll) || 0) !== 0)) return;
      const b = this.c.body || {}, tilt = this._bodyTilt();
      const localTorque = {
        x: -Math.max(0, b.pitchStiffness || 0) * tilt.pitch - Math.max(0, b.pitchDamping || 0) * tilt.pitchRate,
        y: 0,
        z: -Math.max(0, b.rollStiffness || 0) * tilt.roll - Math.max(0, b.rollDamping || 0) * tilt.rollRate
      };
      if (V.len(localTorque) > 0) this._applyTorque(Q.rotate(this.state.orientation, localTorque));
    }

    /* ---------------- drivetrain ---------------- */

    _drivetrainLayout() {
      const c = this.c;
      const rawType = c.differential.type || 'open';
      const driveType = c.differential.layout || (['fwd', 'rwd', 'awd'].includes(rawType) ? rawType : 'rwd');
      const diffType = ['lsd', 'limited-slip', 'locked'].includes(rawType)
        ? (rawType === 'locked' ? 'locked' : 'lsd')
        : 'open';
      const driven = [0, 1, 2, 3].filter(i => driveType === 'awd' ||
        (driveType === 'fwd' && i < 2) ||
        (driveType !== 'fwd' && driveType !== 'awd' && i >= 2));
      return { driveType, diffType, driven };
    }

    _engineTorqueAtRpm(rpm) {
      const e = this.c.engine;
      if (Array.isArray(e.torqueCurve) && e.torqueCurve.length) {
        const curve = e.torqueCurve;
        if (rpm <= curve[0][0]) return e.peakTorque * curve[0][1];
        for (let i = 1; i < curve.length; i++) {
          if (rpm <= curve[i][0]) {
            const [r0, t0] = curve[i - 1], [r1, t1] = curve[i];
            const t = (rpm - r0) / Math.max(1, r1 - r0);
            return e.peakTorque * (t0 + (t1 - t0) * t);
          }
        }
        return e.peakTorque * curve[curve.length - 1][1];
      }
      return e.peakTorque * V.clamp(1 - Math.abs(rpm - 3800) / 6500, 0.25, 1);
    }

    _updateDrivetrain(dt, input, wheelSpeeds) {
      const c = this.c, e = c.engine, s = this.state;
      const drivenWheels = this._layout.driven;
      const drivenSpeed = drivenWheels.length
        ? drivenWheels.reduce((sum, i) => sum + Math.abs(s.wheelOmega[i]), 0) / drivenWheels.length
        : s.wheelOmega.reduce((sum, w) => sum + Math.abs(w), 0) / 4;
      const gearRatio = e.gears[this.gear - 1] || 1;
      const wheelRpm = drivenSpeed * c.wheelRadius * e.finalDrive * gearRatio * 60 / (2 * Math.PI);
      this.engineRpm = V.clamp(wheelRpm, e.idleRpm, e.redline);

      if (this.shiftTimer > 0) {
        this.shiftTimer = Math.max(0, this.shiftTimer - dt);
      } else {
        if (this.engineRpm > e.redline * 0.96 && this.gear < e.gears.length) {
          this.gear++;
          this.shiftTimer = Math.max(0, Number(e.shiftTime) || 0);
        } else if (this.engineRpm < e.redline * 0.55 && this.gear > 1) {
          this.gear--;
          this.shiftTimer = Math.max(0, Number(e.shiftTime) || 0);
        }
      }

      let throttle = V.clamp(input.throttle || 0, 0, 1);
      if (this.engineRpm > (e.revLimiter || e.redline)) throttle = 0;
      const shifting = this.shiftTimer > 0;
      let torque = this._engineTorqueAtRpm(this.engineRpm) * throttle;
      if (shifting) torque = 0;

      // Engine braking when the driver is off the throttle.
      const longitudinalSpeed = V.dot(s.velocity, Q.rotate(s.orientation, { x: 0, y: 0, z: -1 }));
      if (throttle < 0.02 && !shifting && Math.abs(longitudinalSpeed) > 0.5) {
        const brakeTorque = -(Number(e.engineBraking) || 0) * (this.engineRpm / Math.max(1, e.redline)) *
          Math.sign(longitudinalSpeed);
        torque += brakeTorque;
      }

      const totalDriveTorque = torque * gearRatio * e.finalDrive * e.efficiency;
      return { totalDriveTorque, throttle, drivenWheels, gearRatio, shifting, longitudinalSpeed };
    }

    _distributeDriveTorque(totalDriveTorque) {
      const c = this.c, s = this.state;
      const wheelDriveTorque = [0, 0, 0, 0];
      const driven = this._layout.driven;
      const diffType = this._layout.diffType;
      const limitedSlip = Math.max(1, Number(c.differential.limitedSlip) || 1);
      const preload = Math.max(0, Number(c.differential.preload) || 0);
      for (const axle of [[0, 1], [2, 3]]) {
        const pair = axle.filter(i => driven.includes(i));
        if (!pair.length) continue;
        const base = totalDriveTorque / Math.max(1, driven.length);
        if (pair.length === 2 && diffType === 'locked') {
          wheelDriveTorque[pair[0]] = base;
          wheelDriveTorque[pair[1]] = base;
        } else if (pair.length === 2 && diffType === 'lsd') {
          const a = pair[0], b = pair[1];
          const wa = Math.abs(s.wheelOmega[a]), wb = Math.abs(s.wheelOmega[b]);
          if (Math.abs(wa - wb) < 0.25) {
            wheelDriveTorque[a] = base;
            wheelDriveTorque[b] = base;
          } else {
            const slow = wa < wb ? a : b;
            const fast = slow === a ? b : a;
            const bias = Math.max(1, limitedSlip + preload * 0.001);
            wheelDriveTorque[slow] = base * 2 * bias / (1 + bias);
            wheelDriveTorque[fast] = base * 2 / (1 + bias);
          }
        } else {
          for (const i of pair) wheelDriveTorque[i] = base;
        }
      }
      return wheelDriveTorque;
    }

    /* ---------------- fixed step ---------------- */

    // Redistribute support load proportionally within an axle/side. Equal
    // per-wheel deltas can drive an already lightly loaded wheel negative;
    // proportional shares keep the sum exactly conserved and non-negative.
    _distributeLoad(contacts, delta, normalLoad, suspensionTelemetry) {
      if (!contacts.length || delta === 0) return;
      const total = contacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      if (total <= 1e-9) return;
      for (const h of contacts) {
        const share = normalLoad[h.index] / total;
        normalLoad[h.index] = Math.max(0, normalLoad[h.index] + delta * share);
        const applied = normalLoad[h.index] - suspensionTelemetry[h.index].normalLoad;
        if (applied) this._applyForce(V.mul(h.normal, applied), h.arm);
        suspensionTelemetry[h.index].normalLoad = normalLoad[h.index];
      }
    }

    _fixed(dt, input) {
      const c = this.c, s = this.state;
      this.previous = clone(s);
      this.aidState = { abs: false, tcs: false, esc: false };
      this._syncBodyFromState();

      const contacts = this._contacts(input);
      this.lastContacts = contacts;
      const normalLoad = [0, 0, 0, 0];
      const forces = [];
      const suspensionTelemetry = Array.from({ length: 4 }, (_, wheel) => ({
        wheel, compression: 0, rawLength: null, bumpStopCompression: 0, bumpStopForce: 0,
        springForce: 0, damperForce: 0, normalLoad: 0, velocity: 0
      }));

      /* 1. suspension: progressive spring, asymmetric damper, bump stop. */
      for (const h of contacts) {
        const susp = c.suspension;
        const bumpStopCompression = Math.max(0, susp.min - h.rawLength);
        const bumpStopForce = Math.max(0, susp.bump || 0) * bumpStopCompression *
          (1 + bumpStopCompression / Math.max(0.001, susp.bumpStopRange || 0.12));
        const compression = Math.max(0, h.compression);
        const springForce = susp.stiffness * compression * (1 + (susp.progressive || 0) * compression);
        const dampingRate = h.normalVelocity < 0
          ? (Number.isFinite(susp.dampingBump) ? susp.dampingBump : susp.damping)
          : (Number.isFinite(susp.dampingRebound) ? susp.dampingRebound : susp.damping);
        const damperForce = -dampingRate * h.normalVelocity;
        const load = Math.max(0, springForce + damperForce + bumpStopForce);
        normalLoad[h.index] = load;
        suspensionTelemetry[h.index] = {
          wheel: h.index, compression, rawLength: h.rawLength,
          bumpStopCompression, bumpStopForce, springForce, damperForce,
          normalLoad: load, velocity: h.normalVelocity
        };
        this._applyForce(V.mul(h.normal, load), h.arm);
      }

      /* 2. anti-roll bars. */
      for (const [left, right, rate] of [[0, 1, c.antiRoll.front], [2, 3, c.antiRoll.rear]]) {
        const a = contacts.find(h => h.index === left);
        const b = contacts.find(h => h.index === right);
        if (a && b && rate > 0) {
          const transfer = (a.compression - b.compression) * rate;
          const fa = Math.max(-normalLoad[left], transfer);
          const fb = Math.max(-normalLoad[right], -transfer);
          normalLoad[left] += fa;
          normalLoad[right] += fb;
          this._applyForce(V.mul(a.normal, fa), a.arm);
          this._applyForce(V.mul(b.normal, fb), b.arm);
          suspensionTelemetry[left].normalLoad = normalLoad[left];
          suspensionTelemetry[right].normalLoad = normalLoad[right];
        }
      }

      /* 3. aerodynamics, relative to the wind. */
      const forward = Q.rotate(s.orientation, { x: 0, y: 0, z: -1 });
      const right = Q.rotate(s.orientation, { x: 1, y: 0, z: 0 });
      const up = Q.rotate(s.orientation, { x: 0, y: 1, z: 0 });
      const wind = c.wind || { x: 0, y: 0, z: 0 };
      const air = {
        x: s.velocity.x - (wind.x || 0),
        y: s.velocity.y - (wind.y || 0),
        z: s.velocity.z - (wind.z || 0)
      };
      const airSpeed = V.len(air);
      const speed = V.len(s.velocity);
      const q = 0.5 * c.airDensity * airSpeed;
      const drag = q * c.dragArea;
      if (airSpeed > 1e-5) this._applyForce(V.mul(air, -drag), V.zero());
      // Crosswind: the lateral air component pushes the body downwind through a
      // centre of pressure behind the centre of gravity, which weathervanes the
      // car into the wind (the same self-aligning behaviour as a real car).
      const lateralAir = V.dot(air, right);
      const sideForce = -q * (c.sideArea || 0) * lateralAir;
      if (Math.abs(sideForce) > 1e-6) {
        this._applyForce(V.mul(right, sideForce), V.mul(forward, -(c.aeroYawArm || 0)));
      }
      const downforce = 0.5 * c.airDensity * c.downforceArea * airSpeed * airSpeed;
      if (downforce) this._applyForce(V.mul(up, -downforce), V.zero());

      // Airborne attitude control (legacy 1.x pitch/roll/yaw inputs). Torque is
      // proportional to the error between the requested rate and the current
      // body rate, so there is no unbounded spin-up. Ground contact disables it.
      if (!contacts.length && c.airControl) {
        const air = c.airControl;
        const avLocal = Q.rotateInverse(s.orientation, s.angularVelocity);
        const torqueLocal = {
          x: ((input.pitch || 0) * air.pitchRate - avLocal.x) * air.pitchGain,
          y: ((input.yaw || 0) * air.yawRate - avLocal.y) * air.yawGain,
          z: ((input.roll || 0) * air.rollRate - avLocal.z) * air.rollGain
        };
        this._applyTorque(Q.rotate(s.orientation, torqueLocal));
      }

      /* 4. drivetrain. */
      this._layout = this._layout || this._drivetrainLayout();
      const drivetrain = this._updateDrivetrain(dt, input, s.wheelOmega);
      const throttle = drivetrain.throttle;
      const brake = V.clamp(input.brake || 0, 0, 1);
      const steer = V.clamp(input.steer || 0, -1, 1);
      const handbrake = V.clamp(input.handbrake || 0, 0, 1);
      const longitudinalSpeed = drivetrain.longitudinalSpeed;

      /* 5. longitudinal load transfer estimate. */
      const driveForce = drivetrain.totalDriveTorque / Math.max(0.05, c.wheelRadius);
      const brakeForce = longitudinalSpeed === 0 ? 0 :
        brake * c.brakeTorque * 4 / Math.max(0.05, c.wheelRadius) * Math.sign(longitudinalSpeed);
      const requestedLongitudinalForce = driveForce - brakeForce;
      const requestedAcceleration = V.clamp(requestedLongitudinalForce / c.mass, -c.tire.mu * c.gravity, c.tire.mu * c.gravity);
      const requestedTransfer = c.mass * requestedAcceleration * Math.max(0, c.com.y) / Math.max(0.1, c.wheelbase);
      const frontContacts = contacts.filter(h => h.index < 2);
      const rearContacts = contacts.filter(h => h.index >= 2);
      const frontAvailable = frontContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const rearAvailable = rearContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const longitudinalTransfer = frontContacts.length && rearContacts.length
        ? V.clamp(requestedTransfer, -rearAvailable, frontAvailable) : 0;
      this._distributeLoad(frontContacts, -longitudinalTransfer, normalLoad, suspensionTelemetry);
      this._distributeLoad(rearContacts, longitudinalTransfer, normalLoad, suspensionTelemetry);

      /* 6. lateral load transfer estimate. */
      const steeringForTransfer = steer * c.steering.maxAngle *
        (1 - (1 - c.steering.highSpeedScale) * V.clamp(speed / (c.steering.speedReference || 55), 0, 1));
      const requestedLateralAcceleration = longitudinalSpeed * Math.abs(longitudinalSpeed) *
        Math.tan(steeringForTransfer) / Math.max(0.1, c.wheelbase);
      const axleRollCenter = axle => axle.length
        ? axle.reduce((sum, h) => sum + h.geometry.rollCenterHeight, 0) / axle.length : 0;
      const frontRollCenterHeight = axleRollCenter(frontContacts);
      const rearRollCenterHeight = axleRollCenter(rearContacts);
      const effectiveRollCenterHeight = (frontRollCenterHeight + rearRollCenterHeight) * 0.5;
      const lateralMomentArm = Math.max(0.05, c.com.y - effectiveRollCenterHeight);
      const requestedLateralTransfer = c.mass * requestedLateralAcceleration * lateralMomentArm / Math.max(0.1, c.track);
      const leftContacts = contacts.filter(h => h.index === 0 || h.index === 2);
      const rightContacts = contacts.filter(h => h.index === 1 || h.index === 3);
      const leftAvailable = leftContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const rightAvailable = rightContacts.reduce((sum, h) => sum + normalLoad[h.index], 0);
      const lateralTransfer = leftContacts.length && rightContacts.length
        ? V.clamp(requestedLateralTransfer, -leftAvailable, rightAvailable) : 0;
      this._distributeLoad(leftContacts, lateralTransfer, normalLoad, suspensionTelemetry);
      this._distributeLoad(rightContacts, -lateralTransfer, normalLoad, suspensionTelemetry);
      const loadTransferTelemetry = { longitudinal: -longitudinalTransfer, lateral: lateralTransfer };

      /* 7. drive torque distribution. */
      const wheelDriveTorque = this._distributeDriveTorque(drivetrain.totalDriveTorque);

      /* 8. combined-slip tire forces, ABS / TCS / ESC, wheel spin dynamics. */
      const steerAngle = steer * c.steering.maxAngle *
        (1 - (1 - c.steering.highSpeedScale) * V.clamp(speed / (c.steering.speedReference || 55), 0, 1));
      const targetYawRate = longitudinalSpeed * Math.tan(steerAngle) / Math.max(0.1, c.wheelbase);
      const yawError = s.angularVelocity.y - targetYawRate;
      const escEnabled = !!c.aids.esc && speed > 6;
      if (escEnabled && Math.abs(yawError) > c.aids.escYawThreshold) this.aidState.esc = true;

      for (const h of contacts) {
        const i = h.index;
        const rear = i >= 2;
        const wheelForward = h.tireForward || forward;
        const wheelRight = h.tireRight || right;
        const vLong = V.dot(h.velocity, wheelForward);
        const vLat = V.dot(h.velocity, wheelRight);
        const omega = s.wheelOmega[i];

        const rawSlip = (omega * c.wheelRadius - vLong) / Math.max(2, Math.abs(vLong));
        const relax = V.clamp(dt / Math.max(0.001, c.tire.relaxation), 0, 1);
        this._slipState[i] += (rawSlip - this._slipState[i]) * relax;
        const slipRatio = this._slipState[i];

        const rawSlipAngle = Math.atan2(vLat, Math.max(1, Math.abs(vLong)));
        const relaxationTime = Math.max(0.02, Math.min(
          Number(c.tire.relaxationAngle) || 0.08,
          (Number(c.tire.relaxationLength) || 0.35) / Math.max(3, Math.abs(vLong))
        ));
        const angleRelax = V.clamp(dt / relaxationTime, 0, 1);
        this._slipAngleState[i] += (rawSlipAngle - this._slipAngleState[i]) * angleRelax;
        let slipAngle = this._slipAngleState[i];
        slipAngle += h.geometry.camber * (c.tire.camberThrustGain ?? 0.08);

        let wheelDrive = wheelDriveTorque[i];
        if (c.aids.tcs && wheelDrive > 0 && slipRatio > c.aids.tcsSlip) {
          wheelDrive *= V.clamp(c.aids.tcsSlip / Math.max(0.001, slipRatio), 0.15, 1);
          this.aidState.tcs = true;
        }
        if (this.aidState.esc && wheelDrive > 0) wheelDrive *= 0.65;

        const loadSensitivityFactor = V.clamp(
          Math.pow(Math.max(1, normalLoad[i]) / Math.max(1, c.tire.referenceLoad || 3000), -(c.tire.loadSensitivity || 0)),
          0.55, 1.25
        );
        const frictionCoefficient = c.tire.mu * h.friction * loadSensitivityFactor;
        const driveAxleForce = wheelDrive / Math.max(0.05, c.wheelRadius);
        let tireForce;
        if (c.tire.combined && c.tire.combined.enabled) {
          tireForce = calculateCombinedTireForces({
            slipRatio: slipRatio + driveAxleForce / Math.max(1, c.tire.longitudinalStiffness),
            slipAngle,
            normalLoad: normalLoad[i],
            frictionCoefficient,
            tire: c.tire,
            camber: h.geometry.camber,
            wheelSurfaceSpeed: vLong
          });
        } else {
          const legacy = calculateTireForces(
            c.tire.longitudinalStiffness * slipRatio + wheelDrive / Math.max(0.05, c.wheelRadius),
            slipAngle, normalLoad[i], frictionCoefficient, c.tire
          );
          tireForce = { ...legacy, combinedSlip: Math.abs(slipRatio) + Math.abs(slipAngle), utilization: legacy.utilization };
        }

        let Fx = tireForce.Fx, Fy = tireForce.Fy;
        const cap = Math.max(0, frictionCoefficient * normalLoad[i]);
        let magnitude = Math.hypot(Fx, Fy);
        if (cap > 0 && magnitude > cap) {
          const scale = cap / magnitude;
          Fx *= scale; Fy *= scale; magnitude = cap;
        }

        const aligningMoment = -Fy * (c.tire.aligningTrail || 0) * V.clamp(1 - Math.abs(slipAngle) / 0.6, 0, 1);
        this._applyForce(V.add(V.mul(wheelForward, Fx), V.mul(wheelRight, Fy)), h.arm);
        if (aligningMoment) this._applyTorque(V.mul(h.normal, aligningMoment));

        let brakeTorque = brake * c.brakeTorque + (handbrake > 0 && rear ? handbrake * c.handbrakeTorque : 0);
        if (c.aids.abs && brakeTorque > 0 && Math.abs(slipRatio) > c.aids.absSlip) {
          brakeTorque *= 0.35;
          this.aidState.abs = true;
        }
        if (this.aidState.esc && h.index < 2 && Math.sign(yawError) !== 0) {
          // Brake the front wheel on the outside of the yaw error.
          const outside = yawError > 0 ? 0 : 1;
          if (h.index === outside) brakeTorque += c.aids.escBrakeTorque;
        }

        const wheelAccel = (wheelDrive - Fx * c.wheelRadius - Math.sign(omega || vLong) * brakeTorque) /
          (c.wheelMass * c.wheelRadius * c.wheelRadius);
        s.wheelOmega[i] += wheelAccel * dt;
        if (Math.abs(s.wheelOmega[i]) < 0.5 && brake > 0.8) s.wheelOmega[i] = 0;
        s.wheelSpin[i] = slipRatio;

        forces.push({
          wheel: i, Fx, Fy, FyRaw: tireForce.FyRaw, normalLoad: normalLoad[i],
          slipRatio, slipAngle, rawSlipAngle, aligningMoment, loadSensitivityFactor,
          frictionCoefficient, driveTorque: wheelDrive, brakeTorque,
          utilization: cap > 0 ? magnitude / cap : 0,
          combinedSlip: tireForce.combinedSlip, wheelOmega: s.wheelOmega[i]
        });
      }

      /* 9. chassis attitude stabilization and rigid-body integration. */
      this._applyBodyStabilization(input);
      this.world.fixedStep(dt);
      this._syncStateFromBody();

      /* 10. post-step support state + telemetry. */
      const after = this._contacts(input);
      s.grounded = after.length >= 2;
      this.tick++;
      const chassisContacts = this.world.manifolds.reduce((n, m) =>
        n + (m.bodyA === this.chassis || m.bodyB === this.chassis ? m.manifold.points.length : 0), 0);
      this.telemetry = {
        tick: this.tick,
        contacts: after.length,
        normalLoads: normalLoad,
        forces,
        suspension: suspensionTelemetry,
        wheelGeometry: after.map(h => ({
          wheel: h.index, axle: h.geometry.axle, camber: h.geometry.camber,
          camberDeg: h.geometry.camberDeg, toe: h.geometry.toe, toeDeg: h.geometry.toeDeg,
          rollCenterHeight: h.geometry.rollCenterHeight, travel: h.geometry.travel,
          hardpointValid: h.geometry.hardpointValid,
          instantCenter: h.geometry.instantCenter ? { ...h.geometry.instantCenter } : null,
          upperOuter: h.geometry.upperOuter ? { ...h.geometry.upperOuter } : null,
          lowerOuter: h.geometry.lowerOuter ? { ...h.geometry.lowerOuter } : null,
          wheelOffsetX: h.geometry.wheelOffsetX, casterDeg: h.geometry.casterDeg,
          kingpinInclinationDeg: h.geometry.kingpinInclinationDeg,
          steeringCamberDeg: h.geometry.steeringCamberDeg, scrubRadius: h.geometry.scrubRadius,
          mechanicalTrail: h.geometry.mechanicalTrail, jackingHeight: h.geometry.jackingHeight,
          steeringAxis: { ...h.geometry.steeringAxis }, wheelCenter: { ...h.wheelCenter },
          contactNormal: { ...h.normal }, tireForward: { ...h.tireForward }, tireRight: { ...h.tireRight }
        })),
        rollCenter: {
          front: frontRollCenterHeight, rear: rearRollCenterHeight,
          average: effectiveRollCenterHeight, lateralMomentArm
        },
        bodyTilt: this._bodyTilt(),
        bodyStabilization: { ...c.body },
        gear: this.gear, rpm: this.engineRpm,
        speed, downforce,
        airSpeed, crosswind: lateralAir, sideForce, wind: { x: wind.x || 0, y: wind.y || 0, z: wind.z || 0 },
        absActive: this.aidState.abs, tcsActive: this.aidState.tcs, escActive: this.aidState.esc,
        antiRoll: { front: c.antiRoll.front, rear: c.antiRoll.rear },
        loadTransfer: loadTransferTelemetry,
        differential: this._layout.diffType, driveLayout: this._layout.driveType,
        drivenWheels: [...this._layout.driven],
        drivetrain: {
          gear: this.gear, rpm: this.engineRpm, shifting: drivetrain.shifting,
          shiftTimer: this.shiftTimer, gearRatio: drivetrain.gearRatio,
          wheelTorque: drivetrain.totalDriveTorque, throttle,
          engineBraking: throttle < 0.02 && Math.abs(longitudinalSpeed) > 0.5
        },
        aids: { abs: this.aidState.abs, tcs: this.aidState.tcs, esc: this.aidState.esc },
        chassisContacts,
        collisionImpacts: this.world.stats.contacts
      };
    }

    advance(frameDelta, input = {}) {
      this.lastInput = input || {};
      const dt = Math.max(0, Math.min(0.25, Number(frameDelta) || 0));
      this.accumulator += dt;
      let steps = 0;
      while (this.accumulator + 1e-10 >= this.c.fixedDt && steps < this.c.maxSubsteps) {
        this._fixed(this.c.fixedDt, this.lastInput);
        this.accumulator -= this.c.fixedDt;
        steps++;
      }
      if (this.accumulator >= this.c.fixedDt) this.accumulator %= this.c.fixedDt;
      return { steps, alpha: V.clamp(this.accumulator / this.c.fixedDt, 0, 1) };
    }

    interpolatedState() {
      const a = this.previous, b = this.state;
      const t = V.clamp(this.accumulator / this.c.fixedDt, 0, 1);
      return {
        position: V.add(a.position, V.mul(V.sub(b.position, a.position), t)),
        velocity: V.add(a.velocity, V.mul(V.sub(b.velocity, a.velocity), t)),
        orientation: Q.slerp(a.orientation, b.orientation, t),
        grounded: b.grounded,
        wheelOmega: [...b.wheelOmega]
      };
    }

    getWheelState() {
      return {
        omega: [...this.state.wheelOmega],
        contacts: this.lastContacts.map(c => c.index),
        steer: (this.lastInput.steer || 0) * this.c.steering.maxAngle
      };
    }

    getTelemetry() { return this.telemetry; }

    getSnapshot() {
      return {
        state: clone(this.state),
        telemetry: clone(this.telemetry),
        config: clone(this.c),
        stats: { ...this.world.stats, time: this.world.time }
      };
    }
  }

  const api = {
    RigidBodyVehicle,
    Vector3: V,
    Quaternion: Q,
    calculateTireForces,
    calculateCombinedTireForces,
    magicFormula,
    solveHardpointKinematics,
    defaults: clone(DEFAULTS),
    core: Core,
    version: '3.0.0'
  };

  root.PlayCarRigidBody = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);

/* PlayCar unified runtime 3.0.0
 * Presents the legacy PlayCarPhysicsEngine API on top of the rigid-body
 * vehicle (raycast suspension + combined-slip tires + core collision solver).
 *
 * The legacy engine still owns weather, checkpoints, input normalization and
 * the simulation clock; the rigid-body vehicle owns motion and contacts.
 */
(function (root) {
  'use strict';

  const V = {
    add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
    sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
    mul: (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s }),
    dot: (a, b) => a.x * b.x + a.y * b.y + a.z * b.z,
    len: a => Math.hypot(a.x, a.y, a.z),
    clamp: (x, a, b) => Math.max(a, Math.min(b, x)),
    make: (x = 0, y = 0, z = 0) => ({ x, y, z })
  };
  const Q = {
    rotate: (q, v) => {
      const u = { x: q.x, y: q.y, z: q.z };
      const t = V.mul(V.cross(u, v), 2);
      return V.add(v, V.add(V.mul(t, q.w), V.cross(u, t)));
    },
    slerp: (a, b, t) => {
      let d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
      if (d < 0) { b = { x: -b.x, y: -b.y, z: -b.z, w: -b.w }; d = -d; }
      if (d > 0.9995) {
        const n = Math.hypot(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t) || 1;
        return { x: (a.x + (b.x - a.x) * t) / n, y: (a.y + (b.y - a.y) * t) / n, z: (a.z + (b.z - a.z) * t) / n, w: (a.w + (b.w - a.w) * t) / n };
      }
      const th = Math.acos(V.clamp(d, -1, 1)), s = Math.sin(th);
      const u = Math.sin((1 - t) * th) / s, v = Math.sin(t * th) / s;
      return { x: a.x * u + b.x * v, y: a.y * u + b.y * v, z: a.z * u + b.z * v, w: a.w * u + b.w * v };
    },
    inverse: q => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w })
  };

  const LegacyAPI = root.PlayCarPhysicsEngine;
  const RigidAPI = root.PlayCarRigidBody;
  if (!LegacyAPI || !LegacyAPI.PlayCarPhysicsEngine) {
    throw new Error('PlayCar unified runtime: playcar-physics-engine.js must be loaded first');
  }
  if (!RigidAPI || !RigidAPI.RigidBodyVehicle) {
    throw new Error('PlayCar unified runtime: rigidbody-vehicle-engine.js must be loaded first');
  }

  const LegacyPlayCarPhysicsEngine = LegacyAPI.PlayCarPhysicsEngine;
  const DEFAULT_BODY_OFFSET = 0.36;

  const UNIFIED_VERSION = '3.0.0';
const nowMs = () => (typeof performance !== 'undefined' && typeof performance.now === 'function')
  ? performance.now() : Date.now();

  class UnifiedPlayCarPhysicsEngine extends LegacyPlayCarPhysicsEngine {
    constructor(options = {}) {
      super(options);
      this.solver = 'rigid-body-' + UNIFIED_VERSION;
      this.bodyOffset = Number.isFinite(options.bodyOffset) ? options.bodyOffset : DEFAULT_BODY_OFFSET;
      this._colliderRef = null;
      this._legacyKey = '';

      const spawn = {
        x: this.state.position.x,
        y: this.state.position.y + this.bodyOffset,
        z: this.state.position.z
      };
      this.rigidBody = new RigidAPI.RigidBodyVehicle({
        surfaceMesh: this.surfaceMesh,
        colliderMeshes: this.colliderMeshes,
        groundPlaneY: this.groundPlaneY,
        position: spawn,
        orientation: this.state.orientation,
        config: this._buildRigidConfig(options.config || {})
      });
      this._colliderRef = this.colliderMeshes;
      this.rigidBody.state.orientation = { ...this.state.orientation };
      this.rigidBody.previous.orientation = { ...this.state.orientation };
      this._syncRigidToLegacy();
    }

    /* ---------------- legacy -> rigid configuration ---------------- */

    _buildRigidConfig(extra = {}) {
      const c = this.config;
      const mass = 1250;
      const wheelRadius = 0.37;
      const finalDrive = 3.9, firstGear = 3.2, efficiency = 0.88;
      const wetGrip = c.tireGrip * (1 - 0.42 * this.weather.wetness);
      const peakTorque = c.engineAcceleration * mass * wheelRadius / (firstGear * finalDrive * efficiency);
      const base = {
        fixedDt: c.fixedTimeStep,
        maxSubsteps: c.maxSubsteps,
        mass,
        gravity: c.gravity,
        wheelbase: c.wheelbase,
        track: c.trackHalfWidth * 2,
        wheelRadius,
        suspension: {
          // The 1.x game-unit default was 0.09, which maps to a car sitting on
          // its bump stops in metre units. Keep a sane minimum ride height while
          // preserving any larger, deliberately tuned value.
          rest: Math.max(0.22, c.suspensionRestLength),
          min: 0.08,
          max: Math.max(Math.max(0.22, c.suspensionRestLength) + 0.1, c.suspensionReach),
          stiffness: 32000,
          damping: 4200,
          dampingBump: 4200,
          dampingRebound: 5600,
          progressive: 0.35,
          bump: 90000
        },
        airControl: {
          yawRate: c.airYawRate, pitchRate: c.airPitchRate, rollRate: c.airRollRate,
          yawGain: 2200, pitchGain: 3000, rollGain: 3000
        },
        antiRoll: { front: 8500, rear: 6500 },
        differential: { type: 'lsd', layout: 'rwd', limitedSlip: 2.2 },
        aids: { abs: true, tcs: true, esc: false, absSlip: 0.18, tcsSlip: 0.22 },
        steering: { maxAngle: c.maxSteerAngle, highSpeedScale: c.highSpeedSteerScale },
        tire: {
          mu: wetGrip,
          corneringStiffness: 78000,
          longitudinalStiffness: 95000,
          relaxation: 0.04,
          relaxationAngle: 0.08,
          relaxationLength: 0.35,
          combined: { enabled: true, C: 1.35, E: 0.92, peak: 1.0 }
        },
        engine: {
          peakTorque,
          idleRpm: 900,
          redline: 6500,
          finalDrive,
          gears: [3.2, 2.1, 1.5, 1.18, 0.94, 0.78],
          efficiency,
          shiftTime: 0.16,
          engineBraking: 46
        },
        brakeTorque: c.brakeAcceleration * mass * wheelRadius / 4,
        handbrakeTorque: c.brakeAcceleration * mass * wheelRadius / 4 * 1.55,
        dragArea: Math.max(0.05, c.aerodynamicDrag * 300),
        airDensity: 1.225,
        downforceArea: c.maxDownforce * 2 / (1.225 * Math.max(1, c.maxSpeed * c.maxSpeed)),
        restitution: c.collisionRestitution,
        collisionFriction: c.collisionFriction,
        bodyHalf: { x: c.bodyHalfExtents.x, y: 0.32, z: Math.min(c.bodyHalfExtents.z, 1.5) },
        ccd: true,
        solver: { velocityIterations: 10, positionIterations: 4 }
      };
      return base;
    }

    _legacyKeyString() {
      const c = this.config;
      return [
        c.gravity, c.tireGrip, this.weather.wetness, this.weather.snow, this.weather.temperature, c.fixedTimeStep, c.maxSubsteps,
        c.wheelbase, c.trackHalfWidth, c.maxSteerAngle, c.highSpeedSteerScale,
        c.brakeAcceleration, c.engineAcceleration, c.aerodynamicDrag, c.maxDownforce,
        c.maxSpeed, c.suspensionRestLength, c.suspensionReach, c.collisionRestitution,
        c.collisionFriction, c.bodyHalfExtents.x, c.bodyHalfExtents.z
      ].join('|');
    }

    _applyLegacyConfig() {
      if (!this.rigidBody) return;
      const key = this._legacyKeyString();
      if (key === this._legacyKey) return;
      this._legacyKey = key;
      const c = this.config, r = this.rigidBody, rc = r.c;
      const mass = 1250, wheelRadius = r.c.wheelRadius;
      rc.tire.mu = c.tireGrip * this._weatherGripFactor();
      rc.gravity = c.gravity;
      rc.fixedDt = c.fixedTimeStep;
      rc.maxSubsteps = c.maxSubsteps;
      rc.wheelbase = c.wheelbase;
      rc.track = c.trackHalfWidth * 2;
      rc.steering.maxAngle = c.maxSteerAngle;
      rc.steering.highSpeedScale = c.highSpeedSteerScale;
      if (rc.airControl) {
        rc.airControl.yawRate = c.airYawRate;
        rc.airControl.pitchRate = c.airPitchRate;
        rc.airControl.rollRate = c.airRollRate;
      }
      rc.brakeTorque = c.brakeAcceleration * mass * wheelRadius / 4;
      rc.handbrakeTorque = rc.brakeTorque * 1.55;
      rc.engine.peakTorque = c.engineAcceleration * mass * wheelRadius / (3.2 * 3.9 * 0.88);
      rc.dragArea = Math.max(0.05, c.aerodynamicDrag * 300);
      rc.downforceArea = c.maxDownforce * 2 / (1.225 * Math.max(1, c.maxSpeed * c.maxSpeed));
      rc.suspension.rest = Math.max(0.22, c.suspensionRestLength);
      rc.suspension.max = Math.max(rc.suspension.rest + 0.1, c.suspensionReach);
      rc.restitution = c.collisionRestitution;
      rc.collisionFriction = c.collisionFriction;
      rc.bodyHalf = { x: c.bodyHalfExtents.x, y: 0.32, z: Math.min(c.bodyHalfExtents.z, 1.5) };
      if (r.chassis && r.chassis.shape.type === 'box') {
        r.chassis.shape.halfExtents = { ...rc.bodyHalf };
        r.chassis.shape.boundingRadius = V.len(rc.bodyHalf);
      }
      if (r.world) {
        r.world.settings.gravity = { x: 0, y: -c.gravity, z: 0 };
        r.world.settings.fixedDt = c.fixedTimeStep;
      }
    }

    /* ---------------- state synchronisation ---------------- */

    _gripState(rigid) {
      if (!rigid.grounded) return 'AIR';
      const t = rigid.telemetry || {};
      const handbrake = (this.lastInput.handbrake || 0) > 0;
      const slipping = (t.forces || []).some(f => Math.abs(f.slipRatio) > 0.22 || Math.abs(f.slipAngle) > 0.22);
      if (handbrake || (slipping && V.len(rigid.state.velocity) > 6)) return 'DRIFT';
      if (slipping) return 'SLIDE';
      return 'GRIP';
    }

    _syncRigidToLegacy() {
      const r = this.rigidBody, s = this.state, off = this.bodyOffset;
      const chassisImpacts = (r.world.events.beginContact || [])
        .filter(e => e.bodyA === r.chassis || e.bodyB === r.chassis).length;
      s.position = { x: r.state.position.x, y: r.state.position.y - off, z: r.state.position.z };
      s.velocity = { ...r.state.velocity };
      s.orientation = { ...r.state.orientation };
      s.angularVelocity = { ...r.state.angularVelocity };
      s.grounded = !!r.state.grounded;
      s.gripState = this._gripState({ ...r.state, state: r.state, telemetry: r.telemetry });
      s.tick = r.tick;
      s.collisionState = r.telemetry.chassisContacts > 0 ? 'wall' : (s.grounded ? 'track' : 'none');
      s.collisionCount = r.telemetry.chassisContacts || 0;
      s.collisionTimer = chassisImpacts > 0 ? 0.35 : Math.max(0, (s.collisionTimer || 0) - 1 / 120);
      s.totalCollisionEvents = (s.totalCollisionEvents || 0) + chassisImpacts;
      this.tick = r.tick;
      this.telemetry = {
        ...this.telemetry,
        speed: V.len(s.velocity),
        downforce: r.telemetry.downforce || 0,
        airborne: !s.grounded,
        wheelContacts: r.telemetry.contacts || 0
      };
    }

    /* ---------------- public API ---------------- */

    setSurfaceMesh(mesh) {
      super.setSurfaceMesh(mesh);
      if (this.rigidBody) this.rigidBody.setSurfaceMesh(mesh);
      return this;
    }

    setColliderMeshes(meshes = []) {
      super.setColliderMeshes(meshes);
      this._colliderRef = this.colliderMeshes;
      if (this.rigidBody) this.rigidBody.setColliderMeshes(this.colliderMeshes);
      return this;
    }

    addColliderMesh(mesh) {
      super.addColliderMesh(mesh);
      this._colliderRef = this.colliderMeshes;
      if (this.rigidBody) this.rigidBody.setColliderMeshes(this.colliderMeshes);
      return this;
    }

    setConfig(patch = {}) {
      const out = super.setConfig(patch);
      this._applyLegacyConfig();
      return out;
    }

    /* Combined grip multiplier for the current weather: surface water, snow
     * cover and tarmac temperature. Mirrors PlayCarWeather.gripFactor(). */
    _weatherGripFactor() {
      const w = this.weather || {};
      const wet = Number.isFinite(w.wetness) ? w.wetness : 0;
      const snow = Number.isFinite(w.snow) ? w.snow : 0;
      const temp = Number.isFinite(w.temperature) ? w.temperature : 20;
      const cold = 1 - Math.min(0.12, Math.max(0, 7 - temp) * 0.012);
      const heat = 1 - Math.min(0.06, Math.max(0, temp - 32) * 0.004);
      return (1 - 0.42 * wet) * (1 - 0.32 * snow) * cold * heat;
    }

    setWeather(weather = {}) {
      const out = super.setWeather(weather);
      this._applyLegacyConfig();
      if (this.rigidBody) {
        const r = this.rigidBody, w = this.weather;
        r.c.wind = { x: w.wind.x, y: 0, z: w.wind.z };
        r.c.airDensity = Number.isFinite(weather.airDensity)
          ? Math.max(0.8, Math.min(1.5, weather.airDensity))
          : 1.225 * (288.15 / (273.15 + w.temperature));
        r.c.tire.mu = this.config.tireGrip * this._weatherGripFactor();
        this._legacyKey = this._legacyKeyString();
      }
      return out;
    }

    setDifferential(type = 'lsd', layout = 'rwd', limitedSlip = 2.2) {
      if (!['open', 'lsd', 'locked'].includes(type) ||
          !['fwd', 'rwd', 'awd'].includes(layout) ||
          !Number.isFinite(limitedSlip) || limitedSlip < 1) {
        throw new TypeError('Invalid differential settings');
      }
      this.rigidBody.c.differential = { ...this.rigidBody.c.differential, type, layout, limitedSlip };
      this.rigidBody._layout = null;
      return { type, layout, limitedSlip };
    }

    setRigidConfig(patch = {}) {
      if (!patch || typeof patch !== 'object' || Array.isArray(patch)) {
        throw new TypeError('setRigidConfig expects an object');
      }
      const merge = (base, next) => {
        for (const [key, value] of Object.entries(next)) {
          if (value && typeof value === 'object' && !Array.isArray(value) && base[key] && typeof base[key] === 'object') {
            merge(base[key], value);
          } else {
            base[key] = value;
          }
        }
        return base;
      };
      merge(this.rigidBody.c, patch);
      this.rigidBody._layout = null;
      return this.rigidBody.c;
    }

    getRigidConfig() { return this.rigidBody.c; }

    addProp(options = {}) { return this.rigidBody.addProp(options); }

    applyImpulse(impulse, worldPoint = null) {
      if (!impulse || ![impulse.x, impulse.y, impulse.z].every(Number.isFinite)) return false;
      const point = worldPoint && [worldPoint.x, worldPoint.y, worldPoint.z].every(Number.isFinite)
        ? { x: worldPoint.x, y: worldPoint.y + this.bodyOffset, z: worldPoint.z }
        : this.rigidBody.chassis.position;
      this.rigidBody.chassis.applyImpulse({ x: impulse.x, y: impulse.y, z: impulse.z }, point);
      return true;
    }

    setVelocity(velocity) {
      if (!velocity || ![velocity.x, velocity.y, velocity.z].every(Number.isFinite)) return false;
      this.rigidBody.state.velocity = { ...velocity };
      this.rigidBody.chassis.linearVelocity = { ...velocity };
      this.state.velocity = { ...velocity };
      return true;
    }

    advance(frameDelta, input) {
      const started = nowMs();
      const normalized = input === undefined ? { ...this.lastInput } : this.setInput(input);
      this._applyLegacyConfig();
      if (this.rigidBody.mesh !== this.surfaceMesh) this.rigidBody.setSurfaceMesh(this.surfaceMesh);
      if (this._colliderRef !== this.colliderMeshes) {
        this._colliderRef = this.colliderMeshes;
        this.rigidBody.setColliderMeshes(this.colliderMeshes);
      }
      const result = this.rigidBody.advance(frameDelta, {
        throttle: normalized.throttle,
        brake: normalized.brake,
        steer: normalized.steer,
        handbrake: normalized.handbrake,
        pitch: normalized.pitch,
        roll: normalized.roll,
        yaw: normalized.yaw
      });
      this._syncRigidToLegacy();
      this.lastStepMs = nowMs() - started;
      this.totalStepMs = (this.totalStepMs || 0) + this.lastStepMs;
      return result;
    }

    interpolatedState() {
      const p = this.rigidBody.interpolatedState();
      return {
        ...p,
        position: { x: p.position.x, y: p.position.y - this.bodyOffset, z: p.position.z },
        gripState: this.state.gripState,
        collisionState: this.state.collisionState,
        checkpointId: this.state.checkpointId,
        tick: p.tick
      };
    }

    getTelemetry() {
      const base = super.getTelemetry();
      const t = this.rigidBody.telemetry || {};
      return {
        ...base,
        rpm: t.rpm || 0,
        gear: t.gear || 1,
        absActive: !!t.absActive,
        tcsActive: !!t.tcsActive,
        escActive: !!t.escActive,
        wheelSlip: (t.forces || []).map(f => f.slipRatio),
        tireLoads: [...(t.normalLoads || [])],
        tireUtilization: (t.forces || []).map(f => f.utilization || 0),
        suspension: t.suspension,
        drivetrain: t.drivetrain,
        loadTransfer: t.loadTransfer,
        chassisContacts: t.chassisContacts || 0,
        downforce: t.downforce || base.downforce || 0,
        wheelContacts: t.contacts || 0,
        drivenWheels: Array.isArray(t.drivenWheels) ? [...t.drivenWheels] : [],
        driveLayout: t.driveLayout || null,
        differential: t.differential || null,
        antiRoll: t.antiRoll,
        rollCenter: t.rollCenter,
        wheelGeometry: t.wheelGeometry,
        bodyTilt: t.bodyTilt
      };
    }

    getWheelState() {
      return {
        omega: [...this.rigidBody.state.wheelOmega],
        contacts: this.rigidBody.lastContacts.map(c => c.index),
        steer: (this.lastInput.steer || 0) * this.config.maxSteerAngle,
        slip: [...this.rigidBody.state.wheelSpin]
      };
    }

    getSnapshot() {
      const base = typeof super.getSnapshot === 'function'
        ? super.getSnapshot()
        : { state: this.getState(), telemetry: this.getTelemetry(), config: this.config, weather: this.weather, input: this.lastInput };
      return {
        ...base,
        version: UNIFIED_VERSION,
        solver: this.solver,
        state: this.getState(),
        telemetry: this.getTelemetry(),
        rigid: {
          solver: this.rigidBody.constructor.name,
          state: JSON.parse(JSON.stringify(this.rigidBody.state)),
          telemetry: JSON.parse(JSON.stringify(this.rigidBody.telemetry || {})),
          world: { ...this.rigidBody.world.stats }
        }
      };
    }

    debugSnapshot() {
      const base = super.debugSnapshot();
      return {
        ...base,
        solver: this.solver,
        rpm: this.rigidBody.engineRpm,
        gear: this.rigidBody.gear,
        wheelOmega: [...this.rigidBody.state.wheelOmega],
        contacts: this.rigidBody.world.stats.contacts,
        substeps: this.rigidBody.world.stepCount
      };
    }

    reset(position = { x: 0, y: 0, z: 0 }, yaw = 0, up = V.make(0, 1, 0)) {
      super.reset(position, yaw, up);
      if (this.rigidBody) {
        this.rigidBody.reset({
          x: position.x,
          y: position.y + this.bodyOffset,
          z: position.z
        });
        this.rigidBody.state.orientation = { ...this.state.orientation };
        this.rigidBody.previous.orientation = { ...this.state.orientation };
        this.rigidBody.chassis.orientation = { ...this.state.orientation };
        this.rigidBody._layout = null;
        this._syncRigidToLegacy();
      }
      return this;
    }
  }

  // Keep the legacy prototype path present for hosts that opt into the old
  // collision fallback (and for API introspection/documentation).
  UnifiedPlayCarPhysicsEngine.prototype._applyLegacyCollisions = function () {
    if (this.collisionPolicy === 'legacy') {
      LegacyPlayCarPhysicsEngine.prototype._applyStaticCollisions.call(this);
    }
  };

  LegacyAPI.PlayCarPhysicsEngine = UnifiedPlayCarPhysicsEngine;
  LegacyAPI.UnifiedPlayCarPhysicsEngine = UnifiedPlayCarPhysicsEngine;
  LegacyAPI.solverVersion = UNIFIED_VERSION;
  LegacyAPI.version = UNIFIED_VERSION;

  root.PlayCarPhysicsEngine = LegacyAPI;
  if (typeof module !== 'undefined' && module.exports) module.exports = LegacyAPI;
})(typeof globalThis !== 'undefined' ? globalThis : this);

/* PlayCar Weather System 1.0.0
 *
 * A deterministic, dependency-free weather model for the 3.0 physics stack.
 * It owns the environment state (surface wetness, rain/snow intensity, fog,
 * cloud cover, temperature, wind, lightning) and feeds the engine:
 *
 *   weather.setPreset('storm');            // smooth 6 s transition
 *   weather.update(dt);                    // wetness soaks / dries over time
 *   weather.applyTo(engine);               // engine.setWeather(...) + wind
 *
 * Physics coupling lives in the engine, not here:
 *   - wetness scales the tire friction coefficient (1 - 0.42 * wetness),
 *   - snow scales it further, cold/hot tarmac trims it slightly,
 *   - wind changes the relative airspeed for drag/downforce and adds a
 *     crosswind side force with a weathervaning yaw moment,
 *   - temperature changes air density and therefore drag/downforce.
 *
 * The model is pure data + timers: no DOM, no Three.js, no Math.random unless
 * you pass your own RNG. That keeps it testable in Node and reusable by both
 * the 3D lab and the proving ground.
 */
(function (root) {
  'use strict';

  const VERSION = '1.0.0';
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const clamp01 = x => clamp(x, 0, 1);
  const lerp = (a, b, t) => a + (b - a) * t;
  const smoothstep = t => t * t * (3 - 2 * t);

  function mulberry32(seed) {
    let a = seed >>> 0;
    return function random() {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  /* Presets. Angles are degrees the wind blows TOWARD, 0 = +Z, 90 = +X.
   * `sun` scales the directional light, `exposure` the tone mapping, `sky` the
   * background/fog colour. Wetness evolves toward the preset value at runtime. */
  const PRESETS = {
    clear: {
      label: '晴朗', wetness: 0, rain: 0, snow: 0, fog: 0.06, cloud: 0.08,
      temperature: 24, windSpeed: 1.5, windAngle: 210, lightning: 0,
      sky: 0x0c1a2a, sun: 1, exposure: 1.05
    },
    overcast: {
      label: '阴天', wetness: 0.18, rain: 0, snow: 0, fog: 0.2, cloud: 0.8,
      temperature: 17, windSpeed: 4, windAngle: 190, lightning: 0,
      sky: 0x1a2836, sun: 0.42, exposure: 0.98
    },
    rain: {
      label: '小雨', wetness: 0.72, rain: 0.55, snow: 0, fog: 0.34, cloud: 0.92,
      temperature: 14, windSpeed: 6, windAngle: 170, lightning: 0,
      sky: 0x16212c, sun: 0.28, exposure: 0.92
    },
    storm: {
      label: '暴雨', wetness: 1, rain: 1, snow: 0, fog: 0.55, cloud: 1,
      temperature: 12, windSpeed: 14, windAngle: 150, lightning: 18,
      sky: 0x0f1720, sun: 0.16, exposure: 0.84
    },
    fog: {
      label: '浓雾', wetness: 0.5, rain: 0.12, snow: 0, fog: 0.95, cloud: 0.9,
      temperature: 10, windSpeed: 2, windAngle: 240, lightning: 0,
      sky: 0x2a3540, sun: 0.24, exposure: 0.96
    },
    snow: {
      label: '降雪', wetness: 0.55, rain: 0, snow: 1, fog: 0.45, cloud: 0.95,
      temperature: -3, windSpeed: 5, windAngle: 200, lightning: 0,
      sky: 0x1b2635, sun: 0.32, exposure: 0.95
    },
    night: {
      label: '夜晚', wetness: 0.2, rain: 0, snow: 0, fog: 0.28, cloud: 0.35,
      temperature: 9, windSpeed: 3, windAngle: 200, lightning: 0,
      sky: 0x050a12, sun: 0.05, exposure: 0.9
    }
  };

  const NUMERIC = ['wetness', 'rain', 'snow', 'fog', 'cloud', 'temperature', 'windSpeed', 'windAngle', 'lightning', 'sun', 'exposure'];

  function presetSnapshot(name) {
    const preset = PRESETS[name];
    if (!preset) throw new Error('PlayCarWeather: unknown preset "' + name + '"');
    return { ...preset };
  }

  class WeatherSystem {
    constructor(options = {}) {
      this.presets = PRESETS;
      this.seed = Number.isFinite(options.seed) ? (options.seed >>> 0) : 0x5eed1;
      this.random = typeof options.random === 'function' ? options.random : mulberry32(this.seed);
      const start = PRESETS[options.preset] ? options.preset : 'clear';
      this.preset = start;
      this.target = presetSnapshot(start);
      this.state = {
        ...presetSnapshot(start),
        preset: start,
        transition: 1,
        flash: 0,
        strikes: 0,
        wind: { x: 0, z: 0 },
        time: 0
      };
      this._boltTimer = 0;
      this._recomputeWind();
    }

    getState() { return this.snapshot(); }

    snapshot() {
      return {
        ...this.state,
        wind: { ...this.state.wind }
      };
    }

    listPresets() {
      return Object.entries(PRESETS).map(([id, p]) => ({ id, label: p.label }));
    }

    /* Blend to a preset. duration = 0 / immediate applies instantly. */
    setPreset(name, options = {}) {
      if (!PRESETS[name]) throw new Error('PlayCarWeather: unknown preset "' + name + '"');
      const duration = Number.isFinite(options.duration) ? Math.max(0, options.duration) : 6;
      this.preset = name;
      this.state.preset = name;
      this.target = presetSnapshot(name);
      if (duration === 0 || options.immediate) {
        for (const key of NUMERIC) this.state[key] = this.target[key];
        this.state.sky = this.target.sky;
        this.state.transition = 1;
      } else {
        this._from = {};
        for (const key of NUMERIC) this._from[key] = this.state[key];
        this._from.sky = this.state.sky;
        this._duration = duration;
        this._elapsed = 0;
        this.state.transition = 0;
      }
      this._recomputeWind();
      return this.snapshot();
    }

    /* Manual override (sliders, scripts, tests). Unknown keys are ignored. */
    set(patch = {}) {
      for (const key of NUMERIC) {
        if (Number.isFinite(patch[key])) this.state[key] = patch[key];
      }
      if (Number.isFinite(patch.sky)) {
        this.state.sky = patch.sky;
        this.target.sky = patch.sky;
        this._from = null;
      }
      if (Number.isFinite(patch.windSpeed) || Number.isFinite(patch.windAngle)) this._recomputeWind();
      this.state.preset = patch.preset || this.preset;
      return this.snapshot();
    }

    update(dt) {
      const step = clamp(Number(dt) || 0, 0, 0.25);
      if (!step) return this.snapshot();
      this.state.time += step;

      /* 1. preset transition: everything except wetness, which soaks/dries. */
      if (this._from) {
        this._elapsed += step;
        const raw = this._duration > 0 ? clamp01(this._elapsed / this._duration) : 1;
        const t = smoothstep(raw);
        for (const key of NUMERIC) this.state[key] = lerp(this._from[key], this.target[key], t);
        this.state.sky = mixHex(this._from.sky, this.target.sky, t);
        this.state.transition = raw;
        if (raw >= 1) {
          this._from = null;
          this.state.transition = 1;
        }
      } else {
        // No transition: keep rain/snow/fog/wind/temperature glued to the preset
        // target, but leave wetness to the surface model below.
        for (const key of NUMERIC) {
          if (key === 'wetness') continue;
          this.state[key] = this.target[key];
        }
        this.state.sky = this.target.sky;
        this.state.transition = 1;
      }

      /* 2. surface water: rain wets, dry weather evaporates. */
      const targetWetness = this._from ? this.state.wetness : this.target.wetness;
      const wetRate = 0.35 + 0.45 * this.state.rain + 0.1 * this.state.snow;
      const dryRate = 0.045 + 0.02 * clamp01(this.state.temperature / 30) + 0.01 * this.state.windSpeed;
      if (targetWetness > this.state.wetness) {
        this.state.wetness = Math.min(targetWetness, this.state.wetness + wetRate * step);
      } else {
        this.state.wetness = Math.max(targetWetness, this.state.wetness - dryRate * step);
      }
      this.state.wetness = clamp01(this.state.wetness);

      /* 3. wind vector (kept in m/s, x/z components). */
      this._recomputeWind();

      /* 4. deterministic lightning + flash decay. */
      this.state.flash *= Math.exp(-step * 7);
      if (this.state.lightning > 0) {
        this._boltTimer -= step;
        if (this._boltTimer <= 0) {
          this.state.flash = 1;
          this.state.strikes += 1;
          const rate = Math.max(0.01, this.state.lightning / 60); // strikes per second
          this._boltTimer = -Math.log(1 - this.random() * 0.999) / rate;
        }
      } else {
        this._boltTimer = 0;
      }
      if (this.state.flash < 1e-4) this.state.flash = 0;
      return this.snapshot();
    }

    _recomputeWind() {
      const angle = (this.state.windAngle * Math.PI) / 180;
      const speed = Math.max(0, this.state.windSpeed);
      this.state.wind = { x: Math.sin(angle) * speed, z: Math.cos(angle) * speed };
    }

    windVector() { return { ...this.state.wind }; }

    /* Effective tire grip multiplier for the current surface. Mirrors the
     * engine mapping so tests can compare model and engine directly. */
    gripFactor(baseGrip = 1) {
      const cold = 1 - Math.min(0.12, Math.max(0, 7 - this.state.temperature) * 0.012);
      const heat = 1 - Math.min(0.06, Math.max(0, this.state.temperature - 32) * 0.004);
      return baseGrip * (1 - 0.42 * this.state.wetness) * (1 - 0.32 * this.state.snow) * cold * heat;
    }

    /* Air density at the current air temperature (sea level, ISA). */
    airDensity() { return 1.225 * (288.15 / (273.15 + clamp(this.state.temperature, -40, 60))); }

    /* Push the state into a unified 3.0 engine. Returns the applied weather. */
    applyTo(engine) {
      if (!engine || typeof engine.setWeather !== 'function') return this.snapshot();
      const state = this.state;
      engine.setWeather({
        wetness: state.wetness,
        wind: { ...state.wind },
        rain: state.rain,
        snow: state.snow,
        fog: state.fog,
        temperature: state.temperature,
        airDensity: this.airDensity()
      });
      return this.snapshot();
    }
  }

  function mixHex(a, b, t) {
    const ar = (a >> 16) & 255, ag = (a >> 8) & 255, ab = a & 255;
    const br = (b >> 16) & 255, bg = (b >> 8) & 255, bb = b & 255;
    return (Math.round(lerp(ar, br, t)) << 16) | (Math.round(lerp(ag, bg, t)) << 8) | Math.round(lerp(ab, bb, t));
  }

  const api = {
    WeatherSystem,
    PRESETS,
    version: VERSION,
    create(options) { return new WeatherSystem(options); }
  };

  root.PlayCarWeather = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);

/* PlayCar Weather Visuals 1.0.0
 *
 * Three.js presentation layer for physics/weather-system.js. It does not own
 * any physics: pass it the weather state each frame and it renders:
 *
 *   - rain / snow particles that follow the camera and slant with the wind,
 *   - sky and fog colour/density, sun + hemisphere dimming, exposure,
 *   - wet-surface roughness/metalness on the materials you hand it,
 *   - deterministic lightning flashes (ambient light + sky flash).
 *
 * Usage:
 *   const visuals = createPlayCarWeatherVisuals(THREE, scene, {
 *     camera, sun, hemi, renderer, materials: [roadMaterial]
 *   });
 *   // each frame:
 *   visuals.update(dt, weather.getState(), camera);
 *
 * The module is a classic script (no imports) so it can be concatenated into
 * the single-file bundle and also loaded with a plain <script> tag by the
 * standalone proving-ground page. In Node/headless it still builds real
 * three.js objects; only WebGL rendering is stubbed by the smoke harness.
 */
(function (root) {
  'use strict';

  const VERSION = '1.0.0';
  const clamp01 = x => Math.max(0, Math.min(1, Number(x) || 0));
  const lerp = (a, b, t) => a + (b - a) * t;

  function mulberry32(seed) {
    let a = seed >>> 0;
    return function random() {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function createPlayCarWeatherVisuals(THREE, scene, options = {}) {
    if (!THREE || !scene) throw new Error('createPlayCarWeatherVisuals: THREE and scene are required');
    const camera = options.camera || null;
    const renderer = options.renderer || null;
    const sun = options.sun || null;
    const hemi = options.hemi || null;
    const materials = Array.isArray(options.materials) ? options.materials.filter(Boolean) : [];
    const seed = Number.isFinite(options.seed) ? options.seed : 0x9e3779b9;
    const random = mulberry32(seed);

    const baseSun = sun ? sun.intensity : 1;
    const baseHemi = hemi ? hemi.intensity : 1;
    const baseExposure = renderer && Number.isFinite(renderer.toneMappingExposure) ? renderer.toneMappingExposure : 1;
    const baseMaterials = materials.map(material => ({
      material,
      roughness: Number.isFinite(material.roughness) ? material.roughness : 0.9,
      metalness: Number.isFinite(material.metalness) ? material.metalness : 0,
      color: material.color && material.color.isColor ? material.color.clone() : null
    }));

    // Keep a private fog so we never overwrite a scene that owns its own.
    let fog = scene.fog;
    if (!fog && THREE.Fog) {
      fog = new THREE.Fog(0x0c1a2a, 160, 420);
      scene.fog = fog;
    }
    const fogBase = fog ? { near: fog.near, far: fog.far } : { near: 160, far: 420 };
    if (!scene.background || !scene.background.isColor) {
      scene.background = new THREE.Color(0x0c1a2a);
    }
    const skyBase = scene.background.clone();

    const flashLight = new THREE.AmbientLight(0xd7e6ff, 0);
    flashLight.name = 'PlayCarWeatherFlash';
    scene.add(flashLight);

    const RAIN_COUNT = options.rainCount || 1200;
    const SNOW_COUNT = options.snowCount || 700;
    const BOX = { x: 70, y: 38, z: 70 };

    function makeParticles(count, color, size, opacity) {
      const positions = new Float32Array(count * 3);
      for (let i = 0; i < count; i++) {
        positions[i * 3] = (random() - 0.5) * BOX.x;
        positions[i * 3 + 1] = random() * BOX.y;
        positions[i * 3 + 2] = (random() - 0.5) * BOX.z;
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      const material = new THREE.PointsMaterial({
        color, size, sizeAttenuation: true, transparent: true, opacity,
        depthWrite: false, fog: false
      });
      const points = new THREE.Points(geometry, material);
      points.frustumCulled = false;
      points.visible = false;
      points.name = 'PlayCarWeatherParticles';
      scene.add(points);
      return { points, positions, count, material };
    }

    const rain = makeParticles(RAIN_COUNT, 0xa9d4ee, 0.085, 0.55);
    const snow = makeParticles(SNOW_COUNT, 0xffffff, 0.17, 0.85);

    const target = { wet: 0, rain: 0, snow: 0, fog: 0, sun: 1, flash: 0, sky: skyBase.getHex() };
    const current = { wet: 0, rain: 0, snow: 0, fog: 0, sun: 1, flash: 0, sky: skyBase.getHex() };
    const wind = { x: 0, z: 0 };
    const tmpColor = new THREE.Color();
    let disposed = false;

    function normalize(state = {}) {
      const wet = clamp01(state.wetness);
      const rainAmount = state.rain === undefined ? wet : clamp01(state.rain);
      const snowAmount = clamp01(state.snow);
      const fogAmount = state.fog === undefined ? clamp01(wet * 0.6) : clamp01(state.fog);
      const sunScale = state.sun === undefined ? 1 : Math.max(0, Math.min(2, state.sun));
      return {
        wet, rain: rainAmount, snow: snowAmount, fog: fogAmount,
        sun: sunScale,
        flash: clamp01(state.flash),
        sky: Number.isFinite(state.sky) ? state.sky : null,
        wind: state.wind || { x: 0, z: 0 }
      };
    }

    function stepParticles(system, dt, fallSpeed, driftScale) {
      const { positions, count } = system;
      const cx = camera ? camera.position.x : (options.center ? options.center.x : 0);
      const cy = camera ? camera.position.y : (options.center ? options.center.y : 0);
      const cz = camera ? camera.position.z : (options.center ? options.center.z : 0);
      const halfX = BOX.x / 2, halfZ = BOX.z / 2;
      for (let i = 0; i < count; i++) {
        const ix = i * 3;
        positions[ix] += wind.x * driftScale * dt;
        positions[ix + 1] -= fallSpeed * dt;
        positions[ix + 2] += wind.z * driftScale * dt;
        // Wrap around the camera so the field never empties.
        if (positions[ix + 1] < -BOX.y * 0.5) {
          positions[ix + 1] += BOX.y;
          positions[ix] = cx + (random() - 0.5) * BOX.x;
          positions[ix + 2] = cz + (random() - 0.5) * BOX.z;
        }
        if (positions[ix] - cx > halfX) positions[ix] -= BOX.x;
        else if (positions[ix] - cx < -halfX) positions[ix] += BOX.x;
        if (positions[ix + 2] - cz > halfZ) positions[ix + 2] -= BOX.z;
        else if (positions[ix + 2] - cz < -halfZ) positions[ix + 2] += BOX.z;
        if (positions[ix + 1] > cy + BOX.y * 0.6) positions[ix + 1] -= BOX.y;
      }
      system.points.geometry.attributes.position.needsUpdate = true;
    }

    function update(dt, state) {
      if (disposed) return;
      const step = Math.max(0, Math.min(0.25, Number(dt) || 0));
      const input = normalize(state || {});
      const k = Math.min(1, step * 2.2);
      target.wet = input.wet; target.rain = input.rain; target.snow = input.snow;
      target.fog = input.fog; target.sun = input.sun; target.flash = input.flash;
      if (input.sky !== null) target.sky = input.sky;
      wind.x = input.wind.x || 0;
      wind.z = input.wind.z || 0;
      for (const key of ['wet', 'rain', 'snow', 'fog', 'sun', 'flash']) current[key] += (target[key] - current[key]) * k;
      current.sky = mixHex(current.sky, target.sky, k);

      // Sky and fog.
      skyBase.setHex(current.sky);
      if (scene.background && scene.background.isColor) {
        const flashBoost = current.flash * 0.55;
        scene.background.copy(skyBase).lerp(tmpColor.setHex(0xdfe9ff), flashBoost);
      }
      if (fog && fog.color) {
        fog.color.setHex(current.sky);
        fog.near = lerp(fogBase.near, 14, current.fog);
        fog.far = lerp(fogBase.far, 85, current.fog);
      }

      // Lights: dim with cloud cover/sun scale, flash with lightning.
      if (sun) sun.intensity = baseSun * current.sun + current.flash * 1.4;
      if (hemi) hemi.intensity = baseHemi * (0.35 + 0.65 * current.sun) + current.flash * 2.2;
      flashLight.intensity = current.flash * 3.2;
      if (renderer && Number.isFinite(renderer.toneMappingExposure)) {
        renderer.toneMappingExposure = baseExposure * lerp(1, 0.92, current.wet) * (1 + current.flash * 0.25);
      }

      // Wet tarmac.
      for (const entry of baseMaterials) {
        const material = entry.material;
        if (Number.isFinite(material.roughness)) {
          material.roughness = lerp(entry.roughness, 0.18, current.wet);
        }
        if (Number.isFinite(material.metalness)) {
          material.metalness = lerp(entry.metalness, 0.32, current.wet);
        }
        if (entry.color && material.color) {
          material.color.copy(entry.color).multiplyScalar(1 - 0.12 * current.wet);
        }
      }

      // Particles (they follow the camera captured at construction time).
      rain.points.visible = current.rain > 0.02;
      rain.material.opacity = 0.62 * current.rain;
      snow.points.visible = current.snow > 0.02;
      snow.material.opacity = 0.9 * current.snow;
      if (camera && rain.points.visible) stepParticles(rain, step, 22, 0.85);
      if (camera && snow.points.visible) stepParticles(snow, step, 1.9, 1.25);
      return snapshot();
    }

    function mixHex(a, b, t) {
      const ar = (a >> 16) & 255, ag = (a >> 8) & 255, ab = a & 255;
      const br = (b >> 16) & 255, bg = (b >> 8) & 255, bb = b & 255;
      return (Math.round(lerp(ar, br, t)) << 16) | (Math.round(lerp(ag, bg, t)) << 8) | Math.round(lerp(ab, bb, t));
    }

    function snapshot() {
      return {
        wetness: current.wet, rain: current.rain, snow: current.snow, fog: current.fog,
        sun: current.sun, flash: current.flash, sky: current.sky, wind: { ...wind }
      };
    }

    function dispose() {
      if (disposed) return;
      disposed = true;
      scene.remove(flashLight);
      for (const system of [rain, snow]) {
        scene.remove(system.points);
        system.points.geometry.dispose();
        system.material.dispose();
      }
      for (const entry of baseMaterials) {
        if (Number.isFinite(entry.roughness)) entry.material.roughness = entry.roughness;
        if (Number.isFinite(entry.metalness)) entry.material.metalness = entry.metalness;
        if (entry.color && entry.material.color) entry.material.color.copy(entry.color);
      }
      if (sun) sun.intensity = baseSun;
      if (hemi) hemi.intensity = baseHemi;
      if (renderer && Number.isFinite(renderer.toneMappingExposure)) renderer.toneMappingExposure = baseExposure;
    }

    return {
      version: VERSION,
      update,
      dispose,
      snapshot,
      setCamera(value) { camera = value; },
      rain, snow, flashLight,
      get state() { return snapshot(); }
    };
  }

  root.createPlayCarWeatherVisuals = createPlayCarWeatherVisuals;
  root.PlayCarWeatherVisuals = { create: createPlayCarWeatherVisuals, version: VERSION };
  if (typeof module !== 'undefined' && module.exports) module.exports = { createPlayCarWeatherVisuals, version: VERSION };
})(typeof globalThis !== 'undefined' ? globalThis : this);

(async function startPlayCar3DScene(){
  let THREE;
  const sources = [
    './vendor/three.module.min.js',
    'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.min.js',
    'https://unpkg.com/three@0.180.0/build/three.module.min.js'
  ];
  let lastError;
  for (const source of sources) {
    try { THREE = await import(source); break; }
    catch (error) { lastError = error; }
  }
  if (!THREE) {
    const status = document.getElementById('scene-status');
    if (status) status.textContent = '3D 模块加载失败：本地 vendor/ 与两个 Three.js CDN 均无法加载。请检查网络或稍后重试。';
    console.error('PlayCar Three.js module load failed:', lastError);
    return;
  }
  try {
    const EngineAPI = window.PlayCarPhysicsEngine;
    const status = document.getElementById('scene-status');
    const host = document.getElementById('scene3d');
    if (!EngineAPI || !host) {
      if (status) status.textContent = '3D 场景初始化失败：缺少物理引擎或容器';
      throw new Error('PlayCar 3D scene prerequisites missing');
    }

    const { PlayCarPhysicsEngine, TriangleMesh, Vector3: V, Quaternion: Q } = EngineAPI;

    host.style.position = 'relative';
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x8fb6cf);
    scene.fog = new THREE.Fog(0x8fb6cf, 130, 340);
    const camera = new THREE.PerspectiveCamera(58, 1, 0.1, 600);
    camera.position.set(0, 8, 46);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 1.75));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.08;
    host.appendChild(renderer.domElement);
    renderer.domElement.setAttribute('aria-label', '可交互三维赛车物理模拟场景');
    renderer.domElement.style.touchAction = 'none';

    const hemiLight = new THREE.HemisphereLight(0xdceeff, 0x344455, 2.0);
    scene.add(hemiLight);
    const sun = new THREE.DirectionalLight(0xfff1d7, 3.1);
    sun.position.set(-55, 80, 35);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -110; sun.shadow.camera.right = 110;
    sun.shadow.camera.top = 110; sun.shadow.camera.bottom = -110;
    sun.shadow.camera.far = 300;
    scene.add(sun);
    const fill = new THREE.DirectionalLight(0x75cfff, 0.7);
    fill.position.set(45, 22, -60);
    scene.add(fill);

    const mats = {
      road: new THREE.MeshStandardMaterial({ color: 0xffffff, vertexColors: true, roughness: 0.9 }),
      ground: new THREE.MeshStandardMaterial({ color: 0x4e7350, roughness: 1 }),
      wall: new THREE.MeshStandardMaterial({ color: 0xdfe9ef, roughness: 0.6 }),
      wallTop: new THREE.MeshStandardMaterial({ color: 0xff713d, roughness: 0.45 }),
      line: new THREE.MeshStandardMaterial({ color: 0xe8f1f5, roughness: 0.7 }),
      cone: new THREE.MeshStandardMaterial({ color: 0xff7b2e, roughness: 0.55 }),
      crate: new THREE.MeshStandardMaterial({ color: 0xb08a53, roughness: 0.8 }),
      orange: new THREE.MeshStandardMaterial({ color: 0xff713d, roughness: 0.34, metalness: 0.25 }),
      glass: new THREE.MeshStandardMaterial({ color: 0x173a4f, roughness: 0.16, metalness: 0.4 }),
      tire: new THREE.MeshStandardMaterial({ color: 0x111820, roughness: 0.92 }),
      rim: new THREE.MeshStandardMaterial({ color: 0xa8bdcc, metalness: 0.85, roughness: 0.22 }),
      skid: new THREE.MeshBasicMaterial({ color: 0x1a2228, transparent: true, opacity: 0.42, depthWrite: false })
    };

    const world = new THREE.Group();
    scene.add(world);

    /* ------------------------------------------------------------------ */
    /* Track                                                              */
    /* ------------------------------------------------------------------ */

    const physicsTriangles = [];
    const colliderTriangles = [];
    const groundTriangles = [];
    const ROAD_HALF_WIDTH = 7.2;

    function addTriangle(target, a, b, c, friction) {
      target.push(friction === undefined ? [a, b, c] : [a, b, c, friction]);
    }
    function addQuad(target, a, b, c, d, friction) {
      addTriangle(target, a, b, c, friction);
      addTriangle(target, a, c, d, friction);
    }
    function meshFromTriangles(tris, material) {
      const positions = [];
      for (const t of tris) for (const p of t) positions.push(p.x, p.y, p.z);
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      g.computeVertexNormals();
      const m = new THREE.Mesh(g, material);
      m.receiveShadow = true;
      m.castShadow = true;
      world.add(m);
      return m;
    }

    // Closed Catmull-Rom circuit. The start straight includes z = 34 so the
    // documented spawn (0, 0.08, 34) sits exactly on the racing surface.
    const CONTROL = [
      [0, 60], [0, 20], [0, -20], [0, -62], [16, -84], [44, -88], [66, -72],
      [74, -44], [70, -12], [74, 22], [60, 52], [34, 66], [12, 66]
    ];
    function catmull(p0, p1, p2, p3, t) {
      const t2 = t * t, t3 = t2 * t;
      const f = (a, b, c, d) => 0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3);
      return { x: f(p0[0], p1[0], p2[0], p3[0]), z: f(p0[1], p1[1], p2[1], p3[1]) };
    }

    const centerPoints = [];
    const SUB = 8;
    for (let i = 0; i < CONTROL.length; i++) {
      const p0 = CONTROL[(i - 1 + CONTROL.length) % CONTROL.length];
      const p1 = CONTROL[i];
      const p2 = CONTROL[(i + 1) % CONTROL.length];
      const p3 = CONTROL[(i + 2) % CONTROL.length];
      for (let s = 0; s < SUB; s++) centerPoints.push(catmull(p0, p1, p2, p3, s / SUB));
    }
    const N = centerPoints.length;
    const track = [];
    for (let i = 0; i < N; i++) {
      const prev = centerPoints[(i - 1 + N) % N];
      const cur = centerPoints[i];
      const next = centerPoints[(i + 1) % N];
      const tPrev = V.norm({ x: cur.x - prev.x, y: 0, z: cur.z - prev.z });
      const tNext = V.norm({ x: next.x - cur.x, y: 0, z: next.z - cur.z });
      const tangent = V.norm(V.add(tPrev, tNext));
      const curvature = tPrev.z * tNext.x - tPrev.x * tNext.z;
      const bank = V.clamp(-curvature * 7.5, -0.34, 0.34);
      const up = Q.rotate(Q.axis(tangent, bank), { x: 0, y: 1, z: 0 });
      const right = V.norm(V.cross(tangent, up));
      track.push({ center: { x: cur.x, y: 0, z: cur.z }, tangent, right, up, bank, curvature, u: i / N });
    }

    // Smooth and clamp the banking. The Catmull-Rom seam and the tightest corner
    // can otherwise produce ~20-degree cross-slopes that throw the car off the
    // road. A short moving average keeps the circuit banked without wall-like kerbs.
    {
      const raw = track.map(f => f.bank);
      for (let i = 0; i < N; i++) {
        let sum = 0;
        for (let k = -4; k <= 4; k++) sum += raw[(i + k + N) % N];
        const bank = V.clamp(sum / 9, -0.16, 0.16);
        const f = track[i];
        f.bank = bank;
        f.up = Q.rotate(Q.axis(f.tangent, bank), { x: 0, y: 1, z: 0 });
        f.right = V.norm(V.cross(f.tangent, f.up));
      }
    }

    const vertexPositions = [];
    const vertexColors = [];
    const indices = [];
    const wetStart = Math.floor(N * 0.55), wetEnd = Math.floor(N * 0.64);
    const dryColor = new THREE.Color(0x3a4b58), wetColor = new THREE.Color(0x27455c);
    for (let i = 0; i < N; i++) {
      const f = track[i];
      const wet = i >= wetStart && i < wetEnd;
      const L = V.add(f.center, V.mul(f.right, -ROAD_HALF_WIDTH));
      const R = V.add(f.center, V.mul(f.right, ROAD_HALF_WIDTH));
      vertexPositions.push(L.x, L.y, L.z, R.x, R.y, R.z);
      const color = wet ? wetColor : dryColor;
      vertexColors.push(color.r, color.g, color.b, color.r, color.g, color.b);
      const j = (i + 1) % N;
      indices.push(i * 2, i * 2 + 1, j * 2 + 1, i * 2, j * 2 + 1, j * 2);
    }
    const roadGeometry = new THREE.BufferGeometry();
    roadGeometry.setAttribute('position', new THREE.Float32BufferAttribute(vertexPositions, 3));
    roadGeometry.setAttribute('color', new THREE.Float32BufferAttribute(vertexColors, 3));
    roadGeometry.setIndex(indices);
    roadGeometry.computeVertexNormals();
    const roadMesh = new THREE.Mesh(roadGeometry, mats.road);
    roadMesh.receiveShadow = true;
    roadMesh.castShadow = true;
    world.add(roadMesh);

    for (let i = 0; i < N; i++) {
      const j = (i + 1) % N;
      const f = track[i], g = track[j];
      const wet = i >= wetStart && i < wetEnd;
      const friction = wet ? 0.55 : 1.0;
      const Li = V.add(f.center, V.mul(f.right, -ROAD_HALF_WIDTH));
      const Ri = V.add(f.center, V.mul(f.right, ROAD_HALF_WIDTH));
      const Lj = V.add(g.center, V.mul(g.right, -ROAD_HALF_WIDTH));
      const Rj = V.add(g.center, V.mul(g.right, ROAD_HALF_WIDTH));
      addTriangle(physicsTriangles, Li, Ri, Rj, friction);
      addTriangle(physicsTriangles, Li, Rj, Lj, friction);

      // Continuous low barriers on both sides, as visual ribbons plus colliders.
      for (const side of [-1, 1]) {
        const ei = V.add(f.center, V.mul(f.right, side * (ROAD_HALF_WIDTH + 0.12)));
        const ej = V.add(g.center, V.mul(g.right, side * (ROAD_HALF_WIDTH + 0.12)));
        const hi = { x: ei.x, y: 1.15, z: ei.z };
        const hj = { x: ej.x, y: 1.15, z: ej.z };
        addQuad(colliderTriangles, ei, ej, hj, hi, 1);
      }
    }

    // Barrier ribbons (one geometry per side).
    for (const side of [-1, 1]) {
      const pos = [], idx = [];
      for (let i = 0; i < N; i++) {
        const f = track[i];
        const e = V.add(f.center, V.mul(f.right, side * (ROAD_HALF_WIDTH + 0.22)));
        pos.push(e.x, 0, e.z, e.x, 1.15, e.z);
        const j = (i + 1) % N;
        idx.push(i * 2, j * 2, j * 2 + 1, i * 2, j * 2 + 1, i * 2 + 1);
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      g.setIndex(idx);
      g.computeVertexNormals();
      const wall = new THREE.Mesh(g, mats.wall);
      wall.material.side = THREE.DoubleSide;
      wall.castShadow = true;
      wall.receiveShadow = true;
      world.add(wall);
      const topPoints = [];
      for (let i = 0; i <= N; i++) {
        const f = track[i % N];
        const e = V.add(f.center, V.mul(f.right, side * (ROAD_HALF_WIDTH + 0.22)));
        topPoints.push(new THREE.Vector3(e.x, 1.17, e.z));
      }
      const topGeo = new THREE.BufferGeometry().setFromPoints(topPoints);
      world.add(new THREE.Line(topGeo, new THREE.LineBasicMaterial({ color: 0xff713d })));
    }

    // Ground: visual lawn plus a solid fallback support plane.
    const groundGeo = new THREE.PlaneGeometry(360, 360, 1, 1);
    groundGeo.rotateX(-Math.PI / 2);
    const ground = new THREE.Mesh(groundGeo, mats.ground);
    ground.position.y = -0.18;
    ground.receiveShadow = true;
    world.add(ground);
    addQuad(groundTriangles,
      { x: -180, y: -0.18, z: 180 }, { x: -180, y: -0.18, z: -180 },
      { x: 180, y: -0.18, z: -180 }, { x: 180, y: -0.18, z: 180 }, 0.72);

    // Start / finish stripe.
    for (let k = -1; k <= 1; k++) {
      const stripe = new THREE.Mesh(new THREE.BoxGeometry(ROAD_HALF_WIDTH * 0.9, 0.02, 0.45), mats.line);
      stripe.position.set(0, 0.012, 34 + k * 0.7);
      world.add(stripe);
    }

    // A tabletop jump on the start straight.
    function createRamp() {
      const idx = track.reduce((best, f, i) => Math.abs(f.center.z + 40) < Math.abs(track[best].center.z + 40) && Math.abs(f.center.x) < 2 ? i : best, 0);
      const f = track[idx];
      const tangent = f.tangent, right = f.right;
      const base = V.add(f.center, V.mul(tangent, -3));
      const height = 1.85;
      const upEnd = V.add(base, V.mul(tangent, 7));
      const flatEnd = V.add(upEnd, V.mul(tangent, 5));
      const downEnd = V.add(flatEnd, V.mul(tangent, 9));
      const halfW = 6.3;
      const L = p => V.add(p, V.mul(right, -halfW));
      const R = p => V.add(p, V.mul(right, halfW));
      const lift = (p, h) => ({ x: p.x, y: h, z: p.z });
      const p0 = { ...base }, p1 = lift(upEnd, height), p2 = lift(flatEnd, height), p3 = { ...downEnd };
      const surface = [
        [L(p0), R(p0), R(p1)], [L(p0), R(p1), L(p1)],
        [L(p1), R(p1), R(p2)], [L(p1), R(p2), L(p2)],
        [L(p2), R(p2), R(p3)], [L(p2), R(p3), L(p3)]
      ];
      // The ramp deck is a wheel surface only. Adding it to the chassis collider
      // mesh makes a long rigid box bridge the crest and high-centre, which stalls
      // the car at full throttle; the wheels still climb and support the car.
      for (const tri of surface) addTriangle(physicsTriangles, tri[0], tri[1], tri[2], 1);
      const side = [L(p0), L(p1), L(p2), L(p3), R(p3), R(p2), R(p1), R(p0)];
      for (let i = 0; i < side.length; i++) {
        const a = side[i], b = side[(i + 1) % side.length];
        addTriangle(colliderTriangles, a, b, { x: b.x, y: 0, z: b.z }, 1);
        addTriangle(colliderTriangles, a, { x: b.x, y: 0, z: b.z }, { x: a.x, y: 0, z: a.z }, 1);
      }
      const positions = [];
      for (const tri of surface) for (const p of tri) positions.push(p.x, p.y, p.z);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      geo.computeVertexNormals();
      const ramp = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
        color: 0x647d90, roughness: 0.62, metalness: 0.08, side: THREE.DoubleSide
      }));
      ramp.receiveShadow = true;
      ramp.castShadow = true;
      world.add(ramp);
      for (let i = 0; i < 6; i++) {
        const stripe = new THREE.Mesh(new THREE.BoxGeometry(halfW * 0.92, 0.05, 0.14), mats.line);
        const p = V.add(V.add(base, V.mul(tangent, 0.8 + i * 1.05)), { x: 0, y: 0.22 + i * 0.29, z: 0 });
        stripe.position.set(p.x, p.y, p.z);
        stripe.rotation.y = -Math.atan2(tangent.x, tangent.z);
        stripe.rotation.z = Math.atan2(height, 7);
        world.add(stripe);
      }
    }
    createRamp();

    function createBarrier(x1, z1, x2, z2, height = 1.15) {
      const a = { x: x1, y: 0, z: z1 }, b = { x: x2, y: 0, z: z2 };
      const c = { x: x2, y: height, z: z2 }, d = { x: x1, y: height, z: z1 };
      addQuad(colliderTriangles, a, b, c, d, 1);
      meshFromTriangles([[a, b, c], [a, c, d]], mats.wall);
    }
    // Two barrier stubs framing the pit lane next to the start straight.
    createBarrier(-9.6, 44, -9.6, 24);
    createBarrier(9.6, 44, 9.6, 24);

    /* ------------------------------------------------------------------ */
    /* Vehicle                                                            */
    /* ------------------------------------------------------------------ */

    const surface = TriangleMesh.fromTriangles([...physicsTriangles, ...groundTriangles], 8);
    const walls = TriangleMesh.fromTriangles(colliderTriangles, 6);

    const vehicle = new PlayCarPhysicsEngine({
      surfaceMesh:surface,
      colliderMeshes:[walls],
      groundPlaneY:-.18,
      position: { x: 0, y: 0.08, z: 34 },
      preset: 'simcade',
      config: {
        gravity: 9.81,
        maxSpeed: 62,
        engineAcceleration: 6.4,
        brakeAcceleration: 11,
        tireGrip: 1.08,
        handbrakeGrip: 0.7,
        maxSteerAngle: 0.5,
        highSpeedSteerScale: 0.44,
        suspensionRestLength: 0.3,
        suspensionReach: 0.52,
        bodyHalfExtents: { x: 0.84, y: 0.4, z: 1.7 },
        bodyCenterY: 0.2
      }
    });
    window.PlayCarLabVehicle = vehicle;

    const rigid = vehicle.rigidBody;
    rigid.c.aids.esc = true;
    rigid.c.suspension.progressive = 0.45;
    rigid.c.suspension.dampingBump = 4200;
    rigid.c.suspension.dampingRebound = 6200;
    rigid.c.tire.relaxationLength = 0.32;

    const carRoot = new THREE.Group();
    world.add(carRoot);

    const chassis = new THREE.Mesh(new THREE.BoxGeometry(1.72, 0.58, 3.4), mats.orange);
    chassis.position.y = 0.02;
    chassis.castShadow = true;
    chassis.receiveShadow = true;
    carRoot.add(chassis);
    const nose = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.22, 0.72), mats.orange);
    nose.position.set(0, -0.1, -1.85);
    nose.castShadow = true;
    carRoot.add(nose);
    const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.32, 0.56, 1.4), mats.glass);
    cabin.position.set(0, 0.52, 0.15);
    cabin.castShadow = true;
    carRoot.add(cabin);
    const spoiler = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.08, 0.4), mats.orange);
    spoiler.position.set(0, 0.42, 1.68);
    carRoot.add(spoiler);
    for (const z of [-1.3, 1.3]) {
      const brace = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.34, 0.08), mats.orange);
      brace.position.set(0.62, 0.26, z);
      carRoot.add(brace);
      const brace2 = brace.clone();
      brace2.position.x = -0.62;
      carRoot.add(brace2);
    }
    const headlightMat = new THREE.MeshStandardMaterial({ color: 0xfff4c4, emissive: 0xffd98b, emissiveIntensity: 1.7 });
    for (const x of [-0.5, 0.5]) {
      const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.12, 0.08), headlightMat);
      lamp.position.set(x, -0.03, -1.72);
      carRoot.add(lamp);
    }

    const wheels = [];
    const WHEEL_RADIUS = rigid.c.wheelRadius;
    for (const z of [-1.35, 1.35]) {
      for (const x of [-0.78, 0.78]) {
        const pivot = new THREE.Group();
        pivot.position.set(x, -0.28, z);
        carRoot.add(pivot);
        const spinner = new THREE.Group();
        pivot.add(spinner);
        const tire = new THREE.Mesh(new THREE.CylinderGeometry(WHEEL_RADIUS, WHEEL_RADIUS, 0.27, 22), mats.tire);
        tire.rotation.z = Math.PI / 2;
        tire.castShadow = true;
        spinner.add(tire);
        const hub = new THREE.Mesh(new THREE.CylinderGeometry(WHEEL_RADIUS * 0.5, WHEEL_RADIUS * 0.5, 0.29, 16), mats.rim);
        hub.rotation.z = Math.PI / 2;
        spinner.add(hub);
        wheels.push({ pivot, spinner, z, x });
      }
    }

    /* Dynamic props: cones and crates the rigid-body core can knock around. */
    const props = [];
    function addPropVisual(kind, body) {
      let mesh;
      if (kind === 'cone') {
        mesh = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.7, 14), mats.cone);
        mesh.position.y = 0.02;
      } else {
        mesh = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.62, 0.62), mats.crate);
      }
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      const group = new THREE.Group();
      group.add(mesh);
      world.add(group);
      props.push({ group, body });
    }
    for (const z of [16, 10, 4, -2, -8]) {
      const body = vehicle.addProp({
        shape: 'box', halfExtents: { x: 0.2, y: 0.35, z: 0.2 }, mass: 7,
        position: { x: -3.6, y: 0.38, z }, friction: 0.75, restitution: 0.1
      });
      addPropVisual('cone', body);
    }
    for (const [x, z] of [[3.8, 2], [3.8, 1.2], [4.3, 1.6]]) {
      const body = vehicle.addProp({
        shape: 'box', halfExtents: { x: 0.31, y: 0.31, z: 0.31 }, mass: 16,
        position: { x, y: 0.35 + (x === 4.3 ? 0.65 : 0), z }, friction: 0.8, restitution: 0.08
      });
      addPropVisual('crate', body);
    }

    /* Skid marks. */
    const skidPool = [];
    const SKID_COUNT = 260;
    const skidGeo = new THREE.PlaneGeometry(0.3, 0.75);
    skidGeo.rotateX(-Math.PI / 2);
    for (let i = 0; i < SKID_COUNT; i++) {
      const m = new THREE.Mesh(skidGeo, mats.skid);
      m.visible = false;
      m.renderOrder = 1;
      world.add(m);
      skidPool.push(m);
    }
    let skidIndex = 0;

    /* HUD overlay */
    const overlay = document.createElement('div');
    overlay.style.cssText = 'position:absolute;left:10px;bottom:10px;padding:8px 11px;border-radius:10px;background:rgba(7,17,31,.68);color:#e8f5ff;font:12px/1.55 ui-monospace,monospace;pointer-events:none;backdrop-filter:blur(4px)';
    overlay.innerHTML = '<b id="hud-speed">0 km/h</b> · <span id="hud-gear">N</span> · <span id="hud-aids"></span><br><span id="hud-slip">grip —</span>';
    host.appendChild(overlay);

    /* ------------------------------------------------------------------ */
    /* Input                                                              */
    /* ------------------------------------------------------------------ */

    const input = { gas: false, brake: false, left: false, right: false, handbrake: false };
    function release() { for (const k of Object.keys(input)) input[k] = false; }
    function bindHold(button, key) {
      const el = document.getElementById(button);
      if (!el) return;
      el.addEventListener('pointerdown', e => {
        e.preventDefault(); input[key] = true;
        try { el.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
      });
      ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type =>
        el.addEventListener(type, e => { if (e) e.preventDefault(); input[key] = false; }));
    }
    [['scene-gas', 'gas'], ['scene-brake', 'brake'], ['scene-left', 'left'], ['scene-right', 'right'], ['scene-handbrake', 'handbrake']]
      .forEach(x => bindHold(...x));

    window.addEventListener('keydown', e => {
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key)) e.preventDefault();
      const k = e.key.toLowerCase();
      if (k === 'arrowup' || k === 'w') input.gas = true;
      if (k === 'arrowdown' || k === 's') input.brake = true;
      if (k === 'arrowleft' || k === 'a') { input.left = true; input.steer = null; }
      if (k === 'arrowright' || k === 'd') { input.right = true; input.steer = null; }
      if (e.key === ' ') input.handbrake = true;
      if (k === 'r') resetVehicle();
    }, { passive: false });
    window.addEventListener('keyup', e => {
      const k = e.key.toLowerCase();
      if (k === 'arrowup' || k === 'w') input.gas = false;
      if (k === 'arrowdown' || k === 's') input.brake = false;
      if (k === 'arrowleft' || k === 'a') input.left = false;
      if (k === 'arrowright' || k === 'd') input.right = false;
      if (e.key === ' ') input.handbrake = false;
    });
    window.addEventListener('blur', release);

    function resetVehicle() {
      vehicle.reset({x:0,y:.08,z:34});
      rigid._layout = null;
      input.steer = null;
      release();
      camera.position.set(0, 7, 45);
    }
    document.getElementById('scene-reset')?.addEventListener('click', resetVehicle);
    document.getElementById('reset-car')?.addEventListener('click', resetVehicle);

    let follow = true;
    document.getElementById('scene-follow')?.addEventListener('click', e => {
      follow = !follow;
      e.currentTarget.textContent = follow ? '跟随视角：开' : '跟随视角：关';
    });

    /* Control-center facade: the lab UI only talks to this stable contract. */
    const runtime = {
      engine: vehicle,
      input,
      setInput(patch = {}) { Object.assign(input, patch); return { ...input }; },
      setWeather(value = {}) { return vehicle.setWeather(value); },
      setConfig(patch = {}) { return vehicle.setConfig(patch); },
      setRigidConfig(patch = {}) { return vehicle.setRigidConfig(patch); },
      reset() { resetVehicle(); return vehicle.getSnapshot(); },
      snapshot() { return vehicle.getSnapshot(); }
    };
    /* Weather visuals follow engine.weather: the control center's wetness/wind
     * sliders and the preset menu both end up here through setWeather(). */
    const weatherVisuals = (typeof createPlayCarWeatherVisuals === 'function')
      ? (() => {
          try {
            return createPlayCarWeatherVisuals(THREE, scene, {
              camera, renderer, sun, hemi: hemiLight,
              materials: [mats.road, mats.ground, mats.line, mats.wall]
            });
          } catch (error) {
            console.warn('PlayCar weather visuals unavailable:', error && error.message);
            return null;
          }
        })()
      : null;
    runtime.weatherVisuals = weatherVisuals;

    window.PlayCarRuntime = runtime;
    window.dispatchEvent(new CustomEvent('playcar:ready', { detail: { version: EngineAPI.version } }));

    /* ------------------------------------------------------------------ */
    /* Render loop                                                        */
    /* ------------------------------------------------------------------ */

    const clock = new THREE.Clock();
    let frame = 0;
    let lastStatus = 0;
    const cameraTarget = new THREE.Vector3(0, 1, 34);
    const cameraPosition = new THREE.Vector3(0, 7, 45);
    const tmpForward = new THREE.Vector3();

    function resize() {
      const w = Math.max(1, host.clientWidth);
      const h = Math.max(280, host.clientHeight);
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    }
    const ro = new ResizeObserver(resize);
    ro.observe(host);
    resize();

    function render() {
      requestAnimationFrame(render);
      const dt = Math.min(clock.getDelta(), 0.05);

      // Digital keys/buttons and the control center's analog sliders share the
      // same input object. A numeric value is an analog level; a boolean is 0/1.
      const level = v => typeof v === 'number' ? V.clamp(v, 0, 1) : (v ? 1 : 0);
      const steer = Number.isFinite(input.steer)
        ? V.clamp(input.steer, -1, 1)
        : (input.right ? 1 : 0) - (input.left ? 1 : 0);
      vehicle.advance(dt, {
        throttle: level(input.gas),
        brake: level(input.brake),
        steer,
        handbrake: level(input.handbrake),
        pitch: Number(input.pitch) || 0,
        roll: Number(input.roll) || 0,
        yaw: Number(input.yaw) || 0
      });

      if (weatherVisuals) {
        const w = vehicle.weather || {};
        const wet = Number.isFinite(w.wetness) ? w.wetness : 0;
        const snow = Number.isFinite(w.snow) ? w.snow : 0;
        weatherVisuals.update(dt, {
          wetness: wet,
          rain: Number.isFinite(w.rain) ? w.rain : wet,
          snow,
          fog: Number.isFinite(w.fog) ? w.fog : wet * 0.5,
          wind: w.wind || { x: 0, z: 0 },
          sun: Math.max(0.35, 1 - 0.45 * wet - 0.35 * snow)
        });
      }

      const pose = rigid.interpolatedState();
      const wheelState = vehicle.getWheelState();
      carRoot.position.set(pose.position.x, pose.position.y, pose.position.z);
      carRoot.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

      const suspension = rigid.telemetry.suspension || [];
      const slipRatios = wheelState.slip || [0, 0, 0, 0];
      for (let ix = 0; ix < wheels.length; ix++) {
        const w = wheels[ix];
        const compression = suspension[ix] ? suspension[ix].compression : 0;
        const length = Math.max(rigid.c.suspension.min, rigid.c.suspension.rest - compression);
        w.pivot.position.y = -length;
        w.pivot.rotation.y=w.z<0?-wheelState.steer:0;
        w.spinner.rotation.x -= wheelState.omega[ix] * dt;
      }

      for (const prop of props) {
        prop.group.position.set(prop.body.position.x, prop.body.position.y, prop.body.position.z);
        prop.group.quaternion.set(prop.body.orientation.x, prop.body.orientation.y, prop.body.orientation.z, prop.body.orientation.w);
      }

      // Skid marks at slipping contact patches.
      if (frame % 2 === 0) {
        const contacts = rigid.lastContacts || [];
        for (const c of contacts) {
          const slip = Math.abs(slipRatios[c.index] || 0);
          if (slip < 0.22 || !pose.grounded) continue;
          const mark = skidPool[skidIndex++ % SKID_COUNT];
          mark.position.set(c.point.x, c.point.y + 0.012, c.point.z);
          const f = Q.rotate(pose.orientation, { x: 0, y: 0, z: -1 });
          mark.rotation.y = Math.atan2(f.x, f.z);
          mark.visible = true;
        }
      }

      const speed = V.len(rigid.state.velocity);
      const forward = Q.rotate(pose.orientation, { x: 0, y: 0, z: -1 });
      tmpForward.set(forward.x, forward.y, forward.z);

      if (follow) {
        const back = new THREE.Vector3(0, 3.1, 8.2).applyQuaternion(carRoot.quaternion);
        const desired = new THREE.Vector3(pose.position.x, pose.position.y, pose.position.z).add(back);
        cameraPosition.lerp(desired, Math.min(1, dt * 6));
        camera.position.copy(cameraPosition);
        camera.lookAt(
          pose.position.x + tmpForward.x * 4.5,
          pose.position.y + 1.0 + tmpForward.y * 2.5,
          pose.position.z + tmpForward.z * 4.5
        );
      } else {
        camera.lookAt(pose.position.x, pose.position.y + 0.6, pose.position.z);
      }
      camera.fov = 58 + Math.min(16, speed * 0.5);
      camera.updateProjectionMatrix();

      renderer.render(scene, camera);

      if (++frame % 6 === 0) {
        const t = vehicle.getTelemetry();
        const hudSpeed = document.getElementById('hud-speed');
        const hudGear = document.getElementById('hud-gear');
        const hudAids = document.getElementById('hud-aids');
        const hudSlip = document.getElementById('hud-slip');
        const aids = [];
        if (t.absActive) aids.push('ABS');
        if (t.tcsActive) aids.push('TCS');
        if (t.escActive) aids.push('ESC');
        if (hudSpeed) hudSpeed.textContent = (speed * 3.6).toFixed(0) + ' km/h';
        if (hudGear) hudGear.textContent = t.rpm ? `${t.gear} 挡 · ${Math.round(t.rpm)} rpm` : 'N';
        if (hudAids) hudAids.textContent = aids.join(' ') || '—';
        if (hudSlip) {
          const maxSlip = Math.max(...(t.wheelSlip || [0]));
          hudSlip.textContent = `${t.grounded ? '抓地' : '腾空'} · slip ${maxSlip.toFixed(2)} · ${t.contacts} 轮接触`;
        }
        if (status && performance.now() - lastStatus > 1200) {
          lastStatus = performance.now();
          status.textContent = `3D 场景运行中 · ${rigid.world.stepCount} 物理子步 · ${rigid.world.stats.contacts} 接触点`;
        }
      }
    }

    if (status) status.textContent = '3D 场景已就绪 · 刚体内核 3.0';
    render();

  } catch (error) {
    const status = document.getElementById('scene-status');
    if (status) status.textContent = '3D 场景运行失败：' + (error && error.message ? error.message : String(error));
    console.error('PlayCar 3D scene failed:', error);
    throw error;
  }
})();

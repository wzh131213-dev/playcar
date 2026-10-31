/* PlayCar Physics Engine + Three.js Scene — single-file build.
 * Physics engine is registered synchronously before the 3D scene is imported.
 * Three.js uses runtime import with a CDN fallback.
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
      this.weather={wind:{x:Number.isFinite(wind.x)?V.clamp(wind.x,-20,20):0,z:Number.isFinite(wind.z)?V.clamp(wind.z,-20,20):0},wetness:Number.isFinite(weather.wetness)?V.clamp(weather.wetness,0,1):0};
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
        weather:{wind:{...this.weather.wind},wetness:this.weather.wetness},input:{...this.lastInput},
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
/* PlayCar rigid-body vehicle physics 2.7.0
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
 suspensionGeometry:{
  front:{camberStatic:-.5,camberGain:-22,toeStatic:0,toeGain:.35,rollCenterHeight:.12,rollCenterTravelGain:.10,rollCenterLateralGain:.08,rollCamberGain:.28,
   hardpoints:{enabled:true,upperInner:{x:.28,y:.82},upperOuter:{x:.68,y:.72},lowerInner:{x:.30,y:.34},lowerOuter:{x:.76,y:.30},wheelCenterX:.88}},
  rear:{camberStatic:-1.0,camberGain:-15,toeStatic:.08,toeGain:-.22,rollCenterHeight:.17,rollCenterTravelGain:.07,rollCenterLateralGain:.06,rollCamberGain:.18,
   hardpoints:{enabled:true,upperInner:{x:.30,y:.80},upperOuter:{x:.69,y:.70},lowerInner:{x:.32,y:.33},lowerOuter:{x:.77,y:.29},wheelCenterX:.89}}
 },
 tire:{mu:1.08,corneringStiffness:78000,longitudinalStiffness:95000,relaxation:.04,relaxationAngle:.08,loadSensitivity:.12,referenceLoad:3000,aligningTrail:.08,pacejka:{enabled:false,B:0,C:1.3,E:.97,peak:1},frictionEllipse:{longitudinal:1,lateral:1}},
 engine:{peakTorque:280,idleRpm:900,redline:6500,finalDrive:3.9,gears:[3.2,2.1,1.5,1.18,.94,.78],efficiency:.88},
 antiRoll:{front:8500,rear:6500},body:{rollStiffness:9000,rollDamping:2200,pitchStiffness:14000,pitchDamping:2800},differential:{type:'open',limitedSlip:2.2},aids:{abs:true,tcs:true,absSlip:.18,tcsSlip:.22},steering:{maxAngle:.48,highSpeedScale:.42},
 brakeTorque:4200,handbrakeTorque:6500,dragArea:0.68,airDensity:1.225,downforceArea:0.0,
 restitution:.08,collisionFriction:.55,bodyHalf:{x:.88,y:.48,z:2.05},ccd:true
};
function circleIntersections(a,ra,b,rb){
 const dx=b.x-a.x,dy=b.y-a.y,d=Math.hypot(dx,dy);
 if(d<1e-8||d>ra+rb+1e-8||d<Math.abs(ra-rb)-1e-8)return [];
 const along=(ra*ra-rb*rb+d*d)/(2*d),height=Math.sqrt(Math.max(0,ra*ra-along*along));
 const x=a.x+along*dx/d,y=a.y+along*dy/d,px=-dy*height/d,py=dx*height/d;
 return [{x:x+px,y:y+py},{x:x-px,y:y-py}];
}
function lineIntersection(a,b,c,d){
 const rx=b.x-a.x,ry=b.y-a.y,sx=d.x-c.x,sy=d.y-c.y,den=rx*sy-ry*sx;
 if(Math.abs(den)<1e-8)return null;
 const t=((c.x-a.x)*sy-(c.y-a.y)*sx)/den;
 return{x:a.x+t*rx,y:a.y+t*ry};
}
function solveHardpointKinematics(g,compression,fallbackHeight){
 const hp=g.hardpoints;
 if(!hp?.enabled)return null;
 const ui=hp.upperInner,li=hp.lowerInner,uo=hp.upperOuter,lo=hp.lowerOuter;
 if(![ui,li,uo,lo].every(p=>Number.isFinite(p?.x)&&Number.isFinite(p?.y)))return{valid:false,rollCenterHeight:fallbackHeight,wheelOffsetX:0,camberDeltaDeg:0};
 const distance=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
 const lowerLength=distance(li,lo),upperLength=distance(ui,uo),uprightLength=distance(uo,lo);
 const solveAt=travel=>{
  const y=lo.y+travel,dy=y-li.y,remain=lowerLength*lowerLength-dy*dy;
  if(remain<0)return null;
  const dx=Math.sqrt(Math.max(0,remain)),lower={x:li.x+(lo.x>=li.x?dx:-dx),y};
  const candidates=circleIntersections(ui,upperLength,lower,uprightLength);
  if(!candidates.length)return null;
  candidates.sort((a,b)=>distance(a,uo)-distance(b,uo));
  return{lower,upper:candidates[0]};
 };
 const current=solveAt(compression),baseline=solveAt(0);
 if(!current||!baseline)return{valid:false,rollCenterHeight:fallbackHeight,wheelOffsetX:0,camberDeltaDeg:0,instantCenter:null};
 const angle=p=>Math.atan2(p.upper.x-p.lower.x,p.upper.y-p.lower.y);
 const camberDeltaDeg=(angle(current)-angle(baseline))*180/Math.PI;
 const ic=lineIntersection(ui,current.upper,li,current.lower);
 const contactX=Number.isFinite(hp.contactPatchX)?hp.contactPatchX:hp.wheelCenterX??.88;
 let rollCenterHeight=fallbackHeight;
 if(ic&&Math.abs(ic.x-contactX)>1e-5){
  const candidate=ic.y*(0-contactX)/(ic.x-contactX);
  if(Number.isFinite(candidate))rollCenterHeight=V.clamp(candidate,-.15,1.2);
 }
 return{valid:true,rollCenterHeight,wheelOffsetX:current.lower.x-lo.x,camberDeltaDeg,instantCenter:ic,upperOuter:current.upper,lowerOuter:current.lower};
}
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
  this.c=Object.assign({},clone(DEFAULTS),options.config||{});this.c.inertia=Object.assign({},DEFAULTS.inertia,options.config?.inertia||{});this.c.com=Object.assign({},DEFAULTS.com,options.config?.com||{});this.c.suspension=Object.assign({},DEFAULTS.suspension,options.config?.suspension||{});this.c.suspensionGeometry={front:Object.assign({},DEFAULTS.suspensionGeometry.front,options.config?.suspensionGeometry?.front||{}),rear:Object.assign({},DEFAULTS.suspensionGeometry.rear,options.config?.suspensionGeometry?.rear||{})};for(const axle of ['front','rear']){const base=DEFAULTS.suspensionGeometry[axle].hardpoints,custom=options.config?.suspensionGeometry?.[axle]?.hardpoints||{};this.c.suspensionGeometry[axle].hardpoints=Object.assign({},base,custom);for(const key of ['upperInner','upperOuter','lowerInner','lowerOuter'])this.c.suspensionGeometry[axle].hardpoints[key]=Object.assign({},base[key],custom[key]||{});}this.c.tire=Object.assign({},DEFAULTS.tire,options.config?.tire||{});this.c.tire.pacejka=Object.assign({},DEFAULTS.tire.pacejka,options.config?.tire?.pacejka||{});this.c.tire.frictionEllipse=Object.assign({},DEFAULTS.tire.frictionEllipse,options.config?.tire?.frictionEllipse||{});this.c.engine=Object.assign({},DEFAULTS.engine,options.config?.engine||{});this.c.antiRoll=Object.assign({},DEFAULTS.antiRoll,options.config?.antiRoll||{});this.c.body=Object.assign({},DEFAULTS.body,options.config?.body||{});this.c.differential=Object.assign({},DEFAULTS.differential,options.config?.differential||{});this.c.aids=Object.assign({},DEFAULTS.aids,options.config?.aids||{});this.c.steering=Object.assign({},DEFAULTS.steering,options.config?.steering||{});
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
 _wheelKinematics(index,compression,normal,input={}) {
  const c=this.c,s=this.state,front=index<2,side=(index===0||index===2)?-1:1;
  const g=front?c.suspensionGeometry.front:c.suspensionGeometry.rear;
  const tilt=this._bodyTilt(),travel=V.clamp(compression/Math.max(.001,c.suspension.max-c.suspension.min),-1,1);
  const fallbackRollCenter=Math.max(0,g.rollCenterHeight+g.rollCenterTravelGain*compression+g.rollCenterLateralGain*side*compression);
  const hardpoint=solveHardpointKinematics(g,compression,fallbackRollCenter);
  const camberDeg=g.camberStatic+(hardpoint?.valid?hardpoint.camberDeltaDeg:g.camberGain*compression)+g.rollCamberGain*tilt.roll*180/Math.PI;
  const toeDeg=g.toeStatic+g.toeGain*compression;
  const speed=V.len(s.velocity),steer=front?V.clamp(input.steer||0,-1,1)*c.steering.maxAngle*(1-(1-c.steering.highSpeedScale)*V.clamp(speed/55,0,1)):0;
  const n=V.norm(normal),bodyForward=Q.rotate(s.orientation,{x:0,y:0,z:-1});
  let forward=V.norm(V.sub(bodyForward,V.mul(n,V.dot(bodyForward,n))));
  if(V.len(forward)<1e-8)forward=V.norm(V.cross({x:1,y:0,z:0},n));
  const angle=(steer+toeDeg*Math.PI/180);
  forward=V.norm(V.add(V.mul(forward,Math.cos(angle)),V.mul(V.cross(forward,n),Math.sin(angle))));
  const right=V.norm(V.cross(forward,n));
  const camberedNormal=Q.rotate(Q.axis(forward,camberDeg*Math.PI/180),n);
  const camberedRight=V.norm(V.cross(forward,camberedNormal));
  const rollCenterHeight=hardpoint?.valid?hardpoint.rollCenterHeight:fallbackRollCenter;
  return {camber:camberDeg*Math.PI/180,camberDeg,toe:toeDeg*Math.PI/180,toeDeg,steer,forward,right,camberedNormal,camberedRight,rollCenterHeight,travel,side,axle:front?'front':'rear',hardpointValid:!!hardpoint?.valid,instantCenter:hardpoint?.instantCenter||null,upperOuter:hardpoint?.upperOuter||null,lowerOuter:hardpoint?.lowerOuter||null,wheelOffsetX:hardpoint?.wheelOffsetX||0};
 }
 _contacts(input={}){
  const c=this.c,s=this.state,out=[],q=s.orientation,half=c.wheelbase/2,track=c.track/2;
  const mounts=[[-track,-half],[track,-half],[-track,half],[track,half]];
  for(let i=0;i<4;i++){
   const local={x:mounts[i][0],y:0,z:mounts[i][1]},r=Q.rotate(q,local),origin=V.add(s.position,r);
   const down=Q.rotate(q,{x:0,y:-1,z:0}),reach=c.suspension.max+c.wheelRadius;
   let hit=null;
   if(this.mesh?.closestPoint){hit=this.mesh.closestPoint(origin,reach+c.wheelRadius,{x:0,y:1,z:0});}
   if(!hit&&Number.isFinite(this.groundY)){const dist=origin.y-this.groundY;if(dist<reach+c.wheelRadius)hit={point:{x:origin.x,y:this.groundY,z:origin.z},normal:{x:0,y:1,z:0},distance:Math.max(0,dist),signedDistance:dist-c.wheelRadius};}
   if(!hit)continue;
   let n=V.norm(hit.normal||{x:0,y:1,z:0});if(V.dot(n,{x:0,y:1,z:0})<0)n=V.mul(n,-1);
   const rawLength=hit.distance-c.wheelRadius,length=V.clamp(rawLength, c.suspension.min,c.suspension.max),compression=c.suspension.rest-length;
   const geometry=this._wheelKinematics(i,compression,n,input),rawPoint=hit.point||V.sub(origin,V.mul(n,hit.distance));
   const bodyRight=Q.rotate(q,{x:1,y:0,z:0}),tangentRight=V.norm(V.sub(bodyRight,V.mul(n,V.dot(bodyRight,n))));
   const point=V.add(rawPoint,V.mul(tangentRight,geometry.wheelOffsetX*geometry.side)),arm=V.sub(point,s.position),pv=this._pointVelocity(arm),vn=V.dot(pv,n),wheelCenter=V.add(point,V.mul(n,c.wheelRadius));
   out.push({index:i,point,normal:n,arm,compression,length,rawLength,normalVelocity:vn,velocity:pv,local,wheelCenter,geometry,tireForward:geometry.forward,tireRight:geometry.right,friction:Number.isFinite(hit.friction)?Math.max(0,hit.friction):1});
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
  const contacts=this._contacts(input);this.lastContacts=contacts;
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
  const frontGeometry=contacts.filter(h=>h.index<2),rearGeometry=contacts.filter(h=>h.index>=2);
  const axleRollCenter=axle=>axle.length?axle.reduce((sum,h)=>sum+h.geometry.rollCenterHeight,0)/axle.length:0;
  const frontRollCenterHeight=axleRollCenter(frontGeometry),rearRollCenterHeight=axleRollCenter(rearGeometry);
  const effectiveRollCenterHeight=(frontRollCenterHeight+rearRollCenterHeight)*.5;
  const lateralMomentArm=Math.max(.05,c.com.y-effectiveRollCenterHeight);
  const requestedLateralTransfer=c.mass*requestedLateralAcceleration*lateralMomentArm/Math.max(.1,c.track);
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
   let wheelForward=h.tireForward||forward,wheelRight=h.tireRight||right;
   const vLong=V.dot(h.velocity,wheelForward),vLat=V.dot(h.velocity,wheelRight),omega=s.wheelOmega[i];
   const rawSlip=(omega*c.wheelRadius-vLong)/Math.max(2,Math.abs(vLong)),relax=V.clamp(dt/Math.max(.001,c.tire.relaxation),0,1);this._slipState[i]+=(rawSlip-this._slipState[i])*relax;
   let slipRatio=this._slipState[i],rawSlipAngle=Math.atan2(vLat,Math.max(1,Math.abs(vLong))),angleRelax=V.clamp(dt/Math.max(.001,c.tire.relaxationAngle||c.tire.relaxation),0,1);this._slipAngleState[i]+=(rawSlipAngle-this._slipAngleState[i])*angleRelax;let slipAngle=this._slipAngleState[i];
   slipAngle+=h.geometry.camber*(c.tire.camberThrustGain??.08);
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
  const after=this._contacts(input);s.grounded=after.length>=2;
  this.tick++;
  this.telemetry={tick:this.tick,contacts:after.length,normalLoads:normalLoad,forces,suspension:suspensionTelemetry,wheelGeometry:after.map(h=>({wheel:h.index,axle:h.geometry.axle,camber:h.geometry.camber,camberDeg:h.geometry.camberDeg,toe:h.geometry.toe,toeDeg:h.geometry.toeDeg,rollCenterHeight:h.geometry.rollCenterHeight,travel:h.geometry.travel,hardpointValid:h.geometry.hardpointValid,instantCenter:h.geometry.instantCenter?{...h.geometry.instantCenter}:null,upperOuter:h.geometry.upperOuter?{...h.geometry.upperOuter}:null,lowerOuter:h.geometry.lowerOuter?{...h.geometry.lowerOuter}:null,wheelOffsetX:h.geometry.wheelOffsetX,wheelCenter:{...h.wheelCenter},contactNormal:{...h.normal},tireForward:{...h.tireForward},tireRight:{...h.tireRight}})),rollCenter:{front:frontRollCenterHeight,rear:rearRollCenterHeight,average:effectiveRollCenterHeight,lateralMomentArm},bodyTilt:this._bodyTilt(),bodyStabilization:{...c.body},gear:this.gear,rpm:this.engineRpm,speed:V.len(s.velocity),downforce,absActive:this.aidState.abs,tcsActive:this.aidState.tcs,antiRoll:{front:c.antiRoll.front,rear:c.antiRoll.rear},loadTransfer:loadTransferTelemetry,differential:diffType,driveLayout:driveType,drivenWheels:[...driven]};
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
root.PlayCarRigidBody={RigidBodyVehicle,Vector3:V,Quaternion:Q,calculateTireForces,defaults:clone(DEFAULTS),version:'2.7.0'};
if(typeof module!=='undefined'&&module.exports)module.exports=root.PlayCarRigidBody;
})(typeof globalThis!=='undefined'?globalThis:this);


  const LegacyPlayCarPhysicsEngine=PlayCarPhysicsEngine;
  class UnifiedPlayCarPhysicsEngine extends LegacyPlayCarPhysicsEngine{
    constructor(options={}){
      super(options);const c=this.config,p=options.position||{x:0,y:0,z:0};
      const cfg={fixedDt:c.fixedTimeStep,maxSubsteps:c.maxSubsteps,mass:1250,gravity:c.gravity,
        inertia:{x:520,y:1450,z:1650},com:{x:0,y:.52,z:0},wheelbase:c.wheelbase,track:c.trackHalfWidth*2,wheelRadius:.37,
        suspension:{rest:.32,min:.08,max:.48,stiffness:32000,damping:4200,bump:90000},
        antiRoll:{front:8500,rear:6500},differential:{type:'lsd',layout:'rwd',limitedSlip:2.2},aids:{abs:true,tcs:true,absSlip:.18,tcsSlip:.22},steering:{maxAngle:c.maxSteerAngle,highSpeedScale:c.highSpeedSteerScale},
        tire:{mu:c.tireGrip,corneringStiffness:78000,longitudinalStiffness:95000,relaxation:.04},
        engine:{peakTorque:280,idleRpm:900,redline:6500,finalDrive:3.9,gears:[3.2,2.1,1.5,1.18,.94,.78],efficiency:.88},
        brakeTorque:4200,handbrakeTorque:6500,dragArea:.68,airDensity:1.225,
        restitution:c.collisionRestitution,collisionFriction:c.collisionFriction,bodyHalf:{x:c.bodyHalfExtents.x,y:.42,z:c.bodyHalfExtents.z},ccd:true};
      this.rigidBody=new root.PlayCarRigidBody.RigidBodyVehicle({surfaceMesh:this.surfaceMesh,colliderMeshes:this.colliderMeshes,
        groundPlaneY:this.groundPlaneY,position:{x:p.x,y:(p.y||0)+.36,z:p.z},config:cfg});
      this.rigidBody.state.orientation={...this.state.orientation};this.rigidBody.previous.orientation={...this.state.orientation};this._syncRigidState();
    }
    _syncRigidState(){const r=this.rigidBody.state,s=this.state;s.position={x:r.position.x,y:r.position.y-.36,z:r.position.z};s.velocity={...r.velocity};s.orientation={...r.orientation};s.angularVelocity={...r.angularVelocity};s.grounded=!!r.grounded;s.gripState=r.grounded?(this.lastInput.handbrake>0?'DRIFT':'GRIP'):'AIR';s.tick=r.tick;this.tick=r.tick;this.telemetry={longitudinalSpeed:0,lateralSpeed:0,slipAngle:0,downforce:this.rigidBody.telemetry.downforce||0,airborne:!r.grounded,wheelContacts:this.rigidBody.telemetry.contacts||0};}
    advance(frameDelta,input){const n=input===undefined?this.lastInput:this.setInput(input);this.rigidBody.setSurfaceMesh(this.surfaceMesh);this.rigidBody.c.tire.mu=this.config.tireGrip*(1-.42*this.weather.wetness);this.rigidBody.c.gravity=this.config.gravity;this.rigidBody.c.fixedDt=this.config.fixedTimeStep;
      const result=this.rigidBody.advance(frameDelta,{throttle:n.throttle,brake:n.brake,steer:n.steer,handbrake:n.handbrake>0});this._syncRigidState();
      LegacyPlayCarPhysicsEngine.prototype._applyStaticCollisions.call(this);
      this.rigidBody.state.position={x:this.state.position.x,y:this.state.position.y+.36,z:this.state.position.z};this.rigidBody.state.velocity={...this.state.velocity};this.rigidBody.state.orientation={...this.state.orientation};
      const t=this.getTelemetry();this.telemetry.longitudinalSpeed=t.longitudinalSpeed;this.telemetry.lateralSpeed=t.lateralSpeed;this.telemetry.slipAngle=t.slipAngle;this.telemetry.wheelContacts=this.rigidBody.telemetry.contacts||0;return result;}
    interpolatedState(){const p=this.rigidBody.interpolatedState();return {...p,position:{x:p.position.x,y:p.position.y-.36,z:p.position.z},gripState:this.state.gripState,collisionState:this.state.collisionState,checkpointId:this.state.checkpointId,tick:p.tick};}
    getTelemetry(){const f=Q.rotate(this.state.orientation,V.make(0,0,-1)),r=Q.rotate(this.state.orientation,V.make(1,0,0)),longitudinalSpeed=V.dot(this.state.velocity,f),lateralSpeed=V.dot(this.state.velocity,r);return {...this.telemetry,speed:V.len(this.state.velocity),longitudinalSpeed,lateralSpeed,slipAngle:Math.atan2(lateralSpeed,Math.max(1,Math.abs(longitudinalSpeed))),airborne:!this.state.grounded,grounded:this.state.grounded,verticalSpeed:this.state.velocity.y,position:{...this.state.position},velocity:{...this.state.velocity},wheelContacts:this.rigidBody.telemetry.contacts||0};}
    getWheelState(){return {omega:[...this.rigidBody.state.wheelOmega],contacts:this.rigidBody.lastContacts.map(x=>x.index),steer:this.lastInput.steer*.48};}
    setConfig(patch={}){const out=super.setConfig(patch);if(this.rigidBody){const c=this.config,r=this.rigidBody;r.c.tire.mu=c.tireGrip*(1-.42*this.weather.wetness);r.c.gravity=c.gravity;r.c.fixedDt=c.fixedTimeStep;r.c.maxSubsteps=c.maxSubsteps;r.c.wheelbase=c.wheelbase;r.c.track=c.trackHalfWidth*2;r.c.steering.maxAngle=c.maxSteerAngle;r.c.steering.highSpeedScale=c.highSpeedSteerScale;r.c.tire.mu=c.tireGrip*(1-.42*this.weather.wetness);r.c.brakeTorque=c.brakeAcceleration*1250*.37/4;r.c.handbrakeTorque=r.c.brakeTorque*1.55;r.c.engine.peakTorque=c.engineAcceleration*1250*.37/(3.2*3.9*.88);r.c.dragArea=Math.max(.05,c.aerodynamicDrag*300);r.c.downforceArea=c.maxDownforce*2/(1.225*Math.max(1,c.maxSpeed*c.maxSpeed));r.c.antiRoll.front=8500;r.c.antiRoll.rear=6500;r.c.aids.abs=true;r.c.aids.tcs=true;r.c.suspension.rest=c.suspensionRestLength;r.c.suspension.max=c.suspensionReach;r.c.restitution=c.collisionRestitution;r.c.collisionFriction=c.collisionFriction;r.c.bodyHalf={x:c.bodyHalfExtents.x,y:c.bodyHalfExtents.y,z:c.bodyHalfExtents.z};}return out;}
    setDifferential(type='lsd',layout='rwd',limitedSlip=2.2){if(!['open','lsd'].includes(type)||!['fwd','rwd','awd'].includes(layout)||!Number.isFinite(limitedSlip)||limitedSlip<1)throw new TypeError('Invalid differential settings');this.rigidBody.c.differential={type,layout,limitedSlip};return {...this.rigidBody.c.differential};}
    setWeather(weather={}){const out=super.setWeather(weather);if(this.rigidBody)this.rigidBody.c.tire.mu=this.config.tireGrip*(1-.42*this.weather.wetness);return out;}
    reset(position={x:0,y:0,z:0},yaw=0,up=V.make(0,1,0)){super.reset(position,yaw,up);if(this.rigidBody){this.rigidBody.reset({x:position.x,y:position.y+.36,z:position.z});this.rigidBody.state.orientation={...this.state.orientation};this.rigidBody.previous.orientation={...this.state.orientation};this._syncRigidState();}}
    debugSnapshot(){return {...super.debugSnapshot(),solver:'rigid-body-2.0.0',rpm:this.rigidBody.engineRpm,gear:this.rigidBody.gear,wheelOmega:[...this.rigidBody.state.wheelOmega]};}
  }
  api.PlayCarPhysicsEngine=UnifiedPlayCarPhysicsEngine;

  root.PlayCarPhysicsEngine=api;
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);


(async function startPlayCar3DScene(){
  let THREE;
  const sources = [
    'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js',
    'https://unpkg.com/three@0.180.0/build/three.module.js'
  ];
  let lastError;
  for (const source of sources) {
    try { THREE = await import(source); break; }
    catch (error) { lastError = error; }
  }
  if (!THREE) {
    const status = document.getElementById('scene-status');
    if (status) status.textContent = '3D 模块加载失败：两个 Three.js CDN 均无法加载。请检查网络或稍后重试。';
    console.error('PlayCar Three.js module load failed:', lastError);
    return;
  }
  try {
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
    const physicsTriangles=[],colliderTriangles=[],groundTriangles=[];
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
    const vehicle=new PlayCarPhysicsEngine({surfaceMesh:surface,colliderMeshes:[walls],groundPlaneY:-.18,position:{x:0,y:.08,z:34},preset:'simcade',config:{gravity:9.81,maxSpeed:60,engineAcceleration:5.2,brakeAcceleration:9.5,tireGrip:1.05,handbrakeGrip:.72,bodyHalfExtents:{x:.82,y:.42,z:1.55},wheelbase:2.55}});
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
    
    const input={gas:false,brake:false,left:false,right:false,handbrake:0,pitch:0,roll:0,yaw:0};
    const runtime={
      engine:vehicle,input,
      setInput(patch={}){Object.assign(input,patch);return {...input};},
      setWeather(value={}){return vehicle.setWeather(value);},
      setConfig(patch={}){return vehicle.setConfig(patch);},
      reset(){resetVehicle();return vehicle.getSnapshot();},
      snapshot(){return vehicle.getSnapshot();}
    };
    window.PlayCarRuntime=runtime;
    window.dispatchEvent(new CustomEvent('playcar:ready',{detail:{version:EngineAPI.version}}));
    function release(){for(const k of Object.keys(input))input[k]=false;}
    function bindHold(button,key){
     const el=document.getElementById(button);if(!el)return;
     el.addEventListener('pointerdown',e=>{e.preventDefault();input[key]=true;try{el.setPointerCapture(e.pointerId);}catch(_){}});
     ['pointerup','pointercancel','lostpointercapture'].forEach(type=>el.addEventListener(type,e=>{if(e)e.preventDefault();input[key]=false;}));
    }
    [['scene-gas','gas'],['scene-brake','brake'],['scene-left','left'],['scene-right','right']].forEach(x=>bindHold(...x));
    window.addEventListener('keydown',e=>{if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(e.key))e.preventDefault();if(e.key==='ArrowUp'||e.key.toLowerCase()==='w')input.gas=true;if(e.key==='ArrowDown'||e.key.toLowerCase()==='s')input.brake=true;if(e.key==='ArrowLeft'||e.key.toLowerCase()==='a'){input.left=true;input.steer=-1;}if(e.key==='ArrowRight'||e.key.toLowerCase()==='d'){input.right=true;input.steer=1;}},{passive:false});
    window.addEventListener('keyup',e=>{if(e.key==='ArrowUp'||e.key.toLowerCase()==='w')input.gas=false;if(e.key==='ArrowDown'||e.key.toLowerCase()==='s')input.brake=false;if(e.key==='ArrowLeft'||e.key.toLowerCase()==='a')input.left=false;if(e.key==='ArrowRight'||e.key.toLowerCase()==='d')input.right=false;if(!input.left&&!input.right)input.steer=0;});
    window.addEventListener('blur',release);
    function resetVehicle(){vehicle.reset({x:0,y:.08,z:34});release();}
    document.getElementById('scene-reset')?.addEventListener('click',resetVehicle);
    let follow=true;
    document.getElementById('scene-follow')?.addEventListener('click',e=>{follow=!follow;e.currentTarget.textContent=follow?'跟随视角：开':'跟随视角：关';});
    const speedEl=document.getElementById('scene-speed'),gripEl=document.getElementById('scene-grip'),heightEl=document.getElementById('scene-height'),slipEl=document.getElementById('scene-slip');
    const clock=new THREE.Clock();let frame=0,lastStatus=0;
    function resize(){const w=Math.max(1,host.clientWidth),h=Math.max(260,host.clientHeight);renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
    const ro=new ResizeObserver(resize);ro.observe(host);resize();
    function render(){
     requestAnimationFrame(render);const dt=Math.min(clock.getDelta(),.05);vehicle.advance(dt,input);
     const wheelState=vehicle.getWheelState();const pose=vehicle.interpolatedState();carRoot.position.set(pose.position.x,pose.position.y,pose.position.z);carRoot.quaternion.set(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
     for(let wi=0;wi<wheels.length;wi++){const w=wheels[wi],ix=w.z<0?(w.pivot.position.x<0?0:1):(w.pivot.position.x<0?2:3);w.tire.rotation.x-=wheelState.omega[ix]*dt;w.pivot.rotation.y=w.z<0?wheelState.steer:0;}
     shadow.position.x=vehicle.state.position.x;shadow.position.z=vehicle.state.position.z;
     if(follow){const p=carRoot.position.clone();const back=new THREE.Vector3(0,5.2,11).applyQuaternion(carRoot.quaternion);camera.position.lerp(p.clone().add(back),.075);camera.lookAt(p.x,p.y+.65,p.z-1.2);}
     renderer.render(scene,camera);
     if(++frame%6===0){const t=vehicle.getTelemetry();if(speedEl)speedEl.textContent=(t.speed*3.6).toFixed(0)+' km/h';if(gripEl)gripEl.textContent=vehicle.state.gripState;if(heightEl)heightEl.textContent=vehicle.state.position.y.toFixed(2)+' m';if(slipEl)slipEl.textContent=(t.slipAngle*180/Math.PI).toFixed(1)+'°';if(status)status.textContent='3D 场景运行中 · '+vehicle.state.tick+' ticks';}
    }
    status.textContent='3D 场景已就绪';
    render();
    
  } catch (error) {
    const status = document.getElementById('scene-status');
    if (status) status.textContent = '3D 场景初始化失败：' + (error?.message || error);
    console.error('PlayCar 3D scene initialization failed:', error);
  }
})();

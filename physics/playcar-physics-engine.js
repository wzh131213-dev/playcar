/* PlayCar Physics Engine 1.0.3
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
      for(const tri of triangles)this.addTriangle(tri[0],tri[1],tri[2]);
    }
    static fromTriangles(triangles,cellSize=8){return new TriangleMesh(triangles,cellSize);}
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
    addTriangle(a,b,c){
      const face=V.norm(V.cross(V.sub(b,a),V.sub(c,a)));
      const tri={id:this.triangles.length,a:{...a},b:{...b},c:{...c},rawNormal:face,
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
          best2=d2;
          // Orient the contact normal consistently with the previous support normal.
          // Road triangles may be wound either way; trusting rawNormal can make the
          // suspension push the car through the road and invert steering on banked sections.
          let n=tri.rawNormal;
          if(preferredNormal&&V.len2(preferredNormal)>EPS&&V.dot(n,preferredNormal)<0)n=V.mul(n,-1);
          best={point:q,normal:n,distance:Math.sqrt(d2),signedDistance:V.dot(V.sub(p,q),n),triangleIndex:tri.id,face:tri};
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
    gravity:24,maxSpeed:78,engineAcceleration:28,brakeAcceleration:56,
    rollingResistance:.22,aerodynamicDrag:.0026,
    wheelbase:2.7,maxSteerAngle:.48,highSpeedSteerScale:.48,
    tireGrip:1.65,handbrakeGrip:.38,handbrakeYawMultiplier:1.55,
    airYawRate:.62,airPitchRate:.55,airRollRate:.82,
    wheelbaseHalf:1.35,trackHalfWidth:.78,wheelProbeY:.07,
    suspensionRestLength:.09,suspensionReach:.9,contactPositionGain:1,
    collisionRestitution:.16,collisionFriction:.42,
    bodyHalfExtents:{x:1.05,y:.84,z:2.1},bodyCenterY:.84,
    maxCollisionIterations:3
  };

  class PlayCarPhysicsEngine {
    constructor(options={}){
      this.config=Object.assign({},DEFAULTS,options.config||{});
      this.config.bodyHalfExtents=Object.assign({},DEFAULTS.bodyHalfExtents,(options.config||{}).bodyHalfExtents||{});
      this.surfaceMesh=options.surfaceMesh||null;
      this.colliderMeshes=options.colliderMeshes||[];
      this.frameAt=typeof options.frameAt==='function'?options.frameAt:null;
      this.accumulator=0;this.tick=0;this.droppedTime=0;this.lastStepMs=0;this.totalStepMs=0;
      this.checkpoints=[];this.currentCheckpointIndex=0;
      this.state={position:V.make(0,0,0),velocity:V.make(0,0,0),orientation:Q.identity(),
        angularVelocity:V.make(0,0,0),grounded:false,contactNormal:V.make(0,1,0),collisionState:'none',
        collisionCount:0,collisionTimer:0,totalCollisionEvents:0,checkpointId:'start',gripState:'AIR',tick:0};
      this.previousState=cloneState(this.state);
      if(options.position)this.reset(options.position,options.yaw||0);
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
      this.accumulator=0;this.state={position:{...position},velocity:V.make(0,0,0),
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
    advance(frameDelta,input={}){
      const dt=V.clamp(Number.isFinite(frameDelta)?frameDelta:0,0,this.config.maxFrameDelta);
      this.accumulator+=dt;let steps=0;
      while(this.accumulator+EPS>=this.config.fixedTimeStep&&steps<this.config.maxSubsteps){
        this._fixedStep(this.config.fixedTimeStep,input);this.accumulator-=this.config.fixedTimeStep;steps++;
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
      const steer=V.clamp((input.right?1:0)-(input.left?1:0),-1,1);
      const airSteer=-steer;
      const throttle=V.clamp(Number(input.throttle??(input.gas?1:0))||0,0,1);
      const brake=V.clamp(Number(input.brake)||0,0,1);
      const handbrake=V.clamp(Number(input.handbrake??(input.hand?1:0))||0,0,1);
      const fwd=Q.rotate(s.orientation,V.make(0,0,-1)),right=Q.rotate(s.orientation,V.make(1,0,0));
      const longitudinal=V.dot(s.velocity,fwd),speed=V.len(s.velocity);
      const speedRatio=V.clamp(Math.abs(longitudinal)/c.maxSpeed,0,1);
      const drive=throttle*c.engineAcceleration*(1-speedRatio*.68);
      s.velocity=V.add(s.velocity,V.mul(fwd,drive*dt));
      let longNow=V.dot(s.velocity,fwd);
      const braking= Math.min(Math.abs(longNow),brake*c.brakeAcceleration*dt);
      if(braking>0)s.velocity=V.sub(s.velocity,V.mul(fwd,Math.sign(longNow)*braking));
      const postBrakeSpeed=V.len(s.velocity);
      s.velocity=V.mul(s.velocity,Math.exp(-(c.rollingResistance+c.aerodynamicDrag*postBrakeSpeed)*dt));
      if(wasGrounded){
        const grip=handbrake>0?c.handbrakeGrip:c.tireGrip;
        const sideSpeed=V.dot(s.velocity,right);
        const sideAccel=V.clamp(-sideSpeed*grip,-grip*c.gravity,grip*c.gravity);
        s.velocity=V.add(s.velocity,V.mul(right,sideAccel*dt));
        const speedForSteer=Math.max(1,Math.abs(V.dot(s.velocity,fwd)));
        const steerAngle=c.maxSteerAngle*(1-speedRatio*(1-c.highSpeedSteerScale))*steer;
        let yawRate=-V.dot(s.velocity,fwd)/c.wheelbase*Math.tan(steerAngle); // Forward is -Z: invert yaw so left input turns left and right input turns right.
        if(handbrake>0)yawRate*=c.handbrakeYawMultiplier;
        const maxYaw=Math.max(.08,(handbrake>0?c.handbrakeGrip:c.tireGrip)*c.gravity/speedForSteer);
        yawRate=V.clamp(yawRate,-maxYaw,maxYaw);
        s.orientation=Q.normalize(Q.mul(Q.axis(normal,yawRate*dt),s.orientation));
      }else{
        const airYaw=Q.axis(V.make(0,1,0),airSteer*c.airYawRate*dt);
        s.orientation=Q.normalize(Q.mul(airYaw,s.orientation));
        const localRight=Q.rotate(s.orientation,V.make(1,0,0)),localForward=Q.rotate(s.orientation,V.make(0,0,-1));
        s.orientation=Q.normalize(Q.mul(Q.axis(localRight,((input.brake?1:0)-(input.gas||input.throttle?1:0))*c.airPitchRate*dt),s.orientation));
        s.orientation=Q.normalize(Q.mul(Q.axis(localForward,airSteer*c.airRollRate*dt),s.orientation));
      }
      s.velocity.y-=c.gravity*dt;
      const velocityMagnitude=V.len(s.velocity);
      if(velocityMagnitude>c.maxSpeed*1.35)s.velocity=V.mul(s.velocity,(c.maxSpeed*1.35)/velocityMagnitude);
      s.position=V.add(s.position,V.mul(s.velocity,dt));
      this._applyStaticCollisions();
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
      this._updateCheckpoints(this.previousState.position,s.position);
      s.tick=++this.tick;
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

  const api={PlayCarPhysicsEngine,TriangleMesh,Vector3:V,Quaternion:Q,closestPointOnTriangle,version:'1.0.3'};
  root.PlayCarPhysicsEngine=api;
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
})(typeof globalThis!=='undefined'?globalThis:this);

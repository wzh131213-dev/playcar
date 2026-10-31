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

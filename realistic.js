/* PolyTrack Realistic Mod —— 只改渲染，不碰物理与玩法。加 ?classic 可回到原版画面。 */
(function () {
  if (/[?&]classic\b/.test(location.search)) return;
  const CFG = { exposure: 0.9, sun: 3.0, hemi: 1.9, sky: 1.5, env: 1.2, fogNear: 80, fogFar: 1600 };
  window.__realMod = function (rd, sc, T) {
    try { init(rd, sc, T); } catch (e) { console.warn("[realistic] 初始化失败，已回退原版画面", e); }
  };
  // 程序化 HDR 天空（等距柱状投影），渲染器会自动做 PMREM 预过滤用于 PBR 反射
  function makeSkyEnv(T, sunDir) {
    const W = 256, H = 128, d = new Float32Array(W * H * 4);
    const mix = (a, b, t) => a + (b - a) * t, sm = (t) => t * t * (3 - 2 * t);
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
      const lon = ((i + 0.5) / W - 0.5) * 2 * Math.PI, lat = ((j + 0.5) / H - 0.5) * Math.PI;
      const x = Math.cos(lat) * Math.cos(lon), y = Math.sin(lat), z = Math.cos(lat) * Math.sin(lon);
      let r, g, b;
      if (y >= 0) { const t = sm(Math.min(1, Math.pow(y, 0.55)));
        r = mix(0.75, 0.16, t); g = mix(0.86, 0.36, t); b = mix(1.0, 0.78, t);
        const k = CFG.sky; r *= k; g *= k; b *= k;
        const s = Math.max(0, x * sunDir[0] + y * sunDir[1] + z * sunDir[2]);
        const glow = Math.pow(s, 6) * 0.5 + Math.pow(s, 220) * 40;
        r += glow * 1.0; g += glow * 0.88; b += glow * 0.7;
      } else { const t = Math.min(1, -y * 3); r = mix(0.5, 0.16, t); g = mix(0.55, 0.19, t); b = mix(0.5, 0.15, t); }
      const o = (j * W + i) * 4; d[o] = r; d[o + 1] = g; d[o + 2] = b; d[o + 3] = 1;
    }
    const tex = new T.GYF(d, W, H, 1023, 1015);   // RGBA / Float
    tex.mapping = 303; tex.magFilter = tex.minFilter = 1006; tex.needsUpdate = true; // Equirect / Linear
    return tex;
  }

  const GLSL_FN = `
varying vec3 vRWP;
varying vec3 vRWN;
float rh(vec2 p){ p=fract(p*vec2(123.34,456.21)); p+=dot(p,p+45.32); return fract(p.x*p.y); }
float rn(vec2 p){ vec2 i=floor(p),f=fract(p); f=f*f*(3.-2.*f);
  return mix(mix(rh(i),rh(i+vec2(1,0)),f.x),mix(rh(i+vec2(0,1)),rh(i+vec2(1,1)),f.x),f.y); }
float rf(vec2 p){ float a=.5,v=0.; for(int i=0;i<4;i++){ v+=a*rn(p); p*=2.03; a*=.5; } return v; }
vec3 realAlbedo(vec3 c){
  c = pow(max(c,0.), vec3(2.2));
  c += .018 * (1. - smoothstep(0., .06, dot(c, vec3(.3,.59,.11))));
  #ifdef REAL_NOISE
    vec3 fn = normalize(vRWN);
    vec2 uvp = abs(fn.y) > .7 ? vRWP.xz : (abs(fn.x) > abs(fn.z) ? vRWP.zy : vRWP.xy);   // 按法线主轴选投影面，避免竖面拉伸条纹
    float lum = dot(c, vec3(.3,.59,.11));
    if (c.g > c.r && c.g > c.b) {                       // 植被/草地
      float big = rf(uvp*.08), mid = rf(uvp*.9), fine = rn(uvp*7.);
      c *= 1.7; c = mix(c, c*vec3(1.45,1.1,.55), smoothstep(.35,.7,big));            // 草色：暗绿↔枯黄斑驳
      c *= .6 + .8*big;  c *= .82 + .36*mid;  c *= .9 + .2*fine;
      c = mix(c, vec3(.19,.16,.12)*(.6+.8*mid), smoothstep(.08,.4,1.-abs(fn.y)-.08) ); // 陡坡变土石
      c = mix(c, vec3(.09,.085,.08)*(.7+.6*big), smoothstep(.55,.85,1.-abs(fn.y)));  // 更陡变岩石
    } else if (lum < .35) {                              // 沥青/深色表面颗粒
      float g = rn(uvp*22.)*.5 + rn(uvp*90.)*.5;
      c *= 1.75; c *= .78 + .44*g;  c *= .9 + .2*rf(uvp*.6);
    }
  #endif
  return c;
}`;
  function patchShader(mat, sh) {
    if (sh.fragmentShader.indexOf("#include <color_fragment>") < 0) return;
    const noise = mat.isMeshLambertMaterial;
    let fs = sh.fragmentShader;
    const mk = ["#include <lights_physical_fragment>", "#include <lights_lambert_fragment>", "#include <lightmap_fragment>"].find(m => fs.indexOf(m) >= 0);
    if (!mk) return;
    fs = fs.replace(mk, "diffuseColor.rgb = realAlbedo(diffuseColor.rgb);\n" + mk);
    sh.vertexShader = "varying vec3 vRWP;\nvarying vec3 vRWN;\n" + sh.vertexShader.replace("#include <project_vertex>",
      "#include <project_vertex>\n vec4 rwp = vec4(transformed,1.);\n#ifdef USE_INSTANCING\n rwp = instanceMatrix * rwp;\n#endif\n vRWP = (modelMatrix * rwp).xyz;\n#ifdef USE_INSTANCING\n vRWN = normalize(mat3(modelMatrix) * mat3(instanceMatrix) * objectNormal);\n#else\n vRWN = normalize(mat3(modelMatrix) * objectNormal);\n#endif");
    sh.fragmentShader = (noise ? "#define REAL_NOISE\n" : "") + GLSL_FN + "\n" + fs;
  }
  function hookMaterials(T) {
    const P = T.imn.prototype, wrap = (fn) => {
      const w = function (sh, r) { if (fn) fn.call(this, sh, r); patchShader(this, sh); };
      w.toString = () => (fn ? fn.toString() : "") + "#real"; return w; };
    const def = wrap(null);
    Object.defineProperty(P, "onBeforeCompile", { configurable: true,
      get() { return this._rw || def; },
      set(fn) { this._rw = wrap(fn); } });
  }
  // 车辆 PBR 参数 + 地形平滑法线（只改外观数据，不碰物理）
  const PBR = { Main: [0.3, 0.35], Metal: [0.35, 0.9], Rim: [0.28, 0.85], Tire: [0.92, 0.0], AirIntake: [0.6, 0.0], BrakeLight: [0.2, 0.0] };
  function tune(o) {
    const ms = Array.isArray(o.material) ? o.material : [o.material];
    for (const m of ms) {
      if (!m || m.__real) continue; m.__real = 1;
      if (m.isMeshStandardMaterial) {
        if (m.flatShading) { m.flatShading = false; m.needsUpdate = true; }
        const p = PBR[m.name]; if (p) { m.roughness = p[0]; m.metalness = p[1]; }
        if (m.isMeshPhysicalMaterial && m.name === "Tire") { m.roughness = 0.92; }
      }
    }
    const g = o.geometry;
    if (o.name === "Body" && !g.__smoothed && g.attributes.normal) { g.__smoothed = 1; smoothNormals(g, 45); }
    if (!o.isInstancedMesh && !g.index && g.attributes.normal && g.attributes.position.count > 100 && !g.__smoothed &&
        ms[0] && ms[0].isMeshLambertMaterial) { g.__smoothed = 1; smoothNormals(g, 80); }
  }
  function smoothNormals(g, deg) {     // 按位置焊接，仅平均夹角 < deg 的法线 → 圆滑棱面、保留硬折线
    const P = g.attributes.position, N = g.attributes.normal, n = P.count, groups = new Map(), key = new Array(n), cosT = Math.cos(deg * Math.PI / 180);
    for (let i = 0; i < n; i++) {
      const k = Math.round(P.getX(i) * 500) + "," + Math.round(P.getY(i) * 500) + "," + Math.round(P.getZ(i) * 500); key[i] = k;
      (groups.get(k) || groups.set(k, []).get(k)).push(i);
    }
    const src = new Float32Array(N.array);               // 以原始面法线为基准，避免级联平均
    for (let i = 0; i < n; i++) {
      const grp = groups.get(key[i]); if (grp.length < 2) continue;
      let x = 0, y = 0, z = 0; const nx = src[i * 3], ny = src[i * 3 + 1], nz = src[i * 3 + 2];
      for (const j of grp) { const mx = src[j * 3], my = src[j * 3 + 1], mz = src[j * 3 + 2];
        if (nx * mx + ny * my + nz * mz > cosT) { x += mx; y += my; z += mz; } }
      const l = Math.hypot(x, y, z) || 1; N.setXYZ(i, x / l, y / l, z / l);
    }
    N.needsUpdate = true;
  }
  function init(rd, sc, T) {
    hookMaterials(T);
    rd.outputColorSpace = "srgb";
    rd.toneMapping = 7;                      // Khronos PBR Neutral（保留涂装色）
    rd.toneMappingExposure = CFG.exposure;
    rd.shadowMap.type = 2;                   // PCFSoft
    const v = document.createElement("div"); v.id = "real-vignette";
    v.style.cssText = "position:fixed;inset:0;pointer-events:none;z-index:0;background:radial-gradient(ellipse at 50% 45%,rgba(0,0,0,0) 55%,rgba(8,10,20,.38) 100%)";
    (rd.domElement.parentNode || document.body).insertBefore(v, rd.domElement.nextSibling);
    let frame = 0, envDone = false, sunLight = null, hemi = null;
    const render0 = rd.render.bind(rd);
    rd.render = function (s, c) {
      if (s === sc) {
        if (frame++ % 20 === 0) {
          sc.traverse(o => {
            if (o.isDirectionalLight) sunLight = o; if (o.isHemisphereLight) hemi = o;
            if (o.isMesh) tune(o);
          });
        }
        if (sunLight && !envDone) {
          const d = sunLight.position.clone().sub(sunLight.target.position).normalize();
          sc.environment = makeSkyEnv(T, [d.x, d.y, d.z]); sc.environmentIntensity = CFG.env; envDone = true;
        }
        if (sunLight) { sunLight.intensity = CFG.sun; sunLight.color.setRGB(1.0, 0.93, 0.82); }
        if (hemi) { hemi.intensity = CFG.hemi; }
        if (sc.fog && sc.fog.far > 5000) { sc.fog.near = CFG.fogNear; sc.fog.far = CFG.fogFar; }
      }
      return render0(s, c);
    };
  }
})();

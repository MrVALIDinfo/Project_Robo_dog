// Взрыв-схема: собака разлетается на отдельные детали и собирается обратно (болты выкручиваются и закручиваются).
// Детали — настоящие STL (tools/explode/make_explode.py раскладывает склеенные URDF-меши обратно на детали печати),
// крепёж — по найденным в деталях отверстиям (M3/M4/M5, гайки в шестигранных гнёздах) -> parts/explode.json.
import * as THREE from 'three';

const MM = v => v / 1000;
const ease = t => t <= 0 ? 0 : t >= 1 ? 1 : t < .5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
const clamp01 = t => Math.max(0, Math.min(1, t));

// направления разлёта в системе base_link (мм): X вперёд, Y влево, Z вверх
const BODY = {
  // порядок как при настоящей разборке: болты крышек -> крышки -> ноги -> внутренние болты и болты ног -> боковины, переборки, электроника, детали ног
  'non-mega_T_cover_cls': [[0, 0, 150], 1.0, 2.3, 'Верхняя крышка'],
  'non-mega_B_cover_cls': [[0, 0, -150], 1.0, 2.3, 'Нижняя крышка'],
  'F_cover': [[175, 0, 0], 1.1, 2.4, 'Морда'],
  'R_cover': [[-175, 0, 0], 1.1, 2.4, 'Задняя крышка ALA-TOO'],
  'non-mega_L_side_plate': [[0, 125, 0], 4.4, 5.6, null],
  'R_side_plate': [[0, -125, 0], 4.4, 5.6, 'Боковина'],
  'I_shoulder_cls': [[40, 0, 0], 4.6, 5.8, 'Переборка плеч'],
  'O_shoulder': [[95, 0, 0], 4.5, 5.7, null],
  'plate_plain': [[0, 0, 0], 0, 0, 'Плита электроники'],
};
const ATT = {     // детали внутри корпуса по userData.part
  pi: [[0, 0, 80], 'Raspberry Pi 3B'], piplat: [[0, 0, 40]], pca: [[0, 0, 80], 'PCA9685'], pcaplat: [[0, 0, 40]], plug: [[0, 0, 95]],
  cap: [[0, 0, 110]], mpu: [[0, 0, 75], 'MPU-6050'], cam: [[130, 0, 50], 'Камера Pi v2'], cammount: [[100, 0, 50]],
  lcd: [[-118, 0, 40], 'LCD 16×2'], sw: [[-215, 0, -15]], fuse: [[-95, 0, -60]], batt: [[0, 0, -100], 'LiPo 2S'],
  ubec: [[0, 80, -70], 'UBEC 5 В'], xt60: [[-50, -45, -80]], buzz: [[0, 0, 125]], mic: [[30, 0, 80]], spk: [[95, 75, 0], 'USB-динамик'],
  ads: [[40, -80, -70]],
};
const PART_NAME = {
  I_shoulder_cls: 'Переборка плеч внутренняя (I_shoulder_cls)', O_shoulder: 'Переборка плеч наружная (O_shoulder)',
 plate_plain: 'Плита электроники (plate_plain, mike4192)',
  'non-mega_T_cover_cls': 'Верхняя крышка (non-mega_T_cover_cls)', F_cover: 'Морда (F_cover)', 'non-mega_B_cover_cls': 'Нижняя крышка, отсек аккумулятора (non-mega_B_cover_cls)',
  R_cover: 'Задняя крышка ALA-TOO (R_cover_alatoo)', L_arm_cls_cable: 'Бедро левое с пазом под кабель (L_arm_cls_cable)', R_arm_cls_cable: 'Бедро правое с пазом под кабель (R_arm_cls_cable)',
  L_arm_cover: 'Крышка бедра левая (L_arm_cover)', R_arm_cover: 'Крышка бедра правая (R_arm_cover)', L_wrist_cls: 'Голень левая (L_wrist_cls)', R_wrist_cls: 'Голень правая (R_wrist_cls)',
  L_arm_joint_cls: 'Кронштейн плеча левый (L_arm_joint_cls)', R_arm_joint_cls: 'Кронштейн плеча правый (R_arm_joint_cls)',
};

export function createExplode(ctx) {
  const { robot, camera, controls, scene, view, stlLoader, hideWhileOpen } = ctx;
  const base = robot.links.base_link;
  const units = [];            // { objs:[{o, dl:Vector3 (м, в системе родителя), b:Vector3}], segs:[[vec(base, мм), a, b]], spin, label, bob }
  let data = null, ready = false, swapped = false;
  let frameN = 0, clock = null, lastLabels = [];
  let t = 0, dir = 0, speed = 1, T = 6.2, floatT = 0;
  const LEG_OUT = [2.5, 3.7], INNER = 4.4;     // ноги уходят; затем внутренние детали
  const fused = [], partMeshes = [];

  // ---------- подготовка ----------
  let prepP = null;
  function prepare() { return prepP ??= doPrepare(); }
  async function doPrepare() {
    const tp0 = performance.now();
    data = await (await fetch('parts/explode.json')).json();
    robot.updateMatrixWorld(true);
    const geos = new Map();
    await Promise.all(Object.values(data.parts).flat().map(p => new Promise(res => stlLoader.load(p.file, g => { g.computeVertexNormals(); geos.set(p.file, g); res(); }))));
    const linkRv = linkToRv();
    // 1) склеенные меши -> отдельные детали (в той же системе, что исходный меш)
    for (const [link, rv] of Object.entries(linkRv)) {
      const vis = robot.links[link]?.children.find(c => c.isURDFVisual); if (!vis) continue;
      const src = vis.children.find(c => c.isMesh); if (!src) continue;
      fused.push(src);
      for (const p of data.parts[rv]) {
        const m = new THREE.Mesh(geos.get(p.file), src.material.clone());
        m.position.copy(src.position); m.quaternion.copy(src.quaternion); m.scale.copy(src.scale);
        m.visible = false; m.userData = { ...src.userData, exPart: p.part, link, name: PART_NAME[p.part] || p.part, cat: 'print' };
        vis.add(m); partMeshes.push(m); ctx.pick.push(m);
      }
    }
    robot.updateMatrixWorld(true);
    const baseInv = base.matrixWorld.clone().invert();
    const centerBase = o => new THREE.Box3().setFromObject(o).getCenter(new THREE.Vector3()).applyMatrix4(baseInv).multiplyScalar(1000);
    const boxBase = o => {
      const b = new THREE.Box3(); o.updateMatrixWorld(true);
      o.traverse(m => { if (m.isMesh) { if (!m.geometry.boundingBox) m.geometry.computeBoundingBox(); b.union(m.geometry.boundingBox.clone().applyMatrix4(new THREE.Matrix4().multiplyMatrices(baseInv, m.matrixWorld))); } });
      b.min.multiplyScalar(1000); b.max.multiplyScalar(1000); return b;
    };

    const legOf = name => ['front_left', 'front_right', 'rear_left', 'rear_right'].find(n => name.startsWith(n));
    const owners = { body: [] }; ['front_left', 'front_right', 'rear_left', 'rear_right'].forEach(n => owners[n] = []);

    // 2) корпус
    for (const m of partMeshes) {
      const leg = legOf(m.userData.link); if (leg) continue;
      const c = centerBase(m), cfg = BODY[m.userData.exPart] || [[0, 0, 0], 0, 0, null];
      let v = cfg[0].slice(); if (m.userData.exPart.endsWith('shoulder') || m.userData.exPart.startsWith('I_shoulder')) v = [Math.sign(c.x) * v[0], 0, 0];
      if (m.userData.exPart.includes('side_plate')) { v = [0, Math.sign(c.y) * Math.abs(v[1]), 0]; m.userData.name = `Боковина ${c.y > 0 ? 'левая' : 'правая'} по ходу собаки (файл ${m.userData.exPart}.stl)`; }   // боковина уходит в ту сторону, где стоит (имя L/R в KDY не совпадает со стороной в сборке)
      const u = addUnit([m], [[v, cfg[1], cfg[2]]], { label: cfg[3] && !(m.userData.exPart === 'I_shoulder_cls' && c.x < 0) ? cfg[3] : null, bob: true });
      owners.body.push(owner(u, m));
      if (m.userData.exPart === 'non-mega_T_cover_cls') { const lg = ctx.logo(); if (lg) addUnit([lg], [[[0, 0, v[2]], cfg[1], cfg[2]]], { bob: true, phOf: u }); }
    }
    // 3) электроника и прочее внутри корпуса
    const groups = new Map();
    for (const o of base.children) {
      if (o.isURDFVisual || o.isURDFJoint || o.isURDFLink || o === ctx.logo()) continue;
      let k = null; o.traverse(m => { if (!k && m.isMesh) k = m.userData.part ?? m.userData.cat; });
      if (!k || !ATT[k]) continue;
      if (!groups.has(k)) groups.set(k, []); groups.get(k).push(o);
    }
    let i = 0;
    for (const [k, objs] of groups) {
      const a = INNER + (i++ % 8) * 0.1;
      addUnit(objs, [[ATT[k][0], a, a + 1.4]], { label: ATT[k][1], bob: true });
    }
    // 4) ноги: целиком в сторону, потом детали ноги веером
    for (const leg of ['front_left', 'front_right', 'rear_left', 'rear_right']) {
      const sx = leg.startsWith('front') ? 1 : -1, sy = leg.endsWith('left') ? 1 : -1;
      const root = robot.joints[`${leg}_shoulder`];
      addUnit([root], [[[sx * 45, sy * 185, 0], LEG_OUT[0], LEG_OUT[1]]], { bob: true });
      const sub = {
        shoulder_link: [0, 0, 0], leg_link: [0, sy * 60, 0], leg_link_cover: [0, sy * 125, 0], foot_link: [0, sy * 60, -70],
      };
      for (const [lk, v] of Object.entries(sub)) {
        const ms = partMeshes.filter(m => m.userData.link === `${leg}_${lk}`);
        const u = addUnit(ms, [[v, INNER, INNER + 1.4]], { label: leg === 'front_left' ? { leg_link: 'Бедро', leg_link_cover: 'Крышка бедра', foot_link: 'Голень', shoulder_link: 'Кронштейн плеча' }[lk] : null });
        ms.forEach(m => owners[leg].push(owner(u, m)));
      }
      const toe = robot.links[`${leg}_toe_link`].children.find(c => c.isURDFVisual);
      if (toe) addUnit([toe], [[[0, sy * 60, -135], INNER + .1, INNER + 1.5]], { label: leg === 'front_left' ? 'Стопа' : null });
      // серво: плечевое — вверх из кронштейна, бедренное — между бедром и крышкой, коленное — рядом с голенью
      for (const [lk, kind, v] of [['shoulder_link', 1, [0, 0, 70]], ['leg_link', 2, [0, sy * 92, 10]], ['foot_link', 3, [0, sy * 100, -70]]]) {
        const vis = robot.links[`${leg}_${lk}`].children.find(c => c.isURDFVisual);
        const sv = vis.children.filter(c => !c.isMesh && c.children.some(m => String(m.userData?.part).startsWith('servo_')));
        if (sv.length) addUnit(sv, [[v, INNER + .2, INNER + 1.6]], { label: leg === 'front_left' && kind === 2 ? 'Серво PDI-HV5523MG' : null });
      }
    }
    // 5) крепёж
    const fastMat = new THREE.MeshStandardMaterial({ color: 0xc4c8ce, metalness: 1, roughness: .28 });
    const anchors = { body: [['base_link', null]] };
    for (const leg of ['front_left', 'front_right', 'rear_left', 'rear_right']) {
      const s = leg.endsWith('left') ? 'l' : 'r';
      anchors[s + 'arm'] = (anchors[s + 'arm'] || []).concat([[`${leg}_leg_link`, leg]]);
      anchors[s + 'shoulder'] = (anchors[s + 'shoulder'] || []).concat([[`${leg}_shoulder_link`, leg]]);
      anchors[s + 'foot'] = (anchors[s + 'foot'] || []).concat([[`${leg}_foot_link`, leg]]);
    }
    let nFast = 0;
    for (const [g, list] of Object.entries(data.fasteners)) {
      for (const [link, leg] of anchors[g] || []) {
        const vis = robot.links[link].children.find(c => c.isURDFVisual), src = vis.children.find(c => c.isMesh);
        const frame = new THREE.Matrix4().multiplyMatrices(vis.matrixWorld, new THREE.Matrix4().compose(src.position, src.quaternion, src.scale));   // система URDF-меша (мм)
        const parent = leg ? robot.joints[`${leg}_shoulder`] : base;
        const pInv = parent.matrixWorld.clone().invert();
        for (let f of list) {
          const D = +f.size.slice(1);
          let head = new THREE.Vector3(...f.head), d = new THREE.Vector3(...f.dir).normalize();
          if (f.hexhead) { head = head.addScaledVector(d, f.span); d.negate(); f = { ...f, hexhead: false }; }   // M5 крышек: головка снаружи крышки, болт вворачивается внутрь
          const bolt = makeBolt(D, f.len, f.hexhead, fastMat), nut = f.nut ? makeNut(D, fastMat) : null;
          const place = (obj, p) => {
            const w = new THREE.Matrix4().multiplyMatrices(frame, new THREE.Matrix4().compose(p, new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), d), new THREE.Vector3(1000, 1000, 1000)));
            const l = new THREE.Matrix4().multiplyMatrices(pInv, w); l.decompose(obj.position, obj.quaternion, obj.scale);
            obj.visible = false; parent.add(obj); return obj;
          };
          place(bolt, head); if (nut) place(nut, new THREE.Vector3(...f.nut));
          for (const o of [bolt, nut]) if (o) o.traverse(m => { if (m.isMesh) { m.userData = { name: o.userData.name, cat: 'fast' }; ctx.pick.push(m); } });
          // направление «наружу» в системе base_link
          const wHead = head.clone().applyMatrix4(frame), wDir = d.clone().transformDirection(frame);
          const bHead = wHead.clone().applyMatrix4(baseInv).multiplyScalar(1000), bDir = wDir.clone().transformDirection(baseInv);
          // какие детали стягивает болт — посчитано заранее по геометрии (tools/explode/check_fasteners.py --apply)
          const byName = nm => { const c = owners[leg || 'body'].filter(o => o.name === nm); return c.length ? c.reduce((p, q) => q.box.distanceToPoint(bHead) < p.box.distanceToPoint(bHead) ? q : p) : null; };
          const J = (f.joins || []).map(byName).filter(Boolean).map(o => ({ o }));
          const moving = J.filter(j => j.o.u.segs.some(sg => sg[2] > sg[1] && (sg[0][0] || sg[0][1] || sg[0][2])));
          const sep = moving.length ? Math.min(...moving.map(j => j.o.u.segs[0][1])) : INNER;            // когда эти детали разъединяются
          let a = sep - 0.85 + Math.random() * 0.15, b = a + 0.65;                                      // выкрутить прямо перед этим
          if (leg) { a = Math.max(a, LEG_OUT[1] + 0.02 + Math.random() * 0.1); b = Math.min(INNER - 0.03, a + 0.55); }   // болты ноги — когда нога уже отъехала
          const headU = byName(f.headPart)?.u ?? J[0]?.o.u ?? null;                                     // деталь со стороны головки
          const nutU = byName(f.nutPart)?.u ?? null;                                                    // деталь со стороны гайки
          const out = bDir.clone().multiplyScalar(-(f.len + 12));                                       // болт выходит целиком, головкой наружу
          const bu = addUnit([bolt], [[out.toArray(), a, b], ...(headU ? headU.segs : [])], { spin: { o: bolt.children[0], turns: f.len / 2.5, a, b }, fast: true, bob: headU?.bob, phOf: headU, label: nFast === 0 ? 'Крепёж: болты M3/M4/M5, гайки' : null });
          if (nut) addUnit([nut], [[bDir.clone().multiplyScalar(14).toArray(), a, b], ...((nutU || headU) ? (nutU || headU).segs : [])], { spin: { o: nut.children[0], turns: -2, a, b }, fast: true, bob: (nutU || headU)?.bob, phOf: nutU || headU });
          nFast++;
        }
      }
    }
    ctx.fastenerCount = nFast;
    // смещения: из системы base_link (мм) в систему родителя каждого объекта
    for (const u of units) for (const it of u.objs) {
      const p = it.o.parent; p.updateMatrixWorld(true);
      const M = new THREE.Matrix4().multiplyMatrices(p.matrixWorld.clone().invert(), base.matrixWorld);
      const lin = new THREE.Matrix3().setFromMatrix4(M);
      it.lin = lin;
      it.b = it.o.matrixAutoUpdate ? it.o.position.clone() : new THREE.Vector3().setFromMatrixPosition(it.o.matrix);
    }
    units.forEach(u => u.ph = Math.random() * Math.PI * 2);
    units.forEach(u => { if (u.phOf) u.ph = u.phOf.ph; });         // болты и логотип покачиваются вместе со своей деталью
    buildLabels();
    ready = true; (window.__perf ??= {}).explode = Math.round(performance.now() - tp0);
  }

  // деталь-владелец: имя детали печати и габарит в системе base_link (мм) — чтобы различать две одинаковые переборки
  function owner(u, m) {
    m.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(m).applyMatrix4(base.matrixWorld.clone().invert());
    box.min.multiplyScalar(1000); box.max.multiplyScalar(1000);
    return { u, box, name: m.userData.exPart };
  }
  function nearest(list, p) {
    let best = null, bd = 1e9;
    for (const it of list) { const d = it.box.distanceToPoint(p); if (d < bd) { bd = d; best = it; } }
    return best;
  }
  function addUnit(objs, segs, o = {}) { const u = { objs: objs.map(x => ({ o: x })), segs, ...o }; units.push(u); return u; }
  function linkToRv() {
    const r = { base_link: 'mainbody', front_link: 'frontpart', rear_link: 'backpart' };
    for (const leg of ['front_left', 'front_right', 'rear_left', 'rear_right']) {
      const s = leg.endsWith('left') ? 'l' : 'r';
      r[`${leg}_shoulder_link`] = s + 'shoulder'; r[`${leg}_leg_link`] = s + 'arm'; r[`${leg}_leg_link_cover`] = s + 'arm_cover'; r[`${leg}_foot_link`] = s + 'foot';
    }
    return r;
  }

  // ---------- крепёж: винт с внутренним шестигранником (или с шестигранной головкой) и гайка ----------
  // одна геометрия на типоразмер (кэш): профиль вращения — головка с углублением под ключ и стержень с витками резьбы
  const geoCache = new Map();
  function boltGeo(D, L, hexhead) {
    const key = `${D}|${L}|${hexhead}`; if (geoCache.has(key)) return geoCache.get(key);
    const r = D / 2, p = D * 0.18, pts = [];
    if (hexhead) pts.push([0, 0]);
    else { const hd = D * 1.6, h = D, s = D * .43; pts.push([0, -h + h * .5], [s, -h + h * .5], [s, -h], [hd / 2 - .3, -h], [hd / 2, -h + .3], [hd / 2, -.2], [hd / 2 - .2, 0]); }
    pts.push([r * .97, 0]);
    const t0 = Math.min(L * .3, 3);
    pts.push([r * .97, t0]);
    for (let y = t0; y < L - p; y += p) pts.push([r * .86, y + p * .5], [r * .99, y + p]);
    pts.push([r * .8, L], [0, L]);
    const g = new THREE.LatheGeometry(pts.map(([x, y]) => new THREE.Vector2(MM(x), MM(y))), 18);
    g.computeVertexNormals(); geoCache.set(key, g); return g;
  }
  function makeBolt(D, L, hexhead, mat) {
    const outer = new THREE.Group(), g = new THREE.Group(); outer.add(g);
    g.add(new THREE.Mesh(boltGeo(D, L, hexhead), mat));
    if (hexhead) {
      const key = `hex${D}`;
      if (!geoCache.has(key)) { const s = { 3: 5.5, 4: 7, 5: 8 }[D], h = D * .7, hg = new THREE.CylinderGeometry(MM(s / Math.sqrt(3)), MM(s / Math.sqrt(3)), MM(h), 6); hg.translate(0, MM(-h / 2), 0); geoCache.set(key, hg); }
      g.add(new THREE.Mesh(geoCache.get(key), mat));
    }
    outer.userData = { name: `Болт M${D}×${L}`, cat: 'fast' };
    return outer;
  }
  function makeNut(D, mat) {
    const outer = new THREE.Group(), g = new THREE.Group(); outer.add(g);
    const key = `nut${D}`;
    if (geoCache.has(key)) { g.add(new THREE.Mesh(geoCache.get(key), mat)); outer.userData = { name: `Гайка M${D}`, cat: 'fast' }; return outer; }
    const s = { 3: 5.5, 4: 7, 5: 8 }[D], h = D * .8;
    const shape = new THREE.Shape(); for (let k = 0; k < 6; k++) { const a = k * Math.PI / 3 + Math.PI / 6, R = s / Math.sqrt(3); k ? shape.lineTo(R * Math.cos(a), R * Math.sin(a)) : shape.moveTo(R * Math.cos(a), R * Math.sin(a)); }
    shape.closePath(); const hole = new THREE.Path(); hole.absarc(0, 0, D / 2, 0, Math.PI * 2, true); shape.holes.push(hole);
    const geo = new THREE.ExtrudeGeometry(shape, { depth: h, bevelEnabled: true, bevelSize: .25, bevelThickness: .25, bevelSegments: 1, curveSegments: 16 });
    geo.translate(0, 0, -h / 2); geo.rotateX(-Math.PI / 2); geo.scale(.001, .001, .001); geoCache.set(key, geo);
    g.add(new THREE.Mesh(geo, mat));
    outer.userData = { name: `Гайка M${D}`, cat: 'fast' };
    return outer;
  }

  // ---------- подписи-выноски ----------
  let labelLayer = null, svg = null; const labels = [];
  function buildLabels() {
    labelLayer = document.createElement('div');
    labelLayer.style.cssText = 'position:absolute;inset:0;pointer-events:none;overflow:hidden;opacity:0;transition:opacity .5s';
    svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg'); svg.style.cssText = 'position:absolute;inset:0;width:100%;height:100%';
    labelLayer.appendChild(svg); view.appendChild(labelLayer);
    for (const u of units) if (u.label) {
      const el = document.createElement('div');
      el.textContent = u.label;
      el.style.cssText = 'position:absolute;color:#e9ebef;font:500 12px system-ui,sans-serif;letter-spacing:.02em;white-space:nowrap;text-shadow:0 1px 2px #000a';
      const ln = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
      ln.setAttribute('fill', 'none'); ln.setAttribute('stroke', '#c9ccd2'); ln.setAttribute('stroke-width', '1'); ln.setAttribute('opacity', '.8');
      svg.appendChild(ln); labelLayer.appendChild(el); labels.push({ u, el, ln });
    }
  }
  const tmp = new THREE.Vector3(), box = new THREE.Box3(); let scrBox = null;
  function visBox(u) {
    box.makeEmpty();
    for (const it of u.objs) it.o.traverse(m => { if (m.isMesh && m.visible !== false && !m.isLine) { let v = true, q = m; while (q) { if (q.visible === false) { v = false; break; } q = q.parent; } if (v) box.expandByObject(m); } });
    return box;
  }
  function updateLabels() {
    const show = ready && t / T > 0.97; lastLabels = [];
    labelLayer.style.opacity = show && ctx.labels !== false ? 1 : 0;
    if (!show) return;
    const W = view.clientWidth, H = view.clientHeight, pts = [];
    for (const L of labels) {
      if (visBox(L.u).isEmpty()) { L.el.style.display = 'none'; L.ln.setAttribute('points', ''); continue; }
      box.getCenter(tmp).project(camera);
      pts.push({ L, x: (tmp.x * .5 + .5) * W, y: (-tmp.y * .5 + .5) * H });
    }
    const cx = pts.reduce((a, p) => a + p.x, 0) / Math.max(1, pts.length);
    let sx0 = 1e9, sx1 = -1e9;                                    // края модели на экране — подписи снаружи
    if (!scrBox) scrBox = explodedBounds();
    for (let k = 0; k < 8; k++) { tmp.set(k & 1 ? scrBox.max.x : scrBox.min.x, k & 2 ? scrBox.max.y : scrBox.min.y, k & 4 ? scrBox.max.z : scrBox.min.z).project(camera); const x = (tmp.x * .5 + .5) * W; sx0 = Math.min(sx0, x); sx1 = Math.max(sx1, x); }
    sx0 = Math.max(sx0, 0); sx1 = Math.min(sx1, W);
    for (const side of [-1, 1]) {
      const col = pts.filter(p => (p.x < cx ? -1 : 1) === side).sort((a, b) => a.y - b.y);
      const gap = 24, top = 40, bottom = H - 80;
      let y = top;
      col.forEach(p => { p.ly = Math.max(y, Math.min(p.y, bottom)); y = p.ly + gap; });
      for (let k = col.length - 1, lim = bottom; k >= 0; k--) { col[k].ly = Math.min(col[k].ly, lim); lim = col[k].ly - gap; }
      const wmax = Math.max(0, ...col.map(p => p.L.el.offsetWidth || 120));
      const colX = side < 0 ? Math.max(wmax + 16, sx0 - 30) : Math.min(W - wmax - 16, sx1 + 30);
      for (const p of col) {
        const ex = colX;
        p.L.ln.setAttribute('points', `${p.x},${p.y} ${ex - side * 14},${p.ly} ${ex},${p.ly}`);
        lastLabels.push({ text: p.L.el.textContent, x: p.x, y: p.y, ex, ly: p.ly, side });
        p.L.el.style.display = 'block';
        p.L.el.style.left = (side > 0 ? ex + 6 : ex - 6 - p.L.el.offsetWidth) + 'px'; p.L.el.style.top = (p.ly - 9) + 'px';
      }
    }
  }
  // кадр под разобранную модель: центр и радиус по всем деталям в конце разлёта
  function explodedBounds() {
    const was = swapped; setParts(true);
    const save = t; t = T; apply(); const b = new THREE.Box3();
    for (const u of units) if (!u.fast) b.union(visBox(u));
    t = save; apply(); if (!was) setParts(false); return b;
  }

  // ---------- анимация ----------
  function apply() {
    const tt = t, fl = Math.min(1, floatT);
    for (const u of units) {
      const v = new THREE.Vector3();
      for (const [vec, a, b] of u.segs) { const e = b > a ? ease(clamp01((tt - a) / (b - a))) : 0; v.x += vec[0] * e; v.y += vec[1] * e; v.z += vec[2] * e; }
      if (u.bob && fl > 0) { const s = clock ?? performance.now() / 1000; v.z += Math.sin(s * 1.1 + u.ph) * 2.2 * fl; v.y += Math.cos(s * .8 + u.ph) * 1.2 * fl; }
      v.multiplyScalar(0.001);
      for (const it of u.objs) {
        const d = v.clone().applyMatrix3(it.lin).add(it.b);
        if (it.o.matrixAutoUpdate) it.o.position.copy(d); else it.o.matrix.setPosition(d);
      }
      if (u.spin) { const e = ease(clamp01((tt - u.spin.a) / (u.spin.b - u.spin.a))); u.spin.o.rotation.y = e * u.spin.turns * Math.PI * 2; }
    }
  }
  function setParts(on) {
    if (on === swapped) return; swapped = on;
    fused.forEach(m => m.visible = !on); partMeshes.forEach(m => m.visible = on);
    for (const u of units) if (u.fast) u.objs.forEach(it => it.o.visible = on);
    hideWhileOpen(on);
  }
  function update(dt) {
    if (!ready) return;
    if (dir) {
      t = Math.max(0, Math.min(T, t + dir * dt * speed));
      if ((dir > 0 && t >= T) || (dir < 0 && t <= 0)) { if (dir < 0) { apply(); setParts(false); ctx.onState?.('assembled'); } else ctx.onState?.('exploded'); dir = 0; }
    }
    floatT = t >= T ? floatT + dt * .5 : Math.max(0, floatT - dt * 2);
    if (swapped) apply();
    if ((frameN++ & 1) === 0) updateLabels();
  }
  return {
    get busy() { return dir !== 0; }, get progress() { return t / T; }, get ready() { return ready; },
    async open() { await prepare(); setParts(true); dir = 1; ctx.onState?.('opening'); },
    async close() { await prepare(); if (!swapped) return; dir = -1; ctx.onState?.('closing'); },
    frameAt(tt, clk, fl) { setParts(true); t = Math.max(0, Math.min(T, tt)); clock = clk; floatT = fl; apply(); updateLabels(); return lastLabels; },
    get T() { return T; }, setAssembled() { t = 0; apply(); setParts(false); },
    async seek(f) { await prepare(); setParts(true); t = f * T; dir = 0; apply(); updateLabels(); ctx.onState?.(f >= 1 ? 'exploded' : 'opening'); },
    explodedBounds() { return explodedBounds(); },
    setSpeed(s) { speed = s; }, labelsOn(v) { ctx.labels = v; }, update, prepare,
  };
}

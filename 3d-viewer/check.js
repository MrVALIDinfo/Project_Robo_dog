// ---- проверка стыковки: детали (OBB) против корпуса (треугольники) и друг против друга, провода против стенок ----
// Всё в системе base_link, в миллиметрах.
import * as THREE from 'three';

// треугольники корпуса в мм + AABB каждого треугольника
export function bodyTriangles(meshes, baseInv) {
  const out = [];
  const v = new THREE.Vector3(), M = new THREE.Matrix4();
  for (const m of meshes) {
    M.multiplyMatrices(baseInv, m.matrixWorld);
    const p = m.geometry.attributes.position, idx = m.geometry.index;
    const n = idx ? idx.count : p.count;
    for (let i = 0; i < n; i++) { v.fromBufferAttribute(p, idx ? idx.getX(i) : i).applyMatrix4(M); out.push(v.x * 1000, v.y * 1000, v.z * 1000); }
  }
  const t = new Float32Array(out), nt = t.length / 9, bb = new Float32Array(nt * 6);
  for (let i = 0; i < nt; i++) {
    const o = i * 9;
    for (let k = 0; k < 3; k++) { bb[i * 6 + k] = Math.min(t[o + k], t[o + 3 + k], t[o + 6 + k]); bb[i * 6 + 3 + k] = Math.max(t[o + k], t[o + 3 + k], t[o + 6 + k]); }
  }
  return { t, bb, n: nt };
}

// ориентированный бокс меша по его boundingBox: центр, 3 оси, полуразмеры (мм)
export function obbOf(mesh, baseInv) {
  const g = mesh.geometry; if (!g.boundingBox) g.computeBoundingBox();
  const M = new THREE.Matrix4().multiplyMatrices(baseInv, mesh.matrixWorld);
  const c = g.boundingBox.getCenter(new THREE.Vector3()).applyMatrix4(M).multiplyScalar(1000);
  const s = g.boundingBox.getSize(new THREE.Vector3());
  const ax = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]; M.extractBasis(ax[0], ax[1], ax[2]);
  const h = [0, 1, 2].map(i => { const L = ax[i].length(); ax[i].divideScalar(L); return s.getComponent(i) / 2 * L * 1000; });
  return { c, ax, h };
}
function obbAabb(o) {
  const lo = [0, 0, 0], hi = [0, 0, 0];
  for (let k = 0; k < 3; k++) { const r = o.h[0] * Math.abs(o.ax[0].getComponent(k)) + o.h[1] * Math.abs(o.ax[1].getComponent(k)) + o.h[2] * Math.abs(o.ax[2].getComponent(k)); lo[k] = o.c.getComponent(k) - r; hi[k] = o.c.getComponent(k) + r; }
  return [lo, hi];
}

// треугольник против AABB с полуразмерами h в начале координат (Akenine-Möller, 13 осей)
function triBox(h, a, b, c) {
  for (let k = 0; k < 3; k++) if (Math.min(a[k], b[k], c[k]) > h[k] || Math.max(a[k], b[k], c[k]) < -h[k]) return false;
  const e = [[b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - b[0], c[1] - b[1], c[2] - b[2]], [a[0] - c[0], a[1] - c[1], a[2] - c[2]]];
  const nrm = [e[0][1] * e[1][2] - e[0][2] * e[1][1], e[0][2] * e[1][0] - e[0][0] * e[1][2], e[0][0] * e[1][1] - e[0][1] * e[1][0]];
  const d = nrm[0] * a[0] + nrm[1] * a[1] + nrm[2] * a[2], r = h[0] * Math.abs(nrm[0]) + h[1] * Math.abs(nrm[1]) + h[2] * Math.abs(nrm[2]);
  if (Math.abs(d) > r) return false;
  const P = [a, b, c];
  for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) {
    const ax = [0, 0, 0]; ax[(k + 1) % 3] = -e[i][(k + 2) % 3]; ax[(k + 2) % 3] = e[i][(k + 1) % 3];   // единичный вектор k × ребро
    if (!ax[0] && !ax[1] && !ax[2]) continue;
    let mn = Infinity, mx = -Infinity;
    for (const p of P) { const q = p[0] * ax[0] + p[1] * ax[1] + p[2] * ax[2]; mn = Math.min(mn, q); mx = Math.max(mx, q); }
    const rr = h[0] * Math.abs(ax[0]) + h[1] * Math.abs(ax[1]) + h[2] * Math.abs(ax[2]);
    if (mn > rr || mx < -rr) return false;
  }
  return true;
}

// пересекается ли OBB (раздутый на pad мм) с корпусом; возвращает число задетых треугольников
export function obbVsBody(o, body, pad = 0) {
  const [lo, hi] = obbAabb(o), h = o.h.map(x => x + pad);
  const { t, bb, n } = body; let hits = 0;
  const loc = (i) => { const d = [t[i] - o.c.x, t[i + 1] - o.c.y, t[i + 2] - o.c.z]; return [0, 1, 2].map(k => d[0] * o.ax[k].x + d[1] * o.ax[k].y + d[2] * o.ax[k].z); };
  for (let i = 0; i < n; i++) {
    const j = i * 6;
    if (bb[j] > hi[0] + pad || bb[j + 3] < lo[0] - pad || bb[j + 1] > hi[1] + pad || bb[j + 4] < lo[1] - pad || bb[j + 2] > hi[2] + pad || bb[j + 5] < lo[2] - pad) continue;
    if (triBox(h, loc(i * 9), loc(i * 9 + 3), loc(i * 9 + 6))) hits++;
  }
  return hits;
}

// OBB против OBB (15 осей); eps > 0 — допуск на касание
export function obbObb(A, B, eps = 0) {
  const axes = [...A.ax, ...B.ax];
  for (const a of A.ax) for (const b of B.ax) { const x = new THREE.Vector3().crossVectors(a, b); if (x.lengthSq() > 1e-9) axes.push(x.normalize()); }
  const d = new THREE.Vector3().subVectors(B.c, A.c);
  for (const L of axes) {
    const ra = A.h.reduce((s, h, i) => s + h * Math.abs(A.ax[i].dot(L)), 0), rb = B.h.reduce((s, h, i) => s + h * Math.abs(B.ax[i].dot(L)), 0);
    if (Math.abs(d.dot(L)) > ra + rb - eps) return false;
  }
  return true;
}

// сетка 8 мм над треугольниками: отрезку проверяются только треугольники его ячеек
function gridOf(body, cs = 8) {
  if (body.grid) return body.grid;
  const { bb, n } = body, cells = new Map(), key = (i, j, k) => (i * 73856093) ^ (j * 19349663) ^ (k * 83492791);
  for (let t = 0; t < n; t++) {
    const j = t * 6, lo = [0, 1, 2].map(a => Math.floor(bb[j + a] / cs)), hi = [0, 1, 2].map(a => Math.floor(bb[j + 3 + a] / cs));
    for (let a = lo[0]; a <= hi[0]; a++) for (let b = lo[1]; b <= hi[1]; b++) for (let c = lo[2]; c <= hi[2]; c++) {
      const k = key(a, b, c); let l = cells.get(k); if (!l) cells.set(k, l = []); l.push(t);
    }
  }
  return (body.grid = { cells, cs, key });
}

// отрезок против корпуса (Möller–Trumbore), мм
export function segVsBody(p, q, body) {
  const { t, bb } = body, { cells, cs, key } = gridOf(body);
  const lo = [Math.min(p[0], q[0]), Math.min(p[1], q[1]), Math.min(p[2], q[2])], hi = [Math.max(p[0], q[0]), Math.max(p[1], q[1]), Math.max(p[2], q[2])];
  const dir = [q[0] - p[0], q[1] - p[1], q[2] - p[2]], seen = new Set();
  const c0 = lo.map(v => Math.floor(v / cs)), c1 = hi.map(v => Math.floor(v / cs));
  for (let a = c0[0]; a <= c1[0]; a++) for (let b = c0[1]; b <= c1[1]; b++) for (let c = c0[2]; c <= c1[2]; c++) {
    const l = cells.get(key(a, b, c)); if (!l) continue;
    for (const i of l) {
      if (seen.has(i)) continue; seen.add(i);
      const j = i * 6;
      if (bb[j] > hi[0] || bb[j + 3] < lo[0] || bb[j + 1] > hi[1] || bb[j + 4] < lo[1] || bb[j + 2] > hi[2] || bb[j + 5] < lo[2]) continue;
      const o = i * 9, e1 = [t[o + 3] - t[o], t[o + 4] - t[o + 1], t[o + 5] - t[o + 2]], e2 = [t[o + 6] - t[o], t[o + 7] - t[o + 1], t[o + 8] - t[o + 2]];
      const pv = [dir[1] * e2[2] - dir[2] * e2[1], dir[2] * e2[0] - dir[0] * e2[2], dir[0] * e2[1] - dir[1] * e2[0]];
      const det = e1[0] * pv[0] + e1[1] * pv[1] + e1[2] * pv[2]; if (Math.abs(det) < 1e-12) continue;
      const tv = [p[0] - t[o], p[1] - t[o + 1], p[2] - t[o + 2]], u = (tv[0] * pv[0] + tv[1] * pv[1] + tv[2] * pv[2]) / det; if (u < 0 || u > 1) continue;
      const qv = [tv[1] * e1[2] - tv[2] * e1[1], tv[2] * e1[0] - tv[0] * e1[2], tv[0] * e1[1] - tv[1] * e1[0]];
      const v = (dir[0] * qv[0] + dir[1] * qv[1] + dir[2] * qv[2]) / det; if (v < 0 || u + v > 1) continue;
      const s = (e2[0] * qv[0] + e2[1] * qv[1] + e2[2] * qv[2]) / det; if (s >= 0 && s <= 1) return true;
    }
  }
  return false;
}

// точка внутри замкнутого меша: чётность пересечений луча +Z (мм)
export function insideBody(p, body) {
  const { t, bb, n } = body; let c = 0;
  for (let i = 0; i < n; i++) {
    const j = i * 6;
    if (bb[j] > p[0] || bb[j + 3] < p[0] || bb[j + 1] > p[1] || bb[j + 4] < p[1] || bb[j + 5] < p[2]) continue;
    const o = i * 9, ax = t[o], ay = t[o + 1], bx = t[o + 3], by = t[o + 4], cx = t[o + 6], cy = t[o + 7];
    const d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy); if (Math.abs(d) < 1e-12) continue;
    const l1 = ((by - cy) * (p[0] - cx) + (cx - bx) * (p[1] - cy)) / d, l2 = ((cy - ay) * (p[0] - cx) + (ax - cx) * (p[1] - cy)) / d, l3 = 1 - l1 - l2;
    if (l1 < 0 || l2 < 0 || l3 < 0) continue;
    if (l1 * t[o + 2] + l2 * t[o + 5] + l3 * t[o + 8] > p[2]) c++;
  }
  return c % 2 === 1;
}

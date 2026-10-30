import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { SVGLoader } from 'three/examples/jsm/loaders/SVGLoader.js';
import URDFLoader from './lib/URDFLoader.js';
import { bodyTriangles, obbOf, obbVsBody, obbObb, segVsBody, insideBody } from './check.js';

const view = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14161a);
const camera = new THREE.PerspectiveCamera(40, 1, 0.03, 20);
camera.position.set(0.52, 0.34, 0.66);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.13, 0); controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x333840, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(0.6, 1.2, 0.8); scene.add(sun);
const back = new THREE.DirectionalLight(0x88aaff, 0.5); back.position.set(-0.8, 0.4, -0.6); scene.add(back);
const grid = new THREE.GridHelper(6, 120, 0x3a3f4a, 0x262a31); scene.add(grid);

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
}
addEventListener('resize', resize); resize();

// ---- модель ----
const manager = new THREE.LoadingManager();
const loader = new URDFLoader(manager);
const rig = new THREE.Group(); scene.add(rig);
let robot, shell = [], body = [], layout = null, seats = null, routes = null;
const layoutReady = Promise.all([fetch('parts/layout.json').then(r => r.json()).then(j => layout = j),
  fetch('parts/servo_seats.json').then(r => r.json()).then(j => seats = j),
  fetch('parts/cable_routes.json').then(r => r.json()).then(j => routes = j)]);

loader.load('spot_micro.urdf', r => {
  robot = r;
  robot.rotation.x = -Math.PI / 2;           // URDF: Z вверх -> three: Y вверх
  rig.add(robot);
});
manager.onLoad = async () => {
  if (!robot || shell.length) return;
  robot.traverse(o => { if (o.isMesh) { o.material = new THREE.MeshStandardMaterial({ color: o.material?.color?.getHex?.() ?? 0x222428, roughness: .55, metalness: .1 }); shell.push(o); } });
  body = shell.filter(o => ['base_link', 'rear_link', 'front_link'].includes(linkOf(o)));   // корпус без ног и лидара
  buildJointsUI();
  await layoutReady;
  addElectronics();
  addLogo();
  addRearText();
  applyPose(POSES.stand); groundY = rig.position.y;
  robot.links.lidar_link.traverse(m => { if (m.isMesh) m.visible = false; });
  fit();
  btn('bWalk').onclick();
  applyQuery();
  document.getElementById('load')?.remove();
};
manager.onError = u => console.error('не загрузилось', u);
function linkOf(o) { while (o && !o.isURDFLink) o = o.parent; return o?.urdfName; }

// ===== детали =====
// Всё в системе base_link, мм: X вперёд, Y влево, Z вверх. Плита — гладкая plate_plain mike4192, верх z = -7.1, x -47…48, y ±52.
const stlLoader = new STLLoader();
const pick = [];              // наведение мышью
const solids = [];            // проверка стыковки
const MM = v => v / 1000;
const edgeMat = new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: .35 });
function tagPart(root, name, part, cat = part, extra = {}) {
  root.traverse(o => { if (o.isMesh) { Object.assign(o.userData, { name, part, cat, ...extra }); pick.push(o); if (!o.userData.nocheck) solids.push(o); } });
}
let pending = 0;
function loadPart(file, color, name, part, parent, pos, extra = {}, ud = {}) {
  pending++;
  stlLoader.load('parts/' + file, g => {
    g.scale(0.001, 0.001, 0.001);
    const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color, roughness: .55, metalness: .25, ...extra }));
    m.position.set(MM(pos[0]), MM(pos[1]), MM(pos[2]));
    Object.assign(m.userData, ud); parent.add(m); tagPart(m, name, part);
    if (!--pending) onPartsLoaded();
  });
}
// печатная площадка mike4192: в файле Y — вверх, X×Z — 62–65×25–56 мм; ставится на плиту (o — угол, z — верх плиты)
function platform(file, name, o, part) {
  pending++;
  stlLoader.load('parts/' + file, g => {
    g.scale(0.001, 0.001, 0.001);
    const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0x3a3d44, roughness: .7, metalness: .05 }));
    m.matrix.makeBasis(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, -1, 0)).setPosition(MM(o[0]), MM(o[1]), MM(o[2]));
    m.matrixAutoUpdate = false; Object.assign(m.userData, { touch: ['pi', 'pca', 'mpu', 'cap'], mount: true });   // стоит на плите — касание плиты не в счёт
    robot.links.base_link.add(m); tagPart(m, name, part, 'plat');
    if (!--pending) onPartsLoaded();
  });
}
// бокс по двум углам (мм, в координатах родителя)
function bx(parent, name, part, lo, hi, color, o = {}) {
  const s = [0, 1, 2].map(i => hi[i] - lo[i]);
  const m = new THREE.Mesh(new THREE.BoxGeometry(MM(s[0]), MM(s[1]), MM(s[2])), new THREE.MeshStandardMaterial({ color, roughness: o.rough ?? .55, metalness: o.metal ?? .1 }));
  m.position.set(MM((lo[0] + hi[0]) / 2), MM((lo[1] + hi[1]) / 2), MM((lo[2] + hi[2]) / 2));
  if (o.edges !== false) m.add(new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), edgeMat));
  if (o.check === false) m.userData.nocheck = true;
  parent.add(m); tagPart(m, name, part, o.cat ?? part, o.ud); return m;
}
// цилиндр: центр c, радиус r, длина h вдоль оси axis
function cy(parent, name, part, c, r, h, axis, color, o = {}) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(MM(r), MM(r), MM(h), o.seg ?? 24), new THREE.MeshStandardMaterial({ color, roughness: o.rough ?? .45, metalness: o.metal ?? .2 }));
  if (axis === 'x') m.rotation.z = Math.PI / 2; if (axis === 'z') m.rotation.x = Math.PI / 2;
  m.position.set(MM(c[0]), MM(c[1]), MM(c[2]));
  if (o.check === false) m.userData.nocheck = true;
  parent.add(m); tagPart(m, name, part, o.cat ?? part, o.ud); return m;
}
function group(parent, pos = [0, 0, 0]) { const g = new THREE.Group(); g.position.set(MM(pos[0]), MM(pos[1]), MM(pos[2])); parent.add(g); return g; }

// ---- серво PDI-HV5523MG: 40.5 x 20.2 x 38 мм, ушки 54.5 мм, вал 25T на 10.1 мм от торца ----
// начало координат — ось вала на верхней крышке корпуса, вал +Z, корпус вниз, длинная сторона по X (корпус в сторону -X)
function makeServo(name, part) {
  const g = new THREE.Group();
  const dark = new THREE.MeshStandardMaterial({ color: 0x23262d, roughness: .6, metalness: .1 });
  const steel = new THREE.MeshStandardMaterial({ color: 0xc9ccd2, roughness: .35, metalness: .8 });
  const lab = new THREE.MeshStandardMaterial({ color: 0xd9b000, roughness: .7 });
  const add = (geo, mat, x, y, z, rx = 0) => { const m = new THREE.Mesh(geo, mat); m.position.set(MM(x), MM(y), MM(z)); m.rotation.x = rx; g.add(m); return m; };
  add(new THREE.BoxGeometry(MM(40.5), MM(20.2), MM(38)), dark, -10.15, 0, -19);
  add(new THREE.BoxGeometry(MM(54.5), MM(20.2), MM(2.5)), dark, -10.15, 0, -10.25);
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) add(new THREE.CylinderGeometry(MM(2.3), MM(2.3), MM(2.7), 16), steel, -10.15 + sx * 24.75, sy * 5, -10.25, Math.PI / 2);
  add(new THREE.CylinderGeometry(MM(7), MM(7), MM(3), 32), dark, 0, 0, 1.5, Math.PI / 2).userData.horn = true;
  add(new THREE.CylinderGeometry(MM(3), MM(3), MM(4), 24), steel, 0, 0, 5, Math.PI / 2).userData.horn = true;
  add(new THREE.BoxGeometry(MM(26), MM(0.3), MM(14)), lab, -10.15, 10.2, -22);
  add(new THREE.BoxGeometry(MM(26), MM(0.3), MM(14)), lab, -10.15, -10.2, -22);
  g.traverse(o => { if (o.isMesh) o.add(new THREE.LineSegments(new THREE.EdgesGeometry(o.geometry, 30), edgeMat)); });
  tagPart(g, name, part, 'servo');
  return g;
}
// поставить серво в гнездо: seat — в координатах STL-файла звена (мм): o — вал на верхе корпуса, z — вал, x — от корпуса к валу
function seatServo(link, seat, name, part) {
  const s = makeServo(name, part), vis = robot.links[link].children.find(c => c.isURDFVisual);
  const X = new THREE.Vector3(...seat.x), Z = new THREE.Vector3(...seat.z), Y = new THREE.Vector3().crossVectors(Z, X);
  s.matrix.makeBasis(X, Y, Z).scale(new THREE.Vector3(1000, 1000, 1000)).setPosition(...seat.o);   // визуал звена в мм (scale 0.001)
  s.matrixAutoUpdate = false; vis.add(s); return s;
}

// ---- логотип Ala-Too IT & Business College на верхней крышке: синий диск Ø40 мм 1 мм, красное +0.6 мм, белое +1 мм ----
// logo.svg собирается из docs/logo_source.png скриптом tools/logo/make_logo.py
let logoGroup = null;
async function addLogo() {
  const svgText = await (await fetch('logo.svg', { cache: 'no-cache' })).text();
  const data = new SVGLoader().parse(svgText);
  const S = +data.xml.getAttribute('viewBox').split(/\s+/)[2], K = 40 / S;   // диаметр круга в SVG = 40 мм
  const g = new THREE.Group();
  const mat = c => new THREE.MeshStandardMaterial({ color: c, roughness: .45, metalness: .05 });
  const disk = new THREE.Mesh(new THREE.CylinderGeometry(MM(20), MM(20), MM(1), 96), mat(0x11225f));
  disk.rotation.x = Math.PI / 2; disk.position.z = MM(0.5); g.add(disk);
  const flip = new THREE.Group(); flip.scale.y = -1; g.add(flip);   // ось Y в SVG направлена вниз
  for (const [cls, color, h] of [['red', 0xcc2030, 0.6], ['relief', 0xf4f4f2, 1]]) {
    const shapes = data.paths.filter(q => q.userData?.node?.getAttribute('class') === cls).flatMap(p => SVGLoader.createShapes(p));
    if (!shapes.length) continue;
    const geo = new THREE.ExtrudeGeometry(shapes, { depth: h, bevelEnabled: false, curveSegments: 4 });
    geo.translate(-S / 2, -S / 2, 0); geo.scale(MM(K), MM(K), MM(1));
    const m = new THREE.Mesh(geo, mat(color)); m.position.z = MM(1); flip.add(m);
  }
  g.rotation.z = -Math.PI / 2;                       // верх логотипа к носу собаки (+X)
  const holder = new THREE.Group(); holder.add(g);
  holder.position.set(0.010, 0, 0.036);              // плоский участок крышки: z = 36 мм
  robot.links.base_link.add(holder); logoGroup = holder;
  holder.traverse(o => { if (o.isMesh) { Object.assign(o.userData, { name: 'Логотип Ala-Too IT & Business College (Ø40 мм, рельеф до 2 мм)', cat: 'logo' }); pick.push(o); } });
}

// ---- задняя надпись «ALA-TOO»: гравировка K.D.Y залита в stl/kdy/backpart_alatoo.stl, надпись — parts/rear_text.stl (tools/mounts/make_rear.py) ----
function addRearText() {
  stlLoader.load('parts/rear_text.stl', g => {
    g.scale(0.001, 0.001, 0.001);
    const txt = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0x11225f, roughness: .45, metalness: .1 }));
    robot.links.base_link.add(txt); shell.push(txt);
    Object.assign(txt.userData, { name: 'Надпись ALA-TOO (рельеф 0.8 мм, печатается вместе с крышкой)', cat: 'logo' }); pick.push(txt);
  });
}

// ===== компоновка по списку деталей от 23.10.2026 =====
// Pi 3B <-I2C-> PCA9685 (0x40), MPU-6050 (0x68), LCD 16x2 (0x27), ADS1115 (0x48, опция) -> 12 серв PDI-HV5523MG.
// Силовая цепь: LiPo -> XT60 -> предохранитель 15 А -> выключатель -> Y-разветвитель -> PCA9685 V+ (с конденсатором) и UBEC 5V/5A.
const servos = [];            // { g, ch, name, leg, kind }
const wires = [];             // { mesh, name, r, pts | dyn, check }
const V3 = (a) => new THREE.Vector3(MM(a[0]), MM(a[1]), MM(a[2]));

// Raspberry Pi 3B: плата 85×56, начало — левый нижний угол (вид сверху, гребёнка сверху, USB справа), низ платы z = 0
const PI0 = [-48, -47.5, -1.6];                       // угол платы в base_link; Pi на площадке mike4192 (5.5 мм) на скотче к плите — сверлить плиту не нужно
const piB = (x, y, z) => [PI0[0] + x, PI0[1] + y, PI0[2] + z];
function piPin(n) { const c = Math.floor((n - 1) / 2); return piB(8.37 + 2.54 * c, n % 2 ? 51.23 : 53.77, 1.4 + 8.5); }
const PI_USB = { A: piB(87.1, 47, 12.3), B: piB(87.1, 29, 12.3) };    // верхние порты USB (стопки на Y=47 и Y=29)
const PI_CSI = piB(45, 15.25, 7);
// PCA9685 (CAD Adafruit 815): 16 каналов по 3 контакта со стороны Pi, группы по 4; клеммник V+ с другой стороны
const PCA_AT = [-5, 38, -3.6];                         // PCA9685 на площадке mike4192 (5.5 мм): низ платы z -1.6
const GST = [-25, -12.9, 2.3, 15.6];
function pcaCh(c, row) {      // row: 0 GND, 1 V+, 2 PWM
  const g = Math.floor(c / 4), k = c % 4;
  return [PCA_AT[0] + GST[g] + 1.3 + 2.54 * k, PCA_AT[1] + (-6.5 - 2.5 * row), PCA_AT[2] + 7.5];
}
const plugTop = (c, row) => { const p = pcaCh(c, row); return [p[0], p[1], PCA_AT[2] + 20.1]; };   // провод выходит из верха штекера
const PCA_VPLUS = [PCA_AT[0] - 2.2, PCA_AT[1] + 9, PCA_AT[2] + 12.5], PCA_VGND = [PCA_AT[0] + 2.2, PCA_AT[1] + 9, PCA_AT[2] + 12.5];
const PCA_I2C = { SDA: [PCA_AT[0] - 22, PCA_AT[1] + 11, PCA_AT[2] + 5], SCL: [PCA_AT[0] - 19.5, PCA_AT[1] + 11, PCA_AT[2] + 5], VCC: [PCA_AT[0] - 17, PCA_AT[1] + 11, PCA_AT[2] + 5], GND: [PCA_AT[0] - 14.5, PCA_AT[1] + 11, PCA_AT[2] + 5] };
const PCA_I2C2 = { GND: [PCA_AT[0] + 14.5, PCA_AT[1] + 11, PCA_AT[2] + 5], VCC: [PCA_AT[0] + 17, PCA_AT[1] + 11, PCA_AT[2] + 5], SCL: [PCA_AT[0] + 19.5, PCA_AT[1] + 11, PCA_AT[2] + 5], SDA: [PCA_AT[0] + 22, PCA_AT[1] + 11, PCA_AT[2] + 5] };
// MPU-6050 (GY-521 21.2×16.4) на стойках 4 мм справа от PCA, гребёнка со стороны Pi
const MPU_AT = [37.5, 33, -3];
const MPU_PIN = Object.fromEntries(['VCC', 'GND', 'SCL', 'SDA', 'XDA', 'XCL', 'AD0', 'INT'].map((n, i) => [n, [MPU_AT[0] - 8.89 + 2.54 * i, MPU_AT[1] - 6.9, MPU_AT[2] + 7.6]]));
// нижний отсек (между плитой z -10 и дном z -36): аккумулятор по центру, UBEC слева, разветвитель и ADS1115 справа
const BATT = { lo: [-47, -23.5, -35.5], hi: [47, 23.5, -10.4] };     // Reedy #306 shorty 94×47×25.1 мм; отсек 127×48×26 мм
const UBEC = { lo: [-15, 25.5, -23], hi: [30, 47.5, -11] };
const UBEC_IN = { V: [30.5, 34, -17], G: [30.5, 39, -17] }, UBEC_OUT = { V: [-15.5, 34, -17], G: [-15.5, 39, -17] };
const YS = { lo: [-56, -46, -22], hi: [-16, -28, -12] };
const ADS_AT = [32, -38.25, -14.6];
const ADS_PIN = Object.fromEntries(['VDD', 'GND', 'SCL', 'SDA', 'ADDR', 'ALRT', 'A0', 'A1', 'A2', 'A3'].map((n, i) => [n, [ADS_AT[0] - 11.43 + 2.54 * i, ADS_AT[1] + 7.3, ADS_AT[2] - 6]]));
const XT = { lo: [-98, -8, -23.5], hi: [-70, 8, -15.5] };          // XT60 аккумулятора, в проходе под задними плечевыми серво
// задняя крышка: окно LCD, под ним отверстие выключателя 19×13 мм (центр y -25.6, z -20.4) и держатель предохранителя
const LCD_AT = [-141, 0, 5.5];
const LCD_PIN = { GND: [-136, 22, -2], VCC: [-136, 24.5, -2], SDA: [-136, 27, -2], SCL: [-136, 29.5, -2] };
const FUSE = { lo: [-141, -12, -27], hi: [-127, 28, -17] };
const SW_AT = [-147.79, -25.6, -20.4], SW_TILT = -0.0765;          // наружная грань стенки, наклон стенки 4.4°
const SW_T = { A: [-121.6, -21.6, -20.4], B: [-121.6, -29.6, -20.4] };
const BUZZ = { lo: [-45, -2, 21], hi: [-5, 23, 32] };              // на внутренней стороне верхней крышки
const SPK = { lo: [122, 18, -20], hi: [138, 36, 15] };              // левая «щека» морды, звук вперёд

function addElectronics() {
  const base = robot.links.base_link;
  // --- Raspberry Pi 3B (по механическому чертежу) + microSD ---
  const pi = group(base, PI0), PI = 'Raspberry Pi 3B';
  const silver = 0xb9bdc4, blk = 0x1c1d21;
  bx(pi, `${PI}: плата 85×56 мм`, 'pi', [0, 0, 0], [85, 56, 1.4], 0x2f9e57, { rough: .6 });
  bx(pi, `${PI}: GPIO 40 pin`, 'pi', [7.1, 50.2, 1.4], [57.9, 55.3, 3.9], blk);
  for (let n = 1; n <= 40; n++) { const p = piPin(n); bx(pi, `${PI}: GPIO pin ${n}`, 'pi', [p[0] - PI0[0] - .32, p[1] - PI0[1] - .32, 3.9], [p[0] - PI0[0] + .32, p[1] - PI0[1] + .32, 9.9], 0xd4af37, { edges: false, metal: .8, check: false }); }
  bx(pi, `${PI}: USB 2.0 ×2 (порты 1–2)`, 'pi', [69.7, 40.45, 1.4], [87.1, 53.55, 17], silver, { metal: .7 });
  bx(pi, `${PI}: USB 2.0 ×2 (порты 3–4)`, 'pi', [69.7, 22.45, 1.4], [87.1, 35.55, 17], silver, { metal: .7 });
  bx(pi, `${PI}: Ethernet`, 'pi', [65.8, 2.25, 1.4], [87.1, 18.25, 14.9], silver, { metal: .7 });
  bx(pi, `${PI}: HDMI`, 'pi', [24.5, -1, 1.4], [39.5, 10.5, 7.9], silver, { metal: .7 });
  bx(pi, `${PI}: micro-USB питание (не используется, 5 В идёт на GPIO)`, 'pi', [6.6, -1, 1.4], [14.6, 4.6, 4.4], silver, { metal: .7 });
  bx(pi, `${PI}: аудио 3.5 мм`, 'pi', [50, -2.5, 1.4], [57, 12, 7.4], blk);
  bx(pi, `${PI}: разъём камеры CSI`, 'pi', [35, 12.5, 1.4], [55, 18, 7], 0xe9e4d8);
  bx(pi, `${PI}: разъём дисплея DSI`, 'pi', [1.5, 17, 1.4], [6.5, 39, 7], 0xe9e4d8);
  bx(pi, `${PI}: процессор BCM2837`, 'pi', [22, 25, 1.4], [36, 39, 2.6], 0x55585e, { edges: false });
  bx(pi, `${PI}: слот microSD`, 'pi', [0.5, 20.5, -1.5], [14, 35.5, 0], silver, { metal: .7 });
  bx(pi, 'microSD 16–32 ГБ', 'pi', [-2.5, 22, -1.2], [12, 33, -0.4], 0x2b2f8f, { cat: 'sd' });
  platform('raspberry_pi_platform.stl', 'Площадка Raspberry Pi (печать, mike4192 thing:4596267) на двустороннем скотче, Pi — саморезы/M2.5', [PI0[0], PI0[1] + 56, -7.05], 'piplat');

  // --- PCA9685 + конденсатор 2200 мкФ 16 В на клеммнике V+ ---
  loadPart('pca_c.stl', 0x2b64d6, 'PCA9685, 16-канальный драйвер серв (CAD Adafruit 815), I²C 0x40, логика 3.3 В', 'pca', base, PCA_AT);
  platform('servo_board_platform.stl', 'Площадка PCA9685 (печать, mike4192 thing:4596267) на двустороннем скотче', [PCA_AT[0] - 31.2, PCA_AT[1] + 12.6, -7.05], 'pcaplat');
  cy(base, 'Конденсатор 2200 мкФ 16 В на клеммнике V+/GND PCA9685 (гасит броски от серв)', 'cap', [-18, 38, PCA_AT[2] + 8.85], 6.25, 25, 'x', 0x1d3f8f, { ud: { touch: ['pca'] } });

  // --- MPU-6050 (GY-521) ---
  const mpu = group(base, MPU_AT), MP = 'MPU-6050 (GY-521), I²C 0x68';
  bx(mpu, `${MP}: плата 21.2×16.4`, 'mpu', [-10.6, -8.2, 0], [10.6, 8.2, 1.6], 0x1f4fbf, { rough: .6 });
  bx(mpu, `${MP}: датчик`, 'mpu', [-2, -2, 1.6], [2, 2, 2.5], 0x222222, { edges: false });
  bx(mpu, `${MP}: гребёнка 8 pin`, 'mpu', [-10.2, -8.2, 1.6], [10.2, -5.6, 4.1], 0x1c1d21);
  for (const x of [-7.6, 7.6]) cy(mpu, `${MP}: стойка M3 4 мм`, 'mpu', [x, 5.5, -2.025], 2.5, 4.05, 'z', 0xeeeeee, { seg: 12, ud: { mount: true } });
  for (const n of Object.keys(MPU_PIN)) { const p = MPU_PIN[n]; bx(mpu, `${MP}: pin ${n}`, 'mpu', [p[0] - MPU_AT[0] - .32, p[1] - MPU_AT[1] - .32, 4.1], [p[0] - MPU_AT[0] + .32, p[1] - MPU_AT[1] + .32, 7.6], 0xd4af37, { edges: false, check: false }); }

  // --- камера Raspberry Pi v2 на печатном кронштейне в морде ---
  const L = layout.camera, cam = new THREE.Group(), CM = 'Raspberry Pi Camera v2';
  cam.matrix.makeBasis(new THREE.Vector3(...L.axis), new THREE.Vector3(0, 1, 0), new THREE.Vector3(...L.up)).setPosition(V3(L.origin));
  cam.matrixAutoUpdate = false; base.add(cam);
  const yc = L.yc, bl = L.h / 2 - 9.46;                  // объектив ≈ 9.5 мм от верхнего края
  bx(cam, `${CM}: плата 25×23.9`, 'cam', [0, yc - L.w / 2, -L.h / 2], [1, yc + L.w / 2, L.h / 2], 0x2f9e57, { ud: { touch: ['cammount'] } });
  bx(cam, `${CM}: модуль объектива 8.5×8.5`, 'cam', [1, yc - 4.25, bl - 4.25], [4.5, yc + 4.25, bl + 4.25], 0x1c1d21);
  cy(cam, `${CM}: объектив`, 'cam', [5, yc, bl], 3, 1, 'x', 0x0a0a12, { metal: .9, rough: .1 });
  bx(cam, `${CM}: разъём шлейфа`, 'cam', [-2.7, yc - 10.5, -L.h / 2], [0, yc + 10.5, -L.h / 2 + 5.5], 0xe9e4d8, { ud: { touch: ['cammount'] } });
  for (const [y, b] of L.holes) cy(cam, `${CM}: винт M2×6 + гайка в бобышке`, 'cam', [1.6, y, b], 1.9, 1.2, 'x', 0xc9ccd2, { seg: 12, metal: .8, check: false });
  loadPart('cam_mount.stl', 0xe07a1f, 'Кронштейн камеры (печать, print/cam_mount.stl): на рёбра морды, наклон 10° вниз', 'cammount', base, [0, 0, 0], {}, { nocheck: true });
  addFov(cam, yc, bl);

  // --- LCD 16×2 I²C (0x27) за окном задней крышки ---
  bx(base, 'LCD 16×2 I²C, адрес 0x27 (плата)', 'lcd', [LCD_AT[0] + 2, -40, LCD_AT[2] - 18], [LCD_AT[0] + 5, 40, LCD_AT[2] + 18], 0x1d6b3a, { rough: .6 });
  bx(base, 'LCD 16×2 (рамка)', 'lcd', [LCD_AT[0] - 4, -35.5, LCD_AT[2] - 12], [LCD_AT[0] + 2, 35.5, LCD_AT[2] + 12], 0x15171c);
  bx(base, 'LCD 16×2 (экран)', 'lcd', [LCD_AT[0] - 4.7, -32.25, LCD_AT[2] - 6.9], [LCD_AT[0] - 3.7, 32.25, LCD_AT[2] + 6.9], 0x9fd6b8, { rough: .3, ud: { touch: ['lcd'] } });
  bx(base, 'I²C-переходник PCF8574 (LCD), питание 5 В', 'lcd', [-134, 8, -11], [-124, 44, 3], 0x2a64d6, { rough: .6 });

  // --- питание ---
  bx(base, 'LiPo 2S 4000 мАч shorty (например Reedy #306, 94×47×25.1 мм, 202 г); обычный 138 мм не входит', 'batt', BATT.lo, BATT.hi, 0x1b4fa8, { rough: .6, ud: { note: 'до плиты 0.4 мм: высота аккумулятора не больше 25.5 мм' } });
  bx(base, 'LiPo 2S (наклейка)', 'batt', [-30, -23.8, -35.8], [30, 23.8, -10.2], 0xe8e8e8, { rough: .6, check: false });
  bx(base, 'XT60: разъём аккумулятора (папа+мама)', 'xt60', XT.lo, XT.hi, 0xe6c000, { rough: .5 });
  bx(base, 'Y-разветвитель XT60: PCA9685 V+ и UBEC', 'xt60', YS.lo, YS.hi, 0xe6c000, { rough: .5 });
  bx(base, 'Держатель предохранителя (ATC), провод 14 AWG', 'fuse', FUSE.lo, FUSE.hi, 0x1c1c20, { rough: .5 });
  bx(base, 'Предохранитель 15 А (синий)', 'fuse', [-138, -2, -17], [-130, 18, -13.5], 0x2a6fdb, { rough: .4 });
  const sw = group(base, SW_AT); sw.rotation.y = SW_TILT;
  const SWN = 'Выключатель Switch Components RF1-1A-DC (16 А при 12 В DC, вырез 19.2×13.0 мм) в штатном отверстии';
  bx(sw, `${SWN}: рамка`, 'sw', [-2.5, -10.5, -7.5], [0, 10.5, 7.5], 0x111111, { ud: { note: 'в отверстии 19×13 мм зазор ~0.2 мм — так и надо; от клемм до перегородки 0.6 мм, провода паять и отводить вбок' } });
  bx(sw, `${SWN}: клавиша`, 'sw', [-5, -8, -5.5], [-2.5, 8, 5.5], 0xc0262d);
  bx(sw, `${SWN}: корпус`, 'sw', [0, -9.3, -6.3], [18, 9.3, 6.3], 0x222222, { ud: { note: 'в отверстии 19×13 мм зазор ~0.2 мм — так и надо; от клемм до перегородки 0.6 мм, провода паять и отводить вбок' } });
  for (const y of [-4, 4]) bx(sw, `${SWN}: клемма 4.8 мм`, 'sw', [18, y - .4, -2.4], [26, y + .4, 2.4], 0xd0d3d8, { metal: .8 });
  bx(base, 'UBEC 5V/5A (≈45×22×12): питание Pi, LCD, камеры, USB', 'ubec', UBEC.lo, UBEC.hi, 0x1c1c20, { rough: .5 });
  bx(base, 'UBEC (наклейка)', 'ubec', [-5, 25.2, -22], [20, 47.8, -12], 0xd8a300, { rough: .6, check: false });
  bx(base, 'Пищалка разряда LiPo (на балансировочном разъёме), приклеена к крышке', 'buzz', BUZZ.lo, BUZZ.hi, 0x1c1c20, { rough: .5 });
  cy(base, 'Пищалка: излучатель', 'buzz', [-25, 10.5, 20.5], 6, 1, 'z', 0x333333, { check: false });

  // --- голос ---
  bx(base, 'USB-микрофон (донгл в верхнем USB-порту, выступ 10 мм)', 'mic', [PI_USB.A[0], PI_USB.A[1] - 6.5, PI_USB.A[2] - 3], [PI_USB.A[0] + 10, PI_USB.A[1] + 6.5, PI_USB.A[2] + 3], 0x30343c, { ud: { touch: ['pi'] } });
  bx(base, 'USB-штекер 90° вверх (кабель динамика) — прямой штекер упрётся в серво', 'spk', [PI_USB.B[0], PI_USB.B[1] - 7, PI_USB.B[2] - 4], [PI_USB.B[0] + 11, PI_USB.B[1] + 7, 25], 0x30343c, { cat: 'spk', ud: { touch: ['pi'] } });
  bx(base, 'USB-динамик компактный (≈18×20×35), звук через открытую морду', 'spk', SPK.lo, SPK.hi, 0x30343c);
  bx(base, 'USB-динамик: решётка', 'spk', [138, 20, -16], [138.6, 34, 11], 0x777b84, { check: false });

  // --- опция: ADS1115 под плитой + делитель ---
  const ads = group(base, ADS_AT), AD = 'ADS1115 (опция), I²C 0x48: напряжение батареи';
  bx(ads, `${AD}: плата 28×17.5`, 'ads', [-14, -8.75, 0], [14, 8.75, 1.6], 0x7b3fb5, { rough: .6, ud: { opt: true } });
  bx(ads, `${AD}: гребёнка 10 pin (вниз)`, 'ads', [-12.8, 6, -2.5], [12.8, 8.6, 0], 0x1c1d21, { ud: { opt: true } });
  for (const x of [-11, 11]) cy(ads, `${AD}: стойка M2.5 3 мм к низу плиты`, 'ads', [x, -5, 3.075], 2, 2.95, 'z', 0xc9a227, { seg: 6, ud: { opt: true, mount: true } });
  for (const [i, x] of [[0, -4], [1, 4]]) cy(base, `Делитель для ADS1115 (опция): резистор ${i ? 'R2 10 кОм' : 'R1 20 кОм'}`, 'ads', [ADS_AT[0] + x, ADS_AT[1] - 3, ADS_AT[2] - 4], 1.2, 6, 'x', 0xd8c39a, { check: false, ud: { opt: true } });

  // --- 12 серв PDI-HV5523MG: нумерация и каналы из spot_micro_motion_cmd.yaml ---
  const sides = { front_left: [1, 1], front_right: [1, -1], rear_left: [-1, 1], rear_right: [-1, -1] };
  const NUM = { front_right: [3, 2, 1], rear_right: [6, 5, 4], rear_left: [9, 8, 7], front_left: [12, 11, 10] };  // плечо, бедро, колено
  const LEGN = { front_left: 'LF', front_right: 'RF', rear_left: 'LB', rear_right: 'RB' };
  for (const [leg, [fx, sy]] of Object.entries(sides)) {
    const mk = (link, kind, i, num, file) => {
      const label = `Серво ${LEGN[leg]}_${i} (${kind}) №${num} → канал PCA ${num - 1}, PDI-HV5523MG`;
      const g = seatServo(link, seats[file][0], label, `servo_${LEGN[leg]}_${i}`);
      servos.push({ g, ch: num - 1, name: label, leg, kind: i, fx, sy, link });
    };
    // гнёзда сняты со сборки KDY (tools/kdy/fit_kdy.py -> parts/servo_seats.json): ось вала, верх корпуса, направление корпуса.
    // плечевой — в кронштейне плеча (arm_joint), бедренный — в бедре (arm), коленный — в голени (wrist); качалка — в соседней детали
    const lr = sy > 0 ? 'l' : 'r';
    mk(`${leg}_shoulder_link`, 'плечо', 1, NUM[leg][0], `${lr}shoulder`);
    mk(`${leg}_leg_link`, 'бедро', 2, NUM[leg][1], `${lr}arm`);
    mk(`${leg}_foot_link`, 'колено', 3, NUM[leg][2], `${lr}foot`);
  }
  buildWiring();
}

// поле зрения камеры v2: 62.2° × 48.8°
let fovLines = null, lensAt = null;
const FOV_H = 62.2 / 2 * Math.PI / 180, FOV_V = 48.8 / 2 * Math.PI / 180;
function addFov(cam, yc, bl) {
  lensAt = [5.5, yc, bl];
  const d = 300, pts = [];
  const corner = (sh, sv) => [lensAt[0] + d, yc + sh * d * Math.tan(FOV_H), bl + sv * d * Math.tan(FOV_V)];
  const C = [corner(1, 1), corner(-1, 1), corner(-1, -1), corner(1, -1)];
  for (let i = 0; i < 4; i++) pts.push(lensAt, C[i], C[i], C[(i + 1) % 4]);
  fovLines = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts.map(V3)), new THREE.LineBasicMaterial({ color: 0x3fd0ff, transparent: true, opacity: .8 }));
  fovLines.visible = false; cam.add(fovLines);
}

// ===== провода =====
function makeWire(name, color, r, pts, extra = {}) {
  const mat = new THREE.MeshStandardMaterial({ color, roughness: .5, metalness: .1 });
  const mesh = new THREE.Mesh(new THREE.BufferGeometry(), mat);
  Object.assign(mesh.userData, { name, wire: true, cat: extra.cat ?? 'wire', opt: extra.opt });
  pick.push(mesh);
  const w = { mesh, name, r, pts, dyn: extra.dyn || null, flat: extra.flat || 0, check: !extra.dyn && extra.check !== false, len: 0 };
  wires.push(w); robot.links.base_link.add(mesh);
  rebuildWire(w);
  return w;
}
function wireCurve(w) { return w.dyn ? servoCurve(w.dyn()) : new THREE.CatmullRomCurve3(w.pts.map(V3), false, 'catmullrom', 0.3); }
const servoCurve = (pts) => new THREE.CatmullRomCurve3(pts.map(V3), false, 'centripetal');   // кабели серв: без выгибов на углах трассы
function rebuildWire(w) {
  const curve = wireCurve(w), n = Math.max(10, curve.points.length * 6);
  w.len = curve.getLength() * 1000;
  let g;
  if (w.flat) {                                     // плоский шлейф: лента шириной flat мм
    const P = curve.getSpacedPoints(n * 2), pos = [], idx = [];
    let side = new THREE.Vector3(0, 1, 0);
    for (let i = 0; i < P.length; i++) {
      const t = curve.getTangentAt(i / (P.length - 1));
      const want = Math.abs(t.z) > 0.7 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(-t.y, t.x, 0).normalize();   // в вертикальных участках — поперёк X (как входит в CSI)
      side.lerp(want.dot(side) < 0 ? want.negate() : want, 0.25).normalize();
      const h = side.clone().multiplyScalar(MM(w.flat / 2));
      pos.push(...P[i].clone().add(h).toArray(), ...P[i].clone().sub(h).toArray());
      if (i) { const a = 2 * i; idx.push(a - 2, a - 1, a, a - 1, a + 1, a); }
    }
    g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setIndex(idx); g.computeVertexNormals();
    w.mesh.material.side = THREE.DoubleSide;
  } else g = new THREE.TubeGeometry(curve, n, MM(w.r), 6, false);
  w.mesh.geometry.dispose(); w.mesh.geometry = g;
}
// маршрут между двумя точками с подъёмом над платой
function route(a, b, lift = 8) { const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, Math.max(a[2], b[2]) + lift]; return [a, [a[0], a[1], a[2] + 4], m, [b[0], b[1], b[2] + 4], b]; }
const up = (p, dz) => [p[0], p[1], p[2] + dz];

function buildWiring() {
  const RED = 0xd62828, BLK = 0x1a1a1a, BLU = 0x2a6fdb, YEL = 0xf2c200, ORG = 0xff8c1a, BRN = 0x7a4a21, GRN = 0x2a9d3a, WHT = 0xdddddd, GRY = 0x6b6f78, PUR = 0x9b59b6;
  const P14 = 1.65, P = 'Силовая цепь 14 AWG: ';
  // аккумулятор -> XT60 (выводы с заднего торца)
  makeWire('Выводы LiPo (+) → XT60', RED, P14, [[BATT.lo[0], 5, -18], [-62, 5, -19], [XT.hi[0], 4, -19.5]]);
  makeWire('Выводы LiPo (−) → XT60', BLK, P14, [[BATT.lo[0], -5, -18], [-62, -5, -19], [XT.hi[0], -4, -19.5]]);
  // XT60 (+) -> предохранитель -> выключатель -> Y-разветвитель
  makeWire(P + 'XT60 (+) → предохранитель 15 А', RED, P14, [[XT.lo[0], 4, -19.5], [-108, 8, -18], [-118, 12, -18], [-124, 31, -22], [-134, 31, -22], [-134, 28.5, -22]]);
  makeWire(P + 'предохранитель → выключатель (клемма 1, пайка — до перегородки 0.6 мм)', RED, P14, [[-134, -12.5, -22], [-131, -16, -22], [-124, -18.5, -21], [-122.5, -20.5, -20.4], SW_T.A]);
  makeWire(P + 'выключатель (клемма 2) → Y-разветвитель (+)', RED, P14, [SW_T.B, [-123, -27, -19], [-124, -16, -17], [-112, -12, -17], [-67, -11, -17], [-62, -15, -17], [-60.5, -30, -17], [YS.lo[0] - 1, -35, -17]]);
  makeWire(P + 'XT60 (−) → Y-разветвитель (−)', BLK, P14, [[XT.lo[0], -4, -19.5], [-104, -9, -19], [-80, -14, -19], [-67, -14, -19], [-62, -17, -19], [-60.5, -32, -18], [YS.lo[0] - 1, -40, -17]]);
  // Y-разветвитель -> PCA9685 V+ (через щель за плитой x≈-51), -> UBEC (вокруг переднего торца аккумулятора)
  for (const [nm, col, dy, tgt] of [['(+) → PCA9685 V+', RED, -1.5, PCA_VPLUS], ['(−) → PCA9685 GND', BLK, 1.5, PCA_VGND]])
    makeWire(P + 'Y-разветвитель ' + nm, col, P14, [[YS.lo[0] - 1, -43 + dy, -17], [-57 + dy, -40, -12], [-57 + dy, -20, 2], [-57 + dy, 30, 4], [-46, 44 + dy, 12], [tgt[0], 45 + dy, 13], tgt]);
  for (const [nm, col, dy, tgt] of [['(+) → вход UBEC', RED, -1.5, UBEC_IN.V], ['(−) → вход UBEC', BLK, 1.5, UBEC_IN.G]])
    makeWire(P + 'Y-разветвитель ' + nm, col, P14, [[YS.hi[0] + 1, -37 + dy, -17], [10, -38 + dy, -22], [50, -32 + dy, -24], [61 + dy, -10, -22], [61 + dy, 20, -20], [45, tgt[1], -17], tgt]);
  // UBEC 5 В -> Pi (pin 2, 4 = 5V; pin 6 = GND), LCD
  for (const [n, col, src, dy] of [[2, RED, UBEC_OUT.V, 0], [4, RED, UBEC_OUT.V, 1.2], [6, BLK, UBEC_OUT.G, 2.4]])
    makeWire(`UBEC ${n === 6 ? 'GND' : '5V'} → Raspberry Pi pin ${n} (${n === 6 ? 'GND' : '5V'})`, col, 1.0, [src, [-45, 36 + dy, -14], [-53, 34, -6], [-53, 18 + dy, 8], [-42, 13 + dy, 15], up(piPin(n), 6), piPin(n)]);
  for (const [nm, col, src, pin, dy] of [['5V → LCD VCC', RED, UBEC_OUT.V, LCD_PIN.VCC, 0], ['GND → LCD GND', BLK, UBEC_OUT.G, LCD_PIN.GND, 1.6]])
    makeWire('UBEC ' + nm, col, 0.8, [src, [-50, 33 + dy, -16], [-60.5, 26 + dy, -16], [-60.5, 6 + dy, -13], [-75, 3 + dy, -12.5], [-110, 3 + dy, -12.5], [-121, 8 + dy, -10], [-130, 20, -6], pin]);
  // I²C, логика 3.3 В: Pi -> PCA9685 -> MPU-6050; Pi -> LCD
  makeWire('3.3 В: Raspberry Pi pin 1 → PCA9685 VCC (логика 3.3 В, не 5 В!)', ORG, 0.8, route(piPin(1), PCA_I2C.VCC, 12));
  makeWire('I²C SDA: Raspberry Pi pin 3 (GPIO2) → PCA9685 SDA', BLU, 0.8, route(piPin(3), PCA_I2C.SDA, 11));
  makeWire('I²C SCL: Raspberry Pi pin 5 (GPIO3) → PCA9685 SCL', YEL, 0.8, route(piPin(5), PCA_I2C.SCL, 12));
  makeWire('Земля: Raspberry Pi pin 9 → PCA9685 GND', BLK, 0.8, route(piPin(9), PCA_I2C.GND, 13));
  makeWire('I²C SDA: PCA9685 (выход шины) → MPU-6050 SDA', BLU, 0.8, route(PCA_I2C2.SDA, MPU_PIN.SDA, 8));
  makeWire('I²C SCL: PCA9685 (выход шины) → MPU-6050 SCL', YEL, 0.8, route(PCA_I2C2.SCL, MPU_PIN.SCL, 9));
  makeWire('3.3 В: PCA9685 VCC (выход шины) → MPU-6050 VCC', ORG, 0.8, route(PCA_I2C2.VCC, MPU_PIN.VCC, 10));
  makeWire('Земля: PCA9685 GND (выход шины) → MPU-6050 GND', BLK, 0.8, route(PCA_I2C2.GND, MPU_PIN.GND, 11));
  for (const [nm, col, pin, lp, dy] of [['SDA', BLU, 3, LCD_PIN.SDA, 0], ['SCL', YEL, 5, LCD_PIN.SCL, 1.4]])
    makeWire(`I²C ${nm}: Raspberry Pi pin ${pin} → LCD ${nm} (через Y-Dupont; нужен преобразователь уровней 3.3↔5 В)`, col, 0.8, [piPin(pin), up(piPin(pin), 5), [-49, 5 + dy, 12], [-60.5, -2 + dy, 4], [-60.5, -3 + dy, -12], [-75, -3 + dy, -12.5], [-110, -3 + dy, -12.5], [-121, 4 + dy, -10], [-131, 26, -4], lp]);
  // опция: ADS1115 — шина с PCA9685, вход A0 с делителя от (+) после выключателя
  for (const [nm, col, src, dst, dy] of [['SDA', BLU, PCA_I2C2.SDA, ADS_PIN.SDA, 0], ['SCL', YEL, PCA_I2C2.SCL, ADS_PIN.SCL, 1.2], ['3.3 В', ORG, PCA_I2C2.VCC, ADS_PIN.VDD, 2.4], ['GND', BLK, PCA_I2C2.GND, ADS_PIN.GND, 3.6]])
    makeWire(`ADS1115 (опция) ${nm} ← PCA9685`, col, 0.6, [src, up(src, 3), [40, 46 - dy, 3], [52 + dy / 2, 20, 3], [52 + dy / 2, -30, 1], [50, -32 + dy, -14], [dst[0], dst[1] + 2, dst[2] - 2], dst], { opt: true });
  makeWire('ADS1115 A0 ← делитель 20к/10к ← (+) после выключателя (опция)', WHT, 0.6, [ADS_PIN.A0, [ADS_PIN.A0[0], -35, -21], [0, -27, -22], [-40, -26.5, -22], [YS.lo[0] - 1, -33, -18]], { opt: true });
  // пищалка: балансировочный разъём аккумулятора (3 жилы)
  [[RED, -1], [BLU, 0], [BLK, 1]].forEach(([col, k]) => makeWire('Балансировочный провод LiPo (JST-XH 3 pin) → пищалка разряда', col, 0.5, [[BATT.lo[0], -15 + k, -13], [-60.5, -6 + k, -12], [-60.5, -4 + k, 6], [-50, 2 + k, 18], [-30, 8 + k, 20.5], [-26, 10 + k, 21]]));
  // USB-динамик: кабель от штекера 90° через верх серво и проход в переборке
  makeWire('USB-кабель динамика (≈20 см до морды, остаток свернуть)', GRY, 2.0, [[PI_USB.B[0] + 5.5, PI_USB.B[1], 25], [62, -6, 17], [90, 0, 15], [117, 10, 13], [124, 22, 6], [SPK.lo[0] - 0.5, 28, 0]]);
  // шлейф камеры: вниз между рёбрами морды, назад по проходу под передними серво, вверх к CSI
  const L = layout.camera, ax = new THREE.Vector3(...L.axis), vv = new THREE.Vector3(...L.up), o = new THREE.Vector3(...L.origin);
  const conn = o.clone().addScaledVector(ax, -1.3).add(new THREE.Vector3(0, L.yc, 0)).addScaledVector(vv, -L.h / 2 - 0.5).toArray();
  camRibbon = makeWire('Шлейф камеры 15-pin FFC (16 мм) → разъём CSI Raspberry Pi', WHT, 0.2, [conn, [conn[0] - 4, 0.5, -24], [125, 0.5, -24], [116, 0.5, -19], [100, 0, -17], [75, 0, -16], [61, 0, -12], [60, 0, 6], [58, -2, 27], [30, -24, 28], [PI_CSI[0], PI_CSI[1], 22], up(PI_CSI, 3), PI_CSI], { flat: 16 });
  // 12 серв: жилы GND, V+, PWM от канала PCA9685. Трассы — parts/cable_routes.json (tools/kdy/cable_routes.py: поиск пути
  // по сетке 2 мм с зазором 2 мм от пластика, отдельно для левой и правой стороны — детали R_ не зеркальны L_ точно):
  //   бедро — из торца серво вбок в щель у крышки бедра и по ноге к верху бедра;
  //   колено — из голени через ось колена с внешней стороны (кабель не двигается при сгибании), вверх по передней кромке бедра;
  //   дальше над кронштейном плеча в щель между переборкой I_shoulder и кронштейном (|x| ≈ 72 мм) и внутрь корпуса;
  //   плечо — под кронштейном плеча в ту же щель и через центральный проход (|y| < 20 мм).
  // Точки на ноге двигаются вместе с её звеньями; точки трассы корпуса с |x| ≥ 75 мм крепятся к кронштейну плеча.
  const base = robot.links.base_link;
  const local = (obj, p) => { const v = obj.localToWorld(new THREE.Vector3(...p.map(MM))); base.worldToLocal(v); return v.toArray().map(x => x * 1000); };
  for (const sv of servos) {
    const S = robot.links[`${sv.leg}_shoulder_link`], L = robot.links[`${sv.leg}_leg_link`], F = robot.links[`${sv.leg}_foot_link`];
    const R = routes[sv.sy > 0 ? 'l' : 'r'], k = sv.fx > 0 ? 'front' : 'rear', sy = sv.sy;
    const G = (x, y, z) => local(sv.g, [x, y, z]);                                  // система серво
    const onBody = (p) => Math.abs(p[0]) >= 75 ? local(S, [p[0] - sv.fx * 93, p[1] - sy * 39, p[2]]) : p;   // в кронштейн плеча или в корпус
    sv.path = (row = 1) => {
      const a = plugTop(sv.ch, row), pts = [a, [a[0], a[1] - 4, a[2] + 4]];
      if (sv.kind === 1) pts.push(...[...R[`${k}_shoulder`]].reverse().map(onBody));
      else {
        pts.push(...[...R[`${k}_leg`]].reverse().slice(0, -1).map(onBody));
        if (sv.kind === 2) pts.push(...[...R.hip].reverse().map(p => local(L, p)), G(-33, sy * 11.5, -4), G(-33, sy * 11.5, -30), G(-31.4, sy * 11.5, -33), G(-31.4, sy * 5, -33));   // паз 2 мм в стенке бедра (print/kdy/*_arm_cls_cable.stl)
        else pts.push(...[...R.knee_up].reverse().map(p => local(L, p)), local(L, [14, sy * 32, -82]), local(L, [2, sy * 32, -98]), local(L, [0, sy * 30, -107.5]), local(F, [0, sy * 30, -16]), ...[...R.knee_down].reverse().map(p => local(F, p)));
      }
      pts.push(local(sv.g, [-31, 0, -33])); return pts;                     // вывод кабеля: торец корпуса у дна
    };
    [[0, BRN, 'GND', -1.2], [1, RED, 'V+', 0], [2, ORG, 'сигнал PWM', 1.2]].forEach(([row, col, nm, off]) => {
      makeWire(`${sv.name}: жила ${nm} (канал ${sv.ch})`, col, 0.55, null, {
        cat: 'servowire', dyn: () => { const p = sv.path(row); const e = p[p.length - 1]; p[p.length - 1] = [e[0], e[1], e[2] + off]; return p; }
      });
    });
  }
  // штекеры серв на гребёнке PCA9685 (мама 3 pin, 2.5×7.6×14 мм) — стоят вертикально на штырях
  for (const sv of servos) {
    const lo = pcaCh(sv.ch, 0), hi = pcaCh(sv.ch, 2);
    bx(base, `${sv.name.split(' →')[0]}: штекер на канале ${sv.ch} PCA9685 (GND · V+ · PWM)`, 'plug', [lo[0] - 1.25, hi[1] - 1.3, PCA_AT[2] + 6.1], [lo[0] + 1.25, lo[1] + 1.3, PCA_AT[2] + 20.1], 0x15171c, { cat: 'ext', check: false });
  }
  // стык «своего провода серво (265 мм) и удлинителя»: мама серво (чёрная) + папа удлинителя (серая), вдоль кабеля
  for (const sv of servos.filter(s => s.kind > 1)) {
    const g = new THREE.Group();
    for (const [x0, x1, col] of [[0, 14, 0x15171c], [-12, 0, 0x8a8f99]]) {
      const m = new THREE.Mesh(new THREE.BoxGeometry(MM(x1 - x0), MM(7.6), MM(2.6)), new THREE.MeshStandardMaterial({ color: col, roughness: .6 }));
      m.position.x = MM((x0 + x1) / 2); m.add(new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), edgeMat)); g.add(m);
    }
    robot.links.base_link.add(g); junctions.push({ g, sv });
    g.traverse(o => { if (o.isMesh) { Object.assign(o.userData, { cat: 'ext', part: 'ext', nocheck: true }); pick.push(o); } });
  }
  robot.updateMatrixWorld(true);
  wires.forEach(rebuildWire); updateJunctions();
}
const junctions = [];
function updateJunctions() {
  for (const j of junctions) {
    const c = servoCurve(j.sv.path(1)), L = c.getLength() * 1000, u = Math.max(0, 1 - SERVO_LEAD / L);
    j.g.position.copy(c.getPointAt(u)); j.g.quaternion.setFromUnitVectors(new THREE.Vector3(-1, 0, 0), c.getTangentAt(u)); j.at = j.g.position.toArray().map(x => x * 1000);
    const name = `${j.sv.name.split(' →')[0]}: стык своего провода серво (265 мм, мама) и удлинителя ${extLen(L)} см (папа), ${Math.round(L - SERVO_LEAD)} мм до PCA9685`;
    j.g.traverse(o => { if (o.isMesh) o.userData.name = name; });
  }
}
let camRibbon = null;
let wireTick = 0;
function updateWires() { if ((wireTick++ % 3) === 0) { wires.forEach(w => { if (w.dyn) rebuildWire(w); }); updateJunctions(); } }

// ===== легенда =====
const CAT = {
  pi: 'Raspberry Pi 3B', sd: 'microSD 16–32 ГБ', pca: 'PCA9685', cap: 'Конденсатор 2200 мкФ 16 В', mpu: 'MPU-6050 (GY-521)', cam: 'Камера Pi v2', cammount: 'Кронштейн камеры (печать)',
  lcd: 'LCD 16×2 I²C', batt: 'LiPo 2S 4000 мАч', xt60: 'XT60 + Y-разветвитель', fuse: 'Предохранитель 15 А', sw: 'Выключатель', ubec: 'UBEC 5V/5A', buzz: 'Пищалка разряда',
  mic: 'USB-микрофон', spk: 'USB-динамик', plat: 'Площадки Pi и PCA9685 (печать)', ext: 'Штекеры серв и удлинители', ads: 'ADS1115 + делитель (опция)', servo: 'Серво PDI-HV5523MG ×12', wire: 'Провода', servowire: 'Кабели серв (+ удлинители)', logo: 'Логотип и надпись',
};
function buildLegend() {
  const seen = new Map();
  pick.forEach(m => { const k = m.userData.cat; if (k && !seen.has(k)) seen.set(k, m.material?.color?.getHexString?.() || '888888'); });
  document.getElementById('legend').innerHTML = Object.keys(CAT).filter(k => seen.has(k)).map(k => `<label class="row"><span class="sw" style="background:#${seen.get(k)}"></span>${CAT[k]}</label>`).join('');
}

// ===== проверка стыковки, длин кабелей, обзора камеры =====
const SERVO_LEAD = 265, SLACK = 20;                // провод серво 265 мм (по продавцу), запас на изгибы
const extLen = (L) => [10, 20, 30].find(e => L + SLACK <= SERVO_LEAD + e * 10) ?? 0;   // самый короткий удлинитель, которого хватает: лишний провод не сворачивать в корпусе
let report = null;
function onPartsLoaded() { buildLegend(); setTimeout(runCheck, 50); }
function runCheck() {
  const t0 = performance.now(), base = robot.links.base_link;
  const saved = Object.fromEntries(Object.entries(robot.joints).map(([n, j]) => [n, j.angle]));
  applyJoints(POSES.stand); robot.updateMatrixWorld(true);
  const baseInv = base.matrixWorld.clone().invert();
  const tris = bodyTriangles(body, baseInv);
  // 1) детали против корпуса и друг против друга
  const items = solids.filter(m => !m.userData.nocheck && !m.userData.part?.startsWith('servo_') && !(m.userData.opt && !showOpt)).map(m => ({ m, o: obbOf(m, baseInv) }));
  const parts = new Map();
  const res = (m) => { const k = m.userData.part; if (!parts.has(k)) parts.set(k, { part: k, cat: m.userData.cat, name: m.userData.name.split(':')[0], body: [], near: [], notes: new Set(), with: new Set(), meshes: [] }); return parts.get(k); };
  for (const { m, o } of items) {
    const r = res(m); r.meshes.push(m);
    if (obbVsBody(o, tris, 0)) r.body.push(m.userData.name); else if (!m.userData.mount && obbVsBody(o, tris, 0.5)) { r.near.push(m.userData.name); if (m.userData.note) r.notes.add(m.userData.note); }
  }
  for (let i = 0; i < items.length; i++) for (let j = i + 1; j < items.length; j++) {
    const A = items[i], B = items[j], pa = A.m.userData.part, pb = B.m.userData.part;
    if (pa === pb || A.m.userData.touch?.includes(pb) || B.m.userData.touch?.includes(pa)) continue;
    if (obbObb(A.o, B.o, 0.05)) { res(A.m).with.add(CAT[B.m.userData.cat] || pb); res(B.m).with.add(CAT[A.m.userData.cat] || pa); }
  }
  // серво против корпуса и деталей ног: точки корпуса серво (шаг 2.5 мм, на 0.5 мм внутрь — допуск печати) внутри пластика
  const servoHits = [], legTris = {};
  const trisOf = (link) => legTris[link] ??= bodyTriangles(shell.filter(m => linkOf(m) === link), baseInv);
  const v = new THREE.Vector3();
  for (const sv of servos) {
    const links = ['base_link', 'front_link', 'rear_link', ...['shoulder_link', 'leg_link', 'leg_link_cover', 'foot_link'].map(k => `${sv.leg}_${k}`)];
    const M = new THREE.Matrix4().multiplyMatrices(baseInv, sv.g.matrixWorld), hit = {};
    let tot = 0;
    for (let x = -29.9; x <= 9.6; x += 2.5) for (let y = -9.6; y <= 9.6; y += 2.4) for (let z = -37.5; z <= -0.5; z += 2.5) {
      v.set(MM(x), MM(y), MM(z)).applyMatrix4(M); const p = [v.x * 1000, v.y * 1000, v.z * 1000]; tot++;
      for (const l of links) if (insideBody(p, trisOf(l))) hit[l] = (hit[l] || 0) + 1;
    }
    const cell = 40.5 * 20.2 * 38 / tot;
    for (const [l, n] of Object.entries(hit)) servoHits.push(`${sv.name.split(' №')[0]} — ${PARTN[l.replace(/^(front|rear)_(left|right)_/, '')] || l}: ~${Math.round(n * cell)} мм³`);
  }
  updateJunctions();
  const jIn = junctions.filter(j => Math.abs(j.at[0]) < 66 && Math.abs(j.at[1]) < 52 && j.at[2] > -8 && j.at[2] < 24).length;
  // 2) провода сквозь стенки корпуса
  const wireHits = [];
  for (const w of wires) if (w.check && !(w.mesh.userData.opt && !showOpt)) {
    const P = wireCurve(w).getSpacedPoints(60).map(v => v.toArray().map(x => x * 1000));
    for (let i = 1; i < P.length; i++) if (segVsBody(P[i - 1], P[i], tris)) { wireHits.push(`${w.name} (у точки ${P[i].map(Math.round).join(', ')} мм)`); break; }
  }
  // 3) длина кабелей серв во всех позах (стойка, сидя, присед, цикл шага вперёд и с поворотом)
  const poses = [POSES.stand, POSES.sit];
  for (const crouch of [0, 1]) for (const fast of [0, 1]) for (let k = 0; k < 8; k++) for (const turn of [0, 1]) poses.push(gaitPose(k / 8, turn ? 0 : 1, turn, fast, crouch));
  const need = servos.map(() => 0);
  for (const p of poses) { applyJoints(p); robot.updateMatrixWorld(true); servos.forEach((sv, i) => { need[i] = Math.max(need[i], servoCurve(sv.path(1)).getLength() * 1000); }); }
  // кабели серв сквозь раму и детали ног: стойка, сидя, присед и 4 фазы шага
  const cableHits = new Map(), PN = ['стойка', 'сидя', 'присед'];
  [POSES.stand, POSES.sit, gaitPose(0, 1, 0, 0, 1), ...[0, 2, 4, 6].map(k => gaitPose(k / 8, 1, 0, 1, 0))].forEach((p, pi) => {
    applyJoints(p); robot.updateMatrixWorld(true);
    const inv = base.matrixWorld.clone().invert(), T = {};
    for (const sv of servos) {
      const P = servoCurve(sv.path(1)).getSpacedPoints(48).map(v => v.toArray().map(x => x * 1000));
      for (const l of ['base_link', 'front_link', 'rear_link', ...['shoulder_link', 'leg_link', 'leg_link_cover', 'foot_link'].map(k => `${sv.leg}_${k}`)]) {
        T[l] ??= bodyTriangles(shell.filter(m => linkOf(m) === l), inv);
        for (let i = 1; i < P.length; i++) if (segVsBody(P[i - 1], P[i], T[l])) {
          const k = `${sv.name.split(' №')[0]} — ${PARTN[l.replace(/^(front|rear)_(left|right)_/, '')] || l}`;
          if (!cableHits.has(k)) cableHits.set(k, `${PN[pi] ?? 'шаг'}, у точки ${P[i].map(Math.round).join(', ')}`); break;
        }
      }
    }
  });
  applyJoints(saved); robot.updateMatrixWorld(true); wires.forEach(rebuildWire);
  const cables = servos.map((sv, i) => ({ name: sv.name.split(' (')[0] + ' ' + ['плечо', 'бедро', 'колено'][sv.kind - 1], len: need[i], ext: need[i] + SLACK > SERVO_LEAD, el: extLen(need[i]) }));
  // 4) обзор камеры: лучи по полю зрения против корпуса и ног
  const fov = cameraView();
  report = { parts: [...parts.values()], servoHits, wireHits, jIn, jN: junctions.length, cableHits: [...cableHits].map(([k, v]) => `${k} (${v})`), cables, ribbon: camRibbon.len, fov, ms: performance.now() - t0 };
  window.spotReport = report; window.spot = { robot, servos, body, solids, wires, tris, camera, controls, rig };
  renderReport(); renderElec();
}
function cameraView() {
  if (!lensAt) return null;
  const cam = fovLines.parent; cam.updateMatrixWorld(true);
  const o = cam.localToWorld(V3(lensAt)), ray = new THREE.Raycaster(), N = [9, 7];
  let blocked = 0, legs = 0, total = 0;
  for (let i = 0; i < N[0]; i++) for (let j = 0; j < N[1]; j++) {
    const h = -FOV_H + 2 * FOV_H * i / (N[0] - 1), v = -FOV_V + 2 * FOV_V * j / (N[1] - 1);
    const p = cam.localToWorld(V3([lensAt[0] + 100, lensAt[1] + 100 * Math.tan(h), lensAt[2] + 100 * Math.tan(v)]));
    ray.set(o, p.sub(o).normalize()); ray.far = 0.5; total++;
    const hit = ray.intersectObjects(shell.filter(m => m.visible), false)[0];
    if (hit) { blocked++; if (!['base_link', 'rear_link', 'front_link'].includes(linkOf(hit.object))) legs++; }
  }
  return { blocked, legs, total };
}
function renderReport() {
  const r = report, ok = '<b style="color:#4ad07a">✓</b>', bad = '<b style="color:#ff6b6b">✗</b>', warn = '<b style="color:#f2c200">⚠</b>';
  const bad1 = r.parts.filter(p => p.body.length || p.with.size), near = r.parts.filter(p => !p.body.length && !p.with.size && p.near.length);
  const extN = r.cables.filter(c => c.ext).length;
  const rows = [];
  rows.push(`${bad1.length ? bad : ok} Детали внутри корпуса: ${r.parts.length - bad1.length} из ${r.parts.length} без пересечений`);
  for (const p of bad1) rows.push(`&nbsp;&nbsp;${bad} <a href="#" data-hl="${p.part}">${CAT[p.cat] || p.name}</a>: ${p.body.length ? 'входит в корпус' : ''}${p.body.length && p.with.size ? '; ' : ''}${p.with.size ? 'задевает ' + [...p.with].join(', ') : ''}`);
  for (const p of near) rows.push(`&nbsp;&nbsp;${warn} <a href="#" data-hl="${p.part}">${CAT[p.cat] || p.name}</a>: ${p.notes.size ? [...p.notes].join('; ') : 'зазор до корпуса < 0.5 мм'}`);
  rows.push(`${r.servoHits.length ? bad : ok} Серво PDI-HV5523MG в гнёздах деталей _cls: ${r.servoHits.length ? 'корпус входит в пластик' : 'корпус 40.5×20.2×38 мм не входит ни в раму, ни в детали ног (посадка плотная, допуск 0.5 мм)'}`);
  for (const h of r.servoHits) rows.push(`&nbsp;&nbsp;${bad} ${h}`);
  rows.push(`${r.wireHits.length ? bad : ok} Провода: ${r.wireHits.length ? r.wireHits.length + ' проходят сквозь стенку' : 'ни один не проходит сквозь стенки'}`);
  for (const w of r.wireHits) rows.push(`&nbsp;&nbsp;${bad} ${w}`);
  rows.push(`${r.cableHits.length ? bad : ok} Кабели серв (7 поз, включая шаг и присед): ${r.cableHits.length ? r.cableHits.length + ' проходят сквозь детали' : 'не проходят ни сквозь раму, ни сквозь детали ног'}`);
  for (const w of r.cableHits) rows.push(`&nbsp;&nbsp;${bad} ${w}`);
  rows.push(`${r.ribbon <= 200 ? ok : r.ribbon <= 300 ? warn : bad} Шлейф камеры: нужно ≈${Math.round(r.ribbon)} мм → ${r.ribbon <= 200 ? 'хватит 20 см' : r.ribbon <= 300 ? 'брать 30 см (20 см мало)' : 'не хватит даже 30 см'}`);
  rows.push(`${extN ? warn : ok} Кабели серв (свой провод ${SERVO_LEAD} мм, запас ${SLACK} мм): удлинитель нужен на ${extN} из 12`);
  rows.push(`${r.jIn === r.jN ? ok : warn} Разъёмы удлинителей: ${r.jIn} из ${r.jN} внутри корпуса${r.jIn === r.jN ? '' : ', остальные на ноге или у плеча — закрепить стяжкой, чтобы не попали в сустав'}`);
  rows.push('<table class="cab">' + r.cables.map(c => `<tr><td>${c.name}</td><td>${Math.round(c.len)} мм</td><td>${c.ext ? (c.el ? `удлинитель ${c.el} см` : bad + ' мало 30 см') : 'хватает'}</td></tr>`).join('') + '</table>');
  if (r.fov) rows.push(`${r.fov.blocked - r.fov.legs ? warn : ok} Обзор камеры: корпус закрывает ${r.fov.blocked - r.fov.legs} из ${r.fov.total} лучей${r.fov.legs ? `, ноги — ${r.fov.legs}` : ''} (включи «Поле зрения камеры»)`);
  rows.push(`<span style="color:var(--mut)">проверено за ${Math.round(r.ms)} мс; ${showOpt ? 'с опцией ADS1115' : 'без опций'}</span>`);
  const el = document.getElementById('report'); el.innerHTML = rows.join('<br>');
  el.querySelectorAll('[data-hl]').forEach(a => a.onclick = e => { e.preventDefault(); highlight(a.dataset.hl); });
}
const PARTN = { base_link: 'рама', front_link: 'передняя крышка', rear_link: 'задняя крышка', shoulder_link: 'кронштейн плеча (arm_joint)', leg_link: 'бедро (arm)', leg_link_cover: 'крышка бедра', foot_link: 'голень (wrist)' };
// ---- электрика: расчёт по паспортным данным деталей из списка ----
const E = {
  vFull: 8.4, vNom: 7.4, vCut: 6.4, servoV: [6.0, 8.4], fuse: 15, sw: 16, capV: 16, ubecA: 5, mah: 4000,
  load5: [['Raspberry Pi 3B (пик)', 1.3], ['камера v2', 0.25], ['USB-динамик', 0.5], ['USB-микрофон', 0.1], ['LCD с подсветкой', 0.08], ['логика PCA9685, MPU-6050, ADS1115', 0.02]],
  servoWalk: [3, 6], servoStand: 1.5, balanceLead: 100,
};
function renderElec() {
  const ok = '<b style="color:#4ad07a">✓</b>', bad = '<b style="color:#ff6b6b">✗</b>', warn = '<b style="color:#f2c200">⚠</b>';
  const i5 = E.load5.reduce((s, [, a]) => s + a, 0), iB5 = i5 * 5 / (E.vNom * 0.9);
  const run = (a) => E.mah / 1000 * 0.8 / a * 60;
  const par = (pairs) => { let g = 0, gv = 0; for (const [v, r] of pairs) { g += 1 / r; gv += v / r; } return gv / g; };
  const vLcd = par([[3.3, 1.8], [5, 4.7]]);
  const bal = wires.find(w => w.name.startsWith('Балансировочный'))?.len ?? 0;
  const rows = [
    `${ok} <b>Серво от 2S напрямую:</b> ${E.vCut}–${E.vFull} В при допуске PDI-HV5523MG ${E.servoV[0]}–${E.servoV[1]} В. Полностью заряженный аккумулятор — ровно на верхней границе; LiHV (4.35 В/банка) не использовать.`,
    `${ok} <b>Предохранитель ${E.fuse} А и выключатель ${E.sw} А:</b> предохранитель сгорит раньше, чем перегреется выключатель. С предохранителем 20 А нужен выключатель ≥ 20 А, а такой в штатное отверстие 19×13 мм не встаёт.`,
    `${ok} <b>Конденсатор ${E.capV} В</b> на шине ${E.vFull} В — запас ×${(E.capV / E.vFull).toFixed(1)}. Соблюдать полярность.`,
    `${ok} <b>UBEC ${E.ubecA} А:</b> нагрузка 5 В ≈ ${i5.toFixed(2)} А (${Math.round(i5 / E.ubecA * 100)} %): ${E.load5.map(([n, a]) => `${n} ${a} А`).join(', ')}. Pi питать через pin 2 и 4 (две жилы), UBEC 5.0–5.2 В.`,
    `${ok} <b>I²C-адреса:</b> PCA9685 0x40 (и 0x70 all-call), LCD 0x27, MPU-6050 0x68, ADS1115 0x48 — конфликтов нет.`,
    `${ok} <b>Логика PCA9685 от 3.3 В</b> (pin 1 Pi): её подтяжки 10 кОм тянут шину к 3.3 В. Если запитать VCC от 5 В, шина поднимется выше 3.3 В.`,
    `${warn} <b>LCD-переходник PCF8574 на 5 В:</b> его подтяжки 4.7 кОм к 5 В вместе с подтяжками Pi 1.8 кОм к 3.3 В дают на SDA/SCL ≈ ${vLcd.toFixed(2)} В — больше допустимых 3.6 В для GPIO Pi. У mike4192 так и работает, но это вне спецификации Pi — желателен двунаправленный преобразователь уровней I²C (BSS138, 4 канала); либо выпаять 2 подтяжки на переходнике (тогда работа LCD не гарантирована: PCF8574 на 5 В ждёт «1» от 3.5 В).`,
    `${bad} <b>Делитель для ADS1115 10к+10к:</b> даёт до ${(E.vFull / 2).toFixed(1)} В, а на вход ADS1115 при питании 3.3 В можно не больше 3.6 В. Ставить 20 кОм + 10 кОм (до ${(E.vFull / 3).toFixed(1)} В), усиление ±4.096 В.`,
    `${warn} <b>Ток серв через PCA9685:</b> весь ток 12 серв идёт через клеммник и дорожки платы (при ходьбе ≈ ${E.servoWalk.join('–')} А, на рывках больше). У mike4192 с тем же набором так и работает; при долгой ходьбе дорожки V+/GND можно пропаять медным проводом или подать питание на гребёнки серв отдельной шиной.`,
    `${warn} <b>Время работы:</b> ${E.mah} мА·ч, 80 % ёмкости: ходьба ≈ ${Math.round(run(E.servoWalk[1] + iB5))}–${Math.round(run(E.servoWalk[0] + iB5))} мин, стоя ≈ ${Math.round(run(E.servoStand + iB5))} мин. Ток серв — оценка, паспортного тока у PDI-HV5523MG нет.`,
    `${bal > E.balanceLead ? warn : ok} <b>Балансировочный провод до пищалки:</b> по трассе ≈ ${Math.round(bal)} мм, у аккумулятора обычно ~${E.balanceLead} мм — ${bal > E.balanceLead ? 'нужен удлинитель JST-XH 3 pin' : 'хватает'}.`,
  ];
  document.getElementById('elec').innerHTML = rows.join('<br>');
}
let hl = null;
function highlight(part) {
  for (const m of pick) if (m.material?.emissive) m.material.emissive.setHex(m.userData.part === part && hl !== part ? 0xff2020 : 0x000000);
  hl = hl === part ? null : part;
}

// ---- суставы / позы ----
const NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right'];
const KIND = { shoulder: 'плечо', leg: 'бедро', foot: 'колено' };
const POSES = { stand: {}, sit: {} };
function setStand(shoulder, leg, foot) { return Object.fromEntries(NAMES.flatMap(n => [[n + '_shoulder', shoulder], [n + '_leg', leg], [n + '_foot', foot]])); }
POSES.stand = setStand(0, 0.75, -1.5);
POSES.sit = { ...setStand(0, 0.2, -0.4), rear_left_leg: 1.1, rear_right_leg: 1.1, rear_left_foot: -2.1, rear_right_foot: -2.1 };
function applyPose(p) { for (const [j, v] of Object.entries(p)) setJ(j, v); settle(); }
function applyJoints(p) { for (const [j, v] of Object.entries(p)) robot.joints[j]?.setJointValue(v); }
function setJ(name, v) {
  const j = robot.joints[name]; if (!j) return;
  j.setJointValue(v);
  const s = document.querySelector(`[data-j="${name}"]`); if (s) { s.value = v; s.nextElementSibling.textContent = (v * 57.3).toFixed(0) + '°'; }
}
function settle() {   // поставить собаку лапами на пол
  rig.position.y = 0; rig.updateMatrixWorld(true);
  const b = new THREE.Box3(); robot.traverse(o => { if (o.isMesh && shell.includes(o)) b.expandByObject(o); });
  rig.position.y = -b.min.y;
}
function buildJointsUI() {
  const box = document.getElementById('joints');
  for (const n of NAMES) for (const k of ['shoulder', 'leg', 'foot']) {
    const jn = `${n}_${k}`, j = robot.joints[jn], lim = j.limit;
    const row = document.createElement('div'); row.className = 'sl';
    row.innerHTML = `<span>${n.replace('front', 'пер.').replace('rear', 'зад.').replace('_left', ' лев.').replace('_right', ' прав.')} ${KIND[k]}</span><input type="range" data-j="${jn}" min="${lim.lower}" max="${lim.upper}" step="0.01" value="0"><span>0°</span>`;
    row.querySelector('input').oninput = e => { stopAnim(); setJ(jn, +e.target.value); settle(); };
    box.appendChild(row);
  }
}

// ---- эмуляция ходьбы: обратная кинематика ноги + рысь (WASD) ----
const L1 = 0.1075, L2 = 0.130;                         // бедро и голень из URDF (м)
const X0 = 0.0153, Z0 = -0.1738;                       // стойка: стопа относительно сустава бедра
function ik(fx, fz) {                                   // стопа (x вперёд, z вверх) -> углы бедра и колена
  const d = Math.min(Math.hypot(fx, fz), L1 + L2 - 1e-4);
  const phi = Math.atan2(-fx, -fz);                     // направление на стопу (назад = +)
  const beta = Math.acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d));
  const gamma = Math.acos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2));
  return [phi + beta, -(Math.PI - gamma)];
}
let mode = 'pose', amp = 0, phase = 0, last = 0, groundY = 0, crouch = 0;
const ZS = -0.118;                                      // высота сустава бедра над стопой в приседе
const keys = new Set();
addEventListener('keydown', e => { if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyC', 'Space', 'ShiftLeft', 'ShiftRight', 'ControlLeft', 'ControlRight', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) { keys.add(e.code); if (!e.ctrlKey && !e.metaKey) e.preventDefault(); } });
addEventListener('blur', () => keys.clear());
addEventListener('keyup', e => keys.delete(e.code));
// углы суставов в фазе шага: fwd -1/0/1, turn -1/0/1, fast 0/1, cr — присед 0…1, a — амплитуда
function gaitPose(ph0, fwd, turn, fast, cr, a = 1) {
  const S = fast ? 0.085 : 0.065, H = 0.035;
  const dir = fwd !== 0 ? fwd : 1, stride = fwd !== 0 ? S : S * 0.45;
  const zNow = Z0 + cr * (ZS - Z0), out = {};
  NAMES.forEach(n => {
    const left = n.endsWith('left');
    const diag = (n === 'front_left' || n === 'rear_right') ? 0 : 0.5;
    let ph = (ph0 + diag) % 1, x, z;
    const st = stride * a * (fwd !== 0 ? 1 : (left ? -turn : turn)) * dir;
    if (ph < 0.5) { const p = ph / 0.5; x = X0 + st * (0.5 - p); z = zNow; }
    else { const p = (ph - 0.5) / 0.5; x = X0 + st * (-0.5 + p); z = zNow + H * a * Math.sin(Math.PI * p); }
    const [a1, a2] = ik(x, z);
    out[n + '_shoulder'] = 0; out[n + '_leg'] = a1; out[n + '_foot'] = a2;
  });
  return out;
}
function gaitStep(dt) {
  const fwd = (keys.has('KeyW') || keys.has('ArrowUp') ? 1 : 0) - (keys.has('KeyS') || keys.has('ArrowDown') ? 1 : 0);
  const turn = (keys.has('KeyA') || keys.has('ArrowLeft') ? 1 : 0) - (keys.has('KeyD') || keys.has('ArrowRight') ? 1 : 0);
  const sit = keys.has('ControlLeft') || keys.has('ControlRight') || keys.has('KeyC');
  const fast = keys.has('ShiftLeft') || keys.has('ShiftRight');
  crouch += ((sit ? 1 : 0) - crouch) * Math.min(1, dt * 7);                 // присед на Ctrl / C
  const want = ((fwd !== 0 || turn !== 0) && crouch < 0.25) ? 1 : 0;
  amp += (want - amp) * Math.min(1, dt * 6);                                // плавный старт и остановка
  const T = fast ? 0.42 : 0.55, S = fast ? 0.085 : 0.065;                   // период, длина шага
  if (amp > 0.02) phase = (phase + dt / T) % 1;
  for (const [j, v] of Object.entries(gaitPose(phase, fwd, turn, fast, crouch, amp))) setJ(j, v);
  const v = (S / (T * 0.5)) * amp * fwd;
  rig.rotation.y += turn * (fast ? 1.6 : 1.1) * amp * dt;
  const hx = Math.cos(rig.rotation.y), hz = -Math.sin(rig.rotation.y);
  const dx = hx * v * dt, dz = hz * v * dt;
  rig.position.x += dx; rig.position.z += dz;
  rig.position.y = groundY - crouch * (-Z0 - (-ZS));                         // корпус опускается при приседе
  grid.position.set(Math.round(rig.position.x / 0.05) * 0.05, 0, Math.round(rig.position.z / 0.05) * 0.05);
  camera.position.x += dx; camera.position.z += dz; controls.target.set(rig.position.x, 0.1, rig.position.z);
  hud.innerHTML = `<b>W/S</b> вперёд/назад · <b>A/D</b> поворот · <b>Shift</b> быстрее · <b>Ctrl</b> или <b>C</b> — сесть (держать) · мышь — камера<br>${crouch > 0.5 ? 'сидит' : amp > 0.5 ? 'идёт' : 'стоит'} · x=${rig.position.x.toFixed(2)} м, z=${rig.position.z.toFixed(2)} м`;
}
const hud = document.createElement('div'); hud.style.cssText = 'position:absolute;right:12px;top:10px;background:#000b;color:#fff;padding:7px 11px;border-radius:6px;font-size:12px;line-height:1.5;display:none'; view.appendChild(hud);
function stopAnim() { mode = 'pose'; hud.style.display = 'none'; }
const btn = id => document.getElementById(id);
function setBtn(id) { ['bStand', 'bSit', 'bWalk'].forEach(x => btn(x).classList.toggle('on', x === id)); }
btn('bStand').onclick = () => { stopAnim(); setBtn('bStand'); applyPose(POSES.stand); groundY = rig.position.y; };
btn('bSit').onclick = () => { stopAnim(); setBtn('bSit'); applyPose(POSES.sit); };
btn('bWalk').onclick = () => { if (mode !== 'walk') { applyPose(POSES.stand); groundY = rig.position.y; } mode = 'walk'; amp = 0; crouch = 0; phase = 0; last = performance.now(); setBtn('bWalk'); hud.style.display = 'block'; };
let spin = false; btn('bSpin').onclick = e => { spin = !spin; controls.autoRotate = spin; controls.autoRotateSpeed = 3; e.target.classList.toggle('on', spin); };
btn('bCheck').onclick = () => { if (robot && !pending) runCheck(); };

// ---- слои ----
let showOpt = true;
const optVisible = m => !m.userData.opt || showOpt;
btn('cShell').onchange = e => shell.forEach(m => m.visible = e.target.checked);
btn('sGhost').oninput = e => { const v = +e.target.value / 100; btn('vGhost').textContent = Math.round(v * 100) + '%'; shell.forEach(m => { m.material.transparent = v > 0.01; m.material.opacity = 1 - v * 0.92; m.material.depthWrite = v < 0.35; m.material.needsUpdate = true; }); };
btn('cElec').onchange = e => pick.forEach(m => { if (m.userData.cat !== 'logo') m.visible = e.target.checked && optVisible(m); });
btn('cOpt').onchange = e => { showOpt = e.target.checked; pick.forEach(m => { if (m.userData.opt) m.visible = showOpt && btn('cElec').checked; }); if (report) runCheck(); };
btn('cFov').onchange = e => { if (fovLines) fovLines.visible = e.target.checked; };
btn('cLogo').onchange = e => { if (logoGroup) logoGroup.visible = e.target.checked; };
btn('cLidar').onchange = e => robot.links.lidar_link.traverse(m => { if (m.isMesh) m.visible = e.target.checked; });
btn('cGrid').onchange = e => grid.visible = e.target.checked;

// ---- подсказка при наведении ----
const ray = new THREE.Raycaster(), mouse = new THREE.Vector2(), lbl = document.getElementById('lbl');
renderer.domElement.addEventListener('pointermove', ev => {
  const r = renderer.domElement.getBoundingClientRect();
  mouse.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObjects(pick.filter(m => m.visible), false)[0];
  if (hit) { lbl.style.display = 'block'; lbl.textContent = hit.object.userData.name; lbl.style.left = (ev.clientX - r.left + 14) + 'px'; lbl.style.top = (ev.clientY - r.top + 14) + 'px'; }
  else lbl.style.display = 'none';
});
function applyQuery() {
  const q = new URLSearchParams(location.search);
  if (q.get('nologo') && logoGroup) logoGroup.visible = false;
  if (q.get('ghost')) { btn('sGhost').value = 75; btn('sGhost').oninput({ target: btn('sGhost') }); }
  if (q.get('noshell')) { btn('cShell').checked = false; btn('cShell').onchange({ target: btn('cShell') }); }
  if (q.get('fov')) { btn('cFov').checked = true; btn('cFov').onchange({ target: btn('cFov') }); }
  const c = q.get('cam'), y = rig.position.y + 0.06, d = +(q.get('d') || 0.55);
  if (c === 'top') camera.position.set(0.001, d + .3, 0.001);
  if (c === 'side') camera.position.set(0, y, d);
  if (c === 'front') camera.position.set(d, y, 0);
  if (c === 'back') camera.position.set(-d, y, 0.001);
  if (q.get('pose') === 'sit') btn('bSit').onclick();
  if (q.get('sit')) keys.add('KeyC');
  if (q.get('auto')) { btn('bWalk').onclick(); keys.add(q.get('auto') === 'turn' ? 'KeyA' : 'KeyW'); }
  if (q.get('tz')) { const tz = +q.get('tz') / 1000, ty = +(q.get('ty') || 0) / 1000, tx = +(q.get('tx') || 0) / 1000; camera.position.set(tx - d, rig.position.y + tz, ty + 0.0001); controls.target.set(tx, rig.position.y + tz, ty); }
  else controls.target.set(0, y, 0);
}
function fit() { controls.target.set(0, 0.13, 0); }

let _w = 0, _h = 0;
renderer.setAnimationLoop(() => {
  if (view.clientWidth !== _w || view.clientHeight !== _h) { _w = view.clientWidth; _h = view.clientHeight; resize(); }
  if (robot && mode === 'walk') { const now = performance.now(); gaitStep(Math.min(0.05, (now - last) / 1000)); last = now; }
  if (robot && wires.length) updateWires();
  controls.update(); renderer.render(scene, camera);
});

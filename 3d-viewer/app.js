import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { SVGLoader } from 'three/examples/jsm/loaders/SVGLoader.js';
import { FontLoader } from 'three/examples/jsm/loaders/FontLoader.js';
import { TextGeometry } from 'three/examples/jsm/geometries/TextGeometry.js';
import URDFLoader from './lib/URDFLoader.js';

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
let robot, shell = [], elec = new THREE.Group();

const YEL = 0xe0b400, BLK = 0x222428;
loader.load('spot_micro.urdf', r => {
  robot = r;
  robot.rotation.x = -Math.PI / 2;           // URDF: Z вверх -> three: Y вверх
  rig.add(robot);
});
manager.onLoad = () => {
  if (!robot || shell.length) return;
  robot.traverse(o => { if (o.isMesh) { o.material = new THREE.MeshStandardMaterial({ color: o.material?.color?.getHex?.() ?? BLK, roughness: .55, metalness: .1 }); shell.push(o); } });
  buildJointsUI();
  addElectronics();
  addLogo();
  addRearText();
  applyPose(POSES.stand); groundY = rig.position.y;
  robot.links.lidar_link.traverse(m => { if (m.isMesh) m.visible = false; });
  fit();
  btn('bWalk').onclick();
  applyQuery();
  setTimeout(buildLegend, 1500);
  document.getElementById('load')?.remove();
};
manager.onError = u => console.error('не загрузилось', u);

// ---- электроника: реальные CAD-модели (STL, мм) + серво по паспорту MG996R ----
const stlLoader = new STLLoader();
const pick = [];
function tagPart(root, name) { root.traverse(o => { if (o.isMesh) { o.userData.name = name; pick.push(o); } }); }
function loadPart(file, color, name, parent, pos, rotZ = 0, extra = {}) {
  stlLoader.load('parts/' + file, g => {
    g.scale(0.001, 0.001, 0.001);
    const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color, roughness: .55, metalness: .25, ...extra }));
    m.position.set(pos[0] / 1000, pos[1] / 1000, pos[2] / 1000); m.rotation.z = rotZ;
    parent.add(m); tagPart(m, name);
    pending--; if (!pending) buildLegend();
  });
}
let pending = 0;
// MG996R: корпус 40.7 x 19.7 x 36.9, фланец 54.4 x 2.5, вал на 10 мм от торца. Начало координат — ось вала у верха корпуса, вал +Z, корпус вниз -Z, длинная сторона по X
function makeServo(name) {
  const g = new THREE.Group();
  const dark = new THREE.MeshStandardMaterial({ color: 0x23262d, roughness: .6, metalness: .1 });
  const steel = new THREE.MeshStandardMaterial({ color: 0xc9ccd2, roughness: .35, metalness: .8 });
  const lab = new THREE.MeshStandardMaterial({ color: 0xd9b000, roughness: .7 });
  const mmv = v => v / 1000;
  const body = new THREE.Mesh(new THREE.BoxGeometry(mmv(40.7), mmv(19.7), mmv(36.9 - 9.5)), dark);
  body.position.set(mmv(-9.85), 0, mmv(-(36.9 - 9.5) / 2 - 9.5)); g.add(body);
  const lower = body.clone(); lower.geometry = new THREE.BoxGeometry(mmv(40.7), mmv(19.7), mmv(9.5)); lower.position.z = mmv(-36.9 + 4.75); g.add(lower);
  const flange = new THREE.Mesh(new THREE.BoxGeometry(mmv(54.4), mmv(19.7), mmv(2.5)), dark);
  flange.position.set(mmv(-9.85), 0, mmv(-6.0)); g.add(flange);
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
    const h = new THREE.Mesh(new THREE.CylinderGeometry(mmv(2.3), mmv(2.3), mmv(2.7), 16), steel);
    h.rotation.x = Math.PI / 2; h.position.set(mmv(-9.85 + sx * 24.7), mmv(sy * 5), mmv(-6.0)); g.add(h);
  }
  const boss = new THREE.Mesh(new THREE.CylinderGeometry(mmv(7), mmv(7), mmv(4), 32), dark); boss.rotation.x = Math.PI / 2; boss.position.z = mmv(2); g.add(boss);
  const spl = new THREE.Mesh(new THREE.CylinderGeometry(mmv(3.2), mmv(3.2), mmv(4), 24), steel); spl.rotation.x = Math.PI / 2; spl.position.z = mmv(6); g.add(spl);
  const gear = new THREE.Mesh(new THREE.CylinderGeometry(mmv(6), mmv(6), mmv(2.2), 24), dark); gear.rotation.x = Math.PI / 2; gear.position.set(mmv(-21), 0, mmv(1.1)); g.add(gear);
  const sticker = new THREE.Mesh(new THREE.BoxGeometry(mmv(26), mmv(0.3), mmv(14)), lab); sticker.position.set(mmv(-9.85), mmv(10), mmv(-22)); g.add(sticker);
  const sticker2 = sticker.clone(); sticker2.position.y = mmv(-10); g.add(sticker2);
  g.traverse(o => { if (o.isMesh) { o.add(new THREE.LineSegments(new THREE.EdgesGeometry(o.geometry, 30), new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: .35 }))); } });
  tagPart(g, name);
  return g;
}
// поставить серво: вал в точку p (мм) вдоль d, длинная ось (в сторону -корпуса) вдоль l
function mountServo(parent, name, p, d, l) {
  const s = makeServo(name);
  const D = new THREE.Vector3(...d).normalize(), Lv = new THREE.Vector3(...l).normalize();
  const Y = new THREE.Vector3().crossVectors(D, Lv).normalize();
  s.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(Lv, Y, D));
  s.position.set(p[0] / 1000, p[1] / 1000, p[2] / 1000);
  parent.add(s);
}

// ---- логотип: рельеф на верхней крышке (диск Ø40 мм, 1 мм + белые элементы ещё 1 мм = выступ 2 мм) ----
let logoGroup = null;
async function addLogo() {
  const svgText = await (await fetch('logo.svg')).text();
  const data = new SVGLoader().parse(svgText);
  const MM = 0.001, K = 40 / 512;                    // 512 px = 40 мм
  const g = new THREE.Group();
  const navy = new THREE.MeshStandardMaterial({ color: 0x0b2a6b, roughness: .45, metalness: .1 });
  const white = new THREE.MeshStandardMaterial({ color: 0xf1f1ee, roughness: .4, metalness: .05 });
  const disk = new THREE.Mesh(new THREE.CylinderGeometry(20 * MM, 20 * MM, 1 * MM, 96), navy);
  disk.rotation.x = Math.PI / 2; disk.position.z = 0.5 * MM; g.add(disk);
  const flip = new THREE.Group(); flip.scale.y = -1; g.add(flip);   // ось Y в SVG направлена вниз
  for (const p of data.paths) {
    if (p.userData?.node?.id !== 'relief') continue;
    for (const shape of SVGLoader.createShapes(p)) {
      const geo = new THREE.ExtrudeGeometry(shape, { depth: 1, bevelEnabled: false, curveSegments: 5 });
      // координаты фигуры в пикселях SVG -> мм: центр (256,256), ось Y SVG вниз
      geo.translate(-256, -256, 0); geo.scale(K * MM, K * MM, MM);
      const m = new THREE.Mesh(geo, white); m.position.z = 1 * MM; flip.add(m);
    }
  }
  g.rotation.z = -Math.PI / 2;                       // верх логотипа к носу собаки (+X)
  const holder = new THREE.Group(); holder.add(g);
  holder.position.set(0.010, 0, 0.036);              // плоский участок крышки: z = 36 мм
  robot.links.base_link.add(holder); logoGroup = holder;
  holder.traverse(o => { if (o.isMesh) { o.userData.name = 'Логотип Ala-Too (рельеф, выступ 2 мм)'; pick.push(o); } });
}

// ---- задняя надпись: убрать «K.D.Y», написать «ALA-TOO» ----
function addRearText() {
  let bodyMat = null;
  robot.links.rear_link.traverse(o => { if (!bodyMat && o.isMesh) bodyMat = o.material; });
  stlLoader.load('parts/rear_patch.stl', g => {                  // заплатка по кривизне стенки: закрывает вырезанную надпись
    g.scale(0.001, 0.001, 0.001); g.translate(-0.0004, 0, 0);          // чуть наружу, чтобы старая гравировка не просвечивала
    const mat = bodyMat.clone(); mat.side = THREE.DoubleSide; mat.polygonOffset = true; mat.polygonOffsetFactor = -4; mat.polygonOffsetUnits = -4;
    const patch = new THREE.Mesh(g, mat); robot.links.base_link.add(patch); shell.push(patch);
  });
  new FontLoader().load('lib/helvetiker_bold.typeface.json', font => {
    const geo = new TextGeometry('ALA-TOO', { font, size: 3.7, height: 0.7, depth: 0.7, curveSegments: 8, bevelEnabled: true, bevelThickness: .05, bevelSize: .04, bevelSegments: 2 });
    geo.computeBoundingBox(); const b = geo.boundingBox;
    geo.translate(-(b.min.x + b.max.x) / 2, -(b.min.y + b.max.y) / 2, 0);
    geo.scale(0.001, 0.001, 0.001);
    const txt = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0x0b2a6b, roughness: .45, metalness: .1 }));
    // локальные оси текста -> база: X -> -Y, Y -> +Z, Z (выдавливание) -> -X (наружу, назад)
    txt.matrix.makeBasis(new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(-1, 0, 0));
    txt.matrix.setPosition(-0.1473, 0.0205, -0.0200); txt.matrixAutoUpdate = false;
    robot.links.base_link.add(txt); shell.push(txt);
    txt.userData.name = 'Надпись ALA-TOO (рельеф)'; pick.push(txt);
  });
}

// ===== схема по README mike4192/spotMicro =====
// Pi 3B <-I2C-> PCA9685 -> 12 серв PDI-HV5523MG; LiPo 2S 4000 мАч напрямую на V+ платы серв;
// UBEC HKU5 5V/5A питает Pi, логику PCA9685 и LCD 16x2 I2C (0x27).
const servos = [];            // { g: Group, ch: номер канала PCA (0..11), name }
const wires = [];             // { mesh, build() -> {pts, r, color}, name }
const V3 = (a) => new THREE.Vector3(a[0] / 1000, a[1] / 1000, a[2] / 1000);
const PI_AT = [0, -15, -2], PCA_AT = [-5, 38, -4];
// контакты Pi 3B (мм, система base_link): гребёнка GPIO на длинной стороне платы, пин 1 у края с SD-картой
function piPin(n) {
  const c = Math.floor((n - 1) / 2), odd = n % 2 === 1;
  return [PI_AT[0] - 46.4 + 7.1 + 2.54 * c, PI_AT[1] - 29.2 + (odd ? 50.1 : 52.6), PI_AT[2] + 11.5];
}
// PCA9685: 16 каналов по 3 контакта на нижней стороне платы; группы по 4
const GST = [-25, -12.9, 2.3, 15.6];
function pcaCh(c, row) {      // row: 0 GND, 1 V+, 2 PWM
  const g = Math.floor(c / 4), k = c % 4;
  return [PCA_AT[0] + GST[g] + 1.3 + 2.54 * k, PCA_AT[1] + (-6.5 - 2.5 * row), PCA_AT[2] + 7.5];
}
const PCA_VPLUS = [PCA_AT[0] - 2.2, PCA_AT[1] + 9, PCA_AT[2] + 12.5], PCA_VGND = [PCA_AT[0] + 2.2, PCA_AT[1] + 9, PCA_AT[2] + 12.5];
const PCA_I2C = { SDA: [PCA_AT[0] - 22, PCA_AT[1] + 11, PCA_AT[2] + 5], SCL: [PCA_AT[0] - 19.5, PCA_AT[1] + 11, PCA_AT[2] + 5], VCC: [PCA_AT[0] - 17, PCA_AT[1] + 11, PCA_AT[2] + 5], GND: [PCA_AT[0] - 14.5, PCA_AT[1] + 11, PCA_AT[2] + 5] };
const UBEC_AT = [58, 38, 0];
const UBEC_OUT = { V: [UBEC_AT[0] - 26, UBEC_AT[1] - 5, 7], G: [UBEC_AT[0] - 26, UBEC_AT[1] + 5, 7] };
const UBEC_IN = { V: [UBEC_AT[0] + 26, UBEC_AT[1] - 5, 7], G: [UBEC_AT[0] + 26, UBEC_AT[1] + 5, 7] };
const LCD_AT = [-141, 0, 5.5];
const LCD_PIN = { GND: [-136, 22, -2], VCC: [-136, 24.5, -2], SDA: [-136, 27, -2], SCL: [-136, 29.5, -2] };
const BATT_END = [-69.5, 0, -23];

function makeWire(name, color, r, pts, extra = {}) {
  const mat = new THREE.MeshStandardMaterial({ color, roughness: .5, metalness: .1 });
  const mesh = new THREE.Mesh(new THREE.BufferGeometry(), mat);
  mesh.userData.name = name; mesh.userData.wire = true; pick.push(mesh);
  const w = { mesh, name, r, pts, dyn: extra.dyn || null };
  wires.push(w); robot.links.base_link.add(mesh);
  rebuildWire(w);
  return w;
}
function rebuildWire(w) {
  const pts = (w.dyn ? w.dyn() : w.pts).map(V3);
  const curve = new THREE.CatmullRomCurve3(pts, false, 'catmullrom', 0.3);
  const g = new THREE.TubeGeometry(curve, Math.max(10, pts.length * 5), w.r / 1000, 5, false);
  w.mesh.geometry.dispose(); w.mesh.geometry = g;
}
// маршрут между двумя точками с подъёмом над платой
function route(a, b, lift = 8) { const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, Math.max(a[2], b[2]) + lift]; return [a, [a[0], a[1], a[2] + 4], m, [b[0], b[1], b[2] + 4], b]; }

function boxPart(name, w, d, h, color, parent, pos, rough = .55) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w / 1000, d / 1000, h / 1000), new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: .1 }));
  m.position.set(pos[0] / 1000, pos[1] / 1000, pos[2] / 1000); parent.add(m); tagPart(m, name);
  m.add(new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: .4 }))); return m;
}
function addElectronics() {
  const base = robot.links.base_link;               // X вперёд, Y влево, Z вверх; плита корпуса z ≈ -10…-7 мм
  // --- аккумулятор LiPo 2S 4000 мАч (≈139×48×26 мм) в нижней полости ---
  boxPart('LiPo 2S 4000 мАч, 7.4 В (по README)', 139, 48, 26, 0x1b4fa8, base, [0, 0, -23], .6);
  boxPart('LiPo 2S (наклейка)', 60, 48.6, 26.4, 0xe8e8e8, base, [0, 0, -23], .6);
  boxPart('LiPo 2S (разъём XT60)', 14, 18, 10, 0xe6c000, base, [-76.5, 0, -23], .5);
  // --- UBEC HKU5 5V/5A ---
  boxPart('UBEC HKU5 5V/5A (габариты ≈, CAD не найден)', 50, 26, 13, 0x1c1c20, base, [UBEC_AT[0], UBEC_AT[1], -7 + 6.5], .5);
  boxPart('UBEC (наклейка)', 30, 26.4, 8, 0xd8a300, base, [UBEC_AT[0], UBEC_AT[1], -7 + 6.5], .6);
  // --- LCD 16×2 I2C (0x27) за окном задней крышки ---
  boxPart('LCD 16×2 I²C, адрес 0x27 (плата)', 3, 80, 36, 0x1d6b3a, base, [LCD_AT[0] + 3.5, LCD_AT[1], LCD_AT[2]], .6);
  boxPart('LCD 16×2 (рамка)', 6, 71, 24, 0x15171c, base, [LCD_AT[0] - 1, LCD_AT[1], LCD_AT[2]], .5);
  boxPart('LCD 16×2 (экран)', 1, 64.5, 13.8, 0x9fd6b8, base, [LCD_AT[0] - 4.2, LCD_AT[1], LCD_AT[2]], .3);
  boxPart('I²C-переходник PCF8574 (LCD)', 10, 36, 14, 0x2a64d6, base, [LCD_AT[0] + 12, 26, -4], .6);
  pending = 2;
  loadPart('pi_c.stl', 0x2f9e57, 'Raspberry Pi (CAD: Pi 4B; в проекте Pi 3B, плата 85×56 та же)', base, PI_AT, 0);
  loadPart('pca_c.stl', 0x2b64d6, 'PCA9685, 16-канальный драйвер серв (CAD Adafruit 815), I²C 0x40', base, PCA_AT, 0);

  // --- 12 серв PDI-HV5523MG: нумерация и каналы из spot_micro_motion_cmd.yaml ---
  const sides = { front_left: [1, 1], front_right: [1, -1], rear_left: [-1, 1], rear_right: [-1, -1] };
  const NUM = { front_right: [3, 2, 1], rear_right: [6, 5, 4], rear_left: [9, 8, 7], front_left: [12, 11, 10] };  // плечо, бедро, колено
  const LEGN = { front_left: 'LF', front_right: 'RF', rear_left: 'LB', rear_right: 'RB' };
  for (const [leg, [fx, sy]] of Object.entries(sides)) {
    const sh = robot.joints[`${leg}_shoulder`], lg = robot.joints[`${leg}_leg`], kn = robot.joints[`${leg}_foot`];
    const mk = (parent, nm, kind, num, p, d, l) => {
      const label = `Серво ${LEGN[leg]}_${kind === 'плечо' ? 1 : kind === 'бедро' ? 2 : 3} (${kind}) №${num} → канал PCA ${num - 1}, PDI-HV5523MG`;
      mountServo(parent, label, p, d, l);
      servos.push({ g: parent.children[parent.children.length - 1], ch: num - 1, name: label });
    };
    mk(sh, 'plecho', 'плечо', NUM[leg][0], [0, 0, 0], [fx, 0, 0], [0, sy, 0]);
    mk(sh, 'bedro', 'бедро', NUM[leg][1], [0, sy * 55, 0], [0, sy, 0], [fx, 0, 0]);
    mk(lg, 'koleno', 'колено', NUM[leg][2], [0, 0, -107.5], [0, sy, 0], [0, 0, -1]);
  }
  buildWiring();
}

// ===== провода =====
function buildWiring() {
  const RED = 0xd62828, BLK = 0x1a1a1a, BLU = 0x2a6fdb, YEL = 0xf2c200, ORG = 0xff8c1a, BRN = 0x7a4a21, GRN = 0x2a9d3a;
  // питание серв: LiPo -> клеммник V+ платы PCA9685 (напрямую)
  const bp = [BATT_END[0] - 7, 7, BATT_END[2]], bm = [BATT_END[0] - 7, -7, BATT_END[2]];
  makeWire('Силовой провод: LiPo (+) 7.4 В → PCA9685 клемма V+ (питание серв)', RED, 1.7, [bp, [-82, 18, -20], [-84, 40, -4], [-60, 52, 14], [PCA_VPLUS[0] - 14, PCA_VPLUS[1] + 8, PCA_VPLUS[2] + 8], PCA_VPLUS]);
  makeWire('Силовой провод: LiPo (−) → PCA9685 клемма GND', BLK, 1.7, [bm, [-83, -18, -20], [-85, 8, -2], [-70, 50, 14], [PCA_VGND[0] - 6, PCA_VGND[1] + 9, PCA_VGND[2] + 8], PCA_VGND]);
  // LiPo -> вход UBEC
  makeWire('LiPo (+) → вход UBEC HKU5', RED, 1.2, [[-82, 14, -20], [-84, 30, -8], [-30, 50, 12], [20, 62, 14], UBEC_IN.V]);
  makeWire('LiPo (−) → вход UBEC HKU5', BLK, 1.2, [[-83, -14, -20], [-86, 0, -2], [-40, 54, 16], [24, 66, 16], UBEC_IN.G]);
  // UBEC 5 В -> Pi (pin 2 = 5V, pin 6 = GND)
  makeWire('UBEC 5V → Raspberry Pi, pin 2 (5V)', RED, 1.0, route(UBEC_OUT.V, piPin(2), 14));
  makeWire('UBEC GND → Raspberry Pi, pin 6 (GND)', BLK, 1.0, route(UBEC_OUT.G, piPin(6), 16));
  // UBEC 5 В -> логика PCA9685 (VCC, GND)
  makeWire('UBEC 5V → PCA9685 VCC (логика)', RED, 1.0, route(UBEC_OUT.V, PCA_I2C.VCC, 14));
  makeWire('UBEC GND → PCA9685 GND (логика)', BLK, 1.0, route(UBEC_OUT.G, PCA_I2C.GND, 17));
  // I2C: Pi pin 3 (GPIO2/SDA1) и pin 5 (GPIO3/SCL1) -> PCA9685 -> LCD
  makeWire('I²C SDA: Raspberry Pi pin 3 (GPIO2) → PCA9685 SDA', BLU, 0.8, route(piPin(3), PCA_I2C.SDA, 11));
  makeWire('I²C SCL: Raspberry Pi pin 5 (GPIO3) → PCA9685 SCL', YEL, 0.8, route(piPin(5), PCA_I2C.SCL, 12));
  makeWire('I²C SDA (шина): Pi pin 3 → LCD SDA', BLU, 0.8, route(piPin(3), LCD_PIN.SDA, 18));
  makeWire('I²C SCL (шина): Pi pin 5 → LCD SCL', YEL, 0.8, route(piPin(5), LCD_PIN.SCL, 20));
  makeWire('Земля: Raspberry Pi pin 9 (GND) → LCD GND', BLK, 0.8, route(piPin(9), LCD_PIN.GND, 22));
  makeWire('UBEC 5V → LCD VCC', RED, 0.8, route(UBEC_OUT.V, LCD_PIN.VCC, 26));
  makeWire('Общая земля: Raspberry Pi pin 14 (GND) → PCA9685 GND', BLK, 0.8, route(piPin(14), PCA_I2C.GND, 13));
  // 12 серв: каждый кабель из трёх жил (GND, V+, сигнал PWM) от канала PCA9685 к серво; обновляется при движении ног
  const base = robot.links.base_link;
  for (const sv of servos) {
    const anchor = () => { robot.updateMatrixWorld(true); const p = sv.g.localToWorld(new THREE.Vector3(-0.034, 0, -0.012)); base.worldToLocal(p); return [p.x * 1000, p.y * 1000, p.z * 1000]; };
    [[0, BRN, 'GND', 0.0], [1, RED, 'V+', 1.2], [2, ORG, 'сигнал PWM', 2.4]].forEach(([row, col, nm, off]) => {
      makeWire(`${sv.name}: жила ${nm} (канал ${sv.ch})`, col, 0.55, null, {
        dyn: () => { const a = pcaCh(sv.ch, row), b = anchor(); b[1] += (off - 1.2); return [a, [a[0], a[1] - 4, a[2] + 6], [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, Math.max(a[2], b[2]) + 4], b]; }
      });
    });
  }
  robot.updateMatrixWorld(true);
  wires.forEach(rebuildWire);
}
let wireTick = 0;
function updateWires() { if ((wireTick++ % 3) === 0) wires.forEach(w => { if (w.dyn) rebuildWire(w); }); }

function buildLegend() {
  const cat = m => {
    const n = m.userData.name || '';
    if (m.userData.wire) return 'Провода (наведи — название)';
    if (n.startsWith('Серво')) return 'Серво PDI-HV5523MG ×12';
    if (n.startsWith('LiPo')) return 'LiPo 2S 4000 мАч';
    if (n.startsWith('UBEC')) return 'UBEC HKU5 5V/5A';
    if (n.startsWith('LCD') || n.startsWith('I²C')) return 'LCD 16×2 I²C';
    if (n.startsWith('PCA9685')) return 'PCA9685 (CAD Adafruit)';
    if (n.startsWith('Raspberry')) return 'Raspberry Pi (CAD Pi 4B, плата как у 3B)';
    return 'Логотип и надпись';
  };
  const seen = new Map();
  pick.forEach(m => { const k = cat(m); if (!seen.has(k)) seen.set(k, m.material?.color?.getHexString?.() || '888888'); });
  document.getElementById('legend').innerHTML = [...seen].map(([n, c]) => `<label class="row"><span class="sw" style="background:#${c}"></span>${n}</label>`).join('');
}

// ---- суставы / позы ----
const NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right'];
const KIND = { shoulder: 'плечо', leg: 'бедро', foot: 'колено' };
const POSES = {
  stand: {}, sit: {},
};
function setStand(shoulder, leg, foot) { return Object.fromEntries(NAMES.flatMap(n => [[n + '_shoulder', shoulder], [n + '_leg', leg], [n + '_foot', foot]])); }
POSES.stand = setStand(0, 0.75, -1.5);
POSES.sit = { ...setStand(0, 0.2, -0.4), rear_left_leg: 1.1, rear_right_leg: 1.1, rear_left_foot: -2.1, rear_right_foot: -2.1 };
function applyPose(p) { for (const [j, v] of Object.entries(p)) setJ(j, v); settle(); }
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
function gaitStep(dt) {
  const fwd = (keys.has('KeyW') || keys.has('ArrowUp') ? 1 : 0) - (keys.has('KeyS') || keys.has('ArrowDown') ? 1 : 0);
  const turn = (keys.has('KeyA') || keys.has('ArrowLeft') ? 1 : 0) - (keys.has('KeyD') || keys.has('ArrowRight') ? 1 : 0);
  const sit = keys.has('ControlLeft') || keys.has('ControlRight') || keys.has('KeyC');
  const fast = keys.has('ShiftLeft') || keys.has('ShiftRight');
  crouch += ((sit ? 1 : 0) - crouch) * Math.min(1, dt * 7);                 // присед на Ctrl / C
  const want = ((fwd !== 0 || turn !== 0) && crouch < 0.25) ? 1 : 0;
  amp += (want - amp) * Math.min(1, dt * 6);                                // плавный старт и остановка
  const T = fast ? 0.42 : 0.55, S = fast ? 0.085 : 0.065, H = 0.035;        // период, длина шага, подъём ноги
  if (amp > 0.02) phase = (phase + dt / T) % 1;
  const dir = fwd !== 0 ? fwd : 1, stride = fwd !== 0 ? S : S * 0.45;
  const zNow = Z0 + crouch * (ZS - Z0);
  NAMES.forEach(n => {
    const left = n.endsWith('left');
    const diag = (n === 'front_left' || n === 'rear_right') ? 0 : 0.5;
    let ph = (phase + diag) % 1, x, z;
    const sgn = fwd !== 0 ? 1 : (left ? -turn : turn);
    const st = stride * amp * sgn * dir;
    if (ph < 0.5) { const p = ph / 0.5; x = X0 + st * (0.5 - p); z = zNow; }
    else { const p = (ph - 0.5) / 0.5; x = X0 + st * (-0.5 + p); z = zNow + H * amp * Math.sin(Math.PI * p); }
    const [a1, a2] = ik(x, z);
    setJ(n + '_shoulder', 0); setJ(n + '_leg', a1); setJ(n + '_foot', a2);
  });
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

// ---- слои ----
btn('cShell').onchange = e => shell.forEach(m => m.visible = e.target.checked);
btn('sGhost').oninput = e => { const v = +e.target.value / 100; btn('vGhost').textContent = Math.round(v * 100) + '%'; shell.forEach(m => { m.material.transparent = v > 0.01; m.material.opacity = 1 - v * 0.92; m.material.depthWrite = v < 0.35; m.material.needsUpdate = true; }); };
btn('cElec').onchange = e => pick.forEach(m => m.visible = e.target.checked);
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
function fit() { controls.target.set(0, 0.13, 0); return;  controls.target.set(0, rig.position.y + 0.02, 0); }

let _w=0,_h=0;
renderer.setAnimationLoop(() => {
  if (view.clientWidth !== _w || view.clientHeight !== _h) { _w = view.clientWidth; _h = view.clientHeight; resize();  }
  if (robot && mode === 'walk') { const now = performance.now(); gaitStep(Math.min(0.05, (now - last) / 1000)); last = now; }
  if (robot && wires.length) updateWires();
  controls.update(); renderer.render(scene, camera);
});

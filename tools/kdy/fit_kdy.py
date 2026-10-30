"""Перевод 3D-модели на настоящие детали KDY0523 (Thingiverse 3445283) и mike4192 (4591999, 4596267).

Меши из URDF mike4192 (3d-viewer/stl/*.stl) — это сборки деталей KDY в варианте `_mg` (под MG996R) плюс
модели серво. Каждая деталь `_mg` ложится на свою копию в URDF с ошибкой ~0.002 мм, поэтому:
  1) регистрируем деталь `_mg` на её тело в URDF-меше (ICP) — получаем точное место детали;
  2) ставим на это место деталь нашей сборки (`_cls`, `non-mega`, детали mike4192), уточняя ICP;
  3) из тел-серво в URDF-мешах снимаем гнёзда: ось вала, плоскость верха корпуса, направление корпуса.

Шаг 1–3 нужен один раз, нужны распакованные zip с Thingiverse:
  python tools/kdy/fit_kdy.py fit <папка с распакованными zip>     -> tools/kdy/fit.json
Сборка STL для 3D-модели из print/kdy и fit.json (zip не нужны):
  python tools/kdy/fit_kdy.py build                                 -> 3d-viewer/stl/kdy/*.stl, 3d-viewer/parts/servo_seats.json
pip install numpy trimesh scipy rtree
"""
import glob, json, os, sys, warnings
import numpy as np, trimesh

warnings.filterwarnings('ignore')
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
V = os.path.join(ROOT, '3d-viewer')
PRINT = os.path.join(ROOT, 'print', 'kdy')
FIT = os.path.join(os.path.dirname(__file__), 'fit.json')

# URDF-меш -> [(эталон в URDF (_mg), деталь нашей сборки)]
PLAN = {
    'larm':       [('L_arm_mg', 'L_arm_cls')],
    'rarm':       [('R_arm_mg', 'R_arm_cls')],
    'larm_cover': [('L_arm_cover', 'L_arm_cover')],
    'rarm_cover': [('R_arm_cover', 'R_arm_cover')],
    'lfoot':      [('L_wrist_mg', 'L_wrist_cls')],
    'rfoot':      [('R_wrist_mg', 'R_wrist_cls')],
    'lshoulder':  [('L_arm_joint_mg', 'L_arm_joint_cls')],
    'rshoulder':  [('R_arm_joint_mg', 'R_arm_joint_cls')],
    'mainbody':   [('I_shoulder_mg', 'I_shoulder_cls'), ('I_shoulder_mg', 'I_shoulder_cls'), ('O_shoulder', 'O_shoulder'), ('O_shoulder', 'O_shoulder'),
                   ('L_side_plate', 'non-mega_L_side_plate'), ('R_side_plate', 'R_side_plate'), ('plate', 'plate_plain')],
    'frontpart':  [('T_cover_mg', 'non-mega_T_cover_cls'), ('F_cover', 'F_cover')],
    'backpart':   [('B_cover_mg', 'non-mega_B_cover_cls'), ('R_cover', 'R_cover')],
}
SERVO_VOL = 30.4e3          # тело-серво в URDF-мешах, мм³


def load_dir(d):
    return {os.path.basename(f).rsplit('.', 1)[0]: f for f in glob.glob(os.path.join(d, '**', '*.[sS][tT][lL]'), recursive=True)}


def dist(src, dst, M, n=1500):
    p = trimesh.transform_points(src.sample(n, seed=1), M)
    return trimesh.proximity.closest_point(dst, p)[1]


def register(src, dst, init=None):
    """src -> dst; перебор mesh_other и ICP от init, берём лучшее по медиане расстояний"""
    cands = []
    if init is not None:
        M, _, _ = trimesh.registration.icp(src.sample(3000, seed=2), dst, initial=np.array(init), max_iterations=60)
        cands.append(M); cands.append(np.array(init))
    M, _ = trimesh.registration.mesh_other(src, dst, samples=1000, scale=False, icp_first=20, icp_final=100)
    cands.append(M)
    scored = [(float(np.median(dist(src, dst, M))), M) for M in cands]
    return min(scored, key=lambda s: s[0])


def center_plate(src, dst, M):
    """гладкая плита по ICP скользит в своей плоскости — ставим её по центру плиты KDY, низ к низу"""
    p = src.copy(); p.apply_transform(M)
    d = dst.bounds.mean(0) - p.bounds.mean(0); k = np.argmin(p.extents)
    d[k] = dst.bounds[0][k] - p.bounds[0][k]
    M = np.array(M); M[:3, 3] += d; return M


def servo_seat(b):
    """тело-серво -> {o: точка на оси вала в плоскости верха корпуса, z: вал, x: от корпуса к валу, ext}"""
    T, ext = trimesh.bounds.oriented_bounds(b)          # T: мир -> OBB (центр в 0)
    v = trimesh.transform_points(b.vertices, T)
    order = np.argsort(ext)                              # ширина, высота, длина
    w, h, l = order[0], order[1], order[2]
    ends = []
    for s in (1, -1):                                    # конец по высоте с валом — где сечение маленькое
        top = v[v[:, h] * s > ext[h] / 2 - 2]
        ends.append((np.ptp(top[:, l]) * np.ptp(top[:, w]), s, top))
    _, s, top = min(ends, key=lambda e: e[0])
    c = top.mean(0)
    far = v[np.abs(v[:, l] - c[l]) > 15]                 # верх корпуса — по вершинам вдали от вала
    ztop = (far[:, h] * s).max() * s
    o = np.zeros(3); o[l] = c[l]; o[w] = c[w]; o[h] = ztop
    zx = np.zeros(3); zx[h] = s
    xx = np.zeros(3); xx[l] = np.sign(c[l] - v[:, l].mean())   # от центра корпуса к валу
    Ti = np.linalg.inv(T)
    O = trimesh.transform_points([o], Ti)[0]
    Z = Ti[:3, :3] @ zx; X = Ti[:3, :3] @ xx
    return {'o': O.round(3).tolist(), 'z': Z.round(5).tolist(), 'x': X.round(5).tolist(),
            'case_top_to_shaft_end': float(abs(ext[h] / 2 - ztop * s)), 'shaft_from_case_end': float(ext[l] / 2 - abs(c[l]))}


def fit(things):
    K = load_dir(things); K.update(load_dir(PRINT))
    kdy = {n: trimesh.load(p) for n, p in K.items()}
    out = {'parts': {}, 'servos': {}}
    for rv, plan in PLAN.items():
        m = trimesh.load(os.path.join(V, 'stl', rv + '.stl'))
        bodies = [b for b in m.split(only_watertight=False) if abs(b.volume) > 1e3]
        out['servos'][rv] = [servo_seat(b) for b in bodies if abs(abs(b.volume) - SERVO_VOL) < 300]
        used, rows = set(), []
        for ref, build in plan:
            r = kdy[ref]
            cand = [i for i, b in enumerate(bodies) if i not in used and abs(abs(b.volume) - r.volume) < 150] \
                or [i for i, b in enumerate(bodies) if i not in used and abs(abs(b.volume) - SERVO_VOL) > 300]   # крышка бедра в URDF — старая версия
            best = None
            for i in cand:
                med, M = register(r, bodies[i])
                if best is None or med < best[0]: best = (med, M, i)
            med, M, i = best; used.add(i)
            medb, Mb = register(kdy[build], bodies[i], init=M) if build != ref else (med, M)
            if build == 'plate_plain': Mb = center_plate(kdy[build], bodies[i], Mb)
            print(f'{rv:10s} {ref:16s} -> тело {i}: {med:.3f} мм;  {build:22s}: {medb:.3f} мм')
            rows.append({'ref': ref, 'part': build, 'M': np.round(Mb, 6).tolist(), 'err_ref': round(med, 4), 'err_part': round(medb, 4)})
        out['parts'][rv] = rows
        for s in out['servos'][rv]: print(f'{rv:10s} серво: вал {s["o"]}, верх корпуса→торец вала {s["case_top_to_shaft_end"]:.1f} мм')
    json.dump(out, open(FIT, 'w'), indent=1, ensure_ascii=False)


def build():
    F = json.load(open(FIT)); P = load_dir(PRINT)
    os.makedirs(os.path.join(V, 'stl', 'kdy'), exist_ok=True)
    for rv, rows in F['parts'].items():
        parts = []
        for r in rows:
            p = trimesh.load(P[r['part']]); p.apply_transform(np.array(r['M'])); parts.append(p)
        trimesh.util.concatenate(parts).export(os.path.join(V, 'stl', 'kdy', rv + '.stl'))
    json.dump(F['servos'], open(os.path.join(V, 'parts', 'servo_seats.json'), 'w'), indent=1)
    print('готово:', ', '.join(F['parts']))


if __name__ == '__main__':
    if sys.argv[1] == 'fit': fit(sys.argv[2])
    else: build()

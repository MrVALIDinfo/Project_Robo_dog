"""Кронштейн камеры Raspberry Pi Camera v2 в «морду» Spot Micro.

Камера (25 мм) не входит между двумя рёбрами морды (зазор 24.5 мм), поэтому кронштейн
надевается на передние кромки рёбер С-образными пазами, а камера стоит на клине перед ними
и смотрит вперёд с наклоном 10° вниз. Камера крепится 4 винтами M2x6, гайки утоплены в бобышки.

Пишет:
  3d-viewer/parts/cam_mount.stl   — кронштейн в системе base_link (мм), для 3D-модели
  3d-viewer/parts/layout.json     — поза камеры для 3D-модели
  print/cam_mount.stl             — тот же кронштейн, уложенный боком для печати
Нужно: pip install numpy trimesh manifold3d networkx
"""
import json, os, sys
import numpy as np, trimesh

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
V = os.path.join(ROOT, '3d-viewer')

# рёбра морды (frontpart.stl в base_link, мм): передняя кромка x = FX0 + K*z, толщина по Y
FX0, K = 142.0, 0.4886
FIN_R, FIN_L = (12.8, 16.0), (-15.0, -11.7)
YC = (FIN_R[0] + FIN_L[1]) / 2            # середина между рёбрами (STL смещён на 0.55 мм)
Z0, TILT = -2.0, np.radians(10)           # высота центра, наклон камеры вниз
CLR, WALL = 0.25, 1.6                     # зазор паза, стенка паза
CAM_W, CAM_H = 25.0, 23.86                # плата камеры v2
HOLE_DY, HOLE_B = 10.5, (CAM_H / 2 - 2.0, CAM_H / 2 - 14.5)   # отверстия 21 x 12.5, 2 мм от верхнего края

unit = lambda v: np.asarray(v, float) / np.linalg.norm(v)
u = unit([K, 0, 1]); n = unit([1, 0, -K]); Y = np.array([0., 1, 0])
c = np.array([np.cos(TILT), 0, -np.sin(TILT)]); vv = np.array([np.sin(TILT), 0, np.cos(TILT)])
O_F = np.array([FX0 + K * Z0, 0, Z0])

def frame(o, ax):                          # ax: столбцы локальных осей -> мировая матрица
    M = np.eye(4); M[:3, :3] = np.column_stack(ax); M[:3, 3] = o; return M
F = frame(O_F, (n, Y, u))                  # (t, y, s): t — вперёд от кромки рёбер, s — вдоль ребра

def box(lo, hi, M):
    lo, hi = np.array(lo, float), np.array(hi, float)
    b = trimesh.creation.box(extents=hi - lo); b.apply_translation((lo + hi) / 2); b.apply_transform(M); return b

def cyl(r, a0, a1, y, b, M, sections=32):  # цилиндр вдоль локальной оси a
    m = trimesh.creation.cylinder(radius=r, height=a1 - a0, sections=sections)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    m.apply_translation([(a0 + a1) / 2, y, b]); m.apply_transform(M); return m

S0, S1 = -15.0, 15.0
fin_plate = box([0, FIN_L[0] - CLR - WALL, S0], [2.5, FIN_R[1] + CLR + WALL, S1], F)
channels = [box([-4, y0, S0], [0, y1, S1], F) for y0, y1 in (
    (FIN_R[0] - CLR - WALL, FIN_R[0] - CLR), (FIN_R[1] + CLR, FIN_R[1] + CLR + WALL),
    (FIN_L[0] - CLR - WALL, FIN_L[0] - CLR), (FIN_L[1] + CLR, FIN_L[1] + CLR + WALL))]

# камера: a — по оси объектива (0 = задняя сторона платы), y, b — вверх по плате
PW, PH = CAM_W / 2 + 1.3, CAM_H / 2 + 1.3
def cam_frame(d):
    return frame(O_F + n * d, (c, Y, vv))
def plate_corners_t(d):
    C = cam_frame(d)
    pts = [C[:3, :3] @ [-4, YC + sy * PW, sb * PH] + C[:3, 3] for sy in (-1, 1) for sb in (-1, 1)]
    return min((p - O_F) @ n for p in pts)
d = 2.5
while plate_corners_t(d) < 2.5: d += 0.05   # клин касается пластины на рёбрах ближним углом
C = cam_frame(d)
cam_plate = box([-4, YC - PW, -PH], [-2, YC + PW, PH], C)
wedge = trimesh.convex.convex_hull(np.vstack([fin_plate.vertices, cam_plate.vertices]))

holes = [(YC + sy * HOLE_DY, hb) for sy in (-1, 1) for hb in HOLE_B]
bosses = [cyl(3.25, -2, 0, y, b, C) for y, b in holes]
solid = trimesh.boolean.union([wedge, *channels, *bosses], engine='manifold')
cuts = [cyl(1.1, -12, 0.5, y, b, C, 24) for y, b in holes]                                  # M2 насквозь
cuts += [cyl(4.1 / np.sqrt(3), -1.7, 0.5, y, b, C, 6) for y, b in holes]                     # гайки M2 в бобышках
cuts += [box([-14, YC - 11.5, -PH - 8], [0.5, YC + 11.5, -CAM_H / 2 + 6.0], C)]            # разъём CSI и выход шлейфа вниз
mount = trimesh.boolean.difference([solid, *cuts], engine='manifold')

# проверка: кронштейн не должен входить в корпус
sys.path.insert(0, os.path.dirname(__file__))
def shell():
    out = []
    for f, xyz, yaw in (('kdy/mainbody.stl', (-42, -55, -10), 0), ('kdy/backpart_alatoo.stl', (40, 55, -10), np.pi), ('kdy/frontpart.stl', (40, 55, -10), np.pi)):
        m = trimesh.load(os.path.join(V, 'stl', f)); M = trimesh.transformations.rotation_matrix(yaw, [0, 0, 1]); M[:3, 3] = xyz
        m.apply_transform(M); out.append(m)
    return out
cam_board = box([0, YC - CAM_W / 2, -CAM_H / 2], [1, YC + CAM_W / 2, CAM_H / 2], C)
for name, part in (('кронштейн', mount), ('плата камеры', cam_board)):
    vol = sum(trimesh.boolean.intersection([part, s], engine='manifold').volume for s in shell())
    print(f'{name}: пересечение с корпусом {vol:.2f} мм3')
print(f'кронштейн: объём {mount.volume / 1000:.2f} см3, габарит {np.round(mount.extents, 1).tolist()} мм, watertight={mount.is_watertight}')

mount.export(os.path.join(V, 'parts', 'cam_mount.stl'))
def print_pose(m):                          # плоская грань на стол — та, при которой меньше всего нависаний круче 45°
    best = None
    for i in np.argsort(m.facets_area)[::-1][:12]:
        q = m.copy(); q.apply_transform(trimesh.geometry.align_vectors(m.facets_normal[i], [0, 0, -1])); q.apply_translation(-q.bounds[0])
        over = q.area_faces[(q.face_normals[:, 2] < -np.cos(np.radians(45))) & (q.triangles_center[:, 2] > 0.3)].sum()
        if best is None or over < best[0]: best = (over, q)
    return best
over, p = print_pose(mount)
p.export(os.path.join(ROOT, 'print', 'cam_mount.stl'))
print(f'печать: габарит {np.round(p.extents, 1).tolist()} мм, нависаний круче 45° {over:.0f} мм2 (отверстия и гнёзда гаек — мосты)')
json.dump({'camera': {'origin': np.round(C[:3, 3], 3).tolist(), 'axis': np.round(c, 5).tolist(), 'up': np.round(vv, 5).tolist(),
                      'yc': round(YC, 3), 'w': CAM_W, 'h': CAM_H, 'holes': [[round(y, 3), round(b, 3)] for y, b in holes]}},
          open(os.path.join(V, 'parts', 'layout.json'), 'w'), indent=1)
print('камера: центр задней стороны платы', np.round(C[:3, 3], 2).tolist())

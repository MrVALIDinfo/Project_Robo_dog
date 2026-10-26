"""Задняя крышка Spot Micro (backpart.stl): заливка гравировки «K.D.Y» и надпись «ALA-TOO».

На задней стенке под окном LCD вырезана надпись K.D.Y (местами насквозь). Скрипт заполняет
её телом стенки (между наружной и внутренней плоскостью стенки) и выдавливает «ALA-TOO» на 0.8 мм.

Пишет:
  3d-viewer/stl/backpart_filled.stl — крышка без гравировки (координаты исходного STL), для 3D-модели
  3d-viewer/parts/rear_text.stl     — надпись в системе base_link (мм), в 3D-модели красится отдельно
  print/backpart_alatoo.stl         — крышка с надписью одним телом, для печати
Нужно: pip install numpy trimesh manifold3d networkx shapely matplotlib
"""
import os
import numpy as np, trimesh
from shapely.geometry import Polygon
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
V = os.path.join(ROOT, '3d-viewer')
TEXT, CAP, RELIEF = 'ALA-TOO', 6.0, 0.8        # надпись, высота заглавных, выступ (мм)
YC, ZC = 9.0, -20.5                           # центр надписи на стенке (base_link, мм)
Y0, Y1, Z0, Z1 = -6.0, 35.0, -27.0, -15.5     # область заливки: гравировка y -1.3…32.6, z -25.3…-17.6

# исходная деталь -> base_link (как в URDF: поворот 180° вокруг Z, сдвиг 40, 55, -10 мм)
T = trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1]); T[:3, 3] = [40, 55, -10]
src = trimesh.load(os.path.join(V, 'stl', 'backpart.stl')); part = src.copy(); part.apply_transform(T)

# плоскости стенки: лучи изнутри назад по точкам вне гравировки и вне отверстия выключателя (y -35…-16)
pts_o, pts_i = [], []
for y in (-9, -7, 34, 36):
    for z in (-28, -24, -18, -14):
        h = part.ray.intersects_location([[-120, y, z]], [[-1, 0, 0]])[0]
        xs = sorted(h[:, 0]) if len(h) else []
        if len(xs) >= 2: pts_o.append((xs[0], y, z)); pts_i.append((xs[-1], y, z))
def fit(p):                                   # x = a + b*y + c*z
    p = np.array(p); A = np.c_[np.ones(len(p)), p[:, 1], p[:, 2]]
    k, res, *_ = np.linalg.lstsq(A, p[:, 0], rcond=None); return k, np.abs(A @ k - p[:, 0]).max()
(ko, eo), (ki, ei) = fit(pts_o), fit(pts_i)
xo = lambda y, z: ko[0] + ko[1] * y + ko[2] * z
xi = lambda y, z: ki[0] + ki[1] * y + ki[2] * z
print(f'стенка: снаружи x={ko[0]:.2f}{ko[1]:+.4f}y{ko[2]:+.4f}z (откл. {eo:.2f} мм), толщина {np.mean([xi(y, z) - xo(y, z) for _, y, z in pts_o]):.2f} мм')

# заливка: призма между плоскостями (на 0.01 мм внутрь, чтобы не было совпадающих граней)
c = [(Y0, Z0), (Y1, Z0), (Y1, Z1), (Y0, Z1)]
v = [(xo(y, z) + 0.01, y, z) for y, z in c] + [(xi(y, z) - 0.01, y, z) for y, z in c]
fill = trimesh.convex.convex_hull(np.array(v))
filled = trimesh.boolean.union([part, fill], engine='manifold')

def small_loops(m):
    out = []
    for x in (-148, -147.5, -147, -146.5, -146):
        s = m.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
        for e in (s.discrete if s is not None else []):
            mn, mx = e.min(0), e.max(0)
            if mx[1] > Y0 and mn[1] < Y1 and mx[2] > Z0 and mn[2] < Z1 and (mx[1] - mn[1]) < 30: out.append(round(x, 1))
    return out
print('контуры гравировки до:', len(small_loops(part)), 'после:', len(small_loops(filled)))

# надпись: контуры шрифта -> полигоны с дырками -> выдавливание; оси: вправо = -Y, вверх — по стенке, наружу = -X
tp = TextPath((0, 0), TEXT, size=1, prop=FontProperties(family='DejaVu Sans', weight='bold'))
rings = [Polygon(r) for r in tp.to_polygons() if len(r) >= 3]
rings.sort(key=lambda p: -p.area)
glyphs = []
for r in rings:                               # чётно-нечётное правило: кольцо внутри другого — дырка
    host = next((g for g in glyphs if g.contains(r)), None)
    if host is None: glyphs.append(r)
    else: glyphs[glyphs.index(host)] = host.difference(r)
b = np.array([g.bounds for g in glyphs]); xmin, xmax, ymax = b[:, 0].min(), b[:, 2].max(), b[:, 3].max()
k = CAP / ymax
n = np.array([-1, -ko[1], -ko[2]]); n /= np.linalg.norm(n)            # наружная нормаль стенки
u = np.array([0., -1, 0]); u -= n * (u @ n); u /= np.linalg.norm(u)
w = np.cross(n, u)                                                    # вверх вдоль стенки
O = np.array([xo(YC, ZC), YC, ZC])
M = np.eye(4); M[:3, :3] = np.column_stack([u, w, n]); M[:3, 3] = O
parts = []
for g in glyphs:
    m = trimesh.creation.extrude_polygon(g, height=RELIEF + 0.3)
    m.apply_translation([-(xmin + xmax) / 2, -ymax / 2, -0.3]); m.apply_scale([k, k, 1]); parts.append(m)
text = trimesh.util.concatenate(parts); text.apply_transform(M)
print(f'надпись «{TEXT}»: {text.extents.round(1).tolist()} мм, y {text.bounds[0][1]:.1f}…{text.bounds[1][1]:.1f}, z {text.bounds[0][2]:.1f}…{text.bounds[1][2]:.1f}')

Ti = np.linalg.inv(T)
out = filled.copy(); out.apply_transform(Ti); out.export(os.path.join(V, 'stl', 'backpart_filled.stl'))
text.export(os.path.join(V, 'parts', 'rear_text.stl'))
pr = trimesh.boolean.union([filled, text], engine='manifold'); pr.apply_transform(Ti)
pr.export(os.path.join(ROOT, 'print', 'backpart_alatoo.stl'))
print('watertight:', out.is_watertight, pr.is_watertight)

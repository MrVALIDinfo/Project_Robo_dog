"""Cardboard Robo-dog (SpotMicro proportions, 1:1) — flat parts with tabs/slots.

World frame (mm): x forward, y left, z up. Hip axes at x=+-93, z=0; knee 107.5 below hip.
Плоскости лап по ширине (y) — по настоящим деталям KDY0523 в 3D-модели (3d-viewer/stl/kdy).
Every part is a flat polygon in its own (u, v) frame, extruded by its thickness along n = e1 x e2.
"""
import numpy as np
from shapely.geometry import Polygon, Point, box, LineString
from shapely.ops import unary_union
from shapely import affinity

T = 3.0          # cardboard thickness = slot width
PIN_D = 4.0      # hole for bamboo skewer 3-4 mm
EPS = 0.02

LEGS = {  # code: (name, front?, left?)
    'ПЛ': ('передняя левая', +1, +1),
    'ПП': ('передняя правая', +1, -1),
    'ЗЛ': ('задняя левая', -1, +1),
    'ЗП': ('задняя правая', -1, -1),
}
HIP_X, KNEE_DZ = 93.0, -107.5

# body side profile (XZ), CCW, from mainbody/frontpart/backpart silhouettes
HEX = [(-146, -37), (141, -37), (169, 20), (150, 35), (-138, 35), (-151, 25)]
WALL_OUT = 53.0          # outer face of side walls
PANEL_HALF = 57.0        # half-width of top/bottom/end panels (4 mm lip outside walls)
RIB_X = (45.0, -45.0)

# leg profiles (XZ, front-left leg, legs straight down), simplified from STL silhouettes
SHOULDER = box(76, -24, 110, 21).buffer(5, quad_segs=6)
UPPER = Polygon([(84, 24), (100, 25), (110, 17), (97, -84), (104, -104), (100, -115), (92, -120), (84, -118),
                 (67, -90), (73, -37)]).buffer(-3, join_style='mitre').buffer(3, quad_segs=6)
LOWER = Polygon([(85, -88), (100, -88), (104, -92), (104, -96), (102, -98), (103, -137), (106, -140), (108, -172),
                 (108, -225), (110, -233), (100, -241), (87, -241), (77, -234), (74, -218), (80, -216), (91, -203),
                 (93, -197), (92, -165), (79, -129), (79, -96)]).buffer(-2, join_style='mitre').buffer(2, quad_segs=6)
# y slabs (outer-face coordinates) for the front-left leg — по настоящим деталям KDY (стойка, передняя левая):
#   бедро (arm + cover) y 76.9…112.0, ось бедра y = 94; голень (wrist) y 82.7…103 (у колена серво до 120);
#   кронштейн плеча y 27.5…65 (наполовину внутри корпуса) — в картоне плечо снаружи борта, от борта до бедра
SH_Y = ((53, 56), (71, 74))
UP_Y = ((77, 80), (109, 112))
LO_Y = ((83, 86), (100, 103))
def fill(y0, y1):            # шайбы толщиной T, заполняющие зазор y0…y1 (outer-face y каждой)
    n = int(round((y1 - y0) / T)); return [y0 + i * T for i in range(n)]
KNEE_GAPS = fill(UP_Y[0][1], LO_Y[0][0]) + fill(LO_Y[1][1], UP_Y[1][0])   # между бедром и голенью
HIP_GAPS = fill(SH_Y[1][1], UP_Y[0][0])                                 # между плечом и бедром
KNEE_PIN = (UP_Y[0][0] - T, UP_Y[1][1] + T)                              # шпажка колена: от заглушки до заглушки
AXLE = 2 * (UP_Y[1][1] + T) + 4                                          # шпажка-ось через всю собаку, мм
KNEE_PIN_MM = round(KNEE_PIN[1] - KNEE_PIN[0] + 4)                       # с запасом 2 мм на каждую заглушку
# washer codes: Ш1 — заглушка колена внутри, Ш2…Шk — между бедром и голенью, затем заглушка колена снаружи, шайбы плечо–бедро, заглушка оси
W_KNEE_IN, W_KNEE_GAP = 1, list(range(2, 2 + len(KNEE_GAPS)))
W_KNEE_OUT = 2 + len(KNEE_GAPS)
W_HIP_GAP = list(range(W_KNEE_OUT + 1, W_KNEE_OUT + 1 + len(HIP_GAPS)))
W_HIP_OUT = W_KNEE_OUT + 1 + len(HIP_GAPS)
SH_SPACERS, UP_SPACERS, LO_SPACERS = (18.0, -21.0), (14.0, -25.0, -60.0), (-95.0, -140.0, -226.0)


def unit(v):
    v = np.asarray(v, float); return v / np.linalg.norm(v)


class Part:
    def __init__(self, code, title, poly, O, e1, e2, thick=T, group='body', leg=None, label_n=None,
                 kind='plate', qty_note='', marks=None):
        self.code, self.title, self.poly = code, title, poly
        self.O, self.e1, self.e2 = np.asarray(O, float), unit(e1), unit(e2)
        self.n = np.cross(self.e1, self.e2)
        self.thick, self.group, self.leg, self.kind = thick, group, leg, kind
        self.label_n = unit(label_n) if label_n is not None else self.n
        self.tabs_into = []      # codes of parts this part's tabs go into
        self.marks = marks or []  # (shapely geom in uv, text) drawn dashed on the cut sheet
        self.qty_note = qty_note

    def w(self, uv):
        uv = np.asarray(uv, float)
        return self.O + uv[..., :1] * self.e1 + uv[..., 1:2] * self.e2

    def uv(self, P):
        d = np.asarray(P, float) - self.O
        return np.stack([d @ self.e1, d @ self.e2], -1)


def add_tabs(poly, A, B, outward, n=None, length=None, depth=T, centers=None):
    """Union rectangular tabs onto edge A->B, sticking out along `outward` by depth."""
    A, B, o = np.array(A, float), np.array(B, float), unit(outward)
    L = np.linalg.norm(B - A); d = (B - A) / L
    if centers is None:
        n = n or (1 if L < 70 else 2 if L < 170 else 3)
        centers = [(k + 0.5) / n * L for k in range(n)]
    tl = length or min(22.0, 0.35 * L / max(1, len(centers)) * (1 if len(centers) == 1 else 1.6))
    tabs = []
    for c in centers:
        p0, p1 = A + d * (c - tl / 2), A + d * (c + tl / 2)
        tabs.append(Polygon([p0, p1, p1 + o * depth, p0 + o * depth]))
    return unary_union([poly] + tabs)


def inset_hex(off):
    """Offset each HEX edge inward by `off`; returns vertices (same order)."""
    P = np.array(HEX, float); m = len(P); lines = []
    for i in range(m):
        a, b = P[i], P[(i + 1) % m]; d = unit(b - a); o = np.array([d[1], -d[0]])  # outward for CCW
        lines.append((a - o * off, d))
    out = []
    for i in range(m):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        t = np.linalg.solve(np.array([d1, -d2]).T, p2 - p1)
        out.append(p1 + d1 * t[0])
    return np.array(out)


def spacer_poly(profile, zc, y0, y1, margin=3.0):
    """Horizontal spacer between plates at outer-face y0..y1 (plates T thick), crossing profile at height zc."""
    # usable x range = where the full slab height is inside the profile
    lo = max(profile.intersection(LineString([(-1e3, z), (1e3, z)])).bounds[0] for z in (zc - T / 2, zc + T / 2))
    hi = min(profile.intersection(LineString([(-1e3, z), (1e3, z)])).bounds[2] for z in (zc - T / 2, zc + T / 2))
    x0, x1 = lo + margin, hi - margin
    p = box(x0, y0 + T, x1, y1 - T)
    L = x1 - x0
    centers = [L / 2] if L < 26 else [L * 0.27, L * 0.73]
    tl = min(11.0, L * (0.5 if len(centers) == 1 else 0.3))
    p = add_tabs(p, (x0, y0 + T), (x1, y0 + T), (0, -1), centers=centers, length=tl)
    p = add_tabs(p, (x0, y1 - T), (x1, y1 - T), (0, 1), centers=centers, length=tl)
    return p


def disc(r, c=(0, 0)):
    return Point(c).buffer(r, quad_segs=12)


def build():
    parts = []
    # ---------------- body
    hexp = Polygon(HEX)
    ins = inset_hex(T)
    wall = Polygon(ins)
    for i in range(len(ins)):
        a, b = ins[i], ins[(i + 1) % len(ins)]
        d = unit(b - a); o = (d[1], -d[0])
        wall = add_tabs(wall, a, b, o)
    for side, code, nm in ((+1, 'К1', 'Борт ЛЕВЫЙ'), (-1, 'К2', 'Борт ПРАВЫЙ')):
        legs = [k for k, v in LEGS.items() if v[2] == side]
        marks = []
        for k in legs:
            fx = LEGS[k][1]
            marks.append((affinity.translate(SHOULDER, xoff=0 if fx > 0 else -2 * HIP_X), f'плечо {k}'))
        p = Part(code, nm, wall, (0, side * (WALL_OUT - T / 2), 0), (1, 0, 0), (0, 0, 1),
                 label_n=(0, side, 0), marks=marks)
        p.tabs_into = ['К3', 'К4', 'К5', 'К6', 'К7', 'К8']
        parts.append(p)
    pnames = [('К3', 'Днище'), ('К6', 'Нос нижний'), ('К5', 'Нос верхний'), ('К4', 'Крыша (съёмная)'),
              ('К7', 'Корма верхняя'), ('К8', 'Корма нижняя')]
    H = np.array(HEX, float)
    for i, (code, nm) in enumerate(pnames):
        a, b = ins[i], ins[(i + 1) % 6]
        d = unit(b - a); o = np.array([d[1], -d[0]])
        L = np.linalg.norm(b - a)
        a3 = np.array([a[0], 0, a[1]]) + np.array([o[0], 0, o[1]]) * T / 2
        poly = box(0, -PANEL_HALF, L, PANEL_HALF)
        marks = []
        if code == 'К8':
            marks.append((None, 'ALA-TOO'))
        parts.append(Part(code, nm, poly, a3, (d[0], 0, d[1]), (0, 1, 0), label_n=(o[0], 0, o[1]), marks=marks))
    ztop, zbot = 35 - T, -37 + T
    yi = WALL_OUT - T
    for x, code, nm in ((RIB_X[0], 'К9', 'Перегородка передняя'), (RIB_X[1], 'К10', 'Перегородка задняя')):
        r = box(-yi, zbot, yi, ztop)
        r = add_tabs(r, (-yi, zbot), (-yi, ztop), (-1, 0), centers=[0.3 * (ztop - zbot), 0.7 * (ztop - zbot)], length=12)
        r = add_tabs(r, (yi, zbot), (yi, ztop), (1, 0), centers=[0.3 * (ztop - zbot), 0.7 * (ztop - zbot)], length=12)
        r = add_tabs(r, (-yi, ztop), (yi, ztop), (0, 1), centers=[2 * yi * 0.22, 2 * yi * 0.78], length=16)
        r = add_tabs(r, (-yi, zbot), (yi, zbot), (0, -1), centers=[2 * yi * 0.22, 2 * yi * 0.78], length=16)
        r = r.difference(box(-28, -13, 28, 11).buffer(4, quad_segs=6))   # cable window
        p = Part(code, nm, r, (x, 0, 0), (0, 1, 0), (0, 0, 1), label_n=(np.sign(x), 0, 0))
        p.tabs_into = ['К1', 'К2', 'К3', 'К4']
        parts.append(p)

    # ---------------- legs
    for leg, (lname, fx, side) in LEGS.items():
        dx = 0 if fx > 0 else -2 * HIP_X
        g = dict(group='leg', leg=leg)
        out = (0, side, 0)

        def plate(code, title, prof, yslab):
            yc = side * (yslab[0] + yslab[1]) / 2
            return Part(f'{leg}-{code}', title, affinity.translate(prof, xoff=dx), (0, yc, 0), (1, 0, 0), (0, 0, 1),
                        label_n=out, **g)

        def spacer(code, title, prof, zc, slabs, into):
            sp = spacer_poly(prof, zc, slabs[0][0], slabs[1][1])
            sp = affinity.translate(sp, xoff=dx)
            if side < 0:
                sp = affinity.scale(sp, 1, -1, origin=(0, 0))
            p = Part(f'{leg}-{code}', title, sp, (0, 0, zc), (1, 0, 0), (0, 1, 0), label_n=(0, 0, 1), kind='spacer', **g)
            p.tabs_into = [f'{leg}-{c}' for c in into]
            return p

        parts += [plate('П1', 'Плечо, к корпусу', SHOULDER, SH_Y[0]), plate('П2', 'Плечо, наружная', SHOULDER, SH_Y[1])]
        parts += [spacer(f'П{3 + i}', 'Распорка плеча', SHOULDER, z, SH_Y, ['П1', 'П2']) for i, z in enumerate(SH_SPACERS)]
        parts += [plate('Б1', 'Бедро, внутренняя', UPPER, UP_Y[0]), plate('Б2', 'Бедро, наружная', UPPER, UP_Y[1])]
        parts += [spacer(f'Б{3 + i}', 'Распорка бедра', UPPER, z, UP_Y, ['Б1', 'Б2']) for i, z in enumerate(UP_SPACERS)]
        parts += [plate('Г1', 'Голень, внутренняя', LOWER, LO_Y[0]), plate('Г2', 'Голень, наружная', LOWER, LO_Y[1])]
        parts += [spacer(f'Г{3 + i}', 'Распорка голени', LOWER, z, LO_Y, ['Г1', 'Г2']) for i, z in enumerate(LO_SPACERS)]
        # шайбы: колено (заглушка, зазоры бедро–голень, заглушка), ось бедра (зазор плечо–бедро, заглушка снаружи)
        hx, kz = fx * HIP_X, KNEE_DZ
        wy = [(kz, KNEE_PIN[0], 'колено, заглушка внутри')] + [(kz, y, 'колено, между бедром и голенью') for y in KNEE_GAPS] + \
             [(kz, UP_Y[1][1], 'колено, заглушка снаружи')] + [(0.0, y, 'ось бедра, между плечом и бедром') for y in HIP_GAPS] + \
             [(0.0, UP_Y[1][1], 'ось бедра, заглушка снаружи')]
        for j, (z, y0, t) in enumerate(wy):
            yc = side * (y0 + T / 2)
            parts.append(Part(f'{leg}-Ш{j + 1}', f'Шайба: {t}', disc(8), (hx, yc, z), (1, 0, 0), (0, 0, 1), kind='washer',
                              label_n=out, **g))
        # skewers (not cut): knee pin
        parts.append(Part(f'{leg}-шп', f'Шпажка колена {KNEE_PIN_MM} мм', disc(PIN_D / 2 - 0.3, (0, 0)),
                          (hx, side * (KNEE_PIN[0] + KNEE_PIN[1]) / 2, kz), (1, 0, 0), (0, 0, 1), thick=KNEE_PIN_MM, kind='pin', **g))
    for fx, nm in ((+1, 'перед'), (-1, 'зад')):
        parts.append(Part(f'ОСЬ-{nm}', f'Шпажка-ось ({nm}) {AXLE:.0f} мм', disc(PIN_D / 2 - 0.3), (fx * HIP_X, 0, 0),
                          (1, 0, 0), (0, 0, 1), thick=AXLE, kind='pin', group='pin'))
    for p in parts:
        if p.kind == 'pin' and p.leg:
            p.group = 'leg'
    cut_slots(parts)
    cut_pins(parts)
    return parts


def cut_slots(parts):
    by = {p.code: p for p in parts}
    for A in parts:
        for bc in A.tabs_into:
            B = by[bc]
            g = np.array([A.e1 @ B.n, A.e2 @ B.n])
            if np.linalg.norm(g) < 1e-6:
                continue
            c0 = (A.O - B.O) @ B.n
            h = B.thick / 2 - EPS
            gn = g / np.linalg.norm(g); t = np.array([-gn[1], gn[0]])
            p0 = -c0 * g / (g @ g); w = h / np.linalg.norm(g)
            band = Polygon([p0 - gn * w - t * 2e3, p0 - gn * w + t * 2e3, p0 + gn * w + t * 2e3, p0 + gn * w - t * 2e3])
            inter = A.poly.intersection(band)
            comps = getattr(inter, 'geoms', [inter])
            for c in comps:
                if c.is_empty or c.area < 0.5:
                    continue
                P = A.w(np.array(c.exterior.coords))
                P = np.concatenate([P + A.n * A.thick / 2, P - A.n * A.thick / 2])
                uv = B.uv(P)
                slot = Polygon(uv).convex_hull.intersection(B.poly)
                if slot.area > 0.5:
                    B.poly = B.poly.difference(slot.buffer(0.05, join_style='mitre'))


def cut_pins(parts):
    pins = [p for p in parts if p.kind == 'pin']
    for pin in pins:
        axis, c = pin.n, pin.O
        for B in parts:
            if B.kind == 'pin' or abs(B.n @ axis) < 0.99:
                continue
            s = (B.O - c) @ axis  # where the pin crosses B's mid-plane
            if abs(s) > pin.thick / 2:
                continue
            q = B.uv(c + axis * s)
            if B.poly.buffer(-0.5).contains(Point(q)):
                B.poly = B.poly.difference(disc(PIN_D / 2, q))


def pose(parts, hip=0.0, knee=0.0):
    """4x4 transform per part for given hip/knee pitch (rad, about +y), legs only."""
    def rot_y(a, cen):
        c, s = np.cos(a), np.sin(a)
        M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        C = np.eye(4); C[:3, 3] = cen; Ci = np.eye(4); Ci[:3, 3] = -np.asarray(cen)
        return C @ M @ Ci
    out = {}
    for p in parts:
        M = np.eye(4)
        if p.leg:
            code = p.code.split('-')[1]
            fx = LEGS[p.leg][1]
            hipc = (fx * HIP_X, 0, 0); kneec = (fx * HIP_X, 0, KNEE_DZ)
            if code[0] in 'БГШ' or code == 'шп':
                M = rot_y(hip, hipc)
                if code[0] == 'Г':
                    M = M @ rot_y(knee, kneec)
        out[p.code] = M
    return out


if __name__ == '__main__':
    ps = build()
    for p in ps:
        print(p.code, p.title, round(p.poly.area), len(getattr(p.poly, 'interiors', [])), p.poly.geom_type,
              np.round(p.poly.bounds, 1))

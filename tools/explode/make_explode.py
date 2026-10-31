"""Данные для взрыв-схемы в 3d-viewer (explode.js).

1) Раскладывает склеенные URDF-меши (mainbody, frontpart, backpart, ...) обратно на отдельные детали:
   3d-viewer/stl/parts/<меш>__<n>__<деталь>.stl — в системе координат своего URDF-меша (мм).
2) Ищет в деталях отверстия под крепёж (срезы с шагом 0.5 мм по трём осям: круглые отверстия Ø3.2/4.2/5.2,
   шестигранные/прямоугольные гнёзда под гайки, зенковки под головки) и ставит по ним болты и гайки.
   -> 3d-viewer/parts/explode.json
Запуск из корня репозитория: python tools/explode/make_explode.py   (нужен tools/kdy/fit.json)
Потом длины болтов по геометрии сборки: python tools/explode/check_fasteners.py --apply
"""
import json, os
import numpy as np, trimesh, shapely.geometry as G

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
V = os.path.join(ROOT, '3d-viewer')
PRINT = os.path.join(ROOT, 'print', 'kdy')
FIT = json.load(open(os.path.join(ROOT, 'tools', 'kdy', 'fit.json')))
OUT_STL = os.path.join(V, 'stl', 'parts')

# frontpart/backpart (визуал front_link/rear_link: rpy 0 0 π, xyz 40 55 -10 мм) -> система mainbody (xyz -42 -55 -10 мм)
RZ = np.diag([-1.0, -1.0, 1.0, 1.0]); RZ[:3, 3] = [40 + 42, 55 + 55, 0]
GROUP = {'mainbody': ('body', np.eye(4)), 'frontpart': ('body', RZ), 'backpart': ('body', RZ),
         'larm': ('larm', np.eye(4)), 'larm_cover': ('larm', np.eye(4)), 'rarm': ('rarm', np.eye(4)), 'rarm_cover': ('rarm', np.eye(4)),
         'lshoulder': ('lshoulder', np.eye(4)), 'rshoulder': ('rshoulder', np.eye(4)), 'lfoot': ('lfoot', np.eye(4)), 'rfoot': ('rfoot', np.eye(4))}
# система группы -> система URDF-меша, к визуалу которого крепим болты
ANCHOR = {'body': 'mainbody', 'larm': 'larm', 'rarm': 'rarm', 'lshoulder': 'lshoulder', 'rshoulder': 'rshoulder', 'lfoot': 'lfoot', 'rfoot': 'rfoot'}
SIZES = [('M3', 2.9, 3.7), ('M4', 3.9, 4.7), ('M5', 4.9, 5.7)]
LEN = {'M3': [10, 20], 'M4': [15, 20], 'M5': [15]}            # длины из BOM KDY0523


def features(mesh, step=0.5):
    """Отверстия и гнёзда: сечения, сшитые по соседним срезам в отрезки вдоль оси."""
    out = []
    for ax in range(3):
        n = np.eye(3)[ax]
        hs = np.arange(mesh.bounds[0][ax] + step / 2, mesh.bounds[1][ax], step)
        rings = []
        for h, s in zip(hs, mesh.section_multiplane(plane_origin=[0, 0, 0], plane_normal=n, heights=hs)):
            if s is None: continue
            for poly in s.polygons_full:
                for r in poly.interiors:
                    P = G.Polygon(r)
                    if P.area <= 0: continue
                    d = 2 * np.sqrt(P.area / np.pi)
                    if not 2.6 < d < 9.5: continue
                    c = (s.metadata['to_3D'] @ np.r_[P.centroid.coords[0], 0, 1])[:3]
                    rings.append((h, d, 4 * np.pi * P.area / r.length ** 2, c))
        segs = []
        for h, d, circ, c in sorted(rings, key=lambda r: r[0]):
            for sg in segs:
                if abs(sg['h1'] - h) < step + 1e-6 and np.linalg.norm(np.delete(sg['c'] - c, ax)) < 0.3 and abs(sg['d'] - d) < 0.6:
                    sg['h1'] = h; sg['ds'].append(d); sg['cs'].append(circ); break
            else:
                segs.append({'ax': ax, 'h0': h, 'h1': h, 'c': c, 'd': d, 'ds': [d], 'cs': [circ]})
        for sg in segs:
            a, b = sg['c'].copy(), sg['c'].copy(); a[ax] = sg['h0'] - step / 2; b[ax] = sg['h1'] + step / 2
            out.append({'a': a, 'b': b, 'd': float(np.median(sg['ds'])), 'circ': float(np.median(sg['cs']))})
    return out


def tf(M, p): return (M @ np.r_[p, 1])[:3]


def bolts_for(feats):
    """feats: признаки в системе группы. Болт = цепочка соосных круглых отверстий без разрывов > 4 мм.
    Отверстия Ø4.9+ (M5) у крышек KDY смещены до 5 мм от отверстий переборки — сводим их к одному болту."""
    isH = [f['circ'] > 0.95 and any(lo <= f['d'] <= hi for _, lo, hi in SIZES) for f in feats]
    holes = [f for f, k in zip(feats, isH) if k]
    other = [f for f, k in zip(feats, isH) if not k]
    used, res = set(), []
    for i, h in enumerate(holes):
        if i in used: continue
        u = h['b'] - h['a']; u /= np.linalg.norm(u)
        grp = [i]
        for j, k in enumerate(holes):
            if j == i or j in used: continue
            w = k['b'] - k['a']; w /= np.linalg.norm(w)
            if abs(abs(u @ w) - 1) > 1e-3: continue
            off = (k['a'] - h['a']) - ((k['a'] - h['a']) @ u) * u
            tol = 5.5 if min(h['d'], k['d']) >= 4.9 else 1.0
            if np.linalg.norm(off) < tol: grp.append(j)
        # разрывы вдоль оси > 4 мм — разные болты
        o = h['a']
        iv = sorted(((min((holes[j]['a'] - o) @ u, (holes[j]['b'] - o) @ u), max((holes[j]['a'] - o) @ u, (holes[j]['b'] - o) @ u), j) for j in grp))
        clusters, cur = [], [iv[0]]
        for x in iv[1:]:
            if x[0] - max(c[1] for c in cur) > 4: clusters.append(cur); cur = [x]
            else: cur.append(x)
        clusters.append(cur)
        for cl in clusters:
            members = [c[2] for c in cl]
            if i not in members and any(m in used for m in members): continue
            used.update(members)
            dn = min(holes[j]['d'] for j in members)
            base = max((j for j in members if holes[j]['d'] <= dn + 0.2), key=lambda j: np.linalg.norm(holes[j]['b'] - holes[j]['a']))   # ось — по самому длинному из узких отверстий
            ob = holes[base]['a']; ob = ob - ((ob - o) @ u) * u + ((ob - o) @ u) * u
            t0, t1 = min(c[0] for c in cl), max(c[1] for c in cl)
            dmin = holes[base]['d']
            size = next(s for s, lo, hi in SIZES if lo <= dmin <= hi)
            axis_o = ob - ((ob - o) @ u) * u                             # точка оси при t = 0
            nut = head = None
            for f in other:
                c = (f['a'] + f['b']) / 2; t = (c - axis_o) @ u
                lat = np.linalg.norm((c - axis_o) - t * u)
                if not (t0 - 6 <= t <= t1 + 6) or f['d'] < dmin + 1: continue
                if f['circ'] < 0.95 and 5.0 <= f['d'] <= 9.5 and lat < 3.5: nut = t
                elif f['circ'] >= 0.95 and lat < 0.7: head = t
            span = t1 - t0
            hexhead = size == 'M5' and nut is not None                   # в BOM нет гаек M5: шестигранник — гнездо головки
            if hexhead:
                hs = t0 if abs(nut - t0) < abs(nut - t1) else t1; nut = None
            elif head is not None:
                hs = t0 if abs(head - t0) < abs(head - t1) else t1
            elif nut is not None:
                hs = t1 if abs(nut - t0) < abs(nut - t1) else t0
            else:
                hs = t0
            sgn = 1 if hs == t0 else -1
            L = next((l for l in LEN[size] if l >= span - 0.5), LEN[size][-1])
            if nut is None and size == 'M4':                             # BOM: на каждый M4 — гайка, ставим на выходе
                nut = hs + sgn * min(L - 2, max(span, 3))
            res.append({'size': size, 'head': (axis_o + hs * u).round(2).tolist(), 'dir': (sgn * u).round(4).tolist(), 'len': L,
                        'nut': None if nut is None else (axis_o + nut * u).round(2).tolist(), 'hexhead': hexhead, 'span': round(float(span), 1)})
    return res


def main():
    os.makedirs(OUT_STL, exist_ok=True)
    for f in os.listdir(OUT_STL): os.remove(os.path.join(OUT_STL, f))
    parts, feats = {}, {}
    for rv, rows in FIT['parts'].items():
        g, Mg = GROUP[rv]
        for n, r in enumerate(rows):
            src = r['part']
            if rv == 'backpart' and src == 'R_cover': src_file = os.path.join(ROOT, 'print', 'R_cover_alatoo.stl')
            else: src_file = os.path.join(PRINT, src + '.stl')
            M = np.array(r['M'])
            m = trimesh.load(src_file); m.apply_transform(M)
            name = f'{rv}__{n}__{src}.stl'; m.export(os.path.join(OUT_STL, name))
            parts.setdefault(rv, []).append({'file': 'stl/parts/' + name, 'part': src})
            raw = trimesh.load(os.path.join(PRINT, r['part'] + '.stl'))
            for f in features(raw):
                feats.setdefault(g, []).append({**f, 'a': tf(Mg @ M, f['a']), 'b': tf(Mg @ M, f['b'])})
            print(rv, src, flush=True)
    fast = {g: bolts_for(F) for g, F in feats.items()}
    for g, B in fast.items():
        cnt = {}
        for b in B: k = f"{b['size']}x{b['len']}" + ('+nut' if b['nut'] else ''); cnt[k] = cnt.get(k, 0) + 1
        print(g, len(B), cnt)
    json.dump({'parts': parts, 'fasteners': fast, 'anchor': ANCHOR}, open(os.path.join(V, 'parts', 'explode.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()

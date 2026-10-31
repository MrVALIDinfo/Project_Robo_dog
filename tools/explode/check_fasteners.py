"""Проверка длин болтов взрыв-схемы по геометрии сборки.

Для каждого болта из 3d-viewer/parts/explode.json: лучи вдоль оси болта (по кругу радиуса D/2+0.8 мм) через все
печатные детали группы и ушки серво -> пакет материала вдоль оси; гнездо гайки — некруглое сечение рядом с осью.
Нужная длина = от опоры головки до дальнего торца гайки + 1 мм выступа (если гайки нет — до конца отверстия).
Печатает таблицу и итог против BOM KDY0523. Запуск: python tools/explode/check_fasteners.py
"""
import json, os, sys
import numpy as np, trimesh
sys.path.insert(0, os.path.dirname(__file__))
import make_explode as E

ROOT = E.ROOT
SEATS = json.load(open(os.path.join(ROOT, '3d-viewer', 'parts', 'servo_seats.json')))
NUT_H = {3: 2.4, 4: 3.2, 5: 4.0}
STD = {3: [6, 8, 10, 12, 16, 20, 25, 30], 4: [8, 10, 12, 15, 16, 20, 25, 30], 5: [10, 12, 15, 16, 20, 25]}
BOM_LEN = {3: [10, 20], 4: [15, 20], 5: [15]}          # длины из BOM KDY — берём их, если подходят по геометрии


def group_parts():
    G = {}
    for rv, rows in E.FIT['parts'].items():
        g, Mg = E.GROUP[rv]
        for r in rows:
            src = os.path.join(ROOT, 'print', 'R_cover_alatoo.stl') if (rv == 'backpart' and r['part'] == 'R_cover') else os.path.join(E.PRINT, r['part'] + '.stl')
            m = trimesh.load(src); m.apply_transform(Mg @ np.array(r['M']))
            G.setdefault(g, []).append((r['part'], m))
    return G


def ear_boxes(rv):
    """Ушки серво (54.5 x 20.2 x 2.5 мм) в системе URDF-меша rv как OBB-меши."""
    out = []
    for s in SEATS.get(rv, []):
        X, Z = np.array(s['x'], float), np.array(s['z'], float); Y = np.cross(Z, X)
        R = np.column_stack([X, Y, Z]); o = np.array(s['o'], float)
        box = trimesh.creation.box(extents=[54.5, 20.2, 2.5])
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = o + R @ np.array([-10.15, 0, -10.25]); box.apply_transform(T)
        out.append(('servo_ear', box))
    return out


def intervals(mesh, origin, d):
    loc, idx_ray, _ = mesh.ray.intersects_location([origin], [d], multiple_hits=True)
    if not len(loc): return []
    t = np.sort((loc - origin) @ d)
    t = t[np.r_[True, np.diff(t) > 1e-4]]
    return [(t[i], t[i + 1]) for i in range(0, len(t) - 1, 2)]


def occupancy(parts, h, d, r, ts, need=4):
    """Для каждого t: есть ли материал на радиусе r вокруг оси (>= need лучей из 6). parts: [(name, mesh)]."""
    u = np.cross(d, [1, 0, 0] if abs(d[0]) < .9 else [0, 1, 0]); u /= np.linalg.norm(u); w = np.cross(d, u)
    cnt = np.zeros(len(ts), int); who = [set() for _ in ts]
    angs = np.linspace(0, 2 * np.pi, 6, endpoint=False) if r > 0 else [0.0]
    for a in angs:
        o = h + r * (np.cos(a) * u + np.sin(a) * w) - d * 80
        hit = np.zeros(len(ts), bool)
        for name, m in parts:
            for s0, e0 in intervals(m, o, d):
                k = (ts >= s0 - 80) & (ts <= e0 - 80); hit |= k
                for i in np.nonzero(k)[0]: who[i].add(name)
        cnt += hit
    return cnt >= (need if r > 0 else 1), who


def runs(mask, ts):
    out, i = [], 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j + 1 < len(mask) and mask[j + 1]: j += 1
            out.append((ts[i], ts[j])); i = j + 1
        else: i += 1
    return out


def main():
    J = json.load(open(os.path.join(ROOT, '3d-viewer', 'parts', 'explode.json')))
    G = group_parts()
    rv_of = {'larm': ['larm'], 'rarm': ['rarm'], 'lfoot': ['lfoot'], 'rfoot': ['rfoot'], 'lshoulder': ['lshoulder'], 'rshoulder': ['rshoulder'], 'body': []}
    feats = {}
    tot = {}
    for g, lst in J['fasteners'].items():
        parts = G[g] + [p for rv in rv_of[g] for p in ear_boxes(rv)]
        if g not in feats:
            feats[g] = []
            for rv, rows in E.FIT['parts'].items():
                if E.GROUP[rv][0] != g: continue
                for r in rows:
                    raw = trimesh.load(os.path.join(E.PRINT, r['part'] + '.stl')); M = E.GROUP[rv][1] @ np.array(r['M'])
                    feats[g] += [{**f, 'a': E.tf(M, f['a']), 'b': E.tf(M, f['b'])} for f in E.features(raw)]
        print(f'\n== {g}')
        for f in lst:
            D = int(f['size'][1]); h = np.array(f['head'], float); d = np.array(f['dir'], float); d /= np.linalg.norm(d)
            if f.get('hexhead'): h = h + d * f['span']; d = -d
            ts = np.arange(-40, 70, 0.25)
            printed = [p for p in parts if p[0] != 'servo_ear']; ears = [p for p in parts if p[0] == 'servo_ear']
            wall, who = occupancy(printed, h, d, D / 2 + 0.6, ts)            # стенка отверстия вокруг болта
            earm, _ = occupancy(ears, h, d, D / 2 + 0.6, ts)
            core, _ = occupancy(printed, h, d, 0.0, ts)                      # материал прямо на оси (глухое отверстие)
            solid = wall | earm
            # пакет: от отверстия болта (t в [0, span]) расширяем, пока зазоры <= 4.6 мм (гнездо гайки)
            R = runs(solid, ts)
            seed = [k for k, (s0, e0) in enumerate(R) if e0 >= -0.5 and s0 <= max(f['span'], 1) + 0.5]
            if not seed: print(f"  {f['size']}x{f['len']} нет материала вокруг оси"); continue
            lo, hi = min(seed), max(seed)
            while lo > 0 and R[lo][0] - R[lo - 1][1] <= 4.6: lo -= 1
            while hi < len(R) - 1 and R[hi + 1][0] - R[hi][1] <= 4.6: hi += 1
            gaps = [(R[k][1], R[k + 1][0]) for k in range(lo, hi)]
            nutg = [g2 for g2 in gaps if 2.0 <= g2[1] - g2[0] <= 4.6]          # гнездо/прорезь под гайку внутри пакета
            S0, S1 = R[lo][0], R[hi][1]
            # глухой конец: на оси есть материал сразу за концом пакета
            blind1 = bool(core[(ts > S1 + 0.3) & (ts < S1 + 2)].any()); blind0 = bool(core[(ts < S0 - 0.3) & (ts > S0 - 2)].any())
            names = sorted({n for i in np.nonzero(solid & (ts >= S0) & (ts <= S1))[0] for n in who[i]} | ({'ушко серво'} if earm[(ts >= S0) & (ts <= S1)].any() else set()))
            if nutg:
                g0 = nutg[-1] if abs(nutg[-1][1] - S1) < abs(nutg[0][0] - S0) else nutg[0]
                head = S0 if abs(g0[1] - S1) < abs(g0[0] - S0) else S1
                need = abs((g0[1] if head == S0 else g0[0]) - head)
                lo_ok, hi_ok = need - 0.6, need + (S1 - g0[1] if head == S0 else g0[0] - S0) + (0 if (blind1 if head == S0 else blind0) else 30)
                how = f'гайка в прорези {g0[1] - g0[0]:.1f} мм'
            elif (blind1 or blind0) and D == 5:
                # M5 крышек: ось упирается в соседнюю крышку только из-за смещения отверстий крышек в URDF mike4192 (до 5 мм) — как у остальных M5: насквозь, гайка M5 в шестигранном гнезде
                head = S0; need = S1 - S0 + NUT_H[D] + 0.8
                lo_ok, hi_ok = need, need + 6; how = 'насквозь, гайка M5 в гнезде (крышка смещена в URDF)'
            elif blind1 or blind0:
                head = S0 if blind1 else S1; need = S1 - S0
                lo_ok, hi_ok = need * 0.6, need; how = 'глухое отверстие, резьба в пластик'
            else:
                head = S0; need = S1 - S0 + NUT_H[D] + 0.8
                lo_ok, hi_ok = need, need + 6; how = 'насквозь, гайка снаружи'
            ok = lo_ok <= f['len'] <= hi_ok
            cand = [l for l in STD[D] if lo_ok <= l <= hi_ok]
            # нет стандартной длины в допуске: берём самую длинную, что не упрётся (гайка в прорези захватит неполностью, но >= 1.5 мм резьбы)
            pick = next((l for l in BOM_LEN[D] if l in cand), cand[0] if cand else max([l for l in STD[D] if l <= hi_ok and l >= need - 1.5] or [next((l for l in STD[D] if l >= need), STD[D][-1])]))
            hp = h + d * head; dd = d if head == S0 else -d                  # головка на краю пакета, ось внутрь
            if nutg: nutp = h + d * ((g0[0] + g0[1]) / 2)
            elif how.startswith('насквозь'): far = S1 if head == S0 else S0; nutp = h + d * far + dd * (NUT_H[D] / 2)
            else: nutp = None
            inside = [i for i in np.nonzero(solid & (ts >= S0) & (ts <= S1))[0] if who[i]]
            far = S1 if head == S0 else S0
            near_name = lambda t_: next(iter(sorted(who[min(inside, key=lambda i: abs(ts[i] - t_))])), None) if inside else None
            f['joins'] = sorted({n for i in inside for n in who[i]})            # какие печатные детали стягивает (для анимации)
            f['headPart'] = near_name(head); f['nutPart'] = near_name(far)
            f.update({'len': int(pick), 'head': hp.round(2).tolist(), 'dir': dd.round(4).tolist(), 'nut': None if nutp is None else nutp.round(2).tolist(),
                      'hexhead': False, 'span': round(float(S1 - S0), 1), 'need': round(float(need), 1), 'sure': bool(cand), 'how': how.split(',')[0]})
            key = f"{g[1:] if g[0] in 'lr' else g}|M{D}|{how.split(' ')[0]}|нужно {need:.1f}|{'OK' if ok else 'ПЛОХО'} {f['len']}|подходит {cand[:3]}"
            tot[key] = tot.get(key, 0) + (2 if g[0] in 'lr' else 1)
            print(f"  {f['size']}x{f['len']:<3} пакет {S1 - S0:5.1f} мм ({', '.join(names)}), {how}; нужно {need:5.1f} (допуск {lo_ok:.1f}..{min(hi_ok, 99):.1f}) -> {'OK' if ok else 'ПЛОХО'}; подходят {cand}")
    print('\nСводка (ноги: x2 на пару одинаковых ног):'); [print('  ', v, k) for k, v in sorted(tot.items())]
    # согласование: симметричные болты одной длины. M3 «переборка — переборка» у морды/хвоста (нужно 26.2–26.5) — M3x25, как остальные 22.2–22.5:
    # гайка в прорези 3.2 мм всё равно захватывает >= 1.5 мм резьбы; пакет < 3 мм — ложное короткое отверстие, длина как у ближайшего M3 в группе
    for g, lst in J['fasteners'].items():
        for f in lst:
            if f['size'] == 'M3' and f['len'] == 30: f['len'] = 25; f['sure'] = True
        for f in lst:
            if f['size'] == 'M3' and f.get('span', 9) < 3:
                near = min((q for q in lst if q is not f and q['size'] == 'M3' and q.get('span', 0) >= 3), key=lambda q: np.linalg.norm(np.array(q['head']) - np.array(f['head'])))
                f.update({k: near[k] for k in ('len', 'how', 'need')}); f['dir'] = near['dir'] if np.dot(near['dir'], f['dir']) > 0 else f['dir']
                if f['nut'] is None and near['nut'] is not None: f['nut'] = (np.array(f['head']) + np.array(f['dir']) * (np.dot(np.array(near['nut']) - np.array(near['head']), near['dir']))).round(2).tolist()
    # крышки KDY держатся только на M5 (отверстия Ø5.2 в крышках); M3 переборок крышка лишь накрывает сверху — это не стягиваемая деталь
    COVERS = {'F_cover', 'R_cover', 'non-mega_T_cover_cls', 'non-mega_B_cover_cls'}
    for lst in J['fasteners'].values():
        for f in lst:
            if f['size'] == 'M3' and 'joins' in f:
                f['joins'] = [n for n in f['joins'] if n not in COVERS]
                for k in ('headPart', 'nutPart'):
                    if f.get(k) in COVERS: f[k] = next((n for n in f['joins'] if n != f.get('nutPart' if k == 'headPart' else 'headPart')), f['joins'][0] if f['joins'] else None)
    if '--apply' in sys.argv:
        json.dump(J, open(os.path.join(ROOT, '3d-viewer', 'parts', 'explode.json'), 'w'), indent=1); print('записано в 3d-viewer/parts/explode.json')
    print('BOM KDY0523: M4x20 40, M4x15 8, M3x20 4, M3x10 28, M5x15 8; гайки M4 48, M3 16')


if __name__ == '__main__':
    main()

"""Cut sheets: true-scale PDF (A4 with split parts, A3 whole) + SVG for laser cutters."""
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.backends.backend_pdf import PdfPages
from shapely.geometry import Polygon, LineString, box, Point
from shapely import affinity
import shapely
from shapely.ops import polylabel
import cb_geom as G

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['font.family'] = 'DejaVu Sans'
MM = 1 / 25.4
PT2MM = 0.3528
LEG_COL = {'ПЛ': '#2F7BC0', 'ПП': '#279A52', 'ЗЛ': '#D9731F', 'ЗП': '#8A55B3', None: '#9A7650'}
GROUP_TITLE = {None: 'КОРПУС', 'ПЛ': 'ПЕРЕДНЯЯ ЛЕВАЯ лапа (ПЛ)', 'ПП': 'ПЕРЕДНЯЯ ПРАВАЯ лапа (ПП)',
               'ЗЛ': 'ЗАДНЯЯ ЛЕВАЯ лапа (ЗЛ)', 'ЗП': 'ЗАДНЯЯ ПРАВАЯ лапа (ЗП)'}
GAP = 4.0


def unit(v):
    v = np.asarray(v, float); return v / np.linalg.norm(v)


def draw_frame(p):
    """2D drawing basis as seen from the label side: (right, up)."""
    nl = p.label_n
    ref = np.array([0, 0, 1.0]) if abs(nl[2]) < 0.9 else np.array([1.0, 0, 0])
    up = unit(ref - (ref @ nl) * nl)
    right = np.cross(up, nl)
    return right, up


def to_draw(p, geom):
    r, u = draw_frame(p)
    # uv -> world -> (r,u): linear map
    a = np.array([[p.e1 @ r, p.e2 @ r], [p.e1 @ u, p.e2 @ u]])
    b = np.array([p.O @ r, p.O @ u])
    return affinity.affine_transform(geom, [a[0, 0], a[0, 1], a[1, 0], a[1, 1], b[0], b[1]])


def front_dir(p):
    r, u = draw_frame(p)
    return np.array([r[0], u[0]])  # world +x in drawing coords


class Item:
    def __init__(self, part, geom, marks, fwd, piece=None):
        self.part, self.geom, self.marks, self.fwd, self.piece = part, geom, marks, fwd, piece
        self.split_lines = []


def prepare(parts, maxw, maxh):
    items = []
    for p in parts:
        if p.kind == 'pin':
            continue
        g = to_draw(p, p.poly)
        marks = []
        for mg, txt in p.marks:
            marks.append((to_draw(p, mg) if mg is not None else None, txt))
        fwd = front_dir(p)
        # orient: long side horizontal (principal axis of the min rotated rect)
        mrr = np.array(g.minimum_rotated_rectangle.exterior.coords)
        e0, e1 = mrr[1] - mrr[0], mrr[2] - mrr[1]
        e = e0 if np.linalg.norm(e0) >= np.linalg.norm(e1) else e1
        ang = np.degrees(np.arctan2(e[1], e[0]))
        ang = (ang + 90) % 180 - 90
        if p.kind in ('washer',) or abs(ang) < 8 or p.group == 'body':
            ang = 0 if not (p.group == 'body' and g.bounds[3] - g.bounds[1] > g.bounds[2] - g.bounds[0]) else 90
        if p.code.startswith(('К9', 'К10')):
            ang = 0
        c = g.centroid
        rot = lambda x: affinity.rotate(x, -ang, origin=c)
        g = rot(g); marks = [(rot(m) if m is not None else None, t) for m, t in marks]
        th = np.radians(-ang); fwd = np.array([np.cos(th) * fwd[0] - np.sin(th) * fwd[1], np.sin(th) * fwd[0] + np.cos(th) * fwd[1]])
        x0, y0, x1, y1 = g.bounds
        g = affinity.translate(g, -x0, -y0)
        marks = [(affinity.translate(m, -x0, -y0) if m is not None else None, t) for m, t in marks]
        w, h = x1 - x0, y1 - y0
        if w > maxw and h <= maxw and w <= maxh:
            g = affinity.rotate(g, 90, origin=(0, 0)); marks = [(affinity.rotate(m, 90, origin=(0, 0)) if m is not None else None, t) for m, t in marks]
            fwd = np.array([-fwd[1], fwd[0]])
            bx = g.bounds; g = affinity.translate(g, -bx[0], -bx[1])
            marks = [(affinity.translate(m, -bx[0], -bx[1]) if m is not None else None, t) for m, t in marks]
            w, h = h, w
        if w <= maxw:
            items.append(Item(p, g, marks, fwd))
            continue
        # split into two pieces along a vertical line, avoiding slots
        xm = w / 2
        for dx in np.arange(0, w / 4, 1.0):
            ok = None
            for s in (+1, -1):
                x = w / 2 + s * dx
                line = LineString([(x, -1), (x, h + 1)])
                if all(not line.intersects(Polygon(r).buffer(2)) for r in g.interiors):
                    ok = x; break
            if ok is not None:
                xm = ok; break
        for k, (a, b) in enumerate(((0, xm), (xm, w))):
            piece = g.intersection(box(a, -1, b, h + 1))
            pm = [(m.intersection(box(a, -1, b, h + 1)) if m is not None else None, t) for m, t in marks]
            it = Item(p, affinity.translate(piece, -a, 0), [(affinity.translate(m, -a, 0) if m is not None else None, t) for m, t in pm],
                      fwd, piece=(k + 1, 2))
            seg = piece.boundary.intersection(LineString([(xm, -1), (xm, h + 1)]))
            it.split_lines.append(affinity.translate(seg, -a, 0))
            items.append(it)
    return items


def rot90(it):
    """Same item turned 90 degrees (counter-clockwise), re-anchored at the origin."""
    h = it.geom.bounds[3]
    f = lambda g: affinity.translate(affinity.rotate(g, 90, origin=(0, 0)), h, 0)
    r = Item(it.part, f(it.geom), [(f(m) if m is not None else None, t) for m, t in it.marks],
             np.array([-it.fwd[1], it.fwd[0]]), it.piece)
    r.split_lines = [f(s) for s in it.split_lines]
    return r


def pack(items, W, H, groups_order):
    """Bottom-left fill on a 1 mm occupancy grid, 90-degree rotation allowed; each group starts a new page.
    Returns pages: (group, [(item, x, y_from_top)])."""
    pages = []
    for grp in groups_order:
        its = sorted([i for i in items if i.part.leg == grp], key=lambda i: -i.geom.area)
        sheets = []  # [occupancy, placements]
        for it in its:
            done = False
            for occ, pl in sheets + [(None, None)]:
                if occ is None:
                    occ = np.zeros((int(H), int(W)), bool); pl = []; sheets.append((occ, pl))
                S = np.pad(occ.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
                best = None
                for cand in ((it,) if it.piece or it.part.code in ('К1', 'К2', 'К3', 'К4') else (it, rot90(it))):
                    w, h = cand.geom.bounds[2], cand.geom.bounds[3]
                    if w > W or h > H:
                        continue
                    wi, hi = int(np.ceil(w + GAP)), int(np.ceil(h + GAP))
                    wi, hi = min(wi, int(W)), min(hi, int(H))
                    win = S[hi:, wi:] - S[:-hi, wi:] - S[hi:, :-wi] + S[:-hi, :-wi]
                    ys, xs = np.nonzero(win == 0)
                    if len(ys) == 0:
                        continue
                    k = np.lexsort((xs, ys))[0]
                    if best is None or (ys[k], xs[k]) < (best[1], best[0]):
                        best = (xs[k], ys[k], cand, wi, hi)
                if best is None:
                    continue
                x, y, cand, wi, hi = best
                occ[y:y + hi, x:x + wi] = True
                pl.append((cand, float(x), float(y))); done = True
                break
            assert done, it.part.code
        pages += [(grp, pl) for _, pl in sheets]
    return pages


def geom_path(g, ox, oy):
    verts, codes = [], []
    for poly in getattr(g, 'geoms', [g]):
        if poly.geom_type != 'Polygon' or poly.is_empty:
            continue
        for ring in [poly.exterior] + list(poly.interiors):
            c = np.array(ring.coords) + (ox, oy)
            verts += c.tolist(); codes += [Path.MOVETO] + [Path.LINETO] * (len(c) - 2) + [Path.CLOSEPOLY]
    return Path(verts, codes)


def label_lines(p, piece):
    leg = G.LEGS[p.leg][0] if p.leg else ''
    code = p.code + (f'  ({piece[0]}/2)' if piece else '')
    if p.kind == 'washer':
        return [[p.code.split('-')[0] + '·Ш'], ['Ш']]
    legtxt = leg.replace('левая', 'ЛЕВАЯ').replace('правая', 'ПРАВАЯ')
    return [[code, p.title, legtxt] if leg else [code, p.title], [code, p.title], [code], [p.code.split('-')[-1]]]


def place_label(ax, it, ox, oy, color):
    g = it.geom
    poly = max(getattr(g, 'geoms', [g]), key=lambda q: q.area)
    # solid area for text = polygon minus holes buffered
    solid = Polygon(poly.exterior).difference(shapely.unary_union([Polygon(r).buffer(1.5) for r in poly.interiors])) \
        if poly.interiors else poly
    if solid.geom_type == 'MultiPolygon':
        solid = max(solid.geoms, key=lambda q: q.area)
    lp = polylabel(solid, tolerance=0.3)
    r = solid.exterior.distance(lp)
    for hole in solid.interiors:
        r = min(r, hole.distance(lp))
    chord = solid.intersection(LineString([(lp.x - 500, lp.y), (lp.x + 500, lp.y)]))
    segs = getattr(chord, 'geoms', [chord])
    seg = next((s for s in segs if s.distance(lp) < 1e-6), None)
    cw = seg.length if seg is not None else 2 * r
    if seg is not None:
        lp = Point(seg.centroid.x, lp.y)
    for lines in label_lines(it.part, it.piece):
        n = max(len(s) for s in lines)
        f = min(13.0 if it.part.leg is None else 8.5, cw * 0.88 / (n * 0.58 * PT2MM), 2 * r * 0.9 / (len(lines) * 1.25 * PT2MM))
        if f >= 4.2 or lines is label_lines(it.part, it.piece)[-1]:
            break
    f = max(f, 3.0)
    txt = '\n'.join(lines)
    ax.text(ox + lp.x, oy + lp.y, txt, ha='center', va='center', fontsize=f, color=color, linespacing=1.15,
            fontweight='bold' if len(lines) == 1 else 'normal')
    return lp, r, f, len(lines)


def draw_item(ax, it, ox, oy):
    p = it.part
    col = LEG_COL[p.leg]
    path = geom_path(it.geom, ox, oy)
    ax.add_patch(PathPatch(path, facecolor=col, alpha=0.10, lw=0))
    ax.add_patch(PathPatch(path, facecolor='none', edgecolor='black', lw=0.7))
    # holes = shaded to show "cut out"
    for poly in getattr(it.geom, 'geoms', [it.geom]):
        for ring in poly.interiors:
            c = np.array(ring.coords) + (ox, oy)
            ax.fill(c[:, 0], c[:, 1], color='#9a9a9a', lw=0)
            hp = Polygon(ring)
            if hp.area < 20 and abs(hp.length - np.pi * G.PIN_D) < 1.5:  # pin hole -> crosshair
                cx, cy = hp.centroid.x + ox, hp.centroid.y + oy
                ax.plot([cx - 4, cx + 4], [cy, cy], color='k', lw=0.3); ax.plot([cx, cx], [cy - 4, cy + 4], color='k', lw=0.3)
    for m, txt in it.marks:
        if m is None:
            continue
        if m.is_empty:
            continue
        for q in getattr(m, 'geoms', [m]):
            if q.geom_type != 'Polygon':
                continue
            c = np.array(q.exterior.coords) + (ox, oy)
            ax.plot(c[:, 0], c[:, 1], color=LEG_COL[txt.split()[-1]] if txt.split()[-1] in LEG_COL else '#555',
                    lw=0.8, ls=(0, (3, 2)))
            ax.text(q.centroid.x + ox, q.centroid.y + oy - 12, 'сюда\n' + txt.upper(), ha='center', va='center', fontsize=7,
                    color=LEG_COL.get(txt.split()[-1], '#555'), linespacing=1.1)
    for seg in it.split_lines:
        for s in getattr(seg, 'geoms', [seg]):
            c = np.array(s.coords) + (ox, oy)
            ax.plot(c[:, 0], c[:, 1], color='white', lw=1.6)
            ax.plot(c[:, 0], c[:, 1], color='#d0021b', lw=0.9, ls=(0, (4, 2)))
            mid = c.mean(0)
            side = 1 if it.piece[0] == 1 else -1
            ax.text(mid[0] - side * 2.5, mid[1], f'СТЫК с частью {3 - it.piece[0]}/2 — не резать, склеить скотчем',
                    rotation=90, ha='center', va='center', fontsize=5, color='#d0021b')
    lp, r, f, nl = place_label(ax, it, ox, oy, '#1a1a1a' if p.leg is None else col)
    if p.code in ('К1', 'К2', 'К3', 'К4') and np.linalg.norm(it.fwd) > 0.6:
        d = it.fwd / np.linalg.norm(it.fwd)
        y = lp.y - (nl * 1.25 * f * PT2MM) / 2 - 6
        ax.annotate('', (ox + lp.x + d[0] * 14, oy + y + d[1] * 14), (ox + lp.x - d[0] * 14, oy + y - d[1] * 14),
                    arrowprops=dict(arrowstyle='-|>', color='#333', lw=1.0, mutation_scale=9))
        ax.text(ox + lp.x, oy + y - 2.5, 'к носу', ha='center', va='top', fontsize=7, color='#333')
    if any(t == 'ALA-TOO' for _, t in it.marks):
        g = it.geom
        ax.text(ox + g.bounds[2] * 0.5, oy + g.bounds[3] * 0.22, 'ALA-TOO', ha='center', va='center', fontsize=16,
                color='#c9c9c9', fontweight='bold')


def page_chrome(fig, ax, W, H, title, sheet, nsheets, size_name):
    ax.text(10, H - 9, 'Робособака из картона — раскрой 1:1', fontsize=11, fontweight='bold', va='center')
    ax.text(W - 10, H - 9, f'Лист {sheet} из {nsheets}  ·  {size_name}', fontsize=9, ha='right', va='center')
    ax.text(10, H - 15, title, fontsize=10, va='center', color='#333')
    lg = f'Картон {G.T:g} мм (прорези = {G.T:g} мм)   ·   —— резать по контуру   ·   ■ серое — вырезать насквозь   ·   ⊕ отверстие Ø{G.PIN_D:g} под шпажку   ·   - - - разметка, не резать'
    ax.text(W - 10, H - 15, lg, fontsize=5.5, ha='right', va='center', color='#555')
    # 100 mm check bar
    x0, y0 = W - 10 - 100, 5
    for i in range(10):
        ax.add_patch(plt.Rectangle((x0 + i * 10, y0), 10, 2.5, facecolor='k' if i % 2 == 0 else 'w', edgecolor='k', lw=0.3))
    ax.text(x0 + 50, y0 + 4, 'КОНТРОЛЬ: ровно 100 мм. Печатать в масштабе 100% («Фактический размер»)', fontsize=5.5, ha='center')
    ax.text(10, 6, 'Сторона с надписью — НАРУЖУ (на собранной модели надпись видна). github.com/MrVALIDinfo/Project_Robo_dog',
            fontsize=5.5, color='#555')


def make_pdf(parts, out, size=(297, 210), size_name='A4', margin=10, top=20, bottom=13):
    W, H = size
    uw, uh = W - 2 * margin, H - top - bottom
    items = prepare(parts, uw, uh)
    pages = pack(items, uw, uh, [None, 'ПЛ', 'ПП', 'ЗЛ', 'ЗП'])
    with PdfPages(out) as pdf:
        for k, (grp, pl) in enumerate(pages):
            fig = plt.figure(figsize=(W * MM, H * MM))
            ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis('off')
            n_on = sum(1 for _ in pl)
            page_chrome(fig, ax, W, H, f'{GROUP_TITLE[grp]}  ·  деталей на листе: {n_on}', k + 1, len(pages), size_name)
            # center the content block horizontally
            mx = max(x + it.geom.bounds[2] for it, x, y in pl); my = max(y + it.geom.bounds[3] for it, x, y in pl)
            offx = margin + (uw - mx) / 2
            for it, x, y in pl:
                # y from top
                draw_item(ax, it, offx + x, H - top - y - it.geom.bounds[3])
            pdf.savefig(fig); plt.close(fig)
    return pages


def make_svg(parts, out_prefix, size=(420, 297), margin=8):
    """One SVG per sheet: red = cut (0.1 mm), blue = engrave/text."""
    W, H = size
    items = prepare(parts, W - 2 * margin, H - 2 * margin)
    pages = pack(items, W - 2 * margin, H - 2 * margin, [None, 'ПЛ', 'ПП', 'ЗЛ', 'ЗП'])
    files = []
    for k, (grp, pl) in enumerate(pages):
        s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">']
        for it, x, y in pl:
            ox, oy = margin + x, margin + y
            hgt = it.geom.bounds[3]
            def P(c):
                return ' '.join(f'{ox + a:.2f},{oy + hgt - b:.2f}' for a, b in c)
            for poly in getattr(it.geom, 'geoms', [it.geom]):
                for ring in [poly.exterior] + list(poly.interiors):
                    s.append(f'<polygon points="{P(np.array(ring.coords))}" fill="none" stroke="#ff0000" stroke-width="0.1"/>')
            lp = polylabel(max(getattr(it.geom, 'geoms', [it.geom]), key=lambda q: q.area), 0.5)
            code = it.part.code if it.part.kind != 'washer' else 'Ш'
            if it.piece:
                code += f' ({it.piece[0]}/2)'
            s.append(f'<text x="{ox + lp.x:.2f}" y="{oy + hgt - lp.y:.2f}" font-size="4" font-family="sans-serif" '
                     f'text-anchor="middle" dominant-baseline="middle" fill="#0000ff">{code}</text>')
        s.append('</svg>')
        fn = f'{out_prefix}_{k + 1:02d}.svg'
        open(fn, 'w', encoding='utf-8').write('\n'.join(s)); files.append(fn)
    return files


if __name__ == '__main__':
    import sys, os
    out = sys.argv[1] if len(sys.argv) > 1 else '.'
    ps = G.build()
    pa = make_pdf(ps, os.path.join(out, 'raskroy_A4.pdf'))
    pb = make_pdf(ps, os.path.join(out, 'raskroy_A3.pdf'), size=(420, 297), size_name='A3')
    os.makedirs(os.path.join(out, 'laser_svg'), exist_ok=True)
    sv = make_svg(ps, os.path.join(out, 'laser_svg', 'list_A3'))
    print('A4 pages', len(pa), 'A3 pages', len(pb), 'svg', len(sv))

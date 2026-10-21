"""SpotMicro dimension sheets -> printable A4 PDF (true scale)."""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.backends.backend_pdf import PdfPages
sys.path.insert(0, sys.argv[1]); import spot_geom as G
OUT = sys.argv[2]
MM = 1 / 25.4
A4P, A4L = (210, 297), (297, 210)
# views: name, horizontal axis, vertical axis, (axis, sign) toward viewer
VIEWS = {
    'side':  ('Вид сбоку',   0, 2, (1, -1)),
    'front': ('Вид спереди', 1, 2, (0, +1)),
    'top':   ('Вид сверху',  0, 1, (2, +1)),
}
DIM = '#c0392b'

def new_page(size, title, scale_txt):
    w, h = size
    fig = plt.figure(figsize=(w * MM, h * MM), dpi=300)
    fig._mm = (w, h)
    fig.text(10 / w, 1 - 10 / h, title, fontsize=15, weight='bold', va='top')
    fig.text(1 - 10 / w, 1 - 10 / h, f'{scale_txt}   ·   размеры в мм', fontsize=10, ha='right', va='top')
    fig.text(10 / w, 5 / h, 'Источник: github.com/mike4192/spotMicro (spot_micro_rviz/urdf STL\n+ spot_micro_motion_cmd.yaml). '
             'Модель для RViz, приблизительная.\nДетали для печати — Thingiverse thing:3445283.',
             fontsize=6, color='0.35', va='bottom')
    # 100 mm check bar
    x0, y0 = w - 10 - 100, 6
    ax = fig.add_axes([x0 / w, (y0 + 4) / h, 100 / w, 3 / h]); ax.set_xlim(0, 100); ax.set_ylim(0, 1); ax.axis('off')
    for i in range(10):
        ax.add_patch(plt.Rectangle((i * 10, 0), 10, 1, color='k' if i % 2 == 0 else 'w', ec='k', lw=.4))
    fig.text((x0 + 50) / w, (y0 + 8.5) / h, 'контроль: 100 мм — печатать 100% / «Фактический размер»', fontsize=6, ha='center')
    return fig

def shade(tris, view):
    _, a, b, (d, s) = view
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    eye = np.zeros(3); eye[d] = s
    light = eye * 0.8; light[a] -= 0.35; light[b] += 0.45; light /= np.linalg.norm(light)
    c = 0.38 + 0.55 * np.abs(n @ light)
    o = np.argsort(s * tris[:, :, d].mean(1))
    return tris[o][:, :, [a, b]], np.repeat(c[o, None], 3, 1)

def place(fig, x_mm, y_mm, tris, vkey, scale, pad=14, label=None, dims=True, extra=None):
    """Draw a view so that 1 mm on paper == scale mm of part. (x_mm,y_mm) = lower-left of axes on page."""
    W, H = fig._mm
    view = VIEWS[vkey]
    p2, col = shade(tris, view)
    pts = p2.reshape(-1, 2); mn, mx = pts.min(0), pts.max(0)
    ext = (mx - mn) / scale
    ax = fig.add_axes([x_mm / W, y_mm / H, (ext[0] + 2 * pad) / W, (ext[1] + 2 * pad) / H])
    ax.set_xlim(mn[0] - pad * scale, mx[0] + pad * scale); ax.set_ylim(mn[1] - pad * scale, mx[1] + pad * scale)
    ax.axis('off')
    ax.add_collection(PolyCollection(p2, facecolors=col, edgecolors=col, lw=0.15, rasterized=True))
    if dims:
        hdim(ax, mn[0], mx[0], mn[1] - 6 * scale, mn[1], scale)
        vdim(ax, mn[1], mx[1], mn[0] - 6 * scale, mn[0], scale)
    if extra: extra(ax, mn, mx, scale)
    if label:
        fig.text((x_mm + pad) / W, (y_mm + ext[1] + 2 * pad - 3) / H, label, fontsize=9, weight='bold')
    return ax, mn, mx, ext

def hdim(ax, x1, x2, y, yref, s, txt=None, fs=8):
    ax.plot([x1, x1], [yref, y - 1.5 * s], color=DIM, lw=.4); ax.plot([x2, x2], [yref, y - 1.5 * s], color=DIM, lw=.4)
    ax.annotate('', (x1, y), (x2, y), arrowprops=dict(arrowstyle='<|-|>', color=DIM, lw=.6, mutation_scale=6, shrinkA=0, shrinkB=0))
    ax.text((x1 + x2) / 2, y - 1.2 * s, txt or f'{abs(x2 - x1):.1f}', color=DIM, fontsize=fs, ha='center', va='top',
            bbox=dict(fc='w', ec='none', pad=.5))

def vdim(ax, y1, y2, x, xref, s, txt=None, fs=8):
    ax.plot([xref, x - 1.5 * s], [y1, y1], color=DIM, lw=.4); ax.plot([xref, x - 1.5 * s], [y2, y2], color=DIM, lw=.4)
    ax.annotate('', (x, y1), (x, y2), arrowprops=dict(arrowstyle='<|-|>', color=DIM, lw=.6, mutation_scale=6, shrinkA=0, shrinkB=0))
    ax.text(x - 1.2 * s, (y1 + y2) / 2, txt or f'{abs(y2 - y1):.1f}', color=DIM, fontsize=fs, ha='right', va='center',
            rotation=90, bbox=dict(fc='w', ec='none', pad=.5))

pdf = PdfPages(OUT)
parts = G.assembly()
allt = np.concatenate([t for _, t in parts])
lo, hi = allt.reshape(-1, 3).min(0), allt.reshape(-1, 3).max(0)
J = dict(sx=93.0, sy=39.0, L1=55.0, L2=107.5, L3=130.0)

# ---- Page 1: overview side + front, 1:3
fig = new_page(A4L, 'SpotMicro — общий вид (лапы прямо вниз)', 'Масштаб 1:3')
def side_extra(ax, mn, mx, s):
    # shoulder joint spacing along X, at z=0
    hdim(ax, -J['sx'], J['sx'], mx[1] + 9 * s, 0, s, f"{2 * J['sx']:.0f} (оси плеч)")
    for x in (-J['sx'], J['sx']):
        ax.plot(x, 0, 'o', ms=3, mfc='w', mec=DIM, mew=.8)
        ax.plot([x, x], [0, -J['L2']], color=DIM, lw=.5, ls='--')
        ax.plot(x, -J['L2'], 'o', ms=3, mfc='w', mec=DIM, mew=.8)
        ax.plot([x, x], [-J['L2'], -J['L2'] - J['L3']], color=DIM, lw=.5, ls='--')
    x = J['sx'] + 8 * s
    vdim(ax, 0, -J['L2'], mx[0] + 10 * s, J['sx'], s, f"L2 = {J['L2']}", fs=7)
    vdim(ax, -J['L2'], -J['L2'] - J['L3'], mx[0] + 10 * s, J['sx'], s, f"L3 = {J['L3']:.0f}", fs=7)
place(fig, 8, 52, allt, 'side', 3, pad=22, label='Вид сбоку (перед справа)', extra=side_extra)
def front_extra(ax, mn, mx, s):
    hdim(ax, -J['sy'], J['sy'], mx[1] + 9 * s, 0, s, f"{2 * J['sy']:.0f}")
    for y in (-1, 1):
        ax.plot(y * J['sy'], 0, 'o', ms=3, mfc='w', mec=DIM, mew=.8)
        ax.plot([y * J['sy'], y * (J['sy'] + J['L1'])], [0, 0], color=DIM, lw=.5, ls='--')
        ax.plot(y * (J['sy'] + J['L1']), 0, 'o', ms=3, mfc='w', mec=DIM, mew=.8)
    hdim(ax, J['sy'], J['sy'] + J['L1'], mx[1] + 3 * s, 0, s, f"L1={J['L1']:.0f}", fs=7)
place(fig, 170, 52, allt, 'front', 3, pad=22, label='Вид спереди', extra=front_extra)
ax = fig.add_axes([0, 0, 1, 1]); ax.axis('off')
W, H = A4L
rows = [('Габарит длина × ширина × высота (лапы прямо)', f'{hi[0]-lo[0]:.0f} × {hi[1]-lo[1]:.0f} × {hi[2]-lo[2]:.0f}'),
        ('Между осями плеч: вдоль / поперёк', f"{2*J['sx']:.0f} / {2*J['sy']:.0f}"),
        ('L1 — бедро (смещение оси колена вбок)', f"{J['L1']:.0f}"),
        ('L2 — верх лапы, ось-ось', f"{J['L2']}"),
        ('L3 — низ лапы, ось-носок', f"{J['L3']:.0f}"),
        ('Высота центра корпуса: стоя / лёжа', '155 / 83')]
for i, (k, v) in enumerate(rows):
    ax.text(12 / W, (44 - i * 5) / H, k, fontsize=8.5); ax.text(120 / W, (44 - i * 5) / H, v, fontsize=8.5, weight='bold')
pdf.savefig(fig); plt.close(fig)

# ---- Page 2: top view 1:2 + front with L1
fig = new_page(A4L, 'SpotMicro — вид сверху и кинематика лапы', 'Масштаб 1:2')
def top_extra(ax, mn, mx, s):
    for px in (-1, 1):
        for py in (-1, 1):
            ax.plot(px * J['sx'], py * J['sy'], 'o', ms=3, mfc='w', mec=DIM, mew=.8)
    hdim(ax, -J['sx'], J['sx'], mx[1] + 8 * s, J['sy'], s, f"{2*J['sx']:.0f} (оси плеч)")
    vdim(ax, -J['sy'], J['sy'], mx[0] + 12 * s, J['sx'], s, f"{2*J['sy']:.0f}")
place(fig, 6, 26, allt, 'top', 2, pad=22, label='Вид сверху (перед справа)', extra=top_extra)
# leg schematic (not to scale)
ax = fig.add_axes([200 / 297, 40 / 210, 90 / 297, 140 / 210]); ax.set_xlim(-20, 100); ax.set_ylim(-260, 40); ax.axis('off')
ax.set_title('Кинематика лапы (схема, не в масштабе)', fontsize=8.5, weight='bold')
ax.add_patch(plt.Rectangle((-20, 0), 30, 25, fc='0.85', ec='0.4'))
ax.text(-5, 12, 'корпус', ha='center', va='center', fontsize=7)
pts = [(10, 12), (60, 12), (60, -95), (60, -210)]
ax.plot(*zip(*pts), color='k', lw=3, solid_capstyle='round')
for p in pts: ax.plot(*p, 'o', ms=7, mfc='#e74c3c', mec='k')
ax.text(35, 22, f"L1 = {J['L1']:.0f}", ha='center', fontsize=8, color=DIM)
ax.text(66, -40, f"L2 = {J['L2']}", fontsize=8, color=DIM)
ax.text(66, -155, f"L3 = {J['L3']:.0f}", fontsize=8, color=DIM)
ax.text(10, -5, 'ось плеча\n(вдоль корпуса)', fontsize=6, ha='center', va='top')
ax.text(54, -95, 'колено', fontsize=6, ha='right', va='center')
ax.text(54, 6, 'бедро', fontsize=6, ha='right', va='top')
ax.text(60, -220, 'носок', fontsize=6, ha='center', va='top')
ax.text(-20, -245, 'Все длины — по прямой\nот оси до оси шарнира.', fontsize=6.5, color='0.3')
pdf.savefig(fig); plt.close(fig)

# ---- Part sheets
PARTS = [  # file, title, scale, mirror note
    ('lshoulder.stl', 'Плечо (левое)', 1, 'Правое — зеркальное (rshoulder.stl). Нужно 2 лев. + 2 прав.'),
    ('larm.stl', 'Верх лапы (левый)', 1, 'Правый — зеркальный (rarm.stl). Нужно 2 + 2.'),
    ('larm_cover.stl', 'Крышка верха лапы (левая)', 1, 'Правая — зеркальная (rarm_cover.stl). Нужно 2 + 2.'),
    ('lfoot.stl', 'Низ лапы (левый)', 1, 'Правый — зеркальный (rfoot.stl). Нужно 2 + 2.'),
    ('foot.stl', 'Носок', 1, 'Одинаковый для всех лап. Нужно 4.'),
    ('mainbody.stl', 'Корпус — центральная часть', 2, ''),
    ('frontpart.stl', 'Корпус — передняя часть', 2, ''),
    ('backpart.stl', 'Корпус — задняя часть', 2, ''),
]
for f, title, sc, note in PARTS:
    t = G.load_stl(f); t = t - t.reshape(-1, 3).min(0)
    e = t.reshape(-1, 3).max(0)
    if sc == 1:
        fig = new_page(A4P, f'SpotMicro — {title}', 'Масштаб 1:1')
        # front (X,Z) left, side (Y,Z) right, top (X,Y) below front
        y = 297 - 34 - (e[2] + 32)
        place(fig, 12, y, t, 'side', 1, pad=16, label='Вид A (X–Z)')
        place(fig, 12 + e[0] + 32 + 4, y, t, 'front', 1, pad=16, label='Вид B (Y–Z)')
        if y - (e[1] + 32) >= 24: place(fig, 12, y - (e[1] + 32), t, 'top', 1, pad=16, label='Вид C (X–Y)')
        elif 12 + e[0] + e[1] + 3 * 32 + 8 + e[0] <= 200: place(fig, 12 + e[0] + e[1] + 2 * 32 + 8, y, t, 'top', 1, pad=16, label='Вид C (X–Y)')
    else:
        fig = new_page(A4L, f'SpotMicro — {title}', 'Масштаб 1:2')
        _, _, _, exA = place(fig, 10, 100, t, 'side', 2, pad=16, label='Вид A (X–Z)')
        place(fig, 10, 22, t, 'top', 2, pad=16, label='Вид C (X–Y)')
        place(fig, 10 + exA[0] + 32 + 10, 100, t, 'front', 2, pad=16, label='Вид B (Y–Z)')
    W, H = fig._mm
    fig.text(10 / W, 1 - 21 / H, f'Файл: {f}   ·   габарит X × Y × Z = {e[0]:.1f} × {e[1]:.1f} × {e[2]:.1f} мм', fontsize=8.5)
    if note: fig.text(10 / W, 1 - 26 / H, note, fontsize=8.5, color='0.25')
    pdf.savefig(fig); plt.close(fig)
pdf.close()
print('ok', OUT)

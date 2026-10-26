"""Логотип для 3D-модели: PNG -> SVG по цветам (синий фон, красный, белый рельеф).

Пишет 3d-viewer/logo.svg: круг Ø viewBox (id="navy"), фигуры class="red" и class="relief" (белое).
В 3D-модели: синий диск 1 мм, красное +0.6 мм, белое +1 мм (как многоцветная печать).
Нужно: pip install numpy opencv-python-headless pillow
  python tools/logo/make_logo.py docs/logo_source.png
"""
import os, sys
import numpy as np, cv2
from PIL import Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'docs', 'logo_source.png')
a = np.array(Image.open(src).convert('RGBA')).astype(int)
rgb, alpha = a[:, :, :3], a[:, :, 3]

# круг логотипа по непрозрачной области
ys, xs = np.where(alpha > 128)
cx, cy = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
R = max(xs.max() - xs.min(), ys.max() - ys.min()) / 2
S = 2 * R

# классы цвета: ближайший из трёх
PAL = {'navy': (17, 34, 95), 'red': (204, 32, 48), 'white': (255, 255, 255)}
d = np.stack([((rgb - np.array(c)) ** 2).sum(-1) for c in PAL.values()], -1)
cls = np.argmin(d, -1)
inside = alpha > 128
k = np.ones((3, 3), np.uint8)
def mask(name):
    m = ((cls == list(PAL).index(name)) & inside).astype(np.uint8) * 255
    return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)

def ring(c, ccw):
    c = cv2.approxPolyDP(c, 0.6, True)[:, 0, :].astype(float)
    if len(c) < 3: return ''
    area = 0.5 * np.sum(c[:, 0] * np.roll(c[:, 1], -1) - np.roll(c[:, 0], -1) * c[:, 1])
    if (area > 0) != ccw: c = c[::-1]                      # наружный контур и дырки — в разные стороны
    c = c - [cx, cy] + [R, R]
    return 'M' + ' '.join(f'{x:.1f},{y:.1f}' for x, y in c) + 'Z'

def paths(m, layer, color, min_area=6):
    # каждая фигура — отдельный <path>: наружный контур + её дырки (буквы и полосы не склеиваются при триангуляции)
    cs, h = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    out = []
    for i, c in enumerate(cs):
        if h[0][i][3] != -1 or cv2.contourArea(c) < min_area: continue
        d = [ring(c, True)]
        j = h[0][i][2]
        while j != -1:
            if cv2.contourArea(cs[j]) >= min_area: d.append(ring(cs[j], False))
            j = h[0][j][0]
        out.append(f'<path class="{layer}" fill="{color}" d="{" ".join(x for x in d if x)}"/>')
    return ''.join(out)

svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {S:.1f} {S:.1f}">'
       f'<circle id="navy" cx="{R:.1f}" cy="{R:.1f}" r="{R:.1f}" fill="#11225f"/>'
       f'{paths(mask("red"), "red", "#cc2030")}{paths(mask("white"), "relief", "#ffffff")}</svg>')
out = os.path.join(ROOT, '3d-viewer', 'logo.svg')
open(out, 'w').write(svg)
print(f'{out}: круг {S:.0f} px, {len(svg) // 1024} КБ')

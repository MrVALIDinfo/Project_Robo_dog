"""Step-by-step assembly manual (A4 landscape PDF) with exploded 3D renders."""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import FancyBboxPatch
import cb_geom as G, cb_render as R, cb_sheets as S

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['font.family'] = 'DejaVu Sans'
MM = 1 / 25.4
W, H = 297, 210
LEG_COL = {'ПЛ': '#2F7BC0', 'ПП': '#279A52', 'ЗЛ': '#E0812B', 'ЗП': '#8A55B3'}
BODY_NEW, BODY_OLD, PIN = '#E2A15C', '#E3D6C2', '#EAD9B0'
INK = '#1d1d1f'


def mix(h, t, to=(1, 1, 1)):
    c = R.hexrgb(h); c = c * (1 - t) + np.array(to) * t
    return '#%02x%02x%02x' % tuple(int(round(v * 255)) for v in c)


def color(p, new):
    if p.kind == 'pin':
        return PIN if new else mix(PIN, 0.3)
    if p.leg:
        return LEG_COL[p.leg] if new else mix(LEG_COL[p.leg], 0.55)
    return BODY_NEW if new else BODY_OLD


PARTS = G.build()
BY = {p.code: p for p in PARTS}


def codes(pred):
    return [p.code for p in PARTS if pred(p)]


def leg_codes(leg, prefixes):
    return [p.code for p in PARTS if p.leg == leg and p.code.split('-')[1].startswith(prefixes)]


def center(p, M):
    c = p.poly.centroid
    P = p.w(np.array([c.x, c.y]))
    return M[:3, :3] @ P + M[:3, 3]


def scene_render(old, new, offsets=None, cam=(35, 24), hip=0.0, knee=0.0, labels=None, size=(1500, 1080),
                 label_text=None, arrows=True, fit_pad=0, arrow_for=None):
    offsets = offsets or {}
    M = G.pose(PARTS, hip, knee)
    items = []
    for c in old:
        items.append((BY[c], M[c], color(BY[c], False)))
    for c in new:
        Mc = M[c].copy()
        if c in offsets:
            Mc[:3, 3] += offsets[c]
        items.append((BY[c], Mc, color(BY[c], True)))
    camera = R.Camera(*cam)
    img, proj = R.render(items, camera, *size)
    ann = []
    for c in (labels if labels is not None else new):
        p = BY[c]
        P1 = center(p, M[c]) + (offsets.get(c, 0))
        ann.append((c, proj(P1)))
    arr = []
    if arrows:
        for c, off in offsets.items():
            if np.linalg.norm(off) < 1 or c not in new or (arrow_for is not None and c not in arrow_for):
                continue
            P0 = center(BY[c], M[c])
            arr.append((proj(P0 + off), proj(P0 + off * 0.18)))
    return img, ann, arr


def draw_scene(fig, rect, img, ann, arr, label_text=None, label_color=None):
    ax = fig.add_axes(rect)
    h, w = img.shape[:2]
    ax.imshow(img, extent=(0, w, h, 0), interpolation='lanczos')
    ax.set_xlim(0, w); ax.set_ylim(h, 0); ax.axis('off')
    for a, b in arr:
        ax.annotate('', b, a, arrowprops=dict(arrowstyle='-|>', color='#c0392b', lw=1.3, ls=(0, (4, 3)),
                                              mutation_scale=14, shrinkA=8, shrinkB=2))
    # labels: push outward from the picture centre, then de-overlap per side
    cx, cy = w / 2, h / 2
    lab = []
    for c, P in ann:
        d = np.array([P[0] - cx, P[1] - cy]); n = np.linalg.norm(d)
        d = d / n if n > 1 else np.array([1.0, 0])
        L = P + d * 95
        L[0] = np.clip(L[0], 70, w - 70); L[1] = np.clip(L[1], 25, h - 25)
        lab.append([c, P, L])
    for side in (-1, 1):
        grp = sorted([l for l in lab if np.sign(l[2][0] - cx + 1e-3) == side], key=lambda l: l[2][1])
        for i in range(1, len(grp)):
            if grp[i][2][1] - grp[i - 1][2][1] < 34:
                grp[i][2][1] = grp[i - 1][2][1] + 34
    for c, P, L in lab:
        p = BY[c]
        txt = (label_text or {}).get(c, c)
        col = LEG_COL.get(p.leg, '#7a4d1c') if p.kind != 'pin' else '#7a5b1c'
        ax.plot([P[0], L[0]], [P[1], L[1]], color=col, lw=0.8)
        ax.plot(P[0], P[1], 'o', ms=3.5, color=col)
        ax.text(L[0], L[1], txt, ha='center', va='center', fontsize=8.5, fontweight='bold', color='white',
                bbox=dict(boxstyle='round,pad=0.35', fc=col, ec='none'))
    return ax


def page(pdf, number, title, subtitle=None):
    fig = plt.figure(figsize=(W * MM, H * MM))
    bg = fig.add_axes([0, 0, 1, 1]); bg.set_xlim(0, W); bg.set_ylim(0, H); bg.axis('off')
    if number is not None:
        bg.add_patch(FancyBboxPatch((10, H - 22), 16, 14, boxstyle='round,pad=0,rounding_size=2.5', fc=INK, ec='none'))
        bg.text(18, H - 15, str(number), ha='center', va='center', color='white', fontsize=17, fontweight='bold')
        bg.text(31, H - 12.5, title, fontsize=17, fontweight='bold', va='center', color=INK)
        if subtitle:
            bg.text(31, H - 19.5, subtitle, fontsize=9.5, va='center', color='#555')
    else:
        bg.text(10, H - 14, title, fontsize=19, fontweight='bold', va='center', color=INK)
        if subtitle:
            bg.text(10, H - 21, subtitle, fontsize=10, va='center', color='#555')
    bg.text(W - 10, 6, 'Робособака из картона · инструкция по сборке', fontsize=7, ha='right', color='#999')
    return fig, bg


def text_block(bg, x, y, items, width=82, fs=9.2):
    """items: list of ('h', text) heading | ('p', text) | ('b', text) bullet | ('w', text) warning | ('parts', [codes])."""
    import textwrap
    line = fs * 0.3528 * 1.42
    for kind, t in items:
        if kind == 'h':
            y -= 2
            bg.text(x, y, t, fontsize=fs + 1.6, fontweight='bold', color=INK, va='top'); y -= line * 1.35
        elif kind == 'parts':
            xx = x
            for c in t:
                p = BY[c]
                col = LEG_COL.get(p.leg, '#9a6a35') if p.kind != 'pin' else '#9a7a35'
                lbl = c
                wdt = len(lbl) * fs * 0.3528 * 0.62 + 3
                if xx + wdt > x + width:
                    xx = x; y -= line * 1.25
                bg.text(xx, y, lbl, fontsize=fs - 1, color='white', fontweight='bold', va='top',
                        bbox=dict(boxstyle='round,pad=0.25', fc=col, ec='none'))
                xx += wdt + 1.5
            y -= line * 1.6
        else:
            chars = int(width / (fs * 0.3528 * 0.60))
            pre = {'b': '•  ', 'w': '⚠  ', 'p': '', 'n': ''}[kind]
            lines = textwrap.wrap(t, chars - len(pre))
            for i, l in enumerate(lines):
                bg.text(x + (0 if i == 0 else len(pre) * fs * 0.3528 * 0.5), y, (pre if i == 0 else '') + l,
                        fontsize=fs, va='top', color='#b03a2e' if kind == 'w' else ('#666' if kind == 'n' else '#222'))
                y -= line
            y -= line * 0.35
    return y


def step_page(pdf, n, title, subtitle, render_args, text, x4=False, label_text=None):
    fig, bg = page(pdf, n, title, subtitle)
    img, ann, arr = scene_render(**render_args)
    draw_scene(fig, [0.01, 0.04, 0.66, 0.84], img, ann, arr, label_text)
    if x4:
        bg.add_patch(FancyBboxPatch((W - 105, H - 40), 24, 12, boxstyle='round,pad=0,rounding_size=2', fc='#c0392b', ec='none'))
        bg.text(W - 93, H - 34, '× 4', ha='center', va='center', color='white', fontsize=15, fontweight='bold')
    text_block(bg, 200, H - 48 if x4 else H - 34, text, width=88)
    pdf.savefig(fig); plt.close(fig)
    print('step', n, title, flush=True)


def main(out):
    body = codes(lambda p: p.leg is None and p.kind != 'pin')
    legs = {k: [c for c in codes(lambda p, k=k: p.leg == k)] for k in G.LEGS}
    a4_pages = S.pack(S.prepare(PARTS, 277, 177), 277, 177, [None, 'ПЛ', 'ПП', 'ЗЛ', 'ЗП'])
    sheet_of = {}
    for i, (grp, pl) in enumerate(a4_pages):
        for it, _, _ in pl:
            sheet_of.setdefault(it.part.code, []).append(i + 1)
    leg_sheets = {k: sorted({s for c in legs[k] for s in sheet_of.get(c, [])}) for k in G.LEGS}
    body_sheets = sorted({s for c in body for s in sheet_of.get(c, [])})
    area = sum(p.poly.area for p in PARTS if p.kind != 'pin') / 1e6

    with PdfPages(out) as pdf:
        # ---------- cover
        fig, bg = page(pdf, None, 'Робособака из картона', 'Макет 1:1 по модели Spot Micro из репозитория Project_Robo_dog · инструкция по сборке')
        allc = [p.code for p in PARTS]
        img, ann, arr = scene_render([], allc, cam=(32, 20), hip=0.55, knee=-1.1, labels=[], size=(1700, 1150))
        draw_scene(fig, [0.0, 0.05, 0.70, 0.82], img, [], [])
        y = text_block(bg, 212, H - 40, [
            ('h', 'Что получится'),
            ('p', 'Картонная собака в натуральную величину: длина ≈ 32 см, ширина ≈ 22 см. '
                  'Лапы двигаются в бедре и колене — можно ставить в позу.'),
            ('h', 'Состав'),
            ('b', f'{len([p for p in PARTS if p.kind != "pin"])} картонных деталей: корпус (10) + 4 лапы по 19'),
            ('b', '2 шпажки-оси по 22 см + 4 кусочка по 4 см'),
            ('h', 'Цвета в инструкции'),
        ], width=76)
        for i, (k, (nm, _, _)) in enumerate(G.LEGS.items()):
            bg.add_patch(FancyBboxPatch((212, y - 4.2 - i * 6.5), 9, 4.6, boxstyle='round,pad=0,rounding_size=1', fc=LEG_COL[k], ec='none'))
            bg.text(224, y - 2 - i * 6.5, f'{k} — {nm}', fontsize=9, va='center')
        bg.add_patch(FancyBboxPatch((212, y - 4.2 - 4 * 6.5), 9, 4.6, boxstyle='round,pad=0,rounding_size=1', fc=BODY_NEW, ec='none'))
        bg.text(224, y - 2 - 4 * 6.5, 'К — корпус', fontsize=9, va='center')
        pdf.savefig(fig); plt.close(fig); print('cover', flush=True)

        # ---------- before you start
        fig, bg = page(pdf, None, 'Перед началом', 'Материалы, печать шаблонов и как читать названия деталей')
        text_block(bg, 10, H - 30, [
            ('h', 'Понадобится'),
            ('b', f'Гофрокартон {G.T:g} мм — около {area:.2f} м² (≈ 9 листов A4 или 2–3 стенки от коробки)'),
            ('b', 'Шпажки для шашлыка Ø3–4 мм, 25–30 см — 3 штуки'),
            ('b', 'Канцелярский нож, металлическая линейка, коврик для резки'),
            ('b', 'ПВА или термоклей; клей-карандаш для шаблонов; скотч; шило'),
            ('h', 'Печать шаблонов'),
            ('b', 'raskroy_A4.pdf (9 листов) или raskroy_A3.pdf (большие детали целиком).'),
            ('b', 'Печать 100% / «Фактический размер». Проверь линейкой шкалу внизу листа — ровно 100 мм.'),
            ('b', 'Детали (1/2) и (2/2) — половинки шаблона: стыкуй по красному пунктиру скотчем, потом клей на картон.'),
            ('h', 'Вырезание'),
            ('b', 'Наклей шаблон клеем-карандашом на картон (надписью вверх) и режь прямо по нему.'),
            ('b', 'Сначала серые прорези и окна, потом внешний контур.'),
            ('b', 'Отверстия ⊕ проткни шилом и расширь шпажкой.'),
            ('w', f'Картон толще {G.T:g} мм — расширь прорези ножом: шипы должны входить плотно.'),
            ('h', 'Как читать код детали'),
            ('p', 'ПЛ-Б1 = лапа ПЛ (передняя левая), Б — бедро, 1 — пластина ближе к корпусу. П — плечо, Б — бедро, Г — голень, Ш — шайба; номера 3–5 — распорки.'),
            ('p', 'Лево/право — как у собаки: стоишь позади неё и смотришь в сторону носа.'),
        ], width=118, fs=8.8)
        tags = {'ПЛ-Г2': 'ПЛ передняя левая', 'ЗЛ-Г2': 'ЗЛ задняя левая', 'ПП-Б1': 'ПП передняя правая',
                'ЗП-Б1': 'ЗП задняя правая', 'К4': 'К корпус', 'К6': 'НОС'}
        img, ann, arr = scene_render([], allc, cam=(38, 22), labels=list(tags), size=(1200, 1080))
        draw_scene(fig, [0.45, 0.04, 0.55, 0.84], img, ann, [], label_text=tags)
        pdf.savefig(fig); plt.close(fig); print('intro', flush=True)

        # ---------- body
        z = np.array([0, 0, 1.0]); yv = np.array([0, 1.0, 0])
        step_page(pdf, 1, 'Днище и перегородки', f'Корпус · листы раскроя {", ".join(map(str, body_sheets))}',
                  dict(old=[], new=['К3', 'К9', 'К10'], offsets={'К9': z * 75, 'К10': z * 75}),
                  [('parts', ['К3', 'К9', 'К10']),
                   ('b', 'Положи днище К3 надписью вниз (стрелка «к носу» смотрит вперёд).'),
                   ('b', 'Вставь нижние шипы перегородок К9 (передняя) и К10 (задняя) в прорези днища.'),
                   ('b', 'Окна в перегородках — для проводов, их можно не вырезать.'),
                   ('n', 'Пока не клей: сначала собери весь корпус «на сухую».')])
        step_page(pdf, 2, 'Борта', 'Левый К1 и правый К2',
                  dict(old=['К3', 'К9', 'К10'], new=['К1', 'К2'], offsets={'К1': yv * 85, 'К2': -yv * 85}),
                  [('parts', ['К1', 'К2']),
                   ('b', 'Надписи и пунктирные контуры «сюда плечо …» — снаружи.'),
                   ('b', 'Нижние шипы бортов — в прорези днища, боковые шипы перегородок — в прорези бортов.'),
                   ('b', 'Отверстия ⊕ в бортах — для осей лап, они должны быть в центре пунктирных контуров.'),
                   ('w', 'Не перепутай: на ЛЕВОМ борту К1 метки ПЛ и ЗЛ, на правом К2 — ПП и ЗП.')])
        ends = ['К5', 'К6', 'К7', 'К8']
        offs = {c: BY[c].label_n * 60 for c in ends}
        step_page(pdf, 3, 'Нос и корма', 'Торцевые панели',
                  dict(old=['К3', 'К9', 'К10', 'К1', 'К2'], new=ends, offsets=offs),
                  [('parts', ends),
                   ('b', 'Нос: К6 (нижний, большой) и К5 (верхний, узкий скос).'),
                   ('b', 'Корма: К8 (нижняя, с надписью ALA-TOO снаружи) и К7 (верхняя).'),
                   ('b', 'Шипы бортов входят в прорези панелей. Панели чуть шире бортов — так и задумано.'),
                   ('p', 'Теперь проверь, что всё сидит ровно, и проклей стыки изнутри ПВА или термоклеем.')])
        step_page(pdf, 4, 'Крыша', 'Съёмная крышка К4',
                  dict(old=['К3', 'К9', 'К10', 'К1', 'К2'] + ends, new=['К4'], offsets={'К4': z * 70}),
                  [('parts', ['К4']),
                   ('b', 'Надень крышу на верхние шипы бортов и перегородок, стрелкой «к носу».'),
                   ('b', 'Крышу можно не клеить — она держится на шипах и снимается, если внутрь потом пойдёт электроника.')])

        # ---------- leg sub-assemblies (shown on ПЛ)
        k = 'ПЛ'
        def sand(prefix, y_step):
            cs = leg_codes(k, (prefix,))
            plates = [c for c in cs if c.endswith(('1', '2')) and BY[c].kind == 'plate']
            sp = [c for c in cs if BY[c].kind == 'spacer']
            off = {plates[0]: -yv * 0, plates[1]: yv * y_step * 2}
            off.update({c: yv * y_step for c in sp})
            return plates, sp, off
        pl, sp, off = sand('П', 30)
        step_page(pdf, 5, 'Плечи', 'Собери 4 штуки: ПЛ, ПП, ЗЛ, ЗП',
                  dict(old=[], new=pl + sp, offsets=off, cam=(58, 22), size=(1400, 1080)),
                  [('parts', pl + sp),
                   ('b', 'Вставь шипы двух распорок (П3, П4) в прорези пластины П1.'),
                   ('b', 'Сверху надень пластину П2. Надписи на обеих пластинах — наружу, от корпуса.'),
                   ('b', 'Склей. Отверстия ⊕ в П1 и П2 должны совпасть.'),
                   ('n', f'На картинке — лапа ПЛ. Остальные лапы — так же, детали с кодами ПП-, ЗЛ-, ЗП-.'),
                   ('n', 'Листы раскроя: ' + ', '.join(f'{kk} — {",".join(map(str, leg_sheets[kk]))}' for kk in G.LEGS))],
                  x4=True)
        pl, sp, off = sand('Б', 45)
        step_page(pdf, 6, 'Бёдра', 'Верхняя часть лапы · 4 штуки',
                  dict(old=[], new=pl + sp, offsets=off, cam=(58, 18), size=(1400, 1080)),
                  [('parts', pl + sp),
                   ('b', 'Три распорки Б3, Б4, Б5 вставь в пластину Б1, сверху надень Б2.'),
                   ('b', 'Внизу между пластинами остаётся «вилка» — туда войдёт голень. Её не заклеивай.'),
                   ('b', 'Надписи на пластинах — наружу. Два отверстия ⊕: вверху — ось бедра, внизу — колено.')],
                  x4=True)
        pl, sp, off = sand('Г', 40)
        step_page(pdf, 7, 'Голени', 'Нижняя часть лапы со стопой · 4 штуки',
                  dict(old=[], new=pl + sp, offsets=off, cam=(58, 18), size=(1400, 1080)),
                  [('parts', pl + sp),
                   ('b', 'Распорки Г3 (вверху), Г4, Г5 (в стопе) — в пластину Г1, сверху Г2.'),
                   ('b', 'Стопа — широкая часть внизу, носком вперёд.'),
                   ('b', 'Отверстие ⊕ вверху — колено.')],
                  x4=True)
        up = leg_codes(k, ('Б',)); lo = leg_codes(k, ('Г',))
        ws = [f'{k}-Ш{i}' for i in (1, 2, 3, 4)]
        off = {f'{k}-Ш1': -yv * 35, f'{k}-Ш4': yv * 35, f'{k}-шп': yv * 95,
               f'{k}-Ш2': np.array([0, -1.0, -1.0]) * 22 * 0, f'{k}-Ш3': np.array([0, 0, 0.0])}
        for c in lo:
            off[c] = np.array([0, 0, -45.0])
        step_page(pdf, 8, 'Колено', 'Соедини бедро и голень · 4 раза',
                  dict(old=up, new=lo + ws + [f'{k}-шп'], offsets=off, cam=(22, 12), size=(1400, 1080),
                       arrow_for=[f'{k}-Г2', f'{k}-Ш1', f'{k}-Ш4', f'{k}-шп'],
                       labels=ws + [f'{k}-шп', f'{k}-Г2', f'{k}-Б2']),
                  [('parts', ws + [f'{k}-шп']),
                   ('b', 'Вставь верх голени в «вилку» бедра. С обеих сторон голени, между ней и бедром, положи по шайбе (Ш2, Ш3).'),
                   ('b', 'Проткни шпажку 4 см насквозь: бедро → шайба → голень → шайба → бедро.'),
                   ('b', 'Наклей шайбы-заглушки Ш1 и Ш4 на концы шпажки. Клей только к шпажке, не к лапе!'),
                   ('w', 'Колено должно вращаться туго — тогда лапа держит позу. Болтается — подложи бумажное колечко.')],
                  x4=True, label_text={f'{k}-шп': 'шпажка 4 см', f'{k}-Г2': 'голень', f'{k}-Б2': 'бедро'})

        # ---------- shoulders onto the body
        bodyall = body
        shs = [c for kk in G.LEGS for c in leg_codes(kk, ('П',))]
        off = {}
        for c in shs:
            off[c] = yv * 55 * G.LEGS[BY[c].leg][2]
        step_page(pdf, 9, 'Плечи на корпус', 'Приклей 4 плеча к бортам',
                  dict(old=bodyall, new=shs, offsets=off, cam=(35, 24), arrow_for=[f'{kk}-П2' for kk in G.LEGS],
                       labels=[f'{kk}-П2' for kk in G.LEGS], size=(1500, 1080)),
                  [('parts', [f'{kk}-П1' for kk in G.LEGS]),
                   ('b', 'Плечо ставится пластиной П1 на пунктирный контур «сюда плечо …» на борту.'),
                   ('b', 'Совмести отверстие плеча с отверстием в борту — проверь, проткнув шпажкой насквозь.'),
                   ('b', 'Приклей П1 к борту. Пока клей сохнет, оставь шпажку в отверстии — она держит соосность.'),
                   ('w', 'ПЛ и ЗЛ — на левый борт К1, ПП и ЗП — на правый К2.')],
                  label_text={f'{kk}-П2': f'плечо {kk}' for kk in G.LEGS})

        # ---------- legs on axles
        legparts = [c for kk in G.LEGS for c in leg_codes(kk, ('Б', 'Г', 'Ш', 'шп'))]
        off = {}
        for c in legparts:
            off[c] = yv * 70 * G.LEGS[BY[c].leg][2]
        axles = ['ОСЬ-перед', 'ОСЬ-зад']
        for c in axles:
            off[c] = yv * 260
        caps = [f'{kk}-Ш5' for kk in G.LEGS]
        for c in caps:
            off[c] = yv * 120 * G.LEGS[BY[c].leg][2]
        step_page(pdf, 10, 'Лапы на оси', 'Длинная шпажка проходит через всю собаку',
                  dict(old=body + shs, new=legparts + axles, offsets=off, cam=(35, 24), size=(1500, 1080),
                       arrow_for=axles + [f'{kk}-Б2' for kk in G.LEGS],
                       labels=axles + [f'ПЛ-Б2', f'ЗЛ-Б2'] + ['ПЛ-Ш5']),
                  [('parts', axles + caps),
                   ('b', 'Приложи лапу бедром к плечу так, чтобы верхние отверстия совпали.'),
                   ('b', 'Проткни шпажку 22 см: левая лапа → левое плечо → левый борт → корпус → правый борт → правое плечо → правая лапа.'),
                   ('b', 'Так же для задних лап. Концы шпажки закрой шайбами Ш5 (приклей к шпажке).'),
                   ('n', 'Шпажку можно капнуть клеем в плечах — тогда она не выпадет, а бёдра будут вращаться на ней.')],
                  label_text={'ОСЬ-перед': 'ось 22 см (перед)', 'ОСЬ-зад': 'ось 22 см (зад)', 'ПЛ-Б2': 'лапа ПЛ',
                              'ЗЛ-Б2': 'лапа ЗЛ', 'ПЛ-Ш5': 'Ш5 заглушка'})

        # ---------- done
        fig, bg = page(pdf, 11, 'Готово!', 'Поставь собаку в позу')
        img, _, _ = scene_render([], allc, cam=(-38, 18), hip=0.55, knee=-1.1, labels=[], size=(1100, 900))
        draw_scene(fig, [0.0, 0.08, 0.5, 0.78], img, [], [])
        img2, _, _ = scene_render([], allc, cam=(90, 8), hip=0.0, knee=0.0, labels=[], size=(1100, 900))
        draw_scene(fig, [0.48, 0.30, 0.34, 0.56], img2, [], [])
        bg.text(W * 0.48 + 50, 60, 'Вид слева, лапы прямо', fontsize=8, color='#777', ha='center')
        text_block(bg, 150, 52, [
            ('b', 'Стойка: колено отведи назад, стопу поставь вперёд под корпус — как на картинке слева.'),
            ('b', 'Туго ходят суставы — расширь отверстие шпажкой. Болтаются — капля ПВА на шпажку и подсушить.'),
            ('b', 'Размеры как у настоящего Spot Micro: между осями плеч 186 мм, бедро 107.5 мм, голень 130 мм — можно примерять электронику.'),
        ], width=140, fs=9)
        pdf.savefig(fig); plt.close(fig)

        # ---------- parts list
        fig, bg = page(pdf, None, 'Список деталей', 'Код · название · лист раскроя A4')
        rows = [(c, BY[c].title, ', '.join(map(str, sheet_of.get(c, [])))) for c in body]
        y = H - 34
        bg.text(10, y, 'КОРПУС', fontsize=11, fontweight='bold', color='#9a6a35'); y -= 7
        for c, t, s in rows:
            bg.text(12, y, c, fontsize=9, fontweight='bold'); bg.text(28, y, t, fontsize=9); bg.text(95, y, f'лист {s}', fontsize=9, color='#666')
            y -= 5.6
        y -= 4
        bg.text(10, y, 'ШПАЖКИ (не вырезать)', fontsize=11, fontweight='bold', color='#9a7a35'); y -= 7
        for t in ('ось передняя — 22 см', 'ось задняя — 22 см', 'колено — 4 см × 4'):
            bg.text(12, y, t, fontsize=9); y -= 5.6
        x0 = 125
        y = H - 34
        bg.text(x0, y, 'КАЖДАЯ ЛАПА (одинаковый набор)', fontsize=11, fontweight='bold'); y -= 7
        legrows = [('П1', 'Плечо, пластина к корпусу'), ('П2', 'Плечо, наружная пластина'), ('П3, П4', 'Распорки плеча'),
                   ('Б1', 'Бедро, внутренняя пластина'), ('Б2', 'Бедро, наружная пластина'), ('Б3–Б5', 'Распорки бедра'),
                   ('Г1', 'Голень, внутренняя пластина'), ('Г2', 'Голень, наружная пластина'), ('Г3–Г5', 'Распорки голени'),
                   ('Ш1–Ш5', 'Шайбы Ø16 (колено ×4, ось ×1)')]
        for c, t in legrows:
            bg.text(x0 + 2, y, c, fontsize=9, fontweight='bold'); bg.text(x0 + 22, y, t, fontsize=9); y -= 5.6
        y -= 5
        for i, (kk, (nm, _, _)) in enumerate(G.LEGS.items()):
            bg.add_patch(FancyBboxPatch((x0 + 2, y - 3.6), 12, 5, boxstyle='round,pad=0,rounding_size=1', fc=LEG_COL[kk], ec='none'))
            bg.text(x0 + 8, y - 1.1, kk, fontsize=9, color='white', fontweight='bold', ha='center', va='center')
            bg.text(x0 + 17, y - 1.1, f'{nm} — коды {kk}-П1 … {kk}-Ш5 — лист {", ".join(map(str, leg_sheets[kk]))}', fontsize=9, va='center')
            y -= 7.5
        pdf.savefig(fig); plt.close(fig)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'instrukciya.pdf')

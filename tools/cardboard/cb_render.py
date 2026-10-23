"""Tiny z-buffer renderer for extruded flat parts -> 'instruction manual' style images (flat shade + outlines)."""
import numpy as np
import shapely


def part_mesh(p):
    """Triangles (k,3,3) in part-local 3D and per-triangle normals."""
    tris, norms = [], []
    poly = p.poly
    geoms = getattr(poly, 'geoms', [poly])
    h = p.thick / 2
    for g in geoms:
        tri = shapely.constrained_delaunay_triangles(g)
        for t in tri.geoms:
            uv = np.array(t.exterior.coords)[:3]
            for s in (+1, -1):
                P = p.w(uv) + p.n * h * s
                tris.append(P if s > 0 else P[::-1]); norms.append(p.n * s)
        for ring in [g.exterior] + list(g.interiors):
            uv = np.array(ring.coords)
            P = p.w(uv)
            for i in range(len(P) - 1):
                a, b = P[i], P[i + 1]
                e = b - a
                if np.linalg.norm(e) < 1e-6:
                    continue
                nn = np.cross(e, p.n); nn /= np.linalg.norm(nn)
                a0, a1, b0, b1 = a - p.n * h, a + p.n * h, b - p.n * h, b + p.n * h
                tris += [np.array([a0, b0, b1]), np.array([a0, b1, a1])]; norms += [nn, nn]
    return np.array(tris), np.array(norms)


class Camera:
    def __init__(self, az=35, el=24):
        az, el = np.radians(az), np.radians(el)
        self.c = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])  # toward camera
        f = -self.c
        self.r = np.cross(f, [0, 0, 1]); self.r /= np.linalg.norm(self.r)
        self.u = np.cross(self.r, f)

    def proj(self, P):
        P = np.asarray(P)
        return np.stack([P @ self.r, P @ self.u, P @ self.c], -1)


def hexrgb(h):
    h = h.lstrip('#'); return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4)]) / 255.0


def render(items, cam, W=1600, H=1100, ss=2, margin=0.06, fit=None, light=(0.35, -0.55, 0.85)):
    """items: list of (part, M4x4, color_hex). Returns (rgb image HxWx3, project_fn(P3)->(px,py) in final pixels)."""
    L = np.asarray(light, float); L /= np.linalg.norm(L)
    allT, allN, ids, cols = [], [], [], []
    for k, (p, M, col) in enumerate(items):
        if not hasattr(p, '_mesh'):
            p._mesh = part_mesh(p)
        tr, nr = p._mesh
        R, t = M[:3, :3], M[:3, 3]
        allT.append(tr @ R.T + t); allN.append(nr @ R.T)
        ids.append(np.full(len(tr), k)); cols.append(np.tile(hexrgb(col), (len(tr), 1)))
    Tw = np.concatenate(allT); Nw = np.concatenate(allN); ID = np.concatenate(ids); C = np.concatenate(cols)
    S = cam.proj(Tw)
    if fit is None:
        pts = S[:, :, :2].reshape(-1, 2)
        fit = (pts.min(0), pts.max(0))
    lo, hi = fit
    w, h = W * ss, H * ss
    scale = min(w * (1 - 2 * margin) / (hi[0] - lo[0]), h * (1 - 2 * margin) / (hi[1] - lo[1]))
    cx, cy = (lo + hi) / 2

    def to_px(S2):
        return np.stack([(S2[..., 0] - cx) * scale + w / 2, h / 2 - (S2[..., 1] - cy) * scale], -1)

    X = to_px(S[:, :, :2]); Z = S[:, :, 2]
    zbuf = np.full((h, w), -np.inf); idb = np.full((h, w), -1, int)
    nbuf = np.zeros((h, w, 3)); cbuf = np.ones((h, w, 3))
    view_n = Nw @ cam.c
    shade = 0.62 + 0.38 * np.clip(Nw @ L, -1, 1) * np.sign(view_n + 1e-9)
    shade = np.clip(0.55 + 0.45 * np.abs(Nw @ L), 0, 1) * 0.6 + 0.4 * (0.55 + 0.45 * np.abs(view_n))
    for i in range(len(Tw)):
        x, y = X[i, :, 0], X[i, :, 1]
        x0, x1 = int(max(np.floor(x.min()), 0)), int(min(np.ceil(x.max()), w - 1))
        y0, y1 = int(max(np.floor(y.min()), 0)), int(min(np.ceil(y.max()), h - 1))
        if x1 < x0 or y1 < y0:
            continue
        den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        if abs(den) < 1e-9:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        l0 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
        l1 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
        l2 = 1 - l0 - l1
        m = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        if not m.any():
            continue
        z = l0 * Z[i, 0] + l1 * Z[i, 1] + l2 * Z[i, 2]
        sub = zbuf[y0:y1 + 1, x0:x1 + 1]
        m &= z > sub
        if not m.any():
            continue
        sub[m] = z[m]
        idb[y0:y1 + 1, x0:x1 + 1][m] = ID[i]
        nbuf[y0:y1 + 1, x0:x1 + 1][m] = Nw[i]
        cbuf[y0:y1 + 1, x0:x1 + 1][m] = C[i] * shade[i]
    # outlines
    e = np.zeros((h, w), bool)
    for dy, dx in ((0, 1), (1, 0)):
        a_id, b_id = idb[:h - dy, :w - dx], idb[dy:, dx:]
        a_n, b_n = nbuf[:h - dy, :w - dx], nbuf[dy:, dx:]
        a_z, b_z = zbuf[:h - dy, :w - dx], zbuf[dy:, dx:]
        fg = (a_id >= 0) | (b_id >= 0)
        with np.errstate(invalid='ignore'):
            dz = np.abs(np.nan_to_num(a_z - b_z, nan=0.0, posinf=1e3, neginf=1e3))
        diff = fg & ((a_id != b_id) | ((a_n * b_n).sum(-1) < 0.8) | (dz > 3.0))
        e[:h - dy, :w - dx] |= diff
    # thicken a little
    e2 = e.copy(); e2[1:] |= e[:-1]; e2[:, 1:] |= e[:, :-1]
    img = cbuf.copy(); img[idb < 0] = 1.0
    img[e2] = img[e2] * 0.15 + np.array([0.13, 0.13, 0.15]) * 0.85
    img = img.reshape(H, ss, W, ss, 3).mean((1, 3))

    def project(P3):
        s = cam.proj(np.asarray(P3, float))
        return to_px(s[..., :2]) / ss

    return img, project

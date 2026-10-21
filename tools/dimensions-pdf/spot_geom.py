"""Load SpotMicro RViz STLs and assemble them per spot_micro.urdf.xacro (all joints = 0). Units: mm."""
import numpy as np, os
STL_DIR = os.path.expanduser('~/spotMicro/spot_micro_rviz/urdf/stl')

def load_stl(name):
    b = open(os.path.join(STL_DIR, name), 'rb').read()
    n = int(np.frombuffer(b[80:84], '<u4')[0])
    d = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return d['v'].astype(float)  # (n,3,3)

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def T(xyz=(0, 0, 0), r=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = rpy(*r); M[:3, 3] = np.array(xyz) * 1000.0; return M

def apply(M, tris):
    return tris @ M[:3, :3].T + M[:3, 3]

PI = 3.14159
def assembly():
    """Return list of (group, tris_world)."""
    out = []
    add = lambda g, M, f: out.append((g, apply(M, load_stl(f))))
    base = np.eye(4)
    add('body', base @ T((-0.042, -0.055, -0.010)), 'mainbody.stl')
    add('body', base @ T((0.04, 0.055, -0.010), (0, 0, PI)), 'backpart.stl')
    add('body', base @ T((0.04, 0.055, -0.010), (0, 0, PI)), 'frontpart.stl')
    sx, sy, sh, L2 = 0.093, 0.039, 0.055, 0.1075
    for px in (+1, -1):
        for left in (True, False):
            ys = 1 if left else -1
            S = base @ T((px * sx, ys * sy, 0))
            if left: add('shoulder', S @ T((0.135, 0.015, -0.01), (0, 0, PI)), 'lshoulder.stl')
            else:    add('shoulder', S @ T((0.135, 0.095, -0.01), (0, 0, PI)), 'rshoulder.stl')
            Lg = S @ T((0, ys * sh, 0))
            o = (0.130, -0.040, -0.025) if left else (0.130, 0.15, -0.025)
            add('upper', Lg @ T(o, (0, -0.139, PI)), 'larm.stl' if left else 'rarm.stl')
            add('upper', Lg @ T(o, (0, -0.139, PI)), 'larm_cover.stl' if left else 'rarm_cover.stl')
            F = Lg @ T((0, 0, -L2))
            o = (0.120, -0.04, 0.1) if left else (0.120, 0.15, 0.1)
            add('lower', F @ T(o, (0, 0, PI)), 'lfoot.stl' if left else 'rfoot.stl')
            Tt = F @ T((0, 0, -0.130))
            add('toe', Tt @ T((0.0, 0.01, 0.015), (0, -0.40010, PI)), 'foot.stl')
    return out

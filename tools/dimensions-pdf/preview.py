import sys, numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
sys.path.insert(0, sys.argv[1]); import spot_geom as G
parts = G.assembly(); allt = np.concatenate([t for _, t in parts])
print('assembly bbox', np.round(allt.reshape(-1,3).min(0),1), np.round(allt.reshape(-1,3).max(0),1))
fig, axs = plt.subplots(1, 3, figsize=(18, 6))
for ax, (a, b, d) in zip(axs, [(0, 1, 2), (0, 2, 1), (1, 2, 0)]):
    t = allt; z = t[:, :, d].mean(1); o = np.argsort(z)
    ax.add_collection(PolyCollection(t[o][:, :, [a, b]], facecolor='0.7', edgecolor='0.3', lw=0.05))
    ax.autoscale(); ax.set_aspect('equal'); ax.grid(alpha=.3)
fig.savefig(sys.argv[2], dpi=70)

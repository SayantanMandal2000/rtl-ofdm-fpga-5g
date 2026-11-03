#!/usr/bin/env python3
"""Plot the RX constellations dumped by tb_ofdm_top (constellation.csv).

Usage:  python3 plot_constellation.py [constellation.csv] [out.png]
"""
import sys
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src = sys.argv[1] if len(sys.argv) > 1 else "constellation.csv"
dst = sys.argv[2] if len(sys.argv) > 2 else "constellation.png"

titles = {
    1: "Test 1: phase compensation OFF",
    2: "Test 2: phase compensation ON (chaos ON)",
    3: "Test 3: wrong RX key (eavesdropper)",
    4: "Test 4: chaos OFF, phase comp ON",
}
pts = {k: ([], []) for k in titles}
with open(src) as f:
    for row in csv.reader(f):
        if len(row) != 3:
            continue
        t, i, q = int(row[0]), int(row[1]), int(row[2])
        if t in pts:
            pts[t][0].append(i / 2048.0)
            pts[t][1].append(q / 2048.0)

fig, axes = plt.subplots(1, 4, figsize=(16, 4.3))
for ax, (t, title) in zip(axes, titles.items()):
    ax.scatter(pts[t][0], pts[t][1], s=4, color="#1f5fa8")
    ax.set_title(title, fontsize=10)
    ax.set_xlim(-4.5, 4.5); ax.set_ylim(-4.5, 4.5)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.axhline(0, color="#888", lw=0.6); ax.axvline(0, color="#888", lw=0.6)
    ax.set_xlabel("I"); ax.set_ylabel("Q")
fig.suptitle("Received 16-QAM constellation after FFT and chaotic de-rotation")
fig.tight_layout()
fig.savefig(dst, dpi=130)
print("wrote", dst)

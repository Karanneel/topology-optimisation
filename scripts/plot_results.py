import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "results" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

cfg = json.loads((ROOT / "config" / "engineering_inputs.json").read_text())
with open(ROOT / "results" / "design_summary.csv") as f:
    s = {r["geometry"]: {k: (v if k == "geometry" else float(v)) for k, v in r.items()} for r in csv.DictReader(f)}

# fixed colour per design, same in every figure
COL = {"handling": "#2a78d6", "rotary": "#eb6834", "dual": "#1baf7a", "baseline": "#8f8e88"}
NAME = {"handling": "Handling-only", "rotary": "Rotary-only", "dual": "Dual-mode", "baseline": "Solid baseline"}
INK, MUTED, GRID, BG = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "figure.facecolor": BG,
                     "axes.facecolor": BG, "savefig.facecolor": BG})


def tidy(ax):
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


# mass
mass = cfg["final_mass_g"]
keys = ["baseline", "handling", "rotary", "dual"]
fig, ax = plt.subplots(figsize=(6.4, 3.4))
bars = ax.bar([NAME[k] for k in keys], [mass[k] for k in keys], width=0.6,
              color=[COL[k] for k in keys], edgecolor=BG, linewidth=1.5)
for b, k in zip(bars, keys):
    ax.text(b.get_x() + b.get_width()/2, mass[k] + 25, f"{mass[k]:.1f} g", ha="center", color=INK)
target = cfg["budget"]["target_mass_g"]
ax.axhline(target, color=MUTED, linestyle="--", linewidth=1)
ax.text(3.3, 2300, f"dashed line: target {target:.1f} g", ha="right", color=MUTED, fontsize=8)
ax.set_ylabel("Mass (g), density 2700 kg/m³")
ax.set_ylim(0, 2450)
tidy(ax)
fig.tight_layout()
fig.savefig(OUT / "fig_mass.png", dpi=200)

# cross load, normalised compliance on log axis bc handling-only under B is ~53x
designs = ["handling", "rotary", "dual"]
fig, ax = plt.subplots(figsize=(6.4, 3.6))
x = np.arange(2)
wd = 0.26
for i, g in enumerate(designs):
    vals = [s[g]["CA_norm"], s[g]["CB_norm"]]
    pos = x + (i - 1)*wd
    ax.bar(pos, vals, wd, color=COL[g], edgecolor=BG, linewidth=1.5, label=NAME[g])
    for p, v in zip(pos, vals):
        ax.text(p, v*1.08, f"{v:.2f}", ha="center", color=INK, fontsize=8)
ax.axhline(1, color=MUTED, linestyle="--", linewidth=1)
ax.text(0.5, 1.06, "solid = 1", ha="center", color=MUTED, fontsize=8)
ax.set_yscale("log")
ax.set_ylim(0.8, 120)
ax.set_yticks([1, 2, 5, 10, 20, 50, 100], ["1", "2", "5", "10", "20", "50", "100"])
ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
ax.set_xticks(x, ["Mode A, handling (bending)", "Mode B, rotary (torsion + thrust)"])
ax.set_ylabel("Compliance / solid baseline (log)")
ax.legend(frameon=False, loc="upper left")
tidy(ax)
fig.tight_layout()
fig.savefig(OUT / "fig_cross_load.png", dpi=200)

# how the rotary vs dual ranking moves if the mass difference is corrected, C ~ m^-k
k = np.linspace(0, 3, 61)
md = mass["dual"]
fig, ax = plt.subplots(figsize=(6.4, 3.4))
for g in ("rotary", "dual"):
    y = s[g]["score"] * (mass[g] / md)**k
    ax.plot(k, y, color=COL[g], linewidth=2, label=NAME[g])
    ax.text(3.04, y[-1], NAME[g], color=INK, va="center", fontsize=8)
kx = np.log(s["dual"]["score"] / s["rotary"]["score"]) / np.log(mass["rotary"] / md)
ax.axvline(kx, color=MUTED, linestyle=":", linewidth=1)
ax.text(kx + 0.06, 1.6375, f"ranking swaps at k = {kx:.2f}", color=MUTED, fontsize=8)
ax.set_xlim(0, 3.6)
ax.set_xlabel("Mass-correction exponent k  (C scaled by (m / m_dual)^k)")
ax.set_ylabel("Score (mean normalised compliance)")
ax.legend(frameon=False, loc="upper left")
tidy(ax)
fig.tight_layout()
fig.savefig(OUT / "fig_mass_correction.png", dpi=200)

print("figures written to results/figures")

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATA = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "raw"
cfg = json.loads((ROOT / "config" / "engineering_inputs.json").read_text())
MASS = cfg["final_mass_g"]

# nTop point maps, no header: x y z in mm, displacement in m, stress in Pa, reactions in N
files = {
    ("baseline_4mm", "A"): ("static a solid.csv", "STRESS_MAP_A.csv", None),
    ("baseline_4mm", "B"): ("static b solid.csv", "baseline_B_stress.csv", None),
    ("baseline", "A"): ("baseline_A_displacement_2p5mm.csv", "baseline_A_stress_2p5mm.csv", "reactions_A_2p5mm.csv"),
    ("baseline", "B"): ("baseline_B_displacement_2p5mm.csv", "baseline_B_stress_2p5mm.csv", "reactions_B_2p5mm.csv"),
}
for g in ("handling", "rotary", "dual"):
    for c in "AB":
        files[(g, c)] = (f"{g}_final_{c}_displacement.csv",
                         f"{g}_final_{c}_stress.csv",
                         f"{g}_final_{c}_reactions.csv")


def load(name):
    a = np.loadtxt(DATA / name, delimiter=",", ndmin=2)
    if not np.isfinite(a).all():
        raise ValueError(f"non-finite values in {name}")
    return a


def interface(d, tol):
    x, y, z = d[:, 0], d[:, 1], d[:, 2]
    pad = (abs(z - 35) < tol) & ((x - 185)**2 + y**2 < 26.5**2)
    flange = (abs(x - 240) < tol) & (y**2 + z**2 < 40.5**2)
    return pad, flange


def twist(d, sel):
    # least squares rigid rotation about x, same idea as the rbe3 (flexible) connector
    y = d[sel, 1] - d[sel, 1].mean()
    z = d[sel, 2] - d[sel, 2].mean()
    uy = d[sel, 4] - d[sel, 4].mean()
    uz = d[sel, 5] - d[sel, 5].mean()
    return np.sum(y*uz - z*uy) / np.sum(y*y + z*z)


def compliance(d, case, tol):
    # external work F.u (+ M.theta for B), mm -> m for the force terms
    pad, flange = interface(d, tol)
    if case == "A":
        return 250 * -d[pad, 5].mean() / 1000
    return 300 * -d[flange, 3].mean() / 1000 + 24 * twist(d, flange)


def write(path, rows):
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, restval="")
        w.writeheader()
        for r in rows:
            w.writerow({k: f"{v:.8g}" if isinstance(v, float) else v for k, v in r.items()})


def main():
    rows, disp = [], {}
    print("root reactions (expected A: Fz +250 N, My -46.25 N*m | B: Fx +300 N, Mx -24 N*m)")
    for (geo, case), (fd, fs, fr) in files.items():
        d = load(fd)
        d[:, 3:6] *= 1000
        disp[(geo, case)] = d
        s = load(fs)[:, 3] / 1e6

        # 0.1 mm band = the 0.2 mm selector boxes used for the loads in nTop
        pad, flange = interface(d, 0.1)
        if pad.sum() < 20 or flange.sum() < 20:
            raise ValueError(f"interface selection failed for {geo} {case}")

        row = {"geometry": geo, "case": case, "nodes": len(d),
               "pad_nodes": int(pad.sum()), "flange_nodes": int(flange.sum()),
               "max_disp_mm": float(np.linalg.norm(d[:, 3:6], axis=1).max()),
               "pad_uz_mm": float(d[pad, 5].mean()),
               "flange_ux_mm": float(d[flange, 3].mean()),
               "flange_twist_deg": float(np.degrees(twist(d, flange))),
               "compliance_J": float(compliance(d, case, 0.1)),
               "peak_vm_MPa": float(s.max()),
               "vm_p999_MPa": float(np.percentile(s, 99.9))}

        if fr:
            r = load(fr)
            hit = abs(r[:, 3:6]).max(axis=1) > 1e-8
            F = r[hit, 3:6].sum(axis=0)
            M = np.cross(r[hit, :3] / 1000, r[hit, 3:6]).sum(axis=0)
            row["react_nodes"] = int(hit.sum())
            row.update({k: float(v) for k, v in zip(("Fx", "Fy", "Fz", "Mx", "My", "Mz"), (*F, *M))})
            print(f"  {geo:9} {case}  {hit.sum():4} nodes  F {np.round(F, 4)}  M {np.round(M, 4)}")
        rows.append(row)

    C = {(r["geometry"], r["case"]): r["compliance_J"] for r in rows}
    ca0, cb0 = C[("baseline", "A")], C[("baseline", "B")]

    print("\nbaseline compliance 4 mm -> 2.5 mm")
    for c, fine in (("A", ca0), ("B", cb0)):
        coarse = C[("baseline_4mm", c)]
        print(f"  {c}: {coarse:.6f} -> {fine:.6f} J  ({100*(fine/coarse - 1):+.2f}%)")

    w = cfg["optimization"]["dual_weights"]
    print(f"\ndual weight ratio B/A: used {w['B']/w['A']:.4f}, solid baseline gives {ca0/cb0:.4f}")

    print("\nnormalised to the solid baseline")
    summary = []
    for g in ("handling", "rotary", "dual"):
        a, b = C[(g, "A")] / ca0, C[(g, "B")] / cb0
        summary.append({"geometry": g, "mass_g": MASS[g], "CA_J": C[(g, "A")], "CB_J": C[(g, "B")],
                        "CA_norm": a, "CB_norm": b, "score": 0.5*(a + b), "worst": max(a, b)})
        print(f"  {g:9} {MASS[g]:8.2f} g   C_A/C_A0 {a:7.4f}   C_B/C_B0 {b:8.4f}   "
              f"score {0.5*(a + b):8.4f}   worst {max(a, b):8.4f}")

    s = {r["geometry"]: r for r in summary}
    pa = 100 * (C[("dual", "A")] / C[("handling", "A")] - 1)
    pb = 100 * (C[("dual", "B")] / C[("rotary", "B")] - 1)
    print(f"\ndual vs specialists: P_A {pa:+.2f}%  P_B {pb:+.2f}%")
    print(f"handling-only under B vs dual under B: {C[('handling', 'B')]/C[('dual', 'B')]:.2f}x")

    # the three bodies don't have the same mass after reconstruction,
    # so check if that alone can swap rotary and dual. assumes C ~ m^-k
    print("\nscore gap dual vs rotary with mass correction (scaled to dual mass)")
    for k in (0, 1, 2):
        rot = s["rotary"]["score"] * (MASS["rotary"] / MASS["dual"])**k
        print(f"  k={k}: rotary {rot:.4f}  dual {s['dual']['score']:.4f}  gap {100*(s['dual']['score']/rot - 1):+.2f}%")

    print("\nscore gap dual vs rotary for different selection bands")
    for tol in (0.1, 0.3, 0.5):
        c = {key: compliance(d, key[1], tol) for key, d in disp.items() if key[0] != "baseline_4mm"}
        score = {g: 0.5*(c[(g, "A")] / c[("baseline", "A")] + c[(g, "B")] / c[("baseline", "B")])
                 for g in ("rotary", "dual")}
        print(f"  +-{tol} mm: rotary {score['rotary']:.4f}  dual {score['dual']:.4f}  "
              f"gap {100*(score['dual']/score['rotary'] - 1):+.2f}%")

    write(ROOT / "results" / "tooling_arm_metrics.csv", rows)
    write(ROOT / "results" / "design_summary.csv", summary)
    print("\nwrote results/tooling_arm_metrics.csv and results/design_summary.csv")


if __name__ == "__main__":
    main()

# Baseline CAD

`tooling_arm_baseline_v02.step` is the baseline arm exported from the parametric Fusion model (v02). This file was imported into nTop for the study.

Reimported into Fusion, it is one closed solid with 36 faces, a bounding box of 240 × 80 × 80 mm and a volume of 808 365 mm³. The root, pad and flange holes are in their intended positions. The load and support faces have these areas:
- rear face at x = 0: 6263 mm²
- pad top at z = 35: 2068 mm²
- flange end at x = 240: 4932 mm²

`review/step_import_iso.png` shows the reimported part.

The Fusion file has steel assigned as its material. The study uses an aluminium benchmark (E = 70 GPa, ν = 0.33, ρ = 2700 kg/m³) set in nTop.

The STEP stores coordinates in centimetres (`SI_UNIT(.CENTI.,.METRE.)`). Fusion and nTop read this and import the part at the correct size. Forcing millimetres on import without reading the unit gives a part ten times too small.

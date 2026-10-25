# Dual-purpose robotic tooling arm: topology optimization study

One aluminium arm between a robot flange and two tools. One tool face carries a handling tool that bends the arm, the other a screwdriving tool that twists it. The two jobs never happen at the same time.

I wanted to know what happens to an optimized part when it has more than one job. So I optimized the same arm three times, for handling only, for rotation only and for both, and then tested every result under both loads.

| Handling-only | Rotary-only | Dual-mode |
|---|---|---|
| ![Handling-only](results/figures/topology_handling.png) | ![Rotary-only](results/figures/topology_rotary.png) | ![Dual-mode](results/figures/topology_dual.png) |

## How it was done

The baseline arm was modelled in Fusion with named parameters and taken into nTop 6.0.3 as STEP. The robot plate, the handling pad and the rotary flange were kept solid, and the optimizer was allowed to keep 35 % of the material in between.

The three optimizations used the same mesh, material, budget and settings; only the objective changed. The dual-mode run keeps the two load cases separate and adds their compliances with weights, instead of applying both loads at once.

Each result was rebuilt as a solid, remeshed and analysed under both load cases. With the solid baseline that makes eight static analyses. In all eight, the summed root reactions match the applied loads.

The exported results were evaluated with a small Python script: displacements, flange twist, compliance estimates and the reaction check.

## Results

| Design | Mass (g) | Handling compliance / solid | Rotary compliance / solid |
|---|---|---|---|
| Solid baseline | 2182.8 | 1.00 | 1.00 |
| Handling-only | 1143.3 | 1.36 | 52.98 |
| Rotary-only | 1162.8 | 1.62 | 1.65 |
| Dual-mode | 1156.6 | 1.51 | 1.79 |

![Compliance relative to the solid arm](results/figures/fig_cross_load.png)

All three designs are about 47 % lighter than the solid arm.

The handling-only design is stiff in bending but very weak in torsion. It narrows into a thin connection just before the rotary hub, and under the screwdriving load it is 29.7 times more compliant than the dual-mode design (flange twist 0.61° against 0.021°).

The rotary-only design holds up much better the other way round. Under the handling load it is only 18.9 % more compliant than the handling specialist.

The dual-mode design sits in between. It has 7.0 % lower handling compliance than rotary-only and 8.2 % higher rotary compliance.

Rotary-only against dual-mode is too close to call. Their mean scores differ by 0.65 %, but their masses also differ by 0.54 %, and a reasonable mass correction can flip the order.

## Choosing the weights

The two compliances have different sizes, and both grow with the square of the load, so simply adding them lets the bigger one drive the design. I divided each one by the compliance of the solid arm under the same load:

```
J = 0.5 * C_A / C_A0 + 0.5 * C_B / C_B0     ->     w_A = 0.5 / C_A0,  w_B = 0.5 / C_B0
```

nTop doesn't report compliance for a static analysis, so for the run itself I took the weights from the first iteration of the two single-mode runs (9.718 and 7.372). Afterwards I worked out the solid-arm compliances from the exported displacements. Their weight ratio is 0.7701 against the 0.7586 I used, a 1.5 % difference, and only the ratio matters.

One thing nearly went wrong here. The rotary compliance has to include the torque work, which is 96.5 % of the total. An early estimate left it out, and that gave a weight ratio of 21.7 instead of 0.77. It would have made the rotary case about 29 times too important.

## Limits

This is a simulation study:
- linear static, isotropic aluminium, rigid fixed root, no bolts or contact
- the loads are benchmark values, not from a real robot or tool
- the final designs were meshed at one size with linear elements
- the compliance values are estimates from the exported displacements
- nothing was printed or tested

The report covers each of these.

## Repository

```
docs/report.md, report.pdf     full report with method, results and limits
docs/figures/                  schematics of the geometry, load cases and preserved regions
cad/baseline/                  baseline STEP and import check
config/engineering_inputs.json geometry, loads, material, budget and solver settings
results/                       result tables and figures
scripts/analyze_tooling.py     evaluates the nTop exports
scripts/plot_results.py        draws the charts
ntop/                          note on the nTop notebook
```

The raw nTop exports (about 110 MB) and the nTop notebook (about 1 GB) are too big for GitHub and are available on request. With the exports in `results/raw/`:

```bash
pip install -r requirements.txt
python scripts/analyze_tooling.py
python scripts/plot_results.py
```

## Related

I learned the SIMP basics in my 2D [TO Playground](https://github.com/Karanneel/to-playground), a browser solver for trying out the parameters. It was not used for this study.

## License

MIT, see [LICENSE](LICENSE).

Karan Dutt Neelakantrao · [GitHub](https://github.com/Karanneel) · [LinkedIn](https://linkedin.com/in/karan-dutt-neelakantrao)

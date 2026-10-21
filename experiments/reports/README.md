# experiments/reports — 实验报告

One folder per study, written for the project owner (Chinese). Each `README.md`: question, method (models,
controllers, cost, optimizers, budgets), setup (configs, presets, seeds, spec hash), results (tables, SVG figures in
the same folder), conclusions and limitations. Every number must be reproducible from a config in
`experiments/configs/` and a run stamp in `experiments/runs/` (git-ignored); scripts that make the figures live
next to the study's tools (`tools/flight/<study>/`).

- `barrel-roll/` — MPC: leave the viaduct, jump, roll 360° about the long axis, land softly (twin + soft body)
- `gas-late-burn/` — MPC (gradient / sampling / hybrid): is "free fall first, burn late" the gas-optimal soft landing?
- `gas-two-costs/` — the FALL script's two experiments: MPC with a cost without / with a gas term ("lower the car slowly" vs
  "suicide burn"), gas used by each, trajectories for the film

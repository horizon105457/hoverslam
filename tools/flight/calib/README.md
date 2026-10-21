# Twin touchdown calibration

Fits the rigid twin's touchdown model (spec/README.md §5.11) to the soft-body landing experiments of
`tools/flight/touchdown/` and checks the result in closed loop. Report: `experiments/reports/touchdown-calibration/`.

| file | what |
|---|---|
| `lib.mjs` | load a touchdown-doe run, replay a case on a twin (from the soft body's precontact state), loss per case, RMS report |
| `params.mjs` | fitted parameters (unit box ↔ physical ↔ `vehicle` override), fixed measured values, the 30 underbody points |
| `cma.mjs` | (μ/μ_w, λ)-CMA-ES with mirrored box bounds, batch evaluation |
| `fit.mjs` | CMA-ES on the fit + anchor cases, worker threads; writes `fit.json` (`CALIB_WEIGHT='{"lat":4}'` overrides loss weights) |
| `validate.mjs` | RMS per quantity spec vs fit per split, per-wheel load thresholds, case-level failure confusion |
| `criteria.mjs` | the MPC land objective's failure criteria as the planning twin sees them: impact contact-patch lateral speed, late wheel lateral-load EMA (best split, 90 % recall, held-out scores, OR rule) |
| `accept.mjs` | soft-body closed loop: launch preset × seed × manoeuvre through `tools/sim/flight.mjs`, task / touchdown verdicts, Wilson 95 % intervals; `root=` another source tree, `summary=1` re-summarise |

```
node tools/flight/calib/fit.mjs run=experiments/runs/touchdown-doe/<stamp>[,<stamp2>] out=experiments/runs/calib/<tag> [start=<fit.json>] [evals=9000]
     [drive=experiments/runs/drive-doe/<tag> drive_w=3 drive_hold=steer15m,brake20h,bsteer18h,coast15] [set='{"slide":0.9}']
node tools/flight/calib/validate.mjs run=<doe run> fit=<fit.json>
node tools/flight/calib/criteria.mjs run=<doe run with the current hardware> fit=<fit.json>
node tools/flight/drive/compare.mjs run=experiments/runs/drive-doe/<tag> vehicle=<fit.json>
node tools/flight/calib/accept.mjs tag=<tag> [sets=land,roll] [par=10] [root=<tree>] [summary=1]
```
The fitted values go into `spec/vehicle.json` by hand (`_calibration` records the source); then `npm run spec`,
`npm run flight:goldens`, `npm test`.
Joint fit (2026-10-21, `coldgas-rigid@3`, `experiments/runs/calib/fit-drive`): both landing designs pooled (same lattice
physics until something breaks; they differ in connector strength, i.e. only in where the windows end) + 12 drive tests,
with the combined-slip terms `slip_ratio_sat` and `slide` free. The failure criteria come from the landing design that
matches the flight hardware (`20261021-083616`: connector strength 0.55, kit rear arms ×2) and the drive tests.

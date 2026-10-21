# Soft-body drive tests (brakes and steering)

The kit car rolling on open street under scripted brake / steer commands (`spec/drive.json`, soft-body hardware
`src/softbody/drive.js`): the reference data for the rigid twins' drive model (spec/README.md §5.13) and its joint
calibration with the touchdown experiments (`tools/flight/calib/fit.mjs drive=`).

| file | what |
|---|---|
| `test.mjs` | one case → `<outDir>/<id>.json.gz` (`drive-test@1`) + a one-line summary |
| `doe.mjs` | the 16-case design (`CASES`: coasting, ABS braking, steer steps / weave, brake + steer), parallel, resumable |
| `compare.mjs` | twin replays of a run vs the soft body (RMS of speed, lateral speed, yaw rate, heading; peak wheel lateral-load EMA of the MPC's late criterion), `driveLoss` for the fit |

```
node tools/flight/drive/test.mjs u=20 script='[[0.4,1,0]]' id=brake20 outDir=/tmp/drive   # brake fully from 0.4 s
node tools/flight/drive/doe.mjs run=experiments/runs/drive-doe/<tag> [workers=10]
node tools/flight/drive/compare.mjs run=experiments/runs/drive-doe/<tag> [vehicle=<fit.json | json override>]
```

**Placement.** The settled kit pose (`settled-coldgas.bin`) is put on the street at (`x0`, `z0`) = (60, 13), heading
−X (`heading` deg off it), velocity `u` along the heading, wheels rolling (`CrashSim.setup`'s launch placement moved off
the deck, no kerb kick), gas fill `gas` (0.65 ≈ after a landing burn). The rig arms at once (task arm `start`), the
script sets brake / steer per control step. Recorded at 100 Hz: rig estimate (p q v w), drive state, wheel spins
(relative to the body), speed along the heading `u`, lateral speed `vlat`, heading `psi`, yaw rate; `twin_init` at
`settle` (0.3 s: an explicit-state twin init with gas and measured wheel spins); `first_break` (first broken control
arm / spring), `broken` per wheel at the end. Connector strength `joints` 0.55 (as every launch preset). ~25 s compute
per simulated second on one core.

**What the soft body does** (run `arms1`, kit rear arms ×2): coasting at ≥ 15 m/s diverges in yaw (e-folding ~0.4 s;
15 m/s: 68° in 3.5 s into the kerb; 20 m/s: front arms break after 1.7 s) — the rear-heavy kit car oversteers (twin:
rear cornering stiffness per unit load 3.4 vs 23.9 front). ABS braking is straight (20 m/s → 0 in 2.4 s / ~25 m, 0.86 g,
heading within 4°): sliding tyre nodes grip isotropically, the same per unit load front and rear. Steer steps of 1.1-2.8°
at 15-20 m/s break a front arm within 0.9-1.6 s; braking + 2.8° of steer turns the car 15-17° while it stops.

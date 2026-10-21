# Work rounds (history)

Dated notes from the parallel work rounds of 2026-10-20 / 21: who owned what, what each round found, what was left
open. Kept for the *why* behind the current code; the ownership tables and process rules no longer apply, and numbers
may have been superseded — the code, `spec/` and `experiments/reports/` are the current truth.

## Ownership while the parallel work is in flight

| work stream | owns (may create/edit) |
|---|---|
| lead / integration | `src/app`, `src/core`, `src/cinema`, `src/playback`, `src/render/{stage,carRender,effects,latticeView}.js`, `src/softbody/{solver,recording,worker}.js`, `src/scene`, `src/car`, `src/programs/crash`, `spec/*.json` (except tasks), `index.html`, `style.css`, `package.json`, `tools/{lib,film,spec,audit}`, docs |
| flight-twin (JS + Python dynamics, tasks, controllers, bench) | `src/flight/**` (`propulsion.js`/`contract.js` are contracts: extend, don't break), `python/**`, `tests/parity/**`, `experiments/**`, `tools/flight/**`, `spec/tasks/**`, `spec/README.md` §5 §7 §8 |
| install (motion design) | `src/programs/install/**`, `public/assets/install/**` |
| flight-viewer (soft-body thrusters, brakes / steering, plumes, flight program) | `src/softbody/actuators.js`, `src/softbody/drive.js`, `src/render/plumes.js`, `src/programs/flight/**`, `tools/sim/flight*.mjs`, `public/assets/settled-coldgas.bin`, `public/sims/flight-*.sim` |

Rules: edit only what you own; need a change elsewhere → note it as a request for the lead (or wrap/extend in
your own area); no new npm dependencies; never commit or rewrite git state; dev servers per stream on their own
port (lead 5173, install 5174, flight-viewer 5175) and never stop someone else's process.

## MPC round (2026-10-20 evening): two experiments in parallel

The lead added the pure MPC framework `src/flight/mpc/` (registry name `mpc`, JS only; see its `index.js` header and
`experiments/README.md` "Pure MPC"). No pd anywhere in it: attitude, braking and manoeuvres come from the
optimised command schedule alone. Baseline (`experiments/configs/mpc-baseline.json`, 18 episodes): mpc-cem 100 %
success at 46 kg vs pd 72 % at 48 kg. Two experiment streams build on it; the scene, the soft body, the twins'
physics and the core MPC files are frozen for both (report needed changes instead of making them):

| work stream | owns (may create/edit) |
|---|---|
| lead | `src/flight/mpc/{index,param,planner}.js`, `src/flight/mpc/defaults.json`, `src/flight/mpc/optim/{index,noise,mppi,cem,grad}.js`, `src/flight/mpc/objectives/{index,land}.js`, `src/flight/controllers/**`, `tools/flight/{bench,run}.mjs`, docs, everything not listed below |
| barrel-roll (jump + 360° roll about the long axis + soft landing, twin and soft body) | `src/flight/mpc/objectives/roll.js`, `experiments/configs/roll-*.json`, `experiments/reports/barrel-roll/**`, `tools/flight/roll/**`, `public/sims/flight-mpc-roll*.sim`, `experiments/runs/roll-*`, `experiments/runs/soft-roll*`; one entry in `RUNS` of `src/programs/flight/index.js` |
| gas-study (is "free fall, then burn late" the gas-optimal soft landing? gradient / sampling / hybrid MPC) | `src/flight/mpc/objectives/gas.js`, `src/flight/mpc/optim/extra.js` (+ new files `src/flight/mpc/optim/x-*.js`), `experiments/configs/gas-*.json`, `experiments/reports/gas-late-burn/**`, `tools/flight/gas/**`, `experiments/runs/gas-*`, `experiments/runs/soft-gas*` |

Same rules as below (no commits, no new dependencies, no git state changes, never stop someone else's process);
twin experiments need no dev server; soft-body runs are plain Node (`tools/sim/flight.mjs`). Nobody renders films
or touches `src/**` outside their files while the other stream runs (the user's Vite server on 5173 reloads).

Outcome (reports in `experiments/reports/`): **barrel-roll** — `objectives/roll.js` + `roll-final.json` (hybrid
cem→cem, 4 s horizon): twin flat 10/10 (19/20 on seeds 1-20, 15/18 over all presets untuned), climb 2.3 m, roll
361°, ~90 kg; soft body `public/sims/flight-mpc-roll.sim` succeeded (1.24 m/s, 1.5°, no hull contact; on the
2026-10-21 nozzle layout it strikes the rear floor 0.24 s after touchdown, see the report's dated note), 4/6 over
soft-body variants. **gas-late-burn** — braking itself is bang-bang optimal (twin reference 27.3 kg, earlier
ignition costs more), but only for the lift: attitude must be fixed first (full coast > 0.3 s collapses success);
best MPC = lift-only coast 0.9 s, 39.9 kg 30/30; pd with saturated late braking 35.3 kg (MPC never finds it);
late burns have no thrust margin (−10 % thrust: 0-17 % success; planning at 85 % thrust fixes it for +4.5 kg);
sampling (cem 100 %, mppi 100 %) ≫ gradient (grad 22 %, L-BFGS 17-83 %), gradient polish never saved gas.

MPC backlog from that round (lead):
- promote a time-varying cold-start schedule (`optim/x-stages.js` `xinit`) into `param.initial` (both streams'
  main remaining failure is a bad cold first plan) and a plan/execution-mismatch hook (gas study wraps the
  controller in `tools/flight/gas/lib.mjs`; `tools/sim/flight.mjs` has no such hook)
- planning is not real time: cold first plan 6-7 s (11.5 k rollouts), then 140-190 ms per plan vs the 40 ms replan
  period (fine offline, the soft body waits)
- stop rollouts after the task's hold window (halves episode time); heave mode is solved at the full-tank COM
  (~0.66 kN·m of pitch at full heave after ~45 kg); `slip2` scores 180° slip as 0
- soft body: the pure MPC keeps 1-2 kN of bottom thrust after touchdown (the soft body sits 1-2° tilted, the twin
  level) and most twin-good touchdowns strike the hull 0.15-0.3 s later (rear bottoming, slip): the known gap — the
  touchdown calibration round (below) found the main cause was sideways landings (14-89° of slip), fixed there
- 2026-10-21 (lead): the bottom nozzle pairs moved to z 0.34 / −1.16 (centroid on the COM after ~45 kg): pitch-free
  lift 26.6 → 33.7-34.9 kN. Twin: `mpc-baseline` at least as good everywhere (mppi / hybrid 94 → 100 %),
  `mppi-soft-landing` 97 → 98 % at −4 kg, late burns with −10 % thrust 6 → 56-67 %. Soft body: every recording keeps
  its outcome except `flight-mpc-roll`, which now strikes the rear floor after touchdown (the gap above; opt-in
  `ground_cut` + v_ok 0.8 gets 3/4 soft-body cases at the price of twin 7/10, not adopted)

## Touchdown calibration round (2026-10-21)

Grey-box identification of the twin's touchdown model against designed soft-body landings, report
`experiments/reports/touchdown-calibration/README.md`. Hardware first (kit): bottle rack one slot forward, bottom
nozzles re-centred (0.41 / −1.09), stiffer rear coil-overs for the kit car only (`propulsion.suspension`,
`ThrusterRig.applyMass`, per-run `sim.comp`). Data: `tools/flight/touchdown/` (drop rig, 92-case design, opt-in contact
force probe `CrashSim.fprobe`). Model: twin v2 extensions (spec/README.md §5.11: per-axle suspension, bump-stop
damping, slip tyre, wheel spin, toe / compliance steer, underbody points, wheel loads, `vehicle` override), fitted
values in `spec/vehicle.json` = `coldgas-rigid@2` (v1 = `tests/parity/vehicle-v1.json`). Fit / validation /
acceptance: `tools/flight/calib/`. Failure criteria calibrated on the soft body live in the MPC land objective
(touchdown contact-patch lateral speed, ground wheel lateral load). Soft-body closed loop, 26 untuned cases:
touchdown-clean 8/26 → 17/26 (p = 0.035 paired).

Open after this round (all but the gas budget handled by the drive round below):
- (done) the stiff rear springs break the rear-left control arm when the `flat` preset leaves the deck: kit rear arms ×2
- (done) `public/sims/flight-*.sim` were recorded on the old hardware and the v1 twin: re-recorded
- (done, `tyre.slide`) the twin's tyre fits rolling but not the impact (soft-body tyres grip like Coulomb at impact)
- (done: brakes + steering) landing at ~20 m/s without brakes still ends at a kerb 1-2 s later
- the roll's gas budget is tight (3/8 out of gas)

## Drive round (2026-10-21): kit rear arms, brakes, steering

The user asked for the reinforced rear control arms, re-recorded flights and an install film on the new hardware, and
brakes + steering after the landing. Report: `experiments/reports/drive-brakes-steering/README.md`.
- Hardware: kit rear arms ×2 (`propulsion.suspension.rear_arm_give_scale`, `ThrusterRig.applyMass`); brakes with ABS on
  every wheel and front-wheel steering, the car's own hardware (`spec/drive.json`): two action channels after the
  nozzles (`NA = NU + 2`, contract `coldgas-contract@2`). Soft body `src/softbody/drive.js` (brake = torque couple
  wheel / caliper cluster, ABS from the hub speed; steering = rest lengths of the front arms / coil-overs for the
  kingpin-rotated axle), mounted by `ThrusterRig` (inert without commands: old runs bit-identical).
- Twins: `coldgas-rigid@3` (spec/README.md §5.13: brake pressure lag, implicit Coulomb brake with ABS bound in the
  wheel-spin update, rate-limited steer on the front tyre axes, kerb event; §5.11 `slip_ratio_sat`, `tyre.slide`),
  new task `escape-land-stop`; refit jointly on both landing designs + 16 soft-body drive tests (`tools/flight/drive`).
- The landing design now runs at the flight connector strength (0.55): front control arms are the weak point (they
  break in 46 of 92 landings, mostly at impact; failure criteria in `land.js` re-derived, `tools/flight/calib/criteria.mjs`).
- MPC: `drive: true` adds brake / steer channels (0 until the touchdown), land objective drive terms (kerb crash, kerb
  margin, distance, stop-before-the-kerb in the air and on the ground, stop deadline): `experiments/configs/drive-stop.json`.
- Viewer: brake / steer HUD panel, stop verdict; `ap` brakes fully on the ground; no render trees on the crossing road.
- Outcome: soft body, 26 untuned launches, "lands softly, stops, no kerb, nothing broken after the touchdown" 17/26
  without brakes → 24/26 with brakes + steering (stops in 2-3.6 s / 10-34 m); the old sets without brakes on the new
  arms + refit: touchdown-clean 17/26 → 22/26. Twin: stop task 17/18 (without drive 0/18).

Open after the drive round:
- front control arms: at the flight connector strength they are the weak point (46/92 landings, steer steps of 1-3° at
  15-20 m/s, the `nose` launch); front ×2 is the same one-line kit change (user's call), then the landing design, fit,
  criteria and acceptance again
- the soft kit car is yaw-unstable when rolling free above ~9 m/s; one soft-body case (roof s1) spun out while braking at
  26° of steer: a speed-dependent steer limit, or braking + large-steer cases in the drive tests
- `tyre.slide` makes the twin's impact wheel loads Coulomb-like (lateral load RMS 1.1-1.2 → 1.6 kN on held-out landings)
- the land objective's drive terms only know raised street boxes: render-only scenery (park trees beyond the crossing
  road's end) is open ground to the physics

## Two-cost round (2026-10-21 evening): numbers for the FALL script

The user asked for the two experiments of the film script (文案 04–05): the MPC with a cost that has no gas term
("holds the car and lowers it slowly") and with one ("suicide burn"), and the gas of each. Report
`experiments/reports/gas-two-costs/README.md` (trajectories for segments 04 / 05 in its `traj/`). Findings worth keeping:
- the receding-horizon MPC keeps the basin of its first plan: setting `w_gas` 0 alone does not make it hover-lower
  (it still coasts and burns late, 54 vs 46 kg), and from a hover cold start no gas weight makes it let go; the
  cost decides only when both starts are run and whole flights are scored by it (`tools/flight/gas/twocost.mjs`)
- the slow descent is genuinely "smoother" under `land` without gas (−8.6 points, CI 3.7–13.5) and costs 20–25 kg more;
  at the shipped `w_gas` 0.3 the two cancel (toss-up), at 1 / kg the late burn wins 34/40
- every strategy flies with a mean bottom thrust ≈ the weight, so gas ≈ 19 kg/s × time in the air
- follow-up (report §9): dropping the soft-body failure criteria, the gradient optimizers (`xlbfgs`, `grad`), ppo's
  LTV-MPC (convex QP: there the safe-descent envelope, not the linear fuel term, makes it brake late; `w_fuel` 0 costs
  only 3-5 % more) and a 10x larger cold plan from the hover start all fail to give the result from one start
- MPC additions (opt-in, default off): `starts` in `src/flight/mpc/index.js` (multi-start cold plan, ctl.start), `w_sink` /
  `sink_ok` in `objectives/land.js`

## Integration backlog (from the first parallel round, 2026-10-20)

Promote to shared modules (each requested by a work stream, currently living in its program folder):
- per-key timed camera spline (`src/programs/install/keyed.js`) and the car-attached roll shot
  (`src/programs/flight/edit.js` `attachedRoll`) → `src/cinema/shots.js`
- canvas HUD compositing after the post chain (install `hud.js`, flight `compositeHud`) → a `Stage` overlay hook,
  plus a supported way for programs to set/restore Stage settings (fog, environment, exposure, bloom, sun, clipping)
- `fitFrame` / `matToQuat` (`src/softbody/actuators.js`) → `src/core/quat.js` (recording.js has a private copy)
- sectioned / turned / braided materials, studio environment + reflector floor (install) → `src/render/`
- isentropic helpers (`machFromArea`, ...) and mulberry32 → `src/flight/` / `src/core/` (several private copies)

Open issues:
- twin `pd` lands softly on the soft body but rolls over 0.3 s later (no heading/slip control); the flight program's
  autopilot `ap` (`src/programs/flight/autopilot.js`) aligns heading and cancels drift — port it to both twins and
  add a heading/slip criterion to the landing tasks
- (done 2026-10-21, touchdown calibration round above) with the kit 70 % of the weight was on the rear axle and the
  twin's rear suspension sat on its bump stop at rest: per-axle springs / bump-stop damping are in coldgas-rigid@2
- the soft body's springs are unloaded, so the car sags 5-7 cm (bare) / 5-11 cm (kit, rear on the bump stop) below
  the GLB's ride height and the tyres sit up to ~8 cm inside the wheel arches (only 29-34 mm of clearance are
  modelled), visible in every close-up. The fix exists as an opt-in model update, `CrashSim` scenario `level`
  (`tools/sim/flight.mjs level=1`): coil-overs preloaded for the current static load (`levelSprings`), bump /
  rebound stops 30 / 20 mm along the spring (`SPRING` in `src/car/carPrep.js`), and free wheels keep their spin
  (`keepWheelSpin`: PBD projection bled half of it in a 2.5 s flight, so touchdown spun the tyres up by friction and
  the control arms ended at their 4.95 mm give). With it the car rests at design height and the `ap` soft landing
  still touches down at 0.96 m/s, but the higher, rear-heavy car then oversteers out of control ~1.5 s later
  (yaw rate 0.9 rad/s, ESP jets saturated, control arms snap). To adopt it: retune the `ap` ground mode, port
  preload + stops to the twins, make `level` the default, re-settle, re-run every recording.
- closed-loop MPC on the soft body: done. `mppi` now exists in JS (`src/flight/controllers/mppi.js`), bit-identical
  to Python (shared mulberry32 noise, sequential reductions; parity scenarios `*-mppi-*`), and replans every 60 ms
  from `ThrusterRig`'s state estimate (`public/sims/flight-mppi-{nose,roof}.sim`: 0.52 / 0.76 m/s, level, no hull
  strike all run; open-loop replay: 6.3 m/s, tumbling). The Python cost alone is not enough on the soft body: the
  run needs the opt-in soft-body settings of `experiments/configs/mppi-soft-landing.json` `mppi-soft` (slip and
  ground-yaw terms, 2 s kerb lookahead with a footprint margin, bottom residual faded out below 3 m so pd flies the
  touchdown, base pd `td_speed` 0.5). Open: since the 8.7 kN bottom nozzles all 10 soft-body cases land, but 4 roll
  into a kerb 1-3 s after the hold; the soft body's rear bottoms out where the twin's stiff bump stop does not (model
  gap behind most of these terms); MPPI only shifts its nominal by whole knots, so with `replan` < `knot` it never
  shifts (same in Python, kept for parity)
- PPO solves hover-hold (from a pd warm start) but not escape-soft-landing yet (brakes too late)
- (done 2026-10-21, drive round) the soft body has no brakes or rolling resistance: after landing the car rolls on at launch speed
- `renderFilmFrame` shutter samples straddle shot cuts (one blended frame per cut)

# Soft-body touchdown rig (twin touchdown-model identification)

Designed landings of the kit car on the soft body: the data set for the grey-box calibration of the rigid twin's
touchdown model. Each case releases the car ballistically (no thrust) so that the first tyre touches the street in a
prescribed state, then records the soft body through the touchdown and `post` (2) s beyond.

| file | what |
|---|---|
| `drop.mjs` | one experiment → `<outDir>/<id>.json.gz` (`touchdown-drop@1`) + a one-line summary |
| `doe.mjs` | design (FIT / HOLDOUT maximin LHS + ANCHORS), resumable parallel runner, `report=1` table |
| `lib.mjs` | factor table, conventions, rigid gyrostat propagation, spin-bleed model, design feasibility, spec hash |
| `experiments/configs/touchdown-doe.json` | ranges, seeds, anchors, drop options |

```
node tools/flight/touchdown/drop.mjs vs=1.25 u=15 beta=0 theta=0 phi=0 yaw_rate=0 pitch_rate=0 roll_rate=0 gas=0.4 spin=0.57 id=x outDir=/tmp/td
node tools/flight/touchdown/doe.mjs [workers=10]                  # new run dir experiments/runs/touchdown-doe/<YYYYMMDD-HHMMSS>
node tools/flight/touchdown/doe.mjs run=experiments/runs/touchdown-doe/<stamp>              # resume (finished cases are skipped)
node tools/flight/touchdown/doe.mjs run=experiments/runs/touchdown-doe/<stamp> report=1     # outcome table, per-factor comparison
```
`drop.mjs` options: `post=2 airtime=0.3 cmin=0.004 hz=200 denseHz=1000 denseWin=0.4 precontact=0.05 site=-40,22
pose=rest|settled joints=1 force=0 quiet=0`. `joints` is CrashSim's connector give factor: every launch preset flies with
0.55 (`spec/launch-presets.json`), and the DOE config sets it since 2026-10-21 (`20261021-083616`, kit rear arms ×2);
the first run `20261021-020006` used 1, i.e. control arms 1.8× stronger than in any flight (front arms broke in 13
of 92 cases there, 46 at 0.55; rear 23 → 10 with the reinforced rear arms). Compute: ~22 s per simulated second on one core (~50 s per 2.3 s case); with
10 workers 26–33 s per simulated second each, the 92-case design in 9–12 min wall (`20261021-020006`: 11.9 min).

## Factors and conventions (state at the first tyre contact)

Frames as spec/README §1 (body B = lattice rest frame, +X right, +Y up, +Z forward; `q` body→world `[x,y,z,w]`).
Heading ψ: forward `(sin ψ, 0, cos ψ)`; attitude `q = yaw(ψ)·rotX(−θ)·rotZ(−φ)`.

| factor | range | meaning |
|---|---|---|
| `vs` | 0.3–3 m/s | sink speed of the COM, `−v_y` |
| `u` | 0–22 m/s | forward speed along the horizontal heading `h`, `v·h` |
| `beta` | ±30° | slip, `atan2(v·r_h, v·h)`, `r_h = (cos ψ, 0, −sin ψ)`; + = moving toward the car's right side |
| `theta` | ±8° | pitch, `asin(fwd_W.y)`, nose up + |
| `phi` | ±8° | roll, `atan2(−right_W.y, up_W.y)`, right side down + |
| `yaw_rate` | ±0.6 rad/s | `w_B.y`, + = nose turning right |
| `pitch_rate` | ±0.4 rad/s | `−w_B.x` (= dθ/dt) |
| `roll_rate` | ±0.4 rad/s | `−w_B.z` (= dφ/dt) |
| `gas` | 0.25–1 | gas fill (fraction of 145.7 kg): kit masses as `tools/sim/flight.mjs gas=` |
| `spin` | 0–1.1 | wheel spin ratio `ω r / u` (1 = free rolling), `ω` about the axle (+X), + = rolling forward |

Nominal `vs 1.25, u 15, gas 0.4, spin 0.57`, everything else 0. `spin` 0.57 is the median of `ω r / u` just before
touchdown in 9 soft-body flight recordings (`public/sims/flight-*.sim`, `experiments/runs/soft-gas`: 0.40–0.86; the
wheels leave the deck rolling at 61–69 rad/s and arrive at 33–40 rad/s).

**Site.** COM at first contact `(x, z) = (−40, 22)`, horizontal velocity along +Z (ψ = −β): the cross street
x ∈ [−64, −16] has no kerbs or median; piers (z = 0) and the deck are > 6 m away. Every case logs the distance from
the AABB of all live nodes to every collider box (`boxDist`, `min_box_dist`). The twin replay uses the same world.

## How a case is set up

- **Pose: the lattice rest pose, rigidly placed** (not `settled-coldgas.bin`). In free flight every beam relaxes to
  its rest length and the coil-overs to their free length, so the rest pose is what the settled car becomes in the air
  (checked: a 1 s free fall of the settled pose ends within 0.3 mm of it on the body, springs within 1 mm). The
  settled pose itself would need ~1 s of air (coil-overs overdamped: τ ≈ 0.23 s front, 0.11 s rear; 17 mm of front
  sag left after 0.3 s), but a ballistic arc that ends at `vs` has been above the street for at most `2 vs/g`
  (0.06 s at 0.3 m/s). `pose=settled` places the settled pose instead.
- **Release.** The target state is flown back as a torque-free gyrostat (inertia of the node cloud with kit masses, plus
  the wheels' spin momentum) in 1 ms steps; the release is the latest time ≤ `airtime` with no node below the street
  since and ≥ `cmin` clearance (else the apex). Typical: 0.25 s at 1.25 m/s, 0.04 s at 0.3 m/s. Node velocities are
  rigid (`v + ω×r`) plus the wheel spin; the solver rebuilds velocities from positions, so nothing else to fix up.
  `sim.kick = null` (no kerb kick), the rig's edge/arming fire at t = 0 with no controller (zero thrust).
- **Wheels.** The solver bleeds free-wheel spin (PBD rim projection): `d(1/ω²)/dt ≈ 2.3e-4` measured here (1.84× the
  `(ω dt)²/2` per substep of the solver comment). The release spin is raised so the target spin remains at the
  contact, and the wheels are pre-rotated so that a tyre node of every ring sits at the WORLD bottom at the contact
  (the tyres are 24-gons: with the body pitched, the body-bottom node rises with the spin and was below the street just
  before; `setup.wheel_phase_td`).
- **Feasibility (design).** A factor set must be a genuine first tyre contact: the touching tyre node approaches at
  ≥ 0.1 m/s and no tyre node was below the street in the 60 ms before; and no tyre node may move faster than 38 m/s
  (below). This removes 8.3 % of the box; `drop.mjs` refuses infeasible targets (exit 2) unless `force=1`.
- **Achieved vs target.** The touchdown state is measured (rig estimate at the first-contact substep) and stored next
  to the target (`summary.achieved`).

## Output `touchdown-drop@1` (gzip JSON)

- `target`, `opts`, `site`, `wheel_ids` (FL FR RL RR = `vehicle-massprops.json` order), `spec_hash`, `inputs` (sha1 of
  the settled pose, propulsion spec, solver, actuators).
- `setup`: mass, `comB`, inertia (about the COM, body axes, row-major), wheel axle inertias, `h_wheels`, target state,
  target first node, hull clearance at the target, `t_release`, release clearance, clearance scan, spins.
- `release`, `precontact` (sample nearest 0.05 s before the first contact), `touchdown`: **twin-ready states**
  `{type:'state', t, t_before_contact, p, q, v, w, gas (kg), leftDeck, comB, mass, origin, comB_twin, p_twin, v_twin}`.
  `p`, `v` are the soft body's COM (rig estimator: mass-weighted attached nodes; `q` by shape matching the body chunk to
  its rest pose; `w` body axes from angular momentum, wheels as point masses). `origin = p − R comB` is the body
  origin; `p_twin`/`v_twin` put the twin's own COM (`MassModel.at(gas).com`, a few mm off) on the same body frame.
- `samples` (columns, `n` rows): 1 kHz from the release to 0.4 s after the first contact, 200 Hz after.
  `t p q v w comB M tilt` (rig estimate), `boxDist`, `vmax` (fastest vehicle node), `nclamp` (nodes at the 40 m/s
  clamp); `wheel.*[i][k]` per wheel: `disp` = wheel-chunk centroid minus its rest position in the body frame (m:
  x lateral, y vertical = suspension travel, z longitudinal), `comp` = mean spring compression `L0 − L` (m, + =
  compressed), `spin` (rad/s, relative to the body, + rolling forward), `hubY`, `clear` (lowest tyre node y), `F` =
  ground contact force (N, world; normal = `F[1]`, tangential = `(F[0], 0, F[2])`), `Flonglat` = `F` along the twin's
  rolling direction `(a_z, 0, −a_x)/h` and lateral axis `(a_x, 0, a_z)/h` (`a` = axle), `susp`/`spring` = broken
  control-arm / spring beams (of 4 / 2), `detached`; `hull.*`: `min` = lowest skin clearance `y + allow` of attached
  non-wheel nodes with its rest position `minRest` and piece, `floorMin`/`floorRest` the same over rest `y < 0.35`,
  `F` total hull contact force (N, world), `Fc` its normal-force centroid (rest/body coords), `n` contacting nodes.
  **Forces are means over the interval `(t[i−1], t[i]]`**, from the solver's opt-in probe (`CrashSim.fprobe`: each
  node's static-contact position correction × m / dt², zero-cost and bit-identical when null).
- `summary` (= one line of `summary.jsonl`), `events` (`sim.events.breaks`: detach / latch / glass / crack with `t`),
  `rig` (`ThrusterRig.summary`: its touchdown / hull-strike events, task metrics, damage).

### Outcomes (`summary`)

`outcome` = worst of the `flags` in this order: `no_touchdown`, `rollover` (tilt > 90°), `wheel_detach`,
`arm_break` (a SUSP beam broke), `spring_snap`, `mount_damage` (summed plastic strain > 20 or ≥ 5 torn STRUCT beams
within 0.3 m of a wheel's arm/spring mounts: the wheel then stays 3–10 cm off its place; every landing yields the front
spring mounts a little: nominal 2.4–2.8, clean landings ≤ 9.3, often one torn beam; raw values in `mount_damage[k]`), `hull_strike` (any attached non-wheel node touching the street, detected every substep;
`hull_strike.{t, after_td, piece, rest, region}`, region by rest position: front z > 1.25, rear z < −1.3, sill
|x| > 0.6, floor, upper y ≥ 0.35), else `clean`. Informational flags: `hull_first`, `speed_clamp`. Also:
`first_contact`, `wheel_td` (per-wheel first contact after the first), `springs_at_td_mm` (≈ 0 = relaxed),
`hull_impulse_Ns`, `hull_peak_N`, min hull / floor clearance, max spring compression, arm / spring break times,
`mount_damage[k]`, final and max-lateral wheel displacement, `damage`, `max_tilt_deg`, `final` (tilt, speed, yaw rate,
heading change, `support_ratio` = Σ contact F_y / (M g) over the last 0.3 s), `max_node_speed`, `min_box_dist`,
`rig` (its touchdown / hull-strike times), `compute_s`.

## Run directory (`experiments/runs/touchdown-doe/<stamp>/`)

`manifest.json` (config, factor table, nominal, design statistics, `cases: [{id, split: fit|holdout|anchor, factors,
feasibility}]`), `cases/<id>.json.gz`, `summary.jsonl` (`{split, ...summary}` in manifest order, rebuilt at the end of
every runner pass), `logs/<id>.log`, `runs.jsonl` (wall time per pass). IDs: `F00–F59`, `H00–H19`, `A00-nominal …`.

## Soft-body caveats seen while building this

- **Speed clamp.** `CrashSim.substep` caps every node at 40 m/s. A tyre's top node moves at about `|v_h| + ωr`, i.e. `2u`
  once rolling, so landings above ~20 m/s are clipped after the spin-up (flag `speed_clamp`: `vmax`, `nclamp`,
  `share_after_td`), and too much spin + speed in the air is destroyed within milliseconds (it jerked the body by
  0.2 rad/s): the design excludes tyre speeds > 38 m/s at the contact. Launches above 72 km/h already hit it on the deck.
- **Spin bleed** ≈ 1.84× the solver comment's estimate (above).
- **Yaw divergence after touchdown.** Even the symmetric nominal touchdown (no slip, level, no rates) yaws right
  after it, roughly exponentially (0.04 → 0.32 rad/s within 2 s, 14° heading change) with the body rolling into the
  turn: the rear-heavy kit car is unstable in straight-line rolling on the soft body.

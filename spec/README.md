# spec/ — 跨语言唯一事实源 (single source of truth)

Everything that the browser app, the soft-body solver, the **JS flight twin** (`src/flight/`) and the
**Python flight twin** (`python/coldgas/`) must agree on lives here as JSON. Code never copies these numbers.
JS reads them only through `src/core/spec.js`; Python only through `python/coldgas/spec.py`.

| file | content | edited by |
|---|---|---|
| `world.json` | viaduct/street dimensions, gravity, air | hand |
| `launch-presets.json` | the six launch presets + kerb-kick timing | hand |
| `propulsion.json` | 10 nozzles (order = action order), COPV bank, regulator, valves, control rate, kit masses | hand |
| `vehicle.json` | rigid-twin contact/aero/integrator parameters | hand |
| `tasks/*.json` | challenge tasks (initial conditions, success criteria, reward weights, metrics) | hand |
| `generated/vehicle-massprops.json` | bare-car mass/COM/inertia, wheels, settled COM — from the soft-body lattice | `npm run spec` |
| `generated/colliders.json` | static colliders of the set | `npm run spec` |

`npm run spec:check` fails if a generated file is stale (run it after touching `src/car/carPrep.js`, the GLB,
`settled.bin` or `world.json`). Keys starting with `_` are documentation only and carry no semantics.

## 1. Frames and units

SI units throughout (m, s, kg, N, Pa, K, rad). Angles in JSON are degrees only where the key says `_deg`.

- **World W**: +Y up, gravity `(0, -world.gravity, 0)`. The viaduct runs along X; deck road surface at
  `y = deck.top`; the car drives towards −X on the +Z carriageway and leaves through the parapet gap at
  `z = deck.halfWidth`. The street is the plane `y = 0` plus the boxes in `generated/colliders.json`.
- **Body B** (car body frame = soft-body lattice rest frame): +X right, +Y up, +Z forward, origin on the ground
  under the car centre (rest pose, wheels touching y = 0). All positions in `propulsion.json` / `vehicle.json`
  are in B.
- **Orientation** `q = [x, y, z, w]` (three.js order) rotates B → W: `v_W = R(q) v_B`.
- **Rigid state**: `p` = world position of the **current** centre of mass, `v` = its world velocity,
  `w` = angular velocity in **body** axes, `comB` = centre of mass in B (moves as gas is used).
  A body point `x_B` is at `x_W = p + R(q) (x_B − comB)`.
- Flat state vector (trajectories, batched code): `[p(3), q(4), v(3), w(3)]`, length 13.

## 2. Thrusters

`propulsion.nozzles` order **is** the action order everywhere (controllers, valve/thrust logs, UI, recordings):

| idx | id | group | where |
|---|---|---|---|
| 0–3 | B_FL B_FR B_RL B_RR | bottom | under the floor pan, front pair z = 0.55, rear pair z = −0.95 (heave / roll / pitch); not symmetric about the kit's rear-heavy COM (z ≈ −0.43 full): equal openings pitch the car, the largest pitch-free thrust is ≈ 26.6 kN (rear pair full, front ≈ 0.53), thrust/weight ≈ 2.36 |
| 4–7 | R_UL R_UR R_LL R_LR | rear | 2×2 through the rear bumper (surge; yaw by left/right, pitch by upper/lower) |
| 8–9 | S_L S_R | side | on the sills at the COM (sway) |

Gas leaves along `exhaust`; thrust acts along `−exhaust` at the `exit` point. `mount` is where the flange sits on
the skin (motion design, soft-body anchoring). Throat diameter and expansion ratio are primary; thrust is derived.

## 3. Propulsion model v1 (reference: `src/flight/propulsion.js`)

Per step of length `dt` with commanded openings `u_cmd[i]` (held over the step):

1. **Valves** (proportional, modelled as a variable effective throat area): `c = clamp(u_cmd, 0, 1)`, `c = 0` if
   `u_cmd < valve.deadband`; `τ = tau_open` if `c > s` else `tau_close`; `s ← s + (c − s)(1 − e^(−dt/τ))`.
2. **Feed pressure**: `p_c = min(regulator.p_set, p_tank − regulator.dp_min)`; if `p_c ≤ 2 p_a` then `p_c = 0`
   (cannot choke: tank exhausted). `T_c = T_tank` (isenthalpic regulator, ideal gas).
3. **Nozzle i** (choked, isentropic, frozen once per nozzle: exit Mach `M_e` from `expansion_ratio` by 80-step
   bisection on [1, 50]): `ṁ = s A_t p_c √(γ/(R T_c)) (2/(γ+1))^((γ+1)/(2(γ−1)))`,
   `v_e = M_e √(γ R T_c / (1 + (γ−1)/2 M_e²))`, `p_e = p_c (1 + (γ−1)/2 M_e²)^(−γ/(γ−1))`,
   `F = max(0, ṁ v_e + s (p_e − p_a) A_e)` with `A_e = ε A_t`, `p_a = world.air.pressure`.
4. **Tank** (all bottles = one volume `V = Σ volume`): `m ← max(0, m − Σṁ dt)`, `ρ = m/V`,
   `p = K ρ^n` with `K = p0 / ρ0^n`, `ρ0 = p0/(R T0)`, `n = tank.polytropic_n`; `T = p/(ρ R)`.

Flows and thrust use the tank state at the **start** of the step. Nominal values at full tank: ≈8.7 kN (bottom),
≈4.9 kN (rear), ≈3.0 kN (side), Isp ≈ 65 s, 145.7 kg of air; the regulator drops out after ~75 % of the gas.

## 4. Vehicle mass properties

- `m_dry = lattice.mass + Σ bottle.dry_mass + Σ nozzle.mass + Σ hardware.mass`; bottles and gas are solid
  cylinders (bottle geometry, axis `axis`), nozzles and hardware are point masses at `mount` / `center`.
- Gas mass `m_g(t)` is split over the bottles in proportion to volume.
- `comB`, total mass and inertia (about `comB`, body axes) are recomposed whenever `m_g` changes (parallel-axis
  theorem from `generated/vehicle-massprops.json` `lattice.inertia`, which is about `lattice.com`).

## 5. Rigid-body twin (JS `src/flight/`, Python `python/coldgas/`) — owned by the flight-twin work

Model id `coldgas-rigid@1` (written into trajectories as `model`). Reference code: `src/flight/twin.js`, mirrored
expression by expression by `python/coldgas/twin.py` (numpy, N vehicles at once). The two agree **bit for bit**
(§5.9); any change here needs both files, the goldens (`tests/parity/`) and a new model id.

**5.1 Constants.** `dt = vehicle.integrator.dt`, `n_sub = round(propulsion.control.dt / dt)`, `g = world.gravity`,
`k_a = 0.5 · air.density · aero.CdA` (per body axis). Wheels `j = 0..3`: centre `c_j` and radius `r_j` from
`generated/vehicle-massprops.json` (body frame, unloaded), suspension `k_s c_s travel k_b`, `tyre_mu`, `crr` from
`vehicle.wheels`. Hull points `H_j` and `k_h c_h mu_h` from `vehicle.hull`. Nozzle `i`: exit `e_i`, thrust direction
`d_i = −exhaust_i/|exhaust_i|`. Mass properties: §4 in the linear form of `massprops.js` (dry parts folded into
first/second moments about the body origin; `M = m_dry + m`, `c = (s_dry + m s_gas)/M`,
`I = J_dry + m J_gas − M(|c|²E − c cᵀ)`, `I⁻¹` by cofactors). Model constants (`spec/vehicle.json` `twin`, not
vehicle facts): `contact.v_reg` 0.1 m/s, `contact.wheel_min_h` 0.5, `contact.street_top_max` 1.0 m,
`placement.iterations` 30, `placement.fd_step` 1e-7, `limits.floor_y` −2 m, `limits.max_range` 400 m.

**5.2 World.** Ground plane `y = ground.y` (`ground.mu`) plus the boxes of `generated/colliders.json` with
`cψ = cos yaw`, `sψ = sin yaw`. A box is *deck* (name `deck`), *street* (top `c_y + e_y < street_top_max`:
sidewalks, median) or *other* (parapets, piers); the ground plane is street. Local coordinates of a world point:
`d = x − c`, `l = (cψ d_x − sψ d_z, d_y, sψ d_x + cψ d_z)`; *inside* means `|l_x| < e_x ∧ |l_y| < e_y ∧ |l_z| < e_z`.
*Street level* under `(x, z)`: the highest of `ground.y` and the tops of street boxes whose footprint
(`|l_x| < e_x ∧ |l_z| < e_z`) contains the point.

**5.3 State.** `p, q, v, ω_B` (§1), propulsion state (valves `s`, tank `m, T, p`), mass properties of the current
`m`, integer step counter `k` (`t = k·dt`, never accumulated), and flags: kick (waiting / active with `n_left` /
done), `left_deck`, `armed`, `airborne`, touchdown snapshot, first hull strike, peak specific force.

**5.4 Control step.** `u = armed ? u_cmd : 0`, NaN → 0 (clamping to [0, 1] and the dead band are the propulsion
model's). Run `n_sub` integrator steps with `u` held, then the boundary update §5.8.

**5.5 Integrator step** (in this order; quantities without a prime are those of the step start):

1. `R = R(q)` (row-major, the expressions of `src/core/quat.js`).
2. *Kick* (§6): if waiting and `p_z > deck.halfWidth − kick.trigger`: start with `n_left = round(kick.dur/dt)`.
   While `n_left > 0`: `ω_B += Rᵀ Δω_W`, `n_left −= 1`, with `Δω_W = ((sp r_x + sr f_x)/n, sy/n, (sp r_z + sr f_z)/n)`,
   `[sp, sy, sr] = spin`, `n = round(kick.dur/dt)` (so the full kick is added exactly once).
3. *Propulsion*: `T = Propulsion.step(u, dt)` (§3; uses the tank state at the step start, leaves `m` at `m(t+dt)`).
4. *Forces* at `(p, q, v, ω_B)` (ω after the kick), COM `c`: `ω_W = R ω_B`; a contact point with world offset `r`
   from the COM moves with `v_P = v + ω_W × r`.
   - **Wheels** (only if the axle `a = R x̂` has `h = √(a_x² + a_z²) > wheel_min_h`): massless wheel, tyre point =
     lowest point of the wheel disc, `r_j = R(c_j − c) + r_j δ` with `δ = (a_y a_x/h, −h, a_y a_z/h)`. Support depth
     `d = ground.y − y_P`, then for every box the point is inside of with top-face depth `d_b = e_y − l_y ≤ r_j` and
     `d_b > d`: `d = d_b` (file order; the chosen surface gives `μ_s` and the class). If `d > 0`:
     `N = k_s d − c_s v_P,y`, plus `k_b (d − travel)` if `d > travel`; if `N > 0`, with `ih = 1/h`:
     rolling speed `v_t = (v_x a_z − v_z a_x) ih`, lateral `v_l = (v_x a_x + v_z a_z) ih`,
     `f_t = −(crr N) v_t / max(|v_t|, v_reg)`, `f_l = −(tyre_mu μ_s N) v_l / max(|v_l|, v_reg)`,
     `F = ((f_t a_z + f_l a_x) ih, N, (f_l a_z − f_t a_x) ih)` at the tyre point.
   - **Hull points** `r_j = R(H_j − c)`. Ground: if `d = ground.y − y_P > 0`: `N = k_h d − c_h v_P,y`; if `N > 0`:
     `φ = mu_h ground.mu N / max(√(v_x² + v_z²), v_reg)`, `F += (−φ v_x, N, −φ v_z)`. Every box the point is inside of:
     penetrations `π_a = e_a − |l_a|`; face y if `π_y ≤ π_x ∧ π_y ≤ π_z`, else x if `π_x ≤ π_z`, else z; world normal
     `(0, σ_y, 0)`, `(cψ σ_x, 0, −sψ σ_x)` or `(sψ σ_z, 0, cψ σ_z)` with `σ = l ≥ 0 ? 1 : −1`; `d = π_face`,
     `v_n = v_P·n`, `N = k_h d − c_h v_n`; if `N > 0`: `v_t = v_P − v_n n`,
     `φ = mu_h μ_box N / max(|v_t|, v_reg)`, `F += N n − φ v_t`.
   - Contact sums `F_C = Σ F_j`, `τ_C = Σ r_j × F_j` (world, about the COM): wheels 0..3 then hull points in file
     order, sequential from 0. Thrust (body) `F_T = Σ T_i d_i`, `τ_T = Σ (e_i − c) × (T_i d_i)` in nozzle order.
     Drag (body, at the COM) `v_B = Rᵀ v`, `F_A,a = −k_a,a |v_B,a| v_B,a`.
     Totals `F = F_C + R(F_T + F_A)`, `τ_B = τ_T + Rᵀ τ_C`.
5. *Integrate*: `v_x += dt F_x/M`, `v_y += dt (F_y/M − g)`, `v_z += dt F_z/M`; with
   `f(ω) = I⁻¹(τ_B − ω × Iω)`: `ω_½ = ω + (dt/2) f(ω)`, `ω' = ω + dt f(ω_½)` (midpoint: the free tumble keeps
   energy and |L| to ~1e-9 over 6 s); `p' = p + dt v'`; `q' = normalize(q + (dt/2) q ⊗ (ω', 0))`.
6. *Gas used*: mass properties of `m(t+dt)`; `Δc = c' − c`; with `R' = R(q')`: `p' += R' Δc`, `v' += R'(ω' × Δc)`
   (the body frame stays put, the COM moves inside it; `ω` is continuous since the exhaust carries its own
   angular momentum and the jet force is already the thrust). Skipping this when `m` did not change is exact.
7. `k += 1`. Events and flags of the step carry `t = k·dt` of the step start.

**5.6 Contact classes and flags** (per integrator step, ORed over a control step): *any* (a wheel with `d > 0` or a
hull point inside any surface), *street* (wheel on a street support, hull point in the ground or a street box),
*deck*, *other* (parapet/pier), *hull strike* (hull point in the ground or any non-deck box; the first one is
recorded with the point index). **Touchdown** = first integrator step with street contact while `airborne`; its
snapshot is `(t, p, q, v, ω)` of that step's start. Peak specific force `max |F|/M` (excludes gravity).

**5.7 Placement.** *launch*: §6 with `cos θ`, `sin θ` from `fmath`, COM horizontal at `(x_c, z_c)`, heading
`q_h` = yaw with `cos φ = sin θ`, `sin φ = −cos θ` (half-angle without trig: if `cos > −0.5`:
`w = √((1+cos)/2)`, `y = sin/(2w)`, else `y = ±√((1−cos)/2)` with the sign of `sin`, `w = sin/(2y)`), `v = s f`,
`ω = 0`, full tank, kick waiting, `left_deck = false`. *rest*: COM above the task's `(x, z)`, yaw `ψ` = heading
about +Y (forward `(sin ψ, 0, cos ψ)`), at rest, `left_deck = true`. Both then solve the **static suspension
equilibrium** on the support plane `y_s` (highest box top or ground under the COM): unknowns `(y_c, a, b)`,
`q = normalize((a, 0, b, 1) ⊗ q_h)`, `p = (x_c, y_c, z_c)`; residual `(Σ N_j − M g, −Σ r_z N_j, Σ r_x N_j)` with
the wheel geometry of 5.5 and `N_j = k_s d + k_b (d − travel)⁺` for `d > 0` (no damping/friction); Newton from
`(y_s + c_y − M g/(4 k_s), 0, 0)` with forward-difference Jacobian (`fd_step`), solved by cofactors, exactly
`iterations` times (residual ~1e-11 N). *state*: explicit `p, q, v, ω`, optional `gas` (tank on the polytrope:
`p = K ρⁿ`, `T = p/(ρR)`), `valve`, `leftDeck` (default true), `kick.spin_w` (world, rad/s).

**5.8 Control-step boundary** (after reset and after every control step, in this order): `left_deck` (launch:
first boundary with `p_z > deck.halfWidth`, event `edge`); `airborne` (first boundary with `k > 0`, `left_deck`
and no contact at all during the step just run); `armed` (task `arm`: `'start'` at once, `'edge'` once
`left_deck`; never disarmed); `phase` = `'deck'` before `left_deck`; at `k = 0` `'ground'` for rest and `'air'` for
an explicit state; else `'air'` without contact in the last step, `'deck'` if it touched the deck, otherwise `'ground'`.
Events: `kick`, `edge`, `armed`, `airborne`, `touchdown` (with wheel bit mask), `hull_strike` (point index),
`regulator_dropout` (first step with `p_c < p_set`).

**5.9 Numerics and parity.** Float64, no fused multiply-add, sums strictly sequential in the stated order, and every
expression in the operand order of `twin.js` (numpy evaluates the same ufunc sequence lane by lane; masked lanes add
exact zeros; the broadphase only skips boxes that cannot contain a contact point, which never changes a sum).
Only `+ − × ÷ √` and `pow` reach the state: V8, CPython and numpy agree on `√` and `pow` (2 M samples checked); the
valve-lag `exp` constants agree; `sin/cos` (placement) and `acos/tanh` (rewards, policies) go through
`src/flight/fmath.js` / `coldgas/fmath.py`, the fdlibm algorithms written out identically in both languages (V8's own
trig and Apple's libm disagree in the last bit for 3–18 % of inputs). Parity tolerance (relative to the largest
value of the component over the trajectory): **1e-9 on contact-free rows, 1e-6 on rows with contact** — penalty
contacts amplify a 1-ulp difference (e.g. a platform whose `pow` rounds differently) by orders of magnitude within a
second, so contact segments cannot promise more than that; measured on all 18 scenarios: **0 (bit-identical)**.
`round()` on exact halves differs (JS up, Python to even): no rounded quantity here sits on a half.

**5.10 Limitations** (by design, the soft body is the high-fidelity model): convex box edges only meet hull
points (the deck edge can pass between them); wheels only stand on box tops (a kerb hit from the side lifts the wheel
onto it); massless, non-spinning wheels (no wheel gyro, no brakes or drive); aero without moments or wind; no jet
damping; the bump stop is undamped beyond `c_s` (hard landings bounce). With the kit the static rear-axle load
compresses the rear springs past `travel` (static compression 0.111 m rear / 0.055 m front: the rear sits 1.4 mm
on the bump stop, pitch 1.6° nose up), so a firm landing goes straight into the bump stops.

## 6. Launch placement and kerb kick (shared with the soft body, `CrashSim.setup`)

For a preset (`speed` km/h, `angle` deg, `spin` = [pitch, yaw, roll] rad/s, `lead` s, default
`launch-presets.defaults.lead`):

- `θ = angle·π/180`, `s = speed/3.6`, forward `f = (−cos θ, 0, sin θ)`, right `r = (sin θ, 0, cos θ)`, up `(0,1,0)`;
  the body axes (+X, +Y, +Z) map to (r, up, f); initial velocity `s·f`, no initial rotation rate.
- The vehicle's **full** centre of mass starts at horizontal `x_c = (gap[0]+gap[1])/2 + 1.5 − s f_x lead`,
  `z_c = halfWidth − s f_z lead − 0.9`, with the wheels resting on the deck (`y = deck.top`) at static suspension
  equilibrium. (The soft body gets its height from its settled pose; the twins from their own suspension —
  only the JS and Python twins must agree exactly.)
- **Kick**: once the COM passes `z > deck.halfWidth − kick.trigger`, for `kick.dur` seconds the world angular
  velocity grows at the constant rate `(spin[0]·r + spin[2]·f + spin[1]·up) / kick.dur` (r, f fixed at launch).

## 7. Interchange formats

- **Trajectory** `coldgas-traj@1` (JSON): `{ format, spec_hash, model, producer: 'js'|'python', task, controller,
  seed, preset, dt, n, t[n], state[n][13], comB[n][3], tank[n][3] (m, T, p), u_cmd[n][10], valve[n][10],
  thrust[n][10], events: [{t, type, ...}], metrics: {...} }`, sampled at the control rate (`propulsion.control.dt`).
  Viewers place the car with §1; the JS app can open trajectories from `experiments/runs/` in the dev server.
  Row `k` is the state at `t_k = k·dt` (the last row is the final state); `u_cmd[k]` is the command applied over
  `[t_k, t_k+1)` after arming (zeros in the last row); `valve[k]`/`thrust[k]` are those of the last integrator step
  before `t_k` (zeros at `k = 0`). Optional, written by both twins: `init` (the reset() initial condition incl. the
  RNG draws), `contact[n]` (bit mask of contacts during the step that ended at `t_k`: 1 any, 2 street, 4 deck,
  8 parapet/pier, 16 hull strike), `scenario` (parity goldens). Writers: `src/flight/traj.js`, `coldgas/traj.py`.
- **Soft-body thrust log** `coldgas-log@1`: `meta.propulsion` of a flight recording (`public/sims/flight-*.sim`),
  a superset of the trajectory columns (plus feed pressure, phase, armed, mount clusters, kit masses), sampled at
  `hz` from t = 0; `meta.flight` holds controller, task, events, landing metrics and damage. Authoritative
  description: header of `src/softbody/actuators.js`.
- **Spec hash**: FNV-1a 64-bit over every `*.json` under `spec/` (recursive), sorted by relative path with `/`
  separators; for each file feed `path` UTF-8 bytes, a 0x00 byte, the raw file bytes, a 0x00 byte. Lower-case hex,
  16 digits. Trajectories, policies and benchmark results record it; consumers warn on mismatch.
- **RNG** for task randomisation: mulberry32 (as in `src/softbody/solver.js`) seeded with `seed >>> 0`; draws in the
  order the task lists them (preset index first, then the `randomize` keys in file order), `uniform(a, b) =
  a + (b − a)·r()`. Seed 0 is mapped to 1 (`seed >>> 0 || 1`), so episode seeds 0 and 1 draw the same initial
  condition. Standard normals (MPPI noise): `gauss() = Σ₁₂ r() − 6` (Irwin–Hall; the sum of twelve multiples of
  2⁻³² is exact, so JS `rng.js` and numpy `Mulberry32.gauss_array` agree bit for bit).
- **FlightState** (what controllers see; twin or soft-body estimate): `{t, p[3], q[4], v[3], w[3] (body),
  comB[3], tank: {m, T, p}, valve[10], armed, phase: 'deck'|'air'|'ground'}` (§1 frames; `t` from the episode start).
  The twin adds `pc`, `thrust`, `step`; controllers must not need them.
- **Controller** (same names, settings and numbers in both languages; JS `src/flight/controllers/index.js`
  `createController(nameOrConfig)` / async `loadController`, Python `coldgas.controllers.create_controller`):
  `reset({task, seed})` once per episode (task = resolved task object, may be null), then `act(state, info)` →
  `u_cmd ∈ [0,1]^10` every `CONTROL_DT`, called also while unarmed (the twin forces 0 then). A config is a name or
  `{name, ...settings}`; defaults live in `src/flight/controllers/defaults.json` (read by both). Registry:
  `zero`; `pd` (attitude + braking-profile autopilot on the bottom nozzles, uses only `+ − × ÷ √`, bit-identical in
  both languages); `replay` (`{traj}` or `{file}`: plays `u_cmd[round(t/dt)]`, zeros after the end); `mlp`
  (`{policy}` or `{file}`: runs a `coldgas-policy@1`); `mppi` (sampling MPC, residual on `pd`, rollouts on a twin
  loaded from the FlightState (`RigidTwin.loadState` / `Twin.load_state`) with integrator step `plan_dt`; noise
  from `gauss()` seeded `seed·1000003 + episode seed + 1`, reductions summed sequentially, softmax through the
  portable `exp`, so the two languages are bit-identical; settings and cost in `defaults.json` `mppi`). Python
  only: PPO-trained policies exported as `coldgas-policy@1`. JS only: `mpc` (pure MPC without a base pilot,
  `src/flight/mpc/`, settings in `src/flight/mpc/defaults.json`; deterministic, not parity-tested).
- **Policy** `coldgas-policy@1` (JSON): `{format, spec_hash, model, task, obs: {format: 'coldgas-obs@1', names[],
  mean[], std[], clip}, layers: [{W[out][in], b[out], act: 'tanh'|'relu'|'linear'}], action: {transform:
  'clip01'|'tanh01'}, meta: {...}}`. Evaluation: `x = clip((o − mean)/std, −clip, clip)` with `o` the task's
  observation (§8); per layer `y_j = act((Σ_i W[j][i] x_i, summed in order from 0) + b_j)`; `u = clip(y, 0, 1)`
  (`clip01`) or `(tanh(y)+1)/2` (`tanh01`); 0 while unarmed. `tanh(x) = sign(x)(1 − 2/(exp(2|x|)+1))` (saturated
  beyond |x| = 20) with the portable `exp` of `fmath`, so JS and Python evaluate a policy bit-identically.

## 8. Tasks, observations, rewards — owned by the flight-twin work

`tasks/*.json` are declarative; JS `src/flight/tasks.js` + `obs.js` and Python `coldgas/tasks.py` + `obs.py`
implement the same semantics (parity-tested on full episodes, rewards and metrics included). JS imports each task
file explicitly in `src/core/spec.js`; Python scans `spec/tasks/`.

**8.1 Resolution.** `extends: <id>` merges recursively: objects key by key (the parent's key order first, new keys
appended), arrays and scalars replaced by the child, except `metrics`, which is the parent's list followed by the
child's new entries. Keys starting with `_` are documentation.

**8.2 Initial state** for `(task, seed)`: `r = mulberry32(seed)`. *launch*: `k = floor(r()·len(presets))` (always
drawn; an explicit preset override ignores it), then each `randomize` key in order with `x = uniform(a, b)`:
`speed_kmh` adds km/h, `angle_deg` adds degrees, `spin_scale` multiplies `spin`; `lead` is the preset's or
`defaults.lead`. *rest*: `position` (x, z of the COM; y is found by the equilibrium), `heading_deg`, randomize
`yaw_deg` (added). Placement then follows §5.7.

**8.3 Arming** `arm`: `'edge'` (thrusters enabled at the first control-step boundary after the COM passed
`z > deck.halfWidth`), `'start'` (from t = 0).

**8.4 Per control step** (after the step; `Δm` = gas used in it; tilt = angle between body up and world up,
`acos(1 − 2(q_x² + q_z²))` with `fmath.acos`; height `h` = body-origin height above the street level under the COM,
`p_y − (R c)_y − street(p_x, p_z)`):
`gas_per_kg · Δm`; `tilt_per_rad_after_edge · tilt` once `left_deck`; hover tasks (`target`): until the hold is
complete, `height_error_per_m · |h − target.height|`, and the hold timer `+CONTROL_DT` while `|h − height| ≤
tolerance`, reset to 0 otherwise; the hold is complete once the timer reaches `target.hold` (−1e-9).
`recover_by_height`: the vehicle is *recovered* at the first step (after the edge) with COM height above the street
`≥ recover_by_height`, tilt `≤ recover_tilt_deg` and `|ω| ≤ recover_angular_rate`.

**8.5 Termination** (checked in this order after every control step): `hull_strike` if `success.hull_strike` is
false and a hull strike happened (§5.6) → terminated; `out_of_bounds` (COM below `limits.floor_y` or farther than
`limits.max_range` horizontally from its start) → terminated; `landed_early` (hover task touched down before the
hold was complete) → terminated; `landed` once `t ≥ t_touchdown + success.upright_hold` (0 if absent) →
terminated; `timeout` at `t ≥ horizon` → truncated.

**8.6 Success** (only for `landed`), every key optional, evaluated on the touchdown snapshot (§5.6):
`|v_y| ≤ max_vertical_speed`; tilt `≤ max_tilt_deg` at touchdown and at every control step of the hold window;
`|ω| ≤ max_angular_rate`; pad distance (COM `(x, z)` at touchdown to `pad.center`) `≤ max_pad_distance`;
`recovered` if `recover_tilt_deg` is given; hover tasks need the hold complete. No hull strike is implied by
reaching `landed`.

**8.7 Terminal reward** (added on the final step): `terminal_success` if successful; `terminal_hull_strike` for
`hull_strike`; `terminal_timeout` for the other failures (`timeout`, `out_of_bounds`, `landed_early`) so that
neither hovering out the clock nor hopping back down early pays; if a touchdown happened:
`terminal_vertical_speed_per_mps · |v_y,td| + terminal_tilt_per_rad · tilt_td + terminal_pad_distance_per_m ·
pad_distance`.

**8.8 Metrics** (the task's `metrics` list selects; always added: `return`, `reason`, `t_end`, `touchdown_t`):
`success`, `touchdown_vertical_speed` |v_y|, `touchdown_horizontal_speed`, `touchdown_tilt_deg`,
`touchdown_angular_rate` |ω|, `touchdown_wheels` (count), `peak_accel_g` (max specific force / g over the episode,
deck run included), `hull_strike`, `gas_used_kg`, `landing_point` [x, z], `hold_time` (longest continuous hold),
`rms_height_error` (over the steps until the hold completed), `pad_distance`, `recovered`, `recover_time`,
`recover_height`; `null` when undefined.

**8.9 Observation** `coldgas-obs@1` (default for policies; `obs.js` / `obs.py`), from FlightState + task only:
`[up_B (3) = Rᵀ ŷ, v_B (3) = Rᵀ v, ω_B (3), v_y, h, m/m0, p_c/p_set (p_c from tank.p by the regulator rule, 0 if
exhausted), armed (0/1), valve (10)]` (24), then task extras in this order: `target` → `[h − height,
min(hold_timer/hold, 1)]` (the observer keeps its own hold timer, updated per observation with the rule of 8.4),
`pad` → `[pad.x − p_x, pad.z − p_z]`, `recover_by_height` → `[COM height above street − recover_by_height]`.
Raw SI values; scaling is the policy's `obs.mean/std`.

**8.10 Environments.** JS `src/flight/env.js` and Python `coldgas.env.ColdGasEnv(task, seed=…, preset=…)`:
`reset(seed?, options{preset, init}) → (obs, info)`, `step(u) → (obs, reward, terminated, truncated, info)`;
`info = {t, phase, armed, preset, seed}` plus `reason`, `metrics` on the last step. Python
`coldgas.vecenv.BatchColdGasEnv(task, n, seed)` runs n lanes on one numpy twin, lane `i`'s `j`-th episode with seed
`seed + i + j·n`, same-step auto-reset (final obs/metrics in `info['final'][i]`); every lane equals the scalar env
with the same seed bit for bit.

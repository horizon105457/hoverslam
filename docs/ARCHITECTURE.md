# 架构 · Architecture

坠 · FALL started as an offline soft-body crash film (MINI EV off a viaduct). It is growing into a small lab:
the same car gets a cold-gas thruster kit (10 nozzles, after the Tesla Roadster "Rocket"), controllers (MPC, PPO, ...)
learn to fly it off the viaduct and land, and every outcome can be rendered as film. The code is split so these
lines of work can proceed in parallel against fixed contracts.

## Layers

```
spec/*.json  ── language-neutral facts (world, presets, thrusters, vehicle, tasks) ───────────────┐
   │                                                                                             │
   ├─ src/core/spec.js ── JS view of the spec                       python/coldgas/spec.py ── Py view
   │                                                                                             │
   ├─ src/flight/         rigid 6-DOF cold-gas twin (fast, JS)  ⇆ parity ⇆  python/coldgas/  (fast, Py)
   │     propulsion.js     reference propulsion model                      env / MPC / PPO / bench
   │     contract.js       state / controller / trajectory contract        experiments/ (configs, runs)
   │
   ├─ src/softbody/       XPBD node-beam crash solver (slow, high fidelity, JS only)
   │     solver.js         + `sim.actuators[]` hook for external forces (thrusters)
   │     actuators.js      thrusters on the lattice, closed loop through a Controller
   │     recording.js      compact recordings (public/sims/*.sim)
   │
   ├─ src/render/  stage (renderer + post), carRender (GPU-deformed car), effects, plumes
   ├─ src/scene/   layout (colliders from spec) + procedural viaduct set
   ├─ src/playback/view.js   a recording at sim time t: car, shards, dust, sparks, probes
   ├─ src/cinema/  Sequence (film time → sim time + camera), shot library, key moments, offline film frame
   ├─ src/programs/<id>/     self-contained experiences: crash · flight · install · ...
   └─ src/app/     shell: context + lazy assets, program registry (?program=<id>), transport bar
```

Two physics models on purpose: the **rigid twin** is what controllers are designed and trained on (thousands of
episodes per second, identical in JS and Python); the **soft body** is what gets filmed (plastic crush, glass,
parts coming off), driven by the same `Controller` interface through the actuator hook. Exact agreement is
required only between the two twins; the soft body is the "real world" the controllers are transferred to.

## Contracts (change only deliberately, with a version bump)

| contract | where | consumers |
|---|---|---|
| frames, units, nozzle order, propulsion model, launch/kick, traj format, spec hash, RNG | `spec/README.md` | everything |
| `FlightState`, `Controller`, state layout, `NU`, `CONTROL_DT` | `src/flight/contract.js` (+ Python mirror) | twins, soft-body actuators, viewers |
| `Propulsion` class (`step(u, dt)`, `valve`, `thrust`, `mdot`, `tank`) | `src/flight/propulsion.js` | twins, soft-body actuators |
| `sim.actuators[]` — `apply(sim, dt)` once per substep before integration | `src/softbody/solver.js` | `src/softbody/actuators.js` |
| `Program` — `init()`, `frame(dt)`, `film = {total, render(T, opts), describe()}` | `src/app/program.js` | shell, `tools/film/*.mjs` |
| `Sequence` / shot `cam(u, t) → CamPose` | `src/cinema/sequence.js`, `shots.js` | programs |
| `window.app` — `ready`, `error`, `stage`, `program`, `frame(dt)`, `film` | `src/app/main.js` | Node tools |

## Programs

`?program=<id>` picks `src/programs/<id>/index.js` (auto-discovered). A program builds its own scene objects on
the shared Stage, mounts its UI into `#hud`, and exposes a deterministic `film` so the same exporter renders any
of them: `npm run film -- program=<id> out=renders/x.mp4`.

- `crash` — the original viewer: six presets, live runs in a worker, director edit, HUD, audio (default).
- `flight` — thrust-assisted escape and landing: soft-body recordings with thrust logs, plumes, thruster HUD,
  and quick rigid-twin replays of trajectories from `experiments/`.
- `install` — motion-design piece: the cold-gas kit is installed on the car (hero nozzle with an internal
  fly-through, then the other nine).

## Creative shots

Shots are pure functions `cam(u, t)` (film progress, sim time) so realtime and offline renders agree. Reusable
moves live in `src/cinema/shots.js` (`lockOff`, `dolly`, `orbit`, `track`, `fly` along a spline, `blend`,
`withShake`), easing in `ease.js`, key moments of a launch in `moments.js`. Poses may set `roll` (dutch angle),
`focus` / `aperture` (depth of field in film renders) and `shake`. Programs keep bespoke shots in their folder;
general ones get promoted to the library.

## Experiments (tasks × algorithms)

Tasks are declarative (`spec/tasks/*.json`), controllers implement one interface in both languages, and the
benchmark runner writes per-episode metrics plus trajectories under `experiments/runs/` (git-ignored). A run is
reproducible from (spec hash, task, controller config, seed). See `experiments/README.md`.

## Ownership while the parallel work is in flight

| work stream | owns (may create/edit) |
|---|---|
| lead / integration | `src/app`, `src/core`, `src/cinema`, `src/playback`, `src/render/{stage,carRender,effects,latticeView}.js`, `src/softbody/{solver,recording,worker}.js`, `src/scene`, `src/car`, `src/programs/crash`, `spec/*.json` (except tasks), `index.html`, `style.css`, `package.json`, `tools/{lib,film,spec,audit}`, docs |
| flight-twin (JS + Python dynamics, tasks, controllers, bench) | `src/flight/**` (`propulsion.js`/`contract.js` are contracts: extend, don't break), `python/**`, `tests/parity/**`, `experiments/**`, `tools/flight/**`, `spec/tasks/**`, `spec/README.md` §5 §7 §8 |
| install (motion design) | `src/programs/install/**`, `public/assets/install/**` |
| flight-viewer (soft-body thrusters, plumes, flight program) | `src/softbody/actuators.js`, `src/render/plumes.js`, `src/programs/flight/**`, `tools/sim/flight*.mjs`, `public/assets/settled-coldgas.bin`, `public/sims/flight-*.sim` |

Rules: edit only what you own; need a change elsewhere → note it as a request for the lead (or wrap/extend in
your own area); no new npm dependencies; never commit or rewrite git state; dev servers per stream on their own
port (lead 5173, install 5174, flight-viewer 5175) and never stop someone else's process.

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
- with the kit 70 % of the weight is on the rear axle: the twin's rear suspension sits on its bump stop at rest;
  needs per-axle spring rates and bump-stop damping in `spec/vehicle.json` (model change → regenerate goldens)
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
- the soft body has no brakes or rolling resistance: after landing the car rolls on at launch speed
- `renderFilmFrame` shutter samples straddle shot cuts (one blended frame per cut)

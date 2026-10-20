# 坠 · FALL — notes for agents

Read `docs/ARCHITECTURE.md` (layers, contracts, ownership) and `spec/README.md` (frames, units, models, formats)
before changing anything.

- Numbers shared by JS and Python live in `spec/*.json`. JS reads them only via `src/core/spec.js`, Python only via
  `python/coldgas/spec.py`. Never hard-code a spec value. After changing the lattice (`src/car/carPrep.js`), the GLB,
  `settled.bin` or `spec/world.json`, run `npm run spec`.
- Nozzle order in `spec/propulsion.json` is the action-vector order everywhere.
- The JS and Python flight twins must agree to rounding: any change to `src/flight/**` physics needs the matching
  change in `python/coldgas/**`, `spec/README.md` and the parity goldens (`tests/parity/`).
- Soft body: changing the lattice (incl. triangle order) needs `npm run settle` and `npm run flight:settle` (the car with
  the thruster kit, `settled-coldgas.bin`), then `npm run spec`, `npm run precompute:all` and `npm run flight:sim:all`,
  otherwise the car falls apart at the start. Render-only fixes go in `src/render/carRender.js`, not the physics input.
- Programs live in `src/programs/<id>/index.js` (`?program=<id>`); `program.film.render(T)` must be a pure function of
  film time T. Film export: `npm run film -- program=<id> out=renders/x.mp4` (needs a dev server; `port=` if not 5173).
- Checks: `npm test` (generated spec up to date, JS/Python parity both ways, pytest).
- Edits stay in the area you own (table in `docs/ARCHITECTURE.md`). No new npm dependencies. Do not commit.
- Style: plain ES modules, no TypeScript, dense code with comments that explain why; match the surrounding file.

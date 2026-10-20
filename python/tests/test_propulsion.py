import numpy as np

from coldgas.contract import GROUPS, NU
from coldgas.propulsion import Propulsion, BatchPropulsion
from coldgas.rng import Mulberry32
from coldgas.spec import WORLD


def test_nominal_numbers():
    """spec/README.md section 3: ~7.0 / 4.9 / 3.0 kN per bottom / rear / side nozzle at 6 MPa feed, Isp ~65 s, 124.9 kg."""
    P = Propulsion()
    assert abs(P.m0 - 124.9) < 0.05
    P.valve = [1.0] * NU  # fully open, full tank: one tiny step reads the nominal point
    F = np.array(P.step(np.ones(NU), 1e-9))
    md = np.array(P.mdot)
    assert P.pc == P.spec['regulator']['p_set']
    assert abs(F[GROUPS['bottom']].mean() - 7000) < 100 and abs(F[GROUPS['rear']].mean() - 4900) < 100
    assert abs(F[GROUPS['side']].mean() - 3000) < 100
    assert abs(F.sum() / (md.sum() * WORLD['gravity']) - 65) < 2


def test_scalar_mirror_matches_js(node):
    js = node("""
import { Propulsion } from './src/flight/propulsion.js';
import { mulberry32 } from './src/flight/rng.js';
const P = new Propulsion(), r = mulberry32(3), out = [];
for (let k = 0; k < 3000; k++) {
  const u = Array.from({ length: 10 }, () => (r() < 0.6 ? r() * 1.2 : 0));
  const dt = k % 7 === 0 ? 0.002 : 0.001;
  P.step(u, dt);
  if (k % 10 === 0) out.push([u, dt, Array.from(P.thrust), Array.from(P.valve), P.m, P.T, P.p, P.pc]);
  else out.push([u, dt]);
}
console.log(JSON.stringify(out));""")
    P = Propulsion()
    B = BatchPropulsion(3)
    for row in js:
        u, dt = row[0], row[1]
        P.step(u, dt)
        B.step(np.array([u] * 3), dt)
        if len(row) > 2:
            assert P.thrust == row[2] and P.valve == row[3]
            assert (P.m, P.T, P.p, P.pc) == tuple(row[4:8])
        assert (B.thrust[1] == np.array(P.thrust)).all() and B.m[2] == P.m and B.T[0] == P.T and B.p[1] == P.p


def test_regulator_dropout_and_exhaustion():
    P = Propulsion()
    u = np.ones(NU)
    pcs = []
    for _ in range(6000):
        P.step(u, 0.001)
        pcs.append(P.pc)
    pcs = np.array(pcs)
    p_set = P.spec['regulator']['p_set']
    assert (pcs[:100] == p_set).all() and pcs[-1] == 0.0 and P.m > 0 and P.thrust == [0.0] * NU
    assert np.all(np.diff(pcs[pcs < p_set]) <= 0)

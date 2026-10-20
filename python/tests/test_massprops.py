from coldgas.massprops import MassModel
from coldgas.propulsion import Propulsion
from coldgas.spec import PROPULSION, GENERATED


def test_matches_js(node):
    js = node("import { MassModel } from './src/flight/massprops.js'; const m = new MassModel(), out = [];"
              "for (const g of [0, 1e-3, 17.5, 62.25, 145.74198896637836]) { const r = m.at(g); out.push([r.M, Array.from(r.com), Array.from(r.I), Array.from(r.Iinv)]); }"
              "console.log(JSON.stringify(out));")
    m = MassModel()
    for g, (M, c, I, Ii) in zip([0, 1e-3, 17.5, 62.25, 145.74198896637836], js):
        r = m.at(g)
        assert r[0] == M and list(r[1]) == c and list(r[2]) == I and list(r[3]) == Ii


def test_composition():
    m = MassModel()
    kit = sum(b['dry_mass'] for b in PROPULSION['tank']['bottles']) + sum(n['mass'] for n in PROPULSION['nozzles']) + sum(h['mass'] for h in PROPULSION['hardware'])
    assert abs(m.m_dry - (GENERATED['massprops']['lattice']['mass'] + kit)) < 1e-9
    m0 = Propulsion().m0
    M, c, I, Ii = m.at(m0)
    assert abs(M - (m.m_dry + m0)) < 1e-9
    # the bottles sit behind the cabin: gas moves the COM rearwards and up
    _, c0, _, _ = m.at(0.0)
    assert c[2] < c0[2] and c[1] > c0[1]
    # inverse really is the inverse
    import numpy as np
    def full(s):
        return np.array([[s[0], s[3], s[4]], [s[3], s[1], s[5]], [s[4], s[5], s[2]]])
    assert np.allclose(full(I) @ full(Ii), np.eye(3), atol=1e-12)

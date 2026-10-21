"""Touchdown model extensions of the twin (spec/README.md 5.11, 5.12), tested independently of the spec's fitted
values: every mechanism / comparison test starts from the explicit v1 override tests/parity/vehicle-v1.json (V1),
which reproduces the coldgas-rigid@1 trajectories bit for bit (tests/parity/v1-reference.json). The v2 paths agree
with JS bit for bit through loadState too (trajectories: tests/parity v2-* scenarios), lanes stay independent, the
spin / overload / hull-group / toe rules hold, and the default twin is exactly what spec/vehicle.json says."""
import copy
import hashlib
import json
import math

import numpy as np
import pytest

from coldgas import parity
from coldgas.env import ColdGasEnv
from coldgas.controllers import create_controller
from coldgas.model import contact_params
from coldgas.spec import ROOT, SPEC_DIR, VEHICLE
from coldgas.state import batch_of
from coldgas.tasks import load_task
from coldgas.twin import Twin
from coldgas.world import GROUND

V1_FILE, V1_REF = ROOT / 'tests' / 'parity' / 'vehicle-v1.json', ROOT / 'tests' / 'parity' / 'v1-reference.json'


def _strip(d):
    """Drop documentation keys (leading '_', spec convention) from a vehicle dict."""
    return {k: _strip(v) if isinstance(v, dict) else v for k, v in d.items() if not k.startswith('_')}


def over(a, b):
    """Plain deep merge (dicts key by key, everything else replaced): builds test vehicles on top of V1."""
    out = copy.deepcopy(a)
    for k, v in b.items():
        out[k] = over(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else copy.deepcopy(v)
    return out


V1 = _strip(json.loads(V1_FILE.read_text(encoding='utf-8')))
V2 = over(V1, {  # every v2 feature at once on top of V1 (as the v2-sideslip parity scenario)
    'wheels': {'spring_k': [30000, 52000], 'damper_c': [2600, 3400], 'travel': [0.11, 0.12], 'bump_k': [400000, 300000],
               'bump_c': [4000, 6000], 'bump_k2': [1000000, 2000000], 'tyre_mu': [1.0, 0.95], 'crr': [0.015, 0.02]},
    'tyre': {'model': 'slip', 'c_alpha': [12, 10], 'v_slip_min': [1.0, 0.8], 'slide': 0},
    'wheel_spin': {'enabled': True, 'inertia': [0.8, 1.1], 'v_long_sat': 0.5, 'slip_ratio_sat': 0, 'spin_ratio_air': 0.5, 'decay': 0.05},
    'failure': {'wheel_normal_max': 40000, 'wheel_lat_max': 6000, 'wheel_lat_impulse_max': 900},
    'hull': {'floor_points': [[-0.5, 0.12, -1.3], [0.5, 0.12, -1.3], [0, 0.1, 0.4]], 'floor_k': 250000, 'floor_c': 7000, 'floor_mu': 0.4},
})
TOE = [0.0041888, 0.00069813, 0.0002618, 0.0034907]  # settled-lattice static toe FL FR RL RR (rad, + = front edge right)
V2T = over(V2, {'wheels': {'toe': TOE, 'toe_compliance': [2e-5, 5e-5]}})  # ... plus toe and compliance steer
# V1 written differently: per-axle lists of equal values, zero v2 knobs spelled out, dormant parameters at junk values
V1_ALT = over(V1, {
    'wheels': {'spring_k': [30000, 30000], 'damper_c': [2600, 2600], 'travel': [0.11, 0.11], 'bump_k': [400000, 400000],
               'bump_c': [0, 0], 'bump_k2': [0, 0], 'tyre_mu': [1.0, 1.0], 'crr': [0.015, 0.015], 'toe_compliance': [0, 0]},
    'tyre': {'model': 'coulomb', 'c_alpha': [3, 30], 'v_slip_min': 2.5},
    'wheel_spin': {'enabled': False, 'inertia': [5, 0.1], 'v_long_sat': 0.2, 'spin_ratio_air': 0.9, 'decay': 3},
    'hull': {'floor_k': 1.0, 'floor_c': 2.0, 'floor_mu': 0.1},
})
SLIP = {'type': 'state', 'p': [-20, 1.0, 10], 'q': [0, 0.3826834324, 0, 0.9238795325], 'v': [-3, -2, 5], 'w': [0, 0.2, 0]}
REAR = {'type': 'state', 'p': [-20, 1.2, 10], 'q': [-0.0644448, 0.3817512, 0.0266947, 0.9216291], 'v': [5.5, -5, 5.5], 'w': [0, 0, 0]}
LAUNCH = {'type': 'launch', 'speed': 60, 'angle': 4, 'spin': [0.4, 0.2, -0.3], 'lead': 0.4}
REST = {'type': 'rest', 'x': -30, 'z': 12, 'heading_deg': 183}
INITS = {'slip': SLIP, 'rear': REAR, 'launch': LAUNCH, 'rest': REST}
U = np.full((1, 10), 0.0)


def _run(tw, steps, u=U):
    out = []
    for _ in range(steps):
        tw.step(u)
        out.append(tw.S[:, 0].copy())
    return np.array(out)


# ------------------------------------------------------------------------------------------- v1 = the override
def _fingerprint(traj):
    S = np.asarray(traj['state'], dtype='<f8')
    return {'n': traj['n'], 'state_sha256': hashlib.sha256(S.tobytes()).hexdigest(), 'final': traj['state'][-1],
            'events': traj['events'], 'metrics': traj['metrics']}


REF = json.loads(V1_REF.read_text(encoding='utf-8'))


@pytest.mark.parametrize('ref', REF['scenarios'], ids=[r['scenario']['id'] for r in REF['scenarios']])
def test_v1_override_reproduces_rigid_v1(ref):
    """V1 is the pre-calibration model: its trajectories equal the coldgas-rigid@1 goldens bit for bit."""
    got = _fingerprint(parity.run_scenario(dict(ref['scenario'], vehicle=V1)))
    assert got['n'] == ref['n'] and got['final'] == ref['final'] and got['events'] == ref['events']
    assert got['state_sha256'] == ref['state_sha256'] and got['metrics'] == ref['metrics']


def write_v1_reference():
    """Regenerate v1-reference.json from the V1 override (only after an intended change of propulsion / world / tasks)."""
    out = dict(REF, scenarios=[dict(_fingerprint(parity.run_scenario(dict(r['scenario'], vehicle=V1))), scenario=r['scenario'])
                               for r in REF['scenarios']])
    V1_REF.write_text(json.dumps(out, indent=1), encoding='utf-8')


@pytest.mark.parametrize('init', list(INITS.values()), ids=list(INITS))
def test_v1_written_differently_is_identical(init):
    """Per-axle lists of equal values equal scalars; zero bump / compliance terms and the parameters of switched-off
    features (slip tyre, wheel spin, floor groups) are dormant: not one bit changes."""
    a, b = Twin(1, vehicle=V1), Twin(1, vehicle=V1_ALT)
    a.reset([init]), b.reset([init])
    assert (_run(a, 80) == _run(b, 80)).all() and a.events == b.events
    assert (a.peak_N == b.peak_N).all() and (a.lat_impulse == b.lat_impulse).all() and not a.wo.any()


def test_v1_override_matches_js(node):
    """The v1 code path (Coulomb tyres, no spin, no toe) is bit-identical in JS and Python (the parity scenarios now
    run on the fitted spec, so this is its cross-language check), including the wheel diagnostics."""
    js = node(f"""
import {{ RigidTwin }} from './src/flight/twin.js';
const out = {{}};
for (const [vn, v] of [['V1', {json.dumps(V1)}], ['V1_ALT', {json.dumps(V1_ALT)}]]) for (const [name, init] of Object.entries({json.dumps(INITS)})) {{
  const tw = new RigidTwin({{ vehicle: v }}).reset(init), rows = [];
  for (let k = 0; k < 80; k++) {{ tw.step(new Float64Array(10)); rows.push([...tw.flat(), ...tw.wheelLoad.N, ...tw.wheelLoad.lat, ...tw.wheelPeak.latImpulse]); }}
  out[vn + '/' + name] = {{ rows, events: tw.events }};
}}
console.log(JSON.stringify(out));""")
    for vn, V in (('V1', V1), ('V1_ALT', V1_ALT)):
        for name, init in INITS.items():
            tw, rows = Twin(1, vehicle=V), []
            tw.reset([init])
            for _ in range(80):
                tw.step(U)
                rows.append([*tw.S[:, 0].tolist(), *tw.wheel_N[0].tolist(), *tw.wheel_lat[0].tolist(), *tw.lat_impulse[0].tolist()])
            assert js[f'{vn}/{name}']['rows'] == rows and js[f'{vn}/{name}']['events'] == tw.events[0], (vn, name)


# --------------------------------------------------------------------------------------- the default = the spec
def _spec_file():
    return json.loads((SPEC_DIR / 'vehicle.json').read_text(encoding='utf-8'))


def test_default_is_the_spec():
    """With no override every contact parameter is spec/vehicle.json's, resolved per axle / per wheel."""
    F = _spec_file()
    P = contact_params()
    assert P['V'] == F
    W, axle = F['wheels'], P['axle']
    pa = lambda x: (x, x) if not isinstance(x, list) else tuple(x)
    for key in ('spring_k', 'damper_c', 'travel', 'bump_k', 'bump_c', 'bump_k2', 'tyre_mu', 'crr', 'toe_compliance'):
        assert P['wheel'][key] == tuple(float(pa(W[key])[a]) for a in axle), key
    assert P['wheel']['slip_ratio_sat'] == tuple(float(pa(F['wheel_spin']['slip_ratio_sat'])[a]) for a in axle)
    assert P['wheel']['slide'] == tuple(float(pa(F['tyre']['slide'])[a]) for a in axle)
    assert P['wheel']['toe'] == tuple(map(float, W['toe']))
    assert P['wheel']['c_alpha'] == tuple(float(pa(F['tyre']['c_alpha'])[a]) for a in axle)
    assert P['wheel']['inertia'] == tuple(float(pa(F['wheel_spin']['inertia'])[a]) for a in axle)
    assert P['slip'] == (F['tyre']['model'] == 'slip') and P['spin_on'] == F['wheel_spin']['enabled']
    assert P['fail_on'] == any(F['failure'][k] is not None for k in ('wheel_normal_max', 'wheel_lat_max', 'wheel_lat_impulse_max'))
    assert P['hull'].tolist() == [list(map(float, p)) for p in F['hull']['points'] + F['hull']['floor_points']]
    assert Twin(1).spin_on == F['wheel_spin']['enabled'] and ('wheel_spin' in Twin(1).state(0)) == F['wheel_spin']['enabled']


def test_default_equals_the_spec_as_override():
    """Overriding with spec/vehicle.json's own values (minus documentation and integrator) changes nothing."""
    own = {k: v for k, v in _strip(_spec_file()).items() if k != 'integrator'}
    own['twin'] = {'contact': {k: own['twin']['contact'][k] for k in ('v_reg', 'wheel_min_h')}, 'placement': own['twin']['placement']}
    for init in (SLIP, LAUNCH, REST):
        a, b = Twin(1), Twin(1, vehicle=own)
        a.reset([init]), b.reset([init])
        assert (_run(a, 60) == _run(b, 60)).all() and a.events == b.events


def test_default_params_match_js(node):
    """JS resolves the spec defaults to the same per-wheel numbers and flags as Python."""
    js = node("""
import { contactParams } from './src/flight/model.js';
const P = contactParams();
console.log(JSON.stringify({ wheels: P.wheels.map(w => [w.ks, w.cs, w.tr, w.kb, w.bc, w.kb2, w.mu, w.crr, w.ca, w.vsm, w.I, w.vls, w.kss, w.sld, w.toe, w.tc, w.ts, w.kap, w.Tb]),
  flags: [P.slip, P.spinOn, P.bumpX, P.toeOn, P.complyOn, P.failOn], hull: P.hull.map(x => Array.from(x)), kh: Array.from(P.kh) }));""")
    P = contact_params()
    w = P['wheel']
    keys = ('spring_k', 'damper_c', 'travel', 'bump_k', 'bump_c', 'bump_k2', 'tyre_mu', 'crr', 'c_alpha', 'v_slip_min', 'inertia',
            'v_long_sat', 'slip_ratio_sat', 'slide', 'toe', 'toe_cos', 'toe_sin', 'toe_compliance', 'brake_torque')
    assert js['wheels'] == [[w[k][j] for k in keys] for j in range(4)]
    assert js['flags'] == [P['slip'], P['spin_on'], P['bump_x'], P['toe_on'], P['comply_on'], P['fail_on']]
    assert js['hull'] == P['hull'].tolist() and js['kh'] == P['kh'].tolist()


def test_override_never_mutates_the_spec(node):
    v = copy.deepcopy(V2T)
    Twin(1, vehicle=v)
    contact_params(v)
    assert v == V2T and VEHICLE == _spec_file() and contact_params()['V'] == _spec_file()
    js = node(f"""
import {{ RigidTwin }} from './src/flight/twin.js';
import {{ VEHICLE }} from './src/core/spec.js';
import fs from 'fs';
const v = {json.dumps(V2T)}, copy = JSON.stringify(v);
new RigidTwin({{ vehicle: v }}).reset({json.dumps(LAUNCH)}).step(new Float64Array(10));
console.log(JSON.stringify([JSON.stringify(v) === copy, JSON.stringify(VEHICLE) === JSON.stringify(JSON.parse(fs.readFileSync('spec/vehicle.json', 'utf8')))]));""")
    assert js == [True, True]


# ------------------------------------------------------------------------------------ v2 mechanisms (on V1)
def _flight_state(tw, e=0, spin=True):
    s = tw.state(e)
    if not spin:
        s.pop('wheel_spin', None)
    return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in s.items()}


@pytest.mark.parametrize('veh', ['V2', 'V2T'])
@pytest.mark.parametrize('case', ['air', 'ground', 'explicit'])
def test_load_state_v2_matches_js(node, case, veh):
    """Planning from a FlightState under v2: spin by phase rule (air: spin_ratio_air x matched, ground: matched) or
    from the state's wheel_spin; then 40 control steps, bit-identical in both languages incl. wheel diagnostics."""
    V = {'V2': V2, 'V2T': V2T}[veh]
    src = Twin(1, vehicle=V)
    src.reset([SLIP if case != 'ground' else REST])
    for _ in range(3 if case != 'ground' else 5):
        src.step(U)
    s = _flight_state(src, spin=case == 'explicit')
    assert s['phase'] == ('ground' if case == 'ground' else 'air')
    u = [0.3, 0.0, 0.5, 0.0, 0.2, 0.0, 0.0, 0.4, 0.1, 0.0]
    py = Twin(1, vehicle=V)
    py.load_state(batch_of(s))
    rows = []
    for _ in range(40):
        py.step(np.array([u]))
        P = py.wheel_peak(0)
        rows.append([*py.S[:, 0].tolist(), *py.wheel_spin[0].tolist(), *py.wheel_N[0].tolist(), *py.wheel_lat[0].tolist(),
                     *py.wheel_long[0].tolist(), *P['N'].tolist(), *P['lat'].tolist(), *P['lat_impulse'].tolist()])
    js = node(f"""
import {{ RigidTwin }} from './src/flight/twin.js';
const tw = new RigidTwin({{ vehicle: {json.dumps(V)} }}).loadState({json.dumps(s)}), rows = [], u = {json.dumps(u)};
for (let k = 0; k < 40; k++) {{
  tw.step(u);
  const L = tw.wheelLoad, P = tw.wheelPeak;
  rows.push([...tw.flat(), ...tw.wheelSpin, ...L.N, ...L.lat, ...L.long, ...P.N, ...P.lat, ...P.latImpulse]);
}}
console.log(JSON.stringify({{ rows, events: tw.events, overload: tw.wheelOverload }}));""")
    assert js['rows'] == rows
    assert js['events'] == py.events[0] and js['overload'] == py.wheel_overload(0)


@pytest.mark.parametrize('V', [V2, V2T], ids=['V2', 'V2T'])
def test_batched_lanes_equal_scalar_v2(V):
    inits = [SLIP, dict(REAR, wheel_spin=[10, 10, 5, 5]), LAUNCH, REST]
    tb = Twin(4, vehicle=V)
    tb.reset(inits)
    for _ in range(60):
        tb.step(np.zeros((4, 10)))
    for i, init in enumerate(inits):
        ts = Twin(1, vehicle=V)
        ts.reset([init])
        _run(ts, 60)
        assert (ts.S[:, 0] == tb.S[:, i]).all() and (ts.wheel_spin[0] == tb.wheel_spin[i]).all()
        assert (ts.peak_N[0] == tb.peak_N[i]).all() and (ts.lat_impulse[0] == tb.lat_impulse[i]).all()
        assert ts.events[0] == tb.events[i]


def _matched(tw, e=0):
    """Forward speed of each tyre point / radius (the rolling-matched spin without toe), recomputed independently."""
    from coldgas.twin import quat_to_mat
    S = tw.S[:, e]
    R = np.array(quat_to_mat(*S[3:7])).reshape(3, 3)
    om = R @ S[10:13]
    a = R[:, 0]
    h = math.hypot(a[0], a[2])
    out = []
    for (c, r) in tw.wheels:
        rr = R @ (np.array(c) - tw.C[:, e]) + r * np.array([a[1] * a[0] / h, -h, a[1] * a[2] / h])
        vp = S[7:10] + np.cross(om, rr)
        fwd = np.array([-a[2], 0.0, a[0]]) / h  # body +Z projected on the ground
        out.append(vp @ fwd / r)
    return np.array(out)


def test_spin_rules():
    tw = Twin(1, vehicle=V2)
    tw.reset([LAUNCH])  # rolling with the deck
    assert np.allclose(tw.wheel_spin[0], _matched(tw), rtol=1e-12) and tw.wheel_spin[0].min() > 50
    tw.reset([SLIP])  # explicit state: spin_ratio_air x matched
    assert np.allclose(tw.wheel_spin[0], 0.5 * _matched(tw), rtol=1e-12)
    tw.reset([dict(SLIP, wheel_spin=[1, 2, 3, 4])])
    assert tw.wheel_spin[0].tolist() == [1, 2, 3, 4]
    tw.reset([REST])
    assert (tw.wheel_spin[0] == 0).all()
    # airborne wheels keep their spin up to the decay; contact spins them up to rolling
    tw.reset([dict(REAR, wheel_spin=[0, 0, 0, 0])])
    tw.step(U)
    assert (tw.wheel_spin[0] == 0).all()
    _run(tw, 60)
    assert np.allclose(tw.wheel_spin[0], _matched(tw), rtol=0.05)
    # spin off: no spin state, no wheel_spin in the FlightState
    t1 = Twin(1, vehicle=V1)
    t1.reset([LAUNCH])
    assert (t1.wheel_spin == 0).all() and 'wheel_spin' not in t1.state(0)


def _episode(task, vehicle, preset='roof', seed=6):
    env = ColdGasEnv(task, seed=seed, preset=preset, vehicle=vehicle)
    ctrl = create_controller('pd')
    env.reset(seed)
    ctrl.reset(task=env.task, seed=seed)
    info, done = env._info(), False
    while not done:
        _, r, te, tr, info = env.step(ctrl.act(env.state, info))
        done = te or tr
    return env, info, r


def test_overload_terminates_landing_tasks():
    base = over(V1, {'wheels': V2['wheels'], 'tyre': V2['tyre']})
    env, info, _ = _episode('escape-soft-landing', base)  # thresholds null: diagnostics only, the landing succeeds
    assert info['reason'] == 'landed' and not env.twin.wo[0] and env.twin.peak_lat[0].max() > 12000
    fail = over(base, {'failure': {'wheel_normal_max': None, 'wheel_lat_max': 12000, 'wheel_lat_impulse_max': None}})
    env, info, r = _episode('escape-soft-landing', fail)
    ov = env.twin.wheel_overload(0)
    assert info['reason'] == 'wheel_overload' and ov['quantity'] == 'lat' and env.twin.peak_lat[0, ov['wheel']] > 12000
    assert any(e['type'] == 'wheel_overload' for e in env.twin.events[0]) and info['metrics']['success'] is False
    assert r <= env.task['reward']['terminal_hull_strike'] + 1  # priced like a hull strike
    task = load_task('escape-soft-landing')
    task['success']['wheel_overload'] = True  # a task may opt out
    env, info, _ = _episode(task, fail)
    assert info['reason'] == 'landed' and env.twin.wo[0]


def test_hull_point_groups():
    tw = Twin(1, vehicle=over(V1, {'hull': {'points': [], 'floor_points': []}}))  # no hull points at all still runs
    tw.reset([REAR])
    _run(tw, 50)
    assert not tw.hs[0] and np.isfinite(tw.S).all()
    tw = Twin(1, vehicle=over(V1, {'hull': {'points': [], 'floor_points': [[0, 0.25, -1.4]], 'floor_k': 2e5}}))
    assert tw.kh.tolist() == [2e5] and tw.ch.tolist() == [9000.0] and tw.hull_mu.tolist() == [0.55]
    tw.reset([dict(REAR, p=[-20, 0.9, 10], v=[0, -6, 0])])
    _run(tw, 40)
    assert tw.hs[0] and tw.hs_point[0] == 0
    tw = Twin(1, vehicle=over(V1, {'hull': {'floor_points': [[0, 0.25, -1.4]]}}))  # floor points follow `points`
    assert len(tw.kh) == len(V1['hull']['points']) + 1


BAD = [{'tyre': {'model': 'magic'}}, {'wheels': {'spring_k': [1, 2, 3]}}, {'failure': {'wheel_lat_max': -1}},
       {'wheel_spin': {'enabled': True, 'inertia': 0}}, {'wheels': {'spring_K': 1}}, {'integrator': {'dt': 0.002}},
       {'wheels': {'toe': [0.001, 0.001]}}, {'wheels': {'toe': 0.001}}, {'wheels': {'toe_compliance': [1e-6, 1e-6, 0]}},
       {'wheel_spin': {'slip_ratio_sat': -0.1}}, {'wheel_spin': {'slip_ratio_sat': [0.1, 0.1, 0.1]}},
       {'tyre': {'slide': 1.5}}, {'tyre': {'slide': [-0.1, 0.5]}}]


@pytest.mark.parametrize('bad', BAD)
def test_override_validation(bad):
    with pytest.raises(ValueError):
        Twin(1, vehicle=bad)


def test_override_validation_js(node):
    js = node(f"""
import {{ RigidTwin }} from './src/flight/twin.js';
console.log(JSON.stringify({json.dumps(BAD)}.map(v => {{ try {{ new RigidTwin({{ vehicle: v }}); return 'accepted'; }} catch (e) {{ return 'rejected'; }} }})));""")
    assert js == ['rejected'] * len(BAD)


STRAIGHT = {'type': 'state', 'p': [-20, 0.9, 10], 'q': [0, -0.7071067812, 0, 0.7071067812], 'v': [-8, -1.5, 0], 'w': [0, 0, 0]}


def _yaw_rate(tw):
    x, y, z, w = tw.S[3:7, 0]
    R1 = (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w))  # world y row of R
    return float(np.dot(R1, tw.S[10:13, 0]))


@pytest.mark.parametrize('model', ['coulomb', 'slip'])
@pytest.mark.parametrize('spin', [False, True])
def test_toe_sign(model, spin):
    """+ toe = front edge to the car's right: toeing the front axle right yaws the car right (+ world yaw rate),
    the rear axle left; zero toe on a straight touchdown gives (almost) no yaw: the car is not exactly symmetric
    (COM x = -0.6 mm, products of inertia). Both tyre models, spin on/off."""
    rates = {}
    for name, toe in (('none', [0, 0, 0, 0]), ('front', [0.004, 0.004, 0, 0]), ('rear', [0, 0, 0.004, 0.004])):
        # slip-tyre constants pinned (the coldgas-rigid@2 fit's), so the test does not move with the spec's refits
        ty = {'model': model, 'c_alpha': [12.749, 4.7614], 'v_slip_min': 1.8248, 'slide': 0}
        tw = Twin(1, vehicle=over(V1, {'wheels': {'toe': toe}, 'tyre': ty, 'wheel_spin': {'enabled': spin, 'slip_ratio_sat': 0}}))
        tw.reset([STRAIGHT])
        _run(tw, 50)
        rates[name] = _yaw_rate(tw)
    assert abs(rates['none']) < 1e-4 and rates['front'] > 1e-3 and rates['rear'] < -1e-3


def test_compliance_steer_reduces_lateral_force():
    """Positive toe_compliance steers a wheel so as to reduce its lateral force: in a sideslip the axle with
    compliance carries less lateral force; on the rear that is oversteer (more yaw), on the front understeer."""
    slide = {'type': 'state', 'p': [-20, 0.55, 10], 'q': [0, -0.7071067812, 0, 0.7071067812], 'v': [-8, 0, 0.8], 'w': [0, 0, 0]}
    out = {}
    for name, k in (('stiff', 0), ('front', [3e-5, 0]), ('rear', [0, 3e-5])):
        tw = Twin(1, vehicle=over(V1, {'wheels': {'toe_compliance': k}, 'tyre': {'model': 'slip'}}))
        tw.reset([slide])
        _run(tw, 5)
        out[name] = (np.abs(tw.wheel_lat[0]), abs(_yaw_rate(tw)))
    assert out['front'][0][:2].sum() < out['stiff'][0][:2].sum() and out['rear'][0][2:].sum() < out['stiff'][0][2:].sum()
    assert out['rear'][1] > out['stiff'][1] > out['front'][1]


def test_toe_diagnostics_in_tyre_axes():
    """Wheel loads are reported in the toed tyre axes (lateral = across the wheel's own heading). One integrator step
    of a car rolling straight ahead (-X) with FL toed right by 0.01: only FL is side-loaded, towards its right, by the
    Coulomb law evaluated on the rotated slip v_l' = sin(toe) v_t."""
    tw = Twin(1, vehicle=over(V1, {'wheels': {'toe': [0.01, 0, 0, 0]}}))
    tw.reset([dict(STRAIGHT, p=[-20, 0.55, 10], v=[-8, 0, 0])])
    tw.substep(np.zeros((1, 10)))
    lat, N, s = tw.wheel_lat[0], tw.wheel_N[0], math.sin(0.01)
    vl = s * -8.0  # rotated lateral slip of FL (v_t = -8: rolling forward)
    mu = V1['wheels']['tyre_mu'] * GROUND['mu']  # tyre x ground
    assert lat[0] == pytest.approx(-(mu * N[0]) * vl / max(abs(vl), V1['twin']['contact']['v_reg']), rel=1e-6) and lat[0] > 0
    assert np.abs(lat[1:]).max() < 1e-6 * lat[0]

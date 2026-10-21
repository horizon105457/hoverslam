"""Brakes, steering and the kerb rule of the twins (spec/README.md 5.13, 8.5): zero drive is the old physics, the brake
never reverses a wheel and holds the ABS slip bound, steer is clamped and rate-limited, drive is 0 while unarmed,
load_state restores it (bit-identical to JS), lanes equal the scalar twin, and the stop / kerb task rules. Mechanism tests
run on the v1 override plus the v2 extensions they need (test_twin_v2.V1 / V2), so they do not depend on fitted values;
the trajectories themselves are cross-checked against JS by the drive-* / land-stop-pd parity scenarios."""
import json
import math

import numpy as np
import pytest

from coldgas.contract import NU, NA, ND, DRIVE_IDX
from coldgas.controllers import create_controller
from coldgas.episode import run_episode
from coldgas.model import drive_params
from coldgas.spec import DRIVE
from coldgas.state import batch_of
from coldgas.tasks import load_task
from coldgas.twin import Twin
from test_twin_v2 import V1, V2, V2T, _flight_state, over

DP = drive_params()


def rolling(speed, heading=-90.0, x=60.0, z=13.0, vehicle=None, air=0.0):
    """Explicit state of a car rolling on its wheels (rest-placement equilibrium, speed along the heading, rolling-matched
    spin; `air` lifts it that far, wheels still spinning)."""
    tw = Twin(1, vehicle=vehicle)
    tw.reset([{'type': 'rest', 'x': x, 'z': z, 'heading_deg': heading}])
    ps = heading * math.pi / 180
    tw.S[7, 0], tw.S[9, 0] = speed * math.sin(ps), speed * math.cos(ps)
    tw._match_spin(np.array([0]), np.array([1.0]))
    p = tw.S[0:3, 0].tolist()
    p[1] += air
    return {'type': 'state', 'p': p, 'q': tw.S[3:7, 0].tolist(), 'v': tw.S[7:10, 0].tolist(), 'w': [0.0, 0.0, 0.0],
            'wheel_spin': tw.wheel_spin[0].tolist()}


def act(b=0.0, s=0.0, u=0.0, n=1):
    a = np.zeros((n, NA))
    a[:, :NU] = u
    a[:, DRIVE_IDX['brake']], a[:, DRIVE_IDX['steer']] = b, s
    return a


def matched(tw, e=0):
    """Rolling-matched spin -v_t / r of each wheel at the current state (v1 axes: no toe, no steer)."""
    from test_twin_v2 import _matched
    return _matched(tw, e)


def test_contract_and_constants():
    assert ND == len(DRIVE['channels']) == 2 and NA == NU + ND and DRIVE_IDX == {'brake': NU, 'steer': NU + 1}
    tw = Twin(1)
    assert tw.k_brake == 1 - __import__('coldgas.fmath', fromlist=['exp']).exp(-tw.dt / DRIVE['brake']['tau'])
    assert DP['steer_max'] == DRIVE['steer']['max_deg'] * math.pi / 180 and tw.steer_step == DP['steer_rate'] * tw.dt
    assert tw.brake_T.tolist() == [DRIVE['brake']['torque_max'][a] for a in (0, 0, 1, 1)]


@pytest.mark.parametrize('V', [None, V1, V2], ids=['spec', 'V1', 'V2'])
def test_zero_drive_is_the_valve_only_action(V):
    """An NA action with zero drive commands is the NU action, bit for bit (rolling, braking-free, in the air)."""
    for init in (rolling(12, vehicle=V), rolling(12, vehicle=V, air=0.6)):
        a, b = Twin(1, vehicle=V), Twin(1, vehicle=V)
        a.reset([init]), b.reset([init])
        for k in range(60):
            u = 0.3 if k % 7 == 0 else 0.0
            a.step(np.full((1, NU), u))
            b.step(act(u=u))
            assert (a.S == b.S).all() and (a.wheel_spin == b.wheel_spin).all()
        assert a.events == b.events and b.drive(0) == {'brake': 0.0, 'steer': 0.0}


def test_command_clamp_nan_and_missing():
    tw = Twin(1)
    tw.reset([rolling(5)])
    for cmd, want in (([2.0, 5.0], [1.0, 1.0]), ([-1.0, -3.0], [0.0, -1.0]), ([math.nan, math.nan], [0.0, 0.0]),
                      ([0.4, -0.25], [0.4, -0.25]), ([0.7], [0.7, 0.0]), ([], [0.0, 0.0])):
        tw.step(np.array([[0.0] * NU + cmd]))
        assert tw.ud[0].tolist() == want, cmd


def test_drive_forced_to_zero_while_unarmed():
    """On the deck run (arm 'edge') brake and steer commands are ignored like the valves; after the edge they act."""
    tw = Twin(1, arm='edge')
    # a launch like the presets (14 deg towards the edge): a shallow one may yaw away from the edge on the deck with
    # the fitted toe and never arm; the step cap keeps a regression from hanging the suite
    tw.reset([{'type': 'launch', 'speed': 60, 'angle': 14, 'spin': [0.4, 0.2, -0.3], 'lead': 0.4}])
    for _ in range(200):
        if tw.armed[0]:
            break
        tw.step(act(1.0, 1.0))
        assert tw.ud[0].tolist() == [0.0, 0.0] and tw.drive(0) == {'brake': 0.0, 'steer': 0.0}
    assert tw.armed[0]
    tw.step(act(1.0, 1.0))
    assert tw.ud[0].tolist() == [1.0, 1.0] and tw.drive(0)['brake'] > 0.3 and tw.drive(0)['steer'] > 0


def test_steer_rate_limit_and_lock():
    """delta moves by at most rate * dt per integrator step towards s * lock, reaches the lock exactly, reverses at the
    same rate, and returns exactly to 0 (then the front wheels are the static toe again, bit for bit)."""
    tw = Twin(1)
    tw.reset([rolling(8)])
    c0, s0 = tw.c0.copy(), tw.s0.copy()
    r, lock, prev = tw.steer_step, DP['steer_max'], 0.0
    tw.ud[0] = [0.0, 1.0]
    tw._cmd_s = True
    for k in range(1, 800):
        tw.substep(np.zeros((1, NU)))
        d = tw.drive_steer[0]
        assert d - prev <= r * (1 + 1e-12) and d <= lock
        assert d == pytest.approx(min(k * r, lock), rel=1e-9, abs=1e-15)
        prev = d
    assert tw.drive_steer[0] == lock
    for _ in range(int(lock / (0.02 * DRIVE['steer']['rate_deg_s'] * math.pi / 180)) + 3):
        tw.step(act(0.0, 0.0))
    assert tw.drive_steer[0] == 0.0 and (tw.c0 == c0).all() and (tw.s0 == s0).all()
    # +steer turns the car right (+ world yaw rate about -Y ... heading increases from -90 towards -X/+Z): body right = +Z here
    tw.reset([rolling(10)])
    for _ in range(40):
        tw.step(act(0.0, 1.0))
    assert tw.S[9, 0] > 1.0  # moving towards +Z = the car's right at heading -90


def test_brake_never_reverses_spin():
    """In the air the brake takes each wheel's spin towards 0 by at most T dt / I per step and stops it there (both
    signs); on the ground a rolling car brakes to a standstill without any wheel ever spinning backwards."""
    tw = Twin(1, vehicle=V2)
    init = dict(rolling(10, vehicle=V2, air=1.5), wheel_spin=[30.0, -20.0, 5.0, 0.0])
    tw.reset([init])
    tw.ud[0] = [1.0, 0.0]
    tw._cmd_b = True
    prev = tw.wheel_spin[0].copy()
    for _ in range(300):
        tw.substep(np.zeros((1, NU)))
        w = tw.wheel_spin[0]
        assert (np.abs(w) <= np.abs(prev)).all() and (w * prev >= 0).all()
        dmax = tw.drive_brake[0] * tw.brake_T * tw.dt / tw.w_inertia
        assert (np.abs(prev) * tw.spin_keep - np.abs(w) <= dmax * (1 + 1e-12) + 1e-12).all()  # bearing decay + brake
        prev = w.copy()
    assert (tw.wheel_spin[0] == 0).all() and tw.wheel_N[0].max() == 0  # still in the air, wheels held
    tw.reset([rolling(15, vehicle=V2)])
    for _ in range(200):
        tw.step(act(1.0))
        assert (tw.wheel_spin[0] >= 0).all()
    assert math.hypot(tw.S[7, 0], tw.S[9, 0]) < 0.2


def test_abs_slip_bound_on_loaded_wheels():
    """Full brake on a rolling car: once the pressure is up the loaded wheels sit at the ABS target (1 - abs_slip) x
    rolling-matched, never below it (no lock-up), and the car decelerates at the tyres' friction limit."""
    keep = 1 - DRIVE['brake']['abs_slip']
    tw = Twin(1, vehicle=V2)
    tw.reset([rolling(20, vehicle=V2)])
    ratios, v0 = [], math.hypot(tw.S[7, 0], tw.S[9, 0])
    for k in range(60):
        tw.step(act(1.0))
        m = matched(tw)
        if k >= 15:
            assert tw.wheel_N[0].min() > 0
            ratios.append(tw.wheel_spin[0] / m)
    ratios = np.array(ratios)
    assert ratios.min() >= keep - 2e-3 and np.median(ratios) == pytest.approx(keep, abs=5e-3)
    v1 = math.hypot(tw.S[7, 0], tw.S[9, 0])
    assert (v0 - v1) / (60 * 0.02) > 5.0  # m/s^2: braking at the friction limit
    # with brake 0 the same car keeps rolling (crr only)
    t0 = Twin(1, vehicle=V2)
    t0.reset([rolling(20, vehicle=V2)])
    for _ in range(60):
        t0.step(act(0.0))
    assert math.hypot(t0.S[7, 0], t0.S[9, 0]) > 19


def test_brake_lag_and_release():
    tw = Twin(1, vehicle=V2)
    tw.reset([rolling(15, vehicle=V2)])
    tw.step(act(1.0))
    kb = tw.k_brake
    assert tw.drive_brake[0] == pytest.approx(1 - (1 - kb) ** tw.nsub, rel=1e-12)
    for _ in range(100):
        tw.step(act(1.0))
    assert tw.drive_brake[0] == 1.0  # snapped onto the command
    for _ in range(100):
        tw.step(act(0.0))
    assert tw.drive_brake[0] == 0.0 and not tw._brake_any  # released completely: the drive path is idle again


def test_brake_is_a_noop_without_wheel_spin():
    """v1 (Coulomb tyre, no spin): the brake pressure follows the command but changes nothing; steering still acts."""
    init = rolling(12, vehicle=V1)
    a, b, c = Twin(1, vehicle=V1), Twin(1, vehicle=V1), Twin(1, vehicle=V1)
    for t in (a, b, c):
        t.reset([init])
    for _ in range(50):
        a.step(act(0.0)), b.step(act(1.0)), c.step(act(0.0, 0.5))
    assert (a.S == b.S).all() and b.drive_brake[0] > 0.999
    assert abs(c.S[2, 0] - a.S[2, 0]) > 0.5


V2S = over(V2, {'tyre': {'slide': [0.5, 1]}, 'wheel_spin': {'slip_ratio_sat': [0.08, 0]}})  # per-axle slide and kappa


@pytest.mark.parametrize('V', [None, V2, V2T, V2S], ids=['spec', 'V2', 'V2T', 'V2S'])
def test_lanes_equal_scalar(V):
    """Lanes with different drive (none, brake, steer, both, released) equal the scalar twin bit for bit, with and
    without static toe (without toe only steered lanes rotate their tyre axes)."""
    inits = [rolling(14, vehicle=V), rolling(18, vehicle=V), rolling(10, heading=-60, z=18, vehicle=V),
             rolling(16, vehicle=V, air=0.4), rolling(12, vehicle=V)]
    plan = [lambda k: (0.0, 0.0), lambda k: (1.0, 0.0), lambda k: (0.0, 1.0 if k < 40 else -0.5),
            lambda k: (0.8, 0.4), lambda k: (1.0, 0.7) if k < 20 else (0.0, 0.0)]
    tb = Twin(len(inits), vehicle=V)
    tb.reset(inits)
    for k in range(90):
        tb.step(np.concatenate([act(*plan[i](k), u=0.05 * (k % 3 == 0)) for i in range(len(inits))]))
    for i, init in enumerate(inits):
        ts = Twin(1, vehicle=V)
        ts.reset([init])
        for k in range(90):
            ts.step(act(*plan[i](k), u=0.05 * (k % 3 == 0)))
        assert (ts.S[:, 0] == tb.S[:, i]).all() and (ts.wheel_spin[0] == tb.wheel_spin[i]).all(), i
        assert ts.drive(0) == tb.drive(i) and (ts.wheel_lat[0] == tb.wheel_lat[i]).all() and ts.events[0] == tb.events[i]


@pytest.mark.parametrize('V', [None, V2], ids=['spec', 'V2'])
def test_load_state_restores_drive_and_matches_js(node, V):
    """A FlightState carries drive {brake, steer}; load_state restores it (0 if absent), the steered heading enters the
    rolling-matched spin, and planning from it is bit-identical in JS (incl. the drive state and wheel spin)."""
    src = Twin(1, vehicle=V)
    src.reset([rolling(15, vehicle=V)])
    for _ in range(12):
        src.step(act(0.7, -0.6))
    s = _flight_state(src, spin=False)  # spin from the phase rule along the steered heading
    assert s['drive'] == src.drive(0) and s['drive']['brake'] > 0.5 and s['drive']['steer'] < -0.1
    py = Twin(1, vehicle=V)
    py.load_state(batch_of(s))
    assert py.drive(0) == s['drive'] and (py.c0 == src.c0).all() and (py.s0 == src.s0).all()
    nod = Twin(1, vehicle=V)
    nod.load_state(batch_of({k: v for k, v in s.items() if k != 'drive'}))
    assert nod.drive(0) == {'brake': 0.0, 'steer': 0.0}
    rows, cmd = [], [0.1] * NU + [0.3, 0.8]
    for _ in range(30):
        py.step(np.array([cmd]))
        rows.append([*py.S[:, 0].tolist(), *py.wheel_spin[0].tolist(), py.drive_brake[0], py.drive_steer[0]])
    js = node(f"""
import {{ RigidTwin }} from './src/flight/twin.js';
const tw = new RigidTwin({{ vehicle: {json.dumps(V)} }}).loadState({json.dumps(s)}), rows = [], u = {json.dumps(cmd)};
const d0 = tw.drive;
for (let k = 0; k < 30; k++) {{ tw.step(u); rows.push([...tw.flat(), ...tw.wheelSpin, tw.beta, tw.delta]); }}
console.log(JSON.stringify({{ rows, d0, events: tw.events }}));""")
    assert js['d0'] == s['drive'] and js['rows'] == rows and js['events'] == py.events[0]


def _sidewalk_task(kerb=False):
    """escape-land-stop with the car parked on the +Z sidewalk (a rest start is off the deck at once)."""
    t = load_task('escape-land-stop')
    t['initial'] = {'type': 'rest', 'position': [60, 0, 30], 'heading_deg': -90}
    if kerb is None:
        del t['success']['kerb']
    return t


def test_kerb_terminates_where_forbidden(node):
    r = run_episode(_sidewalk_task(), 'zero')
    m, ev = r['metrics'], r['events']
    assert m['reason'] == 'kerb' and m['kerb'] is True and m['success'] is False and m['t_end'] == pytest.approx(0.02)
    assert [e['type'] for e in ev].count('kerb') == 1 and next(e for e in ev if e['type'] == 'kerb')['wheel'] == 0
    task = _sidewalk_task()
    assert m['return'] == pytest.approx(task['reward']['terminal_hull_strike'], abs=0.01)  # terminal_kerb defaults to it
    ok = run_episode(_sidewalk_task(kerb=None), 'zero')  # not forbidden: the event is only a diagnostic
    assert ok['metrics']['reason'] != 'kerb' and ok['metrics']['kerb'] is True and ok['traj']['contact'][1] & 32
    js = node(f"""
import {{ runEpisode }} from './src/flight/run.js';
console.log(JSON.stringify(runEpisode({{ task: {json.dumps(_sidewalk_task())}, controller: 'zero' }}).metrics));""")
    assert js == m


def test_stop_task_rules():
    """land-stop-pd (pd + full brake after the touchdown): 'stopped' once slow after the hold, stop metrics consistent,
    stop_within enforced; without brakes the car does not stop."""
    from coldgas.parity import WithDrive
    drive = [[3.5, 1, 0], [4.6, 1, -0.15]]
    r = run_episode('escape-land-stop', WithDrive(create_controller('pd'), drive), seed=3, preset='flat')
    m, T = r['metrics'], r['traj']
    assert m['reason'] == 'stopped' and m['stopped'] is True and m['success'] is True and m['kerb'] is False
    assert m['stop_time'] == pytest.approx(m['t_end'] - m['touchdown_t'], abs=1e-12) and m['stop_time'] <= 8
    s = T['state'][-1]
    assert math.hypot(s[7], s[9]) <= 0.5 and 5 < m['stop_distance'] < 40
    assert T['drive_cmd'][-1] == [0.0, 0.0] and T['drive_cmd'][-2] == [1.0, -0.15] and T['drive'][-1][0] == 1.0
    task = load_task('escape-land-stop')
    task['success']['stop_within'] = 0.5
    r2 = run_episode(task, WithDrive(create_controller('pd'), drive), seed=3, preset='flat')
    assert r2['metrics']['reason'] == 'stopped' and r2['metrics']['success'] is False
    assert r2['metrics']['return'] < m['return'] - 5  # no terminal_success
    task['horizon'] = 6.0
    r3 = run_episode(task, 'pd', seed=3, preset='flat')  # no brakes: rolls on (into the kerb)
    assert r3['metrics']['reason'] != 'stopped' and r3['metrics']['stopped'] is False and r3['metrics']['stop_time'] is None


def test_replay_plays_drive_cmd():
    traj = {'format': 'coldgas-traj@1', 'dt': 0.02, 'u_cmd': [[0.5] * NU, [0.0] * NU], 'drive_cmd': [[0.25, -0.5], [0.0, 0.0]]}
    rp = create_controller({'name': 'replay', 'traj': traj})
    assert rp.act({'t': 0.0}).tolist() == [0.5] * NU + [0.25, -0.5] and len(rp.act({'t': 1.0})) == NA
    old = create_controller({'name': 'replay', 'traj': {k: v for k, v in traj.items() if k != 'drive_cmd'}})
    assert len(old.act({'t': 0.0})) == NU


def _heading_deg(tw, e=0):
    x, y, z, w = tw.S[3:7, e]
    return math.degrees(math.atan2(2 * (x * z + y * w), 1 - 2 * (x * x + y * y)))


def test_slip_ratio_saturation_keeps_lateral_grip_under_abs():
    """wheel_spin.slip_ratio_sat kappa (5.11): with a tiny v_long_sat a wheel braked at the ABS slip bound saturates the
    friction circle longitudinally (braking + steering barely turns); with kappa the saturation velocity grows with the
    rolling speed, the braked front wheels keep lateral force and the car turns while it brakes. kappa = 0 = the old
    model bit for bit; the ABS bound itself is unchanged."""
    # V2 with the coldgas-rigid@2 fit's tiny v_long_sat (0.036 m/s), fit-independent like the other mechanism tests
    base = over(V2, {'wheel_spin': {'v_long_sat': 0.036378}})
    init = rolling(18, vehicle=base)
    out = {}
    for name, k in (('spec', None), ('zero', 0), ('kappa', 0.08)):
        tw = Twin(1, vehicle=base if k is None else over(base, {'wheel_spin': {'slip_ratio_sat': k}}))
        tw.reset([init])
        h0 = _heading_deg(tw)
        for _ in range(40):
            tw.step(act(1.0, 0.8))
        out[name] = (tw.S.copy(), _heading_deg(tw) - h0, math.hypot(tw.S[7, 0], tw.S[9, 0]))
    assert (out['spec'][0] == out['zero'][0]).all()
    assert out['kappa'][1] > 5 and out['kappa'][1] - out['zero'][1] > 15  # degrees turned in 0.8 s (right, as steered)
    assert out['kappa'][2] < 18 - 3.5  # and it still brakes hard (> 4 m/s² over the 0.8 s, incl. the pressure lag)


def test_tyre_slide_blends_towards_isotropic_coulomb():
    """tyre.slide (5.11): a locked wheel sliding sideways (sigma_x = 30) with slide = 1 carries (to 1 - xi = 0.1 %) the
    isotropic Coulomb lateral force -mu_e N v_l / |(v_t + omega r, v_l)|, the same per unit load front and rear whatever
    c_alpha says; slide = 0 keeps the slip law (sigma_x dominates its friction circle); per axle, the wheels with
    slide = 0 are untouched bit for bit."""
    from coldgas.twin import quat_to_mat
    from coldgas.world import GROUND
    init = dict(rolling(15, vehicle=V2), v=[-15.0, 0.0, 3.0], wheel_spin=[0.0] * 4)  # locked, sliding to the car's right
    out = {}
    for name, sl in (('off', 0), ('on', 1), ('front', [1, 0])):
        tw = Twin(1, vehicle=over(V2, {'tyre': {'slide': sl}}))
        tw.reset([init])
        tw.substep(np.zeros((1, NU)))  # one integrator step from the locked state
        out[name] = (tw.wheel_lat[0].copy(), tw.wheel_N[0].copy())
    R = quat_to_mat(*init['q'])
    ax, az = R[0], R[6]
    h = math.hypot(ax, az)
    vt, vl = (init['v'][0] * az - init['v'][2] * ax) / h, (init['v'][0] * ax + init['v'][2] * az) / h
    mu = np.array([V2['wheels']['tyre_mu'][a] for a in (0, 0, 1, 1)]) * GROUND['mu']
    coulomb = -mu * vl / math.hypot(vt, vl)  # lateral force per unit load of isotropic sliding
    lat, N = out['on']
    assert N.min() > 0 and np.allclose(lat / N, coulomb, rtol=2e-3)
    assert (np.abs(out['off'][0] / out['off'][1]) < 0.8 * np.abs(coulomb)).all()  # the slip law grips less here
    assert (out['front'][0][:2] == out['on'][0][:2]).all() and (out['front'][0][2:] == out['off'][0][2:]).all()

"""Physical sanity checks of the rigid twin (python -m coldgas.sanity). The JS twin gives identical numbers (parity).

1 free fall from deck height (level drop, and the 'flat' launch)   4 gas use / blowdown with all bottom nozzles open
2 parked at rest: settling, jitter, creep                         5 energy / angular momentum in a free tumble
3 hover: bottom thrust vs weight, gas rate                       6 no tunnelling: hard drop onto the deck, deck run
"""
from __future__ import annotations

import math

import numpy as np

from .contract import NU, GROUPS, CONTROL_DT
from .controllers import create_controller
from .episode import run_episode
from .spec import WORLD, LAUNCH
from .twin import Twin, quat_to_mat
from .tasks import load_task, sample_initial


def _run(tw, u, steps, cb=None):
    for k in range(steps):
        tw.step(np.asarray(u).reshape(1, NU))
        if cb:
            cb(k)


def free_fall():
    top = WORLD['deck']['top']
    tw = Twin(1)
    tw.reset([{'type': 'rest', 'x': -30, 'z': 12, 'heading_deg': 0}])
    y_rest = float(tw.S[1, 0])
    tw.reset([{'type': 'state', 'p': [-30, y_rest + top, 12], 'q': tw.S[3:7, 0].tolist(), 'v': [0, 0, 0], 'w': [0, 0, 0]}])
    for _ in range(200):
        tw.step(np.zeros((1, NU)))
        if tw.td[0]:
            break
    t_ideal = math.sqrt(2 * top / WORLD['gravity'])
    out = {'drop_m': top, 'impact_t': float(tw.td_t[0]), 'impact_vy': float(-tw.td_S[8, 0]),
           'ideal_t': t_ideal, 'ideal_vy': WORLD['gravity'] * t_ideal}
    r = run_episode('escape-soft-landing', 'zero', seed=0, preset='flat', record=True)
    ev = {e['type']: e['t'] for e in r['events']}
    tr = r['traj']
    vy = np.array(tr['state'])[:, 8]
    k0 = int(np.argmax(vy < -0.1))  # start of the fall: COM sinking off the edge
    out.update({'flat_edge_t': ev['edge'], 'flat_fall_start_t': tr['t'][k0], 'flat_touchdown_t': ev['touchdown'],
                'flat_fall_time': ev['touchdown'] - tr['t'][k0], 'flat_impact_vy': r['metrics']['touchdown_vertical_speed']})
    return out


def rest(seconds=10.0):
    tw = Twin(1)
    tw.reset([{'type': 'rest', 'x': -30, 'z': 12, 'heading_deg': 37}])
    p0, res = tw.S[0:3, 0].copy(), tw.settle_residual[:, 0].copy()
    vmax, wmax, late = [], [], []
    n = round(seconds / CONTROL_DT)

    def cb(k):
        v, w = np.linalg.norm(tw.S[7:10, 0]), np.linalg.norm(tw.S[10:13, 0])
        vmax.append(v)
        wmax.append(w)
    _run(tw, np.zeros(NU), n, cb)
    drift = float(np.linalg.norm(tw.S[0:3, 0] - p0))
    R = quat_to_mat(*tw.S[3:7, 0])
    out = {'settle_residual_N': float(np.abs(res).max()), 'max_speed': float(max(vmax)), 'max_rate': float(max(wmax)),
           'speed_last_s': float(max(vmax[-50:])), 'drift_m': drift, 'pitch_deg': math.degrees(math.asin(-R[7])),
           'com_height': float(tw.S[1, 0]), 'wheels': bin(int(tw.wheel_mask[0])).count('1')}
    # perturbed: dropped from 5 cm above equilibrium with 2 deg of roll, then left alone for `seconds`
    q = tw.S[3:7, 0]
    qr = (math.sin(math.radians(1)), 0.0, 0.0, math.cos(math.radians(1)))  # 2 deg about world x
    qn = (qr[3] * q[0] + qr[0] * q[3], qr[3] * q[1] - qr[0] * q[2], qr[3] * q[2] + qr[0] * q[1], qr[3] * q[3] - qr[0] * q[0])
    tw.reset([{'type': 'state', 'p': [float(p0[0]), float(p0[1]) + 0.05, float(p0[2])], 'q': list(qn), 'v': [0, 0, 0], 'w': [0, 0, 0]}])
    sp, pos = [], []
    _run(tw, np.zeros(NU), n, lambda k: (sp.append(float(np.linalg.norm(tw.S[7:10, 0])) + float(np.linalg.norm(tw.S[10:13, 0]))), pos.append(tw.S[0:3, 0].copy())))
    half = len(sp) // 2
    out['perturbed'] = {'speed+rate_after_2s': float(max(sp[100:])), 'speed+rate_last_5s': float(max(sp[half:])),
                        'creep_last_5s_m': float(np.linalg.norm(pos[-1] - pos[half])), 'offset_from_equilibrium_m': float(np.linalg.norm(pos[-1] - p0))}
    return out


def hover():
    """pd hover-hold: thrust/weight while holding, gas rate, height error."""
    task = load_task('hover-hold')
    env_ctrl = create_controller('pd')
    tw = Twin(1, arm='start')
    tw.reset([sample_initial(task, 0)])
    env_ctrl.reset(task=task)
    tw_ratio, m_hold = [], []
    for _ in range(round(4.0 / CONTROL_DT)):
        u = env_ctrl.act(tw.state(0))
        tw.step(u.reshape(1, NU))
        if 2.6 < tw.k[0] * tw.dt < 3.4:  # steady hold (the hold completes at ~3.4 s, then it lands)
            M = tw.M[0]
            tw_ratio.append(float(tw.prop.thrust[0, GROUPS['bottom']].sum() / (M * WORLD['gravity'])))
            m_hold.append(float(tw.prop.m[0]))
    r = run_episode('hover-hold', 'pd', seed=0)
    return {'thrust_over_weight_mean': float(np.mean(tw_ratio)), 'thrust_over_weight_min': float(np.min(tw_ratio)),
            'thrust_over_weight_max': float(np.max(tw_ratio)), 'gas_rate_kg_s': (m_hold[0] - m_hold[-1]) / (len(m_hold) - 1) / CONTROL_DT,
            'episode': {k: r['metrics'][k] for k in ('success', 'hold_time', 'rms_height_error', 'gas_used_kg', 'touchdown_vertical_speed')}}


def blowdown():
    """All four bottom nozzles fully open from a full tank, in the air."""
    tw = Twin(1)
    tw.reset([{'type': 'state', 'p': [0, 500, 0], 'q': [0, 0, 0, 1], 'v': [0, 0, 0], 'w': [0, 0, 0]}])
    u = np.zeros(NU)
    u[GROUPS['bottom']] = 1.0
    m0 = float(tw.prop.m[0])
    rows, impulse = [], 0.0
    t_drop = t_empty = None
    for k in range(round(6.0 / CONTROL_DT)):
        tw.step(u.reshape(1, NU))
        F = float(tw.prop.thrust[0].sum())
        impulse += F * CONTROL_DT  # rectangle rule at the control rate (report only)
        t = float(tw.k[0] * tw.dt)
        if t_drop is None and tw.dropout[0]:
            t_drop = t
        if t_empty is None and tw.prop.pc[0] == 0:
            t_empty = t
        if k % 25 == 24:
            rows.append((round(t, 2), round(float(tw.prop.m[0]), 2), round(float(tw.prop.p[0]) / 1e6, 2), round(float(tw.prop.T[0]), 1), round(F / 1e3, 2)))
    used = m0 - float(tw.prop.m[0])
    return {'gas0_kg': m0, 'dropout_t': t_drop, 'exhausted_t': t_empty, 'used_kg': used, 'impulse_kNs': impulse / 1e3,
            'mean_isp_s': impulse / (used * WORLD['gravity']), 'rows(t, m kg, p MPa, T K, F kN)': rows}


def energy(seconds=6.0):
    """Free tumble, no thrust/aero/contacts: drift of total energy and of |angular momentum| (world)."""
    tw = Twin(1)
    tw.ka = [0.0, 0.0, 0.0]
    tw.reset([{'type': 'state', 'p': [0, 2000, 0], 'q': [0.3, -0.2, 0.5, 0.787400787], 'v': [3, 0, 1], 'w': [1.6, -0.9, 2.4]}])

    def E():
        S, I = tw.S[:, 0], tw.I[:, 0]
        w = S[10:13]
        Iw = np.array([I[0] * w[0] + I[3] * w[1] + I[4] * w[2], I[3] * w[0] + I[1] * w[1] + I[5] * w[2], I[4] * w[0] + I[5] * w[1] + I[2] * w[2]])
        R = np.array(quat_to_mat(*S[3:7])).reshape(3, 3)
        return 0.5 * tw.M[0] * S[7:10] @ S[7:10] + tw.M[0] * WORLD['gravity'] * S[1] + 0.5 * w @ Iw, np.linalg.norm(R @ Iw), 0.5 * w @ Iw
    E0, L0, Er0 = E()
    _run(tw, np.zeros(NU), round(seconds / CONTROL_DT))
    E1, L1, Er1 = E()
    return {'seconds': seconds, 'rel_dE_total': (E1 - E0) / E0, 'rel_dE_rot': (Er1 - Er0) / Er0, 'rel_dL': (L1 - L0) / L0}


def tunnelling():
    top = WORLD['deck']['top']
    tw = Twin(1)
    tw.reset([{'type': 'rest', 'x': -30, 'z': 0, 'heading_deg': 90}])
    y_rest = float(tw.S[1, 0])
    # 1) 20 m/s straight down onto the deck, level, with 25 m/s along it
    tw.reset([{'type': 'state', 'p': [40, top + y_rest + 0.3, 0], 'q': tw.S[3:7, 0].tolist(), 'v': [25, -20, 0], 'w': [0, 0, 0]}])
    ymin = math.inf
    for _ in range(round(1.5 / CONTROL_DT)):
        tw.step(np.zeros((1, NU)))
        ymin = min(ymin, float(tw.S[1, 0]))
    deck_drop = {'min_com_height_above_deck': ymin - top, 'final_com_height_above_deck': float(tw.S[1, 0]) - top, 'hull_strike': bool(tw.hs[0])}
    # 2) the launch run-up at the fastest preset speed + 10 km/h: stays on the deck until the edge
    P = LAUNCH['presets']['corner']
    tw.reset([{'type': 'launch', 'speed': P['speed'] + 10, 'angle': P['angle'], 'spin': [0, 0, 0]}])
    y0, ymin = float(tw.S[1, 0]), math.inf
    while not tw.left_deck[0]:
        tw.step(np.zeros((1, NU)))
        ymin = min(ymin, float(tw.S[1, 0]))
    return {'deck_drop': deck_drop, 'runup_com_dip_m': y0 - ymin}


def report():
    out = {'free_fall': free_fall(), 'rest': rest(), 'hover': hover(), 'blowdown': blowdown(), 'energy': energy(), 'tunnelling': tunnelling()}
    return out


if __name__ == '__main__':
    import json
    print(json.dumps(report(), indent=1, default=float))

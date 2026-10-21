"""Twin structure constants (spec/vehicle.json `twin`) and the contact parameters of one twin instance (mirror of
src/flight/model.js): spec VEHICLE, optionally deep-merged with a partial override (fitting / experiments only),
resolved per wheel and per hull point, and the brake / steer constants of spec/drive.json (the car's own hardware, not
overridable). Semantics: spec/README.md section 5 (5.11, 5.12, 5.13)."""
from __future__ import annotations

import math

import numpy as np

from . import fmath
from .spec import MODEL, VEHICLE, GENERATED, DRIVE

MODEL_ID: str = MODEL['model']
CONTACT = MODEL['contact']
PLACEMENT = MODEL['placement']
LIMITS = MODEL['limits']

_AXLE_KEYS = ('wheels.spring_k', 'wheels.damper_c', 'wheels.travel', 'wheels.bump_k', 'wheels.bump_c', 'wheels.bump_k2',
              'wheels.tyre_mu', 'wheels.crr', 'wheels.toe_compliance', 'tyre.c_alpha', 'tyre.v_slip_min', 'wheel_spin.inertia',
              'wheel_spin.v_long_sat', 'wheel_spin.slip_ratio_sat', 'tyre.slide')


def merge_params(a: dict, b: dict | None, path: str = 'vehicle') -> dict:
    """Deep merge for parameter overrides: dicts key by key, lists / scalars / None replace (same as model.js). Keys
    the spec does not have are an error (a misspelt parameter would otherwise be silently ignored by a fit)."""
    out = dict(a)
    for k, v in (b or {}).items():
        if k not in a:
            raise ValueError(f'{path}.{k}: no such parameter in spec/vehicle.json')
        out[k] = merge_params(a[k], v, f'{path}.{k}') if isinstance(v, dict) and isinstance(a.get(k), dict) else v
    return out


def _is_num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def _per_axle(x, name):
    """A number (both axles) or [front, rear] -> (front, rear) as floats."""
    if _is_num(x):
        return float(x), float(x)
    if isinstance(x, (list, tuple)) and len(x) == 2 and all(_is_num(y) for y in x):
        return float(x[0]), float(x[1])
    raise ValueError(f'vehicle {name}: expected a number or [front, rear], got {x!r}')


def _thresh(x, name):
    if x is None:
        return math.inf  # off: `>` never fires
    if not _is_num(x) or not x > 0:
        raise ValueError(f'vehicle failure.{name}: expected None or a positive number')
    return float(x)


def drive_params() -> dict:
    """Brakes and steering (spec/drive.json, spec/README.md 5.13; model.js driveParams): brake torque per wheel
    (front, rear) N m, pressure lag tau (s), ABS slip bound, steer lock and rate in rad, rad/s (deg * pi / 180)."""
    B, S = DRIVE['brake'], DRIVE['steer']
    torque = _per_axle(B['torque_max'], 'drive brake.torque_max')
    if not (all(x >= 0 for x in torque) and B['tau'] > 0 and 0 <= B['abs_slip'] < 1 and S['max_deg'] >= 0 and S['rate_deg_s'] >= 0):
        raise ValueError('spec/drive.json: need torque_max >= 0, tau > 0, 0 <= abs_slip < 1, max_deg >= 0, rate_deg_s >= 0')
    return {'torque': torque, 'tau': float(B['tau']), 'abs_slip': float(B['abs_slip']),
            'steer_max': S['max_deg'] * math.pi / 180, 'steer_rate': S['rate_deg_s'] * math.pi / 180}


def contact_params(override: dict | None = None) -> dict:
    """Contact parameters of one twin (see model.js contactParams): per-wheel float tuples (wheel order of the
    generated massprops; axle 0 = front = centre z > 0), per-hull-point arrays (hull.points then hull.floor_points)."""
    if override and 'integrator' in override:
        raise ValueError("vehicle.integrator: use the twin's dt option")
    V = merge_params(VEHICLE, override) if override else VEHICLE
    H, TY, SP, FA, C = V['hull'], V['tyre'], V['wheel_spin'], V['failure'], V['twin']['contact']
    if TY['model'] not in ('coulomb', 'slip'):
        raise ValueError(f"vehicle tyre.model: 'coulomb' or 'slip', got {TY['model']!r}")

    def get(name):
        o = V
        for k in name.split('.'):
            o = o[k]
        return _per_axle(o, name)

    ax = {n: get(n) for n in _AXLE_KEYS}
    wheels = GENERATED['massprops']['wheels']
    axle = [0 if w['centre'][2] > 0 else 1 for w in wheels]
    per_wheel = {n.split('.')[1]: tuple(ax[n][a] for a in axle) for n in _AXLE_KEYS}
    toe = V['wheels']['toe']
    if not (isinstance(toe, (list, tuple)) and len(toe) == len(wheels) and all(_is_num(x) for x in toe)):
        raise ValueError(f'vehicle wheels.toe: expected {len(wheels)} numbers (FL FR RL RR), got {toe!r}')
    # static toe as its cos / sin (portable fmath, so both languages rotate the tyre axes identically)
    per_wheel['toe'] = tuple(float(x) for x in toe)
    per_wheel['toe_cos'] = tuple(fmath.cos(float(x)) for x in toe)
    per_wheel['toe_sin'] = tuple(fmath.sin(float(x)) for x in toe)
    per_wheel['brake_torque'] = tuple(drive_params()['torque'][a] for a in axle)  # spec/drive.json, per axle
    floor = H.get('floor_points') or []
    pts = list(H['points']) + list(floor)
    n0 = len(H['points'])
    pick = lambda x, d: d if x is None else x
    P = {
        'V': V, 'axle': axle, 'wheel': per_wheel, 'ks_axle': ax['wheels.spring_k'],
        'slip': TY['model'] == 'slip', 'spin_on': bool(SP['enabled']), 'spin_air': SP['spin_ratio_air'], 'spin_decay': SP['decay'],
        'fail_N': _thresh(FA['wheel_normal_max'], 'wheel_normal_max'), 'fail_L': _thresh(FA['wheel_lat_max'], 'wheel_lat_max'),
        'fail_I': _thresh(FA['wheel_lat_impulse_max'], 'wheel_lat_impulse_max'),
        'hull': np.array(pts, dtype=np.float64).reshape(-1, 3),
        'kh': np.array([H['k'] if j < n0 else pick(H.get('floor_k'), H['k']) for j in range(len(pts))], dtype=np.float64),
        'ch': np.array([H['c'] if j < n0 else pick(H.get('floor_c'), H['c']) for j in range(len(pts))], dtype=np.float64),
        'muh': np.array([H['mu'] if j < n0 else pick(H.get('floor_mu'), H['mu']) for j in range(len(pts))], dtype=np.float64),
        'vreg': C['v_reg'], 'hmin': C['wheel_min_h'], 'placement': V['twin']['placement'], 'CdA': V['aero']['CdA'],
    }
    w = per_wheel
    P['bump_x'] = any(x != 0 for x in w['bump_c'] + w['bump_k2'])  # v2 bump-stop terms skipped when zero (v1 path)
    P['comply_on'] = any(x != 0 for x in w['toe_compliance'])
    P['toe_on'] = P['comply_on'] or any(x != 0 for x in w['toe'])  # tyre axes rotated (else v1 axes, no rotation evaluated)
    P['fail_on'] = any(P[k] != math.inf for k in ('fail_N', 'fail_L', 'fail_I'))
    if P['spin_on'] and not all(x > 0 for x in w['inertia'] + w['v_long_sat']):
        raise ValueError('vehicle wheel_spin: inertia and v_long_sat must be > 0')
    if not all(x >= 0 for x in w['slip_ratio_sat']):
        raise ValueError('vehicle wheel_spin.slip_ratio_sat must be >= 0')
    if not all(0 <= x <= 1 for x in w['slide']):
        raise ValueError('vehicle tyre.slide must be in [0, 1]')
    if P['slip'] and not all(x > 0 for x in w['v_slip_min']):
        raise ValueError('vehicle tyre.v_slip_min must be > 0')
    return P

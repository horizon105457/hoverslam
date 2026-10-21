"""coldgas-traj@1 writer (spec/README.md section 7), same layout as src/flight/traj.js (incl. contact bit 32 = kerb,
drive_cmd = applied [brake, steer] over [t_k, t_k+1), drive = [beta, delta] at t_k: 5.13)."""
from __future__ import annotations

import json

from .contract import TRAJ_FORMAT, MODEL, NU, ND, CONTROL_DT


class TrajRecorder:
    def __init__(self, meta: dict | None = None):
        self.meta = meta or {}
        self.t, self.state, self.comB, self.tank, self.u_cmd, self.valve, self.thrust = [], [], [], [], [], [], []
        self.contact, self.drive_cmd, self.drive = [], [], []

    def sample(self, tw, e: int = 0):
        P = tw.prop
        self.t.append(float(tw.k[e] * tw.dt))
        self.state.append(tw.S[:, e].tolist())
        self.comB.append(tw.C[:, e].tolist())
        self.tank.append([float(P.m[e]), float(P.T[e]), float(P.p[e])])
        self.valve.append(P.valve[e].tolist())
        self.thrust.append(P.thrust[e].tolist())
        self.u_cmd.append([0.0] * NU)
        self.drive_cmd.append([0.0] * ND)
        self.drive.append([float(tw.drive_brake[e]), float(tw.drive_steer[e])])
        self.contact.append(0 if tw.k[e] == 0 else (int(tw.f_any[e]) | 2 * int(tw.f_street[e]) | 4 * int(tw.f_deck[e])
                                                    | 8 * int(tw.f_other[e]) | 16 * int(tw.f_hull[e]) | 32 * int(tw.f_kerb[e])))

    def applied(self, u, ud=None):
        """Commands applied in the step after the last sample: valves (tw.u[e]), drive (tw.ud[e], optional)."""
        self.u_cmd[-1] = [float(x) for x in u]
        if ud is not None:
            self.drive_cmd[-1] = [float(x) for x in ud]

    def finish(self, events=(), metrics=None, spec_hash=None) -> dict:
        m = self.meta
        return {'format': TRAJ_FORMAT, 'spec_hash': spec_hash, 'model': MODEL, 'producer': 'python', 'task': m.get('task'),
                'controller': m.get('controller'), 'seed': m.get('seed'), 'preset': m.get('preset'), 'init': m.get('init'),
                'dt': CONTROL_DT, 'n': len(self.t), 't': self.t, 'state': self.state, 'comB': self.comB, 'tank': self.tank,
                'u_cmd': self.u_cmd, 'valve': self.valve, 'thrust': self.thrust, 'contact': self.contact,
                'drive_cmd': self.drive_cmd, 'drive': self.drive, 'events': list(events), 'metrics': metrics or {}}


def write_traj(path, traj: dict):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(traj, f, separators=(',', ':'))


def read_traj(path) -> dict:
    with open(path, encoding='utf-8') as f:
        t = json.load(f)
    if t.get('format') != TRAJ_FORMAT:
        raise ValueError(f'{path}: not {TRAJ_FORMAT}')
    return t

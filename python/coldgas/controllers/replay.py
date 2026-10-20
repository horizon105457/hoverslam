import json

import numpy as np

from ..contract import NU, CONTROL_DT, TRAJ_FORMAT


class Replay:
    """Plays u_cmd of a coldgas-traj@1 trajectory: row round(t / dt) at time t, zeros after the end."""
    name = 'replay'

    def __init__(self, traj=None, file=None, **_):
        if traj is None and file is not None:
            with open(file, encoding='utf-8') as f:
                traj = json.load(f)
        if not traj or traj.get('format') != TRAJ_FORMAT:
            raise ValueError(f'replay: need a {TRAJ_FORMAT} trajectory (traj= or file=)')
        self.rows = traj['u_cmd']
        self.dt = traj.get('dt') or CONTROL_DT
        self.config = {'source': traj.get('controller'), 'seed': traj.get('seed'), 'task': traj.get('task')}

    def reset(self, task=None, seed=0):
        pass

    def act(self, state, info=None):
        k = round(float(state['t']) / self.dt)  # JS Math.round; t/dt is never a half-integer here
        u = np.zeros(NU)
        if 0 <= k < len(self.rows):
            u[:] = self.rows[k]
        return u

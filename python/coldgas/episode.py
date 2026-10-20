"""One closed-loop episode on the Python twin, the same loop as src/flight/run.js runEpisode (parity)."""
from __future__ import annotations

from .controllers import create_controller
from .env import ColdGasEnv
from .spec import spec_hash
from .traj import TrajRecorder


def run_episode(task='escape-soft-landing', controller='zero', seed: int = 0, preset=None, init=None, record=True,
                spec_hash_value: str | None = None):
    env = ColdGasEnv(task, seed=seed, preset=preset)
    ctrl = controller if hasattr(controller, 'act') else create_controller(controller)
    env.reset(seed, {'init': init} if init else {})
    ctrl.reset(task=env.task, seed=seed)
    cfg = {'name': ctrl.name, **{k: v for k, v in getattr(ctrl, 'config', {}).items() if k not in ('traj', 'policy')}} \
        if hasattr(controller, 'act') else ({'name': controller} if isinstance(controller, str) else
                                            {k: ('<embedded>' if k in ('traj', 'policy') else v) for k, v in controller.items()})
    rec = TrajRecorder({'task': env.task['id'], 'controller': cfg, 'seed': seed, 'preset': env.init.get('preset'), 'init': env.init}) if record else None
    info, done, metrics = env._info(), False, None
    while not done:
        s = env.state
        if rec:
            rec.sample(env.twin)
        u = ctrl.act(s, info)
        _, _, term, trunc, info = env.step(u)
        if rec:
            rec.applied(env.twin.u[0])
        done = term or trunc
        if done:
            metrics = info['metrics']
    if rec:
        rec.sample(env.twin)
    events = env.twin.events[0]
    traj = rec.finish(events, metrics, spec_hash_value if spec_hash_value is not None else spec_hash()) if rec else None
    return {'metrics': metrics, 'events': events, 'traj': traj}

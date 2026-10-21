"""JS <-> Python parity harness (tests/parity/). Scenarios: tests/parity/scenarios.json.

    python -m coldgas.parity make  [dir]   write Python goldens (default tests/parity/goldens/py/*.json.gz)
    python -m coldgas.parity check [dir]   re-run the JS goldens (default tests/parity/goldens/js) and report
                                          max abs / rel deviation per state component, contact-free vs contact rows
    python -m coldgas.parity compare a.json b.json   the same report for any two coldgas-traj@1 files
The JS side (tests/parity/make.mjs, check.mjs) is the mirror image.
"""
from __future__ import annotations

import gzip
import json
import math
import sys
from pathlib import Path

import numpy as np

from .contract import NU, NA, CONTROL_DT, DRIVE_IDX
from .controllers import create_controller
from .episode import run_episode
from .rng import Mulberry32
from .spec import ROOT, spec_hash
from .traj import TrajRecorder
from .twin import Twin

PARITY = ROOT / 'tests' / 'parity'
# relative tolerance (deviation / max |golden component| over the trajectory); see spec/README.md section 5.9
TOL = {'free': 1e-9, 'contact': 1e-6}


class RandomValves:
    """Scenario controller: mulberry32(seed); per control step and nozzle a = r(); u = a < p_on ? r() : 0."""
    name = 'random'

    def __init__(self, seed=1, p_on=0.5, **_):
        self.seed, self.p_on = seed, p_on
        self.config = {'seed': seed, 'p_on': p_on}

    def reset(self, task=None, seed=0):
        self.r = Mulberry32(self.seed)

    def act(self, state, info=None):
        u = np.zeros(NU)
        for i in range(NU):
            a = self.r()
            u[i] = self.r() if a < self.p_on else 0.0
        return u


class WithDrive:
    """Scenario drive overlay (spec/README.md 5.13; lib.mjs withDrive): drive = [[t_start, brake, steer], ...],
    piecewise constant from each t_start on, written into action entries NU, NU + 1 on top of the controller's action
    (before the first t_start the controller's own action is passed through)."""

    def __init__(self, ctrl, drive):
        self.ctrl, self.drive = ctrl, drive
        self.name, self.config = getattr(ctrl, 'name', 'controller'), getattr(ctrl, 'config', {})

    def reset(self, task=None, seed=0):
        self.ctrl.reset(task=task, seed=seed)

    def act(self, state, info=None):
        u = np.asarray(self.ctrl.act(state, info), dtype=np.float64).reshape(-1)
        seg = None
        for d in self.drive:
            if float(state['t']) >= d[0] - 1e-9:
                seg = d
        if seg is None:
            return u
        out = np.zeros(NA)
        n = min(len(u), NA)
        out[:n] = u[:n]
        out[DRIVE_IDX['brake']], out[DRIVE_IDX['steer']] = seg[1], seg[2]
        return out


def scenarios() -> list[dict]:
    with open(PARITY / 'scenarios.json', encoding='utf-8') as f:
        return json.load(f)['scenarios']


def _controller(sc, golden=None):
    c = sc['controller']
    if isinstance(c, dict) and c.get('name') == 'random':  # re-runs replay the golden's u_cmd and drive_cmd
        ctrl = create_controller({'name': 'replay', 'traj': golden}) if golden else RandomValves(**c)
    else:
        ctrl = create_controller(c)
    return WithDrive(ctrl, sc['drive']) if sc.get('drive') else ctrl


def run_scenario(sc: dict, golden: dict | None = None) -> dict:
    ctrl = _controller(sc, golden)
    if sc.get('task'):
        r = run_episode(sc['task'], ctrl, seed=sc.get('seed', 0), preset=sc.get('preset'), vehicle=sc.get('vehicle'))
        traj = r['traj']
    else:
        tw = Twin(1, arm='start', vehicle=sc.get('vehicle'))
        tw.reset([sc['init']])
        ctrl.reset(task=None, seed=sc.get('seed', 0))
        rec = TrajRecorder({'task': None, 'controller': sc['controller'], 'seed': sc.get('seed'), 'init': sc['init']})
        for _ in range(round(sc['duration'] / CONTROL_DT)):
            rec.sample(tw)
            tw.step(np.asarray(ctrl.act(tw.state(0))).reshape(1, -1))
            rec.applied(tw.u[0], tw.ud[0])
        rec.sample(tw)
        traj = rec.finish(tw.events[0], {}, spec_hash())
    traj['scenario'] = sc
    return traj


def make(out: Path = PARITY / 'goldens' / 'py'):
    out.mkdir(parents=True, exist_ok=True)
    for sc in scenarios():
        traj = run_scenario(sc)
        with gzip.open(out / f"{sc['id']}.json.gz", 'wt', encoding='utf-8') as f:
            json.dump(traj, f, separators=(',', ':'))
        print(f"wrote {sc['id']}: {traj['n']} rows, events {[e['type'] for e in traj['events']]}")


GROUPS = {'p': ('state', 0, 3), 'q': ('state', 3, 7), 'v': ('state', 7, 10), 'w': ('state', 10, 13), 'comB': ('comB', 0, 3),
          'tank_m': ('tank', 0, 1), 'tank_T': ('tank', 1, 2), 'tank_p': ('tank', 2, 3), 'valve': ('valve', 0, NU),
          'thrust': ('thrust', 0, NU), 'u_cmd': ('u_cmd', 0, NU), 'drive': ('drive', 0, 2), 'drive_cmd': ('drive_cmd', 0, 2)}


def compare(golden: dict, mine: dict) -> dict:
    """Per group: max abs and max rel deviation on contact-free rows and on contact rows (+ structural checks)."""
    # events: type, t and every payload field (wheel mask, hull point, overloaded wheel / quantity), keys sorted
    evs = lambda t: [(e['type'], e['t'], sorted((k, v) for k, v in e.items() if k not in ('type', 't'))) for e in t['events']]
    rep = {'rows': [golden['n'], mine['n']], 'groups': {}, 'events_equal': evs(golden) == evs(mine), 'missing': []}
    n = min(golden['n'], mine['n'])
    contact = np.array(golden.get('contact', [0] * golden['n'])[:n]) != 0
    for g, (key, a, b) in GROUPS.items():
        if key not in golden or key not in mine:  # optional columns (drive: 5.13): both or neither
            rep['groups'][g] = {'free': {'abs': 0.0, 'rel': 0.0}, 'contact': {'abs': 0.0, 'rel': 0.0}}
            if (key in golden) != (key in mine):
                rep['missing'].append(g)
            continue
        G, Mn = np.array(golden[key])[:n, a:b], np.array(mine[key])[:n, a:b]
        scale = np.maximum(np.abs(G).max(axis=0), 1e-300)
        d = np.abs(G - Mn)
        rel = d / scale
        out = {}
        for name, mask in (('free', ~contact), ('contact', contact)):
            out[name] = {'abs': float(d[mask].max()) if mask.any() else 0.0, 'rel': float(rel[mask].max()) if mask.any() else 0.0}
        rep['groups'][g] = out
    mg, mm = golden.get('metrics') or {}, mine.get('metrics') or {}
    md = 0.0
    for k, v in mg.items():
        w = mm.get(k)
        if isinstance(v, (int, float)) and not isinstance(v, bool) and isinstance(w, (int, float)):
            md = max(md, abs(v - w) / max(abs(v), 1e-12))
        elif v != w and not (isinstance(v, list) and isinstance(w, list) and np.allclose(v, w, rtol=1e-9, atol=0)):
            md = math.inf
    rep['metrics_rel'] = md
    rep['ok'] = (rep['rows'][0] == rep['rows'][1] and rep['events_equal'] and not rep['missing'] and md <= TOL['contact']
                 and all(v['free']['rel'] <= TOL['free'] and v['contact']['rel'] <= TOL['contact'] for v in rep['groups'].values()))
    return rep


def check(src: Path = PARITY / 'goldens' / 'js', verbose=True) -> dict:
    reports = {}
    for p in sorted(src.glob('*.json.gz')):
        with gzip.open(p, 'rt', encoding='utf-8') as f:
            golden = json.load(f)
        mine = run_scenario(golden['scenario'], golden)
        reports[golden['scenario']['id']] = compare(golden, mine)
    if verbose:
        print_report(reports, f'Python re-run of {src.relative_to(ROOT) if src.resolve().is_relative_to(ROOT) else src}')
    return reports


def print_report(reports: dict, title: str):
    print(title)
    print(f"{'scenario':24s} ok   rows  events  " + '  '.join(f'{g:>15s}' for g in GROUPS))
    worst = {g: {'free': [0, 0], 'contact': [0, 0]} for g in GROUPS}
    for sid, r in reports.items():
        cells = []
        for g in GROUPS:
            f, c = r['groups'][g]['free']['rel'], r['groups'][g]['contact']['rel']
            cells.append(f'{f:7.1e}/{c:7.1e}')
            for name in ('free', 'contact'):
                worst[g][name][0] = max(worst[g][name][0], r['groups'][g][name]['abs'])
                worst[g][name][1] = max(worst[g][name][1], r['groups'][g][name]['rel'])
        print(f"{sid:24s} {'OK ' if r['ok'] else 'BAD'} {r['rows'][0]:5d}  {str(r['events_equal']):6s}  " + '  '.join(cells))
    print('max abs / max rel per component (contact-free rows | contact rows):')
    for g, w in worst.items():
        print(f"  {g:7s} free abs {w['free'][0]:.3e} rel {w['free'][1]:.3e} | contact abs {w['contact'][0]:.3e} rel {w['contact'][1]:.3e}")
    print('ALL OK' if all(r['ok'] for r in reports.values()) else 'PARITY FAILURES')


def _load_any(path):
    p = Path(path)
    if p.suffix == '.gz':
        with gzip.open(p, 'rt', encoding='utf-8') as f:
            return json.load(f)
    with open(p, encoding='utf-8') as f:
        return json.load(f)


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'check'
    arg = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    if cmd == 'compare':  # python -m coldgas.parity compare a.json b.json  (any two coldgas-traj@1 files)
        rep = compare(_load_any(sys.argv[2]), _load_any(sys.argv[3]))
        print_report({f'{Path(sys.argv[2]).name} vs {Path(sys.argv[3]).name}': rep}, 'trajectory comparison')
        sys.exit(0 if rep['ok'] else 1)
    if cmd == 'make':
        make(arg or PARITY / 'goldens' / 'py')
    else:
        reps = check(arg or PARITY / 'goldens' / 'js')
        sys.exit(0 if reps and all(r['ok'] for r in reps.values()) else 1)

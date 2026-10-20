"""Benchmark runner: tasks x controllers x presets x seeds -> experiments/runs/<name>/<stamp>/.

    python -m coldgas.bench experiments/configs/baseline-soft-landing.json [--workers 6] [--out DIR] [--seeds 0:3]

Config (JSON; schema in experiments/README.md):
    {"name": "...", "task": "escape-soft-landing", "controllers": [{"name": "pd", "label": "pd"}, ...],
     "presets": ["flat", ...] | null, "seeds": [0, 1] | {"start": 0, "count": 3}, "record": "all"|"first"|"none",
     "workers": 4}
Writes config.json (with spec hash), episodes.jsonl (one line per episode), summary.json (per controller and per
controller x preset) and traj/<label>__<preset>__s<seed>.json (coldgas-traj@1), and prints a markdown table.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path

from .spec import ROOT, spec_hash


def _seeds(spec):
    if isinstance(spec, dict):
        return list(range(spec.get('start', 0), spec.get('start', 0) + spec['count']))
    return list(spec)


def _job(job):
    from .episode import run_episode
    from .traj import write_traj
    t0 = time.time()
    r = run_episode(job['task'], job['controller'], seed=job['seed'], preset=job['preset'], record=True,
                    spec_hash_value=job['spec_hash'])  # always recorded in memory (cheap): the drawn preset lives there
    wall = time.time() - t0
    if job['traj']:
        r['traj']['controller'] = job['controller_cfg']
        write_traj(job['traj'], r['traj'])
    preset = r['traj']['preset']
    return {'label': job['label'], 'controller': job['controller_cfg'], 'task': job['task'], 'preset': preset, 'seed': job['seed'],
            'metrics': r['metrics'], 'wall_s': wall, 'traj': os.path.relpath(job['traj'], job['out']) if job['traj'] else None}


def _resolve_controller(cfg, base: Path):
    c = dict(cfg)
    for key in ('policy', 'traj'):
        if isinstance(c.get(key), str):  # path to a policy / trajectory file, relative to the config or repo
            p = Path(c[key])
            p = p if p.is_absolute() else (base / p if (base / p).exists() else ROOT / p)
            with open(p, encoding='utf-8') as f:
                c[key] = json.load(f)
    return c


def run(config_path, workers=None, out=None, seeds=None, quiet=False):
    config_path = Path(config_path)
    with open(config_path, encoding='utf-8') as f:
        cfg = json.load(f)
    name = cfg.get('name', config_path.stem)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    out = Path(out) if out else ROOT / 'experiments' / 'runs' / name / stamp
    (out / 'traj').mkdir(parents=True, exist_ok=True)
    h = spec_hash()
    seeds = _seeds(seeds if seeds is not None else cfg.get('seeds', [0]))
    presets = cfg.get('presets') or [None]
    record = cfg.get('record', 'all')
    with open(out / 'config.json', 'w', encoding='utf-8') as f:
        json.dump({**cfg, 'spec_hash': h, 'seeds_resolved': seeds, 'stamp': stamp}, f, indent=1)
    jobs = []
    for ci, ccfg in enumerate(cfg['controllers']):
        label = ccfg.get('label', ccfg['name'])
        cc = {k: v for k, v in ccfg.items() if k != 'label'}
        resolved = _resolve_controller(cc, config_path.parent)
        shown = {k: ('<file>' if k in ('policy', 'traj') else v) for k, v in cc.items()}
        for pi, preset in enumerate(presets):
            for si, seed in enumerate(seeds):
                keep = record == 'all' or (record == 'first' and si == 0)
                tf = out / 'traj' / f"{label}__{preset or 'task'}__s{seed}.json" if keep else None
                jobs.append({'task': cfg['task'], 'controller': resolved, 'controller_cfg': shown, 'label': label, 'preset': preset,
                             'seed': seed, 'traj': str(tf) if tf else None, 'spec_hash': h, 'out': str(out)})
    workers = workers or cfg.get('workers') or max(1, (os.cpu_count() or 2) - 1)
    t0 = time.time()
    rows = []
    with open(out / 'episodes.jsonl', 'w', encoding='utf-8') as fj:
        if workers == 1:
            it = map(_job, jobs)
            for r in it:
                rows.append(r)
                fj.write(json.dumps(r) + '\n')
                fj.flush()
        else:
            with ProcessPoolExecutor(max_workers=workers) as ex:
                futs = [ex.submit(_job, j) for j in jobs]
                for k, fu in enumerate(as_completed(futs)):
                    r = fu.result()
                    rows.append(r)
                    fj.write(json.dumps(r) + '\n')
                    fj.flush()
                    if not quiet:
                        print(f"\r{k + 1}/{len(jobs)} episodes", end='', file=sys.stderr)
    if not quiet:
        print(file=sys.stderr)
    summary = summarize(rows, [c.get('label', c['name']) for c in cfg['controllers']])
    summary.update({'name': name, 'task': cfg['task'], 'spec_hash': h, 'episodes': len(rows), 'wall_s': time.time() - t0, 'stamp': stamp})
    with open(out / 'summary.json', 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1)
    if not quiet:
        print(table(summary))
        print(f'\n-> {out.relative_to(ROOT) if out.is_relative_to(ROOT) else out}')
    return out, summary


_KEYS = ('touchdown_vertical_speed', 'touchdown_horizontal_speed', 'touchdown_tilt_deg', 'touchdown_angular_rate', 'peak_accel_g',
         'gas_used_kg', 'pad_distance', 'hold_time', 'rms_height_error', 'return')


def _stats(xs):
    xs = [x for x in xs if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)]
    if not xs:
        return None
    xs = sorted(xs)
    return {'mean': sum(xs) / len(xs), 'median': xs[len(xs) // 2], 'min': xs[0], 'max': xs[-1], 'n': len(xs)}


def summarize(rows, labels):
    out = {'controllers': {}}
    for lab in labels:
        rs = [r for r in rows if r['label'] == lab]
        if not rs:
            continue
        m = [r['metrics'] for r in rs]
        reasons = {}
        for x in m:
            reasons[x['reason']] = reasons.get(x['reason'], 0) + 1
        per = {}
        for r in rs:
            p = r['preset'] or 'task'
            per.setdefault(p, []).append(bool(r['metrics']['success']))
        out['controllers'][lab] = {
            'n': len(rs), 'success_rate': sum(bool(x['success']) for x in m) / len(m),
            'hull_strike_rate': sum(bool(x.get('hull_strike')) for x in m) / len(m), 'reasons': reasons,
            'metrics': {k: _stats([x.get(k) for x in m]) for k in _KEYS if any(x.get(k) is not None for x in m)},
            'per_preset_success': {p: sum(v) / len(v) for p, v in sorted(per.items())},
            'wall_s_per_episode': sum(r['wall_s'] for r in rs) / len(rs),
        }
    return out


def table(summary) -> str:
    C = summary['controllers']
    presets = sorted({p for c in C.values() for p in c['per_preset_success']})
    head = ['controller', 'n', 'success', 'hull strike', 'vy_td mean', 'tilt_td mean', 'gas kg', 'return', *presets, 's/ep']
    lines = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    f = lambda s, k: (f"{s['metrics'][k]['mean']:.2f}" if s['metrics'].get(k) else '-')
    for lab, s in C.items():
        cells = [lab, str(s['n']), f"{100 * s['success_rate']:.0f}%", f"{100 * s['hull_strike_rate']:.0f}%", f(s, 'touchdown_vertical_speed'),
                 f(s, 'touchdown_tilt_deg'), f(s, 'gas_used_kg'), f(s, 'return'),
                 *[f"{100 * s['per_preset_success'].get(p, float('nan')):.0f}%" for p in presets], f"{s['wall_s_per_episode']:.1f}"]
        lines.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python -m coldgas.bench', description=__doc__.split('\n')[0])
    ap.add_argument('config')
    ap.add_argument('--workers', type=int)
    ap.add_argument('--out')
    ap.add_argument('--seeds', help='a:b (range) or comma list; overrides the config')
    a = ap.parse_args(argv)
    seeds = None
    if a.seeds:
        seeds = list(range(*map(int, a.seeds.split(':')))) if ':' in a.seeds else [int(x) for x in a.seeds.split(',')]
    run(a.config, a.workers, a.out, seeds)


if __name__ == '__main__':
    main()

import json

from coldgas import bench
from coldgas.traj import read_traj


def test_bench_runner_end_to_end(tmp_path):
    cfg = {'name': 'tiny', 'task': 'escape-soft-landing', 'controllers': [{'name': 'zero'}, {'name': 'pd', 'label': 'pd-default'}],
           'presets': ['side'], 'seeds': [1, 2], 'record': 'first'}
    p = tmp_path / 'tiny.json'
    p.write_text(json.dumps(cfg))
    out, summary = bench.run(p, workers=1, out=tmp_path / 'run', quiet=True)
    rows = [json.loads(l) for l in (out / 'episodes.jsonl').read_text().splitlines()]
    assert len(rows) == 4 and {r['label'] for r in rows} == {'zero', 'pd-default'}
    assert summary['controllers']['zero']['success_rate'] == 0.0 and summary['controllers']['pd-default']['n'] == 2
    trajs = sorted((out / 'traj').glob('*.json'))
    assert len(trajs) == 2
    t = read_traj(trajs[0])
    assert t['spec_hash'] == summary['spec_hash'] and t['n'] == len(t['state']) and t['preset'] == 'side'
    assert '| pd-default |' in bench.table(summary)

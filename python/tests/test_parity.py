"""JS <-> Python parity on tests/parity/scenarios.json (both golden sets)."""
import subprocess

from coldgas import parity
from coldgas.spec import ROOT
from conftest import NODE
import pytest


def test_python_reproduces_js_goldens():
    reps = parity.check(parity.PARITY / 'goldens' / 'js', verbose=False)
    assert len(reps) == len(parity.scenarios())
    bad = {k: v for k, v in reps.items() if not v['ok']}
    assert not bad, bad


@pytest.mark.skipif(not NODE, reason='node not available')
def test_js_reproduces_python_goldens():
    r = subprocess.run([NODE, 'tests/parity/check.mjs'], cwd=ROOT, capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr

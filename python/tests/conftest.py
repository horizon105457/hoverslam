import json
import shutil
import subprocess

import pytest

from coldgas.spec import ROOT

NODE = shutil.which('node')


def node_json(code: str):
    """Run an ES module snippet in node from the repo root; it must print one JSON document."""
    if not NODE:
        pytest.skip('node not available')
    r = subprocess.run([NODE, '--input-type=module', '-e', code], cwd=ROOT, capture_output=True, text=True, timeout=600)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    return json.loads(r.stdout)


@pytest.fixture
def node():
    return node_json

"""The language-neutral spec (spec/*.json) as Python objects: the Python view of src/core/spec.js.

This is the only place Python reads spec files. The repository root is found relative to this package
(python/coldgas/spec.py -> ../../), so the package works from any working directory. Never copy spec numbers
into code. Also implements the spec hash of spec/README.md section 7 (mirrored by src/flight/spechash.js).
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC_DIR = ROOT / 'spec'


def _load(rel: str) -> dict:
    with open(SPEC_DIR / rel, encoding='utf-8') as f:
        return json.load(f)


WORLD = _load('world.json')
LAUNCH = _load('launch-presets.json')
PROPULSION = _load('propulsion.json')
VEHICLE = _load('vehicle.json')
#: brakes and steering: two action channels after the nozzles (spec/README.md 5.13)
DRIVE = _load('drive.json')
GENERATED = {'massprops': _load('generated/vehicle-massprops.json'), 'colliders': _load('generated/colliders.json')}
#: raw task specs by id (``extends`` is NOT resolved here; see coldgas.tasks)
TASKS = {t['id']: t for t in (_load(f'tasks/{p.name}') for p in sorted((SPEC_DIR / 'tasks').glob('*.json')))}
#: nozzle ids in action-vector order
NOZZLE_IDS = [n['id'] for n in PROPULSION['nozzles']]

#: twin structure constants (spec/vehicle.json `twin`; see spec/README.md section 5)
MODEL = VEHICLE['twin']

_FNV_OFFSET, _FNV_PRIME, _MASK = 0xCBF29CE484222325, 0x100000001B3, (1 << 64) - 1


def fnv1a64(chunks) -> int:
    h = _FNV_OFFSET
    for b in chunks:
        for byte in b:
            h = ((h ^ byte) * _FNV_PRIME) & _MASK
    return h


def spec_hash(spec_dir: Path | str = SPEC_DIR) -> str:
    """FNV-1a 64 over every *.json under spec/ sorted by relative path ('/' separators): path, 0x00, bytes, 0x00."""
    spec_dir = Path(spec_dir)
    files = sorted((p.relative_to(spec_dir).as_posix(), p) for p in spec_dir.rglob('*.json'))
    chunks = []
    for rel, p in files:
        chunks += [rel.encode('utf-8'), b'\x00', p.read_bytes(), b'\x00']
    return format(fnv1a64(chunks), '016x')

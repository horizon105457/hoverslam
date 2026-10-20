"""Static world of the flight twins (mirror of src/flight/world.js): ground plane + classified oriented boxes."""
from __future__ import annotations

import math

import numpy as np

from . import fmath
from .spec import GENERATED
from .model import CONTACT

KIND = {'street': 0, 'deck': 1, 'other': 2}
GROUND = {'y': GENERATED['colliders']['ground']['y'], 'mu': GENERATED['colliders']['ground']['mu'], 'kind': 0}


def _box(b):
    cy, sy = fmath.cos(b['yaw']), fmath.sin(b['yaw'])
    top = b['c'][1] + b['e'][1]
    kind = KIND['deck'] if b['name'] == 'deck' else (KIND['street'] if top < CONTACT['street_top_max'] else KIND['other'])
    rx = abs(cy) * b['e'][0] + abs(sy) * b['e'][2]
    rz = abs(sy) * b['e'][0] + abs(cy) * b['e'][2]
    return {'name': b['name'], 'c': list(map(float, b['c'])), 'e': list(map(float, b['e'])), 'mu': float(b['mu']),
            'cy': cy, 'sy': sy, 'top': top, 'kind': kind, 'rx': rx, 'rz': rz}


BOXES = [_box(b) for b in GENERATED['colliders']['boxes']]


def street_level(x, z):
    """Street height under (x, z); x, z floats or arrays (same ops either way)."""
    scalar = np.isscalar(x)
    x, z = np.asarray(x, dtype=np.float64), np.asarray(z, dtype=np.float64)
    y = np.full(np.broadcast(x, z).shape, float(GROUND['y']))
    for b in BOXES:
        if b['kind'] != KIND['street']:
            continue
        dx, dz = x - b['c'][0], z - b['c'][2]
        lx, lz = b['cy'] * dx - b['sy'] * dz, b['sy'] * dx + b['cy'] * dz
        y = np.where((np.abs(lx) < b['e'][0]) & (np.abs(lz) < b['e'][2]) & (b['top'] > y), b['top'], y)
    return float(y) if scalar else y


def support_level(x, z, y_max=math.inf):
    scalar = np.isscalar(x)
    x, z = np.asarray(x, dtype=np.float64), np.asarray(z, dtype=np.float64)
    y = np.full(np.broadcast(x, z).shape, float(GROUND['y']))
    for b in BOXES:
        dx, dz = x - b['c'][0], z - b['c'][2]
        lx, lz = b['cy'] * dx - b['sy'] * dz, b['sy'] * dx + b['cy'] * dz
        y = np.where((np.abs(lx) < b['e'][0]) & (np.abs(lz) < b['e'][2]) & (b['top'] > y) & (b['top'] <= y_max), b['top'], y)
    return float(y) if scalar else y

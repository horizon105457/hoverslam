"""Python mirror of src/flight/contract.js (state layout, action size, formats). See spec/README.md 5, 7, 8."""
from __future__ import annotations

from .spec import PROPULSION, NOZZLE_IDS, VEHICLE
from .model import MODEL_ID

CONTRACT_VERSION = 'coldgas-contract@1'
NU = len(PROPULSION['nozzles'])
GROUPS = {g: [i for i, n in enumerate(PROPULSION['nozzles']) if n['group'] == g] for g in ('bottom', 'rear', 'side')}
CONTROL_DT = PROPULSION['control']['dt']
DT = VEHICLE['integrator']['dt']
#: integrator steps per control step
SUBSTEPS = round(CONTROL_DT / DT)
STATE = {'p': 0, 'q': 3, 'v': 7, 'w': 10, 'size': 13}
TRAJ_FORMAT = 'coldgas-traj@1'
POLICY_FORMAT = 'coldgas-policy@1'
MODEL = MODEL_ID
PHASES = ('deck', 'air', 'ground')
__all__ = ['CONTRACT_VERSION', 'NU', 'NOZZLE_IDS', 'GROUPS', 'CONTROL_DT', 'DT', 'SUBSTEPS', 'STATE', 'TRAJ_FORMAT',
           'POLICY_FORMAT', 'MODEL', 'PHASES']

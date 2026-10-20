"""coldgas: Python twin of the 坠 · FALL cold-gas thruster car (mirror of src/flight, spec in spec/README.md).

    from coldgas import ColdGasEnv, BatchColdGasEnv, Twin, create_controller, load_task, run_episode, spec_hash
"""
from .spec import spec_hash, ROOT
from .tasks import load_task, sample_initial
from .twin import Twin
from .env import ColdGasEnv, register
from .vecenv import BatchColdGasEnv
from .controllers import create_controller
from .episode import run_episode

__all__ = ['spec_hash', 'ROOT', 'load_task', 'sample_initial', 'Twin', 'ColdGasEnv', 'register', 'BatchColdGasEnv',
           'create_controller', 'run_episode']

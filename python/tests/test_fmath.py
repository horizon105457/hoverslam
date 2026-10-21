import struct

import numpy as np

from coldgas import fmath
from coldgas.rng import Mulberry32


def bits(x):
    return struct.unpack('<q', struct.pack('<d', float(x)))[0]


def test_ports_match_js_bit_for_bit(node):
    js = node("""
import * as fm from './src/flight/fmath.js';
import { mulberry32 } from './src/flight/rng.js';
const r = mulberry32(99), out = [];
for (let i = 0; i < 4000; i++) { const a = (r() - 0.5) * 80, b = (r() - 0.5) * 40, c = r() * 2 - 1;
  out.push([a, b, c, fm.exp(a), fm.sin(b), fm.cos(b), fm.acos(c), fm.tanh(b / 4)]); }
console.log(JSON.stringify(out));""")
    A = np.array(js)
    for a, b, c, e, s, co, ac, th in js:
        assert bits(fmath.exp(a)) == bits(e)
        assert bits(fmath.sin(b)) == bits(s) and bits(fmath.cos(b)) == bits(co)
        assert bits(fmath.acos(c)) == bits(ac) and bits(fmath.tanh(b / 4)) == bits(th)
    assert (fmath.exp_np(A[:, 0]) == A[:, 3]).all()
    assert (fmath.acos_np(A[:, 2]) == A[:, 6]).all()
    assert (fmath.tanh_np(A[:, 1] / 4) == A[:, 7]).all()
    assert (fmath.sin_np(A[:, 1]) == A[:, 4]).all() and (fmath.cos_np(A[:, 1]) == A[:, 5]).all()  # incl. |x| > pi/4


def test_vectorised_sin_cos_small_angles():
    """sin_np / cos_np (steer angles, |x| <= pi/4 on the kernel path) equal the scalar ports bit for bit."""
    r = Mulberry32(7)
    x = np.array([(r() - 0.5) * 1.6 for _ in range(20000)] + [0.0, -0.0, 1e-9, -3e-9, 0.3, 0.78125, 0.7853981633974483, 0.79])
    bs = np.array([bits(fmath.sin(v)) for v in x])
    bc = np.array([bits(fmath.cos(v)) for v in x])
    assert (fmath.sin_np(x).view(np.int64) == bs).all() and (fmath.cos_np(x).view(np.int64) == bc).all()


def test_ports_are_accurate():
    r = Mulberry32(5)
    for _ in range(2000):
        x = (r() - 0.5) * 30
        assert abs(fmath.sin(x) - np.sin(x)) <= 2.3e-16 and abs(fmath.cos(x) - np.cos(x)) <= 2.3e-16
        assert abs(fmath.exp(x) / np.exp(x) - 1) <= 2.3e-16
        c = r() * 2 - 1
        assert abs(fmath.acos(c) - np.arccos(c)) <= 4.5e-16

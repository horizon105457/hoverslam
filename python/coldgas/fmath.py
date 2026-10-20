"""Bit-exact ports of the fdlibm functions V8 uses for Math.exp / Math.sin / Math.cos / Math.acos.

CPython and numpy call the platform libm, which disagrees with V8 in the last bit for a few percent of inputs
(measured on macOS: exp 9 %, sin/cos 3.5 %, acos 18 %; pow and sqrt agree everywhere). The flight twins must agree
to rounding, so every transcendental call on the Python side whose result can reach the dynamics goes through
these ports (scalar and numpy-vectorised); JS keeps using Math.* (V8 = fdlibm). tests/test_fmath.py checks them
against node. Constants and branch structure follow fdlibm 5.3 (e_exp.c, k_sin.c, k_cos.c, e_rem_pio2.c,
e_acos.c) exactly; do not "simplify" the arithmetic.
"""
from __future__ import annotations

import math
import struct

import numpy as np

_pk, _uk = struct.Struct('<d').pack, struct.Struct('<q').unpack
_pq, _ud = struct.Struct('<q').pack, struct.Struct('<d').unpack


def _hi(x: float) -> int:
    """High 32 bits of x as a signed int32 (fdlibm __HI)."""
    h = (_uk(_pk(x))[0] >> 32) & 0xFFFFFFFF
    return h - (1 << 32) if h & 0x80000000 else h


def _lo(x: float) -> int:
    return _uk(_pk(x))[0] & 0xFFFFFFFF


def _with_lo0(x: float) -> float:
    return _ud(_pq(_uk(_pk(x))[0] & ~0xFFFFFFFF))[0]


def _from_hi(hi: int) -> float:
    return _ud(_pq((hi & 0xFFFFFFFF) << 32 if hi >= 0 else ((hi & 0xFFFFFFFF) << 32) - (1 << 64)))[0]


# ---------------------------------------------------------------------------------------------------- exp
_o_threshold = 7.09782712893383973096e+02
_u_threshold = -7.45133219101941108420e+02
_ln2HI = (6.93147180369123816490e-01, -6.93147180369123816490e-01)
_ln2LO = (1.90821492927058770002e-10, -1.90821492927058770002e-10)
_invln2 = 1.44269504088896338700e+00
_P1 = 1.66666666666666019037e-01
_P2 = -2.77777777770155933842e-03
_P3 = 6.61375632143793436117e-05
_P4 = -1.65339022054652515390e-06
_P5 = 4.13813679705723846039e-08


def exp(x: float) -> float:
    x = float(x)
    if x != x:
        return x
    hx = _hi(x)
    xsb = (hx >> 31) & 1
    hx &= 0x7FFFFFFF
    if hx >= 0x40862E42:
        if hx >= 0x7FF00000:
            return x if xsb == 0 else 0.0
        if x > _o_threshold:
            return math.inf
        if x < _u_threshold:
            return 0.0
    k = 0
    hi = lo = 0.0
    if hx > 0x3FD62E42:
        if hx < 0x3FF0A2B2:
            hi = x - _ln2HI[xsb]
            lo = _ln2LO[xsb]
            k = 1 - xsb - xsb
        else:
            k = int(_invln2 * x + (0.5, -0.5)[xsb])  # C (int) truncates toward zero, as int() does
            t = float(k)
            hi = x - t * _ln2HI[0]
            lo = t * _ln2LO[0]
        x = hi - lo
    elif hx < 0x3E300000:
        return 1.0 + x
    t = x * x
    c = x - t * (_P1 + t * (_P2 + t * (_P3 + t * (_P4 + t * _P5))))
    if k == 0:
        return 1.0 - ((x * c) / (c - 2.0) - x)
    y = 1.0 - ((lo - (x * c) / (2.0 - c)) - hi)
    return math.ldexp(y, k)  # exact scaling by 2^k (fdlibm adds k to the exponent)


def exp_np(x: np.ndarray) -> np.ndarray:
    """Vectorised exp(), identical to exp() element by element (finite inputs with |x| < 709)."""
    x = np.asarray(x, dtype=np.float64)
    hx = (x.view(np.int64) >> 32).astype(np.int64) & 0x7FFFFFFF
    neg = x < 0
    mid = (hx > 0x3FD62E42) & (hx < 0x3FF0A2B2)
    big = hx >= 0x3FF0A2B2
    tiny = hx < 0x3E300000
    with np.errstate(all='ignore'):
        kb = np.trunc(_invln2 * x + np.where(neg, -0.5, 0.5))
        k = np.where(mid, np.where(neg, -1.0, 1.0), np.where(big, kb, 0.0))
        hi = np.where(mid, x - np.where(neg, _ln2HI[1], _ln2HI[0]), x - k * _ln2HI[0])
        lo = np.where(mid, np.where(neg, _ln2LO[1], _ln2LO[0]), k * _ln2LO[0])
        red = mid | big
        xr = np.where(red, hi - lo, x)
        t = xr * xr
        c = xr - t * (_P1 + t * (_P2 + t * (_P3 + t * (_P4 + t * _P5))))
        y0 = 1.0 - ((xr * c) / (c - 2.0) - xr)
        y1 = np.ldexp(1.0 - ((lo - (xr * c) / (2.0 - c)) - hi), k.astype(np.int64))
        return np.where(tiny, 1.0 + x, np.where(red, y1, y0))


# ---------------------------------------------------------------------------------------------- sin / cos
_S1 = -1.66666666666666324348e-01
_S2 = 8.33333333332248946124e-03
_S3 = -1.98412698298579493134e-04
_S4 = 2.75573137070700676789e-06
_S5 = -2.50507602534068634195e-08
_S6 = 1.58969099521155010221e-10
_C1 = 4.16666666666666019037e-02
_C2 = -1.38888888888741095749e-03
_C3 = 2.48015872894767294178e-05
_C4 = -2.75573143513906633035e-07
_C5 = 2.08757232129817482790e-09
_C6 = -1.13596475577881948265e-11


def _ksin(x: float, y: float, iy: int) -> float:
    ix = _hi(x) & 0x7FFFFFFF
    if ix < 0x3E400000 and int(x) == 0:
        return x
    z = x * x
    v = z * x
    r = _S2 + z * (_S3 + z * (_S4 + z * (_S5 + z * _S6)))
    if iy == 0:
        return x + v * (_S1 + z * r)
    return x - ((z * (0.5 * y - v * r) - y) - v * _S1)


def _kcos(x: float, y: float) -> float:
    ix = _hi(x) & 0x7FFFFFFF
    if ix < 0x3E400000 and int(x) == 0:
        return 1.0
    z = x * x
    r = z * (_C1 + z * (_C2 + z * (_C3 + z * (_C4 + z * (_C5 + z * _C6)))))
    if ix < 0x3FD33333:
        return 1.0 - (0.5 * z - (z * r - x * y))
    qx = 0.28125 if ix > 0x3FE90000 else _from_hi(ix - 0x00200000)
    hz = 0.5 * z - qx
    a = 1.0 - qx
    return a - (hz - (z * r - x * y))


_invpio2 = 6.36619772367581382433e-01
_pio2_1 = 1.57079632673412561417e+00
_pio2_1t = 6.07710050650619224932e-11
_pio2_2 = 6.07710050630396597660e-11
_pio2_2t = 2.02226624879595063154e-21
_pio2_3 = 2.02226624871116645580e-21
_pio2_3t = 8.47842766036889956997e-32


def _rem_pio2(x: float) -> tuple[int, float, float]:
    """fdlibm __ieee754_rem_pio2 for |x| <= 2^19 pi/2 (the twins only need angles of a few turns)."""
    hx = _hi(x)
    ix = hx & 0x7FFFFFFF
    if ix < 0x4002D97C:  # |x| < 3pi/4: special case of n = +-1
        if hx > 0:
            z = x - _pio2_1
            if ix != 0x3FF921FB:
                y0 = z - _pio2_1t
                return 1, y0, (z - y0) - _pio2_1t
            z -= _pio2_2
            y0 = z - _pio2_2t
            return 1, y0, (z - y0) - _pio2_2t
        z = x + _pio2_1
        if ix != 0x3FF921FB:
            y0 = z + _pio2_1t
            return -1, y0, (z - y0) + _pio2_1t
        z += _pio2_2
        y0 = z + _pio2_2t
        return -1, y0, (z - y0) + _pio2_2t
    if ix > 0x413921FB:
        raise ValueError('fmath: argument too large for the medium-range reduction')
    t = abs(x)
    n = int(t * _invpio2 + 0.5)
    fn = float(n)
    r = t - fn * _pio2_1
    w = fn * _pio2_1t
    j = ix >> 20
    y0 = r - w
    i = j - ((_hi(y0) >> 20) & 0x7FF)
    if i > 16:  # 2nd iteration, good to 118 bits
        t = r
        w = fn * _pio2_2
        r = t - w
        w = fn * _pio2_2t - ((t - r) - w)
        y0 = r - w
        i = j - ((_hi(y0) >> 20) & 0x7FF)
        if i > 49:  # 3rd iteration, 151 bits
            t = r
            w = fn * _pio2_3
            r = t - w
            w = fn * _pio2_3t - ((t - r) - w)
            y0 = r - w
    y1 = (r - y0) - w
    if hx < 0:
        return -n, -y0, -y1
    return n, y0, y1


def sin(x: float) -> float:
    x = float(x)
    ix = _hi(x) & 0x7FFFFFFF
    if ix <= 0x3FE921FB:
        return _ksin(x, 0.0, 0)
    if ix >= 0x7FF00000:
        return x - x
    n, y0, y1 = _rem_pio2(x)
    n &= 3
    if n == 0:
        return _ksin(y0, y1, 1)
    if n == 1:
        return _kcos(y0, y1)
    if n == 2:
        return -_ksin(y0, y1, 1)
    return -_kcos(y0, y1)


def cos(x: float) -> float:
    x = float(x)
    ix = _hi(x) & 0x7FFFFFFF
    if ix <= 0x3FE921FB:
        return _kcos(x, 0.0)
    if ix >= 0x7FF00000:
        return x - x
    n, y0, y1 = _rem_pio2(x)
    n &= 3
    if n == 0:
        return _kcos(y0, y1)
    if n == 1:
        return -_ksin(y0, y1, 1)
    if n == 2:
        return -_kcos(y0, y1)
    return _ksin(y0, y1, 1)


# --------------------------------------------------------------------------------------------------- acos
_pi = 3.14159265358979311600e+00
_pio2_hi = 1.57079632679489655800e+00
_pio2_lo = 6.12323399573676603587e-17
_pS0 = 1.66666666666666657415e-01
_pS1 = -3.25565818622400915405e-01
_pS2 = 2.01212532134862925881e-01
_pS3 = -4.00555345006794114027e-02
_pS4 = 7.91534994289814532176e-04
_pS5 = 3.47933107596021167570e-05
_qS1 = -2.40339491173441421878e+00
_qS2 = 2.02094576023350569471e+00
_qS3 = -6.88283971605453293030e-01
_qS4 = 7.70381505559019352791e-02


def acos(x: float) -> float:
    x = float(x)
    hx = _hi(x)
    ix = hx & 0x7FFFFFFF
    if ix >= 0x3FF00000:
        if ((ix - 0x3FF00000) | _lo(x)) == 0:
            return 0.0 if hx > 0 else _pi + 2.0 * _pio2_lo
        return math.nan
    if ix < 0x3FE00000:
        if ix <= 0x3C600000:
            return _pio2_hi + _pio2_lo
        z = x * x
        p = z * (_pS0 + z * (_pS1 + z * (_pS2 + z * (_pS3 + z * (_pS4 + z * _pS5)))))
        q = 1.0 + z * (_qS1 + z * (_qS2 + z * (_qS3 + z * _qS4)))
        r = p / q
        return _pio2_hi - (x - (_pio2_lo - x * r))
    if hx < 0:
        z = (1.0 + x) * 0.5
        p = z * (_pS0 + z * (_pS1 + z * (_pS2 + z * (_pS3 + z * (_pS4 + z * _pS5)))))
        q = 1.0 + z * (_qS1 + z * (_qS2 + z * (_qS3 + z * _qS4)))
        s = math.sqrt(z)
        r = p / q
        w = r * s - _pio2_lo
        return _pi - 2.0 * (s + w)
    z = (1.0 - x) * 0.5
    s = math.sqrt(z)
    df = _with_lo0(s)
    c = (z - df * df) / (s + df)
    p = z * (_pS0 + z * (_pS1 + z * (_pS2 + z * (_pS3 + z * (_pS4 + z * _pS5)))))
    q = 1.0 + z * (_qS1 + z * (_qS2 + z * (_qS3 + z * _qS4)))
    r = p / q
    w = r * s + c
    return 2.0 * (df + w)


def acos_np(x: np.ndarray) -> np.ndarray:
    """Vectorised acos() for |x| <= 1, identical to acos() element by element."""
    x = np.asarray(x, dtype=np.float64)
    bits = x.view(np.int64)
    ix = (bits >> 32) & 0x7FFFFFFF
    with np.errstate(all='ignore'):
        # |x| < 0.5
        z = x * x
        p = z * (_pS0 + z * (_pS1 + z * (_pS2 + z * (_pS3 + z * (_pS4 + z * _pS5)))))
        q = 1.0 + z * (_qS1 + z * (_qS2 + z * (_qS3 + z * _qS4)))
        a_small = np.where(ix <= 0x3C600000, _pio2_hi + _pio2_lo, _pio2_hi - (x - (_pio2_lo - x * (p / q))))
        # x <= -0.5
        z = (1.0 + x) * 0.5
        p = z * (_pS0 + z * (_pS1 + z * (_pS2 + z * (_pS3 + z * (_pS4 + z * _pS5)))))
        q = 1.0 + z * (_qS1 + z * (_qS2 + z * (_qS3 + z * _qS4)))
        s = np.sqrt(z)
        a_neg = _pi - 2.0 * (s + ((p / q) * s - _pio2_lo))
        # x >= 0.5
        z = (1.0 - x) * 0.5
        s = np.sqrt(z)
        df = (s.view(np.int64) & ~np.int64(0xFFFFFFFF)).view(np.float64)
        c = (z - df * df) / (s + df)
        p = z * (_pS0 + z * (_pS1 + z * (_pS2 + z * (_pS3 + z * (_pS4 + z * _pS5)))))
        q = 1.0 + z * (_qS1 + z * (_qS2 + z * (_qS3 + z * _qS4)))
        a_pos = 2.0 * (df + ((p / q) * s + c))
        one = ix >= 0x3FF00000
        a_one = np.where(x > 0, 0.0, _pi + 2.0 * _pio2_lo)
        return np.where(one, a_one, np.where(ix < 0x3FE00000, a_small, np.where(x < 0, a_neg, a_pos)))


def tanh(x: float) -> float:
    """Activation for coldgas-policy@1 MLPs. NOT fdlibm tanh: defined from exp() so JS (tanh01 in mlp.js) and
    Python agree bit for bit: tanh(x) = sign(x) (1 - 2 / (exp(2|x|) + 1)), saturating at |x| > 20."""
    a = abs(x)
    if a > 20.0:
        return math.copysign(1.0, x)
    t = 1.0 - 2.0 / (exp(2.0 * a) + 1.0)
    return t if x >= 0 else -t


def tanh_np(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    a = np.abs(x)
    t = 1.0 - 2.0 / (exp_np(np.minimum(2.0 * a, 40.0)) + 1.0)
    t = np.where(a > 20.0, 1.0, t)
    return np.where(x >= 0, t, -t)

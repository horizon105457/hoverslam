"""Physical sanity of the twin (numbers reported by python -m coldgas.sanity)."""
import math

from coldgas import sanity


def test_free_fall_from_deck():
    r = sanity.free_fall()
    assert abs(r['impact_t'] - r['ideal_t']) < 0.02 and 12.9 < r['impact_vy'] < r['ideal_vy']  # drag makes it a bit slower
    assert 1.3 < r['flat_fall_time'] < 1.5 and 12 < r['flat_impact_vy'] < 13.5


def test_parked_car_is_quiet():
    r = sanity.rest(seconds=6.0)
    assert r['settle_residual_N'] < 1e-6 and r['max_speed'] < 1e-9 and r['drift_m'] < 1e-9 and r['wheels'] == 4
    p = r['perturbed']
    assert p['speed+rate_last_5s'] < 1e-3 and p['creep_last_5s_m'] < 1e-3


def test_hover_thrust_equals_weight():
    r = sanity.hover()
    assert abs(r['thrust_over_weight_mean'] - 1) < 0.02 and r['episode']['success']


def test_blowdown():
    r = sanity.blowdown()
    assert 1.5 < r['dropout_t'] < 2.5 and r['exhausted_t'] < 5 and 100 < r['used_kg'] < r['gas0_kg']
    assert 45 < r['mean_isp_s'] < 66


def test_free_tumble_conserves_energy_and_momentum():
    r = sanity.energy(seconds=4.0)
    assert abs(r['rel_dE_rot']) < 1e-6 and abs(r['rel_dL']) < 1e-6 and abs(r['rel_dE_total']) < 1e-4


def test_no_tunnelling():
    r = sanity.tunnelling()
    assert r['deck_drop']['min_com_height_above_deck'] > 0.05 and r['runup_com_dip_m'] < 0.05

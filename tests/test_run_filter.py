'''Tests for the RunSignalFilter that debounces flickery body-tracking into a run signal.'''

from apm.control.run_filter import RunSignalFilter


def test_acquires_immediately_by_default():
    f = RunSignalFilter(release_s=0.5)
    assert f.update(True, now=0.0) is True


def test_bridges_short_dropout():
    '''A dropout shorter than release_s must keep the run signal asserted.'''
    f = RunSignalFilter(release_s=0.5)
    assert f.update(True, now=0.0) is True
    # Runner missing for 0.1 s and 0.3 s since last detection -> still present
    assert f.update(False, now=0.1) is True
    assert f.update(False, now=0.3) is True
    # Detection returns -> stays present, timer resets
    assert f.update(True, now=0.4) is True


def test_releases_after_timeout():
    '''Continuous absence beyond release_s releases the run signal.'''
    f = RunSignalFilter(release_s=0.5)
    f.update(True, now=0.0)
    assert f.update(False, now=0.4) is True      # 0.4 s < 0.5 s -> still present
    assert f.update(False, now=0.6) is False     # 0.6 s >= 0.5 s -> released


def test_acquire_delay_rejects_single_frame_blip():
    '''With acquire_s>0, a lone detection frame should not assert the run signal.'''
    f = RunSignalFilter(release_s=0.5, acquire_s=0.2)
    assert f.update(True, now=0.0) is False       # first detection, not yet sustained
    assert f.update(False, now=0.05) is False      # blip gone -> acquire streak resets
    # Sustained detection for >= 0.2 s now asserts
    assert f.update(True, now=1.0) is False
    assert f.update(True, now=1.25) is True


def test_reset_clears_state():
    f = RunSignalFilter(release_s=0.5)
    f.update(True, now=0.0)
    f.reset()
    assert f.present is False

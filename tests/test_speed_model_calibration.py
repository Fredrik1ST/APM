'''Test speed model calibration data.

Loads logs/calibration.csv (if it exists), plots the collected samples,
and runs the linear regression fit.

Run with:
    pytest tests/test_speed_model_calibration.py -s --plot
'''

import pytest
from apm.calibration import SpeedModelCalibrator


@pytest.fixture
def plot(request):
    return request.config.getoption("--plot")


@pytest.fixture
def calibrator():
    path = SpeedModelCalibrator.DEFAULT_PATH
    if not path.exists():
        pytest.skip(f'No calibration data found at {path}')
    return SpeedModelCalibrator.load()


def test_load(calibrator):
    assert len(calibrator.samples) > 0, 'Calibration file is empty'
    print(f'\n  Loaded {len(calibrator.samples)} samples')


def test_fit(calibrator):
    factor, neutral_pwm = calibrator.fit()
    print(f'\n  factor={factor:.4f}, neutral_pwm={neutral_pwm:.2f}')
    assert factor > 0, 'Fitted factor should be positive'


def test_plot(calibrator, plot):
    calibrator.plot(save_path=None if plot else 'tests/output/calibration.pdf')

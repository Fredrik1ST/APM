'''
Calibration tool for the linear speed model used to convert m/s to PWM.

The model is:  pwm = factor * speed + neutral_pwm

Usage:
    cal = SpeedModelCalibrator.load_or_new()   # resumes previous run if file exists

    # Inside a control loop, call record() when you want a new sample:
    cal.record(setpoint_mps=setpoint, gnss_mps=measured, pwm=pwm_sent)

    cal.save()   # saves to calibration.pkl by default

    # Offline:
    cal = SpeedModelCalibrator.load("calibration_run.pkl")
    cal.plot()
    factor, neutral = cal.fit()
'''

import pickle
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

log = logging.getLogger(__name__)


@dataclass
class SpeedSample:
    timestamp: float   # monotonic
    setpoint_mps: float
    gnss_mps: float
    pwm: float


@dataclass
class SpeedModelCalibrator:
    '''Collects speed samples and fits a linear pwm = factor * speed + neutral model.'''

    samples: list[SpeedSample] = field(default_factory=list)

    def record(self, setpoint_mps: float, gnss_mps: float, pwm: float) -> None:
        '''Append one sample. Call this each control loop tick during a calibration run.'''
        self.samples.append(SpeedSample(
            timestamp=time.monotonic(),
            setpoint_mps=setpoint_mps,
            gnss_mps=gnss_mps,
            pwm=pwm,
        ))


    DEFAULT_PATH = Path('calibration.pkl')

    def save(self, path: str | Path | None = None) -> None:
        '''Pickle the calibrator to disk.'''
        path = Path(path) if path else self.DEFAULT_PATH
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'wb') as f:
            pickle.dump(self, f)
        log.info(f'Saved {len(self.samples)} samples to {path}')


    @classmethod
    def load(cls, path: str | Path | None = None) -> 'SpeedModelCalibrator':
        '''Load a pickled calibrator from disk.'''
        path = Path(path) if path else cls.DEFAULT_PATH
        with open(path, 'rb') as f:
            obj = pickle.load(f)
        if not isinstance(obj, cls):
            raise TypeError(f'Expected SpeedModelCalibrator, got {type(obj)}')
        log.info(f'Loaded {len(obj.samples)} samples from {path}')
        return obj


    @classmethod
    def load_or_new(cls, path: str | Path | None = None) -> 'SpeedModelCalibrator':
        '''Load an existing calibrator from disk, or return a fresh one if the file does not exist.'''
        path = Path(path) if path else cls.DEFAULT_PATH
        if path.exists():
            return cls.load(path)
        log.info(f'No calibration file found at {path}, starting fresh.')
        return cls()


    def fit(self) -> tuple[float, float]:
        '''
        Linear regression of pwm vs. measured GNSS speed.

        Returns (factor, neutral_pwm) for the model  pwm = factor * gnss_mps + neutral_pwm.
        Only samples where the vehicle was actually moving (gnss_mps > 0) are used.
        '''
        moving = [s for s in self.samples if s.gnss_mps > 0.0]
        if len(moving) < 2:
            raise ValueError(f'Need at least 2 moving samples to fit, got {len(moving)}')

        speeds = np.array([s.gnss_mps for s in moving])
        pwms   = np.array([s.pwm      for s in moving])

        result = linregress(speeds, pwms)
        factor      = float(result.slope)
        neutral_pwm = float(result.intercept)

        log.info(
            f'Fit: factor={factor:.4f}, neutral_pwm={neutral_pwm:.2f}, '
            f'R²={result.rvalue**2:.4f}  (n={len(moving)})'
        )
        return factor, neutral_pwm


    def plot(self, save_path: str | Path | None = None) -> None:
        '''
        Plot collected data and the fitted linear model.

        Pass save_path to write the figure to disk instead of (or as well as) showing it.
        '''
        if not self.samples:
            log.warning('No samples to plot.')
            return

        t0       = self.samples[0].timestamp
        t        = np.array([s.timestamp - t0    for s in self.samples])
        setpoint = np.array([s.setpoint_mps      for s in self.samples])
        gnss     = np.array([s.gnss_mps          for s in self.samples])
        pwm      = np.array([s.pwm               for s in self.samples])

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
        fig.suptitle('Speed Model Calibration Data')

        # Top: speed over time
        ax = axes[0]
        ax.plot(t, setpoint, label='Setpoint (m/s)', linestyle='--')
        ax.plot(t, gnss,     label='GNSS speed (m/s)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.legend()
        ax.grid(True)

        # Middle: PWM over time
        ax = axes[1]
        ax.plot(t, pwm, color='tab:orange', label='PWM output (µs)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('PWM (µs)')
        ax.legend()
        ax.grid(True)

        # Bottom: PWM vs GNSS speed scatter + regression line
        ax = axes[2]
        moving = [s for s in self.samples if s.gnss_mps > 0.0]
        if len(moving) >= 2:
            speeds_m = np.array([s.gnss_mps for s in moving])
            pwms_m   = np.array([s.pwm      for s in moving])
            ax.scatter(speeds_m, pwms_m, s=4, alpha=0.5, label='Samples (moving)')

            try:
                factor, neutral = self.fit()
                x_line = np.linspace(speeds_m.min(), speeds_m.max(), 200)
                ax.plot(x_line, factor * x_line + neutral, color='red',
                        label=f'Fit: pwm = {factor:.2f}·v + {neutral:.1f}')
            except ValueError:
                pass
        else:
            speeds_all = np.array([s.gnss_mps for s in self.samples])
            pwms_all   = np.array([s.pwm      for s in self.samples])
            ax.scatter(speeds_all, pwms_all, s=4, alpha=0.5)

        ax.set_xlabel('GNSS speed (m/s)')
        ax.set_ylabel('PWM (µs)')
        ax.legend()
        ax.grid(True)

        fig.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=150)
            log.info(f'Plot saved to {save_path}')

        plt.show()

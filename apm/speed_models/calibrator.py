'''
Calibration tool for the affine speed model used to convert m/s to PWM.

The default model is based on linear regression:  pwm = factor * speed + neutral_pwm

Usage:
    cal = SpeedModelCalibrator.load_or_new()   # resumes previous run if file exists

    # Inside a control loop, call record() when you want a new sample.
    # Ideally at some point when the setpoint is stable and a new GNSS speed measurement is available.
    cal.record(setpoint_mps=setpoint, gnss_mps=measured, pwm=pwm_sent)

    cal.save()   # saves to logs/calibration.csv by default

    # Offline:
    cal = SpeedModelCalibrator.load()
    cal.plot()
    factor, neutral = cal.fit()
'''

import csv
import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

log = logging.getLogger(__name__)


@dataclass
class SpeedSample:
    setpoint_mps: float
    gnss_mps: float
    pwm: float


@dataclass
class SpeedModelCalibrator:
    '''Collects speed samples and fits a linear pwm = factor * speed + neutral model.'''

    samples: list[SpeedSample] = field(default_factory=list)

    DEFAULT_PATH = Path(__file__).resolve().parent.parent.parent / 'logs' / 'calibration.csv'


    def record(self, setpoint_mps: float, gnss_mps: float, pwm: float) -> None:
        '''Append one sample.'''
        self.samples.append(SpeedSample(
            setpoint_mps=setpoint_mps,
            gnss_mps=gnss_mps,
            pwm=pwm,
        ))


    def save(self, path: str | Path | None = None) -> None:
        '''Save samples to a CSV file, appending to any existing data.'''
        path = Path(path) if path else self.DEFAULT_PATH
        path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not path.exists()
        with open(path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['setpoint_mps', 'gnss_mps', 'pwm'])
            for s in self.samples:
                writer.writerow([s.setpoint_mps, s.gnss_mps, s.pwm])
        log.info(f'Saved {len(self.samples)} samples to {path}')


    @classmethod
    def load(cls, path: str | Path | None = None) -> 'SpeedModelCalibrator':
        '''Load samples from a CSV file.'''
        path = Path(path) if path else cls.DEFAULT_PATH
        obj = cls()
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                obj.samples.append(SpeedSample(
                    setpoint_mps = float(row['setpoint_mps']),
                    gnss_mps     = float(row['gnss_mps']),
                    pwm          = float(row['pwm']),
                ))
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


    def plot(self, save_path: str | Path | None = None, show: bool = True) -> None:
        '''
        Plot PWM (x) vs. speed (y) for all samples, with the fitted linear model overlaid.

        Pass save_path to write the figure to disk instead of (or as well as) showing it.
        Pass show=False to skip the (blocking) interactive window.
        '''
        if not self.samples:
            log.warning('No samples to plot.')
            return

        pwm      = np.array([s.pwm          for s in self.samples])
        gnss     = np.array([s.gnss_mps     for s in self.samples])
        setpoint = np.array([s.setpoint_mps for s in self.samples])

        fig, ax = plt.subplots(figsize=(8, 5))
        fig.suptitle('Speed Model Calibration')

        ax.scatter(pwm, setpoint, s=4, alpha=0.4, label='Setpoint (m/s)')
        ax.scatter(pwm, gnss,     s=4, alpha=0.4, label='GNSS speed (m/s)')

        moving = [s for s in self.samples if s.gnss_mps > 0.0]
        if len(moving) >= 2:
            try:
                factor, neutral = self.fit()
                # Invert the model (pwm = factor*v + neutral) to draw speed as a function of PWM
                x_line = np.linspace(pwm.min(), pwm.max(), 200)
                ax.plot(x_line, (x_line - neutral) / factor, color='red',
                        label=f'Fit: v = (pwm − {neutral:.1f}) / {factor:.2f}')
            except ValueError:
                pass

        ax.set_xlabel('PWM (µs)')
        ax.set_ylabel('Speed (m/s)')
        ax.legend()
        ax.grid(True)
        fig.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=150)
            log.info(f'Plot saved to {save_path}')

        if show:
            plt.show()
        else:
            plt.close(fig)

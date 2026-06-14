"""This module contains feedforward speed models, outputting a PWM signal based on a desired speed.

The original model was created from manual linear regression on pwm vs GNSS speed measurements:
pwm = desired_speed * factor + neutral_pwm = v_d * 20.23 + 1540
"""

from abc import ABC, abstractmethod
import numpy as np

class SpeedModel(ABC):
    '''Abstract base class for feedforward speed models.'''

    @abstractmethod
    def speed_to_pwm(self, speed: float) -> float:
        '''Convert a desired speed in m/s to a PWM signal.'''
        pass


class AffineSpeedModel(SpeedModel):
    '''Basic affine feedforward speed model: pwm = factor * speed + neutral_pwm'''

    def __init__(self, factor: float, neutral_pwm: float):
        self.factor = factor
        self.neutral_pwm = neutral_pwm

    def speed_to_pwm(self, speed: float) -> float:
        return self.factor * speed + self.neutral_pwm
    

class PolynomialSpeedModel(SpeedModel):
    '''Polynomial feedforward speed model: pwm = a_n * speed^n + ... + a_1 * speed + a_0
    
    Attributes:
        coefficients: list of polynomial coefficients, ordered from highest to lowest degree
    
    Examples:
        [a_2, a_1, a_0] gives a quadratic (2nd order) model: :math:`pwm = (a_2 * speed^2) + (a_1 * speed) + a_0`

        [a_1, a_0] gives a linear or affine (1st order) model: :math:`pwm = (a_1 * speed) + a_0`
    '''

    def __init__(self, coefficients: list[float]):
        self.coefficients = np.array(coefficients)

    def speed_to_pwm(self, speed: float) -> float:
        return float(np.polyval(self.coefficients, speed))
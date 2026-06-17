r'''
Cruise control is the primary speed control mode of the APM during runs.

During cruise control, the APM follows a pacing profile (a CSV of ``(speed [m/s], duration [s])`` rows) 
as long as the runner stays within a tolerance distance. 

In Normal mode: If the runner falls behind, the controller gets switched to the distance controller (see ``distance_controller.py``)
until the runner is able to catch up to the pacing profile again, at which point control returns here.


Architecture:
===========

The cruise controller is a **cascade controller**: a fast velocity (inner) loop
nested inside a slow odometer-distance / schedule (outer) loop. This is the
classic motion-control / automotive-cruise topology.

    inner loop   feedforward speed model + PI trim on the PWM actuator,
                 with the speed feedback supplied by a complementary filter
                 that fuses the command v_d with the (slow) GNSS speed.
    outer loop   integral correction that nulls accumulated *distance* error
                 against the pacing schedule, so the car arrives "at the right
                 place at the right time", not merely "at the right speed".


Signal flow
===========

(Σ = summing junction; sign shown next to each input.)

                outer (schedule / distance) loop
       ┌──────────────────────────────────────────────────────────┐
       │                                                          │
  s_ref = ∫v_p                                                    │
       └─►(Σ)── e_s ──►[ K_s^out · ∫(·)dt ]── Δv ──┐              │
           ▲(+)                                    │              │
           │(-) s_meas = ∫v_gnss                   │(+)           │
                                                   ▼              │
  v_p ───────────────────────────────────►[ Ramp ]──►(Σ)── v_d ──┬┴──────────────────────────────►[ g(·) ]── u_ff ──►(Σ)──►[ sat ]── u ──►[ Vehicle/ESC ]── v ──►
  (profile)                                                      │                                                     ▲(+)                                 │
                                                                 │     e_v = v_d - v̂                                   │ u_fb                               │
                                                                 ├───────────►(Σ)──►[ PI ]─────────────────────────────┘                                    │
                                                                 │            ▲(+)  (-)                                                                     │
                                                                 │            │ v̂                                                                           │
                                                                 │   [ complementary filter ]◄── v_gnss ◄──────────[ GNSS ]◄────────────────────────────────┘
                                                                 └────────────┘  ▲
                                                                   (v_d feeds the HPF branch of the filter)

Symbols
-------
    v_p       pacing-profile speed setpoint                        [m/s]
    v_d       desired speed after ramp + outer correction          [m/s]
    v_hat (v̂) fused speed estimate (complementary filter output)   [m/s]
    v_gnss    measured ground speed from GNSS                      [m/s]
    v         true vehicle speed                                   [m/s]
    u_ff      feedforward PWM from the speed model                 [pwm]
    u_fb      PI trim PWM                                          [pwm]
    u         commanded PWM after saturation                       [pwm]
    s_ref     reference (schedule) distance                        [m]
    s_meas    measured travelled distance                          [m]
    e_v       velocity error                                       [m/s]
    e_s       schedule (distance) error                            [m]
    Δv        outer-loop speed correction                          [m/s]


Mathematics
===========

Inner loop - feedforward + PI on the actuator
---------------------------------------------
The static feedforward speed model g(·) (``SpeedModel.speed_to_pwm``, e.g. the
affine ``u = a·v_d + b``) inverts the steady-state speed→PWM map and carries the
bulk of the command. The motor/ESC is treated as instantaneous relative to the
vehicle, so g(·) is memoryless.

    u_ff(t) = g(v_d(t))

The PI **trims the PWM**, not v_d - the disturbances it rejects (grade, wind,
battery sag, drivetrain loss) enter at the actuator, and trimming PWM keeps the
loop gain uniform instead of scaling it by g'(v_d):

    e_v(t) = v_d(t) - v_hat(t)
    u_fb(t) = K_p · e_v(t) + K_i · ∫₀ᵗ e_v(τ) dτ
    u(t)   = sat( u_ff(t) + u_fb(t) )        sat: clamp to [u_min, u_max]

Complementary filter (GNSS-aided dead reckoning)
------------------------------------------------
The control loop runs far faster than GNSS updates, so we dead-reckon on the
command v_d between fixes and let GNSS anchor the low-frequency truth. The filter
is built so its steady state equals the *measurement*, not the command -
otherwise e_v becomes self-referential and the PI goes blind to model error:

    V̂(s) = G_lp(s)·V_gnss(s) + G_hp(s)·V_d(s)
    G_lp(s) = 1/(τs+1),   G_hp(s) = τs/(τs+1),   G_lp + G_hp = 1

The high-pass on v_d means only its *changes* propagate fast; at DC, V̂ → V_gnss.
Discrete realisation (sample time T, a = τ/(τ+T)):

    v̂[k] = a·( v̂[k-1] + v_d[k] - v_d[k-1] ) + (1-a)·v_gnss[k]

Outer loop - schedule (distance) regulation
-------------------------------------------
A velocity loop with zero steady-state speed error can still accumulate a
permanent *position* offset from any transient. For a pacemaker, distance-vs-time
is the whole point, so the outer loop regulates accumulated distance against the
schedule. Crucially the reference is the integral of the *raw profile* v_p (the
untrimmed plan), and the feedback is measured travel:

    s_ref(t)  = ∫₀ᵗ v_p(τ) dτ                (planned distance)
    s_meas(t) = ∫₀ᵗ v_gnss(τ) dτ             (or wheel/ESC odometry)
    e_s(t)    = s_ref(t) - s_meas(t)
    Δv(t)     = K_s^out · e_s(t)             (slow; |K_s^out| ≪ inner bandwidth)

Δv is added to the ramped profile speed to form the inner setpoint:

    v_d(t) = Ramp(v_p)(t) + Δv(t)

Because Δv evolves slowly relative to the ramp time constant it does not provoke
setpoint kick, so adding it downstream of the ramp is acceptable.

Anti-windup
-----------
Two integrators (the inner PI's I-term and the outer Δv integrator) ultimately
drive one saturating PWM. Both must be conditioned against ``sat`` - e.g. clamp
/ back-calculation, and freeze the outer integrator while u is saturated - or
they wind up against the limit and overshoot on release.
'''

from apm.control.pid_controller import PIDController
from apm.speed_models import SpeedModel
from apm.control.velocity_profiles import LinearRamp as Ramp


class ComplementaryFilter:
    """Fuses a fast, drift-prone signal with a slow, absolute one.

    High-passes the fast signal (only its *changes* are trusted) and low-passes
    the slow one; the two transfer functions sum to 1, so the steady-state
    output equals the slow (absolute) source. Here it blends the commanded speed
    v_d (fast, updated every tick) with the GNSS speed (slow, accurate) into the
    speed estimate v_hat used by the inner velocity loop.

        v_hat[k] = a*(v_hat[k-1] + v_d[k] - v_d[k-1]) + (1-a)*v_gnss[k],
        a = tau / (tau + dt)

    Larger tau -> trust the dead-reckoned command for longer between GNSS fixes.
    """

    def __init__(self, tau: float):
        self.tau = tau              # filter time constant [s]
        self.estimate = 0.0         # v_hat
        self._last_fast = 0.0       # v_d from the previous tick
        self._primed = False        # seed from the first measurement, not 0.0

    def update(self, fast: float, slow: float, dt: float) -> float:
        if not self._primed:
            self.estimate = slow
            self._last_fast = fast
            self._primed = True
            return self.estimate

        a = self.tau / (self.tau + dt) if (self.tau + dt) > 0 else 0.0
        self.estimate = a * (self.estimate + fast - self._last_fast) + (1.0 - a) * slow
        self._last_fast = fast
        return self.estimate

    def reset(self, speed: float = 0.0):
        self.estimate = speed
        self._last_fast = speed
        self._primed = False


class CruiseController:
    """Cascade speed controller that follows a pacing profile (see module docstring).

    Inner loop: feedforward speed model + PI trim on the PWM, with the speed
    feedback supplied by a complementary filter. Outer loop: a slow integral that
    nulls accumulated distance error against the schedule.
    """

    def __init__(
        self,
        pid: PIDController,
        speed_model: SpeedModel,
        ramp: Ramp,
        cf: ComplementaryFilter,
        k_s: float,
        pwm_min: float,
        pwm_max: float,
    ):
        self.pid = pid                  # inner velocity PI (use kd=0)
        self.speed_model = speed_model  # feedforward g(.)
        self.ramp = ramp                # setpoint shaping on the profile speed
        self.cf = cf                    # speed estimator (v_d + GNSS -> v_hat)
        self.k_s = k_s                  # outer gain K_s^out [1/s]: speed per metre of schedule error
        self.pwm_min = pwm_min          # actuator saturation, e.g. neutral PWM (no reverse)
        self.pwm_max = pwm_max

        # Outer-loop integrator: schedule error e_s = integral(v_p - v_gnss) [m]
        # (equivalently s_ref - s_meas). One state keeps anti-windup simple.
        self.schedule_error = 0.0

        # Last-computed intermediates, exposed for telemetry (refreshed each update()).
        self.last_outer_correction = 0.0   # Delta v
        self.last_desired_speed = 0.0      # v_d
        self.last_speed_estimate = 0.0     # v_hat
        self.last_feedforward = 0.0        # u_ff
        self.last_feedback = 0.0           # u_fb
        self.last_pwm = 0.0                # u (after saturation)
        self.last_saturated = False

    def update(self, profile_speed: float, gnss_speed: float, dt: float) -> float:
        """Compute the motor PWM for one tick.

        Args:
            profile_speed: Target speed v_p from the pacing profile [m/s].
            gnss_speed: Measured ground speed from GNSS [m/s].
            dt: Time since the previous update [s].
        """
        # --- Outer loop: integrate the schedule error and turn it into a slow speed trim ---
        outer_inc = (profile_speed - gnss_speed) * dt
        self.schedule_error += outer_inc
        delta_v = self.k_s * self.schedule_error

        # --- Setpoint shaping: ramp the profile, then add the slow outer trim ---
        # delta_v is slow relative to the ramp, so adding it downstream causes no setpoint kick.
        v_d = self.ramp.update(profile_speed, dt) + delta_v

        # --- Speed estimate: fuse the fast command with the slow GNSS measurement ---
        v_hat = self.cf.update(v_d, gnss_speed, dt)

        # --- Inner loop: feedforward + PI trim, both summed at the PWM actuator ---
        u_ff = self.speed_model.speed_to_pwm(v_d)
        pid_integral_before = self.pid.integral
        u_fb = self.pid.update(v_d, v_hat)      # PID error = v_d - v_hat = e_v
        e_v = v_d - v_hat
        u = u_ff + u_fb

        # --- Saturation + anti-windup ---
        # Conditional integration: when clamped, roll back any integrator increment
        # that would push *further* into the active limit. Both integrators act in the
        # +PWM direction for positive error, so we only need each increment's sign.
        u_sat = min(self.pwm_max, max(self.pwm_min, u))
        if u_sat > u:        # clamped up against pwm_min: freeze terms pulling further down
            if e_v < 0:
                self.pid.integral = pid_integral_before
            if outer_inc < 0:
                self.schedule_error -= outer_inc
        elif u_sat < u:      # clamped down against pwm_max: freeze terms pushing further up
            if e_v > 0:
                self.pid.integral = pid_integral_before
            if outer_inc > 0:
                self.schedule_error -= outer_inc

        self.last_outer_correction = delta_v
        self.last_desired_speed = v_d
        self.last_speed_estimate = v_hat
        self.last_feedforward = u_ff
        self.last_feedback = u_fb
        self.last_pwm = u_sat
        self.last_saturated = u_sat != u
        return u_sat

    def integrate_schedule(self, profile_speed: float, gnss_speed: float, dt: float) -> float:
        """Advance only the outer-loop schedule integral, without touching the inner loop.

        Use this while another controller holds the actuator (e.g. distance control in
        Normal mode): the schedule deficit s_ref - s_meas = integral(v_p - v_gnss) stays
        live, so the supervisor can tell when the runner has made up the lost ground.
        Returns the updated ``schedule_error`` [m]."""
        self.schedule_error += (profile_speed - gnss_speed) * dt
        return self.schedule_error

    def reset(self, speed: float = 0.0, keep_schedule: bool = False):
        """Clear controller state. Call when (re)entering cruise mode.

        Seeds the inner loop (ramp/CF/PID) to ``speed`` for a bumpless resume. Set
        ``keep_schedule`` to preserve the outer-loop ``schedule_error`` across a handoff
        (e.g. returning from distance control) so the outer loop nulls the residual
        deficit instead of forgetting it."""
        if not keep_schedule:
            self.schedule_error = 0.0
        self.pid.integral = 0.0
        self.pid.previous_error = 0.0
        self.ramp.reset(speed)
        self.cf.reset(speed)

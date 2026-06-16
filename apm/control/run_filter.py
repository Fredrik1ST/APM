'''
Debounce filter for the runner-detection / run signal.

Filters the run-signal to the motor in case the body-tracking detection flickers.

Time-based (pass a monotonic timestamp) so behaviour is independent of frame rate.
'''


class RunSignalFilter:
    '''Hysteresis/debounce for a flickering boolean detection -> stable run signal.

    Args:
        release_s: Seconds the detection must be continuously absent before releasing.
        acquire_s: Seconds the detection must be continuously present before asserting. (Default = 0)
    '''

    def __init__(self, release_s: float, acquire_s: float = 0.0):
        self.release_s = release_s
        self.acquire_s = acquire_s
        self._present = False
        self._last_detect_t: float | None = None   # last time detected was True
        self._acquire_start: float | None = None    # start of the current presence streak

    @property
    def present(self) -> bool:
        return self._present

    def update(self, detected: bool, now: float) -> bool:
        '''Advance the filter with one raw detection and return the debounced run signal.'''
        if detected:
            self._last_detect_t = now
            if not self._present:
                if self._acquire_start is None:
                    self._acquire_start = now
                if now - self._acquire_start >= self.acquire_s:
                    self._present = True
        else:
            self._acquire_start = None  # reset the acquire streak on any miss
            if self._present and self._last_detect_t is not None:
                if now - self._last_detect_t >= self.release_s:
                    self._present = False
        return self._present

    def reset(self) -> None:
        '''Clear all state (e.g. when (re)starting a mode).'''
        self._present = False
        self._last_detect_t = None
        self._acquire_start = None

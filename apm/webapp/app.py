'''
NiceGUI web interface for the Autonomous Pacemaker.

Passive UI layer: 
reads state from and calls methods on the Orchestrator instance it receives.

Started as a daemon thread by Orchestrator.run().
'''

import threading
import logging
from nicegui import ui, app as nicegui_app
from fastapi import HTTPException
from fastapi.responses import JSONResponse

import config_handler as config
from orchestrator import Orchestrator, State, Mode

log = logging.getLogger(__name__)


_LEVEL_COLORS = {
    logging.DEBUG:    'text-gray-400',
    logging.INFO:     'text-gray-200',
    logging.WARNING:  'text-amber-400',
    logging.ERROR:    'text-red-400',
    logging.CRITICAL: 'text-red-600',
}


class _LastLogHandler(logging.Handler):
    """Keeps the most recent log record for each logger name."""

    def __init__(self) -> None:
        super().__init__()
        self._records: dict[str, logging.LogRecord] = {}
        self._lock = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        with self._lock:
            self._records[record.name] = record

    def latest(self, name: str) -> logging.LogRecord | None:
        with self._lock:
            return self._records.get(name)


STATE_COLORS: dict[State, str] = {
    State.IDLE:          'grey',
    State.CONFIG:        'blue',
    State.STARTING:      'orange',
    State.RUNNING:       'green',
    State.RUNNING_PACE:  'green',
    State.RUNNING_DIST:  'green',
    State.RUNNING_CONST: 'green',
    State.STOPPING:      'orange',
    State.FINISHED:      'teal',
    State.ERROR:         'red',
}

MODE_LABELS: dict[Mode, str] = {
    Mode.NORMAL:            'Normal (pace + distance)',
    Mode.PACE_ONLY:         'Pace only',
    Mode.DISTANCE_ONLY:     'Distance only',
    Mode.CONSTANT_SPEED:    'Constant speed',
    Mode.CAMERA_TEST:       'Camera test (both)',
    Mode.CAMERA_TEST_FRONT: 'Camera test (front)',
    Mode.CAMERA_TEST_BACK:  'Camera test (back)',
    Mode.ARDUINO_TEST:      'Arduino test',
}

_ACTIVE_STATES = {State.STARTING, State.RUNNING, State.RUNNING_PACE,
                  State.RUNNING_DIST, State.RUNNING_CONST, State.STOPPING}


#MODE_LOGGERS: dict[Mode, str] = {
#    Mode.NORMAL:       'apm.modes.normal',
#    Mode.PACE_ONLY:    'apm.modes.pace_only',
#    Mode.DISTANCE_ONLY:'apm.modes.distance_only',
#    Mode.CONSTANT_SPEED:'apm.modes.constant_speed',
#    Mode.CAMERA_TEST:  'apm.modes.camera_test',
#    Mode.CAMERA_TEST_FRONT: 'apm.modes.camera_test_front',
#    Mode.CAMERA_TEST_BACK:  'apm.modes.camera_test_back',
#    Mode.ARDUINO_TEST: 'apm.modes.arduino_test',
#    Mode.GNSS_TEST:    'apm.modes.gnss_test'
#}


# ---------------------------------------------------------------------------
# Config editor
# ---------------------------------------------------------------------------

def _build_config_editor() -> tuple[dict, list]:
    doc = config.load()
    inputs: list[tuple[list[str], 'ui.input']] = []

    def render_section(parent_keys: list[str], node) -> None:
        for key, value in node.items():
            keys = parent_keys + [key]
            if isinstance(value, dict):
                ui.label(' > '.join(keys)).classes(
                    'text-xs text-gray-400 mt-4 mb-1 font-mono uppercase tracking-wider'
                )
                render_section(keys, value)
            else:
                with ui.row().classes('items-center gap-3 w-full'):
                    ui.label(key).classes('text-sm w-40 shrink-0 font-mono')
                    inp = ui.input(value=str(value)).classes('grow font-mono text-sm')
                    inp.on('input', lambda el=inp: el.props('color=amber'))
                    inputs.append((keys, inp))

    render_section([], doc)
    return doc, inputs


# ---------------------------------------------------------------------------
# Page
# ---------------------------------------------------------------------------

def _register_page(orchestrator: Orchestrator, log_handler: _LastLogHandler) -> None:
    @ui.page('/')
    def _index() -> None:
        ui.dark_mode().enable()
        ui.query('body').style('background-color: #121212')

        # --- header -------------------------------------------------------
        with ui.header().classes('bg-gray-900 px-6 py-3 flex items-center justify-between'):
            with ui.row().classes('items-center gap-3'):
                ui.icon('directions_run', size='lg', color='green')
                ui.label('Autonomous Pacemaker').classes('text-xl font-bold text-white')
            state_badge = ui.badge('IDLE', color='grey').classes('text-sm px-3 py-1')

        # --- main grid ----------------------------------------------------
        with ui.grid(columns=2).classes('w-full gap-4 p-4'):

            # control card
            with ui.card().classes('col-span-1'):
                ui.label('Control').classes('text-lg font-semibold mb-2')

                start_stop_btn = ui.button('Start', color='green').classes('w-full text-lg')

                def toggle_start_stop() -> None:
                    if orchestrator.state in _ACTIVE_STATES:
                        orchestrator.request_stop()
                    else:
                        orchestrator.mode = mode_select.value
                        orchestrator.request_start()

                start_stop_btn.on_click(toggle_start_stop)

                ui.separator().classes('my-3')

                ui.label('Mode').classes('text-sm text-gray-400 mb-1')
                mode_select = ui.select(
                    options={m: label for m, label in MODE_LABELS.items()},
                    value=orchestrator.mode if orchestrator.mode != Mode.NONE else Mode.NORMAL,
                ).classes('w-full')

            # status card
            with ui.card().classes('col-span-1'):
                ui.label('Module Status').classes('text-lg font-semibold mb-2')
                arduino_row   = ui.row().classes('items-center gap-2')
                front_cam_row = ui.row().classes('items-center gap-2')
                back_cam_row  = ui.row().classes('items-center gap-2')
                gnss_row      = ui.row().classes('items-center gap-2')
                ui.separator().classes('my-2')
                mode_log_row  = ui.row().classes('items-start gap-2 w-full')

        # --- config editor ------------------------------------------------
        with ui.card().classes('w-full mx-4 mb-4'):
            with ui.expansion('Configuration', icon='settings').classes('w-full'):
                with ui.row().classes('mb-3'):
                    def save_config() -> None:
                        try:
                            for keys, inp in inputs:
                                d = doc
                                for k in keys[:-1]:
                                    d = d[k]
                                raw = inp.value
                                original = d[keys[-1]]
                                if isinstance(original, bool):
                                    d[keys[-1]] = raw.lower() in ('true', '1', 'yes')
                                elif isinstance(original, int):
                                    d[keys[-1]] = int(raw)
                                elif isinstance(original, float):
                                    d[keys[-1]] = float(raw)
                                else:
                                    d[keys[-1]] = raw
                            config.save(doc)
                            for _, inp in inputs:
                                inp.props(remove='color')
                            ui.notify('Saved', type='positive', position='bottom-right')
                        except Exception as e:
                            ui.notify(f'Error: {e}', type='negative', position='bottom-right')

                    def reset_defaults() -> None:
                        nonlocal doc, inputs
                        config.reset_to_defaults()
                        config_col.clear()
                        doc, inputs = _build_config_editor()
                        ui.notify('Reset to defaults', type='warning', position='bottom-right')

                    ui.button('Save', icon='save', color='green', on_click=save_config)
                    ui.button('Reset to defaults', icon='restart_alt',
                              color='orange', on_click=reset_defaults)

                with ui.column().classes('w-full gap-1') as config_col:
                    doc, inputs = _build_config_editor()

        # --- status helpers -----------------------------------------------
        def update_status_row(row, label: str, ok: bool, logger_name: str) -> None:
            row.clear()
            with row:
                ui.icon('check_circle' if ok else 'cancel',
                        color='green' if ok else 'red', size='sm')
                ui.label(label).classes('text-sm font-medium w-40 shrink-0')
                record = log_handler.latest(logger_name)
                if record:
                    color = _LEVEL_COLORS.get(record.levelno, 'text-gray-200')
                    ui.label(record.getMessage()).classes(f'text-xs font-mono {color} truncate')

        # --- periodic refresh ---------------------------------------------
        def refresh_ui() -> None:
            s = orchestrator.state
            state_badge.set_text(s.name)
            state_badge.props(f'color={STATE_COLORS.get(s, "grey")}')

            if s in _ACTIVE_STATES:
                start_stop_btn.set_text('Stop')
                start_stop_btn.props('color=red')
            else:
                start_stop_btn.set_text('Start')
                start_stop_btn.props('color=green')

            update_status_row(arduino_row,   'Arduino',         orchestrator.arduino.ok,        'apm.drivers.arduino')
            update_status_row(front_cam_row, 'Front camera',    orchestrator.front_camera.ok,   'apm.drivers.camera.front')
            update_status_row(back_cam_row,  'Back camera',     orchestrator.back_camera.ok,    'apm.drivers.camera.back')
            update_status_row(gnss_row,      'GNSS',            orchestrator.gnss.ok,           'apm.drivers.gnss')

            # Show latest log entry from the currently running program mode
            mode_log_row.clear()
            #logger_name = MODE_LOGGERS.get(orchestrator.mode)
            logger_name = f'apm.modes.{orchestrator.mode.name.lower()}' if orchestrator.mode != Mode.NONE else None

            if logger_name:
                record = log_handler.latest(logger_name)
                if record:
                    with mode_log_row:
                        ui.icon('terminal', size='sm').classes('text-gray-400 mt-0.5 shrink-0')
                        color = _LEVEL_COLORS.get(record.levelno, 'text-gray-200')
                        ui.label(record.getMessage()).classes(f'text-xs font-mono {color}')

        ui.timer(1.0, refresh_ui)


# ---------------------------------------------------------------------------
# Public API — called by Orchestrator
# ---------------------------------------------------------------------------

def start(orchestrator: Orchestrator, host: str = '0.0.0.0', port: int = 8080) -> None:
    '''Register the UI page and start NiceGUI on a daemon thread.'''
    log_handler = _LastLogHandler()
    log_handler.setLevel(logging.DEBUG)
    logging.getLogger().addHandler(log_handler)
    _register_page(orchestrator, log_handler)
    thread = threading.Thread(
        target=ui.run,
        kwargs=dict(host=host, port=port, title='APM Control',
                    dark=True, reload=False, show=False),
        daemon=True,
        name='nicegui',
    )
    thread.start()
    log.info(f'Web interface started at http://{host}:{port}')

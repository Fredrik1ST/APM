'''
NiceGUI web interface for the Autonomous Pacemaker.

Passive UI layer: 
reads state from and calls methods on the Orchestrator instance it receives.

Started as a daemon thread by Orchestrator.run().
'''

import threading
import logging
from nicegui import ui, app as nicegui_app
from fastapi.responses import Response

import apm.config_handler as config
from apm.orchestrator import Orchestrator, State, Mode
from apm.control.pacing_profile import PacingProfile, available_profiles, PROFILES_DIR
from apm.vision.frame_encoder import encode_jpeg

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
    Mode.CONSTANT_SPEED_CLOSED: 'Constant speed (closed loop)',
    Mode.CAMERA_TEST:       'Camera test (both)',
    Mode.CAMERA_TEST_FRONT: 'Camera test (front)',
    Mode.CAMERA_TEST_BACK:  'Camera test (back)',
    Mode.GNSS_TEST:         'GNSS test',
    Mode.ARDUINO_TEST:      'Arduino test',
    Mode.LANE_KEEPER_TEST:  'Lane keeper test'
}

_ACTIVE_STATES = {State.STARTING, State.RUNNING, State.RUNNING_PACE,
                  State.RUNNING_DIST, State.RUNNING_CONST, State.STOPPING}


# ---------------------------------------------------------------------------
# Pacing profile chart
# ---------------------------------------------------------------------------

def _profile_chart_options(profile: PacingProfile) -> dict:
    '''ECharts option dict plotting cumulative distance and the speed steps over time.'''
    dist_t, dist_d = profile.distance_curve()
    spd_t, spd_v = profile.speed_curve()
    green, blue, grey = '#4ade80', '#60a5fa', '#aaa'
    return {
        'darkMode': True,
        'backgroundColor': 'transparent',
        'grid': {'left': 55, 'right': 55, 'top': 40, 'bottom': 45},
        'tooltip': {'trigger': 'axis'},
        'legend': {'data': ['Distance', 'Speed'], 'textStyle': {'color': '#ccc'}},
        'xAxis': {
            'type': 'value', 'name': 'time (s)', 'nameLocation': 'middle', 'nameGap': 28,
            'axisLabel': {'color': grey}, 'nameTextStyle': {'color': grey},
        },
        'yAxis': [
            {'type': 'value', 'name': 'distance (m)', 'position': 'left',
             'axisLabel': {'color': green}, 'nameTextStyle': {'color': green}},
            {'type': 'value', 'name': 'speed (m/s)', 'position': 'right',
             'axisLabel': {'color': blue}, 'nameTextStyle': {'color': blue},
             'splitLine': {'show': False}},
        ],
        'series': [
            {'name': 'Distance', 'type': 'line', 'yAxisIndex': 0, 'showSymbol': False,
             'lineStyle': {'color': green, 'width': 2}, 'areaStyle': {'opacity': 0.08, 'color': green},
             'data': [[round(t, 3), round(d, 3)] for t, d in zip(dist_t, dist_d)]},
            {'name': 'Speed', 'type': 'line', 'step': 'end', 'yAxisIndex': 1, 'showSymbol': False,
             'lineStyle': {'color': blue, 'width': 2},
             'data': [[round(t, 3), round(v, 3)] for t, v in zip(spd_t, spd_v)]},
        ],
    }


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
                with ui.row().classes('items-start gap-3 w-full'):
                    ui.label(key).classes('text-sm w-40 shrink-0 font-mono break-all')
                    inp = ui.input(value=str(value)).classes('grow min-w-0 font-mono text-sm')
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
        # grid-cols-1 on narrow screens (mobile), 2 columns from 768px up (md:)
        with ui.element('div').classes('w-full gap-4 p-4 grid grid-cols-1 md:grid-cols-2'):

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

        # --- pacing profile -----------------------------------------------
        with ui.card().classes('mx-4 mb-4 self-stretch'):
            with ui.row().classes('items-center gap-3 w-full'):
                ui.icon('timeline', size='md', color='green')
                ui.label('Pacing profile').classes('text-lg font-semibold')
                ui.space()
                prof_options = {p.name: p.name for p in available_profiles()}
                _current = config.load().get('cruise_control', {}).get('pacing_profile')
                _initial = _current if _current in prof_options else next(iter(prof_options), None)
                profile_select = ui.select(
                    prof_options, value=_initial, label='CSV',
                ).classes('min-w-48').props('outlined dense')

            profile_chart = ui.echart({}).classes('w-full').style('height: 300px')
            profile_summary = ui.label().classes('text-xs text-gray-400 font-mono')

            def show_profile(name: str | None) -> None:
                profile_chart.options.clear()
                if not name:
                    profile_chart.update()
                    profile_summary.set_text('No pacing profiles found in pacing_profiles/.')
                    return
                try:
                    profile = PacingProfile.from_csv(PROFILES_DIR / name)
                except (OSError, ValueError) as e:
                    profile_chart.update()
                    profile_summary.set_text(f'Could not load {name}: {e}')
                    return
                profile_chart.options.update(_profile_chart_options(profile))
                profile_chart.update()
                profile_summary.set_text(
                    f'{len(profile.segments)} segments · {profile.total_duration:.0f} s '
                    f'· {profile.total_distance:.0f} m total'
                )

            def on_profile_change(e) -> None:
                doc = config.load()
                doc['cruise_control']['pacing_profile'] = e.value
                config.save(doc)
                show_profile(e.value)
                ui.notify(f'Pacing profile: {e.value}', type='positive', position='bottom-right')

            profile_select.on_value_change(on_profile_change)
            show_profile(_initial)

        # --- camera feeds -------------------------------------------------
        def _camera_card(label: str, src: str):
            with ui.card().classes('col-span-1'):
                ui.label(label).classes('text-lg font-semibold mb-2')
                img = ui.image(src).classes('w-full rounded')
                placeholder = (
                    ui.element('div')
                    .classes('w-full rounded flex items-center justify-center bg-gray-800 text-gray-500 text-sm')
                    .style('aspect-ratio: 16/9')
                )
                with placeholder:
                    ui.icon('videocam_off', size='sm').classes('mr-1')
                    ui.label('No signal')
                placeholder.set_visibility(False)
            return img, placeholder

        with ui.element('div').classes('w-full gap-4 px-4 grid grid-cols-1 md:grid-cols-2'):
            front_img, front_placeholder = _camera_card('Front camera', '/camera/front')
            back_img,  back_placeholder  = _camera_card('Back camera',  '/camera/back')

        # --- config editor ------------------------------------------------
        # mx-4 matches the grid padding; w-full omitted to avoid overflow (margins are outside 100%)
        with ui.card().classes('mx-4 mb-4 self-stretch'):
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
                        with config_col:
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
        _refresh_counter = [0]

        def refresh_ui() -> None:
            _refresh_counter[0] += 1
            t = _refresh_counter[0]
            for img, placeholder, ok, path in (
                (front_img, front_placeholder, orchestrator.front_camera.ok, 'front'),
                (back_img,  back_placeholder,  orchestrator.back_camera.ok,  'back'),
            ):
                img.set_visibility(ok)
                placeholder.set_visibility(not ok)
                if ok:
                    img.set_source(f'/camera/{path}?t={t}')

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
# Camera image routes
# ---------------------------------------------------------------------------

def _register_camera_routes(orchestrator: Orchestrator) -> None:
    _JPEG_HEADERS = {'Cache-Control': 'no-store'}

    def _encode(image):
        cfg = orchestrator.cfg.get('webapp', {})
        return encode_jpeg(image, quality=cfg.get('jpeg_quality', 80), scale=cfg.get('jpeg_scale', 0.5))

    @nicegui_app.get('/camera/front')
    def camera_front():
        if orchestrator.front_image:
            return Response(orchestrator.front_image, media_type='image/jpeg', headers=_JPEG_HEADERS)
        snap = orchestrator.front_camera.get_snapshot()
        if snap is None or snap.image is None:
            return Response(status_code=204)
        return Response(_encode(snap.image), media_type='image/jpeg', headers=_JPEG_HEADERS)

    @nicegui_app.get('/camera/back')
    def camera_back():
        if orchestrator.back_image:
            return Response(orchestrator.back_image, media_type='image/jpeg', headers=_JPEG_HEADERS)
        snap = orchestrator.back_camera.get_snapshot()
        if snap is None or snap.image is None:
            return Response(status_code=204)
        return Response(_encode(snap.image), media_type='image/jpeg', headers=_JPEG_HEADERS)


# ---------------------------------------------------------------------------
# Public API - called by Orchestrator
# ---------------------------------------------------------------------------

def start(orchestrator: Orchestrator, host: str = '0.0.0.0', port: int = 8080) -> None:
    '''Register the UI page and start NiceGUI on a daemon thread.'''
    log_handler = _LastLogHandler()
    log_handler.setLevel(logging.DEBUG)
    logging.getLogger().addHandler(log_handler)
    _register_camera_routes(orchestrator)
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

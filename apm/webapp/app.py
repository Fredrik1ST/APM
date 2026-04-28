'''
Minimal Flask webapp for starting/stopping the APM and editing its config.

Run:
    python -m apm.webapp.app
Then open http://localhost:5000
'''

import signal
import subprocess
import sys
from pathlib import Path

from flask import Flask, jsonify, render_template, request

from apm import config_handler

app = Flask(__name__)

_process: subprocess.Popen | None = None
REPO_ROOT = Path(__file__).resolve().parents[2]


def _is_running() -> bool:
    return _process is not None and _process.poll() is None


@app.get("/")
def index():
    return render_template("index.html")


@app.get("/status")
def status():
    return jsonify(running=_is_running())


@app.post("/start")
def start():
    global _process
    if _is_running():
        return jsonify(running=True, message="Already running"), 409
    _process = subprocess.Popen(
        [sys.executable, "-m", "apm.main"],
        cwd=REPO_ROOT,
    )
    return jsonify(running=True, pid=_process.pid)


@app.post("/stop")
def stop():
    global _process
    if not _is_running():
        return jsonify(running=False, message="Not running"), 409
    _process.send_signal(signal.SIGTERM)
    try:
        _process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        _process.kill()
        _process.wait()
    _process = None
    return jsonify(running=False)


@app.get("/config")
def get_config():
    return jsonify(text=config_handler.read_text())


@app.post("/config")
def post_config():
    text = request.get_json(force=True).get("text", "")
    try:
        config_handler.write_text(text)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 400
    return jsonify(ok=True)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)

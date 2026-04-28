'''
Reads and writes configuration parameters for the APM.

'''

from pathlib import Path
import tomllib

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "default.toml"


def read_text() -> str:
    return CONFIG_PATH.read_text()


def write_text(text: str) -> None:
    tomllib.loads(text)  # validate before writing
    CONFIG_PATH.write_text(text)


def load() -> dict:
    with CONFIG_PATH.open("rb") as f:
        return tomllib.load(f)

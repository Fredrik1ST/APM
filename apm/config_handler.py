'''
Reads and writes configuration parameters for the APM.

On first startup, creates a user-editable settings.toml from the default.toml template in the config folder.
The rest of the program uses settings.toml, allowing users to customize parameters.

'''

from pathlib import Path
import tomlkit
import shutil

# Define paths for config files
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "default.toml"
SETTINGS_PATH = PROJECT_ROOT / "config" / "settings.toml"


def initialize() -> None:
    """Create user settings "settings.toml" from "default.toml" if not already done."""
    if not SETTINGS_PATH.exists():
        reset_to_defaults()


def read_text() -> str:
    """Read the user-editable settings file."""
    return SETTINGS_PATH.read_text()


def write_text(text: str) -> None:
    """Write and validate settings file."""
    tomlkit.loads(text)  # validate before writing
    SETTINGS_PATH.write_text(text)


def load() -> tomlkit.TOMLDocument:
    """Load user settings."""
    return tomlkit.loads(SETTINGS_PATH.read_text())


def save(config: tomlkit.TOMLDocument) -> None:
    """Write modified settings back to file."""
    SETTINGS_PATH.write_text(tomlkit.dumps(config))


def reset_to_defaults() -> None:
    """Reset user settings to default values."""
    shutil.copy(DEFAULT_CONFIG_PATH, SETTINGS_PATH)

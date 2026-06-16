''' Configuration file for pytest '''


# --- Add current directory to path so that pytest can find the modules to test ---
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))


# --- Create default input / output directory for test data (images, figures, etc.) ---
from pathlib import Path
import pytest

INPUT_DIR = Path(__file__).resolve().parent / 'tests' / 'input'
INPUT_DIR.mkdir(exist_ok=True)

INPUT_DIR = Path(__file__).resolve().parent / 'tests' / 'input' / 'img'
INPUT_DIR.mkdir(exist_ok=True)

INPUT_DIR = Path(__file__).resolve().parent / 'tests' / 'csv'
INPUT_DIR.mkdir(exist_ok=True)

OUTPUT_DIR = Path(__file__).resolve().parent / 'tests' / 'output'
OUTPUT_DIR.mkdir(exist_ok=True)

@pytest.fixture(scope='session', autouse=True)
def setup_input_dir():
    return INPUT_DIR

@pytest.fixture(scope='session', autouse=True)
def setup_output_dir():
    return OUTPUT_DIR


#  --- Hooks (optional command line arguments for tests) ---

def pytest_addoption(parser):
    parser.addoption("--plot", action="store_true", default=False)

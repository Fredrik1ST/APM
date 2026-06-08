''' Configuration file for pytest '''


# --- Add current directory to path so that pytest can find the modules to test ---
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))


# --- Create default output directory for figures etc. ---
from pathlib import Path
import pytest
OUTPUT_DIR = Path(__file__).resolve().parent / 'tests' / 'output'
OUTPUT_DIR.mkdir(exist_ok=True)

@pytest.fixture(scope='session', autouse=True)
def setup_output_dir():
    return OUTPUT_DIR


#  --- Hooks (optional command line arguments for tests) ---

def pytest_addoption(parser):
    parser.addoption("--plot", action="store_true", default=False)

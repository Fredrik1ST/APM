''' Configuration file for pytest '''


# --- Add current directory to path so that pytest can find the modules to test ---
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))


#  --- Hooks (optional command line arguments for tests) ---

def pytest_addoption(parser):
    parser.addoption("--plot", action="store_true", default=False)

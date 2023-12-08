"""Path hack to put main dir in sys.path"""

import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

name = "kinematic_car"

FILE_PATH = Path(__file__).resolve()
PROJECT_DIR = FILE_PATH.parents[2]
DATA_DIR = PROJECT_DIR / "results" / name
# MODEL_DIR = DATA_DIR / "model"
# MEMORY_DIR = DATA_DIR / "memory"

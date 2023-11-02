"""Path hack to put main dir in sys.path"""

import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

FILE_PATH = Path(__file__).resolve()
PROJECT_DIR = FILE_PATH.parents[1]

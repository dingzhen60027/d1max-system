#!/usr/bin/env python3
"""Run the standalone, YAML-configured structure-preserving PCD pipeline."""
from pathlib import Path
import sys

# The pipeline package lives in tools/, one level above this entry point.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pointcloud_preprocessing.runner import main  # noqa: E402

if __name__ == '__main__':
    main()

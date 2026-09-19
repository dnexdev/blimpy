"""Lets tools/calib/*.py import the laptop package when run from anywhere."""
import pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

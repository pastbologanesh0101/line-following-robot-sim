"""Ensure the repo root is importable as ``lfr_sim`` regardless of how
pytest is invoked (flat, src-less package layout)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

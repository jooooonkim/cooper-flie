from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CACHE_DIR = ROOT / "cache"  # cflib parameter/log TOC cache
LOG_DIR = ROOT / "logs"
MODEL_PATH = ROOT / "models" / "hand_landmarker.task"

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CACHE_DIR = ROOT / "cache"
LOG_DIR = ROOT / "logs"


def load_config(path=None):
    with open(path or ROOT / "config.yaml", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve(path):
    """Paths in config.yaml are relative to the project root."""
    path = Path(path)
    return path if path.is_absolute() else ROOT / path

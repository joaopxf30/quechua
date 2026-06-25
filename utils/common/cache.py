import hashlib
import json
from pathlib import Path
from typing import Any

# Resolves to the project root regardless of where the script is run from
CACHE_DIR = Path(__file__).resolve().parent.parent.parent / ".cache"


def _params_hash(params: dict) -> str:
    """Stable, deterministic hash of a parameter dict."""
    canonical = json.dumps(params, sort_keys=True, default=str)
    return hashlib.sha256(canonical.encode()).hexdigest()[:16]


def load(name: str, params: dict) -> Any | None:
    """Return cached data if a matching file exists, otherwise None."""
    path = CACHE_DIR / f"{name}_{_params_hash(params)}.json"
    if not path.exists():
        return None
    with path.open() as f:
        return json.load(f)["data"]


def save(name: str, params: dict, data: Any) -> None:
    """Persist data alongside the params that produced it (useful for debugging)."""
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    path = CACHE_DIR / f"{name}_{_params_hash(params)}.json"
    with path.open("w") as f:
        json.dump({"params": params, "data": data}, f, indent=2)
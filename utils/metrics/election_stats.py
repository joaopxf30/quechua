# ---------------------------------------------------------------------------
# Election statistics registry
# ---------------------------------------------------------------------------
# Written by ElectionMixin during simulation; read by main.py post-simulation.

from typing import Dict, List
from dataclasses import dataclass, field


@dataclass
class ElectionEvent:
    """Record of a single leader-election round."""
    election_id: int
    trigger: str              # "initial" | "split" | "merge"
    start_time: float         # simulation timestamp when election began
    end_time: float = 0.0     # simulation timestamp when leader was accepted
    duration: float = 0.0     # end_time - start_time
    winner_id: int = -1
    participants: List[int] = field(default_factory=list)
    buffer_sizes: Dict[int, int] = field(default_factory=dict)
    # uav_id → total packets buffered during election


ELECTION_LOG: List[ElectionEvent] = []

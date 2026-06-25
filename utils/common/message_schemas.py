# ---------------------------------------------------------------------------
# Message schemas
# ---------------------------------------------------------------------------
from typing import TypedDict, Dict

class SimpleMessage(TypedDict):
    """Basic message type used for template classes"""
    packets:       Dict[str, int]   # sensor_name → packet count
    sender_type:   int
    sender_id:     int
    lamport:       int              # sender's Lamport time at send
    
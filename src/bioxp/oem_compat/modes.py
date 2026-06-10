from enum import Enum


class RuntimeMode(str, Enum):
    DRY_RUN = "dry_run"
    SHADOW = "shadow"
    LIVE = "live"

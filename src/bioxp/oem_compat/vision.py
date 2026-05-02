from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VisionResult:
    operation: str
    status: str = "unavailable"
    reason: str = "linux_oem_compat_dry_run_no_camera_backend"


class VisionFacade:
    def check_camera(self) -> VisionResult:
        return VisionResult("check_camera")

    def snapshot_image(self) -> VisionResult:
        return VisionResult("snapshot_image")

    def scan_barcode(self) -> VisionResult:
        return VisionResult("scan_barcode")

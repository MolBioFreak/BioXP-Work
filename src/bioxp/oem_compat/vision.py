from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VisionResult:
    operation: str
    oem_method: str
    status: str = "unavailable"
    reason: str = "linux_oem_compat_dry_run_no_camera_backend"


class VisionFacade:
    @classmethod
    def dry_run(cls) -> "VisionFacade":
        return cls()

    def check_camera(self) -> VisionResult:
        return VisionResult("check_camera", "isOpened")

    def snapshot_image(self) -> VisionResult:
        return VisionResult("snapshot_image", "SaveImage")

    def scan_barcode(self) -> VisionResult:
        return VisionResult("scan_barcode", "ScanBarcode")

    def set_gain(self, gain: float) -> VisionResult:
        return VisionResult(f"set_gain:{gain}", "setGain")

    def set_exposure(self, exposure: float) -> VisionResult:
        return VisionResult(f"set_exposure:{exposure}", "setExposure")

    def inspect_pool_plate(self) -> VisionResult:
        return VisionResult("inspect_pool_plate", "checkPoolPlate")

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class VisionResult:
    operation: str
    oem_method: str
    status: str = "unavailable"
    reason: str = "linux_oem_compat_dry_run_no_camera_backend"
    location_id: str | None = None
    artifact_contract: dict = field(default_factory=lambda: {
        "image_required": True,
        "failure_is_semantic": True,
        "include_confidence": True,
        "preserve_raw_frame": True,
    })


class VisionFacade:
    @classmethod
    def dry_run(cls) -> "VisionFacade":
        return cls()

    def check_camera(self, *, location_id: str | None = None) -> VisionResult:
        return VisionResult("check_camera", "isOpened", location_id=location_id)

    def snapshot_image(self, *, location_id: str | None = None) -> VisionResult:
        return VisionResult("snapshot_image", "SaveImage", location_id=location_id)

    def scan_barcode(self, *, location_id: str | None = None) -> VisionResult:
        return VisionResult("scan_barcode", "ScanBarcode", location_id=location_id)

    def set_gain(self, gain: float) -> VisionResult:
        return VisionResult(f"set_gain:{gain}", "setGain", artifact_contract={"image_required": False, "failure_is_semantic": True})

    def set_exposure(self, exposure: float) -> VisionResult:
        return VisionResult(f"set_exposure:{exposure}", "setExposure", artifact_contract={"image_required": False, "failure_is_semantic": True})

    def inspect_pool_plate(self, *, location_id: str | None = None) -> VisionResult:
        return VisionResult("inspect_pool_plate", "checkPoolPlate", location_id=location_id)

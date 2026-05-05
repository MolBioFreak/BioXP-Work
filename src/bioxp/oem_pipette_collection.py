from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


OEM_FULL_INIT_SOURCE_COVERAGE = {
    "ControlLib.initializeMotion": "ControlLib.cs:8797-8856",
    "ClassPipetteCollection.initiateGroup": "ClassPipetteCollection.cs:677-693",
    "ClassPipetteCollection.checkedPipetteStatus": "ClassPipetteCollection.cs:726-748",
    "ClassPipetteCollection.ejectAllTips": "ClassPipetteCollection.cs:1176-1235",
    "ClassPipetteCollection.verifyEjectTip": "ClassPipetteCollection.cs:1265-1323",
    "ClassPipetteCollection.queryTipStatus": "ClassPipetteCollection.cs:1336-1357",
    "ClassPipette.QueryTipStatus": "ClassPipette.cs:571-589",
    "ClassPipette.QueryPressure": "ClassPipette.cs:622-628",
    "ControlLib.inspectCover": "ControlLib.cs:3663-3768",
    "ClassFrameGrabber.ScanBarcode": "ClassFrameGrabber.cs:125",
    "ClassFrameGrabber.CamCalibration": "ClassFrameGrabber.cs:4292",
    "ClassFrameGrabber.locateCover": "ClassFrameGrabber.cs:4578",
    "ClassFrameGrabber.checkPoolPlate": "ClassFrameGrabber.cs:6485",
    "ClassFrameGrabber.checkBioSecurityCover": "ClassFrameGrabber.cs:11918",
    "ClassFrameGrabber.adjustFocus": "ClassFrameGrabber.cs:12273",
}


@dataclass(frozen=True)
class OemPipetteChannelState:
    channel: int
    tip_loaded: bool

    def to_payload(self) -> dict:
        return {"channel": int(self.channel), "tip_loaded": bool(self.tip_loaded)}


class OemPipetteCollection:
    """Dry-run, OEM-shaped ClassPipetteCollection startup-cleanup model.

    This class intentionally models source order and artifacts only. It does not
    send CAN frames or claim live readiness.
    """

    def __init__(self, channels: Iterable[OemPipetteChannelState]):
        self.channels = list(channels)
        if len(self.channels) != 4:
            raise ValueError("OEM ClassPipetteCollection requires exactly four channels")

    @classmethod
    def dry_run(cls, *, tip_loaded: tuple[bool, bool, bool, bool] | list[bool] | None = None) -> "OemPipetteCollection":
        loaded = tuple(bool(v) for v in (tip_loaded if tip_loaded is not None else (False, False, False, False)))
        if len(loaded) != 4:
            raise ValueError("tip_loaded must contain four channel states")
        return cls(OemPipetteChannelState(channel=i, tip_loaded=loaded[i]) for i in range(4))

    def _tip_channels(self) -> list[int]:
        return [row.channel for row in self.channels if row.tip_loaded]

    def plan_initialize_motion_cleanup(self, *, mode: str = "dry_run") -> dict:
        targeted = self._tip_channels()
        dry_run = True
        return {
            "ok": False,
            "live_ready": False,
            "dry_run": dry_run,
            "mode": mode,
            "artifact_format": "bioxp-oem-pipette-init-v1",
            "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ControlLib.initializeMotion"],
            "source_coverage": dict(OEM_FULL_INIT_SOURCE_COVERAGE),
            "channels": [row.to_payload() for row in self.channels],
            "steps": [
                {
                    "operation": "query_tip_status_before",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipetteCollection.queryTipStatus"],
                    "pipette_command_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipette.QueryTipStatus"],
                    "tip_count": len(targeted),
                    "channels_with_tips": targeted,
                    "physical_motion": False,
                },
                {
                    "operation": "open_thermal_door",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ControlLib.initializeMotion"],
                    "physical_motion": not dry_run,
                },
                {
                    "operation": "scriptmove_to_waste_bin",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ControlLib.initializeMotion"],
                    "physical_motion": not dry_run,
                },
                {
                    "operation": "eject_all_tips",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipetteCollection.ejectAllTips"],
                    "channels_targeted": targeted,
                    "physical_motion": not dry_run,
                },
                {
                    "operation": "move_z_80000",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ControlLib.initializeMotion"],
                    "physical_motion": not dry_run,
                },
                {
                    "operation": "move_x_79000",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ControlLib.initializeMotion"],
                    "physical_motion": not dry_run,
                },
                {
                    "operation": "query_tip_status_after",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipetteCollection.queryTipStatus"],
                    "pipette_command_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipette.QueryTipStatus"],
                    "expected_tip_count": 0,
                    "physical_motion": False,
                },
                {
                    "operation": "clear_tip_machine_state",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipetteCollection.verifyEjectTip"],
                    "physical_motion": False,
                },
                {
                    "operation": "initiate_group",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipetteCollection.initiateGroup"],
                    "physical_motion": False,
                },
                {
                    "operation": "checked_pipette_status",
                    "source_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipetteCollection.checkedPipetteStatus"],
                    "pressure_anchor": OEM_FULL_INIT_SOURCE_COVERAGE["ClassPipette.QueryPressure"],
                    "retry_once_on_failure": True,
                    "physical_motion": False,
                },
            ],
            "hardware_truth_level": "dry_run_model_no_transport",
            "planned_physical_motion_if_live_oem_bound": True,
            "blockers": ["pipette_can_shadow_proof_required_before_live"],
            "reason": "dry-run OEM initializeMotion pipette cleanup plan only; live CAN ACK/readback not yet proven",
        }


def dry_run_initialize_motion_pipette_cleanup(*, mode: str = "dry_run", tip_loaded: tuple[bool, bool, bool, bool] | list[bool] | None = None) -> dict:
    return OemPipetteCollection.dry_run(tip_loaded=tip_loaded).plan_initialize_motion_cleanup(mode=mode)

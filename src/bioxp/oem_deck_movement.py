from __future__ import annotations

from contextlib import contextmanager, nullcontext
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
import threading
from typing import Any, Callable, Iterator, Mapping

from .oem_deck_catalog import DeckCatalog, configured_location_names
from .oem_compat.position_table import PositionTable, well_id_from_label


def _controller_terminal_truth(result: Mapping[str, Any]) -> bool:
    """Require acknowledged source motion and non-contradictory terminal proof."""
    if result.get("source_noop") is True:
        return result.get("delivery_attempted") is False
    if (
        result.get("controller_command_acknowledged") is not True
        or result.get("controller_completion_verified") is not True
    ):
        return False
    subordinate: list[Mapping[str, Any]] = []
    primitive = result.get("primitive_result")
    if isinstance(primitive, Mapping):
        subordinate.append(primitive)
    children = result.get("source_children")
    if isinstance(children, list):
        for child in children:
            child_result = child.get("result") if isinstance(child, Mapping) else None
            if isinstance(child_result, Mapping) and any(
                key in child_result
                for key in (
                    "controller_command_acknowledged",
                    "controller_terminal_state_verified",
                    "controller_completion_verified",
                )
            ):
                subordinate.append(child_result)
    for row in subordinate:
        if "controller_command_acknowledged" in row and row.get("controller_command_acknowledged") is not True:
            return False
        if "controller_terminal_state_verified" in row and row.get("controller_terminal_state_verified") is not True:
            return False
        if "controller_completion_verified" in row and row.get("controller_completion_verified") is not True:
            return False
    return True


# plateName.cs:3-26 exact finite enum; ordinals are movement branch authority.
OEM_PLATE_NAME_ORDINALS: dict[str, int] = {
    "POOL_PLATE": 0,
    "OUTPUT_PLATE": 1,
    "REAGENT_PLATE": 2,
    "BIO_SECURITY_COVER": 3,
    "OUTPUT_COVER": 4,
    "REAGENT_COVER": 5,
    "TIP_TRAY": 6,
    "STRIP_ONE": 7,
    "STRIP_TWO": 8,
    "STRIP_THREE": 9,
    "STRIP_FOUR": 10,
    "TROUGH": 11,
    "SYNTHESIS_PLATE": 12,
    "OLIGO_QUANTITATION_PLATE": 13,
    "GENE_QUANTITATION_PLATE": 14,
    "REF_QUANTITATION_PLATE": 15,
    "ELUTION_PLATE": 16,
    "ACCUMULATION_PLATE": 17,
    "TIP_HOTEL": 18,
    "TFF_REAGENT_BLOCK": 19,
    "VOLUME_CALCULATION": 20,
    "WASTE_BIN": 21,
}
_OEM_PLATE_ORDINALS = frozenset(OEM_PLATE_NAME_ORDINALS.values())
_OEM_PLATE_NAMES_BY_ORDINAL = {value: key for key, value in OEM_PLATE_NAME_ORDINALS.items()}

OEM_SCRIPT_PLATE_TOKENS: Mapping[str, int] = {
    "PL_POOL": 0, "PL_OUTPUT": 1, "PL_REAGENT": 2,
    "TROUGH": 11, "STRIP1": 7, "STRIP2": 8, "STRIP3": 9, "STRIP4": 10,
    "CV_REAGENT": 5, "CV_OUTPUT": 4, "CV_BIOSECURITY": 3,
    "PL_SYNTHESIS": 12, "PL_ELUTION": 16, "PL_OLIGO_QUANT": 13,
    "PL_GENE_QUANT": 14, "PL_TFF_REAGENT": 19, "PL_ACC": 17,
}


def translate_oem_plate_move(
    plate_token: str, target_token: str, mode_token: str | None,
) -> dict[str, Any]:
    """Translate source MP/MC tokens through ClassGlobals' exact lookup rules."""
    if plate_token not in OEM_SCRIPT_PLATE_TOKENS:
        raise ValueError("No plate identified")
    plate = OEM_SCRIPT_PLATE_TOKENS[plate_token]
    cover = plate in {3, 4, 5}
    aliases = {
        "LOC_BSCS": 4,
        "LOC_TC": 5 if cover else 23,
        "LOC_RC": 19 if cover else 32,
        "LOC_RCS": 20,
        "LOC_OC": 17 if cover else 21,
        "LOC_OCS": 18,
        "LOC_MS": 25,
    }
    if target_token in aliases:
        destination = aliases[target_token]
    else:
        from .oem_compat.pathing import LOCATION_NAME_TO_ID
        if target_token not in LOCATION_NAME_TO_ID:
            raise ValueError("no location identified")
        destination = int(LOCATION_NAME_TO_ID[target_token])
    return {
        "plate": plate,
        "destination": destination,
        "press_plate": mode_token == "PRESS",
    }


def serial206_position_table_ordinals(table_or_rows: Any) -> frozenset[int]:
    """Return only ordinals backed by authoritative PositionTable rows."""
    from .oem_compat.pathing import LOCATION_NAME_TO_ID

    rows_reader = getattr(table_or_rows, "rows", None)
    if callable(rows_reader):
        rows = rows_reader()
    elif isinstance(table_or_rows, Mapping):
        rows = table_or_rows.values()
    else:
        raise TypeError("PositionTable rows are unavailable")
    ordinals: set[int] = set()
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        location_name = str(row.get("location_id") or "").strip().upper()
        ordinal = LOCATION_NAME_TO_ID.get(location_name)
        if ordinal is not None:
            ordinals.add(int(ordinal))
    return frozenset(ordinals)


def require_serial206_machine_target(target: int, table_or_rows: Any) -> None:
    if type(target) is not int or target not in serial206_position_table_ordinals(table_or_rows):
        raise RuntimeError(f"machine_target_absent_from_serial206_position_table:{target}")


def compiled_wp8_machine_targets(plan_or_child: Mapping[str, Any]) -> frozenset[int]:
    """Enumerate every PositionTable target emitted at a WP8 provider boundary."""
    rows = list(plan_or_child.get("children") or [plan_or_child])
    targets: set[int] = set()
    for child in rows:
        if not isinstance(child, Mapping):
            continue
        operation = str(child.get("operation") or "")
        arguments = child.get("arguments")
        if isinstance(arguments, Mapping):
            for key in ("destination", "location", "location_id", "pressure_target"):
                value = arguments.get(key)
                if type(value) is int:
                    targets.add(value)
        if operation == "parkGantry":
            targets.add(28)
        elif operation in {"scriptmoveToWaste", "cleanupWastePrelude"}:
            targets.add(6)
    return frozenset(targets)


def validate_compiled_wp8_machine_targets(
    plan_or_child: Mapping[str, Any], table_or_rows: PositionTable | Mapping[str, Any],
) -> None:
    for target in sorted(compiled_wp8_machine_targets(plan_or_child)):
        require_serial206_machine_target(target, table_or_rows)


def canonical_plate_name(value: Any) -> int | None:
    """Return the exact OEM ordinal, preserving nullable constructor state."""
    if value is None:
        return None
    if type(value) is int and value in _OEM_PLATE_ORDINALS:
        return value
    if type(value) is str and value in OEM_PLATE_NAME_ORDINALS:
        return OEM_PLATE_NAME_ORDINALS[value]
    raise ValueError("plate_on_gantry is outside the source domain")


def plate_name_for_storage(value: Any) -> str | None:
    ordinal = canonical_plate_name(value)
    return None if ordinal is None else _OEM_PLATE_NAMES_BY_ORDINAL[ordinal]


# ClassMachineStatus.cs:419-453 exact constructor-owned movable state.
OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS: dict[str, str] = {
    "POOL_PLATE": "LOC_P_TC",          # locationID 23
    "OUTPUT_PLATE": "LOC_P_OC",        # locationID 21
    "REAGENT_PLATE": "LOC_RC",         # locationID 3
    "STRIP_ONE": "LOC_STRIP1",         # locationID 11
    "STRIP_TWO": "LOC_STRIP2",         # locationID 12
    "STRIP_THREE": "LOC_STRIP3",       # locationID 13
    "STRIP_FOUR": "LOC_STRIP4",        # locationID 14
    "REAGENT_COVER": "LOC_RC_COVER_STORAGE",  # locationID 20
    "OUTPUT_COVER": "LOC_OC_COVER_STORAGE",   # locationID 18
    "BIO_SECURITY_COVER": "LOC_BSC",   # locationID 5
    "TROUGH": "LOC_TROUGH",            # locationID 16
}


def canonical_movable_object_locations(value: Any) -> dict[str, str]:
    if not isinstance(value, Mapping) or set(value) != set(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS):
        raise ValueError("movable plate state is not authoritative")
    locations = configured_location_names()
    canonical: dict[str, str] = {}
    for key, location in value.items():
        if type(key) is not str or type(location) is not str or location not in locations:
            raise ValueError("movable plate state is not authoritative")
        canonical[key] = location
    if len(set(canonical.values())) != len(canonical):
        raise ValueError("movable plate state is not authoritative")
    return canonical


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _digest(value: Any) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


@dataclass(frozen=True)
class DeckAuthoritySnapshot:
    ownership_generation: int
    provider_owner_id: str
    board_epoch_4: int
    board_epoch_5: int
    position_table_sha256: str
    machine_state_revision: int
    reference_versions: Mapping[str, int]
    safety_epochs: Mapping[str, int]
    latch_observation_id: str
    controller_position_observation_id: str
    captured_at: float
    current_x: int
    current_y: int
    current_z: int
    current_location_id: str
    current_well_id: int
    tip_loaded: bool
    tip_dirty: bool
    tip_location: int
    clean_path: bool
    plate_on_gantry: int | str | None
    pseudo_z_home: int
    device_type: str
    latch_status: bool
    machine_latch_closed: bool
    semantic_state_provenance_digest: str | None = None

    def __post_init__(self) -> None:
        if type(self.captured_at) not in {int, float} or not math.isfinite(float(self.captured_at)):
            raise ValueError("captured_at must be finite")
        if set(self.reference_versions) != {"x", "y", "z", "g"}:
            raise ValueError("reference versions must contain x,y,z,g")
        if set(self.safety_epochs) != {"global", "x", "y", "z"}:
            raise ValueError("safety epochs must contain global,x,y,z")
        if self.tip_loaded and not 0 <= self.tip_location <= 3:
            raise ValueError("loaded tip requires a valid tip_location")
        if len(self.position_table_sha256) != 64:
            raise ValueError("invalid PositionTable digest")
        object.__setattr__(self, "plate_on_gantry", canonical_plate_name(self.plate_on_gantry))

    @property
    def digest(self) -> str:
        authoritative = asdict(self)
        return _digest({"schema_version": "bioxp.oem_deck_authority.v1", **authoritative})


@dataclass(frozen=True)
class NamedLocationIntent:
    target: str
    camera_offset: bool = False


@dataclass(frozen=True)
class DeckPlanStep:
    order: int
    operation: str
    source_anchor: str
    resources: tuple[str, ...] = ()
    arguments: Mapping[str, Any] | None = None


@dataclass(frozen=True)
class DeckMovementPlan:
    target: str
    target_label: str
    resolved_location_id: int
    source_branch: str
    authority_digest: str
    position_table_sha256: str
    catalog_revision: str
    steps: tuple[DeckPlanStep, ...]
    semantic_transition: Mapping[str, Any]
    blocked_reason: str | None
    source_hazards: tuple[str, ...] = ()

    @property
    def plan_digest(self) -> str:
        payload = asdict(self)
        payload.pop("blocked_reason", None)
        return _digest(payload)


def compile_named_location(intent: NamedLocationIntent, catalog: DeckCatalog, table: PositionTable, authority: DeckAuthoritySnapshot) -> DeckMovementPlan:
    destination = catalog.resolve(intent.target)
    if table.digest != authority.position_table_sha256 or table.digest != catalog.position_table_sha256:
        raise ValueError("position_table_authority_mismatch")
    if destination.branch in {"barcode", "park"} and intent.camera_offset:
        raise ValueError("contradictory_input")
    steps = [
        DeckPlanStep(0, "ForceToHighHome", "DefaultParameters.ForceToHighHome", arguments={"pseudo_z_home": 500}),
        DeckPlanStep(1, "check_latch_status", "ClassControlInterface.btnLOC1_Click:m_latchStatus"),
        DeckPlanStep(2, "check_machine_latch_closed", "ClassControlInterface.btnLOC1_Click:MachineStatus.LatchClosed"),
    ]
    blocked = None
    if not authority.latch_status or not authority.machine_latch_closed:
        blocked = "latch_not_closed"
    elif destination.branch == "park":
        steps.append(DeckPlanStep(3, "parkGantry", "ClassControlInterface.btnLOC1_Click:Park", ("x", "y", "z", "g")))
    elif destination.branch == "barcode":
        steps.extend((
            DeckPlanStep(3, "moveTo", "ClassControlInterface.btnLOC1_Click:barcode", ("x", "y", "z"), {"location_id": destination.location_id, "camera_offset": True}),
            DeckPlanStep(4, "moveZCamera", "ClassControlInterface.btnLOC1_Click:1932-1945", ("z",), {"location_id": destination.location_id}),
        ))
    else:
        steps.append(DeckPlanStep(3, "moveTo", "ClassControlInterface.btnLOC1_Click:ordinary", ("x", "y", "z"), {"location_id": destination.location_id, "camera_offset": bool(intent.camera_offset)}))
    pinned_binary_resolution = (
        "BARCODE_PARK_INVALID_IL_RESOLVED_FROM_PINNED_BINARY:"
        "BioXPControlLib.dll:sha256=163db8f7835cecbc87da4d14734a8224d79ea1e2ccc77bbb299998fa31bf14ed:"
        "tokens=0x060000CB,0x0600011E,0x0600012E,0x06000351"
    )
    source_hazards = {
        "ordinary": (
            "ordinary:ROUTE_ALWAYS_TRUE_LOCATION_TEST:retain_decompiled_predicate_pending_raw_il",
        ),
        "barcode": (
            "barcode:ROUTE_ALWAYS_TRUE_LOCATION_TEST:retain_decompiled_predicate_pending_raw_il",
            f"barcode:{pinned_binary_resolution}",
        ),
        "park": (
            f"park:{pinned_binary_resolution}",
        ),
    }[destination.branch]
    return DeckMovementPlan(
        target=destination.target, target_label=destination.panel_label,
        resolved_location_id=destination.location_id, source_branch=destination.branch,
        authority_digest=authority.digest, position_table_sha256=table.digest, catalog_revision=catalog.revision,
        steps=tuple(steps), semantic_transition={
            "current_location_id": destination.location_id,
            "current_location": destination.location_name,
            "current_well_id": 0,
            "ownership_generation": authority.ownership_generation,
            "board_epoch_4": authority.board_epoch_4,
            "board_epoch_5": authority.board_epoch_5,
            "authority_snapshot_digest": authority.digest,
            "position_table_revision": table.digest,
            "destination_catalog_revision": catalog.revision,
            "latch_status": authority.latch_status,
            "machine_latch_closed": authority.machine_latch_closed,
            "latch_observation_id": authority.latch_observation_id,
        },
        blocked_reason=blocked, source_hazards=source_hazards,
    )


class MovementAuthorityChanged(RuntimeError):
    pass


class DeckExecutionFailure(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        delivery_attempted: bool,
        controller_command_acknowledged: bool = False,
        controller_completion_verified: bool = False,
        hardware_postcondition_verified: bool = False,
        provider_results: list[Mapping[str, Any]] | None = None,
    ) -> None:
        super().__init__(message)
        self.delivery_attempted = bool(delivery_attempted)
        self.controller_command_acknowledged = bool(controller_command_acknowledged)
        self.controller_completion_verified = bool(controller_completion_verified)
        self.hardware_postcondition_verified = bool(hardware_postcondition_verified)
        self.provider_results = [dict(row) for row in (provider_results or [])]


class DeckMovementExecutor:
    """Lease-bound execution seam; provider writes are reachable only here."""

    def __init__(self, provider: Any, snapshot_reader: Callable[[], DeckAuthoritySnapshot]) -> None:
        self.provider = provider
        self.snapshot_reader = snapshot_reader
        self._lease = threading.Lock()

    @contextmanager
    def movement_lease(self) -> Iterator[None]:
        with self._lease:
            yield

    def execute(
        self,
        plan: DeckMovementPlan,
        *,
        before_first_write: Callable[[DeckMovementPlan], None] | None = None,
        after_each_child: Callable[[DeckPlanStep], None] | None = None,
    ) -> list[Any]:
        if plan.blocked_reason:
            raise MovementAuthorityChanged(plan.blocked_reason)
        with self.movement_lease():
            if before_first_write is not None:
                before_first_write(plan)
            if self.snapshot_reader().digest != plan.authority_digest:
                raise MovementAuthorityChanged("deck_authority_changed_before_first_tx")
            results = []
            for step in plan.steps:
                if step.operation in {"ForceToHighHome", "check_latch_status", "check_machine_latch_closed"}:
                    continue
                method = getattr(self.provider, step.operation, None)
                if not callable(method):
                    raise RuntimeError(f"source_authority_missing:{step.operation}")
                result = method(**dict(step.arguments or {}))
                if isinstance(result, Mapping) and result.get("ok") is not True:
                    raise RuntimeError(f"provider_stage_failed:{step.operation}")
                results.append(result)
                if after_each_child is not None:
                    after_each_child(step)
            return results


@dataclass(frozen=True)
class ClassMoveToIntent:
    """Caller-owned ClassMoveTo input; all machine state is deliberately absent."""

    script_line: int
    plate_name: int | None = None
    location_id: int | None = None
    well: str | int | None = None
    material: str | None = None
    continuation: str | None = None

    def __post_init__(self) -> None:
        if type(self.script_line) is not int:
            raise ValueError("script_line must be an integer")
        if (self.plate_name is None) == (self.location_id is None):
            raise ValueError("exactly one destination is required")
        if self.plate_name is not None:
            object.__setattr__(self, "plate_name", canonical_plate_name(self.plate_name))
        if self.location_id is not None and self.location_id not in {6, 16}:
            raise ValueError("unexpected plate")
        if self.material is not None and type(self.material) is not str:
            raise ValueError("material must be text")
        if self.continuation is not None and type(self.continuation) is not str:
            raise ValueError("continuation must be text")


@dataclass(frozen=True)
class MovExecutionChild:
    operation: str
    arguments: Mapping[str, Any] | None = None


@dataclass(frozen=True)
class MovExecutionStep:
    order: int
    operation: str
    source_anchor: str
    arguments: Mapping[str, Any]
    source_children: tuple[MovExecutionChild, ...] = ()
    join: str | None = None
    semantic_transition: Mapping[str, Any] | None = None

    @property
    def resources(self) -> tuple[str, ...]:
        return ("x", "y", "z", "g") if self.operation == "scriptmoveTo" else ()


@dataclass(frozen=True)
class MovExecutionPlan:
    script_line: int
    source_branch: str
    authority_digest: str
    destination: int
    plate_name: int
    well_id: int
    well_source: str
    steps: tuple[MovExecutionStep, ...]
    source_hazards: tuple[str, ...]
    machine_state_updates: Mapping[str, Any]
    destination_translation: Mapping[str, Any]
    schema_version: str = "bioxp.oem_mov_execution_plan.v1"

    @property
    def plan_digest(self) -> str:
        return _digest(asdict(self))

    # Adapter properties let WP7 reuse the existing immutable deck tables.
    @property
    def target(self) -> str:
        return f"movExecution:{self.plate_name}"

    @property
    def target_label(self) -> str:
        return self.target

    @property
    def resolved_location_id(self) -> int:
        return self.destination

    @property
    def catalog_revision(self) -> str:
        return _digest({"source": "movExecution", "token": "0x0600034C"})

    @property
    def position_table_sha256(self) -> str:
        return self.authority_digest if len(self.authority_digest) == 64 else _digest(self.authority_digest)


_STATION_TRANSLATIONS = {23: 2, 21: 1, 25: 0}
_R_PUNCH_PLATES = frozenset({0, 1, 2, 7, 8, 9, 10})


def _state_value(state: Mapping[str, Any], *names: str, default: Any = None) -> Any:
    for name in names:
        if name in state:
            return state[name]
    return default


def _plate_location(state: Mapping[str, Any], plate: int) -> int:
    locations = _state_value(state, "plate_locations", "movable_plate_locations")
    if not isinstance(locations, Mapping):
        raise ValueError("source_authority_missing:plate_location")
    value = locations.get(plate, locations.get(str(plate)))
    if type(value) is not int or value < 0:
        raise ValueError("source_authority_missing:plate_location")
    return value


def _pierced(state: Mapping[str, Any], plate: int, well: int) -> bool:
    if plate in {7, 8, 9, 10}:
        values = state.get("strip_pierced", {})
        marker: int | str = "strip"
    else:
        values = state.get("well_pierced", {})
        marker = 1 if int(state.get("tip_location", -1)) != -1 else 0
    if not isinstance(values, Mapping):
        return False
    lookups: tuple[Any, ...] = (
        (plate, well, marker),
        str((plate, well, marker)),
        f"{plate}:{well}:{marker}",
        (plate, well),
        str((plate, well)),
        f"{plate}:{well}",
    )
    return any(bool(values.get(key, False)) for key in lookups)


def _pierce_transition(plate: int, well: int, state: Mapping[str, Any]) -> dict[str, Any]:
    marker: int | str = "strip" if plate in {7, 8, 9, 10} else (1 if int(state.get("tip_location", -1)) != -1 else 0)
    return {"well_pierced": [plate, well, marker]}


def _continuation_step(
    code: str | None, *, plate: int, destination: int, well: int, state: Mapping[str, Any], order: int
) -> MovExecutionStep | None:
    column, row = well % 12, well // 12
    if code == "r" and plate in _R_PUNCH_PLATES and not _pierced(state, plate, well):
        return MovExecutionStep(
            order, "rPunchFoil", "movExecution:r",
            {"plate_name": plate, "location_id": destination},
            semantic_transition=_pierce_transition(plate, well, state),
        )
    if code == "w" and int(_state_value(state, "trough_version", "TroughVersion", default=0)) <= 0:
        rows = state.get("position_table_by_location")
        target_row = rows.get("16", rows.get(16)) if isinstance(rows, Mapping) else None
        if not isinstance(target_row, Mapping):
            raise ValueError("source_authority_missing:continuation_position_table")
        coordinates = target_row.get("base_coordinates")
        if not isinstance(coordinates, Mapping) or type(coordinates.get("y")) is not int:
            raise ValueError("source_authority_missing:continuation_base_y")
        base_y = int(coordinates["y"])
        z_low_value = target_row.get("z_low")
        if type(z_low_value) is not int:
            raise ValueError("source_authority_missing:continuation_z_low")
        z_low = int(z_low_value)
        actions = (
            ("moveZ", z_low),
            ("moveY", base_y + 600),
            ("moveY", base_y - 600),
            ("moveY", base_y + 600),
            ("moveY", base_y - 600),
            ("moveY", base_y),
            ("moveZ", z_low - 4000),
            ("moveZ", z_low),
            ("moveZ", z_low - 4000),
            ("moveZ", z_low),
            ("moveZ", z_low - 10000),
            ("MoveZHome", None),
        )
        return MovExecutionStep(order, "troughOscillation", "movExecution:w", {"location_id": 16, "actions": actions})
    if code == "h" and not _pierced(state, 2, well):
        return MovExecutionStep(
            order, "hokeypokey", "movExecution:h",
            {"destination": destination, "column": column, "row": row},
            semantic_transition=_pierce_transition(2, well, state),
        )
    if code == "d" and not _pierced(state, plate, well):
        return MovExecutionStep(
            order, "scriptmoveTo", "movExecution:d",
            {"destination": destination, "well": well, "positionflag": -1},
            semantic_transition=_pierce_transition(plate, well, state),
        )
    if code == "t" and plate == 0 and not _pierced(state, 0, well):
        return MovExecutionStep(
            order, "CirclePunch", "movExecution:t",
            {"destination": destination, "column": column, "row": row},
            semantic_transition=_pierce_transition(0, well, state),
        )
    return None


def compile_mov_execution(
    intent: ClassMoveToIntent | None = None,
    machine_state: Mapping[str, Any] | None = None,
    *,
    get_next_well: Callable[[int, str, float], int] | None = None,
    movement_children: tuple[str, ...] = (),
    # Frozen compatibility-only preview signature. It never reaches execution.
    current_plate_location: int | None = None,
    destination: int | None = None,
    well: str | int | None = None,
    plate_name: str | None = None,
) -> MovExecutionPlan | dict[str, Any]:
    """Compile movExecution from immutable caller intent and server-owned state."""
    if intent is None:
        if current_plate_location is None or destination is None or well is None:
            raise TypeError("intent and machine_state are required")
        well_id = well_id_from_label(well)
        resolved = _STATION_TRANSLATIONS.get(destination, destination)
        if plate_name is not None and current_plate_location < 0:
            raise ValueError("source_authority_missing:plate_location")
        return {"schema_version": "bioxp.oem_mov_execution_plan.v1", "current_plate_location": current_plate_location, "destination": resolved, "well_id": well_id, "plate_name": plate_name, "source_operation": "movExecution->scriptmoveTo"}
    if machine_state is None:
        raise ValueError("source_authority_missing:machine_state")

    requested_plate = intent.plate_name
    synthetic = intent.location_id is not None
    plate = requested_plate if requested_plate is not None else (21 if intent.location_id == 6 else 11)
    assert plate is not None
    save_tip = bool(_state_value(machine_state, "save_tip", "m_savetip", default=False))
    old_well = bool(_state_value(machine_state, "old_well", "m_oldWell", default=False))
    forced_old = save_tip and requested_plate == 21
    if forced_old:
        old_well = True

    authority_digest = str(machine_state.get("authority_digest") or _digest(dict(machine_state)))
    machine_updates: dict[str, Any] = {"save_tip": False, "old_well": True} if forced_old else {}
    children = tuple(MovExecutionChild(name) for name in movement_children)
    if children and tuple(child.operation for child in children) != ("moveX", "moveY"):
        raise ValueError("parallel movement children must be moveX,moveY")

    if old_well:
        old_text = str(_state_value(machine_state, "old_well_text", "m_oldWellText", default=""))
        if len(old_text) < 2 or not old_text[0].isalpha() or not old_text[1:].isdigit():
            raise ValueError("source_authority_missing:old_well")
        numeric_enum_value = int(old_text[1:])
        motion_row = ord(old_text[0].upper()) - ord("A")
        motion_column = numeric_enum_value - 1
        old_location = _state_value(machine_state, "old_location", "m_old_location")
        if type(old_location) is not int:
            raise ValueError("source_authority_missing:old_location")
        steps = (
            MovExecutionStep(0, "scriptmoveTo", "movExecution:oldWell", {"destination": old_location, "column": motion_column, "row": motion_row, "positionflag": 1, "runInParallel": True}, children, "Task.WaitAll" if children else None),
            MovExecutionStep(1, "updateLocation", "movExecution:oldWell:updateLocation", {"location_id": old_location, "well_id": numeric_enum_value}, semantic_transition={"current_location": old_location, "current_well": numeric_enum_value}),
        )
        return MovExecutionPlan(intent.script_line, "old_well_terminal", authority_digest, old_location, plate, numeric_enum_value, "old_well_numeric_enum_parse", steps, ("confirmed_oem_old_well_numeric_enum_parse",), machine_updates, {"kind": "old_well", "translated": False})

    if intent.well is not None:
        well_id, well_source = well_id_from_label(intent.well), "explicit"
    elif intent.material is not None:
        if get_next_well is None:
            raise ValueError("source_authority_missing:getNextWell")
        well_id, well_source = get_next_well(2, intent.material, 0.0), "material"
        if well_id == 96:
            raise ValueError("Unknow well")
        if type(well_id) is not int or not 0 <= well_id <= 95:
            raise ValueError("source_authority_missing:getNextWell")
    else:
        well_id, well_source = 0, "default"

    if synthetic:
        resolved_destination = int(intent.location_id)
        translation = {"kind": "direct", "source": intent.location_id, "resolved": resolved_destination, "synthetic_plate": plate}
    else:
        current = _plate_location(machine_state, plate)
        resolved_destination = _STATION_TRANSLATIONS.get(current, current)
        translation = {"kind": "plate", "source": current, "resolved": resolved_destination, "synthetic_plate": None}
    column, row = well_id % 12, well_id // 12
    steps_list = [
        MovExecutionStep(0, "scriptmoveTo", "movExecution:scriptmoveTo", {"destination": resolved_destination, "column": column, "row": row, "positionflag": 1, "runInParallel": True}, children, "Task.WaitAll" if children else None),
        MovExecutionStep(1, "updateLocation", "movExecution:updateLocation", {"location_id": resolved_destination, "well_id": well_id}, semantic_transition={"current_location": resolved_destination, "current_well": well_id}),
        MovExecutionStep(2, "updatePlateLocation", "movExecution:updatePlateLocation", {"plate_name": plate, "location_id": resolved_destination}, semantic_transition={"plate_name": plate, "plate_location": resolved_destination}),
    ]
    continuation_step = _continuation_step(intent.continuation, plate=plate, destination=resolved_destination, well=well_id, state=machine_state, order=3)
    if continuation_step is not None:
        steps_list.append(continuation_step)
    return MovExecutionPlan(intent.script_line, "normal", authority_digest, resolved_destination, plate, well_id, well_source, tuple(steps_list), (), machine_updates, translation)


def bind_mov_execution_script_plan(
    plan: MovExecutionPlan, script_plan: Mapping[str, Any]
) -> MovExecutionPlan:
    """Bind the exact no-I/O scriptmoveTo preview before durable execution."""
    if not isinstance(script_plan, Mapping) or not isinstance(script_plan.get("steps"), list):
        raise ValueError("scriptmoveTo preview is not authoritative")
    source_children: tuple[MovExecutionChild, ...] = ()
    join: str | None = None
    for row in script_plan["steps"]:
        if isinstance(row, Mapping) and row.get("op") == "parallel":
            children = row.get("steps")
            if not isinstance(children, list):
                raise ValueError("parallel scriptmoveTo preview is malformed")
            source_children = tuple(
                MovExecutionChild(
                    str(child["op"]),
                    {str(key): value for key, value in child.items() if key != "op"},
                )
                for child in children
                if isinstance(child, Mapping) and child.get("op") in {"moveX", "moveY"}
            )
            if tuple(child.operation for child in source_children) != ("moveX", "moveY"):
                raise ValueError("parallel scriptmoveTo children are not exact")
            join = str(row.get("join") or "")
            if join != "Task.WaitAll":
                raise ValueError("parallel scriptmoveTo join is not exact")
            break
    first = plan.steps[0]
    arguments = dict(first.arguments)
    arguments["expected_script_plan_digest"] = _digest(dict(script_plan))
    arguments["source_plan"] = dict(script_plan)
    rebound = replace(first, arguments=arguments, source_children=source_children, join=join)
    hazards = tuple(sorted(set(plan.source_hazards) | set(script_plan.get("source_hazards") or [])))
    return replace(plan, steps=(rebound, *plan.steps[1:]), source_hazards=hazards)


def _mov_terminal_truth(result: Any) -> bool:
    if not isinstance(result, Mapping) or result.get("ok") is not True:
        return False
    if result.get("delivery_attempted") is False:
        return result.get("semantic_update_ready") is True or result.get("source_noop") is True
    return (
        result.get("controller_command_acknowledged") is True
        and result.get("controller_completion_verified") is True
    )


def execute_mov_execution(command_id: str, plan: MovExecutionPlan, *, provider: Any, command_store: Any) -> dict[str, Any]:
    """Execute one precompiled movExecution plan without replaying ambiguous I/O."""
    persist = getattr(command_store, "persist_mov_execution_plan", None)
    if not callable(persist):
        raise RuntimeError("mov_execution_plan_store_unavailable")
    terminalize = getattr(command_store, "terminalize_mov_execution_stage", None)
    complete_stage = getattr(command_store, "complete_mov_execution_stage", None)
    publish = getattr(command_store, "publish_mov_execution_transition", None)
    record_delivery = getattr(command_store, "record_delivery_attempt", None)
    if not callable(record_delivery):
        raise RuntimeError("mov_execution_delivery_store_unavailable")
    lease_factory = getattr(provider, "movement_lease", None)
    lease = lease_factory() if callable(lease_factory) else nullcontext()
    assert_current = getattr(command_store, "assert_deck_execution_current", None)
    with lease:
        if callable(assert_current):
            assert_current(command_id, boundary="before_plan_write")
        persist(command_id, plan)
        provider_results: list[Mapping[str, Any]] = []
        transition_revisions: list[Any] = []
        if plan.machine_state_updates:
            if not callable(publish):
                return {"ok": False, "delivery_attempted": False, "ambiguity_state": "failed", "semantic_state_committed": False}
            try:
                transition_revisions.append(publish(command_id, dict(plan.machine_state_updates)))
            except Exception:
                return {"ok": False, "delivery_attempted": False, "ambiguity_state": "failed", "semantic_state_committed": False}
        for step in plan.steps:
            if callable(assert_current):
                assert_current(command_id, boundary=f"before:{step.order}:{step.operation}")
            method = getattr(provider, step.operation, None)
            special = step.operation in {"troughOscillation", "pierceCurrentWell"}
            if not callable(method) and not special:
                if callable(terminalize):
                    terminalize(command_id, step, state="failed", reason=f"source_authority_missing:{step.operation}")
                return {"ok": False, "delivery_attempted": bool(provider_results), "ambiguity_state": "failed", "semantic_state_committed": False}
            step_delivery_attempted = False
            step_dispatch_attempt_id: str | None = None

            def mark_delivery(work_identity: str) -> None:
                nonlocal step_delivery_attempted, step_dispatch_attempt_id
                marker = record_delivery(
                    command_id,
                    work_kind="wp7_stage",
                    work_identity=work_identity,
                    plan_digest=plan.plan_digest,
                )
                if not isinstance(marker, Mapping) or not marker.get("dispatch_attempt_id"):
                    raise RuntimeError("mov_execution_delivery_marker_invalid")
                step_delivery_attempted = True
                step_dispatch_attempt_id = str(marker["dispatch_attempt_id"])

            try:
                if step.operation == "troughOscillation":
                    child_results: list[Mapping[str, Any]] = []
                    for child_order, (operation, value) in enumerate(step.arguments["actions"]):
                        child_method = getattr(provider, operation, None)
                        if operation == "Sleep" and not callable(child_method):
                            child_method = getattr(provider, "sleep", None)
                        if not callable(child_method):
                            raise RuntimeError(f"source_authority_missing:{operation}")
                        mark_delivery(
                            f"stage:{step.order}:child:{child_order}:{operation}",
                        )
                        child = child_method() if value is None else child_method(value)
                        if not _mov_terminal_truth(child):
                            raise DeckExecutionFailure(
                                f"provider_terminal_proof_missing:{operation}", delivery_attempted=True
                            )
                        child_results.append(dict(child))
                    result = {
                        "ok": True,
                        "delivery_attempted": True,
                        "controller_command_acknowledged": True,
                        "controller_completion_verified": True,
                        "source_children": child_results,
                    }
                elif step.operation == "pierceCurrentWell":
                    move = getattr(provider, "scriptmoveTo", None)
                    if not callable(move):
                        raise RuntimeError("source_authority_missing:scriptmoveTo")
                    mark_delivery(f"stage:{step.order}:pierceCurrentWell:scriptmoveTo")
                    result = move(**dict(step.arguments))
                    if not _mov_terminal_truth(result):
                        raise DeckExecutionFailure(
                            "provider_terminal_proof_missing:scriptmoveTo", delivery_attempted=True
                        )
                else:
                    assert callable(method)
                    mark_delivery(f"stage:{step.order}:{step.operation}")
                    result = method(**dict(step.arguments))
                if callable(assert_current):
                    assert_current(
                        command_id,
                        boundary=f"after_provider:{step.order}:{step.operation}",
                    )
            except Exception as exc:
                current_stage_delivery = (
                    exc.delivery_attempted
                    if isinstance(exc, DeckExecutionFailure)
                    else step_delivery_attempted
                )
                delivery_attempted = any(
                    row.get("delivery_attempted") is True for row in provider_results
                ) or current_stage_delivery
                if str(exc).startswith("deck_execution_"):
                    raise
                if callable(terminalize):
                    terminalize(
                        command_id, step, state="ambiguous" if delivery_attempted else "failed",
                        reason=f"provider_stage_exception:{type(exc).__name__}",
                        dispatch_attempt_id=step_dispatch_attempt_id,
                    )
                return {
                    "ok": False, "delivery_attempted": delivery_attempted,
                    "ambiguity_state": "ambiguous" if delivery_attempted else "failed",
                    "semantic_state_committed": False,
                }
            result_map = dict(result) if isinstance(result, Mapping) else {"source_return": result}
            result_map["source_return_disposition"] = "ignored"
            delivery_attempted = (
                result_map.get("delivery_attempted") is True
                or any(row.get("delivery_attempted") is True for row in provider_results)
            )
            if not _mov_terminal_truth(result):
                terminal_state = "ambiguous" if delivery_attempted else "failed"
                if callable(terminalize):
                    terminalize(
                        command_id, step, state=terminal_state, result=result_map,
                        reason="provider_terminal_proof_missing",
                        dispatch_attempt_id=step_dispatch_attempt_id,
                    )
                return {
                    "ok": False,
                    "delivery_attempted": delivery_attempted,
                    "controller_command_acknowledged": bool(result_map.get("controller_command_acknowledged")),
                    "controller_completion_verified": False,
                    "semantic_state_committed": False,
                    "ambiguity_state": "ambiguous" if delivery_attempted else "failed",
                    "source_return_disposition": "ignored",
                    "provider_results": [*provider_results, result_map],
                }
            provider_results.append(result_map)
            if step.semantic_transition is not None and callable(complete_stage):
                try:
                    revision = complete_stage(
                        command_id, step, result=result_map,
                        semantic_transition=dict(step.semantic_transition),
                        source_return_disposition="ignored",
                        dispatch_attempt_id=step_dispatch_attempt_id,
                    )
                    if revision is not None:
                        transition_revisions.append(revision)
                except Exception:
                    return {"ok": False, "delivery_attempted": True, "controller_command_acknowledged": True, "controller_completion_verified": True, "semantic_state_committed": False, "ambiguity_state": "recovery_required", "provider_results": provider_results}
            else:
                if callable(terminalize):
                    terminalize(
                        command_id, step, state="completed", result=result_map,
                        source_return_disposition="ignored",
                        dispatch_attempt_id=step_dispatch_attempt_id,
                    )
                if step.semantic_transition is not None and not callable(publish):
                    return {"ok": False, "delivery_attempted": True, "controller_command_acknowledged": True, "controller_completion_verified": True, "semantic_state_committed": False, "ambiguity_state": "recovery_required", "provider_results": provider_results}
                if step.semantic_transition is not None:
                    try:
                        transition_revisions.append(publish(command_id, dict(step.semantic_transition)))
                    except Exception:
                        return {"ok": False, "delivery_attempted": True, "controller_command_acknowledged": True, "controller_completion_verified": True, "semantic_state_committed": False, "ambiguity_state": "recovery_required", "provider_results": provider_results}
            if callable(assert_current):
                assert_current(command_id, boundary=f"after:{step.order}:{step.operation}")
        return {"ok": True, "delivery_attempted": True, "controller_command_acknowledged": True, "controller_completion_verified": True, "semantic_state_committed": True, "ambiguity_state": "none", "script_line": plan.script_line, "source_branch": plan.source_branch, "authority_digest": plan.authority_digest, "plan_digest": plan.plan_digest, "well_source": plan.well_source, "destination_translation": dict(plan.destination_translation), "source_hazards": list(plan.source_hazards), "source_return_disposition": "ignored", "transition_revisions": transition_revisions, "continuation_disposition": plan.steps[-1].operation if len(plan.steps) > (2 if plan.source_branch == "old_well_terminal" else 3) else "noop", "terminal_evidence_truth": True, "provider_results": provider_results}


WP8_AUTHORITY_DIGEST = "a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c"


def _wp8_child(
    children: list[dict[str, Any]], operation: str, *, arguments: Mapping[str, Any] | None = None,
    ignored_return: bool = False, state_mutation: Mapping[str, Any] | None = None,
    awaited: bool = True, exception_policy: str = "propagate",
    source_condition: Mapping[str, Any] | None = None,
) -> None:
    order = len(children)
    children.append({
        "order": order,
        "depends_on": [] if order == 0 else [order - 1],
        "operation": operation,
        "arguments": dict(arguments or {}),
        "ignored_return": bool(ignored_return),
        "state_mutation": dict(state_mutation or {}),
        "awaited": bool(awaited),
        "exception_policy": exception_policy,
        "source_condition": dict(source_condition or {}),
    })


def _wp8_plan(operation: str, children: list[dict[str, Any]], **metadata: Any) -> dict[str, Any]:
    plan = {
        "schema_version": "bioxp.oem_wp8_operation.v1",
        "operation": operation,
        "source_owned": True,
        "state_update": "source_ordered_partial_residuals",
        "authority_digest": WP8_AUTHORITY_DIGEST,
        "children": children,
        "residual_policy": "retain_completed_children_without_rollback",
        "parent_return_allows_background_pending": False,
        **metadata,
    }
    policy = str(plan.get("exception_policy") or "propagate")
    finally_names = set(plan.get("finally_children") or [])
    for child in children:
        if child["operation"] in finally_names:
            child["exception_policy"] = "finally"
        elif child.get("exception_policy") == "propagate":
            child["exception_policy"] = policy
        mutation = dict(child.get("state_mutation") or {})
        child["state_transition_timing"] = (
            "terminal_source_point" if child["operation"] == "updatePlateLocation"
            else "immediate_after_source_call" if mutation else "none"
        )
    plan["plan_digest"] = _digest(plan)
    return plan


FINITE_PLATE_OPERATIONS = frozenset({
    "catch_plate", "release_plate", "park_gantry", "waste_sequence",
    "press_plate", "press_plates", "send_z_and_gripper_home", "thermal_door", "cleanup",
    "move_plate",
})

WP8_OPERATION_INTENT_KEYS: Mapping[str, frozenset[str]] = {
    "move_plate": frozenset({"plate", "destination", "press_plate"}),
    "catch_plate": frozenset({"plate", "run_in_parallel"}),
    "release_plate": frozenset({"destination", "press_plate", "run_in_parallel"}),
    "park_gantry": frozenset(),
    "waste_sequence": frozenset(),
    "press_plate": frozenset({"plate", "run_in_parallel"}),
    "press_plates": frozenset({"plates", "run_in_parallel"}),
    "send_z_and_gripper_home": frozenset({"run_in_parallel"}),
    "thermal_door": frozenset({"open"}),
    "cleanup": frozenset(),
}

WP8_COMPILED_CHILD_OPERATIONS = frozenset({
    "CloseGripper", "HomeAxisD", "LoadGantry", "LoadGantryNull", "LockGripperOperation",
    "MoveZHome", "OpenGripper", "OpenGripperWide", "ReleaseLockGripperOperation", "Sleep",
    "SnapshotImage", "StopCloseGripper", "backgroundGripperHomeAndUnlock", "catchPlate",
    "checkDoorStatus", "cleanupWastePrelude", "clearTipLoaded", "doorOpen", "ejectAllTipsCleanup",
    "getG", "led2Off", "led2On", "moveDoorClosed", "moveDoorOpen", "moveGClosedPlus3000",
    "moveStepsYMinus800", "moveStepsYPlus1600", "moveStepsZMinus6000", "moveX79000", "moveZ",
    "moveZ80000", "moveZLow", "moveZPress", "moveZPressApproach", "moveZPseudoHome",
    "parkGantry", "queryTipStatus", "readDoorSensors", "releasePlate", "scriptmoveTo", "scriptmoveToWaste",
    "sendGripperHome", "sendZandGripperHome", "setDoorMaxCurrent", "setDoorStallThreshold",
    "setDoorStallThresholdPlus2",
    "setGripperCurrent", "setGripperVMax", "setZCurrent31", "setZaxisCurrentmax100",
    "startGripperHomeAndUnlock", "startMoveZPseudoHome", "updateLocation", "updatePlateLocation", "updateThermalDoorOpen",
    "waitMoveZOnly", "waitStop", "waitZ",
})


def _compile_finite_plate_operation_unchecked(
    operation: str, *, source_leaf_available: bool, **inputs: Any,
) -> dict[str, Any]:
    """Compile literal, finite WP8 source children without strengthening OEM semantics."""
    if operation not in FINITE_PLATE_OPERATIONS:
        raise ValueError("unknown finite deck operation")
    if not source_leaf_available:
        raise RuntimeError(f"source_authority_missing:{operation}")
    children: list[dict[str, Any]] = []

    if operation == "move_plate":
        plate = canonical_plate_name(inputs.get("plate"))
        destination = inputs.get("destination")
        if plate is None or type(destination) is not int:
            raise RuntimeError("source_authority_missing:move_plate_intent")
        door_open = inputs.get("thermal_door_open")
        if type(door_open) is not bool:
            raise RuntimeError("source_authority_missing:thermal_door_open")
        if plate in {0, 3} or destination in {5, 23}:
            if not door_open:
                raise RuntimeError("thermal_door_must_be_open")
        if destination in {18, 20} and door_open:
            raise RuntimeError("thermal_door_must_be_closed")
        _wp8_child(
            children, "catchPlate",
            arguments={"plate": plate, "run_in_parallel": True},
            ignored_return=True,
        )
        _wp8_child(
            children, "releasePlate",
            arguments={
                "destination": int(destination),
                "press_plate": bool(inputs.get("press_plate", False)),
                "run_in_parallel": True,
            },
            ignored_return=True,
        )
        return _wp8_plan(operation, children, exception_policy="propagate")

    if operation == "park_gantry":
        _wp8_child(children, "parkGantry", arguments={"rehome": bool(inputs.get("rehome", False))}, ignored_return=True)
        return _wp8_plan(operation, children, provider_method="parkGantry")
    if operation == "waste_sequence":
        _wp8_child(children, "cleanupWastePrelude")
        return _wp8_plan(operation, children, provider_method="cleanupWastePrelude")

    if operation == "catch_plate":
        plate = int(inputs.get("plate", 0))
        location = int(inputs.get("plate_location", inputs.get("location", 0)))
        destination = {1: 21, 2: 23, 0: 25}.get(location, location)
        door_open = bool(inputs.get("thermal_door_open", False))
        parallel = bool(inputs.get("run_in_parallel", True))
        if destination in {17, 18, 19, 20}:
            if door_open:
                _wp8_child(children, "doorOpen", arguments={"open": False}, ignored_return=True)
        elif not door_open:
            _wp8_child(children, "doorOpen", arguments={"open": True}, ignored_return=True)
        _wp8_child(children, "scriptmoveTo", arguments={"destination": destination, "well": 0, "position_flag": 0}, ignored_return=True)
        _wp8_child(children, "updateLocation", arguments={"destination": destination, "well": 0}, state_mutation={"current_location": destination, "current_well": 0})
        if parallel:
            _wp8_child(children, "LockGripperOperation")
        _wp8_child(children, "setGripperCurrent", arguments={"current": 31}, state_mutation={"gripper_current": 31})
        version = int(inputs.get("gripper_version", 0))
        _wp8_child(children, "setGripperVMax", arguments={"vmax": 900 if version == 0 else 1500})
        if version == 0:
            _wp8_child(children, "StopCloseGripper", arguments={"reset_speed": False})
        wide = destination in {0, 1, 2, 21, 23, 25}
        _wp8_child(children, "OpenGripperWide" if wide else "OpenGripper", arguments={"recover": True})
        _wp8_child(children, "led2On", ignored_return=True)
        _wp8_child(children, "Sleep", arguments={"milliseconds": 1000})
        _wp8_child(children, "SnapshotImage", arguments={"name": "CatchPlate"}, ignored_return=True)
        _wp8_child(children, "Sleep", arguments={"milliseconds": 100})
        _wp8_child(children, "led2Off", ignored_return=True)
        offset = -30236 if destination == 25 and int(inputs.get("output_plate_location", -1)) in {25, 0} else 0
        _wp8_child(children, "moveZ", arguments={"location": destination, "z_low_offset": offset})
        _wp8_child(children, "Sleep", arguments={"milliseconds": 100})
        if plate == 0:
            _wp8_child(children, "moveGClosedPlus3000", arguments={"offset": 3000})
        else:
            _wp8_child(children, "CloseGripper")
        if destination == 5:
            _wp8_child(children, "moveStepsZMinus6000", arguments={"steps": -6000})
            _wp8_child(children, "moveStepsYMinus800", arguments={"steps": -800})
            _wp8_child(children, "moveStepsYPlus1600", arguments={"steps": 1600})
        _wp8_child(children, "LoadGantry", arguments={"plate": plate}, state_mutation={"plate_on_gantry": plate, "pseudo_z_home": 65000 if plate == 3 else 500})
        _wp8_child(children, "moveZPseudoHome")
        _wp8_child(children, "updatePlateLocation", arguments={"plate": plate, "location": 29}, state_mutation={"plate": plate, "location": 29})
        if parallel:
            _wp8_child(children, "ReleaseLockGripperOperation")
        return _wp8_plan(
            operation, children, resolved_location=destination, z_offset=offset,
            exception_policy="log_and_suppress", finally_children=["ReleaseLockGripperOperation"] if parallel else [],
        )

    if operation == "release_plate":
        destination = int(inputs["destination"])
        parallel = bool(inputs.get("run_in_parallel", True))
        offset = -30236 if destination == 25 and int(inputs.get("plate_on_gantry", -1)) == 1 and int(inputs.get("output_plate_location", -1)) in {25, 29} else 0
        _wp8_child(children, "scriptmoveTo", arguments={"destination": destination, "well": 0, "position_flag": 0}, ignored_return=True)
        _wp8_child(children, "moveZLow", arguments={"location": destination, "z_low_offset": offset})
        _wp8_child(children, "Sleep", arguments={"milliseconds": 5})
        if parallel:
            _wp8_child(children, "LockGripperOperation")
        _wp8_child(children, "setGripperCurrent", arguments={"current": 31}, state_mutation={"gripper_current": 31})
        _wp8_child(children, "OpenGripperWide" if destination in {21, 23, 25} else "OpenGripper")
        _wp8_child(children, "Sleep", arguments={"milliseconds": 200})
        _wp8_child(children, "setGripperVMax", arguments={"vmax": 900 if int(inputs.get("gripper_version", 0)) == 0 else 1500})
        _wp8_child(children, "Sleep", arguments={"milliseconds": 200})
        pressure_target = {21: 22, 23: 24, 25: 26}.get(destination)
        if bool(inputs.get("press_plate", False)) and pressure_target is not None:
            _wp8_child(children, "moveZPressApproach", arguments={"pressure_target": pressure_target, "offset": -26000})
            _wp8_child(children, "CloseGripper")
            _wp8_child(children, "setZCurrent31", arguments={"current": 31}, state_mutation={"z_current": 31})
            _wp8_child(children, "moveZPress", arguments={"pressure_target": pressure_target})
            _wp8_child(children, "setZaxisCurrentmax100", arguments={"percent": 100, "selection": "configured_Z_MOTOR_MAX_CURRENT_DOWN"}, state_mutation={"z_current_selection": "configured_down"})
        _wp8_child(children, "LoadGantryNull", state_mutation={"plate_on_gantry": None, "pseudo_z_home": 65000})
        _wp8_child(children, "sendZandGripperHome", arguments={"run_in_parallel": parallel})
        plate = inputs.get("current_tray")
        _wp8_child(children, "updatePlateLocation", arguments={"plate": plate, "location": destination}, state_mutation={"plate": plate, "location": destination})
        _wp8_child(children, "updateLocation", arguments={"destination": destination, "well": 0}, state_mutation={"current_location": destination, "current_well": 0})
        _wp8_child(children, "led2On", ignored_return=True)
        _wp8_child(children, "Sleep", arguments={"milliseconds": 1000})
        _wp8_child(children, "SnapshotImage", arguments={"name": "ReleasePlate"}, ignored_return=True)
        _wp8_child(children, "Sleep", arguments={"milliseconds": 100})
        _wp8_child(children, "led2Off", ignored_return=True)
        return _wp8_plan(operation, children, z_offset=offset, pressure_target=pressure_target, exception_policy="log_and_suppress", finally_children=[])

    if operation == "send_z_and_gripper_home":
        parallel = bool(inputs.get("run_in_parallel", True))
        delay = 1000 if int(inputs.get("gripper_position", 0)) > int(inputs.get("closed_position", 0)) else 0
        _wp8_child(children, "getG")
        if parallel:
            _wp8_child(children, "startMoveZPseudoHome", awaited=False)
            _wp8_child(children, "Sleep", arguments={"milliseconds": delay})
            _wp8_child(children, "startGripperHomeAndUnlock", awaited=False)
            _wp8_child(children, "waitMoveZOnly")
        else:
            _wp8_child(children, "moveZPseudoHome")
            _wp8_child(children, "sendGripperHome")
            _wp8_child(children, "ReleaseLockGripperOperation")
        return _wp8_plan(
            operation, children, exception_policy="propagate", finally_children=[],
            parent_return_allows_background_pending=parallel,
        )

    if operation in {"press_plate", "press_plates"}:
        plates = list(inputs.get("plates", [inputs.get("plate")]))
        recognized = [int(p) for p in plates if p in {0, 1, 2}]
        pressure_targets = {0: 24, 1: 22, 2: 27}
        locations = dict(inputs.get("plate_locations", {}))
        single = len(plates) == 1
        parallel = bool(inputs.get("run_in_parallel", True))
        for plate in recognized:
            destination = locations.get(1) if plate == 1 else 27 if plate == 2 else 24
            if type(destination) is not int:
                raise RuntimeError(f"source_authority_missing:press_plate_location:{plate}")
            pressure_target = pressure_targets[plate]
            _wp8_child(children, "scriptmoveTo", arguments={"destination": destination, "runInParallel": parallel}, ignored_return=True)
            _wp8_child(children, "updateLocation", arguments={"destination": destination, "well": 0}, state_mutation={"current_location": destination, "current_well": 0})
            if parallel:
                _wp8_child(children, "LockGripperOperation")
            _wp8_child(children, "setGripperCurrent", arguments={"current": 31}, state_mutation={"gripper_current": 31})
            _wp8_child(children, "moveZPressApproach", arguments={"pressure_target": pressure_target, "offset": -16000, "wait": False, "current_selection": "configured_down"})
            _wp8_child(children, "CloseGripper")
            _wp8_child(children, "waitZ", arguments={"timeout_ms": 15000})
            _wp8_child(children, "setZCurrent31", arguments={"current": 31}, state_mutation={"z_current": 31})
            _wp8_child(children, "moveZPress", arguments={"pressure_target": pressure_target})
            _wp8_child(children, "setZaxisCurrentmax100", arguments={"percent": 100, "selection": "configured_Z_MOTOR_MAX_CURRENT_DOWN"}, state_mutation={"z_current_selection": "configured_down"})
            if single:
                _wp8_child(children, "sendZandGripperHome", arguments={"run_in_parallel": parallel})
            else:
                _wp8_child(children, "MoveZHome")
        if not single and recognized:
            _wp8_child(children, "backgroundGripperHomeAndUnlock", awaited=False)
        return _wp8_plan(operation, children, exception_policy="propagate", finally_children=[], parent_return_allows_background_pending=not single and bool(recognized))

    if operation == "thermal_door":
        opening = bool(inputs.get("open"))
        door_state = inputs.get("door_is_open")
        if type(door_state) is not bool:
            raise RuntimeError("source_authority_missing:thermal_door_state")
        board_test = bool(inputs.get("board_test_mode", False))
        if door_state is opening:
            return _wp8_plan(
                operation, children, source_noop=True,
                success_return=False if board_test else True,
            )
        _wp8_child(children, "parkGantry", arguments={"rehome": False}, ignored_return=True)
        motion = "moveDoorOpen" if opening else "moveDoorClosed"
        for name in ("setDoorStallThresholdPlus2", "setDoorMaxCurrent", motion, "readDoorSensors"):
            _wp8_child(children, name, arguments={"open": opening})
        condition = {
            "child_order": 4,
            "result_field": "door_open" if opening else "door_closed",
            "equals": False,
        }
        if opening:
            for name in ("HomeAxisD", "setDoorStallThreshold", "setDoorMaxCurrent", motion, "readDoorSensors"):
                _wp8_child(children, name, arguments={"open": opening}, source_condition=condition)
        else:
            for name in ("SnapshotImage", "HomeAxisD"):
                _wp8_child(children, name, arguments={"open": opening}, source_condition=condition)
        update_condition: dict[str, Any] | None = None
        if board_test:
            update_condition = {
                "any_child_orders": [4, 9] if opening else [4],
                "result_field": "door_open" if opening else "door_closed",
                "equals": True,
                "on_false": "raise",
                "error": "thermal_door_open_failed" if opening else "thermal_door_close_failed",
            }
        _wp8_child(
            children, "updateThermalDoorOpen",
            arguments={"value": opening},
            state_mutation={"thermal_door_open": opening},
            source_condition=update_condition,
        )
        return _wp8_plan(operation, children, sensor_success_predicate="openSensor && !closedSensor" if opening else "!openSensor && closedSensor", null_board_return=True, success_return=False if board_test else True, failure_policy="throw" if board_test else ("home_and_retry_once" if opening else "image_log_and_home_no_retry"), normal_state_update_unconditional=not board_test)

    # ControlLib.cleanup: distinct cleanup waste prelude, then output/reagent cover storage.
    _wp8_child(children, "waitStop")
    _wp8_child(children, "checkDoorStatus")
    _wp8_child(children, "queryTipStatus", ignored_return=True)
    if bool(inputs.get("tip_exists", False)):
        _wp8_child(children, "scriptmoveToWaste", ignored_return=True)
        _wp8_child(children, "updateLocation", arguments={"destination": 6, "well": 0}, state_mutation={"current_location": 6, "current_well": 0})
        _wp8_child(children, "ejectAllTipsCleanup", arguments={"first": False, "second": True}, ignored_return=True)
        _wp8_child(children, "moveZ80000")
        _wp8_child(children, "moveX79000")
        _wp8_child(children, "clearTipLoaded", state_mutation={"tip_loaded": False})
    _wp8_child(children, "sendGripperHome")
    cover_locations = dict(inputs.get("cover_locations", {}))
    for plate, destination in ((4, 18), (5, 20)):
        source_location = int(cover_locations.get(plate, -1))
        if source_location == destination:
            continue
        _wp8_child(children, "doorOpen", arguments={"open": False}, ignored_return=True)
        if source_location != 29:
            _wp8_child(children, "catchPlate", arguments={"plate": plate, "run_in_parallel": True})
            _wp8_child(
                children, "updatePlateLocation",
                arguments={"plate": plate, "location": 29},
                state_mutation={"plate": plate, "location": 29},
            )
        _wp8_child(children, "releasePlate", arguments={"destination": destination, "press_plate": False, "run_in_parallel": True})
        _wp8_child(
            children, "updatePlateLocation",
            arguments={"plate": plate, "location": destination},
            state_mutation={"plate": plate, "location": destination},
        )
    _wp8_child(children, "doorOpen", arguments={"open": True}, ignored_return=True)
    _wp8_child(children, "parkGantry", arguments={"rehome": False}, ignored_return=True)
    cleanup_condition = {
        "child_order": 1,
        "result_field": "door_ok",
        "equals": True,
    }
    for child in children[2:]:
        child["source_condition"] = dict(cleanup_condition)
    return _wp8_plan(operation, children, exception_policy="propagate", finally_children=[], source_hazards=["destination != 20 || destination != 18", "raw_il_tautology: destination != 20 || destination != 18"])


def compile_finite_plate_operation(
    operation: str, *, source_leaf_available: bool, **inputs: Any,
) -> dict[str, Any]:
    plan = _compile_finite_plate_operation_unchecked(
        operation, source_leaf_available=source_leaf_available, **inputs,
    )
    table_rows = inputs.get("position_table_by_location")
    if isinstance(table_rows, Mapping):
        validate_compiled_wp8_machine_targets(plan, table_rows)
    return plan


def execute_finite_plate_operation(
    plan: Mapping[str, Any], invoke_child: Callable[[Mapping[str, Any]], Any],
    skip_child: Callable[[Mapping[str, Any], Mapping[str, Any]], None] | None = None,
) -> dict[str, Any]:
    """Run and evidence source children in order, retaining literal partial state."""
    completed: list[dict[str, Any]] = []
    completed_by_order: dict[int, Any] = {}
    residual: dict[str, Any] = {}
    failed: str | None = None
    suppressed = False
    finally_names = set(plan.get("finally_children") or [])
    children = list(plan.get("children") or [])
    try:
        for child in children:
            if child["operation"] in finally_names:
                continue
            condition = dict(child.get("source_condition") or {})
            if condition:
                field = str(condition["result_field"])
                expected = condition.get("equals")
                if "any_child_orders" in condition:
                    values = []
                    for source_order in condition["any_child_orders"]:
                        source_result = completed_by_order.get(int(source_order))
                        if isinstance(source_result, Mapping):
                            values.append(source_result.get(field))
                    matched = any(value == expected for value in values)
                    actual: Any = values
                else:
                    source_order = int(condition["child_order"])
                    source_result = completed_by_order.get(source_order)
                    if not isinstance(source_result, Mapping):
                        raise RuntimeError("wp8_source_condition_result_unavailable")
                    actual = source_result.get(field)
                    matched = actual == expected
                if not matched:
                    if condition.get("on_false") == "raise":
                        raise RuntimeError(str(condition.get("error") or "wp8_source_condition_failed"))
                    if skip_child is not None:
                        skip_child(child, condition)
                    skipped = {
                        "source_branch_skipped": True,
                        "condition": condition,
                        "observed": actual,
                    }
                    completed.append({"order": child["order"], "operation": child["operation"], "result": skipped})
                    completed_by_order[int(child["order"])] = skipped
                    continue
            try:
                result = invoke_child(child)
            except Exception:
                failed = str(child["operation"])
                raise
            completed.append({"order": child["order"], "operation": child["operation"], "result": result})
            completed_by_order[int(child["order"])] = result
            mutation = dict(child.get("state_mutation") or {})
            if mutation:
                if set(mutation) == {"plate", "location"}:
                    residual.setdefault("plate_locations", {})[str(mutation["plate"])] = mutation["location"]
                else:
                    residual.update(mutation)
    except Exception:
        if plan.get("exception_policy") == "log_and_suppress":
            suppressed = True
        else:
            raise
    finally:
        for child in children:
            if child["operation"] not in finally_names:
                continue
            result = invoke_child(child)
            completed.append({"order": child["order"], "operation": child["operation"], "result": result})
    return {
        "ok": failed is None,
        "exception_suppressed": suppressed,
        "failed_child": failed,
        "completed_children": completed,
        "residual_state": residual,
        "background_pending": any(not bool(row.get("awaited", True)) for row in children),
    }


def make_wp8_operation_executor(
    *, provider_getter: Callable[[], Any], command_store: Any,
) -> Callable[..., dict[str, Any]]:
    """Bind finite WP8 plans to the provider lease and append-only child ledger."""
    def execute(*, command_id: str, plan: Mapping[str, Any]) -> dict[str, Any]:
        provider = provider_getter()
        if provider is None:
            raise RuntimeError("canonical_deck_provider_unavailable")
        required = (
            getattr(command_store, "persist_wp8_plan", None),
            getattr(command_store, "terminalize_wp8_child", None),
            getattr(command_store, "persist_wp8_state_mutation", None),
            getattr(command_store, "assert_deck_execution_current", None),
        )
        if not all(callable(method) for method in required):
            raise RuntimeError("wp8_durable_store_not_bound")
        lease_factory = getattr(provider, "movement_lease", None)
        lease = lease_factory() if callable(lease_factory) else nullcontext()
        with lease:
            command_store.assert_deck_execution_current(command_id, boundary="before_plan_write")
            stamp_reader = getattr(provider, "deck_owner_authority_stamps", None)
            stamps = dict(stamp_reader()) if callable(stamp_reader) else {}
            settler_binder = getattr(provider, "bind_wp8_background_task_settler", None)
            if callable(settler_binder):
                settler_binder(command_store.settle_wp8_background_task)
            command_store.persist_wp8_plan(command_id, plan, authority_stamps=stamps)

            delivery_attempted = False

            def invoke(child: Mapping[str, Any]) -> Any:
                nonlocal delivery_attempted
                order = int(child["order"])
                command_store.assert_deck_execution_current(command_id, boundary=f"before_child_{order}")
                dispatch = getattr(provider, "execute_wp8_child", None)
                if not callable(dispatch):
                    raise RuntimeError("source_authority_missing:execute_wp8_child")
                operation = str(child["operation"])
                if not bool(child.get("awaited", True)):
                    suffix = "z-home" if operation == "startMoveZPseudoHome" else "gripper-home"
                    delivery_marker = command_store.create_wp8_background_task(
                        command_id, order,
                        task_id=f"{command_id}:{order}:{plan['plan_digest']}:{suffix}",
                        task_kind=operation, plan_digest=str(plan["plan_digest"]),
                        authority_stamps=stamps,
                    )
                else:
                    delivery_marker = command_store.record_delivery_attempt(
                        command_id, work_kind="wp8_child",
                        work_identity=f"child:{order}:{operation}",
                        plan_digest=str(plan["plan_digest"]), authority_stamps=stamps,
                    )
                child_dispatch_attempt_id = str(delivery_marker["dispatch_attempt_id"])
                try:
                    dispatch_child = {
                        **dict(child),
                        "_delivery_identity": {
                            "dispatch_attempt_id": child_dispatch_attempt_id,
                            "ownership_generation": int(stamps["ownership_generation"]),
                            "board_epoch_4": int(stamps["board_epoch_4"]),
                            "board_epoch_5": int(stamps["board_epoch_5"]),
                        },
                    }
                    result = dispatch(
                        dispatch_child,
                        command_id=command_id,
                        child_order=order,
                        plan_digest=str(plan["plan_digest"]),
                    )
                    command_store.assert_deck_execution_current(
                        command_id, boundary=f"after_provider_child_{order}",
                    )
                except Exception as exc:
                    if str(exc).startswith("deck_execution_"):
                        raise
                    child_delivery_attempted = (
                        exc.delivery_attempted
                        if isinstance(exc, DeckExecutionFailure)
                        else True
                    )
                    delivery_attempted = delivery_attempted or child_delivery_attempted
                    command_store.terminalize_wp8_child(
                        command_id, order,
                        state="ambiguous" if child_delivery_attempted else "failed",
                        result={
                            "exception_type": type(exc).__name__, "exception": str(exc),
                            "delivery_attempted": child_delivery_attempted,
                        },
                        dispatch_attempt_id=child_dispatch_attempt_id,
                    )
                    raise
                child_delivery_attempted = bool(
                    isinstance(result, Mapping)
                    and (
                        result.get("delivery_attempted") is True
                        or result.get("controller_command_acknowledged") is True
                    )
                )
                delivery_attempted = delivery_attempted or child_delivery_attempted
                if (
                    isinstance(result, Mapping)
                    and result.get("ok") is not True
                ):
                    command_store.terminalize_wp8_child(
                        command_id, order,
                        state="ambiguous" if child_delivery_attempted else "failed",
                        result=result,
                        dispatch_attempt_id=child_dispatch_attempt_id,
                    )
                    raise DeckExecutionFailure(
                        f"wp8 child failed: {child['operation']}",
                        delivery_attempted=child_delivery_attempted,
                    )
                if not bool(child.get("awaited", True)):
                    if (
                        not isinstance(result, Mapping)
                        or not isinstance(result.get("background_task_id"), str)
                        or not result.get("background_task_id")
                        or result.get("background_task_state") != "running_unawaited"
                    ):
                        command_store.terminalize_wp8_child(
                            command_id, order,
                            state="ambiguous" if child_delivery_attempted else "failed",
                            result=result,
                            dispatch_attempt_id=child_dispatch_attempt_id,
                        )
                        raise DeckExecutionFailure(
                            f"wp8 background task evidence missing: {child['operation']}",
                            delivery_attempted=child_delivery_attempted,
                        )
                    command_store.mark_wp8_background_task(
                        command_id, order, state="running", evidence=dict(result),
                    )
                elif isinstance(result, Mapping) and isinstance(result.get("background_task_id"), str):
                    settle = getattr(command_store, "settle_wp8_background_task", None)
                    if callable(settle):
                        settle(
                            result["background_task_id"],
                            state="completed" if result.get("ok") is True else "failed",
                            evidence=dict(result),
                        )
                mutation = dict(child.get("state_mutation") or {})
                complete_child = getattr(command_store, "complete_wp8_child", None)
                if callable(complete_child):
                    complete_child(
                        command_id, order, result=result,
                        state_mutation=mutation, authority_stamps=stamps,
                        dispatch_attempt_id=child_dispatch_attempt_id,
                    )
                else:
                    command_store.terminalize_wp8_child(
                        command_id, order, state="completed", result=result,
                        dispatch_attempt_id=child_dispatch_attempt_id,
                    )
                    if mutation:
                        command_store.persist_wp8_state_mutation(
                            command_id, order, mutation, authority_stamps=stamps,
                        )
                command_store.assert_deck_execution_current(command_id, boundary=f"after_child_{order}")
                return result

            def skip(child: Mapping[str, Any], condition: Mapping[str, Any]) -> None:
                order = int(child["order"])
                command_store.terminalize_wp8_child(
                    command_id, order, state="completed",
                    result={
                        "source_branch_skipped": True,
                        "condition": dict(condition),
                    },
                )

            try:
                result = execute_finite_plate_operation(plan, invoke, skip)
                background_pending = bool(plan.get("parent_return_allows_background_pending"))
                if not background_pending:
                    command_store.assert_wp8_background_tasks_settled(command_id)
            except Exception as exc:
                failure = {
                    "ok": False, "exception_suppressed": False,
                    "exception_type": type(exc).__name__, "exception": str(exc),
                    "delivery_attempted": delivery_attempted,
                    "outcome_unknown": delivery_attempted,
                    "background_pending": any(not bool(row.get("awaited", True)) for row in plan.get("children", [])),
                }
                finalize = getattr(command_store, "finalize_wp8_operation", None)
                if callable(finalize):
                    finalize(command_id, failure)
                raise
            result = {
                **dict(result), "delivery_attempted": delivery_attempted,
                "background_pending": bool(plan.get("parent_return_allows_background_pending")),
            }
            finalize = getattr(command_store, "finalize_wp8_operation", None)
            if callable(finalize) and not result["background_pending"]:
                finalize(command_id, result)
            return result
    return execute


def make_deck_command_executor(
    *,
    provider_getter: Callable[[], Any],
    position_table_provider: Callable[[], PositionTable],
    command_store: Any,
) -> Callable[..., dict[str, Any]]:
    """Bind the durable worker to one provider-owned named-movement seam."""

    def execute(
        *,
        command_id: str,
        target: str,
        camera_offset: bool,
        expected_ownership_generation: int,
        expected_board_epoch_by_board: Mapping[str, int],
    ) -> dict[str, Any]:
        expected_board_epochs = dict(expected_board_epoch_by_board)
        if (
            set(expected_board_epochs) != {"4", "5"}
            or any(type(value) is not int or value < 0 for value in expected_board_epochs.values())
        ):
            raise DeckExecutionFailure("board_epoch_fence_invalid", delivery_attempted=False)

        def require_expected_board_epochs(authority: DeckAuthoritySnapshot, *, phase: str) -> None:
            observed = {"4": authority.board_epoch_4, "5": authority.board_epoch_5}
            if (
                any(type(value) is not int or value < 0 for value in observed.values())
                or observed != expected_board_epochs
            ):
                raise DeckExecutionFailure(
                    f"board_epoch_fence_stale:{phase}:expected={expected_board_epochs}:observed={observed}",
                    delivery_attempted=False,
                )

        provider = provider_getter()
        if provider is None:
            raise RuntimeError("canonical_deck_provider_unavailable")
        lease_factory = getattr(provider, "movement_lease", None)
        lease = lease_factory() if callable(lease_factory) else nullcontext()
        with lease:
            snapshot_fn = getattr(provider, "deck_authority_snapshot", None)
            if not callable(snapshot_fn):
                raise RuntimeError("canonical_deck_authority_snapshot_unavailable")
            dispatch_authority = DeckAuthoritySnapshot(**dict(
                snapshot_fn(expected_generation=expected_ownership_generation)
            ))
            require_expected_board_epochs(dispatch_authority, phase="before_first_provider_write")
            force = getattr(provider, "force_to_high_home", None)
            if not callable(force):
                raise RuntimeError("source_authority_missing:ForceToHighHome")
            force_result = force(command_id=command_id)
            if not isinstance(force_result, Mapping) or force_result.get("ok") is not True:
                return {
                    "ok": False,
                    "admitted": True,
                    "delivery_attempted": False,
                    "controller_command_acknowledged": False,
                    "controller_completion_verified": False,
                    "hardware_postcondition_verified": False,
                    "semantic_state_committed": False,
                    "physical_effect_verified": False,
                    "error": "force_to_high_home_failed",
                }
            persist_pseudo = getattr(command_store, "persist_deck_pseudo_home", None)
            if callable(persist_pseudo):
                stamp_reader = getattr(provider, "deck_owner_authority_stamps", None)
                stamps = dict(stamp_reader()) if callable(stamp_reader) else {
                    "ownership_generation": dispatch_authority.ownership_generation,
                    "board_epoch_4": dispatch_authority.board_epoch_4,
                    "board_epoch_5": dispatch_authority.board_epoch_5,
                }
                persist_pseudo(
                    command_id, 500, source_operation="ForceToHighHome", **stamps
                )
            authority = DeckAuthoritySnapshot(**dict(snapshot_fn(expected_generation=expected_ownership_generation)))
            require_expected_board_epochs(authority, phase="planning")
            table = position_table_provider()
            catalog = DeckCatalog.from_position_table(table)
            plan = compile_named_location(
                NamedLocationIntent(target=target, camera_offset=camera_offset),
                catalog,
                table,
                authority,
            )
            persist_plan = getattr(command_store, "persist_deck_plan", None)
            if callable(persist_plan):
                persist_plan(command_id, plan)
            terminalize_stage = getattr(command_store, "terminalize_deck_stage", None)
            if callable(terminalize_stage):
                terminalize_stage(
                    command_id,
                    plan.steps[0],
                    state="completed",
                    result=dict(force_result),
                    reason="source_mutation_completed",
                )
                latch_predicates = {
                    "check_latch_status": authority.latch_status,
                    "check_machine_latch_closed": authority.machine_latch_closed,
                }
                for step in plan.steps[1:3]:
                    value = latch_predicates[step.operation]
                    terminalize_stage(
                        command_id,
                        step,
                        state="completed" if value else "failed",
                        result={
                            "value": value,
                            "observation_id": authority.latch_observation_id,
                            "delivery_attempted": False,
                            "physical_motion_commanded": False,
                        },
                        reason=(
                            "source_predicate_satisfied"
                            if value
                            else "source_predicate_not_satisfied"
                        ),
                    )
            if plan.blocked_reason:
                return {
                    "ok": False,
                    "admitted": True,
                    "delivery_attempted": False,
                    "controller_command_acknowledged": False,
                    "controller_completion_verified": False,
                    "hardware_postcondition_verified": False,
                    "semantic_state_committed": False,
                    "physical_effect_verified": False,
                    "error": plan.blocked_reason,
                    "deck_movement": {
                        "target": plan.target,
                        "target_label": catalog.resolve(plan.target).panel_label,
                        "source_branch": plan.source_branch,
                        "controller_completion_verified": False,
                        "semantic_state_committed": False,
                        "physical_observation_verified": False,
                    },
                }
            revalidated = DeckAuthoritySnapshot(**dict(snapshot_fn(expected_generation=expected_ownership_generation)))
            require_expected_board_epochs(revalidated, phase="before_first_movement_write")
            if revalidated.digest != authority.digest:
                raise MovementAuthorityChanged("deck_authority_changed_before_first_tx")
            results: list[Mapping[str, Any]] = []
            delivery_attempted = False
            record_delivery = getattr(command_store, "record_delivery_attempt", None)
            authority_stamps = {
                "ownership_generation": revalidated.ownership_generation,
                "board_epoch_4": revalidated.board_epoch_4,
                "board_epoch_5": revalidated.board_epoch_5,
            }
            for step in plan.steps:
                if step.operation in {"ForceToHighHome", "check_latch_status", "check_machine_latch_closed"}:
                    continue
                method = getattr(provider, step.operation, None)
                if not callable(method):
                    if callable(terminalize_stage):
                        try:
                            terminalize_stage(command_id, step, state="failed", reason=f"source_authority_missing:{step.operation}")
                        except Exception as exc:
                            raise DeckExecutionFailure(
                                f"stage_persistence_failed:{step.operation}:{type(exc).__name__}",
                                delivery_attempted=delivery_attempted,
                                controller_command_acknowledged=any(
                                    row.get("controller_command_acknowledged") is True for row in results
                                ),
                                controller_completion_verified=any(
                                    row.get("controller_completion_verified") is True for row in results
                                ),
                                hardware_postcondition_verified=any(
                                    row.get("hardware_postcondition_verified") is True for row in results
                                ),
                                provider_results=results,
                            ) from exc
                    raise DeckExecutionFailure(
                        f"source_authority_missing:{step.operation}", delivery_attempted=delivery_attempted
                    )
                if not callable(record_delivery):
                    raise DeckExecutionFailure(
                        "named_delivery_ledger_not_bound", delivery_attempted=False,
                    )
                try:
                    record_delivery(
                        command_id,
                        work_kind="named_stage",
                        work_identity=f"stage:{step.order}:{step.operation}",
                        plan_digest=plan.plan_digest,
                        authority_stamps=authority_stamps,
                    )
                except Exception as exc:
                    raise DeckExecutionFailure(
                        f"named_delivery_marker_failed:{step.operation}:{type(exc).__name__}",
                        delivery_attempted=False,
                    ) from exc
                delivery_attempted = True
                try:
                    result = method(
                        **dict(step.arguments or {}),
                        authority_snapshot=asdict(revalidated),
                    )
                except DeckExecutionFailure:
                    raise
                except Exception as exc:
                    try:
                        if callable(terminalize_stage):
                            terminalize_stage(command_id, step, state="ambiguous", reason=f"provider_stage_exception:{type(exc).__name__}")
                    except Exception as persistence_exc:
                        raise DeckExecutionFailure(
                            f"stage_persistence_failed:{step.operation}:{type(persistence_exc).__name__}",
                            delivery_attempted=True,
                            provider_results=results,
                        ) from persistence_exc
                    raise DeckExecutionFailure(
                        f"provider_stage_exception:{step.operation}:{type(exc).__name__}",
                        delivery_attempted=True,
                    ) from exc
                current_results = [*results, dict(result)] if isinstance(result, Mapping) else list(results)
                terminal_truth = bool(
                    isinstance(result, Mapping)
                    and result.get("ok") is True
                    and _controller_terminal_truth(result)
                )
                if not terminal_truth:
                    result_delivery_attempted = bool(
                        not isinstance(result, Mapping)
                        or result.get("delivery_attempted") is not False
                    )
                    terminal_state = (
                        "failed"
                        if not result_delivery_attempted
                        and isinstance(result, Mapping)
                        and result.get("controller_command_acknowledged") is not True
                        else "ambiguous"
                    )
                    try:
                        if callable(terminalize_stage):
                            terminalize_stage(command_id, step, state=terminal_state, result=result if isinstance(result, Mapping) else None, reason="provider_terminal_proof_missing")
                    except Exception as exc:
                        raise DeckExecutionFailure(
                            f"stage_persistence_failed:{step.operation}:{type(exc).__name__}",
                            delivery_attempted=True,
                            controller_command_acknowledged=any(
                                row.get("controller_command_acknowledged") is True for row in current_results
                            ),
                            controller_completion_verified=any(
                                row.get("controller_completion_verified") is True for row in current_results
                            ),
                            hardware_postcondition_verified=any(
                                row.get("hardware_postcondition_verified") is True for row in current_results
                            ),
                            provider_results=current_results,
                        ) from exc
                    return {
                        "ok": False,
                        "admitted": True,
                        "delivery_attempted": result_delivery_attempted,
                        "controller_command_acknowledged": bool(
                            isinstance(result, Mapping) and result.get("controller_command_acknowledged") is True
                        ),
                        "controller_completion_verified": False,
                        "hardware_postcondition_verified": False,
                        "semantic_state_committed": False,
                        "physical_effect_verified": False,
                        "error": f"provider_stage_failed:{step.operation}",
                    }
                if callable(terminalize_stage):
                    try:
                        terminalize_stage(command_id, step, state="completed", result=result)
                    except Exception as exc:
                        raise DeckExecutionFailure(
                            f"stage_persistence_failed:{step.operation}:{type(exc).__name__}",
                            delivery_attempted=True,
                            controller_command_acknowledged=any(
                                row.get("controller_command_acknowledged") is True for row in current_results
                            ),
                            controller_completion_verified=any(
                                row.get("controller_completion_verified") is True for row in current_results
                            ),
                            hardware_postcondition_verified=any(
                                row.get("hardware_postcondition_verified") is True for row in current_results
                            ),
                            provider_results=current_results,
                        ) from exc
                assert isinstance(result, Mapping)
                results.append({str(key): value for key, value in result.items()})
            all_source_noop = bool(results) and all(row.get("source_noop") is True for row in results)
            controller_ack = bool(results) and not all_source_noop and all(
                row.get("source_noop") is True
                or row.get("controller_command_acknowledged") is True
                for row in results
            )
            source_terminal_complete = bool(results) and all(_controller_terminal_truth(row) for row in results)
            controller_complete = source_terminal_complete and not all_source_noop
            postcondition = controller_complete and not all_source_noop and all(
                row.get("source_noop") is True
                or row.get("hardware_postcondition_verified") is True
                for row in results
            )
            semantic_committed = False
            if source_terminal_complete:
                commit = getattr(command_store, "commit_deck_success", None)
                if callable(commit):
                    try:
                        commit(command_id, plan, results)
                    except Exception as exc:
                        raise DeckExecutionFailure(
                            f"semantic_commit_failed:{type(exc).__name__}",
                            delivery_attempted=not all(row.get("source_noop") is True for row in results),
                            controller_command_acknowledged=controller_ack,
                            controller_completion_verified=controller_complete,
                            hardware_postcondition_verified=postcondition,
                            provider_results=results,
                        ) from exc
                semantic_committed = True
            destination = catalog.resolve(plan.target)
            return {
                "ok": source_terminal_complete and semantic_committed,
                "admitted": True,
                "delivery_attempted": any(
                    row.get("source_noop") is not True
                    and row.get("delivery_attempted") is not False
                    for row in results
                ),
                "controller_command_acknowledged": controller_ack,
                "controller_completion_verified": controller_complete,
                "hardware_postcondition_verified": postcondition,
                "semantic_state_committed": semantic_committed,
                "physical_effect_verified": False,
                "authority_snapshot_digest": authority.digest,
                "plan_digest": plan.plan_digest,
                "source_branch": plan.source_branch,
                "provider_results": [dict(row) for row in results],
                "deck_movement": {
                    "target": plan.target,
                    "target_label": destination.panel_label,
                    "source_branch": plan.source_branch,
                    "controller_completion_verified": controller_complete,
                    "semantic_state_committed": semantic_committed,
                    "physical_observation_verified": False,
                },
            }

    return execute

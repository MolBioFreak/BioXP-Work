from __future__ import annotations

from contextlib import contextmanager, nullcontext
from dataclasses import asdict, dataclass
import hashlib
import json
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
        authoritative.pop("captured_at", None)
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
    source_hazards = {
        "ordinary": (
            "ordinary:ROUTE_ALWAYS_TRUE_LOCATION_TEST:retain_decompiled_predicate_pending_raw_il",
        ),
        "barcode": (
            "barcode:ROUTE_ALWAYS_TRUE_LOCATION_TEST:retain_decompiled_predicate_pending_raw_il",
            "barcode:VISION_INVALID_IL_REGIONS:binary_disposition_required_before_live_parity_claim",
        ),
        "park": (
            "park:VISION_INVALID_IL_REGIONS:binary_disposition_required_before_live_parity_claim",
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

    def execute(self, plan: DeckMovementPlan, *, before_first_write: Callable[[DeckMovementPlan], None] | None = None) -> list[Any]:
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
            return results


def compile_mov_execution(*, current_plate_location: int, destination: int, well: str | int, plate_name: str | None = None) -> dict[str, Any]:
    well_id = well_id_from_label(well)
    resolved = {23: 2, 21: 1, 25: 0}.get(destination, destination)
    if plate_name is not None and current_plate_location < 0:
        raise ValueError("source_authority_missing:plate_location")
    return {"schema_version": "bioxp.oem_mov_execution_plan.v1", "current_plate_location": current_plate_location, "destination": resolved, "well_id": well_id, "plate_name": plate_name, "source_operation": "movExecution->scriptmoveTo"}


def compile_finite_plate_operation(operation: str, *, source_leaf_available: bool) -> dict[str, Any]:
    allowed = {"catch_plate", "release_plate", "press_plate", "park_gantry", "waste_sequence"}
    if operation not in allowed:
        raise ValueError("unknown finite deck operation")
    if not source_leaf_available:
        raise RuntimeError(f"source_authority_missing:{operation}")
    return {"operation": operation, "state_update": "success_only", "source_owned": True}


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
            controller_ack = bool(results) and all(
                row.get("source_noop") is True
                or row.get("controller_command_acknowledged") is True
                for row in results
            )
            controller_complete = bool(results) and all(_controller_terminal_truth(row) for row in results)
            postcondition = controller_complete and all(
                row.get("source_noop") is True
                or row.get("hardware_postcondition_verified") is True
                for row in results
            )
            semantic_committed = False
            if controller_complete:
                all_source_noop = all(row.get("source_noop") is True for row in results)
                if not all_source_noop:
                    commit = getattr(command_store, "commit_deck_success", None)
                    if callable(commit):
                        try:
                            commit(command_id, plan, results)
                        except Exception as exc:
                            raise DeckExecutionFailure(
                                f"semantic_commit_failed:{type(exc).__name__}",
                                delivery_attempted=True,
                                controller_command_acknowledged=controller_ack,
                                controller_completion_verified=True,
                                hardware_postcondition_verified=postcondition,
                                provider_results=results,
                            ) from exc
                semantic_committed = True
            destination = catalog.resolve(plan.target)
            return {
                "ok": controller_complete and semantic_committed,
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

"""Source wrapper tests; all native entries are explicit offline doubles."""
from types import SimpleNamespace

import pytest

from bioxp.services.pipette_service import build_oem_pipette_handlers


def action(opcode, arguments=(), key=37):
    return SimpleNamespace(oem_opcode=opcode, source_key=key,
                           source_occurrence_id="oem:7", params={"arguments": arguments})


class Model:
    def __init__(self):
        self.calls = []
        self.logical_tip_present = True
        self.selected = [(1, [0, 24]), (3, [1])]

    def la(self, tokens):
        self.calls.append(("la", tokens))
        return [tokens[0]]

    def select_strip(self, material, volume):
        self.calls.append(("select", material, volume))
        return 12, 1

    def retip_wells(self):
        return self.selected

    def retip_committed(self, tray, wells):
        self.calls.append(("committed", tray, wells))

    def add_pressure_base(self, values):
        self.calls.append(("baseline", values))


class Native:
    def __init__(self, initializations=(True,), tips=False):
        self.calls = []
        self.initializations = iter(initializations)
        self.tips = tips

    def _tip_location_channels(self):
        return [2]

    def set_top_speed(self, value):
        self.calls.append(("speed", value))
        return {"ok": True, "source": "speed"}

    def aspirate_air(self, volume, *, channels, front_air):
        self.calls.append(("air", volume, channels, front_air))
        return {"ok": True, "source": "air"}

    def read_pressure_for_oem_source(self, tip_exists):
        from tests.pressure_source_v1_support import pressure_source_transport
        self.calls.append(("pressure",))
        return pressure_source_transport(loaded_channel=2 if self.tips else None).read_pressure_for_oem_source(tip_exists)

    def reinitialize_pipette(self):
        self.calls.append(("init",))
        return {"ok": next(self.initializations)}

    def query_tip_status_all(self):
        self.calls.append(("tips",))
        return {"ok": True, "source_tip_exists": self.tips}

    def eject_all_tips(self, **kwargs):
        self.calls.append(("eject", kwargs))
        return {"ok": True}


def bindings(native=None, settings=None):
    model = Model()
    state = SimpleNamespace(source_model=model)
    entries = []
    effects = []
    native = native or Native()

    def guard(identity, actual_state):
        assert actual_state is state
        entries.append(identity)

    def effect(name, *values):
        effects.append((name, *values))
        return {"ok": True, "native": name}

    def pipette(name, operation, actual_action, actual_state, identity):
        assert actual_state is state
        assert identity == entries[-1]
        return operation(native)

    args = dict(before_native_entry=guard,
                script_move=lambda loc, row, a, s: effect("move", loc, row),
                publish_location=lambda loc, well, a, s: effect("location", loc, well),
                publish_tip_transition=lambda tray, wells, a, s: effect("retip", tray, wells),
                pipette_call=pipette,
                lift_for_air=lambda a, s: effect("lift"),
                move_to_waste=lambda a, s: effect("waste"),
                move_z=lambda value, a, s: effect("z", value),
                move_x=lambda value, a, s: effect("x", value),
                source_bindings=SimpleNamespace(tip_exists=lambda a, s: native.tips),
                settings=settings if settings is not None else {"LogPressure": False})
    return args, state, native, entries, effects


def test_finite_mapping_does_not_advertise_missing_composites():
    handlers = build_oem_pipette_handlers(before_native_entry=lambda *a: None)
    assert set(handlers) == {"la"}
    args, *_ = bindings()
    args.pop("source_bindings")
    assert set(build_oem_pipette_handlers(**args)) == {"la", "ms", "retip"}
    args["settings"] = {"LogPressure": True}
    assert set(build_oem_pipette_handlers(**args)) == {"la", "ms", "retip", "aa"}


@pytest.mark.parametrize("key", [37, "prepared-key", None])
def test_la_preserves_source_key_and_arguments(key):
    args, state, native, entries, effects = bindings()
    prepared = ("REAGENT_PLATE", "A1", "-2.25", "F")
    result = build_oem_pipette_handlers(**args)["la"](action("la", prepared, key), state)
    assert state.source_model.calls == [("la", [key, "la", *prepared])]
    assert result["source_return"] == [key]
    assert result["physical_effect_verified"] is False
    assert native.calls == effects == []
    assert entries == ["oem:7"]


def test_ms_selects_before_move_and_publishes_exact_source_well():
    args, state, _, entries, effects = bindings()
    result = build_oem_pipette_handlers(**args)["ms"](action("ms", ("material", "4.75")), state)
    assert state.source_model.calls == [("select", "material", 4.75)]
    assert effects == [("move", 12, 1), ("location", 12, 12)]
    assert result["ok"] is True
    assert len(entries) == len(set(entries)) == 2


def test_ms_no_selection_preserves_source_negative_row_not_an_invented_fallback():
    args, state, _, _, effects = bindings()
    state.source_model.select_strip = lambda *_: (9, -1)
    build_oem_pipette_handlers(**args)["ms"](action("ms", ("absent", "1")), state)
    assert effects == [("move", 9, -1), ("location", 9, -12)]


def test_ms_native_failure_does_not_publish_location():
    args, state, _, _, effects = bindings()
    original = {"ok": False, "partial": {"x": "returned", "y": "failed"}}
    args["script_move"] = lambda *a: original
    result = build_oem_pipette_handlers(**args)["ms"](action("ms", ("m", "1")), state)
    assert result["ok"] is False
    assert result["native_results"][0]["result"] == original
    assert effects == []


def test_retip_only_publishes_selected_wells_then_commits_model():
    args, state, native, entries, effects = bindings()
    result = build_oem_pipette_handlers(**args)["retip"](action("retip"), state)
    assert effects == [("retip", 1, [0, 24]), ("retip", 3, [1])]
    assert state.source_model.calls == [("committed", 1, [0, 24]), ("committed", 3, [1])]
    assert result["ok"] is True
    assert native.calls == []
    assert len(entries) == 2


def test_retip_failed_publication_does_not_mutate_model():
    args, state, _, _, _ = bindings()
    args["publish_tip_transition"] = lambda *a: {"ok": False, "outcome": "rejected"}
    result = build_oem_pipette_handlers(**args)["retip"](action("retip"), state)
    assert result["ok"] is False
    assert state.source_model.calls == []
    assert len(result["native_results"]) == 1


@pytest.mark.parametrize("log_pressure", [False, True])
def test_aa_source_sequence_channels_integer_cast_and_pressure_branch(log_pressure):
    settings = {"LogPressure": log_pressure}
    args, state, native, entries, effects = bindings(settings=settings)
    handlers = build_oem_pipette_handlers(**args)
    settings["LogPressure"] = not log_pressure  # builder captures its source setting
    result = handlers["aa"](action("aa", ("8.9",)), state)
    expected = [("speed", 30.0), ("air", 8, [2], True)]
    if not log_pressure:
        expected.append(("pressure",))
    assert native.calls == expected
    assert effects == [("lift",)]
    assert result["ok"] is True
    assert len(entries) == len(set(entries)) == len(expected) + 1


def test_aa_invalid_number_is_source_noop():
    args, state, native, entries, effects = bindings()
    result = build_oem_pipette_handlers(**args)["aa"](action("aa", ("invalid",)), state)
    assert result["source_noop"] is True
    assert native.calls == entries == effects == []


def test_missing_native_outcome_is_not_success():
    args, state, _, _, _ = bindings()
    args["lift_for_air"] = lambda *a: None
    with pytest.raises(RuntimeError, match="explicit native outcome"):
        build_oem_pipette_handlers(**args)["aa"](action("aa", ("1",)), state)


def test_native_exception_is_not_replaced_and_keeps_completed_siblings():
    args, state, _, _, _ = bindings()
    error = RuntimeError("native error")
    def fail(*a):
        raise error
    args["publish_location"] = fail
    with pytest.raises(RuntimeError) as caught:
        build_oem_pipette_handlers(**args)["ms"](action("ms", ("m", "1")), state)
    assert caught.value is error
    assert error.oem_partial_results[0]["operation"] == "scriptmoveTo"


@pytest.mark.parametrize("prior_failures", [0, 1, 2, 3])
def test_init_exact_source_retry_count_and_baseline(prior_failures):
    native = Native([False] * prior_failures + [True])
    args, state, _, entries, effects = bindings(native)
    result = build_oem_pipette_handlers(**args)["iniPipette"](action("iniPipette"), state)
    assert native.calls.count(("init",)) == prior_failures + 1
    assert native.calls.count(("tips",)) == prior_failures
    assert effects == []
    assert result["source_init_return"] == 0
    assert result["ok"] is True
    assert state.source_model.calls == [("baseline", [0.0, 0.0, 0.0, 0.0])]
    assert len(entries) == len(set(entries))


def test_init_final_false_stops_after_four_calls_without_baseline():
    args, state, native, _, effects = bindings(Native([False] * 4))
    result = build_oem_pipette_handlers(**args)["iniPipette"](action("iniPipette"), state)
    assert native.calls.count(("init",)) == 4
    assert native.calls.count(("tips",)) == 3
    assert ("pressure",) not in native.calls
    assert result["ok"] is False
    assert state.source_model.calls == effects == []


def test_init_retry_tip_branch_does_not_substitute_generic_initialize_or_home():
    args, state, native, _, effects = bindings(Native([False, True], tips=True))
    build_oem_pipette_handlers(**args)["iniPipette"](action("iniPipette"), state)
    assert native.calls == [("init",), ("init",), ("tips",),
                            ("eject", {"check_missing_tip": True, "wait": True}), ("pressure",)]
    assert effects == [("waste",), ("z", 80000), ("x", 79000)]
    assert state.source_model.logical_tip_present is True
    assert state.source_model.calls == [("baseline", [0.0, 0.0, 12.25, 0.0])]


def test_init_exception_does_not_enter_source_false_return_retry():
    args, state, native, _, _ = bindings()
    def fail():
        raise RuntimeError("unwound")
    native.reinitialize_pipette = fail
    with pytest.raises(RuntimeError, match="unwound"):
        build_oem_pipette_handlers(**args)["iniPipette"](action("iniPipette"), state)
    assert native.calls == []

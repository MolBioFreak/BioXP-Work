from __future__ import annotations

import argparse
import ast
import hashlib
import json
import re
import subprocess
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

SCHEMA = "bioxp.runtime_audit.entrypoint_denominator.v3"
GENERATOR_PATH = "scripts/generate_bioxp_runtime_audit_entrypoint_denominator.py"
CANONICAL_ARTIFACT_PATH = "docs/specs/evidence/bioxp-runtime-audit-entrypoint-denominator.json"
_GIT_OBJECT_RE = re.compile(r"^[0-9a-f]{40}$")
_FRONTEND_DEFINITION_RE = re.compile(
    r"^(?:export\s+)?(?:(?:async\s+)?function\s+(?P<function>[A-Za-z_][A-Za-z0-9_]*)\b|"
    r"const\s+(?P<const>[A-Za-z_][A-Za-z0-9_]*)\s*=)",
    re.MULTILINE,
)
_FRONTEND_TOP_LEVEL_DEFINITION_RE = re.compile(
    r"^(?:"
    r"(?P<export>export\s+)?(?:(?:async\s+)?function\s+(?P<function>[A-Za-z_][A-Za-z0-9_]*)\b|"
    r"(?:const|let|var)\s+(?P<binding>[A-Za-z_][A-Za-z0-9_]*)\s*=|"
    r"(?:type|interface|class|enum|namespace|module)\s+(?P<other>[A-Za-z_][A-Za-z0-9_]*)\b)"
    r"|(?P<export_default>export\s+default\b)"
    r"|(?P<export_list>export\s*\{)"
    r"|(?P<export_star>export\s*\*)"
    r")",
    re.MULTILINE,
)
_FRONTEND_ENDPOINT_RE = re.compile(r"(?P<quote>['\"`])(?P<path>/api/bioxp/.*?)(?P=quote)", re.DOTALL)
_ES_IMPORT_RE = re.compile(
    r"^[ \t]*import\s+(?P<clause>(?:type\s+)?(?:[A-Za-z_$][A-Za-z0-9_$]*\s*,\s*)?"
    r"(?:\{[^}]*\}|\*\s+as\s+[A-Za-z_$][A-Za-z0-9_$]*|[A-Za-z_$][A-Za-z0-9_$]*))"
    r"\s+from\s+['\"](?P<module>[^'\"]+)['\"]\s*;?",
    re.MULTILINE,
)
_ES_SIDE_EFFECT_IMPORT_RE = re.compile(
    r"^[ \t]*import\s+['\"](?P<module>[^'\"]+)['\"]\s*;?",
    re.MULTILINE,
)
_ES_EXPORT_FROM_RE = re.compile(
    r"^[ \t]*export\s+(?:type\s+)?(?:\*|\{[^}]*\})\s+from\s+"
    r"['\"](?P<module>[^'\"]+)['\"]\s*;?",
    re.MULTILINE,
)
_ES_DYNAMIC_IMPORT_RE = re.compile(
    r"\bimport\(\s*['\"](?P<module>[^'\"]+)['\"]\s*\)",
    re.MULTILINE,
)
_DYNAMIC_COMPONENT_IMPORT_RE = re.compile(
    r"\bconst\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*lazy\s*\(\s*\(\)\s*=>\s*"
    r"import\(\s*['\"](?P<module>[^'\"]+)['\"]\s*\)",
    re.MULTILINE,
)
_FRONTEND_APP_PATH = "platform/frontend/src/App.tsx"
_FRONTEND_CLIENT_PATH = "platform/frontend/src/lib/bioxpClient.ts"
_FRONTEND_CODE_SUFFIXES = (".tsx", ".ts", ".jsx", ".js")
_FRONTEND_OPAQUE_SUFFIXES = frozenset({".css", ".json", ".svg"})
_FRONTEND_COMPONENT_RE = re.compile(
    r"^export\s+function\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*\(",
    re.MULTILINE,
)
_ALLOWED_CONTROL_CLASSES = frozenset(
    {
        "projection_only",
        "hardware_query",
        "pipette_state_command",
        "physical_liquid_command",
        "machine_composite",
        "interrupt",
        "callback_event",
        "read_only_relay",
        "mutation_relay",
        "local_query",
        "local_action",
        "frontend_query",
        "frontend_action",
        "frontend_consumer",
    }
)
_REQUIRED_CORRELATION_FIELDS = (
    "command_id",
    "pipette_operation_id",
    "entrypoint_id",
    "caller_class",
)
_PRIVATE_CALLBACK_IDENTITIES = frozenset(
    {
        "FourPipetteTransport._record_pipette_error",
        "BioXpCanDriver.process_pipette_message",
        "NovoRouter._dispatch",
        "NovoRouter._receive_loop",
    }
)
SOURCE_AUTHORITY_SHA256_POLICY: Mapping[tuple[str, str], str] = {
    ("robot", "src/bioxp/api.py"): "cb7ce60c2a9453b470c1f07bb248ad329b35175511c720d35e928104c0a029e2",
    ("robot", "src/bioxp/operator_controls.py"): "3c99be703b8b83a9609359ae820f4d1b2c0324bbb53f7e46a937d6dd12475492",
    ("robot", "src/bioxp/operator_command_plane.py"): "c6d31a74429b9e2e2bb27583b0710fc3466d305bbc665b1e36b487836d0f8a59",
    ("robot", "src/bioxp/pipette/transport.py"): "ece0884af026ed608619ea20c05adb8982d976c343d854dfd26f5d1bed9726f6",
    ("robot", "src/bioxp/can_driver.py"): "6b9ade7d710f26e241cf08eb304a1a73288760fbb76c64ab0541a4c99b6061fd",
    ("robot", "src/bioxp/novo_router.py"): "226529e5b97626afd163a384bb07232aeaa8d8986bc94bc009efd66b03735006",
    ("robot", "src/bioxp/novo_usb_can.py"): "1494bf1133ec748a6c8ed77f9550d27cbeda5c3842b4d6d5f320830a683677cd",
    ("robot", "src/bioxp/usb_driver.py"): "84b6eff82718541eaa80c26e0ba7f417d6c50187368443ac7be96ac7207a40b8",
    ("bms", "platform/frontend/src/lib/bioxpClient.ts"): "19160a0e608bb38430ab53e5f4f6045147b3c93c2e1acfa767517bf42fd65c06",
}
OPERATOR_CATALOG_AST_SHA256 = "5b61d2274e0ec67011e4879ab4b47e5fd197eff651318e68899c18b084bcefa2"
COMMAND_PLANE_ROUTE_POLICY = frozenset({
    ("POST", "/commands", "admit_command"),
    ("POST", "/methods", "admit_method"),
    ("GET", "/commands/{command_id}", "command_detail"),
    ("GET", "/methods/{method_id}", "method_detail"),
    ("GET", "/methods/{method_id}/commands", "method_commands"),
    ("GET", "/queue", "queue"),
    ("GET", "/transitions", "transitions"),
    ("POST", "/commands/{command_id}/cancel", "cancel_command"),
    ("POST", "/methods/{method_id}/pause", "pause_method"),
    ("POST", "/methods/{method_id}/resume", "resume_method"),
    ("POST", "/methods/{method_id}/cancel", "cancel_method"),
    ("GET", "/recovery", "recovery"),
    ("POST", "/recovery/{recovery_epoch}/resolve", "resolve_recovery"),
    ("GET", "/idempotency/{operation_kind}/{idempotency_key}", "idempotency"),
})
REPORT_ROUTE_POLICY = frozenset({
    ("GET", "/reports/summary", "report_summary"),
    ("GET", "/reports/commands", "report_commands"),
    ("GET", "/reports/commands/{command_id}", "report_command_detail"),
    ("GET", "/reports/commands/{command_id}/transitions", "report_command_transitions"),
    ("GET", "/reports/commands/{command_id}/evidence", "report_command_evidence"),
    ("GET", "/reports/pipette", "report_pipette"),
    ("GET", "/reports/pipette/{pipette_operation_id}", "report_pipette_detail"),
    ("GET", "/reports/pipette/{pipette_operation_id}/channels", "report_pipette_channels"),
    ("GET", "/reports/pipette/{pipette_operation_id}/exchanges", "report_pipette_exchanges"),
    ("GET", "/reports/events", "report_events"),
    ("GET", "/reports/events/{event_id}", "report_event_detail"),
    ("GET", "/reports/pressure-streams", "report_pressure_streams"),
    ("GET", "/reports/pressure-streams/{stream_session_id}", "report_pressure_stream_detail"),
    ("GET", "/reports/pressure-streams/{stream_session_id}/samples", "report_pressure_stream_samples"),
    ("POST", "/reports/exports", "create_export"),
    ("GET", "/reports/exports", "export_list"),
    ("GET", "/reports/exports/{export_id}", "export_metadata"),
    ("GET", "/reports/exports/{export_id}/download", "export_download"),
    ("GET", "/audit-health", "audit_health"),
})

# This is an independent expectation table, not a list populated from discovered
# source. Discovery must match it exactly before rows can be emitted.
ROBOT_ROUTE_POLICY: Mapping[tuple[str, str], tuple[str, bool, bool, str]] = {
    ("GET", "/liquid/application/status"): ("projection_only", False, False, "none"),
    ("POST", "/liquid/application/plan"): ("machine_composite", False, False, "plan"),
    ("GET", "/liquid/status"): ("projection_only", False, False, "none"),
    ("POST", "/liquid/readback"): ("hardware_query", True, False, "query"),
    ("POST", "/liquid/init"): ("pipette_state_command", True, True, "command"),
    ("POST", "/liquid/tip"): ("pipette_state_command", True, True, "command"),
    ("POST", "/liquid/eject-all"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/keep-tip"): ("pipette_state_command", True, True, "command"),
    ("POST", "/liquid/aspirate"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/dispense"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/mix"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/dispense-all"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/aspirate-air"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/dispense-air"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/mix-all"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/fluid-detection"): ("physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/terminate"): ("interrupt", True, True, "interrupt"),
    ("POST", "/liquid/heartbeat"): ("pipette_state_command", True, True, "command"),
    ("POST", "/liquid/diagnoses"): ("hardware_query", True, False, "query"),
    ("POST", "/liquid/error-log"): ("hardware_query", True, False, "query"),
    ("POST", "/liquid/tip-status"): ("hardware_query", True, False, "query"),
    ("GET", "/liquid/data"): ("projection_only", False, False, "none"),
    ("POST", "/liquid/data"): ("hardware_query", True, False, "query"),
    ("GET", "/liquid/fluid-detection/{channel}/timestamp"): ("projection_only", False, False, "none"),
    ("POST", "/liquid/fluid-detection/{channel}/timestamp"): ("hardware_query", True, False, "query"),
    ("POST", "/liquid/set-top-speed"): ("pipette_state_command", True, True, "command"),
    ("GET", "/liquid/pressure"): ("projection_only", False, False, "none"),
    ("POST", "/liquid/pressure"): ("hardware_query", True, False, "query"),
    ("POST", "/liquid/firmware"): ("hardware_query", True, False, "query"),
    ("POST", "/liquid/reinitialize"): ("pipette_state_command", True, True, "command"),
    ("GET", "/liquid/condition"): ("projection_only", False, False, "none"),
    ("POST", "/liquid/condition"): ("hardware_query", True, False, "query"),
    ("GET", "/liquid/status/readback"): ("projection_only", False, False, "none"),
    ("POST", "/liquid/status/readback"): ("hardware_query", True, False, "query"),
}

# The route inventory above defines the required semantics. These two tables
# independently bind every call occurrence and the exact calls that can reach
# the pipette coordinator or transport for each mounted liquid route.
ROBOT_ROUTE_CALL_SHA256_POLICY: Mapping[tuple[str, str], str] = {
    ("GET", "/liquid/application/status"): "0ac889430f9ea4df3e67c79ded9a8a091f78f4d8e3da8c9a0ef42460770f1c25",
    ("GET", "/liquid/condition"): "f8714cd780837a723846fe0426d9698a390a54576342d00e562acd65df4d1cbb",
    ("GET", "/liquid/data"): "f8714cd780837a723846fe0426d9698a390a54576342d00e562acd65df4d1cbb",
    ("GET", "/liquid/fluid-detection/{channel}/timestamp"): "f8714cd780837a723846fe0426d9698a390a54576342d00e562acd65df4d1cbb",
    ("GET", "/liquid/pressure"): "f8714cd780837a723846fe0426d9698a390a54576342d00e562acd65df4d1cbb",
    ("GET", "/liquid/status"): "74952fc3f962f6368c651f56842ab2bd5015cd53924c8a3b1638daeb8aed7a9e",
    ("GET", "/liquid/status/readback"): "f8714cd780837a723846fe0426d9698a390a54576342d00e562acd65df4d1cbb",
    ("POST", "/liquid/application/plan"): "b909d728814e9844e94631b3fd98b0ae5def077808cb9dd5895eaaa450534dd7",
    ("POST", "/liquid/aspirate"): "a7a780a63e2490f22ab9347de3e794c3f0a018db7088d474fb294f74ec5cf0ba",
    ("POST", "/liquid/aspirate-air"): "31db571c2f79b8f343114501f355beba1dc884da6960d744b3433ad71d9b7bba",
    ("POST", "/liquid/condition"): "92d4396acbda060bda970a89574b72bba7fbe91755560358409d709bf06fd8e4",
    ("POST", "/liquid/data"): "4aa7e1be91aa3dd95a3ba3d8a3de4eaf681e25a2d3bd78090eadad45ef45eec8",
    ("POST", "/liquid/diagnoses"): "477c58f26edced650bd15f4e7a4fc67301251d3e4dc914af1b50d3cd39e78a05",
    ("POST", "/liquid/dispense"): "0c6a545e3aa82d34ca4bc58229aae9eda3ec3a48b2cbff4ce6e23b277c417d4d",
    ("POST", "/liquid/dispense-air"): "5f4ce68a8a12332de6e17cfecfa89f35713cb8fef8118c1cb3193a783db28e8e",
    ("POST", "/liquid/dispense-all"): "3fe0a648711e68f35c02656ae5276fc0cc54410c615a431ad912db70f9fe590d",
    ("POST", "/liquid/eject-all"): "ce680a8a7a51b393d5b31447795a34ca7bce1bf29bc34532aea01d0f2e74d1ac",
    ("POST", "/liquid/error-log"): "20db5b8fd5194e06f1c77111f1dd61ec2646cc92df52b52332dd38211096ca58",
    ("POST", "/liquid/firmware"): "dd8e13ad448058711a46418b23392e7b27a703f9becec28c4745cc15fc00245c",
    ("POST", "/liquid/fluid-detection"): "b9abf4bba5a4e28c7b5c668c655504ef090762f3369610fecccad41f5287196a",
    ("POST", "/liquid/fluid-detection/{channel}/timestamp"): "3ef330c1456e72214bc71fea8c10a91505f362442b15672c7122447a04b52bcb",
    ("POST", "/liquid/heartbeat"): "947b03fcf27088490af70e1cca08ee04f5a6b983b8d988c423b9054728f37bf8",
    ("POST", "/liquid/init"): "7a3de997774a8bd01225472d4fd238f4427b22389f5cf99e31693e1578d6b2ff",
    ("POST", "/liquid/keep-tip"): "5b9f949dfe060aa61493d066194fe32d221590078daa74aa8ddb04dd9a766bf0",
    ("POST", "/liquid/mix"): "b8dcf54b5e7f6a847008bb3bbce6c9ef22f65dabb6715dbc73218722406f7d43",
    ("POST", "/liquid/mix-all"): "436ccce3d672299f22ed2813c1d115c7cf5919d50db81011e9d49e5383de570e",
    ("POST", "/liquid/pressure"): "47188fa1ccd50f76c400aa82f96d18379da8975f21c90d482967a57c9963f9eb",
    ("POST", "/liquid/readback"): "89a21a6b0963fb6635862273fae729939703d5ac723b7be863bc1a56948726a5",
    ("POST", "/liquid/reinitialize"): "5ca87677b6254bd3a645f71bc2b09395820391205fcc50f24e05a9ae3d764208",
    ("POST", "/liquid/set-top-speed"): "ad2ea5a9d65996bcf83785680dbd453033ec4e05582bd4944d81350586a7b92a",
    ("POST", "/liquid/status/readback"): "1e0dd988adb1fd16af21381d16152ad779140a8e2c79adf4afb0887d271852f5",
    ("POST", "/liquid/terminate"): "882ac23e0b0c5d591526b72b63ea5d8938a21a91f56d8169f6aa3f4cdb65947d",
    ("POST", "/liquid/tip"): "675f123f315e74b368a4d4b8b90b53d87201611375ac03d2144e914a32b7e87c",
    ("POST", "/liquid/tip-status"): "2a309527b786394b0b79a06e4c6ed4ca0a409504916d5d1196228dec1139fed0",
}

ROBOT_ROUTE_DISPATCH_CALL_POLICY: Mapping[tuple[str, str], tuple[str, ...]] = {
    ("GET", "/liquid/application/status"): (),
    ("GET", "/liquid/condition"): (),
    ("GET", "/liquid/data"): (),
    ("GET", "/liquid/fluid-detection/{channel}/timestamp"): (),
    ("GET", "/liquid/pressure"): (),
    ("GET", "/liquid/status"): (),
    ("GET", "/liquid/status/readback"): (),
    ("POST", "/liquid/application/plan"): (
        "_pipette_application.plan_detect_fluid",
        "_pipette_application.plan_load_tip",
        "_pipette_application.plan_move_to_waste",
        "_pipette_application.plan_plunger",
    ),
    ("POST", "/liquid/aspirate"): ("run_pipette_aspirate_command",),
    ("POST", "/liquid/aspirate-air"): ("run_pipette_operation", "transport.aspirate_air"),
    ("POST", "/liquid/condition"): ("run_pipette_operation", "transport.checked_pipette_condition"),
    ("POST", "/liquid/data"): ("run_pipette_operation", "transport.get_data"),
    ("POST", "/liquid/diagnoses"): ("run_pipette_operation", "transport.execute_diagnoses"),
    ("POST", "/liquid/dispense"): ("run_pipette_dispense_command",),
    ("POST", "/liquid/dispense-air"): ("run_pipette_operation", "transport.dispense_air"),
    ("POST", "/liquid/dispense-all"): ("run_pipette_operation", "transport.dispense_all"),
    ("POST", "/liquid/eject-all"): ("run_pipette_operation", "transport.eject_all_tips"),
    ("POST", "/liquid/error-log"): ("run_pipette_operation", "transport.query_error_log"),
    ("POST", "/liquid/firmware"): ("run_pipette_operation", "transport.query_firmware"),
    ("POST", "/liquid/fluid-detection"): ("run_pipette_operation", "transport.detect_fluid"),
    ("POST", "/liquid/fluid-detection/{channel}/timestamp"): ("run_pipette_operation", "transport.get_fluid_timestamp"),
    ("POST", "/liquid/heartbeat"): ("run_pipette_operation", "transport.heartbeat"),
    ("POST", "/liquid/init"): ("run_pipette_init_command",),
    ("POST", "/liquid/keep-tip"): ("run_pipette_operation", "transport.KeepTip"),
    ("POST", "/liquid/mix"): ("run_pipette_mix_command",),
    ("POST", "/liquid/mix-all"): ("run_pipette_operation", "transport.mix_all"),
    ("POST", "/liquid/pressure"): ("run_pipette_operation", "transport.read_pressure"),
    ("POST", "/liquid/readback"): ("run_pipette_operation", "transport.readback_all"),
    ("POST", "/liquid/reinitialize"): ("run_pipette_operation", "transport.reinitialize_pipette"),
    ("POST", "/liquid/set-top-speed"): ("run_pipette_operation", "transport.set_top_speed"),
    ("POST", "/liquid/status/readback"): ("run_pipette_operation", "transport.checked_pipette_status"),
    ("POST", "/liquid/terminate"): ("run_pipette_operation", "transport.terminate"),
    ("POST", "/liquid/tip"): ("run_pipette_tip_command",),
    ("POST", "/liquid/tip-status"): ("run_pipette_operation", "transport.query_tip_status_all"),
}

OPERATOR_CATALOG_POLICY: Mapping[
    tuple[str, str], tuple[str, str, bool, bool, str]
] = {
    ("GET", "/liquid/application/status"): ("liquid_application_status", "projection_only", False, False, "none"),
    ("POST", "/liquid/application/plan"): ("liquid_application_plan", "machine_composite", False, False, "plan"),
    ("GET", "/liquid/status"): ("liquid_status", "projection_only", False, False, "none"),
    ("POST", "/liquid/readback"): ("liquid_readback", "hardware_query", True, False, "query"),
    ("POST", "/liquid/init"): ("liquid_init", "pipette_state_command", True, True, "command"),
    ("POST", "/liquid/tip"): ("liquid_tip", "pipette_state_command", True, True, "command"),
    ("POST", "/liquid/eject-all"): ("liquid_eject_all", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/keep-tip"): ("liquid_keep_tip", "pipette_state_command", True, True, "command"),
    ("POST", "/liquid/aspirate"): ("liquid_aspirate", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/dispense"): ("liquid_dispense", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/mix"): ("liquid_mix", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/dispense-all"): ("liquid_dispense_all", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/aspirate-air"): ("liquid_aspirate_air", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/dispense-air"): ("liquid_dispense_air", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/mix-all"): ("liquid_mix_all", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/fluid-detection"): ("liquid_fluid_detection", "physical_liquid_command", True, True, "command"),
    ("POST", "/liquid/terminate"): ("liquid_terminate", "interrupt", True, True, "interrupt"),
    ("POST", "/liquid/heartbeat"): ("liquid_heartbeat", "pipette_state_command", True, True, "command"),
    ("POST", "/liquid/diagnoses"): ("liquid_diagnoses", "hardware_query", True, False, "query"),
    ("POST", "/liquid/error-log"): ("liquid_error_log", "hardware_query", True, False, "query"),
    ("POST", "/liquid/tip-status"): ("liquid_tip_status", "hardware_query", True, False, "query"),
    ("GET", "/liquid/data"): ("liquid_data_get_retired", "projection_only", False, False, "none"),
    ("POST", "/liquid/data"): ("liquid_data", "hardware_query", True, False, "query"),
    ("GET", "/liquid/fluid-detection/{channel}/timestamp"): ("liquid_fluid_timestamp_get_retired", "projection_only", False, False, "none"),
    ("POST", "/liquid/fluid-detection/{channel}/timestamp"): ("liquid_fluid_timestamp", "hardware_query", True, False, "query"),
    ("POST", "/liquid/set-top-speed"): ("liquid_set_top_speed", "pipette_state_command", True, True, "command"),
    ("GET", "/liquid/pressure"): ("liquid_pressure_get_retired", "projection_only", False, False, "none"),
    ("POST", "/liquid/pressure"): ("liquid_pressure", "hardware_query", True, False, "query"),
    ("POST", "/liquid/firmware"): ("liquid_firmware", "hardware_query", True, False, "query"),
    ("POST", "/liquid/reinitialize"): ("liquid_reinitialize", "pipette_state_command", True, True, "command"),
    ("GET", "/liquid/condition"): ("liquid_condition_get_retired", "projection_only", False, False, "none"),
    ("POST", "/liquid/condition"): ("liquid_condition", "hardware_query", True, False, "query"),
    ("GET", "/liquid/status/readback"): ("liquid_status_readback_get_retired", "projection_only", False, False, "none"),
    ("POST", "/liquid/status/readback"): ("liquid_status_readback", "hardware_query", True, False, "query"),
}

PROTOCOL_HANDLER_POLICY: Mapping[str, str] = {
    "MOVE": "_protocol_live_move_handler",
    "PIPETTE_INIT": "_protocol_live_pipette_handler",
    "PIPETTE_TIP": "_protocol_live_pipette_handler",
    "TIP_EJECT": "_protocol_live_pipette_handler",
    "PIPETTE_ASPIRATE": "_protocol_live_pipette_handler",
    "PIPETTE_DISPENSE": "_protocol_live_pipette_handler",
    "PIPETTE_MIX": "_protocol_live_pipette_handler",
}

# Pipette denominator classification is separate from complete registry ownership.
PROTOCOL_POLICY: Mapping[
    str, tuple[str, str, bool, str, bool, str]
] = {
    "PIPETTE_INIT": ("_protocol_live_pipette_handler", "pipette_state_command", True, "command", True, "protocol_live_contract_and_robot_durable_claim"),
    "PIPETTE_TIP": ("_protocol_live_pipette_handler", "pipette_state_command", True, "command", True, "protocol_live_contract_and_robot_durable_claim"),
    "TIP_EJECT": ("_protocol_live_pipette_handler", "physical_liquid_command", True, "command", True, "protocol_live_contract_and_robot_durable_claim"),
    "PIPETTE_ASPIRATE": ("_protocol_live_pipette_handler", "physical_liquid_command", True, "command", True, "protocol_live_contract_and_robot_durable_claim"),
    "PIPETTE_DISPENSE": ("_protocol_live_pipette_handler", "physical_liquid_command", True, "command", True, "protocol_live_contract_and_robot_durable_claim"),
    "PIPETTE_MIX": ("_protocol_live_pipette_handler", "physical_liquid_command", True, "command", True, "protocol_live_contract_and_robot_durable_claim"),
}

LIFECYCLE_POLICY: Sequence[tuple[str, str, str, bool, bool]] = (
    ("src/bioxp/api.py", "_constructor_pipette_action", "pipette_state_command", True, True),
    ("src/bioxp/oem_serial206_initialization.py", "query_tip_status", "hardware_query", True, False),
    ("src/bioxp/oem_serial206_initialization.py", "query_all_pipette_tip_states", "hardware_query", True, False),
    ("src/bioxp/oem_serial206_initialization.py", "eject_all_tips", "physical_liquid_command", True, True),
    ("src/bioxp/oem_serial206_initialization.py", "eject_all_pipette_tips_for_oem_startup", "physical_liquid_command", True, True),
    ("src/bioxp/oem_serial206_initialization.py", "initiate_pipette_group", "pipette_state_command", True, True),
    ("src/bioxp/oem_serial206_initialization.py", "initiate_pipette_group_for_oem_initialize_motion", "pipette_state_command", True, True),
    ("src/bioxp/oem_serial206_initialization.py", "checked_pipette_status_for_oem_initialize_motion", "hardware_query", True, False),
)

_TRANSPORT_METHOD_POLICY: Mapping[
    tuple[str, str, str], tuple[str, str, bool, str, bool]
] = {
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "__init__"): ("CanPipetteTransport.__init__", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_default_driver_factory"): ("CanPipetteTransport._default_driver_factory", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_get_driver"): ("CanPipetteTransport._get_driver", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_status_payload"): ("CanPipetteTransport._status_payload", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_driver_evidence"): ("CanPipetteTransport._driver_evidence", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "apply_completed_effect"): ("CanPipetteTransport.apply_completed_effect", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_require_initialized"): ("CanPipetteTransport._require_initialized", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_assert_driver_result"): ("CanPipetteTransport._assert_driver_result", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_safe_query_tip_status"): ("CanPipetteTransport._safe_query_tip_status", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_safe_query_pressure"): ("CanPipetteTransport._safe_query_pressure", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_require_tip_loaded"): ("CanPipetteTransport._require_tip_loaded", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "get_status"): ("CanPipetteTransport.get_status", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "initialize"): ("CanPipetteTransport.initialize", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "set_tip"): ("CanPipetteTransport.set_tip", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "aspirate"): ("CanPipetteTransport.aspirate", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "dispense"): ("CanPipetteTransport.dispense", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "mix"): ("CanPipetteTransport.mix", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "dispense_all"): ("CanPipetteTransport.dispense_all", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "aspirate_air"): ("CanPipetteTransport.aspirate_air", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "dispense_air"): ("CanPipetteTransport.dispense_air", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "wait_for_completion"): ("CanPipetteTransport.wait_for_completion", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "terminate"): ("CanPipetteTransport.terminate", "interrupt", True, "interrupt", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "set_top_speed"): ("CanPipetteTransport.set_top_speed", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "heartbeat"): ("CanPipetteTransport.heartbeat", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "query_pressure"): ("CanPipetteTransport.query_pressure", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "query_firmware"): ("CanPipetteTransport.query_firmware", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "query_error_log"): ("CanPipetteTransport.query_error_log", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "execute_diagnoses"): ("CanPipetteTransport.execute_diagnoses", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "get_data"): ("CanPipetteTransport.get_data", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "get_all_data"): ("CanPipetteTransport.get_all_data", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "start_fluid_detection"): ("CanPipetteTransport.start_fluid_detection", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "close"): ("CanPipetteTransport.close", "interrupt", True, "interrupt", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "__init__"): ("FourPipetteTransport.__init__", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_record_pipette_error"): ("FourPipetteTransport._record_pipette_error", "callback_event", False, "callback", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_channel_status"): ("FourPipetteTransport._channel_status", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "get_status"): ("FourPipetteTransport.get_status", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "readback_all"): ("FourPipetteTransport.readback_all", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_run_group_cycle"): ("FourPipetteTransport._run_group_cycle", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_query_condition"): ("FourPipetteTransport._query_condition", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_query_status"): ("FourPipetteTransport._query_status", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "initiate_group_once_for_oem_initialize_motion"): ("FourPipetteTransport.initiate_group_once_for_oem_initialize_motion", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "checked_pipette_status_for_oem_initialize_motion"): ("FourPipetteTransport.checked_pipette_status_for_oem_initialize_motion", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "initialize"): ("FourPipetteTransport.initialize", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_selected_channels"): ("FourPipetteTransport._selected_channels", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_tip_eligibility"): ("FourPipetteTransport._tip_eligibility", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_cached_tip_channels"): ("FourPipetteTransport._cached_tip_channels", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "reinitialize_pipette"): ("FourPipetteTransport.reinitialize_pipette", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "checked_pipette_condition"): ("FourPipetteTransport.checked_pipette_condition", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "checked_pipette_status"): ("FourPipetteTransport.checked_pipette_status", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "read_pressure"): ("FourPipetteTransport.read_pressure", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "set_top_speed"): ("FourPipetteTransport.set_top_speed", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "query_firmware"): ("FourPipetteTransport.query_firmware", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "query_error_log"): ("FourPipetteTransport.query_error_log", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "execute_diagnoses"): ("FourPipetteTransport.execute_diagnoses", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "get_data"): ("FourPipetteTransport.get_data", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "terminate"): ("FourPipetteTransport.terminate", "interrupt", True, "interrupt", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "heartbeat"): ("FourPipetteTransport.heartbeat", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "disable_heartbeat"): ("FourPipetteTransport.disable_heartbeat", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_channels_from_metadata"): ("FourPipetteTransport._channels_from_metadata", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_require_physical_command_admission"): ("FourPipetteTransport._require_physical_command_admission", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_require_liquid_mutation"): ("FourPipetteTransport._require_liquid_mutation", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_run_group_liquid_operation"): ("FourPipetteTransport._run_group_liquid_operation", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_liquid_timeout_ms"): ("FourPipetteTransport._liquid_timeout_ms", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "dispense_all"): ("FourPipetteTransport.dispense_all", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "aspirate_air"): ("FourPipetteTransport.aspirate_air", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "dispense_air"): ("FourPipetteTransport.dispense_air", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "mix_all"): ("FourPipetteTransport.mix_all", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "detect_fluid"): ("FourPipetteTransport.detect_fluid", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "get_fluid_timestamp"): ("FourPipetteTransport.get_fluid_timestamp", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "waitforcompletion"): ("FourPipetteTransport.waitforcompletion", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "queryIndividualTipStatus"): ("FourPipetteTransport.queryIndividualTipStatus", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "loadTip"): ("FourPipetteTransport.loadTip", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "set_tip"): ("FourPipetteTransport.set_tip", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_eject_tip_channels_once"): ("FourPipetteTransport._eject_tip_channels_once", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "eject_all_tips"): ("FourPipetteTransport.eject_all_tips", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "KeepTip"): ("FourPipetteTransport.KeepTip", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "verifyEjectTip"): ("FourPipetteTransport.verifyEjectTip", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "query_tip_status_all"): ("FourPipetteTransport.query_tip_status_all", "hardware_query", False, "query", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "eject_all_tips_for_oem_startup"): ("FourPipetteTransport.eject_all_tips_for_oem_startup", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_eject_all_tips_for_oem_startup_locked"): ("FourPipetteTransport._eject_all_tips_for_oem_startup_locked", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_mutation_blocked"): ("FourPipetteTransport._mutation_blocked", "pipette_state_command", True, "state_command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "aspirate"): ("FourPipetteTransport.aspirate", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "dispense"): ("FourPipetteTransport.dispense", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "mix"): ("FourPipetteTransport.mix", "physical_liquid_command", True, "command", False),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "close"): ("FourPipetteTransport.close", "interrupt", True, "interrupt", False),
}
TRANSPORT_ENTRYPOINT_POLICY: Mapping[
    tuple[str, str, str], tuple[str, str, bool, str, bool]
] = {
    key: value
    for key, value in _TRANSPORT_METHOD_POLICY.items()
    if not key[2].startswith("_")
}

CALLBACK_POLICY: Mapping[
    tuple[str, str, str], tuple[str, str, bool, str, bool]
] = {
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_record_pipette_error"): ("FourPipetteTransport._record_pipette_error", "callback_event", False, "callback", False),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "process_pipette_message"): ("BioXpCanDriver.process_pipette_message", "callback_event", False, "callback", False),
    ("src/bioxp/novo_router.py", "NovoRouter", "_dispatch"): ("NovoRouter._dispatch", "callback_event", False, "callback", False),
    ("src/bioxp/novo_router.py", "NovoRouter", "_receive_loop"): ("NovoRouter._receive_loop", "callback_event", False, "callback", False),
}
CALLBACK_OWNER_POLICY: Sequence[tuple[str, str]] = (
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport"),
    ("src/bioxp/can_driver.py", "BioXpCanDriver"),
    ("src/bioxp/novo_router.py", "NovoRouter"),
)
CALLBACK_SOURCE_PATHS: Sequence[str] = (
    "src/bioxp/pipette/transport.py",
    "src/bioxp/can_driver.py",
    "src/bioxp/novo_router.py",
    "src/bioxp/novo_usb_can.py",
    "src/bioxp/usb_driver.py",
)
CALLBACK_CALLABLE_PARAMETER_POLICY = frozenset({
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "__init__", "error_callback"),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "__init__", "error_callback"),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_run_group_liquid_operation", "callback"),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "__init__", "pipette_error_callback"),
    ("src/bioxp/novo_router.py", "NovoRouter", "transact_many", "matcher"),
    ("src/bioxp/novo_router.py", "NovoRouter", "transact", "matcher"),
    ("src/bioxp/novo_usb_can.py", "BioXpNovoUsbDriver", "__init__", "pipette_error_callback"),
})
CALLBACK_CALLABLE_PARAMETER_EXCLUSION_POLICY = frozenset({
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "__init__", "driver_factory"),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "__init__", "sleep"),
    ("src/bioxp/novo_router.py", "NovoRouter", "__init__", "decode"),
    ("src/bioxp/novo_router.py", "NovoRouter", "__init__", "clock"),
})
CALLBACK_SIGNATURE_POLICY: Mapping[tuple[str, str, str], tuple[tuple[str, ...], tuple[str, ...]]] = {
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_record_pipette_error"): (("self", "channel", "error_code"), ()),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "process_pipette_message"): (("self", "dlc", "message"), ("arbitration_id", "command_name", "received_at")),
    ("src/bioxp/novo_router.py", "NovoRouter", "_dispatch"): (("self", "frame"), ()),
    ("src/bioxp/novo_router.py", "NovoRouter", "_receive_loop"): (("self",), ()),
}
CALLBACK_DEFAULT_POLICY: Mapping[tuple[str, str, str], tuple[tuple[str, ...], tuple[str | None, ...]]] = {
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_record_pipette_error"): ((), ()),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "process_pipette_message"): ((), ("None", "None", "None")),
    ("src/bioxp/novo_router.py", "NovoRouter", "_dispatch"): ((), ()),
    ("src/bioxp/novo_router.py", "NovoRouter", "_receive_loop"): ((), ()),
}
CALLBACK_DECLARATION_POLICY: Mapping[tuple[str, str, str], str] = {
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_record_pipette_error"): "def _record_pipette_error(self, channel: int, error_code: int) -> None:\n    pass",
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "process_pipette_message"): "def process_pipette_message(self, dlc: int, message: list[int] | bytes | bytearray, *, arbitration_id: int | None = None, command_name: str | None = None, received_at: float | None = None) -> dict[str, Any]:\n    pass",
    ("src/bioxp/novo_router.py", "NovoRouter", "_dispatch"): "def _dispatch(self, frame: NovoFrame) -> None:\n    pass",
    ("src/bioxp/novo_router.py", "NovoRouter", "_receive_loop"): "def _receive_loop(self) -> None:\n    pass",
}
CALLBACK_DATAFLOW_POLICY: Mapping[
    tuple[str, str, str],
    tuple[tuple[str, tuple[str, ...], int], ...],
] = {
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_record_pipette_error"): (
        (
            "self._last_error = {'channel': int(channel), 'error_code': int(error_code), 'source': 'ClassPipetteCollection.handlePipetteMessage'}",
            ("channel", "error_code"),
            1,
        ),
        (
            "self._error_callback(int(channel), int(error_code))",
            ("channel", "error_code"),
            1,
        ),
    ),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "process_pipette_message"): (
        (
            "selected = command_name or self._pipette_last_command",
            ("command_name", "self._pipette_last_command"),
            1,
        ),
        (
            "self._pipette_message_state = process_pipette_message(dlc, message, arbitration_id=arbitration_id, command_name=selected, state=getattr(self, '_pipette_message_state', {}), received_at=received_at)",
            ("dlc", "message", "arbitration_id", "selected", "received_at"),
            1,
        ),
        (
            "event_error_code = self._pipette_message_state.get('event_error_code')",
            ("self._pipette_message_state",),
            1,
        ),
        (
            "callback(int(self.pipette_id), int(event_error_code))",
            ("self.pipette_id", "event_error_code"),
            1,
        ),
        (
            "return dict(self._pipette_message_state)",
            ("self._pipette_message_state",),
            1,
        ),
    ),
    ("src/bioxp/novo_router.py", "NovoRouter", "_receive_loop"): (
        (
            "raw = bytes(self.ep_in.read(self.read_size, timeout=self.read_timeout_ms))",
            ("self.ep_in", "self.read_size", "self.read_timeout_ms"),
            1,
        ),
        (
            "frame = self._decode_record(self._decode(raw), raw, received_at)",
            ("raw", "received_at"),
            1,
        ),
        (
            "self._dispatch(frame)",
            ("frame",),
            1,
        ),
    ),
    ("src/bioxp/novo_router.py", "NovoRouter", "_dispatch"): (
        ("completion.ack_frame = frame", ("frame",), 1),
        ("completion.frame = frame", ("frame",), 2),
        ("decision = pending.matcher(frame)", ("frame",), 1),
        ("pending.frames.append(frame)", ("frame",), 1),
        ("pending.skipped.append(frame.provenance())", ("frame",), 1),
        ("self._queues[queue_name].append(frame)", ("queue_name", "frame"), 1),
    ),
}
CALLBACK_WIRING_STATEMENT_POLICY: Mapping[tuple[str, str, str], tuple[str, ...]] = {
    ("src/bioxp/pipette/transport.py", "", "build_default_pipette_transport"): (
        "return FourPipetteTransport(transports, error_callback=error_callback)",
    ),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "__init__"): (
        "self._error_callback = error_callback",
    ),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "__init__"): (
        "self._error_callback = error_callback",
        "transport._error_callback = self._record_pipette_error",
    ),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_record_pipette_error"): (
        "self._error_callback(int(channel), int(error_code))",
    ),
    ("src/bioxp/pipette/transport.py", "FourPipetteTransport", "_run_group_liquid_operation"): (
        "result = callback(channel, self._transports[channel], defer_completion)",
    ),
    ("src/bioxp/pipette/transport.py", "CanPipetteTransport", "_get_driver"): (
        'setattr(self._driver, "_pipette_error_callback", self._error_callback)',
    ),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "__init__"): (
        "self._pipette_error_callback = pipette_error_callback",
    ),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "process_pipette_message"): (
        '''self._pipette_message_state = process_pipette_message(
    dlc,
    message,
    arbitration_id=arbitration_id,
    command_name=selected,
    state=getattr(self, "_pipette_message_state", {}),
    received_at=received_at,
)''',
        'callback = getattr(self, "_pipette_error_callback", None)',
        "callback(int(self.pipette_id), int(event_error_code))",
    ),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "_send_packet"): (
        '''message_state = self.process_pipette_message(
    int(ack.get("dlc", len(ack.get("data", [])))),
    list(ack.get("data", [])),
    arbitration_id=ack.get("arbitration_id"),
    command_name=command_name,
    received_at=ack.get("received_at"),
)''',
    ),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "_apply_pipette_provenance"): (
        '''state = self.process_pipette_message(
    int(frame.get("dlc", len(frame.get("data", [])))),
    list(frame.get("data", [])),
    arbitration_id=frame.get("arbitration_id"),
    command_name=command_name,
    received_at=frame.get("received_at"),
)''',
        '''state = self.process_pipette_message(
    int(completion.get("observed_rx_dlc", len(completion["data"]))),
    list(completion["data"]),
    arbitration_id=completion.get("observed_rx_id"),
    command_name=completion.get("command_name") or command_name,
    received_at=completion.get("receive_timestamp"),
)''',
    ),
    ("src/bioxp/can_driver.py", "BioXpCanDriver", "_enrich_pipette_completion"): (
        '''state = self.process_pipette_message(
    int(result.get("observed_rx_dlc", len(data))),
    data,
    arbitration_id=result.get("observed_rx_id"),
    command_name=result.get("command_name") or self._pipette_last_command,
    received_at=result.get("receive_timestamp"),
)''',
    ),
    ("src/bioxp/novo_usb_can.py", "BioXpNovoUsbDriver", "__init__"): (
        "self._pipette_error_callback = pipette_error_callback",
    ),
    ("src/bioxp/usb_driver.py", "BioXpTester", "_connect"): (
        "self.novo_router = NovoRouter(ep_in=self.ep_in, ep_out=self.ep_out, decode=novo_decode)",
        "self.novo_router.start()",
    ),
    ("src/bioxp/novo_router.py", "NovoRouter", "start"): (
        '''self._reader = threading.Thread(
    target=self._receive_loop,
    name=f"bioxp-novo-router-{self._reader_generation}",
    daemon=True,
)''',
    ),
    ("src/bioxp/novo_router.py", "NovoRouter", "_receive_loop"): (
        "self._dispatch(frame)",
    ),
    ("src/bioxp/novo_router.py", "NovoRouter", "_dispatch"): (
        "decision = pending.matcher(frame)",
    ),
    ("src/bioxp/novo_router.py", "NovoRouter", "transact_many"): (
        '''pending = _PendingTransaction(
    transaction_id,
    matcher_name,
    matcher,
    registered_at,
    self._reader_generation,
)''',
    ),
    ("src/bioxp/novo_router.py", "NovoRouter", "transact"): (
        '''pending = _PendingTransaction(
    transaction_id,
    matcher_name,
    matcher,
    registered_at,
    self._reader_generation,
)''',
    ),
}

OPERATOR_CATALOG_OWNER_POLICY: Sequence[tuple[str, str]] = (
    ("src/bioxp/api.py", "app"),
    ("src/bioxp/oem_compat/api.py", "router"),
    ("src/bioxp/oem_runtime_api.py", "router"),
    ("src/bioxp/oem_homing_routes.py", "router"),
)
MOUNTED_ROUTER_CALL_POLICY = frozenset(
    {
        "oem_compat_router",
        "oem_runtime_router",
        "oem_homing_router",
        "command_plane.router",
        "router",
        "create_operator_reports_router(_pipette_receipts, writer_health_provider=runtime_write_coordinator(_pipette_receipts.root).snapshot)",
    }
)
OPERATOR_CATALOG_EXCLUSION_POLICY = frozenset({
    "/", "/openapi.json", "/docs", "/docs/oauth2-redirect", "/redoc",
    "/oem/startup/status/{session_id}",
    "/motion/axis/relative", "/motion/axis/absolute", "/motion/axis/home", "/motion/axis/zero",
    "/motion/oem/z/live_right_reference",
})

OPERATOR_CATALOG_ROUTE_POLICY = frozenset({
    ('GET', '/camera/frame/latest', 'camera_frame_latest_camera_frame_latest_get'),
    ('GET', '/camera/status', 'camera_status_camera_status_get'),
    ('GET', '/chiller/snapshot', 'chiller_snapshot_chiller_snapshot_get'),
    ('GET', '/diagnostics/usb-sniff/runs', 'diagnostics_usb_sniff_runs_diagnostics_usb_sniff_runs_get'),
    ('GET', '/diagnostics/usb-sniff/runs/{run_id}/files', 'diagnostics_usb_sniff_files_diagnostics_usb_sniff_runs__run_id__files_get'),
    ('GET', '/diagnostics/usb-sniff/runs/{run_id}/tail', 'diagnostics_usb_sniff_tail_diagnostics_usb_sniff_runs__run_id__tail_get'),
    ('GET', '/diagnostics/usb-sniff/status', 'diagnostics_usb_sniff_status_diagnostics_usb_sniff_status_get'),
    ('GET', '/latch/status', 'latch_status_latch_status_get'),
    ('GET', '/liquid/application/status', 'liquid_application_status_liquid_application_status_get'),

    ('GET', '/liquid/status', 'liquid_status_liquid_status_get'),

    ('GET', '/maintenance/usb/state', 'maintenance_usb_state_maintenance_usb_state_get'),
    ('GET', '/motion/axes/status', 'axes_status_motion_axes_status_get'),
    ('GET', '/motion/axis/{axis}/status', 'axis_status_motion_axis__axis__status_get'),
    ('GET', '/motion/diagnostics/catalog', 'motion_diagnostics_catalog_motion_diagnostics_catalog_get'),
    ('GET', '/motion/diagnostics/status', 'motion_diagnostics_status_motion_diagnostics_status_get'),
    ('GET', '/motion/gripper/status', 'motion_gripper_status_motion_gripper_status_get'),
    ('GET', '/motion/interlock/override', 'motion_interlock_override_status_motion_interlock_override_get'),
    ('GET', '/motion/oem/initialization/provider-status', 'motion_oem_initialization_provider_status_motion_oem_initialization_provider_status_get'),
    ('GET', '/motion/oem/machine_config', 'get_oem_machine_config_motion_oem_machine_config_get'),
    ('GET', '/motion/oem/movement_readiness/comparison', 'get_oem_movement_readiness_comparison_motion_oem_movement_readiness_comparison_get'),
    ('GET', '/motion/oem/pathing/default_parameters', 'get_oem_pathing_default_parameters_motion_oem_pathing_default_parameters_get'),
    ('GET', '/motion/oem/pathing/scriptmove_plan', 'plan_oem_scriptmove_path_motion_oem_pathing_scriptmove_plan_get'),
    ('GET', '/motion/oem/position_table', 'get_oem_position_table_motion_oem_position_table_get'),
    ('GET', '/motion/oem/position_table/plan', 'plan_oem_position_table_move_motion_oem_position_table_plan_get'),
    ('GET', '/motion/oem/programs', 'list_oem_homing_programs_motion_oem_programs_get'),
    ('GET', '/motion/oem/programs/{program_name}', 'get_oem_homing_program_motion_oem_programs__program_name__get'),
    ('GET', '/motion/oem/shadow_readback', 'oem_shadow_readback_motion_oem_shadow_readback_get'),
    ('GET', '/motion/oem/x/status', 'motion_oem_x_status_motion_oem_x_status_get'),
    ('GET', '/motion/oem/y/status', 'motion_oem_y_status_motion_oem_y_status_get'),
    ('GET', '/motion/oem/z/status', 'motion_oem_z_status_motion_oem_z_status_get'),
    ('GET', '/motion/power/status', 'motion_power_status_motion_power_status_get'),
    ('GET', '/motion/range/status', 'motion_range_status_motion_range_status_get'),
    ('GET', '/motion/reference/status', 'motion_reference_status_motion_reference_status_get'),
    ('GET', '/oem-compat/capabilities/test-prep', 'capability_matrix_test_prep_oem_compat_capabilities_test_prep_get'),
    ('GET', '/oem/runtime/commands/history', 'runtime_commands_history_oem_runtime_commands_history_get'),
    ('GET', '/oem/runtime/commands/{command_id}', 'runtime_command_result_oem_runtime_commands__command_id__get'),
    ('GET', '/oem/runtime/events/latest', 'runtime_events_latest_oem_runtime_events_latest_get'),
    ('GET', '/oem/runtime/movement-runs/contract', 'full_lifecycle_contract_oem_runtime_movement_runs_contract_get'),
    ('GET', '/oem/runtime/movement-runs/{run_id}', 'get_full_lifecycle_run_oem_runtime_movement_runs__run_id__get'),
    ('GET', '/oem/runtime/movement-runs/{run_id}/ledger', 'get_full_lifecycle_ledger_oem_runtime_movement_runs__run_id__ledger_get'),
    ('GET', '/oem/runtime/state', 'runtime_state_oem_runtime_state_get'),
    ('GET', '/oem/runtime/status', 'runtime_status_oem_runtime_status_get'),
    ('GET', '/oem/runtime/worker/status', 'runtime_worker_status_oem_runtime_worker_status_get'),
    ('GET', '/oem/startup/status/latest', 'oem_startup_status_latest_oem_startup_status_latest_get'),
    ('GET', '/protocol/jobs', 'protocol_jobs_protocol_jobs_get'),
    ('GET', '/protocol/jobs/{job_id}', 'protocol_job_detail_protocol_jobs__job_id__get'),
    ('GET', '/status', 'get_status_status_get'),
    ('GET', '/thermal/snapshot', 'thermal_snapshot_thermal_snapshot_get'),
    ('POST', '/camera/snapshot', 'camera_snapshot_camera_snapshot_post'),
    ('POST', '/chiller/baseline', 'chiller_baseline_chiller_baseline_post'),
    ('POST', '/chiller/fan', 'set_chiller_fan_chiller_fan_post'),
    ('POST', '/chiller/hard_reset', 'chiller_hard_reset_chiller_hard_reset_post'),
    ('POST', '/chiller/pwm', 'set_chiller_pwm_chiller_pwm_post'),
    ('POST', '/chiller/rates', 'set_chiller_rates_chiller_rates_post'),
    ('POST', '/chiller/set_temp', 'set_chiller_temp_chiller_set_temp_post'),
    ('POST', '/diagnostics/usb-sniff/export', 'diagnostics_usb_sniff_export_diagnostics_usb_sniff_export_post'),
    ('POST', '/diagnostics/usb-sniff/start', 'diagnostics_usb_sniff_start_diagnostics_usb_sniff_start_post'),
    ('POST', '/diagnostics/usb-sniff/stop', 'diagnostics_usb_sniff_stop_diagnostics_usb_sniff_stop_post'),
    ('POST', '/hardware/snapshot/collect', 'hardware_snapshot_collect_hardware_snapshot_collect_post'),
    ('POST', '/latch/lock', 'latch_lock_latch_lock_post'),
    ('POST', '/latch/unlock', 'latch_unlock_latch_unlock_post'),
    ('POST', '/led/off', 'led_off_led_off_post'),
    ('POST', '/led/on', 'led_on_led_on_post'),
    ('POST', '/led/pct', 'led_pct_led_pct_post'),
    ('POST', '/led/rgb', 'led_rgb_led_rgb_post'),
    ('POST', '/liquid/application/plan', 'liquid_application_plan_liquid_application_plan_post'),
    ('POST', '/liquid/aspirate', 'liquid_aspirate_liquid_aspirate_post'),
    ('POST', '/liquid/aspirate-air', 'liquid_aspirate_air_liquid_aspirate_air_post'),
    ('POST', '/liquid/diagnoses', 'liquid_diagnoses_liquid_diagnoses_post'),
    ('POST', '/liquid/condition', 'liquid_condition_liquid_condition_post'),
    ('POST', '/liquid/data', 'liquid_data_liquid_data_post'),
    ('POST', '/liquid/dispense', 'liquid_dispense_liquid_dispense_post'),
    ('POST', '/liquid/dispense-air', 'liquid_dispense_air_liquid_dispense_air_post'),
    ('POST', '/liquid/dispense-all', 'liquid_dispense_all_liquid_dispense_all_post'),
    ('POST', '/liquid/eject-all', 'liquid_eject_all_liquid_eject_all_post'),
    ('POST', '/liquid/error-log', 'liquid_error_log_liquid_error_log_post'),
    ('POST', '/liquid/firmware', 'liquid_firmware_liquid_firmware_post'),
    ('POST', '/liquid/fluid-detection', 'liquid_fluid_detection_liquid_fluid_detection_post'),
    ('POST', '/liquid/fluid-detection/{channel}/timestamp', 'liquid_fluid_timestamp_liquid_fluid_detection__channel__timestamp_post'),
    ('POST', '/liquid/heartbeat', 'liquid_heartbeat_liquid_heartbeat_post'),
    ('POST', '/liquid/init', 'liquid_init_liquid_init_post'),
    ('POST', '/liquid/keep-tip', 'liquid_keep_tip_liquid_keep_tip_post'),
    ('POST', '/liquid/mix', 'liquid_mix_liquid_mix_post'),
    ('POST', '/liquid/mix-all', 'liquid_mix_all_liquid_mix_all_post'),
    ('POST', '/liquid/readback', 'liquid_readback_liquid_readback_post'),
    ('POST', '/liquid/pressure', 'liquid_pressure_liquid_pressure_post'),
    ('POST', '/liquid/reinitialize', 'liquid_reinitialize_liquid_reinitialize_post'),
    ('POST', '/liquid/set-top-speed', 'liquid_set_top_speed_liquid_set_top_speed_post'),
    ('POST', '/liquid/status/readback', 'liquid_status_readback_liquid_status_readback_post'),
    ('POST', '/liquid/terminate', 'liquid_terminate_liquid_terminate_post'),
    ('POST', '/liquid/tip', 'liquid_tip_liquid_tip_post'),
    ('POST', '/liquid/tip-status', 'liquid_tip_status_liquid_tip_status_post'),
    ('POST', '/maintenance/usb/reconnect', 'maintenance_usb_reconnect_maintenance_usb_reconnect_post'),
    ('POST', '/maintenance/usb/recover_motion', 'maintenance_usb_recover_motion_maintenance_usb_recover_motion_post'),
    ('POST', '/maintenance/usb/release', 'maintenance_usb_release_maintenance_usb_release_post'),
    ('POST', '/motion/arm/strict_startup', 'motion_arm_strict_startup_motion_arm_strict_startup_post'),
    ('POST', '/motion/axes/current', 'motion_axes_current_motion_axes_current_post'),
    ('POST', '/motion/clear_lock', 'clear_lock_motion_clear_lock_post'),
    ('POST', '/motion/diagnostics/execute', 'motion_diagnostics_execute_motion_diagnostics_execute_post'),
    ('POST', '/motion/diagnostics/stop', 'motion_diagnostics_stop_motion_diagnostics_stop_post'),
    ('POST', '/motion/emergency_stop', 'motion_emergency_stop_motion_emergency_stop_post'),
    ('POST', '/motion/gripper/clear', 'motion_gripper_clear_motion_gripper_clear_post'),
    ('POST', '/motion/gripper/close', 'motion_gripper_close_motion_gripper_close_post'),
    ('POST', '/motion/gripper/home', 'motion_gripper_home_motion_gripper_home_post'),
    ('POST', '/motion/gripper/open', 'motion_gripper_open_motion_gripper_open_post'),
    ('POST', '/motion/gripper/open_wide', 'motion_gripper_open_wide_motion_gripper_open_wide_post'),
    ('POST', '/motion/gripper/restore_idle_current', 'motion_gripper_restore_idle_current_motion_gripper_restore_idle_current_post'),
    ('POST', '/motion/hard_reset', 'motion_hard_reset_motion_hard_reset_post'),
    ('POST', '/motion/interlock/override', 'motion_interlock_override_set_motion_interlock_override_post'),
    ('POST', '/motion/interlock/prepare', 'prepare_interlock_motion_interlock_prepare_post'),
    ('POST', '/motion/oem/home_gz', 'execute_oem_home_gz_motion_oem_home_gz_post'),
    ('POST', '/motion/oem/home_xy', 'motion_oem_home_xy_motion_oem_home_xy_post'),
    ('POST', '/motion/oem/initialization/initialize_motion', 'motion_oem_serial206_initialize_motion_motion_oem_initialization_initialize_motion_post'),
    ('POST', '/motion/oem/initialization/initialize_motors', 'motion_oem_serial206_initialize_motors_motion_oem_initialization_initialize_motors_post'),
    ('POST', '/motion/oem/initialization/observation', 'motion_oem_initialization_observation_motion_oem_initialization_observation_post'),
    ('POST', '/motion/oem/manual/absolute', 'motion_oem_manual_absolute_motion_oem_manual_absolute_post'),
    ('POST', '/motion/oem/manual/home', 'motion_oem_manual_home_motion_oem_manual_home_post'),
    ('POST', '/motion/oem/manual/relative', 'motion_oem_manual_relative_motion_oem_manual_relative_post'),
    ('POST', '/motion/oem/manual/sethome', 'motion_oem_manual_sethome_motion_oem_manual_sethome_post'),
    ('POST', '/motion/oem/move_to', 'motion_oem_move_to_motion_oem_move_to_post'),
    ('POST', '/motion/oem/move_xy', 'motion_oem_move_xy_motion_oem_move_xy_post'),
    ('POST', '/motion/oem/pathing/scriptmove_execute', 'execute_oem_scriptmove_path_motion_oem_pathing_scriptmove_execute_post'),
    ('POST', '/motion/oem/prepare_without_motion', 'motion_oem_prepare_without_motion_motion_oem_prepare_without_motion_post'),
    ('POST', '/motion/oem/shadow_readback/capture', 'oem_shadow_readback_capture_motion_oem_shadow_readback_capture_post'),
    ('POST', '/motion/oem/x/abort', 'motion_oem_x_abort_motion_oem_x_abort_post'),
    ('POST', '/motion/oem/x/caught_plate_recovery_home', 'motion_oem_x_caught_plate_recovery_home_motion_oem_x_caught_plate_recovery_home_post'),
    ('POST', '/motion/oem/x/diagnostic_home_axis', 'motion_oem_x_diagnostic_home_axis_motion_oem_x_diagnostic_home_axis_post'),
    ('POST', '/motion/oem/x/internal/enable_xy', 'motion_oem_x_internal_enable_xy_motion_oem_x_internal_enable_xy_post'),
    ('POST', '/motion/oem/x/internal/enable_xyz', 'motion_oem_x_internal_enable_xyz_motion_oem_x_internal_enable_xyz_post'),
    ('POST', '/motion/oem/x/manual_home', 'motion_oem_x_manual_home_motion_oem_x_manual_home_post'),
    ('POST', '/motion/oem/x/move_absolute', 'motion_oem_x_move_absolute_motion_oem_x_move_absolute_post'),
    ('POST', '/motion/oem/x/move_steps', 'motion_oem_x_move_steps_motion_oem_x_move_steps_post'),
    ('POST', '/motion/oem/x/move_to_origin_home', 'motion_oem_x_move_to_origin_home_motion_oem_x_move_to_origin_home_post'),
    ('POST', '/motion/oem/x/observation', 'motion_oem_x_observation_motion_oem_x_observation_post'),
    ('POST', '/motion/oem/x/prepare', 'motion_oem_x_prepare_motion_oem_x_prepare_post'),
    ('POST', '/motion/oem/x/reconcile_switch_masks', 'motion_oem_x_reconcile_switch_masks_motion_oem_x_reconcile_switch_masks_post'),
    ('POST', '/motion/oem/x/restore_original_speed', 'motion_oem_x_restore_original_speed_motion_oem_x_restore_original_speed_post'),
    ('POST', '/motion/oem/x/set_home', 'motion_oem_x_set_home_motion_oem_x_set_home_post'),
    ('POST', '/motion/oem/x/set_max_acc', 'motion_oem_x_set_max_acc_motion_oem_x_set_max_acc_post'),
    ('POST', '/motion/oem/x/set_max_speed', 'motion_oem_x_set_max_speed_motion_oem_x_set_max_speed_post'),
    ('POST', '/motion/oem/x/set_stall_guard', 'motion_oem_x_set_stall_guard_motion_oem_x_set_stall_guard_post'),
    ('POST', '/motion/oem/x/startup_home', 'motion_oem_x_startup_home_motion_oem_x_startup_home_post'),
    ('POST', '/motion/oem/x/stop', 'motion_oem_x_stop_motion_oem_x_stop_post'),
    ('POST', '/motion/oem/y/home', 'motion_oem_y_home_motion_oem_y_home_post'),
    ('POST', '/motion/oem/y/internal/acceleration_overload', 'motion_oem_y_acceleration_overload_motion_oem_y_internal_acceleration_overload_post'),
    ('POST', '/motion/oem/y/internal/board_test_my', 'motion_oem_y_board_test_my_motion_oem_y_internal_board_test_my_post'),
    ('POST', '/motion/oem/y/move_absolute', 'motion_oem_y_move_absolute_motion_oem_y_move_absolute_post'),
    ('POST', '/motion/oem/y/move_steps', 'motion_oem_y_move_steps_motion_oem_y_move_steps_post'),
    ('POST', '/motion/oem/y/prepare', 'motion_oem_y_prepare_motion_oem_y_prepare_post'),
    ('POST', '/motion/oem/y/set_home', 'motion_oem_y_set_home_motion_oem_y_set_home_post'),
    ('POST', '/motion/oem/y/stop', 'motion_oem_y_stop_motion_oem_y_stop_post'),
    ('POST', '/motion/oem/z/abort', 'motion_oem_z_abort_motion_oem_z_abort_post'),
    ('POST', '/motion/oem/z/clear', 'motion_oem_z_clear_motion_oem_z_clear_post'),
    ('POST', '/motion/oem/z/control', 'motion_oem_z_control_motion_oem_z_control_post'),
    ('POST', '/motion/oem/z/diagnostic_home_axis', 'motion_oem_z_diagnostic_home_axis_motion_oem_z_diagnostic_home_axis_post'),
    ('POST', '/motion/oem/z/lift_pipette', 'motion_oem_z_lift_pipette_motion_oem_z_lift_pipette_post'),
    ('POST', '/motion/oem/z/lower_pipette', 'motion_oem_z_lower_pipette_motion_oem_z_lower_pipette_post'),
    ('POST', '/motion/oem/z/move_gz', 'motion_oem_z_move_gz_motion_oem_z_move_gz_post'),
    ('POST', '/motion/oem/z/move_z_home', 'motion_oem_move_z_home_motion_oem_z_move_z_home_post'),
    ('POST', '/motion/oem/z/observation', 'motion_oem_z_observation_motion_oem_z_observation_post'),
    ('POST', '/motion/oem/z/path_clean_mode', 'motion_oem_z_path_clean_mode_motion_oem_z_path_clean_mode_post'),
    ('POST', '/motion/oem/z/prepare', 'motion_oem_z_prepare_motion_oem_z_prepare_post'),
    ('POST', '/motion/oem/z/reconcile_switch_masks', 'motion_oem_z_reconcile_switch_masks_motion_oem_z_reconcile_switch_masks_post'),
    ('POST', '/motion/oem/z/resume_after_abort', 'motion_oem_z_resume_after_abort_motion_oem_z_resume_after_abort_post'),
    ('POST', '/motion/oem/z/self_test', 'motion_oem_z_self_test_motion_oem_z_self_test_post'),
    ('POST', '/motion/oem/z/set_home', 'motion_oem_z_set_home_motion_oem_z_set_home_post'),
    ('POST', '/motion/oem/z/stop', 'motion_oem_z_stop_motion_oem_z_stop_post'),
    ('POST', '/motion/oem/{program_name}/dry_run', 'dry_run_oem_homing_program_motion_oem__program_name__dry_run_post'),
    ('POST', '/motion/power/diag', 'motion_power_diag_motion_power_diag_post'),
    ('POST', '/motion/power/enable', 'motion_power_enable_motion_power_enable_post'),
    ('POST', '/motion/reference/mark_desynced', 'motion_reference_mark_desynced_motion_reference_mark_desynced_post'),
    ('POST', '/motion/reference/mark_referenced', 'motion_reference_mark_referenced_motion_reference_mark_referenced_post'),
    ('POST', '/motion/thermal_door/close', 'motion_thermal_door_close_motion_thermal_door_close_post'),
    ('POST', '/motion/thermal_door/home', 'motion_thermal_door_home_motion_thermal_door_home_post'),
    ('POST', '/motion/thermal_door/open', 'motion_thermal_door_open_motion_thermal_door_open_post'),
    ('POST', '/oem-compat/protocols/import/dry-run', 'protocol_import_dry_run_oem_compat_protocols_import_dry_run_post'),
    ('POST', '/oem-compat/scripts/translate/dry-run', 'script_translate_dry_run_oem_compat_scripts_translate_dry_run_post'),
    ('POST', '/oem-compat/startup/dry-run', 'startup_dry_run_oem_compat_startup_dry_run_post'),
    ('POST', '/oem/initial_check', 'oem_initial_check_oem_initial_check_post'),
    ('POST', '/oem/runtime/commands/PrepareToRunJob', 'runtime_command_prepare_to_run_job_oem_runtime_commands_PrepareToRunJob_post'),
    ('POST', '/oem/runtime/commands/abortjob', 'runtime_command_abortjob_oem_runtime_commands_abortjob_post'),
    ('POST', '/oem/runtime/commands/enqueue', 'runtime_commands_enqueue_oem_runtime_commands_enqueue_post'),
    ('POST', '/oem/runtime/commands/unlockProcess', 'runtime_command_unlock_process_oem_runtime_commands_unlockProcess_post'),
    ('POST', '/oem/runtime/commands/validateJob', 'runtime_command_validate_job_oem_runtime_commands_validateJob_post'),
    ('POST', '/oem/runtime/commands/wakefrompause', 'runtime_command_wakefrompause_oem_runtime_commands_wakefrompause_post'),
    ('POST', '/oem/runtime/events/door', 'runtime_event_door_oem_runtime_events_door_post'),
    ('POST', '/oem/runtime/events/pause', 'runtime_event_pause_oem_runtime_events_pause_post'),
    ('POST', '/oem/runtime/events/resume', 'runtime_event_resume_oem_runtime_events_resume_post'),
    ('POST', '/oem/runtime/movement-runs', 'create_full_lifecycle_run_oem_runtime_movement_runs_post'),
    ('POST', '/oem/runtime/movement-runs/{run_id}/cancel', 'cancel_full_lifecycle_run_oem_runtime_movement_runs__run_id__cancel_post'),
    ('POST', '/oem/runtime/readiness/prepare-to-run-job/dry-run', 'runtime_prepare_to_run_job_readiness_dry_run_oem_runtime_readiness_prepare_to_run_job_dry_run_post'),
    ('POST', '/oem/runtime/recover', 'runtime_recover_oem_runtime_recover_post'),
    ('POST', '/oem/startup/constructor_pipettes', 'oem_startup_constructor_pipettes_oem_startup_constructor_pipettes_post'),
    ('POST', '/oem/startup/door_event', 'oem_startup_door_event_oem_startup_door_event_post'),
    ('POST', '/oem/startup/initialize_environment', 'oem_startup_initialize_environment_oem_startup_initialize_environment_post'),
    ('POST', '/oem/startup/initialize_without_motion', 'oem_startup_initialize_without_motion_oem_startup_initialize_without_motion_post'),
    ('POST', '/oem/startup/request', 'oem_startup_request_oem_startup_request_post'),
    ('POST', '/oem/switch_audit', 'oem_switch_audit_oem_switch_audit_post'),
    ('POST', '/protocol/compile', 'protocol_compile_protocol_compile_post'),
    ('POST', '/protocol/execute', 'protocol_execute_protocol_execute_post'),
    ('POST', '/protocol/jobs/{job_id}/review', 'protocol_job_review_protocol_jobs__job_id__review_post'),
    ('POST', '/reconnect', 'reconnect_runtime_reconnect_post'),
    ('POST', '/thermal/baseline', 'thermal_baseline_thermal_baseline_post'),
    ('POST', '/thermal/fan', 'set_thermal_fan_thermal_fan_post'),
    ('POST', '/thermal/fast_profile', 'thermal_fast_profile_thermal_fast_profile_post'),
    ('POST', '/thermal/hard_reset', 'thermal_hard_reset_thermal_hard_reset_post'),
    ('POST', '/thermal/pwm', 'set_thermal_pwm_thermal_pwm_post'),
    ('POST', '/thermal/rates', 'set_thermal_rates_thermal_rates_post'),
    ('POST', '/thermal/set_temp', 'set_thermal_temp_thermal_set_temp_post'),
    ('POST', '/vision/barcode/read', 'vision_barcode_read_vision_barcode_read_post'),
    ('POST', '/vision/inspect', 'vision_inspect_vision_inspect_post'),
})

BMS_RELAY_POLICY: Mapping[
    str, tuple[str, str, float, str, str, str, bool, bool]
] = {
    "status": ("GET", "/status", 5.0, "BioXpRobotClient.request:status", "read_only_relay", "GET /status", False, False),
    "activate_usb_for_service": ("POST", "/reconnect", 30.0, "BioXpRobotClient.request:activate_usb_for_service", "mutation_relay", "POST /reconnect", False, False),
    "collect_hardware_snapshot": ("POST", "/hardware/snapshot/collect", 210.0, "BioXpRobotClient.request:collect_hardware_snapshot", "mutation_relay", "POST /hardware/snapshot/collect", False, False),
    "oem_full_lifecycle_contract": ("GET", "/oem/runtime/movement-runs/contract", 10.0, "BioXpRobotClient.request:oem_full_lifecycle_contract", "read_only_relay", "GET /oem/runtime/movement-runs/contract", False, False),
    "plan_oem_full_lifecycle": ("POST", "/oem/runtime/movement-runs", 30.0, "BioXpRobotClient.request:plan_oem_full_lifecycle", "mutation_relay", "POST /oem/runtime/movement-runs", False, False),
    "get_oem_full_lifecycle_run": ("GET", "/oem/runtime/movement-runs/{run_id}", 10.0, "BioXpRobotClient.request:get_oem_full_lifecycle_run", "read_only_relay", "GET /oem/runtime/movement-runs/{run_id}", False, False),
    "get_oem_full_lifecycle_ledger": ("GET", "/oem/runtime/movement-runs/{run_id}/ledger", 10.0, "BioXpRobotClient.request:get_oem_full_lifecycle_ledger", "read_only_relay", "GET /oem/runtime/movement-runs/{run_id}/ledger", False, False),
    "cancel_oem_full_lifecycle_run": ("POST", "/oem/runtime/movement-runs/{run_id}/cancel", 15.0, "BioXpRobotClient.request:cancel_oem_full_lifecycle_run", "mutation_relay", "POST /oem/runtime/movement-runs/{run_id}/cancel", False, False),
    "collect_axis_diagnostics": ("GET", "/motion/diagnostics/status", 45.0, "BioXpRobotClient.request:collect_axis_diagnostics", "read_only_relay", "GET /motion/diagnostics/status", False, False),
    "run_axis_diagnostic": ("POST", "/motion/diagnostics/execute", 180.0, "BioXpRobotClient.request:run_axis_diagnostic", "mutation_relay", "POST /motion/diagnostics/execute", False, False),
    "stop_axis_diagnostic": ("POST", "/motion/diagnostics/stop", 25.0, "BioXpRobotClient.request:stop_axis_diagnostic", "mutation_relay", "POST /motion/diagnostics/stop", False, False),
    "recover_motion_non_homing": ("POST", "/motion/arm/strict_startup", 90.0, "BioXpRobotClient.request:recover_motion_non_homing", "mutation_relay", "POST /motion/arm/strict_startup", False, False),
    "emergency_stop": ("POST", "/oem/runtime/emergency_stop", 5.0, "BioXpRobotClient.request:emergency_stop", "mutation_relay", "POST /oem/runtime/emergency_stop", False, False),
    "camera_status": ("GET", "/camera/status", 5.0, "BioXpRobotClient.request:camera_status", "read_only_relay", "GET /camera/status", False, False),
    "camera_latest": ("GET", "/camera/frame/latest", 5.0, "BioXpRobotClient.request:camera_latest", "read_only_relay", "GET /camera/frame/latest", False, False),
    "camera_snapshot": ("POST", "/camera/snapshot", 15.0, "BioXpRobotClient.request:camera_snapshot", "mutation_relay", "POST /camera/snapshot", False, False),
    "camera_stream_start": ("POST", "/camera/stream/start", 15.0, "BioXpRobotClient.request:camera_stream_start", "mutation_relay", "POST /camera/stream/start", False, False),
    "camera_stream_state": ("GET", "/camera/stream/state", 5.0, "BioXpRobotClient.request:camera_stream_state", "read_only_relay", "GET /camera/stream/state", False, False),
    "camera_mjpeg": ("GET", "/camera/mjpeg", 10.0, "BioXpRobotClient.request:camera_mjpeg", "read_only_relay", "GET /camera/mjpeg", False, False),
    "camera_stream_stop": ("POST", "/camera/stream/stop", 15.0, "BioXpRobotClient.request:camera_stream_stop", "mutation_relay", "POST /camera/stream/stop", False, False),
    "operator_control_catalog": ("GET", "/operator/control-catalog", 10.0, "BioXpRobotClient.request:operator_control_catalog", "read_only_relay", "GET /operator/control-catalog", False, False),
    "operator_control_catalog_v2": ("GET", "/operator/v2/control-catalog", 5.0, "BioXpRobotClient.request:operator_control_catalog_v2", "read_only_relay", "GET /operator/v2/control-catalog", False, False),
    "operator_dashboard": ("GET", "/operator/dashboard", 10.0, "BioXpRobotClient.request:operator_dashboard", "read_only_relay", "GET /operator/dashboard", False, False),
    "operator_dashboard_v2": ("GET", "/operator/v2/dashboard", 5.0, "BioXpRobotClient.request:operator_dashboard_v2", "read_only_relay", "GET /operator/v2/dashboard", False, False),
    "pipette_readback": ("POST", "/liquid/readback", 120.0, "BioXpRobotClient.request:pipette_readback", "mutation_relay", "POST /liquid/readback", True, False),
    "pipette_application_status": ("GET", "/liquid/application/status", 10.0, "BioXpRobotClient.request:pipette_application_status", "read_only_relay", "GET /liquid/application/status", False, False),
    "pipette_application_plan": ("POST", "/liquid/application/plan", 10.0, "BioXpRobotClient.request:pipette_application_plan", "mutation_relay", "POST /liquid/application/plan", True, False),
    "operator_action_admission": ("POST", "/operator/actions/{action_id}/admission", 10.0, "BioXpRobotClient.request:operator_action_admission", "mutation_relay", "POST /operator/actions/{action_id}/admission", False, False),
    "invoke_operator_action": ("POST", "/operator/actions/{action_id}", 900.0, "BioXpRobotClient.request:invoke_operator_action", "mutation_relay", "POST /operator/actions/{action_id}", False, False),
    "invoke_operator_action_v2": ("POST", "/operator/v2/actions/{action_id}", 5.0, "BioXpRobotClient.request:invoke_operator_action_v2", "mutation_relay", "POST /operator/v2/actions/{action_id}", False, False),
    "interrupt_operator_action_v1": ("POST", "/operator/v2/actions/{action_id}", 10.0, "BioXpRobotClient.request:interrupt_operator_action_v1", "mutation_relay", "POST /operator/v2/actions/{action_id}", False, False),
    "submit_operator_method_v1": ("POST", "/operator/v2/methods", 5.0, "BioXpRobotClient.request:submit_operator_method_v1", "mutation_relay", "POST /operator/v2/methods", False, False),
    "operator_method_status_v1": ("GET", "/operator/v2/methods/{method_id}", 5.0, "BioXpRobotClient.request:operator_method_status_v1", "read_only_relay", "GET /operator/v2/methods/{method_id}", False, False),
    "operator_command_status_v2": ("GET", "/operator/v2/commands/{command_id}", 5.0, "BioXpRobotClient.request:operator_command_status_v2", "read_only_relay", "GET /operator/v2/commands/{command_id}", False, False),
    "operator_action_history": ("GET", "/operator/actions/history", 10.0, "BioXpRobotClient.request:operator_action_history", "read_only_relay", "GET /operator/actions/history", False, False),
    "operator_action_history_v2": ("GET", "/operator/v2/actions/history", 5.0, "BioXpRobotClient.request:operator_action_history_v2", "read_only_relay", "GET /operator/v2/actions/history", False, False),
    "operator_action_receipt": ("GET", "/operator/actions/receipts/{command_id}", 10.0, "BioXpRobotClient.request:operator_action_receipt", "read_only_relay", "GET /operator/actions/receipts/{command_id}", False, False),
    "operator_action_receipt_v2": ("GET", "/operator/v2/actions/receipts/{command_id}", 5.0, "BioXpRobotClient.request:operator_action_receipt_v2", "read_only_relay", "GET /operator/v2/actions/receipts/{command_id}", False, False),
    "assess_operator_action": ("POST", "/operator/actions/receipts/{command_id}/assessment", 15.0, "BioXpRobotClient.request:assess_operator_action", "mutation_relay", "POST /operator/actions/receipts/{command_id}/assessment", False, False),
    "operator_report_summary": ("GET", "/operator/reports/summary", 10.0, "BioXpRobotClient.request:operator_report_summary", "read_only_relay", "GET /operator/reports/summary", False, False),
    "operator_report_commands": ("GET", "/operator/reports/commands", 10.0, "BioXpRobotClient.request:operator_report_commands", "read_only_relay", "GET /operator/reports/commands", False, False),
    "operator_report_command_detail": ("GET", "/operator/reports/commands/{command_id}", 10.0, "BioXpRobotClient.request:operator_report_command_detail", "read_only_relay", "GET /operator/reports/commands/{command_id}", False, False),
    "operator_report_command_transitions": ("GET", "/operator/reports/commands/{command_id}/transitions", 10.0, "BioXpRobotClient.request:operator_report_command_transitions", "read_only_relay", "GET /operator/reports/commands/{command_id}/transitions", False, False),
    "operator_report_command_evidence": ("GET", "/operator/reports/commands/{command_id}/evidence", 10.0, "BioXpRobotClient.request:operator_report_command_evidence", "read_only_relay", "GET /operator/reports/commands/{command_id}/evidence", False, False),
    "operator_report_pipette": ("GET", "/operator/reports/pipette", 10.0, "BioXpRobotClient.request:operator_report_pipette", "read_only_relay", "GET /operator/reports/pipette", False, False),
    "operator_report_pipette_detail": ("GET", "/operator/reports/pipette/{pipette_operation_id}", 10.0, "BioXpRobotClient.request:operator_report_pipette_detail", "read_only_relay", "GET /operator/reports/pipette/{pipette_operation_id}", False, False),
    "operator_report_pipette_channels": ("GET", "/operator/reports/pipette/{pipette_operation_id}/channels", 10.0, "BioXpRobotClient.request:operator_report_pipette_channels", "read_only_relay", "GET /operator/reports/pipette/{pipette_operation_id}/channels", False, False),
    "operator_report_pipette_exchanges": ("GET", "/operator/reports/pipette/{pipette_operation_id}/exchanges", 10.0, "BioXpRobotClient.request:operator_report_pipette_exchanges", "read_only_relay", "GET /operator/reports/pipette/{pipette_operation_id}/exchanges", False, False),
    "operator_report_events": ("GET", "/operator/reports/events", 10.0, "BioXpRobotClient.request:operator_report_events", "read_only_relay", "GET /operator/reports/events", False, False),
    "operator_report_event_detail": ("GET", "/operator/reports/events/{event_id}", 10.0, "BioXpRobotClient.request:operator_report_event_detail", "read_only_relay", "GET /operator/reports/events/{event_id}", False, False),
    "operator_report_pressure_streams": ("GET", "/operator/reports/pressure-streams", 10.0, "BioXpRobotClient.request:operator_report_pressure_streams", "read_only_relay", "GET /operator/reports/pressure-streams", False, False),
    "operator_report_pressure_detail": ("GET", "/operator/reports/pressure-streams/{stream_session_id}", 10.0, "BioXpRobotClient.request:operator_report_pressure_detail", "read_only_relay", "GET /operator/reports/pressure-streams/{stream_session_id}", False, False),
    "operator_report_pressure_samples": ("GET", "/operator/reports/pressure-streams/{stream_session_id}/samples", 10.0, "BioXpRobotClient.request:operator_report_pressure_samples", "read_only_relay", "GET /operator/reports/pressure-streams/{stream_session_id}/samples", False, False),
    "operator_report_audit_health": ("GET", "/operator/audit-health", 10.0, "BioXpRobotClient.request:operator_report_audit_health", "read_only_relay", "GET /operator/audit-health", False, False),
    "operator_report_export_create": ("POST", "/operator/reports/exports", 30.0, "BioXpRobotClient.request:operator_report_export_create", "mutation_relay", "POST /operator/reports/exports", False, False),
    "operator_report_export_list": ("GET", "/operator/reports/exports", 10.0, "BioXpRobotClient.request:operator_report_export_list", "read_only_relay", "GET /operator/reports/exports", False, False),
    "operator_report_export_detail": ("GET", "/operator/reports/exports/{export_id}", 10.0, "BioXpRobotClient.request:operator_report_export_detail", "read_only_relay", "GET /operator/reports/exports/{export_id}", False, False),
    "operator_report_export_download": ("GET", "/operator/reports/exports/{export_id}/download", 30.0, "BioXpRobotClient.request:operator_report_export_download", "read_only_relay", "GET /operator/reports/exports/{export_id}/download", False, False),
}

FRONTEND_DEFINITION_POLICY: Mapping[
    str, tuple[tuple[str, ...], str, str, str, bool, bool]
] = {
    "useBioXpStatus": (("/api/bioxp/status",), "useBioXpStatus", "frontend_query", "query", False, False),
    "useBioXpOperatorControlCatalog": (("/api/bioxp/operator-controls/catalog",), "useBioXpOperatorControlCatalog", "frontend_query", "query", False, False),
    "useBioXpOperatorDashboard": (("/api/bioxp/operator-controls/dashboard",), "useBioXpOperatorDashboard", "frontend_query", "query", False, False),
    "useBioXpOperatorDashboardV2": (("/api/bioxp/operator-controls/v2/dashboard",), "useBioXpOperatorDashboardV2", "frontend_query", "query", False, False),
    "useBioXpOperatorControlCatalogV2": (("/api/bioxp/operator-controls/v2/catalog",), "useBioXpOperatorControlCatalogV2", "frontend_query", "query", False, False),
    "useInvokeBioXpOperatorActionV2": (("/api/bioxp/operator-controls/v2/actions/{actionId}",), "useInvokeBioXpOperatorActionV2", "frontend_action", "action", False, False),
    "useInterruptBioXpOperatorActionV1": (("/api/bioxp/operator-controls/v2/interrupts/{actionId}",), "useInterruptBioXpOperatorActionV1", "frontend_action", "action", False, False),
    "useBioXpOperatorReceiptV2": (("/api/bioxp/operator-controls/v2/receipts/{commandId}",), "useBioXpOperatorReceiptV2", "frontend_query", "query", False, False),
    "useSubmitBioXpOperatorMethodV1": (("/api/bioxp/operator-controls/v2/methods",), "useSubmitBioXpOperatorMethodV1", "frontend_action", "action", False, False),
    "useBioXpOperatorMethodV1": (("/api/bioxp/operator-controls/v2/methods/{methodId}",), "useBioXpOperatorMethodV1", "frontend_query", "query", False, False),
    "useBioXpOperatorCommandV2": (("/api/bioxp/operator-controls/v2/commands/{commandId}",), "useBioXpOperatorCommandV2", "frontend_query", "query", False, False),
    "useReadBioXpPipetteReadback": (("/api/bioxp/operator-controls/pipettes/readback",), "useReadBioXpPipetteReadback", "frontend_action", "action", False, False),
    "useBioXpPipetteApplicationStatus": (("/api/bioxp/operator-controls/pipettes/application/status",), "useBioXpPipetteApplicationStatus", "frontend_query", "query", False, False),
    "usePlanBioXpPipetteApplication": (("/api/bioxp/operator-controls/pipettes/application/plan",), "usePlanBioXpPipetteApplication", "frontend_action", "action", False, False),
    "useBioXpOperatorActionAdmission": (("/api/bioxp/operator-controls/actions/{actionId}/admission",), "useBioXpOperatorActionAdmission", "frontend_query", "query", False, False),
    "useBioXpOperatorActionHistory": (("/api/bioxp/operator-controls/history?limit={limit}",), "useBioXpOperatorActionHistory", "frontend_query", "query", False, False),
    "useBioXpOperatorReportSummary": (("/api/bioxp/operator-controls/reports/summary",), "useBioXpOperatorReportSummary", "frontend_query", "query", False, False),
    "useBioXpOperatorReportCommands": (("/api/bioxp/operator-controls/reports/commands",), "useBioXpOperatorReportCommands", "frontend_query", "query", False, False),
    "useBioXpOperatorReportCommandDetail": (("/api/bioxp/operator-controls/reports/commands/{commandId}",), "useBioXpOperatorReportCommandDetail", "frontend_query", "query", False, False),
    "useBioXpOperatorReportPipette": (("/api/bioxp/operator-controls/reports/pipette",), "useBioXpOperatorReportPipette", "frontend_query", "query", False, False),
    "useBioXpOperatorReportEvents": (("/api/bioxp/operator-controls/reports/events",), "useBioXpOperatorReportEvents", "frontend_query", "query", False, False),
    "useBioXpOperatorReportPressureStreams": (("/api/bioxp/operator-controls/reports/pressure-streams",), "useBioXpOperatorReportPressureStreams", "frontend_query", "query", False, False),
    "useCreateBioXpOperatorReportExport": (("/api/bioxp/operator-controls/reports/exports",), "useCreateBioXpOperatorReportExport", "frontend_action", "action", False, False),
    "useBioXpOperatorReportExports": (("/api/bioxp/operator-controls/reports/exports",), "useBioXpOperatorReportExports", "frontend_query", "query", False, False),
    "useBioXpCameraStatus": (("/api/bioxp/camera/status",), "useBioXpCameraStatus", "frontend_query", "query", False, False),
    "useBioXpCameraStreamState": (("/api/bioxp/camera/stream/state",), "useBioXpCameraStreamState", "frontend_query", "query", False, False),
    "startBioXpCameraStream": (("/api/bioxp/camera/stream/start",), "startBioXpCameraStream", "frontend_action", "direct_action", False, False),
    "stopBioXpCameraStream": (("/api/bioxp/camera/stream/stop",), "stopBioXpCameraStream", "frontend_action", "direct_action", False, False),
    "buildBioXpCameraMjpegUrl": (("/api/bioxp/camera/mjpeg",), "buildBioXpCameraMjpegUrl", "frontend_query", "url_builder", False, False),
    "fetchBioXpCameraFrame": (("/api/bioxp/camera/frame/latest",), "fetchBioXpCameraFrame", "frontend_query", "direct_query", False, False),
    "captureBioXpCameraSnapshot": (("/api/bioxp/camera/snapshot",), "captureBioXpCameraSnapshot", "frontend_action", "direct_action", False, False),
    "useBioXpOemFullLifecycleContract": (("/api/bioxp/oem-full-lifecycle/contract",), "useBioXpOemFullLifecycleContract", "frontend_query", "query", False, False),
    "useBioXpOemFullLifecycleRun": (("/api/bioxp/oem-full-lifecycle/runs/{runId}/ledger",), "useBioXpOemFullLifecycleRun", "frontend_query", "query", False, False),
    "useBioXpProfile": (("/api/bioxp/profile",), "useBioXpProfile", "frontend_query", "query", False, False),
    "useBioXpJobs": (("/api/bioxp/jobs",), "useBioXpJobs", "frontend_query", "query", False, False),
    "useSaveBioXpProfile": (("/api/bioxp/profile",), "useSaveBioXpProfile", "frontend_action", "action", False, False),
    "useForgetBioXpProfile": (("/api/bioxp/profile",), "useForgetBioXpProfile", "frontend_action", "action", False, False),
    "useConnectBioXp": (("/api/bioxp/connection/connect",), "useConnectBioXp", "frontend_action", "action", False, False),
    "useDisconnectBioXp": (("/api/bioxp/connection/disconnect",), "useDisconnectBioXp", "frontend_action", "action", False, False),
    "useProbeBioXp": (("/api/bioxp/connection/probe",), "useProbeBioXp", "frontend_action", "action", False, False),
    "useUpdateBioXpFreshness": (("/api/bioxp/settings/freshness",), "useUpdateBioXpFreshness", "frontend_action", "action", False, False),
    "useRecoverBioXpMotion": (("/api/bioxp/connection/recover-motion-non-homing",), "useRecoverBioXpMotion", "frontend_action", "action", False, False),
    "useCompileBioXpProtocol": (("/api/bioxp/protocols/compile",), "useCompileBioXpProtocol", "frontend_action", "action", False, False),
    "useSubmitBioXpProtocol": (("/api/bioxp/protocols/submit",), "useSubmitBioXpProtocol", "frontend_action", "action", False, False),
    "useInvokeBioXpOperatorAction": (("/api/bioxp/operator-controls/actions/{actionId}",), "useInvokeBioXpOperatorAction", "frontend_action", "action", False, False),
    "useAssessBioXpOperatorAction": (("/api/bioxp/operator-controls/receipts/{commandId}/assessment",), "useAssessBioXpOperatorAction", "frontend_action", "action", False, False),
    "usePlanBioXpOemFullLifecycle": (("/api/bioxp/oem-full-lifecycle/runs",), "usePlanBioXpOemFullLifecycle", "frontend_action", "action", False, False),
    "useCancelBioXpOemFullLifecycle": (("/api/bioxp/oem-full-lifecycle/runs/{runId}/cancel",), "useCancelBioXpOemFullLifecycle", "frontend_action", "action", False, False),
}

# This is an independent closed-world inventory of mounted frontend consumers.
# Import and mount discovery must match it exactly; source discovery never adds
# identities to this policy.
FRONTEND_CONSUMER_POLICY: Mapping[str, Mapping[str, Any]] = {
    "platform/frontend/src/components/BioXpCockpit.tsx": {
        "component": "BioXpCockpit",
        "client_module": "../lib/bioxpClient",
        "client_imports": (
            "BIOXP_Y_ABSOLUTE_MAX_STEPS",
            "BIOXP_Y_ABSOLUTE_MIN_STEPS",
            "BIOXP_Y_RELATIVE_MAX_STEPS",
            "BioXpOperatorActionV2Request",
            "BioXpOperatorDashboardXAxis",
            "BioXpOperatorInputSpec",
            "bioXpErrorPresentation",
            "bioXpErrorText",
            "useBioXpOperatorActionHistory",
            "useBioXpOperatorControlCatalog",
            "useBioXpOperatorControlCatalogV2",
            "useBioXpOperatorDashboard",
            "useBioXpOperatorDashboardV2",
            "useBioXpOperatorReceiptV2",
            "useBioXpStatus",
            "useConnectBioXp",
            "useDisconnectBioXp",
            "useInterruptBioXpOperatorActionV1",
            "useInvokeBioXpOperatorAction",
            "useInvokeBioXpOperatorActionV2",
            "useRecoverBioXpMotion",
            "useSubmitBioXpOperatorMethodV1",
            "useUpdateBioXpFreshness",
        ),
        "client_type_imports": (
            "BioXpOperatorActionV2Request",
            "BioXpOperatorDashboardXAxis",
            "BioXpOperatorInputSpec",
        ),
        "client_calls": (
            "bioXpErrorPresentation",
            "bioXpErrorText",

            "useBioXpOperatorActionHistory",
            "useBioXpOperatorControlCatalog",
            "useBioXpOperatorControlCatalogV2",
            "useBioXpOperatorDashboard",
            "useBioXpOperatorDashboardV2",
            "useBioXpOperatorReceiptV2",
            "useBioXpStatus",
            "useConnectBioXp",
            "useDisconnectBioXp",
            "useInterruptBioXpOperatorActionV1",
            "useInvokeBioXpOperatorAction",
            "useInvokeBioXpOperatorActionV2",
            "useRecoverBioXpMotion",
            "useSubmitBioXpOperatorMethodV1",
            "useUpdateBioXpFreshness",
        ),
        "parent_path": "platform/frontend/src/App.tsx",
        "parent_component": "App",
        "parent_import_module": "./components/BioXpCockpit",
    },
    "platform/frontend/src/components/BioXpCameraPanel.tsx": {
        "component": "BioXpCameraPanel",
        "client_module": "../lib/bioxpClient",
        "client_imports": (
            "bioXpErrorText",
            "buildBioXpCameraMjpegUrl",
            "captureBioXpCameraSnapshot",
            "fetchBioXpCameraFrame",
            "startBioXpCameraStream",
            "stopBioXpCameraStream",
            "useBioXpCameraStatus",
            "useBioXpCameraStreamState",
        ),
        "client_type_imports": (),
        "client_calls": (
            "bioXpErrorText",
            "buildBioXpCameraMjpegUrl",
            "captureBioXpCameraSnapshot",
            "fetchBioXpCameraFrame",
            "startBioXpCameraStream",
            "stopBioXpCameraStream",
            "useBioXpCameraStatus",
            "useBioXpCameraStreamState",
        ),
        "parent_path": "platform/frontend/src/components/BioXpCockpit.tsx",
        "parent_component": "BioXpCockpit",
        "parent_import_module": "./BioXpCameraPanel",
    },
    "platform/frontend/src/components/BioXpOperatorControlTabs.tsx": {
        "component": "BioXpOperatorControlTabs",
        "client_module": "../lib/bioxpClient",
        "client_imports": (
            "BioXpOperatorActionReceipt",
            "BioXpOperatorActionSpec",
            "bioXpErrorText",
            "useAssessBioXpOperatorAction",
            "useBioXpOperatorActionAdmission",
            "useBioXpOperatorActionHistory",
            "useBioXpOperatorControlCatalog",
            "useBioXpOperatorDashboard",
            "useInvokeBioXpOperatorAction",
        ),
        "client_type_imports": ("BioXpOperatorActionReceipt", "BioXpOperatorActionSpec"),
        "client_calls": (
            "bioXpErrorText",
            "useAssessBioXpOperatorAction",
            "useBioXpOperatorActionAdmission",
            "useBioXpOperatorActionHistory",
            "useBioXpOperatorControlCatalog",
            "useBioXpOperatorDashboard",
            "useInvokeBioXpOperatorAction",
        ),
        "parent_path": "platform/frontend/src/components/BioXpCockpit.tsx",
        "parent_component": "BioXpCockpit",
        "parent_import_module": "./BioXpOperatorControlTabs",
    },
    "platform/frontend/src/components/BioXpOperatorReports.tsx": {
        "component": "BioXpOperatorReports",
        "client_module": "../lib/bioxpClient",
        "client_imports": (
            "BioXpOperatorReportCommandRow",
            "BioXpOperatorReportFilters",
            "useBioXpOperatorReportCommandDetail",
            "useBioXpOperatorReportCommandEvidence",
            "useBioXpOperatorReportCommandTransitions",
            "useBioXpOperatorReportCommands",
            "useBioXpOperatorReportEventDetail",
            "useBioXpOperatorReportEvents",
            "useBioXpOperatorReportExports",
            "useBioXpOperatorReportPipette",
            "useBioXpOperatorReportPipetteChannels",
            "useBioXpOperatorReportPipetteDetail",
            "useBioXpOperatorReportPipetteExchanges",
            "useBioXpOperatorReportPressureDetail",
            "useBioXpOperatorReportPressureSamples",
            "useBioXpOperatorReportPressureStreams",
            "useBioXpOperatorReportSummary",
            "useCreateBioXpOperatorReportExport",
        ),
        "client_type_imports": ("BioXpOperatorReportCommandRow", "BioXpOperatorReportFilters"),
        "client_calls": (
            "useBioXpOperatorReportCommandDetail",
            "useBioXpOperatorReportCommandEvidence",
            "useBioXpOperatorReportCommandTransitions",
            "useBioXpOperatorReportCommands",
            "useBioXpOperatorReportEventDetail",
            "useBioXpOperatorReportEvents",
            "useBioXpOperatorReportExports",
            "useBioXpOperatorReportPipette",
            "useBioXpOperatorReportPipetteChannels",
            "useBioXpOperatorReportPipetteDetail",
            "useBioXpOperatorReportPipetteExchanges",
            "useBioXpOperatorReportPressureDetail",
            "useBioXpOperatorReportPressureSamples",
            "useBioXpOperatorReportPressureStreams",
            "useBioXpOperatorReportSummary",
            "useCreateBioXpOperatorReportExport",
        ),
        "parent_path": "platform/frontend/src/components/BioXpCockpit.tsx",
        "parent_component": "BioXpCockpit",
        "parent_import_module": "./BioXpOperatorReports",
    },
    "platform/frontend/src/components/BioXpPipetteControlPanel.tsx": {
        "component": "BioXpPipetteControlPanel",
        "client_module": "../lib/bioxpClient",
        "client_imports": (
            "BioXpOperatorActionSpec",
            "BioXpOperatorDashboard",
            "BioXpPipetteApplicationOperation",
            "BioXpPipetteApplicationPlanRequest",
            "BioXpPipetteChannel",
            "BioXpPipetteHardwareEvidence",
            "bioXpErrorText",
            "useBioXpPipetteApplicationStatus",
            "usePlanBioXpPipetteApplication",
            "useReadBioXpPipetteReadback",
        ),
        "client_type_imports": (
            "BioXpOperatorActionSpec",
            "BioXpOperatorDashboard",
            "BioXpPipetteApplicationOperation",
            "BioXpPipetteApplicationPlanRequest",
            "BioXpPipetteChannel",
            "BioXpPipetteHardwareEvidence",
        ),
        "client_calls": (
            "bioXpErrorText",
            "useBioXpPipetteApplicationStatus",
            "usePlanBioXpPipetteApplication",
            "useReadBioXpPipetteReadback",
        ),
        "parent_path": "platform/frontend/src/components/BioXpCockpit.tsx",
        "parent_component": "BioXpCockpit",
        "parent_import_module": "./BioXpPipetteControlPanel",
    },
    "platform/frontend/src/components/BioXpQuickDashboard.tsx": {
        "component": "BioXpQuickDashboard",
        "client_module": "../lib/bioxpClient.js",
        "client_imports": ("BioXpOperatorDashboard",),
        "client_type_imports": ("BioXpOperatorDashboard",),
        "client_calls": (),
        "parent_path": "platform/frontend/src/components/BioXpCockpit.tsx",
        "parent_component": "BioXpCockpit",
        "parent_import_module": "./BioXpQuickDashboard",
    },
}

BMS_BACKEND_CHILD_ROUTER_POLICY: Mapping[str, str] = {
    "connection": "platform/api/routers/bioxp/connection.py",
    "camera": "platform/api/routers/bioxp/camera.py",
    "protocols": "platform/api/routers/bioxp/protocols.py",
    "jobs": "platform/api/routers/bioxp/jobs.py",
    "oem_full_lifecycle": "platform/api/routers/bioxp/oem_full_lifecycle.py",
    "operator_controls": "platform/api/routers/bioxp/operator_controls.py",
}
BMS_REPOSITORY_ROOT_IMPORT_PACKAGES = frozenset({"scripts"})
BMS_BACKEND_ROUTE_CLOSURE_SHA256 = "3152af04b13e1da039d4974718af092aa99a91c2ec34c02376f2d2f6b67bf7d8"
BMS_DIRECT_CLIENT_METHOD_POLICY: Mapping[str, str] = {
    "client.stream_camera_mjpeg": "camera_mjpeg",
}
_BMS_ROBOT_CLIENT_PATH = "platform/api/services/bioxp/robot_client.py"

ROBOT_SOURCE_PATHS = (
    GENERATOR_PATH,
    "src/bioxp/api.py",
    "src/bioxp/operator_controls.py",
    "src/bioxp/operator_command_plane.py",
    "src/bioxp/oem_compat/api.py",
    "src/bioxp/oem_runtime_api.py",
    "src/bioxp/oem_homing_routes.py",
    "src/bioxp/oem_serial206_initialization.py",
    "src/bioxp/operator_reports.py",
    "src/bioxp/pipette/transport.py",
    "src/bioxp/services/pipette_service.py",
    "src/bioxp/can_driver.py",
    "src/bioxp/novo_router.py",
    "src/bioxp/novo_usb_can.py",
    "src/bioxp/usb_driver.py",
)
BMS_SOURCE_PATHS = (
    "platform/api/main.py",
    "platform/api/routers/bioxp/__init__.py",
    "platform/api/routers/bioxp/dependencies.py",
    *BMS_BACKEND_CHILD_ROUTER_POLICY.values(),
    _BMS_ROBOT_CLIENT_PATH,
    "platform/api/services/bioxp/connection.py",
    "platform/frontend/src/App.tsx",
    *FRONTEND_CONSUMER_POLICY.keys(),
    "platform/frontend/src/lib/bioxpClient.ts",
)
ROBOT_PYTHON_IMPORT_ROOT_POLICY = tuple(
    path
    for path in ROBOT_SOURCE_PATHS
    if path.startswith("src/") and path.endswith(".py")
)
BMS_PYTHON_IMPORT_ROOT_POLICY = tuple(
    path
    for path in BMS_SOURCE_PATHS
    if path.startswith("platform/api/") and path != "platform/api/main.py"
)
BMS_MOUNT_ANCHOR_POLICY = ("platform/api/main.py",)


class DenominatorError(ValueError):
    pass


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True).encode("utf-8")


def canonical_payload_sha256(payload: Mapping[str, Any]) -> str:
    body = dict(payload)
    body.pop("canonical_payload", None)
    return _sha256_bytes(_canonical_bytes(body))


def _validate_repository_identity(
    name: str,
    root: Path,
    url: str,
    commit: str,
    tree: str,
) -> dict[str, str]:
    if type(url) is not str or not url or any(character.isspace() for character in url):
        raise DenominatorError(f"{name} repository URL must be supplied explicitly")
    if not (url.startswith(("https://", "ssh://", "git://")) or url.startswith("git@")):
        raise DenominatorError(f"{name} repository URL is not an accepted immutable-source locator")
    if type(commit) is not str or _GIT_OBJECT_RE.fullmatch(commit) is None:
        raise DenominatorError(f"{name} commit must be a full lowercase Git object ID")
    if type(tree) is not str or _GIT_OBJECT_RE.fullmatch(tree) is None:
        raise DenominatorError(f"{name} tree must be a full lowercase Git object ID")
    return {
        "repository_url": url,
        "commit": commit,
        "tree": tree,
    }


def _git(root: Path, *arguments: str) -> bytes:
    completed = subprocess.run(
        ["git", "-C", str(root), *arguments],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.decode("utf-8", errors="replace").strip()[:500]
        raise DenominatorError(f"Git identity lookup failed for {root}: {detail}")
    return completed.stdout


def _git_blob_at_commit(root: Path, commit: str, relative: str) -> bytes | None:
    completed = subprocess.run(
        ["git", "-C", str(root), "show", f"{commit}:{relative}"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if completed.returncode == 0:
        return completed.stdout
    detail = completed.stderr.decode("utf-8", errors="replace").strip()
    if "exists on disk, but not in" in detail or "does not exist in" in detail:
        return None
    raise DenominatorError(f"Git identity lookup failed for {root}: {detail[:500]}")


def _verify_repository_root(root: Path, name: str, identity: Mapping[str, str]) -> None:
    observed_root = Path(
        _git(root, "rev-parse", "--show-toplevel").decode("utf-8").strip()
    ).expanduser().resolve()
    observed_url = _git(root, "remote", "get-url", "origin").decode("utf-8").strip()
    observed_commit = _git(root, "rev-parse", "--verify", "HEAD").decode("ascii").strip()
    observed_tree = _git(root, "rev-parse", "--verify", "HEAD^{tree}").decode("ascii").strip()
    expected = (
        str(root),
        identity["repository_url"],
        identity["commit"],
        identity["tree"],
    )
    observed = (str(observed_root), observed_url, observed_commit, observed_tree)
    if observed != expected:
        raise DenominatorError(
            f"{name} source root identity does not match supplied root/URL/commit/tree: "
            f"expected={expected} observed={observed}"
        )


def _verified_root(value: str | Path, label: str) -> Path:
    root = Path(value).expanduser().resolve()
    if not root.is_dir():
        raise DenominatorError(f"{label} source root does not exist: {root}")
    return root


def _verified_path(root: Path, relative: str) -> Path:
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise DenominatorError(f"source path escapes supplied root: {relative}") from exc
    if not candidate.is_file():
        raise DenominatorError(f"required source path is missing: {relative}")
    return candidate


def _source_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _ast(path: Path) -> ast.Module:
    return ast.parse(_source_text(path), filename=str(path))


def _function(tree: ast.AST, name: str) -> ast.FunctionDef | ast.AsyncFunctionDef:
    matches = [
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    ]
    if len(matches) != 1:
        raise DenominatorError(f"expected exactly one function {name!r}, found {len(matches)}")
    return matches[0]


def _class(tree: ast.AST, name: str) -> ast.ClassDef:
    matches = [node for node in ast.walk(tree) if isinstance(node, ast.ClassDef) and node.name == name]
    if len(matches) != 1:
        raise DenominatorError(f"expected exactly one class {name!r}, found {len(matches)}")
    return matches[0]


def _call_name(node: ast.AST) -> str:
    try:
        return ast.unparse(node)
    except Exception:
        return ""


def _expression_identity(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        owner = _expression_identity(node.value)
        return None if owner is None else f"{owner}.{node.attr}"
    if isinstance(node, ast.Call):
        binding = _bind_call_target(node.func)
        targets = binding["resolved_target_identities"]
        if binding["unresolved_reason"] is not None or not targets:
            return None
        return f"call-result({'|'.join(targets)})"
    if isinstance(node, ast.Subscript):
        owner = _expression_identity(node.value)
        if owner is None:
            return None
        return f"subscription({owner}[{ast.unparse(node.slice)}])"
    if isinstance(node, ast.BoolOp):
        values = [_expression_identity(value) for value in node.values]
        if any(value is None for value in values):
            return None
        return f"boolop({type(node.op).__name__}:{'|'.join(str(value) for value in values)})"
    if isinstance(node, ast.BinOp):
        left = _expression_identity(node.left)
        right = _expression_identity(node.right)
        if left is None:
            left = ast.unparse(node.left)
        if right is None:
            right = ast.unparse(node.right)
        return f"binop({left}:{type(node.op).__name__}:{right})"
    if isinstance(node, ast.IfExp):
        body = _expression_identity(node.body)
        other = _expression_identity(node.orelse)
        if body is None or other is None:
            return None
        return f"ifexp({ast.unparse(node.test)}?{body}:{other})"
    if isinstance(
        node,
        (
            ast.Constant,
            ast.JoinedStr,
            ast.Dict,
            ast.Set,
            ast.List,
            ast.Tuple,
            ast.ListComp,
            ast.SetComp,
            ast.DictComp,
            ast.GeneratorExp,
            ast.UnaryOp,
            ast.Compare,
            ast.NamedExpr,
            ast.Lambda,
        ),
    ):
        return f"structural-expression({type(node).__name__}:{ast.unparse(node)})"
    return None


def _bind_call_target(node: ast.AST) -> dict[str, Any]:
    expression = ast.unparse(node)
    binding: dict[str, Any] = {
        "resolution_kind": "unresolved",
        "expression": expression,
        "selector_expression": None,
        "candidate_owner_identities": [],
        "resolved_target_identities": [],
        "unresolved_reason": None,
    }
    if isinstance(node, ast.Name):
        binding.update(
            resolution_kind="direct_name",
            candidate_owner_identities=[node.id],
            resolved_target_identities=[node.id],
        )
        return binding
    if isinstance(node, ast.Attribute):
        identity = _expression_identity(node)
        if identity is None:
            binding["unresolved_reason"] = "attribute_owner_unresolved"
            return binding
        binding.update(
            resolution_kind=(
                "direct_attribute"
                if isinstance(node.value, (ast.Name, ast.Attribute))
                else "computed_owner_attribute"
            ),
            candidate_owner_identities=[identity],
            resolved_target_identities=[identity],
        )
        return binding
    if isinstance(node, ast.Call):
        called = _expression_identity(node.func)
        if called in {"getattr", "builtins.getattr"}:
            if (
                len(node.args) == 2
                and isinstance(node.args[1], ast.Constant)
                and type(node.args[1].value) is str
            ):
                owner = _expression_identity(node.args[0])
                if owner is not None:
                    identity = f"{owner}.{node.args[1].value}"
                    binding.update(
                        resolution_kind="literal_getattr",
                        candidate_owner_identities=[identity],
                        resolved_target_identities=[identity],
                    )
                    return binding
            binding["unresolved_reason"] = "getattr_attribute_is_not_one_literal"
            return binding
        if called is not None and called.rsplit(".", 1)[-1] == "cast" and len(node.args) >= 2:
            wrapped = _bind_call_target(node.args[-1])
            if wrapped["unresolved_reason"] is None and wrapped["resolved_target_identities"]:
                binding.update(
                    resolution_kind="transparent_cast",
                    candidate_owner_identities=wrapped["candidate_owner_identities"],
                    resolved_target_identities=wrapped["resolved_target_identities"],
                    selector_expression=wrapped["selector_expression"],
                )
                return binding
            binding["unresolved_reason"] = "cast_callable_is_unresolved"
            return binding
        binding["unresolved_reason"] = "arbitrary_factory_return_callable"
        return binding
    if isinstance(node, ast.Subscript):
        values: Sequence[ast.AST]
        if isinstance(node.value, ast.Dict):
            values = tuple(value for value in node.value.values if value is not None)
        elif isinstance(node.value, (ast.List, ast.Tuple)):
            values = tuple(node.value.elts)
        else:
            binding["unresolved_reason"] = "open_callable_subscription"
            return binding
        candidate_bindings = [_bind_call_target(value) for value in values]
        if not values or any(
            candidate["unresolved_reason"] is not None
            or not candidate["resolved_target_identities"]
            for candidate in candidate_bindings
        ):
            binding["unresolved_reason"] = "finite_dispatch_contains_unresolved_candidate"
            return binding
        candidates = sorted(
            {
                target
                for candidate in candidate_bindings
                for target in candidate["resolved_target_identities"]
            }
        )
        binding.update(
            resolution_kind="finite_literal_dispatch",
            selector_expression=ast.unparse(node.slice),
            candidate_owner_identities=candidates,
            resolved_target_identities=candidates,
        )
        return binding
    if isinstance(node, ast.IfExp):
        branches = [_bind_call_target(node.body), _bind_call_target(node.orelse)]
        if any(branch["unresolved_reason"] is not None for branch in branches):
            binding["unresolved_reason"] = "conditional_callable_branch_unresolved"
            return binding
        candidates = sorted(
            {
                target
                for branch in branches
                for target in branch["resolved_target_identities"]
            }
        )
        binding.update(
            resolution_kind="finite_conditional_dispatch",
            selector_expression=ast.unparse(node.test),
            candidate_owner_identities=candidates,
            resolved_target_identities=candidates,
        )
        return binding
    if isinstance(node, ast.BoolOp):
        branches = [_bind_call_target(value) for value in node.values]
        if any(branch["unresolved_reason"] is not None for branch in branches):
            binding["unresolved_reason"] = "fallback_callable_branch_unresolved"
            return binding
        candidates = sorted(
            {
                target
                for branch in branches
                for target in branch["resolved_target_identities"]
            }
        )
        binding.update(
            resolution_kind="finite_fallback_dispatch",
            candidate_owner_identities=candidates,
            resolved_target_identities=candidates,
        )
        return binding
    if isinstance(node, ast.Lambda):
        identity = f"lambda@{getattr(node, 'lineno', 0)}:{getattr(node, 'col_offset', 0)}"
        binding.update(
            resolution_kind="immediate_lambda",
            candidate_owner_identities=[identity],
            resolved_target_identities=[identity],
        )
        return binding
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mult):
        left = _expression_identity(node.left)
        right = _expression_identity(node.right)
        if (left and left.startswith("ctypes.")) or (right and right.startswith("ctypes.")):
            identity = f"operator-result({expression})"
            binding.update(
                resolution_kind="ctypes_array_factory",
                candidate_owner_identities=[identity],
                resolved_target_identities=[identity],
            )
            return binding
    binding["unresolved_reason"] = f"unsupported_call_target:{type(node).__name__}"
    return binding


def _resolved_call_name(node: ast.AST) -> str:
    binding = _bind_call_target(node)
    targets = binding["resolved_target_identities"]
    if binding["unresolved_reason"] is not None or not targets:
        raise DenominatorError(
            f"call target is indirect or unresolved at line {getattr(node, 'lineno', '?')}: "
            f"expression={_call_name(node)!r} reason={binding['unresolved_reason']}"
        )
    if len(targets) == 1:
        return str(targets[0])
    return "finite{" + "|".join(str(target) for target in targets) + "}"


def _call_names(node: ast.AST) -> set[str]:
    return {
        _resolved_call_name(child.func)
        for child in ast.walk(node)
        if isinstance(child, ast.Call)
    }


def _dispatch_path(
    node: ast.AST,
    *,
    route_key: tuple[str, str] | None = None,
    no_effect: bool = False,
    plan_only: bool = False,
) -> str:
    call_occurrences = tuple(
        sorted(
            _resolved_call_name(candidate.func)
            for candidate in ast.walk(node)
            if isinstance(candidate, ast.Call)
        )
    )
    calls = set(call_occurrences)
    if route_key is not None:
        expected_digest = ROBOT_ROUTE_CALL_SHA256_POLICY.get(route_key)
        expected_dispatch = ROBOT_ROUTE_DISPATCH_CALL_POLICY.get(route_key)
        if expected_digest is None or expected_dispatch is None:
            raise DenominatorError(f"liquid route call policy is missing: {route_key}")
        observed_digest = _sha256_bytes(_canonical_bytes(call_occurrences))
        if observed_digest != expected_digest:
            raise DenominatorError(
                f"liquid route call inventory changed: route={route_key} "
                f"expected_sha256={expected_digest} observed_sha256={observed_digest} "
                f"calls={call_occurrences}"
            )
        observed_dispatch = tuple(
            name
            for name in call_occurrences
            if (
                name.startswith("run_pipette_")
                or name.startswith("transport.")
                or name.startswith("_pipette_application.plan_")
            )
        )
        if observed_dispatch != expected_dispatch:
            raise DenominatorError(
                f"liquid route dispatch targets changed: route={route_key} "
                f"expected={expected_dispatch} observed={observed_dispatch}"
            )
    if no_effect:
        if route_key is not None and ROBOT_ROUTE_DISPATCH_CALL_POLICY[route_key]:
            raise DenominatorError(f"projection route has a dispatch target: {route_key}")
        return "projection_only"
    if plan_only:
        if route_key is not None and not ROBOT_ROUTE_DISPATCH_CALL_POLICY[route_key]:
            raise DenominatorError(f"plan route has no planner target: {route_key}")
        return "plan_only"
    if route_key is not None:
        if not ROBOT_ROUTE_DISPATCH_CALL_POLICY[route_key]:
            raise DenominatorError(f"effectful liquid route has no dispatch target: {route_key}")
        return "coordinator"
    if any(
        name.startswith("run_pipette_")
        or name.endswith("._run_audited_pipette")
        or name.endswith(".query_tip_status")
        or name == "_run_serial206_pipette_audit"
        for name in calls
    ):
        return "coordinator"
    raise DenominatorError(
        "endpoint dispatch does not resolve to a governed coordinator: "
        f"calls={sorted(calls)!r}"
    )


def _row(
    *,
    family: str,
    repository: str,
    source_file: str,
    source_sha256: str,
    source_line: int,
    identifier: str,
    final_callable: str,
    control_class: str,
    transport_effect: str,
    physical_effect_possible: bool,
    requires_durable_claim: bool,
    dispatch_path: str,
    verification_id: str,
    oem_anchor: str | None = None,
    mutation_gate: str = "robot_durable_claim",
    correlation_fields: Iterable[str] = _REQUIRED_CORRELATION_FIELDS,
    report_projection: str = "robot_runtime_audit",
    bms_exposure: Any = "none",
    cockpit_control: str = "none",
) -> dict[str, Any]:
    return {
        "id": f"{family}:{repository}:{source_file}:{source_line}:{identifier}",
        "family": family,
        "repository": repository,
        "source_file": source_file,
        "source_sha256": source_sha256,
        "source_line": int(source_line),
        "public_method_or_path": identifier,
        "final_callable": final_callable,
        "oem_anchor": oem_anchor,
        "control_class": control_class,
        "transport_effect": transport_effect,
        "physical_effect_possible": bool(physical_effect_possible),
        "requires_durable_claim": bool(requires_durable_claim),
        "mutation_gate": mutation_gate,
        "dispatch_path": dispatch_path,
        "correlation_fields": sorted(set(correlation_fields)),
        "report_projection": report_projection,
        "bms_exposure": bms_exposure,
        "cockpit_control": cockpit_control,
        "verification_id": verification_id,
    }


def _python_module_name(relative: str, source_prefix: str) -> str:
    path = Path(relative)
    try:
        module_path = path.relative_to(source_prefix).with_suffix("")
    except ValueError as exc:
        raise DenominatorError(
            f"Python source path is outside its scoped import root: {relative}"
        ) from exc
    parts = list(module_path.parts)
    if parts and parts[-1] == "__init__":
        parts.pop()
    return ".".join(parts)


def _python_module_variants(module: str, source_prefix: str) -> tuple[str, ...]:
    variants = [module]
    if source_prefix == "platform/api":
        for prefix in ("platform.api.", "api."):
            if module.startswith(prefix):
                variants.append(module[len(prefix):])
        if module in {"platform.api", "api"}:
            variants.append("")
    elif source_prefix == "src" and module.startswith("src."):
        variants.append(module[4:])
    return tuple(dict.fromkeys(variants))


def _resolve_python_module(
    root: Path,
    *,
    source_prefix: str,
    module: str,
) -> tuple[str, tuple[str, ...]] | None:
    source_root = (root / source_prefix).resolve()
    matches: set[Path] = set()
    for variant in _python_module_variants(module, source_prefix):
        module_path = Path(*([part for part in variant.split(".") if part]))
        base = source_root / module_path
        candidates = (
            base.with_suffix(".py") if variant else source_root / "__init__.py",
            base / "__init__.py",
        )
        for candidate in candidates:
            absolute = candidate.resolve()
            try:
                absolute.relative_to(source_root)
            except ValueError:
                continue
            if absolute.is_file():
                matches.add(absolute)
    if not matches:
        return None
    if len(matches) != 1:
        raise DenominatorError(
            f"local Python module owner is ambiguous: module={module!r} "
            f"matches={[path.relative_to(root).as_posix() for path in sorted(matches)]!r}"
        )
    target = next(iter(matches))
    package_paths: list[str] = []
    relative_parent = target.parent.relative_to(source_root)
    current = source_root
    for part in relative_parent.parts:
        current /= part
        package_init = current / "__init__.py"
        if package_init.is_file():
            package_paths.append(package_init.relative_to(root).as_posix())
    target_relative = target.relative_to(root).as_posix()
    if target.name == "__init__.py" and target_relative not in package_paths:
        package_paths.append(target_relative)
    return target_relative, tuple(package_paths)


def _effective_python_source_prefix(source_path: str, source_prefix: str) -> str:
    if (
        source_prefix == "platform/api"
        and source_path.split("/", 1)[0] in BMS_REPOSITORY_ROOT_IMPORT_PACKAGES
    ):
        return ""
    return source_prefix


def _resolve_scoped_python_module(
    root: Path,
    *,
    source_path: str,
    source_prefix: str,
    module: str,
) -> tuple[str, tuple[str, ...]] | None:
    effective_prefix = _effective_python_source_prefix(source_path, source_prefix)
    prefixes = [effective_prefix]
    first = module.split(".", 1)[0] if module else ""
    if (
        source_prefix == "platform/api"
        and first in BMS_REPOSITORY_ROOT_IMPORT_PACKAGES
        and "" not in prefixes
    ):
        prefixes.insert(0, "")
    matches: dict[str, tuple[str, tuple[str, ...]]] = {}
    for candidate_prefix in prefixes:
        resolved = _resolve_python_module(
            root,
            source_prefix=candidate_prefix,
            module=module,
        )
        if resolved is not None:
            matches[resolved[0]] = resolved
    if len(matches) > 1:
        raise DenominatorError(
            f"scoped Python module owner is ambiguous: source={source_path} "
            f"module={module!r} matches={sorted(matches)!r}"
        )
    return None if not matches else next(iter(matches.values()))


def _relative_import_module(
    source_path: str,
    *,
    source_prefix: str,
    level: int,
    module: str | None,
) -> str:
    current_module = _python_module_name(source_path, source_prefix)
    package_parts = current_module.split(".") if current_module else []
    if not source_path.endswith("/__init__.py") and package_parts:
        package_parts.pop()
    ascend = max(0, int(level) - 1)
    if ascend > len(package_parts):
        raise DenominatorError(
            f"relative Python import escapes scoped package: {source_path}:{level}"
        )
    if ascend:
        package_parts = package_parts[:-ascend]
    if module:
        package_parts.extend(module.split("."))
    return ".".join(package_parts)


def _python_import_graph(
    root: Path,
    *,
    repository: str,
    source_prefix: str,
    roots: Sequence[str],
) -> dict[str, Any]:
    queue = list(dict.fromkeys(roots))
    visited: set[str] = set()
    local_paths: set[str] = set(queue)
    edges: list[dict[str, Any]] = []
    while queue:
        source_path = queue.pop(0)
        if source_path in visited:
            continue
        effective_source_prefix = _effective_python_source_prefix(
            source_path,
            source_prefix,
        )
        tree = _ast(_verified_path(root, source_path))
        visited.add(source_path)
        imports = sorted(
            (
                node
                for node in ast.walk(tree)
                if isinstance(node, (ast.Import, ast.ImportFrom))
            ),
            key=lambda node: (int(node.lineno), int(node.col_offset)),
        )
        occurrence = 0
        for node in imports:
            if isinstance(node, ast.Import):
                import_rows = [
                    (alias.name, alias.name, alias.asname, 0)
                    for alias in node.names
                ]
            else:
                base_module = (
                    _relative_import_module(
                        source_path,
                        source_prefix=effective_source_prefix,
                        level=int(node.level),
                        module=node.module,
                    )
                    if node.level
                    else str(node.module or "")
                )
                import_rows = [
                    (
                        f"{base_module}.{alias.name}" if base_module and alias.name != "*" else base_module,
                        base_module,
                        alias.asname,
                        int(node.level),
                    )
                    for alias in node.names
                ]
            for requested_module, base_module, alias_name, level in import_rows:
                occurrence += 1
                resolved = _resolve_scoped_python_module(
                    root,
                    source_path=source_path,
                    source_prefix=source_prefix,
                    module=requested_module,
                )
                if resolved is None and base_module != requested_module:
                    resolved = _resolve_scoped_python_module(
                        root,
                        source_path=source_path,
                        source_prefix=source_prefix,
                        module=base_module,
                    )
                expected_local = bool(level)
                if not expected_local and requested_module:
                    first = requested_module.split(".", 1)[0]
                    expected_local = (
                        (root / effective_source_prefix / first).exists()
                        or (root / effective_source_prefix / f"{first}.py").is_file()
                        or (
                            source_prefix == "platform/api"
                            and "." in requested_module
                            and first in {"platform", "api"}
                        )
                        or (
                            source_prefix == "platform/api"
                            and first in BMS_REPOSITORY_ROOT_IMPORT_PACKAGES
                        )
                    )
                if resolved is None and expected_local:
                    raise DenominatorError(
                        f"scoped local Python import is unresolved: "
                        f"repository={repository} source={source_path} "
                        f"line={node.lineno} module={requested_module!r}"
                    )
                target = None if resolved is None else resolved[0]
                package_paths = () if resolved is None else resolved[1]
                edge = {
                    "repository": repository,
                    "source": source_path,
                    "source_line": int(node.lineno),
                    "occurrence": occurrence,
                    "kind": "import" if isinstance(node, ast.Import) else "from_import",
                    "level": level,
                    "module": requested_module,
                    "alias": alias_name,
                    "target": target,
                    "package_paths": list(package_paths),
                    "external": resolved is None,
                }
                edges.append(edge)
                for local_path in (*package_paths, *((target,) if target else ())):
                    local_paths.add(local_path)
                    if local_path not in visited and local_path not in queue:
                        queue.append(local_path)
    canonical_edges = tuple(
        sorted(
            edges,
            key=lambda edge: (
                edge["source"],
                edge["source_line"],
                edge["occurrence"],
                edge["module"],
            ),
        )
    )
    return {
        "repository": repository,
        "source_prefix": source_prefix,
        "roots": list(dict.fromkeys(roots)),
        "paths": sorted(local_paths),
        "edges": canonical_edges,
        "edge_count": len(canonical_edges),
        "external_edge_count": sum(1 for edge in canonical_edges if edge["external"]),
        "unresolved_local_count": 0,
        "sha256": _sha256_bytes(_canonical_bytes(canonical_edges)),
    }


def _source_manifest(
    robot: Path,
    bms: Path,
) -> tuple[
    list[dict[str, Any]],
    dict[tuple[str, str], str],
    dict[str, Any],
]:
    manifest: list[dict[str, Any]] = []
    digests: dict[tuple[str, str], str] = {}
    frontend_graph = _frontend_mount_graph(bms)
    discovered_frontend_paths = set(frontend_graph["reachable"])
    robot_import_graph = _python_import_graph(
        robot,
        repository="robot",
        source_prefix="src",
        roots=ROBOT_PYTHON_IMPORT_ROOT_POLICY,
    )
    bms_import_graph = _python_import_graph(
        bms,
        repository="bms",
        source_prefix="platform/api",
        roots=BMS_PYTHON_IMPORT_ROOT_POLICY,
    )
    robot_source_paths = tuple(
        dict.fromkeys((GENERATOR_PATH, *robot_import_graph["paths"]))
    )
    bms_source_paths = tuple(
        dict.fromkeys(
            (
                *BMS_MOUNT_ANCHOR_POLICY,
                *bms_import_graph["paths"],
                *sorted(discovered_frontend_paths),
            )
        )
    )
    for repository, root, paths in (
        ("robot", robot, robot_source_paths),
        ("bms", bms, bms_source_paths),
    ):
        for relative in paths:
            path = _verified_path(root, relative)
            raw = path.read_bytes()
            digest = _sha256_bytes(raw)
            manifest.append(
                {
                    "repository": repository,
                    "path": relative,
                    "byte_count": len(raw),
                    "sha256": digest,
                }
            )
            digests[(repository, relative)] = digest
    manifest.sort(key=lambda row: (row["repository"], row["path"]))
    if len({(row["repository"], row["path"]) for row in manifest}) != len(manifest):
        raise DenominatorError("source manifest contains duplicate repository/path identities")
    for key, expected_sha256 in SOURCE_AUTHORITY_SHA256_POLICY.items():
        observed_sha256 = digests.get(key)
        if observed_sha256 != expected_sha256:
            raise DenominatorError(
                f"denominator authority source bytes changed: key={key!r} "
                f"expected={expected_sha256} actual={observed_sha256}"
            )
    import_graphs = {
        "robot_python": robot_import_graph,
        "bms_python": bms_import_graph,
        "bms_frontend": {
            "repository": "bms",
            "roots": [_FRONTEND_APP_PATH],
            "paths": sorted(discovered_frontend_paths),
            "edges": frontend_graph["edges"],
            "edge_count": len(frontend_graph["edges"]),
            "unresolved_local_count": 0,
            "sha256": frontend_graph["sha256"],
        },
    }
    return manifest, digests, import_graphs


def _manifest_commit_differences(
    manifest: Sequence[Mapping[str, Any]],
    *,
    robot: Path,
    bms: Path,
    repositories: Mapping[str, Mapping[str, str]],
) -> list[dict[str, Any]]:
    roots = {"robot": robot, "bms": bms}
    differences: list[dict[str, Any]] = []
    for row in manifest:
        repository = str(row["repository"])
        relative = str(row["path"])
        committed = _git_blob_at_commit(
            roots[repository],
            repositories[repository]["commit"],
            relative,
        )
        committed_sha256 = None if committed is None else _sha256_bytes(committed)
        if committed is None or len(committed) != row["byte_count"] or committed_sha256 != row["sha256"]:
            differences.append(
                {
                    "repository": repository,
                    "path": relative,
                    "base_commit_present": committed is not None,
                    "base_commit_sha256": committed_sha256,
                    "working_sha256": str(row["sha256"]),
                }
            )
    return differences


def _fastapi_default_operation_id(function_name: str, path: str, method: str) -> str:
    return re.sub(r"\W", "_", f"{function_name}{path}") + f"_{method.lower()}"


def _operator_catalog_exclusions(robot: Path) -> None:
    relative = "src/bioxp/operator_controls.py"
    tree = _ast(_verified_path(robot, relative))
    compat_assignments = [
        node.value
        for node in tree.body
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        and (
            (isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "_NON_OPERATOR_COMPAT_PATHS" for target in node.targets))
            or (isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == "_NON_OPERATOR_COMPAT_PATHS")
        )
        and node.value is not None
    ]
    if len(compat_assignments) != 1:
        raise DenominatorError("operator catalog compatibility exclusion authority changed")
    compat = _literal(compat_assignments[0])
    expected_compat = set(OPERATOR_CATALOG_EXCLUSION_POLICY) - {"/", "/openapi.json", "/docs", "/docs/oauth2-redirect", "/redoc"}
    if type(compat) is not set or compat != expected_compat:
        raise DenominatorError(f"operator catalog compatibility exclusions changed: {compat!r}")

    builder = _function(tree, "_build_catalog")
    catalog_ast_sha256 = _sha256_bytes(
        ast.dump(builder, include_attributes=False).encode("utf-8")
    )
    if catalog_ast_sha256 != OPERATOR_CATALOG_AST_SHA256:
        raise DenominatorError(
            "operator catalog complete AST authority changed: "
            f"expected={OPERATOR_CATALOG_AST_SHA256} actual={catalog_ast_sha256}"
        )
    expected_prefix = ast.parse(
        '''
document = app.openapi()
actions: list[dict[str, Any]] = []
dispatch: dict[str, dict[str, Any]] = {}
excluded = {"/", "/openapi.json", "/docs", "/docs/oauth2-redirect", "/redoc", *_NON_OPERATOR_COMPAT_PATHS}
for path, path_item in sorted(document.get("paths", {}).items()):
    if path in excluded or path.startswith("/operator/") or not isinstance(path_item, Mapping):
        continue
    for method in ("get", "post", "put", "patch", "delete"):
        operation = path_item.get(method)
        if not isinstance(operation, Mapping):
            continue
        upper = method.upper()
        action_id = _path_action_id(upper, path, operation.get("operationId"))
        inputs, locations = _extract_inputs(operation, document)
        safety = _safety(upper, path)
        local_only = any(path.startswith(prefix) for prefix in _LOCAL_ONLY_PATH_PREFIXES)
        semantic_quarantine_reason = _OPERATOR_SEMANTIC_QUARANTINE_PATHS.get(path)
        unavailable_reason = (
            "Local-only maintenance route is not callable through the operator relay."
            if local_only
            else semantic_quarantine_reason
        )
        dispatchable = unavailable_reason is None
        action = {
            "action_id": action_id,
            "label": str(operation.get("summary") or operation.get("operationId") or f"{upper} {path}")[:160],
            "subsystem": _subsystem(path),
            "category": "route",
            "kind": "primitive",
            "safety_class": safety,
            "description": str(operation.get("description") or f"Exact existing robot route: {upper} {path}")[:2000],
            "source_anchor": None,
            "informational_method": upper,
            "informational_path": path,
            "required_provider_capability": _SERIAL206_PROVIDER_CAPABILITIES.get(path),
            "provider_available": dispatchable,
            "provider_unavailable_reason": unavailable_reason,
            "available": dispatchable,
            "unavailable_reason": unavailable_reason,
            "enabled": dispatchable,
            "disabled_reason": unavailable_reason,
            "dependencies": [],
            "requires_confirmation": safety not in {"read_only", "emergency", "stop"},
            "timeout_seconds": (
                360.0
                if path in _SERIAL206_PROVIDER_CAPABILITIES
                else (120.0 if safety == "motion" else 30.0)
            ),
            "inputs": inputs,
            "stages": [],
        }
        actions.append(action)
        if dispatchable:
            dispatch[action_id] = {"method": upper, "path": path, "locations": locations, "inputs": {row["name"] for row in inputs}}
'''
    ).body
    if len(builder.body) < len(expected_prefix) or tuple(ast.dump(node) for node in builder.body[:5]) != tuple(
        ast.dump(node) for node in expected_prefix
    ):
        raise DenominatorError("operator catalog eligibility prefix changed")

    scope_nodes: list[ast.AST] = []
    pending: list[ast.AST] = [
        node
        for node in reversed(builder.body)
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
    ]
    while pending:
        node = pending.pop()
        scope_nodes.append(node)
        for child in reversed(list(ast.iter_child_nodes(node))):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)):
                continue
            pending.append(child)
    returns = [node for node in scope_nodes if isinstance(node, ast.Return)]
    if len(returns) != 1 or returns[0] is not builder.body[-1]:
        raise DenominatorError("operator catalog outer return authority changed")
    if any(isinstance(node, ast.Break) for node in scope_nodes):
        raise DenominatorError("operator catalog gained an eligibility-changing break")

    def root_name(target: ast.AST) -> str | None:
        selected = target
        while isinstance(selected, (ast.Attribute, ast.Subscript)):
            selected = selected.value
        return selected.id if isinstance(selected, ast.Name) else None

    protected = {"actions", "dispatch", "excluded"}
    for node in scope_nodes:
        if isinstance(node, ast.Delete) and any(root_name(target) in protected for target in node.targets):
            raise DenominatorError("operator catalog deletes authority-bearing eligibility state")

    assignments: dict[str, list[ast.AST]] = {"actions": [], "dispatch": []}
    for node in scope_nodes:
        targets: list[ast.AST] = []
        if isinstance(node, ast.Assign):
            targets = list(node.targets)
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        elif isinstance(node, ast.AugAssign):
            targets = [node.target]
        elif isinstance(node, ast.NamedExpr):
            targets = [node.target]
        for target in targets:
            if isinstance(target, ast.Name) and target.id in assignments:
                assignments[target.id].append(node)
    if len(assignments["actions"]) != 2 or len(assignments["dispatch"]) != 1:
        raise DenominatorError(f"operator catalog actions/dispatch assignment authority changed: {assignments}")

    expected_tail = ast.parse(
        '''
actions = [row for row in actions if not str(row.get("action_id", "")).startswith("oem.y.internal.")]
return actions, dispatch
'''
    ).body
    if tuple(ast.dump(node) for node in builder.body[-2:]) != tuple(ast.dump(node) for node in expected_tail):
        raise DenominatorError("operator catalog final authority projection changed")


def _verify_command_plane_router_owner(
    robot: Path,
) -> dict[tuple[str, str, str], dict[str, Any]]:
    relative = "src/bioxp/operator_command_plane.py"
    tree = _ast(_verified_path(robot, relative))
    owner = _class(tree, "OperatorCommandPlane")
    initializer = _function(owner, "__init__")
    router_assignments = [
        node
        for node in ast.walk(initializer)
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and ast.unparse(node.targets[0]) == "self.router"
        and isinstance(node.value, ast.Call)
        and _call_name(node.value.func) == "APIRouter"
    ]
    install_calls = [
        node
        for node in ast.walk(initializer)
        if isinstance(node, ast.Call) and _call_name(node.func) == "self._install_routes"
    ]
    if len(router_assignments) != 1 or len(install_calls) != 1:
        raise DenominatorError("command-plane router construction or installation ownership changed")
    router_call = router_assignments[0].value
    if not isinstance(router_call, ast.Call):
        raise DenominatorError("command-plane router constructor is unresolved")
    prefix_nodes = [
        keyword.value
        for keyword in router_call.keywords
        if keyword.arg == "prefix"
    ]
    if (
        len(prefix_nodes) != 1
        or not isinstance(prefix_nodes[0], ast.Constant)
        or prefix_nodes[0].value != "/operator"
    ):
        raise DenominatorError("command-plane router prefix changed")

    installer = _function(owner, "_install_routes")
    expected_alias = ast.parse("router = self.router").body[0]
    if not installer.body or ast.dump(installer.body[0]) != ast.dump(expected_alias):
        raise DenominatorError("command-plane route owner alias changed")
    actual_routes: dict[tuple[str, str, str], dict[str, Any]] = {}
    for node in ast.walk(installer):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "router"
            and node.func.attr in {"add_api_route", "api_route", "include_router"}
        ):
            raise DenominatorError("command-plane router gained imperative or nested registration")
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for decorator in node.decorator_list:
            if (
                not isinstance(decorator, ast.Call)
                or not isinstance(decorator.func, ast.Attribute)
                or not isinstance(decorator.func.value, ast.Name)
                or decorator.func.value.id != "router"
            ):
                continue
            if (
                decorator.func.attr not in {"get", "post", "put", "patch", "delete"}
                or len(decorator.args) != 1
                or not isinstance(decorator.args[0], ast.Constant)
                or type(decorator.args[0].value) is not str
            ):
                raise DenominatorError(f"command-plane route declaration is unresolved: {relative}:{decorator.lineno}")
            key = (
                decorator.func.attr.upper(),
                str(decorator.args[0].value),
                node.name,
            )
            if key in actual_routes:
                raise DenominatorError(f"duplicate command-plane route declaration: {key}")
            graph = _reachable_call_graph(tree, node, source_file=relative)
            if graph["unresolved_target_count"]:
                raise DenominatorError(
                    f"command-plane route has unresolved call targets: "
                    f"route={key!r} targets={graph['unresolved_targets']!r}"
                )
            actual_routes[key] = {
                "method": key[0],
                "path": "/operator" + key[1],
                "source_file": relative,
                "source_line": int(node.lineno),
                "decorator_line": int(decorator.lineno),
                "handler_owner": graph["root_owner"],
                "call_graph": graph,
            }
    actual_keys = frozenset(actual_routes)
    if actual_keys != COMMAND_PLANE_ROUTE_POLICY:
        raise DenominatorError(
            f"command-plane mounted route ownership changed: "
            f"missing={sorted(COMMAND_PLANE_ROUTE_POLICY - actual_keys)!r} "
            f"extra={sorted(actual_keys - COMMAND_PLANE_ROUTE_POLICY)!r}"
        )
    return actual_routes


class _ScopeCallVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.calls: list[ast.Call] = []

    def visit_Call(self, node: ast.Call) -> None:
        self.calls.append(node)
        self.generic_visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        return

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        return

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        return

    def visit_Lambda(self, node: ast.Lambda) -> None:
        return


def _scope_calls(
    function: ast.FunctionDef | ast.AsyncFunctionDef,
) -> tuple[ast.Call, ...]:
    visitor = _ScopeCallVisitor()
    for statement in function.body:
        visitor.visit(statement)
    return tuple(
        sorted(
            visitor.calls,
            key=lambda call: (int(call.lineno), int(call.col_offset)),
        )
    )


def _definition_inventory(
    tree: ast.Module,
) -> tuple[
    dict[int, str],
    dict[str, ast.FunctionDef | ast.AsyncFunctionDef],
    set[str],
]:
    owner_by_identity: dict[int, str] = {}
    functions: dict[str, ast.FunctionDef | ast.AsyncFunctionDef] = {}
    classes: set[str] = set()

    def visit_body(statements: Sequence[ast.stmt], prefix: str) -> None:
        for statement in statements:
            if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef)):
                owner = f"{prefix}.{statement.name}" if prefix else statement.name
                if owner in functions:
                    raise DenominatorError(f"duplicate callable owner identity: {owner}")
                owner_by_identity[id(statement)] = owner
                functions[owner] = statement
                visit_body(statement.body, owner)
            elif isinstance(statement, ast.ClassDef):
                owner = f"{prefix}.{statement.name}" if prefix else statement.name
                classes.add(owner)
                visit_body(statement.body, owner)

    visit_body(tree.body, "")
    return owner_by_identity, functions, classes


def _local_callable_targets(
    targets: Sequence[str],
    *,
    caller: str,
    functions: Mapping[str, ast.FunctionDef | ast.AsyncFunctionDef],
    classes: set[str],
) -> tuple[str, ...]:
    resolved: list[str] = []
    caller_parent = caller.rsplit(".", 1)[0] if "." in caller else ""
    enclosing_classes = sorted(
        (owner for owner in classes if caller == owner or caller.startswith(f"{owner}.")),
        key=len,
        reverse=True,
    )
    for target in targets:
        candidates: list[str] = []
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", target):
            candidates.extend(
                candidate
                for candidate in (
                    f"{caller}.{target}",
                    (
                        f"{caller_parent}.{target}"
                        if caller_parent and caller_parent not in classes
                        else target
                    ),
                    target,
                )
                if candidate in functions
            )
        elif target.startswith("self.") and enclosing_classes:
            candidate = f"{enclosing_classes[0]}.{target[5:]}"
            if candidate in functions:
                candidates.append(candidate)
        unique = tuple(dict.fromkeys(candidates))
        if len(unique) > 1:
            raise DenominatorError(
                f"call target resolves to multiple local owners: caller={caller} "
                f"target={target} candidates={unique!r}"
            )
        resolved.extend(unique)
    return tuple(dict.fromkeys(resolved))


def _reachable_call_graph(
    tree: ast.Module,
    root: ast.FunctionDef | ast.AsyncFunctionDef,
    *,
    source_file: str,
) -> dict[str, Any]:
    owner_by_identity, functions, classes = _definition_inventory(tree)
    root_owner = owner_by_identity.get(id(root))
    if root_owner is None:
        raise DenominatorError(f"call-graph root has no qualified source owner: {source_file}:{root.name}")
    queue = [root_owner]
    visited: set[str] = set()
    edges: list[dict[str, Any]] = []
    unresolved: list[dict[str, Any]] = []
    while queue:
        caller = queue.pop(0)
        if caller in visited:
            continue
        function = functions[caller]
        visited.add(caller)
        for occurrence, candidate in enumerate(_scope_calls(function), start=1):
            binding = _bind_call_target(candidate.func)
            edge = {
                "repository_source": source_file,
                "caller_qualname": caller,
                "source_line": int(candidate.lineno),
                "source_column": int(candidate.col_offset),
                "occurrence": occurrence,
                **binding,
            }
            edges.append(edge)
            if binding["unresolved_reason"] is not None:
                unresolved.append(edge)
                continue
            for local_owner in _local_callable_targets(
                tuple(str(target) for target in binding["resolved_target_identities"]),
                caller=caller,
                functions=functions,
                classes=classes,
            ):
                if local_owner not in visited and local_owner not in queue:
                    queue.append(local_owner)
    canonical_edges = tuple(
        sorted(
            edges,
            key=lambda edge: (
                edge["caller_qualname"],
                edge["source_line"],
                edge["source_column"],
                edge["occurrence"],
            ),
        )
    )
    multiplicities = Counter(
        (
            str(edge["caller_qualname"]),
            str(edge["resolution_kind"]),
            str(edge["expression"]),
            tuple(str(value) for value in edge["resolved_target_identities"]),
        )
        for edge in canonical_edges
    )
    multiplicity_rows = tuple(
        {
            "caller_qualname": key[0],
            "resolution_kind": key[1],
            "expression": key[2],
            "resolved_target_identities": list(key[3]),
            "multiplicity": count,
        }
        for key, count in sorted(multiplicities.items())
    )
    graph_body = {
        "root_owner": root_owner,
        "source_file": source_file,
        "visited_owner_identities": sorted(visited),
        "edges": canonical_edges,
        "edge_count": len(canonical_edges),
        "multiplicities": multiplicity_rows,
        "unresolved_targets": tuple(unresolved),
        "unresolved_target_count": len(unresolved),
    }
    return {
        **graph_body,
        "sha256": _sha256_bytes(_canonical_bytes(graph_body)),
    }


def _function_call_graph_sha256(
    tree: ast.Module,
    root: ast.FunctionDef | ast.AsyncFunctionDef,
    *,
    source_file: str,
) -> str:
    graph = _reachable_call_graph(tree, root, source_file=source_file)
    if graph["unresolved_target_count"]:
        raise DenominatorError(
            f"mounted route has unresolved call targets: "
            f"source={source_file} root={graph['root_owner']} "
            f"targets={graph['unresolved_targets']!r}"
        )
    if not graph["edges"]:
        raise DenominatorError(
            f"mounted route has no structurally reachable call graph: {graph['root_owner']}"
        )
    return str(graph["sha256"])


def _report_route_owners(robot: Path) -> dict[tuple[str, str, str], dict[str, Any]]:
    relative = "src/bioxp/operator_reports.py"
    tree = _ast(_verified_path(robot, relative))
    factory = _function(tree, "create_operator_reports_router")
    router_assignments = [
        node
        for node in factory.body
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        and any(
            isinstance(target, ast.Name) and target.id == "router"
            for target in (
                node.targets if isinstance(node, ast.Assign) else [node.target]
            )
        )
    ]
    if len(router_assignments) != 1:
        raise DenominatorError("report router owner is unresolved or duplicated")
    router_value = router_assignments[0].value
    if (
        not isinstance(router_value, ast.Call)
        or _optional_call_name(router_value.func) != "APIRouter"
    ):
        raise DenominatorError("report router is not a direct APIRouter")
    prefixes = [
        keyword.value for keyword in router_value.keywords if keyword.arg == "prefix"
    ]
    if (
        len(prefixes) != 1
        or not isinstance(prefixes[0], ast.Constant)
        or prefixes[0].value != "/operator"
    ):
        raise DenominatorError("report router prefix changed")
    discovered: dict[tuple[str, str, str], dict[str, Any]] = {}
    for candidate in ast.walk(factory):
        if (
            isinstance(candidate, ast.Call)
            and isinstance(candidate.func, ast.Attribute)
            and _optional_call_name(candidate.func.value) == "router"
            and candidate.func.attr in {"add_api_route", "api_route", "include_router"}
        ):
            raise DenominatorError("report router gained imperative or nested registration")
        if not isinstance(candidate, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for decorator in candidate.decorator_list:
            if (
                not isinstance(decorator, ast.Call)
                or not isinstance(decorator.func, ast.Attribute)
                or _optional_call_name(decorator.func.value) != "router"
            ):
                continue
            if (
                decorator.func.attr not in {"get", "post", "put", "patch", "delete"}
                or len(decorator.args) != 1
                or not isinstance(decorator.args[0], ast.Constant)
                or type(decorator.args[0].value) is not str
            ):
                raise DenominatorError(
                    f"report route declaration is unresolved: {relative}:{decorator.lineno}"
                )
            key = (
                decorator.func.attr.upper(),
                str(decorator.args[0].value),
                candidate.name,
            )
            if key in discovered:
                raise DenominatorError(f"duplicate report route declaration: {key}")
            response_models = [
                keyword.value
                for keyword in decorator.keywords
                if keyword.arg == "response_model"
            ]
            if key[1].endswith("/download"):
                if response_models:
                    raise DenominatorError("binary report download gained a JSON response model")
                response_model = "binary_response"
            else:
                if len(response_models) != 1:
                    raise DenominatorError(
                        f"report route lacks one strict response model: {key!r}"
                    )
                response_model = ast.unparse(response_models[0])
            graph = _reachable_call_graph(tree, candidate, source_file=relative)
            if graph["unresolved_target_count"]:
                raise DenominatorError(
                    f"report route has unresolved call targets: "
                    f"route={key!r} targets={graph['unresolved_targets']!r}"
                )
            discovered[key] = {
                "method": key[0],
                "path": "/operator" + key[1],
                "source_file": relative,
                "source_line": int(candidate.lineno),
                "decorator_line": int(decorator.lineno),
                "handler_owner": graph["root_owner"],
                "response_model": response_model,
                "call_graph": graph,
            }
    actual_keys = frozenset(discovered)
    if actual_keys != REPORT_ROUTE_POLICY:
        raise DenominatorError(
            f"report mounted route ownership changed: "
            f"missing={sorted(REPORT_ROUTE_POLICY - actual_keys)!r} "
            f"extra={sorted(actual_keys - REPORT_ROUTE_POLICY)!r}"
        )
    return discovered


def _mounted_command_and_report_rows(
    robot: Path,
    digests: Mapping[tuple[str, str], str],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    command_owners = _verify_command_plane_router_owner(robot)
    for key, owner in sorted(command_owners.items()):
        method, path, _handler = key
        mutating = method != "GET"
        recovery_only = path.startswith("/recovery/")
        rows.append(
            _row(
                family="robot_command_plane_route",
                repository="robot",
                source_file=owner["source_file"],
                source_sha256=digests[("robot", owner["source_file"])],
                source_line=owner["source_line"],
                identifier=f"{method} {owner['path']}",
                final_callable=owner["handler_owner"],
                control_class="machine_composite" if mutating else "projection_only",
                transport_effect="operator_command_plane",
                physical_effect_possible=mutating and not recovery_only,
                requires_durable_claim=mutating,
                dispatch_path=f"command_plane_call_graph:{owner['call_graph']['sha256']}",
                verification_id=f"RA.RW0.ROBOT.COMMAND_ROUTE.{owner['handler_owner']}",
                mutation_gate="operator_command_plane_admission" if mutating else "none",
                correlation_fields=("command_id", "idempotency_key", "ownership_generation"),
                report_projection="operator_command_receipt",
                bms_exposure=None,
                cockpit_control=owner["path"],
            )
        )
    report_owners = _report_route_owners(robot)
    for key, owner in sorted(report_owners.items()):
        method, _path, _handler = key
        rows.append(
            _row(
                family="robot_report_route",
                repository="robot",
                source_file=owner["source_file"],
                source_sha256=digests[("robot", owner["source_file"])],
                source_line=owner["source_line"],
                identifier=f"{method} {owner['path']}",
                final_callable=owner["handler_owner"],
                control_class="projection_only",
                transport_effect="query_only_snapshot" if method == "GET" else "report_export_publication",
                physical_effect_possible=False,
                requires_durable_claim=False,
                dispatch_path=f"report_call_graph:{owner['call_graph']['sha256']}",
                verification_id=f"RA.RW0.ROBOT.REPORT_ROUTE.{owner['handler_owner']}",
                mutation_gate="report_export_receipt" if method == "POST" else "none",
                correlation_fields=("command_id", "pipette_operation_id", "event_sequence"),
                report_projection=owner["response_model"],
                bms_exposure=None,
                cockpit_control=owner["path"],
            )
        )
    return rows


def _mounted_operator_catalog(
    robot: Path,
) -> dict[tuple[str, str, str], tuple[str, int, str, str]]:
    _operator_catalog_exclusions(robot)
    _verify_command_plane_router_owner(robot)
    api_relative = "src/bioxp/api.py"
    api_tree = _ast(_verified_path(robot, api_relative))
    expected_router_imports = {
        "oem_compat_router": (1, "oem_compat.api", "router"),
        "oem_runtime_router": (1, "oem_runtime_api", "router"),
        "oem_homing_router": (1, "oem_homing_routes", "router"),
    }
    discovered_router_imports: dict[str, tuple[int, str, str]] = {}
    for node in api_tree.body:
        if not isinstance(node, ast.ImportFrom):
            continue
        for alias in node.names:
            if alias.asname in expected_router_imports:
                discovered_router_imports[str(alias.asname)] = (int(node.level), str(node.module or ""), str(alias.name))
    if discovered_router_imports != expected_router_imports:
        raise DenominatorError(
            f"mounted router import authority changed: expected={expected_router_imports!r} actual={discovered_router_imports!r}"
        )

    module_mounts = [
        node.value
        for node in api_tree.body
        if isinstance(node, ast.Expr)
        and isinstance(node.value, ast.Call)
        and isinstance(node.value.func, ast.Attribute)
        and isinstance(node.value.func.value, ast.Name)
        and node.value.func.value.id == "app"
        and node.value.func.attr == "include_router"
    ]
    expected_module_mounts = ("oem_compat_router", "oem_runtime_router", "oem_homing_router")
    actual_module_mounts = tuple(
        ast.unparse(call.args[0])
        for call in module_mounts
        if len(call.args) == 1 and not call.keywords
    )
    if len(module_mounts) != 3 or actual_module_mounts != expected_module_mounts:
        raise DenominatorError(
            f"module-scope mounted router graph changed: expected={expected_module_mounts!r} actual={actual_module_mounts!r}"
        )

    lifespan = _function(api_tree, "lifespan")
    report_mount_expression = (
        "create_operator_reports_router(_pipette_receipts, "
        "writer_health_provider=runtime_write_coordinator(_pipette_receipts.root).snapshot)"
    )
    expected_installer_mounts = ("command_plane.router", "router")
    if MOUNTED_ROUTER_CALL_POLICY != {
        *expected_module_mounts,
        *expected_installer_mounts,
        report_mount_expression,
    }:
        raise DenominatorError("mounted router call policy contradicts the exact mount graph")
    report_mounts = [
        node
        for node in ast.walk(lifespan)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "app"
        and node.func.attr == "include_router"
        and len(node.args) == 1
        and not node.keywords
        and ast.unparse(node.args[0]) == report_mount_expression
    ]
    all_api_mounts = [
        node
        for node in ast.walk(api_tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "app"
        and node.func.attr == "include_router"
    ]
    install_calls = [
        node
        for node in ast.walk(lifespan)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "install_operator_control_plane"
    ]
    if (
        len(report_mounts) != 1
        or len(all_api_mounts) != 4
        or len(install_calls) != 1
        or report_mounts[0].lineno >= install_calls[0].lineno
    ):
        raise DenominatorError("operator report router must mount exactly once before operator catalog installation")

    reports_tree = _ast(_verified_path(robot, "src/bioxp/operator_reports.py"))
    reports_factory = _function(reports_tree, "create_operator_reports_router")
    report_router_assignments = [
        node
        for node in ast.walk(reports_factory)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "router" for target in node.targets)
        and isinstance(node.value, ast.Call)
        and _call_name(node.value.func) == "APIRouter"
    ]
    if len(report_router_assignments) != 1:
        raise DenominatorError("operator report router factory authority is unresolved")
    report_router_call = report_router_assignments[0].value
    if not isinstance(report_router_call, ast.Call):
        raise DenominatorError("operator report router factory is not a direct APIRouter call")
    prefix_values = [
        keyword.value
        for keyword in report_router_call.keywords
        if keyword.arg == "prefix"
    ]
    if len(prefix_values) != 1 or not isinstance(prefix_values[0], ast.Constant) or prefix_values[0].value != "/operator":
        raise DenominatorError("operator report router prefix is not the excluded /operator authority")

    controls_tree = _ast(_verified_path(robot, "src/bioxp/operator_controls.py"))
    installer = _function(controls_tree, "install_operator_control_plane")
    expected_catalog_statement = ast.parse("actions, dispatch = _build_catalog(app)").body[0]
    executable_body = list(installer.body)
    if (
        executable_body
        and isinstance(executable_body[0], ast.Expr)
        and isinstance(executable_body[0].value, ast.Constant)
        and type(executable_body[0].value.value) is str
    ):
        executable_body = executable_body[1:]
    if not executable_body or ast.dump(executable_body[0]) != ast.dump(expected_catalog_statement):
        raise DenominatorError("operator catalog snapshot is not the first installer statement")
    installer_mounts = [
        node
        for node in ast.walk(installer)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "app"
        and node.func.attr == "include_router"
    ]
    actual_installer_mounts = tuple(ast.unparse(node.args[0]) for node in sorted(installer_mounts, key=lambda item: item.lineno) if len(node.args) == 1 and not node.keywords)
    if len(installer_mounts) != 2 or actual_installer_mounts != expected_installer_mounts or any(
        node.lineno <= installer.body[0].lineno for node in installer_mounts
    ):
        raise DenominatorError("operator plane routers must mount exactly after the catalog snapshot")

    import_owner_paths = {
        "oem_compat_router": "src/bioxp/oem_compat/api.py",
        "oem_runtime_router": "src/bioxp/oem_runtime_api.py",
        "oem_homing_router": "src/bioxp/oem_homing_routes.py",
    }
    derived_owner_set = {(api_relative, "app")} | {
        (import_owner_paths[name], "router") for name in actual_module_mounts
    }
    if derived_owner_set != set(OPERATOR_CATALOG_OWNER_POLICY):
        raise DenominatorError(
            f"mounted operator owner closure changed: expected={set(OPERATOR_CATALOG_OWNER_POLICY)!r} actual={derived_owner_set!r}"
        )
    discovered: dict[tuple[str, str, str], tuple[str, int, str, str]] = {}
    owner_counts: dict[tuple[str, str], int] = {}
    methods = {"get", "post", "put", "patch", "delete"}
    for relative, owner_name in OPERATOR_CATALOG_OWNER_POLICY:
        tree = _ast(_verified_path(robot, relative))
        for candidate in ast.walk(tree):
            if isinstance(candidate, (ast.Assign, ast.AnnAssign)):
                targets = candidate.targets if isinstance(candidate, ast.Assign) else [candidate.target]
                if (
                    isinstance(candidate.value, ast.Name)
                    and candidate.value.id == owner_name
                    and any(isinstance(target, ast.Name) and target.id != owner_name for target in targets)
                ):
                    raise DenominatorError(f"mounted route owner alias is forbidden: {relative}:{candidate.lineno}")
            if (
                isinstance(candidate, ast.Call)
                and isinstance(candidate.func, ast.Attribute)
                and isinstance(candidate.func.value, ast.Name)
                and candidate.func.value.id == owner_name
                and candidate.func.attr in {"add_api_route", "api_route"}
            ):
                raise DenominatorError(f"imperative or generic route registration is forbidden: {relative}:{candidate.lineno}")
            if (
                relative != api_relative
                and isinstance(candidate, ast.Call)
                and isinstance(candidate.func, ast.Attribute)
                and isinstance(candidate.func.value, ast.Name)
                and candidate.func.value.id == owner_name
                and candidate.func.attr == "include_router"
            ):
                raise DenominatorError(f"nested mounted router graph is forbidden: {relative}:{candidate.lineno}")
        prefixes: list[str] = []
        for node in tree.body:
            targets = node.targets if isinstance(node, ast.Assign) else ([node.target] if isinstance(node, ast.AnnAssign) else [])
            value = node.value if isinstance(node, (ast.Assign, ast.AnnAssign)) else None
            if not any(isinstance(target, ast.Name) and target.id == owner_name for target in targets) or not isinstance(value, ast.Call):
                continue
            constructor = _call_name(value.func)
            if owner_name == "app" and constructor == "FastAPI":
                prefixes.append("")
            elif owner_name == "router" and constructor == "APIRouter":
                prefix_nodes = [keyword.value for keyword in value.keywords if keyword.arg == "prefix"]
                if not prefix_nodes:
                    prefixes.append("")
                elif len(prefix_nodes) == 1 and isinstance(prefix_nodes[0], ast.Constant) and type(prefix_nodes[0].value) is str:
                    prefixes.append(str(prefix_nodes[0].value))
                else:
                    raise DenominatorError(f"router prefix is not one literal string: {relative}")
        if len(prefixes) != 1:
            raise DenominatorError(f"mounted route owner is not uniquely defined: {(relative, owner_name)}")
        prefix = prefixes[0]
        count = 0
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            for decorator in node.decorator_list:
                if not isinstance(decorator, ast.Call) or not isinstance(decorator.func, ast.Attribute):
                    continue
                if not isinstance(decorator.func.value, ast.Name) or decorator.func.value.id != owner_name:
                    continue
                method = decorator.func.attr.lower()
                if method not in methods:
                    continue
                schema_keywords = [keyword.value for keyword in decorator.keywords if keyword.arg == "include_in_schema"]
                if len(schema_keywords) > 1:
                    raise DenominatorError(f"mounted route has duplicate include_in_schema: {relative}:{decorator.lineno}")
                if schema_keywords:
                    schema_node = schema_keywords[0]
                    if not isinstance(schema_node, ast.Constant) or type(schema_node.value) is not bool:
                        raise DenominatorError(f"mounted route include_in_schema is not one literal boolean: {relative}:{decorator.lineno}")
                    if schema_node.value is False:
                        continue
                if len(decorator.args) != 1 or not isinstance(decorator.args[0], ast.Constant) or type(decorator.args[0].value) is not str:
                    raise DenominatorError(f"mounted route path is not one literal string: {relative}:{decorator.lineno}")
                path = prefix + str(decorator.args[0].value)
                if path in OPERATOR_CATALOG_EXCLUSION_POLICY or path.startswith("/operator/"):
                    continue
                operation_keywords = [keyword.value for keyword in decorator.keywords if keyword.arg == "operation_id"]
                if len(operation_keywords) > 1:
                    raise DenominatorError(f"mounted route has duplicate operation_id: {relative}:{decorator.lineno}")
                if operation_keywords:
                    operation_node = operation_keywords[0]
                    if not isinstance(operation_node, ast.Constant) or type(operation_node.value) is not str or not operation_node.value:
                        raise DenominatorError(f"mounted route operation_id is not one literal string: {relative}:{decorator.lineno}")
                    operation_id = str(operation_node.value)
                else:
                    operation_id = _fastapi_default_operation_id(node.name, path, method)
                key = (method.upper(), path, operation_id)
                if key in discovered or any(existing[:2] == key[:2] for existing in discovered):
                    raise DenominatorError(f"duplicate mounted operator catalog route: {key}")
                discovered[key] = (
                    relative,
                    decorator.lineno,
                    node.name,
                    _function_call_graph_sha256(
                        tree,
                        node,
                        source_file=relative,
                    ),
                )
                count += 1
        owner_counts[(relative, owner_name)] = count
    actual_routes = frozenset(discovered)
    if actual_routes != OPERATOR_CATALOG_ROUTE_POLICY:
        missing = sorted(OPERATOR_CATALOG_ROUTE_POLICY - actual_routes)
        extra = sorted(actual_routes - OPERATOR_CATALOG_ROUTE_POLICY)
        raise DenominatorError(
            f"mounted operator catalog route policy changed: missing={missing} extra={extra}"
        )
    if len(discovered) != len(OPERATOR_CATALOG_ROUTE_POLICY):
        raise DenominatorError(
            f"mounted operator catalog does not match its independent closed route inventory: {owner_counts}"
        )
    return discovered


def _robot_route_rows(robot: Path, digests: Mapping[tuple[str, str], str]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    relative = "src/bioxp/api.py"
    path = _verified_path(robot, relative)
    tree = _ast(path)
    discovered: dict[tuple[str, str], tuple[ast.FunctionDef | ast.AsyncFunctionDef, str, int]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for decorator in node.decorator_list:
            if not isinstance(decorator, ast.Call) or not isinstance(decorator.func, ast.Attribute):
                continue
            owner = decorator.func.value
            if not isinstance(owner, ast.Name) or owner.id != "app" or not decorator.args:
                continue
            route_arg = decorator.args[0]
            if not isinstance(route_arg, ast.Constant) or type(route_arg.value) is not str:
                continue
            route = route_arg.value
            if not route.startswith("/liquid"):
                continue
            key = (decorator.func.attr.upper(), route)
            if key in discovered:
                raise DenominatorError(f"duplicate mounted liquid route: {key}")
            discovered[key] = (node, node.name, decorator.lineno)
    if (
        set(ROBOT_ROUTE_CALL_SHA256_POLICY) != set(ROBOT_ROUTE_POLICY)
        or set(ROBOT_ROUTE_DISPATCH_CALL_POLICY) != set(ROBOT_ROUTE_POLICY)
    ):
        raise DenominatorError("liquid route call policy keys do not match the route policy")
    if set(discovered) != set(ROBOT_ROUTE_POLICY):
        missing = sorted(set(ROBOT_ROUTE_POLICY) - set(discovered))
        extra = sorted(set(discovered) - set(ROBOT_ROUTE_POLICY))
        raise DenominatorError(f"mounted liquid route inventory changed: missing={missing} extra={extra}")

    route_rows: list[dict[str, Any]] = []
    for (method, route), (node, function_name, route_line) in sorted(discovered.items()):
        control_class, claim, physical, effect = ROBOT_ROUTE_POLICY[(method, route)]
        dispatch = _dispatch_path(
            node,
            route_key=(method, route),
            no_effect=control_class == "projection_only",
            plan_only=effect == "plan",
        )
        route_rows.append(
            _row(
                family="robot_route", repository="robot", source_file=relative,
                source_sha256=digests[("robot", relative)], source_line=route_line,
                identifier=f"{method} {route}", final_callable=function_name,
                control_class=control_class, transport_effect=effect,
                physical_effect_possible=physical, requires_durable_claim=claim,
                dispatch_path=dispatch, verification_id=f"RA.RW0.ROUTE.{method}.{route}",
                mutation_gate="none" if not claim else "robot_durable_claim",
                bms_exposure="DEFAULT_ROBOT_ROUTES" if route in {value[1] for value in _bms_route_mapping(bms=None, source_path=None).values()} else "operator_catalog",
                cockpit_control="catalog_generated",
            )
        )

    catalog_rows: list[dict[str, Any]] = []
    for (method, route, operation_id), (
        owner_relative,
        route_line,
        function_name,
        call_graph_sha256,
    ) in sorted(_mounted_operator_catalog(robot).items()):
        action_id = _path_action_id(method, route, operation_id)
        is_query = method == "GET"
        if (method, route) in OPERATOR_CATALOG_POLICY:
            (
                policy_callable,
                catalog_control_class,
                catalog_claim,
                catalog_physical,
                catalog_effect,
            ) = OPERATOR_CATALOG_POLICY[(method, route)]
            if function_name != policy_callable:
                raise DenominatorError(
                    f"pipette catalog callable changed for {(method, route)}: {function_name}"
                )
        else:
            catalog_control_class = "hardware_query" if is_query else "machine_composite"
            catalog_claim = not is_query
            catalog_physical = not is_query
            catalog_effect = "query" if is_query else "route_dispatch"
        catalog_rows.append(
            _row(
                family="operator_catalog_dispatch", repository="robot", source_file=owner_relative,
                source_sha256=digests[("robot", owner_relative)], source_line=route_line,
                identifier=action_id, final_callable=function_name,
                control_class=catalog_control_class,
                transport_effect=catalog_effect,
                physical_effect_possible=catalog_physical, requires_durable_claim=catalog_claim,
                dispatch_path=f"operator_catalog_call_graph:{call_graph_sha256}",
                verification_id=f"RA.RW0.CATALOG.{action_id}",
                mutation_gate="none" if not catalog_claim else "robot_durable_claim",
                bms_exposure={"method": method, "path": route, "operation_id": operation_id},
                cockpit_control="catalog_generated",
            )
        )
    return route_rows, catalog_rows


def _path_action_id(method: str, path: str, operation_id: str | None) -> str:
    base = operation_id or f"{method}_{path}"
    slug = re.sub(r"[^a-z0-9_.-]+", "_", base.lower()).strip("_.-")
    digest = hashlib.sha256(f"{method} {path}".encode("utf-8")).hexdigest()[:8]
    return f"route.{slug[:96]}.{digest}"


def _protocol_rows(robot: Path, digests: Mapping[tuple[str, str], str]) -> list[dict[str, Any]]:
    relative = "src/bioxp/api.py"
    path = _verified_path(robot, relative)
    tree = _ast(path)
    registry = _function(tree, "_protocol_live_handlers")
    returns = [node for node in ast.walk(registry) if isinstance(node, ast.Return)]
    if len(returns) != 1 or not isinstance(returns[0].value, ast.Dict):
        raise DenominatorError("protocol live handler registry must return exactly one literal dictionary")
    mapping_node = returns[0].value
    mappings: dict[str, tuple[str, int]] = {}
    for key, value in zip(mapping_node.keys, mapping_node.values):
        if key is None or value is None:
            raise DenominatorError("protocol live handler registry contains an unpacked entry")
        if not (
            isinstance(key, ast.Attribute)
            and isinstance(key.value, ast.Name)
            and key.value.id == "ProtocolActionKind"
            and isinstance(value, ast.Name)
        ):
            raise DenominatorError("protocol live handler registry contains a malformed key or handler")
        kind = key.attr
        if kind in mappings:
            raise DenominatorError(f"duplicate protocol live handler registry entry: {kind}")
        mappings[kind] = (value.id, key.lineno)
    discovered_handlers = {kind: handler for kind, (handler, _line) in mappings.items()}
    if discovered_handlers != dict(PROTOCOL_HANDLER_POLICY):
        raise DenominatorError(
            "complete protocol live handler registry changed: "
            f"expected={dict(PROTOCOL_HANDLER_POLICY)!r} actual={discovered_handlers!r}"
        )
    for handler_name in set(discovered_handlers.values()):
        _function(tree, handler_name)

    handler = _function(tree, "_protocol_live_pipette_handler")
    dispatch = _dispatch_path(handler)
    rows: list[dict[str, Any]] = []
    for kind, policy in sorted(PROTOCOL_POLICY.items()):
        callable_name, line = mappings[kind]
        (
            policy_callable,
            control_class,
            physical,
            effect,
            claim,
            mutation_gate,
        ) = policy
        if callable_name != policy_callable:
            raise DenominatorError(f"pipette protocol handler classification changed for {kind}")
        rows.append(
            _row(
                family="protocol_registry",
                repository="robot",
                source_file=relative,
                source_sha256=digests[("robot", relative)],
                source_line=line,
                identifier=f"ProtocolActionKind.{kind}",
                final_callable=policy_callable,
                control_class=control_class,
                transport_effect=effect,
                physical_effect_possible=physical,
                requires_durable_claim=claim,
                dispatch_path=dispatch,
                verification_id=f"RA.RW0.PROTOCOL.{kind}",
                mutation_gate=mutation_gate,
                correlation_fields=(*_REQUIRED_CORRELATION_FIELDS, "protocol_job_id", "protocol_action_id"),
            )
        )
    return rows


def _lifecycle_rows(robot: Path, digests: Mapping[tuple[str, str], str]) -> list[dict[str, Any]]:
    trees: dict[str, ast.Module] = {}
    rows: list[dict[str, Any]] = []
    for relative, function_name, control_class, claim, physical in LIFECYCLE_POLICY:
        tree = trees.setdefault(relative, _ast(_verified_path(robot, relative)))
        function = _function(tree, function_name)
        dispatch = _dispatch_path(function)
        rows.append(
            _row(
                family="lifecycle_registration",
                repository="robot",
                source_file=relative,
                source_sha256=digests[("robot", relative)],
                source_line=function.lineno,
                identifier=function_name,
                final_callable=function_name,
                control_class=control_class,
                transport_effect="query_or_command",
                physical_effect_possible=physical,
                requires_durable_claim=claim,
                dispatch_path=dispatch,
                verification_id=f"RA.RW0.LIFECYCLE.{function_name}",
                mutation_gate="robot_lifecycle_stage_and_durable_claim",
                correlation_fields=(*_REQUIRED_CORRELATION_FIELDS, "lifecycle_stage_id"),
            )
        )
    return rows


def _is_callback_method_name(name: str) -> bool:
    lowered = name.lower()
    return (
        lowered == "_dispatch"
        or lowered.endswith(("_callback", "_receive_loop"))
        or (lowered.startswith("process_") and lowered.endswith("_message"))
        or (lowered.startswith("_record_") and lowered.endswith("_error"))
    )


def _method_control_class(name: str) -> tuple[str, bool, str]:
    lowered = name.lower()
    if _is_callback_method_name(name):
        return "callback_event", False, "callback"
    if any(token in lowered for token in ("terminate", "abort", "close", "shutdown")):
        return "interrupt", True, "interrupt"
    if any(token in lowered for token in ("aspirat", "dispens", "mix", "eject", "fluid_detection")):
        return "physical_liquid_command", True, "command"
    if any(token in lowered for token in ("query", "read", "status", "data", "pressure", "firmware", "diagnos", "error_log", "condition", "timestamp")):
        return "hardware_query", False, "query"
    return "pipette_state_command", True, "state_command"


def _optional_call_name(node: ast.AST) -> str | None:
    binding = _bind_call_target(node)
    targets = binding["resolved_target_identities"]
    if binding["unresolved_reason"] is not None or not targets:
        return None
    if len(targets) == 1:
        return str(targets[0])
    return "finite{" + "|".join(str(target) for target in targets) + "}"


def _callback_wiring_event(node: ast.AST) -> str | None:
    callback_attributes = {"_error_callback", "_pipette_error_callback"}
    if isinstance(node, (ast.Assign, ast.AnnAssign)):
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if any(
            isinstance(target, ast.Attribute) and target.attr in callback_attributes
            for target in targets
        ):
            return ast.dump(node)
    if not isinstance(node, ast.Call):
        return None
    name = _optional_call_name(node.func)
    if name in {
        "NovoRouter",
        "threading.Thread",
        "self.novo_router.start",
        "self._dispatch",
        "self.process_pipette_message",
        "callback",
        "self._error_callback",
        "self._pipette_error_callback",
    }:
        return ast.dump(node)
    if name in {"getattr", "setattr"} and len(node.args) >= 2:
        attribute = node.args[1]
        if isinstance(attribute, ast.Constant) and attribute.value in callback_attributes:
            return ast.dump(node)
    return None


def _callback_symbol_names(node: ast.AST) -> set[str]:
    symbols: set[str] = set()
    for candidate in ast.walk(node):
        if isinstance(candidate, ast.Name):
            symbols.add(candidate.id)
        elif isinstance(candidate, ast.Attribute):
            name = _optional_call_name(candidate)
            if name is not None:
                symbols.add(name)
                symbols.add(candidate.attr)
        elif isinstance(candidate, ast.Constant) and isinstance(candidate.value, str):
            if candidate.value.startswith("_"):
                symbols.add(candidate.value)
    return symbols


def _callback_tainted_symbols(
    trees: Mapping[str, ast.Module],
    expected_statements: Sequence[ast.stmt],
) -> set[str]:
    tainted = {
        method_name
        for _relative, _class_name, method_name in CALLBACK_POLICY
    }
    tainted.update(_PRIVATE_CALLBACK_IDENTITIES)
    for statement in expected_statements:
        tainted.update(
            symbol
            for symbol in _callback_symbol_names(statement)
            if "callback" in symbol.lower()
            or symbol.split(".")[-1] in tainted
        )
    discovered_callable_parameters: set[tuple[str, str, str, str]] = set()
    for relative, tree in trees.items():
        for owner in (node for node in tree.body if isinstance(node, ast.ClassDef)):
            for function in (
                node
                for node in owner.body
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            ):
                for argument in (
                    *function.args.posonlyargs,
                    *function.args.args,
                    *function.args.kwonlyargs,
                ):
                    if argument.annotation is not None and "Callable" in ast.unparse(argument.annotation):
                        discovered_callable_parameters.add(
                            (relative, owner.name, function.name, argument.arg)
                        )
    expected_callable_parameters = (
        CALLBACK_CALLABLE_PARAMETER_POLICY
        | CALLBACK_CALLABLE_PARAMETER_EXCLUSION_POLICY
    )
    if discovered_callable_parameters != expected_callable_parameters:
        raise DenominatorError(
            "callback callable-parameter inventory changed: "
            f"expected={sorted(expected_callable_parameters)!r} "
            f"actual={sorted(discovered_callable_parameters)!r}"
        )
    tainted.update(parameter for *_owner, parameter in CALLBACK_CALLABLE_PARAMETER_POLICY)

    return tainted


def _callback_statement_event(
    statement: ast.stmt,
    tainted: set[str],
) -> str | None:
    if not isinstance(statement, (ast.Assign, ast.AnnAssign, ast.Expr, ast.Return)):
        return None
    if isinstance(statement, ast.AnnAssign) and statement.value is None:
        return None
    if any(_callback_wiring_event(node) is not None for node in ast.walk(statement)):
        return ast.dump(statement)
    if _callback_symbol_names(statement) & tainted:
        return ast.dump(statement)
    return None


def _verify_callback_wiring(
    trees: Mapping[str, ast.Module],
) -> None:
    parsed_expected: list[tuple[str, ast.stmt]] = []
    for key, statements in CALLBACK_WIRING_STATEMENT_POLICY.items():
        relative, class_name, method_name = key
        owner = _class(trees[relative], class_name) if class_name else trees[relative]
        methods = [
            node
            for node in owner.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == method_name
        ]
        if len(methods) != 1:
            raise DenominatorError(f"callback wiring owner is unresolved or ambiguous: {key}")
        method = methods[0]
        actual_statements = Counter(
            ast.dump(node)
            for node in ast.walk(method)
            if isinstance(node, (ast.Assign, ast.AnnAssign, ast.Expr, ast.Return))
        )
        for source in statements:
            parsed = ast.parse(source).body
            if len(parsed) != 1:
                raise DenominatorError(f"callback wiring policy is not one statement: {key}")
            expected_statement = parsed[0]
            expected_dump = ast.dump(expected_statement)
            if actual_statements[expected_dump] != 1:
                raise DenominatorError(f"callback wiring statement changed or is duplicated: {key}: {source}")
            parsed_expected.append((relative, expected_statement))

    tainted = _callback_tainted_symbols(
        trees,
        [statement for _relative, statement in parsed_expected],
    )
    expected_events: Counter[tuple[str, str]] = Counter()
    for relative, statement in parsed_expected:
        for node in ast.walk(statement):
            if not isinstance(node, ast.stmt):
                continue
            event = _callback_statement_event(node, tainted)
            if event is not None:
                expected_events[(relative, event)] += 1
    actual_events: Counter[tuple[str, str]] = Counter()
    for relative, tree in trees.items():
        for node in ast.walk(tree):
            if not isinstance(node, ast.stmt):
                continue
            event = _callback_statement_event(node, tainted)
            if event is not None:
                actual_events[(relative, event)] += 1
    if actual_events != expected_events:
        missing = sorted((key, count - actual_events[key]) for key, count in expected_events.items() if actual_events[key] < count)
        extra = sorted((key, count - expected_events[key]) for key, count in actual_events.items() if expected_events[key] < count)
        raise DenominatorError(f"callback registration/call-site closure changed: missing={missing!r} extra={extra!r}")


def _verify_callback_dataflow(
    trees: Mapping[str, ast.Module],
) -> dict[tuple[str, str, str], str]:
    if set(CALLBACK_DATAFLOW_POLICY) != set(CALLBACK_POLICY):
        raise DenominatorError("callback dataflow policy does not cover every callback owner")
    digests: dict[tuple[str, str, str], str] = {}
    for key, edges in CALLBACK_DATAFLOW_POLICY.items():
        relative, class_name, method_name = key
        method = _function(_class(trees[relative], class_name), method_name)
        actual_statements = Counter(
            ast.dump(statement)
            for statement in ast.walk(method)
            if isinstance(statement, ast.stmt)
        )
        parameters = {
            argument.arg
            for argument in (
                *method.args.posonlyargs,
                *method.args.args,
                *method.args.kwonlyargs,
            )
            if argument.arg != "self"
        }
        rebound_parameters = {
            ast.unparse(target)
            for statement in ast.walk(method)
            if isinstance(statement, (ast.Assign, ast.AnnAssign, ast.AugAssign))
            for target in (
                statement.targets
                if isinstance(statement, ast.Assign)
                else (statement.target,)
            )
            if isinstance(target, ast.Name) and target.id in parameters
        }
        if rebound_parameters:
            raise DenominatorError(
                f"callback dataflow source parameters are rebound for {key}: {sorted(rebound_parameters)!r}"
            )
        for statement_source, source_symbols, expected_count in edges:
            parsed = ast.parse(statement_source).body
            if len(parsed) != 1:
                raise DenominatorError(f"callback dataflow policy statement is invalid for {key}")
            statement = parsed[0]
            statement_dump = ast.dump(statement)
            if actual_statements[statement_dump] != expected_count:
                raise DenominatorError(
                    f"callback dataflow edge count changed for {key}: "
                    f"statement={statement_source!r} expected={expected_count} "
                    f"actual={actual_statements[statement_dump]}"
                )
            loaded_symbols = {
                node.id
                for node in ast.walk(statement)
                if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
            } | {
                ast.unparse(node)
                for node in ast.walk(statement)
                if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Load)
            }
            missing_symbols = set(source_symbols) - loaded_symbols
            if missing_symbols:
                raise DenominatorError(
                    f"callback dataflow source symbols are absent for {key}: "
                    f"statement={statement_source!r} missing={sorted(missing_symbols)!r}"
                )
        digests[key] = _sha256_bytes(_canonical_bytes(edges))
    return digests


def _transport_rows(robot: Path, digests: Mapping[tuple[str, str], str]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    discovered_methods: dict[
        tuple[str, str, str],
        tuple[ast.FunctionDef | ast.AsyncFunctionDef, tuple[str, str, bool, str, bool]],
    ] = {}
    transport_owners = sorted(
        {(relative, class_name) for relative, class_name, _method in TRANSPORT_ENTRYPOINT_POLICY}
    )
    for relative, class_name in transport_owners:
        tree = _ast(_verified_path(robot, relative))
        owner = _class(tree, class_name)
        methods = [
            node
            for node in owner.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and not node.name.startswith("_")
        ]
        if not methods:
            raise DenominatorError(f"transport owner class has no entrypoints: {class_name}")
        for method in methods:
            control_class, physical, effect = _method_control_class(method.name)
            identifier = f"{class_name}.{method.name}"
            discovered_methods[(relative, class_name, method.name)] = (
                method,
                (identifier, control_class, physical, effect, False),
            )
    discovered_transport_policy = {
        key: values for key, (_method, values) in discovered_methods.items()
    }
    if discovered_transport_policy != dict(TRANSPORT_ENTRYPOINT_POLICY):
        raise DenominatorError(
            "transport owner entrypoint policy changed: "
            f"expected={dict(TRANSPORT_ENTRYPOINT_POLICY)!r} actual={discovered_transport_policy!r}"
        )
    for key, policy in sorted(TRANSPORT_ENTRYPOINT_POLICY.items()):
        relative, _class_name, _method_name = key
        method = discovered_methods[key][0]
        identifier, control_class, physical, effect, claim = policy
        rows.append(
            _row(
                family="transport_owner_entrypoint",
                repository="robot",
                source_file=relative,
                source_sha256=digests[("robot", relative)],
                source_line=method.lineno,
                identifier=identifier,
                final_callable=identifier,
                control_class=control_class,
                transport_effect=effect,
                physical_effect_possible=physical,
                requires_durable_claim=claim,
                dispatch_path="transport_owner",
                verification_id=f"RA.RW0.TRANSPORT.{identifier}",
                mutation_gate="caller_must_hold_robot_durable_claim",
                correlation_fields=("command_id", "pipette_operation_id", "callback_session_id"),
            )
        )

    discovered_callbacks: dict[
        tuple[str, str, str],
        tuple[ast.FunctionDef | ast.AsyncFunctionDef, tuple[str, str, bool, str, bool]],
    ] = {}
    callback_owners = tuple(CALLBACK_OWNER_POLICY)
    if len(set(callback_owners)) != len(callback_owners):
        raise DenominatorError("callback owner policy contains duplicates")
    policy_owner_set = {(relative, owner) for relative, owner, _method in CALLBACK_POLICY}
    if policy_owner_set != set(callback_owners):
        raise DenominatorError(
            f"callback owner closure changed: expected={set(callback_owners)!r} actual={policy_owner_set!r}"
        )
    trees = {
        relative: _ast(_verified_path(robot, relative))
        for relative in CALLBACK_SOURCE_PATHS
    }
    discovered_callback_candidates: set[tuple[str, str, str]] = set()
    for relative, tree in trees.items():
        for owner in (node for node in tree.body if isinstance(node, ast.ClassDef)):
            for method in owner.body:
                if (
                    isinstance(method, (ast.FunctionDef, ast.AsyncFunctionDef))
                    and _is_callback_method_name(method.name)
                ):
                    discovered_callback_candidates.add((relative, owner.name, method.name))
    if discovered_callback_candidates != set(CALLBACK_POLICY):
        raise DenominatorError(
            "callback candidate inventory changed: "
            f"expected={sorted(CALLBACK_POLICY)!r} actual={sorted(discovered_callback_candidates)!r}"
        )
    for key, policy in sorted(CALLBACK_POLICY.items()):
        relative, class_name, method_name = key
        tree = trees[relative]
        owner = _class(tree, class_name)
        matches = [
            node for node in owner.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == method_name
        ]
        if len(matches) != 1:
            raise DenominatorError(f"callback policy target is unresolved or ambiguous: {key}")
        method = matches[0]
        identifier = f"{class_name}.{method_name}"
        expected_policy = (identifier, "callback_event", False, "callback", False)
        if policy != expected_policy:
            raise DenominatorError(
                f"callback policy classification contradicts exact handler identity: {key}"
            )
        discovered_callbacks[key] = (method, policy)
    if set(discovered_callbacks) != set(CALLBACK_POLICY):
        raise DenominatorError("callback entrypoint policy closure is incomplete")
    for key, (method, _policy) in discovered_callbacks.items():
        expected_signature = CALLBACK_SIGNATURE_POLICY.get(key)
        actual_signature = (
            tuple(argument.arg for argument in (*method.args.posonlyargs, *method.args.args)),
            tuple(argument.arg for argument in method.args.kwonlyargs),
        )
        if expected_signature is None or actual_signature != expected_signature:
            raise DenominatorError(
                f"callback input mode/signature changed for {key}: expected={expected_signature!r} actual={actual_signature!r}"
            )
        actual_defaults = (
            tuple(ast.unparse(value) for value in method.args.defaults),
            tuple(ast.unparse(value) if value is not None else None for value in method.args.kw_defaults),
        )
        if (
            method.args.vararg is not None
            or method.args.kwarg is not None
            or actual_defaults != CALLBACK_DEFAULT_POLICY.get(key)
        ):
            raise DenominatorError(
                f"callback defaults/variadic input mode changed for {key}: "
                f"expected={CALLBACK_DEFAULT_POLICY.get(key)!r} actual={actual_defaults!r}"
            )
        declaration_source = CALLBACK_DECLARATION_POLICY.get(key)
        expected_declaration_body = ast.parse(declaration_source).body if declaration_source is not None else []
        if len(expected_declaration_body) != 1 or not isinstance(
            expected_declaration_body[0], (ast.FunctionDef, ast.AsyncFunctionDef)
        ):
            raise DenominatorError(f"callback declaration policy is missing or malformed: {key}")
        expected_declaration = expected_declaration_body[0]
        actual_return = ast.dump(method.returns) if method.returns is not None else None
        expected_return = ast.dump(expected_declaration.returns) if expected_declaration.returns is not None else None
        if (
            type(method) is not type(expected_declaration)
            or ast.dump(method.args) != ast.dump(expected_declaration.args)
            or actual_return != expected_return
        ):
            raise DenominatorError(f"callback asyncness, annotations, or complete signature changed: {key}")
        parameter_names = set(actual_signature[0][1:]) | set(actual_signature[1])
        used_names = {
            node.id
            for node in ast.walk(method)
            if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
        }
        unused_parameters = sorted(parameter_names - used_names)
        if unused_parameters:
            raise DenominatorError(f"callback parameters are not consumed by the bound handler {key}: {unused_parameters}")
    _verify_callback_wiring(trees)
    callback_dataflow_sha256 = _verify_callback_dataflow(trees)
    for key, policy in sorted(CALLBACK_POLICY.items()):
        relative, _class_name, _method_name = key
        method = discovered_callbacks[key][0]
        identifier, control_class, physical, effect, claim = policy
        callback_call_graph_sha256 = _function_call_graph_sha256(
            trees[relative],
            method,
            source_file=relative,
        )
        if any(row["public_method_or_path"] == identifier for row in rows):
            continue
        rows.append(
            _row(
                family="callback_registration",
                repository="robot",
                source_file=relative,
                source_sha256=digests[("robot", relative)],
                source_line=method.lineno,
                identifier=identifier,
                final_callable=identifier,
                control_class=control_class,
                transport_effect=effect,
                physical_effect_possible=physical,
                requires_durable_claim=claim,
                dispatch_path=(
                    f"callback_call_graph:{callback_call_graph_sha256}:"
                    f"dataflow:{callback_dataflow_sha256[key]}"
                ),
                verification_id=f"RA.RW0.CALLBACK.{identifier}",
                mutation_gate="bound_callback_session",
                correlation_fields=("command_id", "pipette_operation_id", "callback_session_id"),
            )
        )
    return rows


def _literal(node: ast.AST) -> Any:
    try:
        return ast.literal_eval(node)
    except (ValueError, TypeError, SyntaxError) as exc:
        raise DenominatorError("authority mapping must remain a literal closed-world value") from exc


def _bms_route_mapping(bms: Path | None, source_path: Path | None) -> dict[str, tuple[str, str, float, int]]:
    if bms is None and source_path is None:
        # Used only for a cheap exposure label while robot rows are being built.
        return {
            "pipette_readback": ("POST", "/liquid/readback", 120.0, 0),
            "pipette_application_status": ("GET", "/liquid/application/status", 10.0, 0),
            "pipette_application_plan": ("POST", "/liquid/application/plan", 10.0, 0),
        }
    path = (
        source_path
        if source_path is not None
        else _verified_path(bms, _BMS_ROBOT_CLIENT_PATH)  # type: ignore[arg-type]
    )
    tree = _ast(path)
    assignments: list[ast.AST] = []
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == "DEFAULT_ROBOT_ROUTES":
            if node.value is not None:
                assignments.append(node.value)
        elif isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "DEFAULT_ROBOT_ROUTES" for target in node.targets):
            assignments.append(node.value)
    if len(assignments) != 1 or assignments[0] is None:
        raise DenominatorError("DEFAULT_ROBOT_ROUTES must have exactly one literal mapping authority")
    raw = _literal(assignments[0])
    if type(raw) is not dict or not raw:
        raise DenominatorError("DEFAULT_ROBOT_ROUTES must be a non-empty literal mapping")
    mapping_node = assignments[0]
    if not isinstance(mapping_node, ast.Dict):
        raise DenominatorError("DEFAULT_ROBOT_ROUTES must remain an exact literal dictionary")
    result: dict[str, tuple[str, str, float, int]] = {}
    for key_node, value_node, (key, value) in zip(mapping_node.keys, mapping_node.values, raw.items()):
        if key_node is None or value_node is None:
            raise DenominatorError("DEFAULT_ROBOT_ROUTES contains an unpacked dictionary entry")
        if type(key) is not str or not isinstance(value, tuple) or len(value) != 3:
            raise DenominatorError("DEFAULT_ROBOT_ROUTES contains a malformed row")
        method, route, timeout = value
        if type(method) is not str or type(route) is not str or type(timeout) not in {int, float}:
            raise DenominatorError("DEFAULT_ROBOT_ROUTES contains a malformed typed row")
        result[key] = (method, route, float(timeout), key_node.lineno)
    return result


def _bms_rows(bms: Path, digests: Mapping[tuple[str, str], str]) -> list[dict[str, Any]]:
    relative = _BMS_ROBOT_CLIENT_PATH
    path = _verified_path(bms, relative)
    mapping = _bms_route_mapping(bms, path)
    discovered_relay_policy = {
        key: (
            method,
            route,
            timeout,
            f"BioXpRobotClient.request:{key}",
            "read_only_relay" if method == "GET" else "mutation_relay",
            f"{method} {route}",
            route.startswith("/liquid") and method != "GET",
            False,
        )
        for key, (method, route, timeout, _line) in mapping.items()
    }
    if discovered_relay_policy != dict(BMS_RELAY_POLICY):
        raise DenominatorError(
            "BMS relay policy changed: "
            f"expected={dict(BMS_RELAY_POLICY)!r} actual={discovered_relay_policy!r}"
        )
    rows: list[dict[str, Any]] = []
    for key, policy in sorted(BMS_RELAY_POLICY.items()):
        method, route, _timeout, final_callable, control_class, effect, physical, claim = policy
        source_line = mapping[key][3]
        rows.append(
            _row(
                family="bms_default_robot_route",
                repository="bms",
                source_file=relative,
                source_sha256=digests[("bms", relative)],
                source_line=source_line,
                identifier=key,
                final_callable=final_callable,
                control_class=control_class,
                transport_effect=effect,
                physical_effect_possible=physical,
                requires_durable_claim=claim,
                dispatch_path="generation_bound_robot_client",
                verification_id=f"RA.RW0.BMS.ROUTE.{key}",
                mutation_gate="BioXpConnectionService.active_request_lease",
                correlation_fields=("expected_connection_generation",),
                report_projection="bms_typed_relay",
                bms_exposure=key,
                cockpit_control="typed_client_definition",
            )
        )

    generation_relative = "platform/api/services/bioxp/connection.py"
    generation_tree = _ast(_verified_path(bms, generation_relative))
    owner = _class(generation_tree, "BioXpConnectionService")
    generation_methods: list[ast.FunctionDef | ast.AsyncFunctionDef] = []
    for method in owner.body:
        if not isinstance(method, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        owns_generation = any(
            (
                isinstance(node, ast.AugAssign)
                and isinstance(node.target, ast.Attribute)
                and node.target.attr == "_generation"
            )
            or (
                isinstance(node, (ast.Assign, ast.AnnAssign))
                and any(
                    isinstance(target, ast.Attribute) and target.attr == "_generation"
                    for target in (
                        node.targets
                        if isinstance(node, ast.Assign)
                        else ([node.target] if node.target is not None else [])
                    )
                )
            )
            for node in ast.walk(method)
        )
        if owns_generation:
            generation_methods.append(method)
    if not generation_methods:
        raise DenominatorError("BioXpConnectionService no longer exposes a generation owner")
    for method in generation_methods:
        identifier = f"BioXpConnectionService.{method.name}"
        rows.append(
            _row(
                family="bms_generation_owner",
                repository="bms",
                source_file=generation_relative,
                source_sha256=digests[("bms", generation_relative)],
                source_line=method.lineno,
                identifier=identifier,
                final_callable=identifier,
                control_class="read_only_relay",
                transport_effect="connection_generation_ownership",
                physical_effect_possible=False,
                requires_durable_claim=False,
                dispatch_path="generation_owner",
                verification_id=f"RA.RW0.BMS.GENERATION.{method.name}",
                mutation_gate="BioXpConnectionService._transition_lock",
                correlation_fields=("expected_connection_generation",),
                report_projection="bms_connection_snapshot",
                bms_exposure="connection_generation",
                cockpit_control="connection_generation",
            )
        )
    return rows


class _ScopeStringVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.values: list[ast.Constant] = []

    def visit_Constant(self, node: ast.Constant) -> None:
        if isinstance(node.value, str):
            self.values.append(node)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        return

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        return

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        return

    def visit_Lambda(self, node: ast.Lambda) -> None:
        return


def _scope_string_literals(
    function: ast.FunctionDef | ast.AsyncFunctionDef,
) -> tuple[ast.Constant, ...]:
    visitor = _ScopeStringVisitor()
    for statement in function.body:
        visitor.visit(statement)
    return tuple(
        sorted(
            visitor.values,
            key=lambda value: (int(value.lineno), int(value.col_offset)),
        )
    )


def _bms_remote_call_target(target: str) -> bool:
    return target.startswith("runtime.connection.request_active") or target in {
        "client.request",
        "client.stream_camera_mjpeg",
        "_robot_request",
        "_leased_robot_request",
        "_leased_camera_call",
        "_proxy_operator_report",
        "_proxy_operator_report_model",
        "_robot_request_body",
    }


def _bms_backend_handler_closure(
    tree: ast.Module,
    root: ast.FunctionDef | ast.AsyncFunctionDef,
    *,
    source_file: str,
) -> dict[str, Any]:
    graph = _reachable_call_graph(tree, root, source_file=source_file)
    if graph["unresolved_target_count"]:
        raise DenominatorError(
            f"BMS backend route has unresolved call targets: "
            f"source={source_file} owner={graph['root_owner']} "
            f"targets={graph['unresolved_targets']!r}"
        )
    _owners, functions, _classes = _definition_inventory(tree)
    relay_occurrences: list[dict[str, Any]] = []
    relay_occurrence = 0
    for owner in graph["visited_owner_identities"]:
        for literal in _scope_string_literals(functions[str(owner)]):
            if literal.value not in BMS_RELAY_POLICY:
                continue
            relay_occurrence += 1
            relay_occurrences.append(
                {
                    "relay_name": literal.value,
                    "owner_qualname": owner,
                    "source_line": int(literal.lineno),
                    "occurrence": relay_occurrence,
                    "source_kind": "literal_policy_key",
                }
            )
    remote_calls: list[dict[str, Any]] = []
    for edge in graph["edges"]:
        targets = [str(target) for target in edge["resolved_target_identities"]]
        direct_remote_targets = [target for target in targets if _bms_remote_call_target(target)]
        if edge["resolution_kind"] == "transparent_cast" and relay_occurrences:
            direct_remote_targets.append(f"transparent_cast:{edge['expression']}")
        if direct_remote_targets:
            remote_calls.append(
                {
                    "caller_qualname": edge["caller_qualname"],
                    "source_line": edge["source_line"],
                    "source_column": edge["source_column"],
                    "occurrence": edge["occurrence"],
                    "resolution_kind": edge["resolution_kind"],
                    "targets": direct_remote_targets,
                }
            )
        for target in targets:
            direct_relay = BMS_DIRECT_CLIENT_METHOD_POLICY.get(target)
            if direct_relay is not None:
                relay_occurrence += 1
                relay_occurrences.append(
                    {
                        "relay_name": direct_relay,
                        "owner_qualname": edge["caller_qualname"],
                        "source_line": edge["source_line"],
                        "occurrence": relay_occurrence,
                        "source_kind": "direct_client_method",
                    }
                )
    if remote_calls and not relay_occurrences:
        raise DenominatorError(
            f"BMS backend route reaches a robot client without a relay policy key: "
            f"{graph['root_owner']}:{remote_calls!r}"
        )
    if not remote_calls:
        relay_occurrences.clear()
    closure_body = {
        "handler_owner": graph["root_owner"],
        "source_file": source_file,
        "call_graph": graph,
        "relay_occurrences": tuple(relay_occurrences),
        "relay_occurrence_count": len(relay_occurrences),
        "remote_calls": tuple(remote_calls),
        "remote_call_count": len(remote_calls),
    }
    return {
        **closure_body,
        "sha256": _sha256_bytes(_canonical_bytes(closure_body)),
    }


def _depends_bindings(
    nodes: Iterable[ast.AST],
    *,
    source_file: str,
    owner_identity: str,
    layer: str,
) -> tuple[dict[str, Any], ...]:
    rows: list[dict[str, Any]] = []
    occurrence = 0
    for root in nodes:
        for candidate in sorted(
            (
                node
                for node in ast.walk(root)
                if isinstance(node, ast.Call)
                and _optional_call_name(node.func) is not None
                and str(_optional_call_name(node.func)).rsplit(".", 1)[-1] == "Depends"
            ),
            key=lambda node: (int(node.lineno), int(node.col_offset)),
        ):
            occurrence += 1
            if len(candidate.args) != 1:
                raise DenominatorError(
                    f"BMS dependency owner is not one explicit callable: "
                    f"{source_file}:{candidate.lineno}:{ast.unparse(candidate)}"
                )
            binding = _bind_call_target(candidate.args[0])
            if (
                binding["unresolved_reason"] is not None
                or len(binding["resolved_target_identities"]) != 1
            ):
                raise DenominatorError(
                    f"BMS dependency target is unresolved or non-singleton: "
                    f"{source_file}:{candidate.lineno}:{binding!r}"
                )
            rows.append(
                {
                    "source_file": source_file,
                    "owner_identity": owner_identity,
                    "layer": layer,
                    "source_line": int(candidate.lineno),
                    "occurrence": occurrence,
                    "dependency_target": str(binding["resolved_target_identities"][0]),
                }
            )
    return tuple(rows)


def _handler_dependency_nodes(
    handler: ast.FunctionDef | ast.AsyncFunctionDef,
) -> tuple[ast.AST, ...]:
    nodes: list[ast.AST] = []
    nodes.extend(handler.args.defaults)
    nodes.extend(default for default in handler.args.kw_defaults if default is not None)
    for argument in (
        *handler.args.posonlyargs,
        *handler.args.args,
        *handler.args.kwonlyargs,
    ):
        if argument.annotation is not None:
            nodes.append(argument.annotation)
    if handler.args.vararg is not None and handler.args.vararg.annotation is not None:
        nodes.append(handler.args.vararg.annotation)
    if handler.args.kwarg is not None and handler.args.kwarg.annotation is not None:
        nodes.append(handler.args.kwarg.annotation)
    return tuple(nodes)


def _bms_backend_route_owners(
    bms: Path,
) -> dict[tuple[str, str], dict[str, Any]]:
    main_relative = "platform/api/main.py"
    main_tree = _ast(_verified_path(bms, main_relative))
    main_mounts = [
        node
        for node in ast.walk(main_tree)
        if isinstance(node, ast.Call)
        and _optional_call_name(node.func) == "app.include_router"
        and len(node.args) == 1
        and ast.unparse(node.args[0]) == "bioxp.router"
    ]
    if len(main_mounts) != 1:
        raise DenominatorError("BMS BioXP package router mount is unresolved or duplicated")
    main_mount = main_mounts[0]
    prefix_values = [
        keyword.value for keyword in main_mount.keywords if keyword.arg == "prefix"
    ]
    if (
        len(prefix_values) != 1
        or not isinstance(prefix_values[0], ast.Constant)
        or prefix_values[0].value != "/api/bioxp"
    ):
        raise DenominatorError("BMS BioXP package router prefix is not exactly /api/bioxp")
    main_mount_gates = _depends_bindings(
        (main_mount,),
        source_file=main_relative,
        owner_identity="app.include_router:bioxp.router",
        layer="main_mount",
    )

    package_relative = "platform/api/routers/bioxp/__init__.py"
    package_tree = _ast(_verified_path(bms, package_relative))
    child_loops = [
        node
        for node in package_tree.body
        if isinstance(node, ast.For)
        and isinstance(node.target, ast.Name)
        and node.target.id == "child_router"
        and isinstance(node.iter, (ast.Tuple, ast.List))
    ]
    if len(child_loops) != 1:
        raise DenominatorError("BMS BioXP child-router mount loop is unresolved")
    child_loop = child_loops[0]
    child_iter = child_loop.iter
    if not isinstance(child_iter, (ast.Tuple, ast.List)):
        raise DenominatorError("BMS BioXP child-router mount loop is unresolved")
    actual_children = tuple(ast.unparse(element) for element in child_iter.elts)
    expected_children = tuple(f"{name}.router" for name in BMS_BACKEND_CHILD_ROUTER_POLICY)
    expected_body = ast.parse("router.routes.extend(child_router.routes)").body[0]
    if (
        actual_children != expected_children
        or len(child_loop.body) != 1
        or ast.dump(child_loop.body[0]) != ast.dump(expected_body)
    ):
        raise DenominatorError(
            f"BMS BioXP child-router ownership changed: "
            f"expected={expected_children!r} actual={actual_children!r}"
        )

    discovered: dict[tuple[str, str], dict[str, Any]] = {}
    methods = {"get", "post", "put", "patch", "delete"}
    for module_name, relative in BMS_BACKEND_CHILD_ROUTER_POLICY.items():
        tree = _ast(_verified_path(bms, relative))
        router_assignments = [
            node
            for node in tree.body
            if isinstance(node, (ast.Assign, ast.AnnAssign))
            and any(
                isinstance(target, ast.Name) and target.id == "router"
                for target in (
                    node.targets if isinstance(node, ast.Assign) else [node.target]
                )
            )
        ]
        if len(router_assignments) != 1:
            raise DenominatorError(f"BMS backend router owner is unresolved: {relative}")
        router_value = router_assignments[0].value
        if (
            not isinstance(router_value, ast.Call)
            or _optional_call_name(router_value.func) != "APIRouter"
        ):
            raise DenominatorError(f"BMS backend router is not a direct APIRouter: {relative}")
        prefix_nodes = [
            keyword.value for keyword in router_value.keywords if keyword.arg == "prefix"
        ]
        if prefix_nodes:
            if (
                len(prefix_nodes) != 1
                or not isinstance(prefix_nodes[0], ast.Constant)
                or type(prefix_nodes[0].value) is not str
            ):
                raise DenominatorError(
                    f"BMS backend router prefix is not one literal: {relative}"
                )
            router_prefix = str(prefix_nodes[0].value)
        else:
            router_prefix = ""
        router_gates = _depends_bindings(
            (router_value,),
            source_file=relative,
            owner_identity=f"{module_name}.router",
            layer="child_router",
        )
        owner_by_identity, _functions, _classes = _definition_inventory(tree)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            handler_owner = owner_by_identity.get(id(node))
            if handler_owner is None:
                continue
            for decorator in node.decorator_list:
                if not isinstance(decorator, ast.Call) or not isinstance(decorator.func, ast.Attribute):
                    continue
                if (
                    _optional_call_name(decorator.func.value) != "router"
                    or decorator.func.attr.lower() not in methods
                ):
                    continue
                if (
                    len(decorator.args) != 1
                    or not isinstance(decorator.args[0], ast.Constant)
                    or type(decorator.args[0].value) is not str
                ):
                    raise DenominatorError(
                        f"BMS backend route path is not one literal: "
                        f"{relative}:{decorator.lineno}"
                    )
                method = decorator.func.attr.upper()
                route_path = "/api/bioxp" + router_prefix + str(decorator.args[0].value)
                key = (method, route_path)
                if key in discovered:
                    raise DenominatorError(f"duplicate mounted BMS BioXP route: {key}")
                decorator_gates = _depends_bindings(
                    (decorator,),
                    source_file=relative,
                    owner_identity=handler_owner,
                    layer="route_decorator",
                )
                parameter_gates = _depends_bindings(
                    _handler_dependency_nodes(node),
                    source_file=relative,
                    owner_identity=handler_owner,
                    layer="handler_parameter",
                )
                handler_closure = _bms_backend_handler_closure(
                    tree,
                    node,
                    source_file=relative,
                )
                discovered[key] = {
                    "method": method,
                    "route_path": route_path,
                    "source_file": relative,
                    "source_line": int(node.lineno),
                    "decorator_line": int(decorator.lineno),
                    "handler_owner": handler_owner,
                    "main_mount_owner": "platform/api/main.py:app.include_router:bioxp.router",
                    "main_mount_multiplicity": len(main_mounts),
                    "child_mount_owner": (
                        f"{package_relative}:router.routes.extend:{module_name}.router"
                    ),
                    "child_mount_multiplicity": actual_children.count(f"{module_name}.router"),
                    "gate_bindings": tuple(
                        (*main_mount_gates, *router_gates, *decorator_gates, *parameter_gates)
                    ),
                    "handler_closure": handler_closure,
                }
    closure_rows = tuple(discovered[key] for key in sorted(discovered))
    closure_digest = _sha256_bytes(_canonical_bytes(closure_rows))
    if closure_digest != BMS_BACKEND_ROUTE_CLOSURE_SHA256:
        raise DenominatorError(
            "mounted BMS BioXP route closure changed: "
            f"expected_sha256={BMS_BACKEND_ROUTE_CLOSURE_SHA256} "
            f"observed_sha256={closure_digest}"
        )
    return discovered


def _bms_backend_route_rows(
    bms: Path,
    digests: Mapping[tuple[str, str], str],
) -> list[dict[str, Any]]:
    owners = _bms_backend_route_owners(bms)
    relay_mapping = _bms_route_mapping(bms, None)
    rows: list[dict[str, Any]] = []
    for (method, route_path), owner in sorted(owners.items()):
        relative = str(owner["source_file"])
        closure = owner["handler_closure"]
        relay_occurrences = tuple(closure["relay_occurrences"])
        remote_calls = tuple(closure["remote_calls"])
        relay_names = tuple(
            str(occurrence["relay_name"]) for occurrence in relay_occurrences
        )
        unknown_relays = set(relay_names) - set(relay_mapping)
        if unknown_relays:
            raise DenominatorError(
                f"mounted BMS backend route has unknown relay keys: "
                f"{sorted(unknown_relays)!r}"
            )
        local_only = not relay_names and not remote_calls
        relay_policies = tuple(BMS_RELAY_POLICY[name] for name in relay_names)
        gate_targets = tuple(
            str(binding["dependency_target"])
            for binding in owner["gate_bindings"]
        )
        rows.append(
            _row(
                family="bms_backend_route",
                repository="bms",
                source_file=relative,
                source_sha256=digests[("bms", relative)],
                source_line=int(owner["source_line"]),
                identifier=f"{method} {route_path}",
                final_callable=str(owner["handler_owner"]),
                control_class=(
                    "local_query"
                    if local_only and method == "GET"
                    else (
                        "local_action"
                        if local_only
                        else (
                            "mutation_relay"
                            if any(policy[0] != "GET" for policy in relay_policies)
                            else "read_only_relay"
                        )
                    )
                ),
                transport_effect=(
                    "local_only"
                    if local_only
                    else "relay:" + ",".join(relay_names)
                ),
                physical_effect_possible=(
                    False
                    if local_only
                    else any(bool(policy[6]) for policy in relay_policies)
                ),
                requires_durable_claim=False,
                dispatch_path=f"bms_backend_call_graph:{closure['sha256']}",
                verification_id=(
                    "RA.RW0.BMS.BACKEND."
                    + re.sub(r"[^A-Za-z0-9_.-]", "_", str(owner["handler_owner"]))
                ),
                mutation_gate=(
                    "dependency:" + ",".join(gate_targets)
                    if gate_targets
                    else "none"
                ),
                correlation_fields=("connection_generation", "command_id", "job_id"),
                report_projection="bms_backend_route_receipt",
                bms_exposure={
                    "method": method,
                    "path": route_path,
                    "main_mount_owner": owner["main_mount_owner"],
                    "main_mount_multiplicity": owner["main_mount_multiplicity"],
                    "child_mount_owner": owner["child_mount_owner"],
                    "child_mount_multiplicity": owner["child_mount_multiplicity"],
                    "handler_owner": owner["handler_owner"],
                    "gate_bindings": list(owner["gate_bindings"]),
                    "relay_occurrences": list(relay_occurrences),
                    "remote_calls": list(remote_calls),
                    "handler_closure_sha256": closure["sha256"],
                    "handler_call_multiplicities": closure["call_graph"]["multiplicities"],
                },
                cockpit_control=route_path,
            )
        )
    return rows


def _route_shape(path: str) -> str:
    return re.sub(r"\{[^}]+\}", "{}", path.split("?", 1)[0])


def _verify_bms_frontend_backend_closure(
    bms: Path,
    frontend_calls: Mapping[str, Sequence[tuple[str, str]]],
) -> dict[str, list[dict[str, Any]]]:
    backend = _bms_backend_route_owners(bms)
    exposure: dict[str, list[dict[str, Any]]] = {}
    for definition, calls in frontend_calls.items():
        rows: list[dict[str, Any]] = []
        for method, endpoint in calls:
            matches = [
                (key, value)
                for key, value in backend.items()
                if key[0] == method.upper() and _route_shape(key[1]) == _route_shape(endpoint)
            ]
            if len(matches) != 1:
                raise DenominatorError(
                    f"frontend endpoint does not resolve to one mounted BMS handler: "
                    f"definition={definition} call={(method, endpoint)!r} matches={[key for key, _ in matches]!r}"
                )
            (backend_method, backend_path), owner = matches[0]
            closure = owner["handler_closure"]
            rows.append(
                {
                    "frontend_method": method.upper(),
                    "frontend_path": endpoint,
                    "backend_method": backend_method,
                    "backend_path": backend_path,
                    "backend_owner": owner["source_file"],
                    "backend_handler": owner["handler_owner"],
                    "backend_call_graph_sha256": closure["call_graph"]["sha256"],
                    "backend_route_closure_sha256": closure["sha256"],
                    "relay_occurrences": list(closure["relay_occurrences"]),
                    "remote_calls": list(closure["remote_calls"]),
                    "gate_bindings": list(owner["gate_bindings"]),
                }
            )
        exposure[definition] = rows
    return exposure


def _named_imports(text: str, module: str) -> dict[str, tuple[str, bool]]:
    imports: dict[str, tuple[str, bool]] = {}
    for match in _ES_IMPORT_RE.finditer(text):
        if match.group("module") != module:
            continue
        clause = match.group("clause").strip()
        clause_type_only = clause.startswith("type ")
        braces = re.search(r"\{(?P<names>.*?)\}", clause, re.DOTALL)
        if braces is None:
            raise DenominatorError(f"BioXP client import from {module!r} must use explicit named imports")
        for raw_entry in braces.group("names").split(","):
            entry = raw_entry.strip()
            if not entry:
                continue
            entry_type_only = clause_type_only
            if entry.startswith("type "):
                entry_type_only = True
                entry = entry[5:].strip()
            parts = re.split(r"\s+as\s+", entry)
            if len(parts) > 2 or not parts[0]:
                raise DenominatorError(f"malformed BioXP client import identity: {raw_entry!r}")
            imported = parts[0].strip()
            local = parts[-1].strip()
            if imported in imports:
                raise DenominatorError(f"duplicate BioXP client import identity: {imported}")
            imports[imported] = (local, entry_type_only)
    return imports


def _text_without_es_imports(text: str) -> str:
    characters = list(text)
    for match in _ES_IMPORT_RE.finditer(text):
        for index in range(match.start(), match.end()):
            if characters[index] != "\n":
                characters[index] = " "
    return "".join(characters)


def _component_import_bindings(text: str) -> set[tuple[str, str]]:
    bindings: set[tuple[str, str]] = set()
    for match in _ES_IMPORT_RE.finditer(text):
        clause = match.group("clause").strip()
        module = match.group("module")
        default_clause = clause.split("{", 1)[0].rstrip(",").strip()
        if default_clause and not default_clause.startswith(("type ", "*")):
            default_name = default_clause.split(",", 1)[0].strip()
            if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", default_name):
                bindings.add((default_name, module))
        braces = re.search(r"\{(?P<names>.*?)\}", clause, re.DOTALL)
        if braces is None:
            continue
        for raw_entry in braces.group("names").split(","):
            entry = raw_entry.strip()
            if not entry:
                continue
            if entry.startswith("type "):
                entry = entry[5:].strip()
            parts = re.split(r"\s+as\s+", entry)
            bindings.add((parts[-1].strip(), module))
    bindings.update(
        (match.group("name"), match.group("module"))
        for match in _DYNAMIC_COMPONENT_IMPORT_RE.finditer(text)
    )
    return bindings


def _resolve_frontend_module(bms: Path, importer_relative: str, module: str) -> str | None:
    if not module.startswith("."):
        return None
    importer = Path(importer_relative)
    candidate = importer.parent / module
    candidates: list[Path] = []
    if candidate.suffix in _FRONTEND_CODE_SUFFIXES:
        candidates.append(candidate)
        if candidate.suffix in {".js", ".jsx"}:
            candidates.extend(
                candidate.with_suffix(suffix) for suffix in _FRONTEND_CODE_SUFFIXES
            )
    elif candidate.suffix in _FRONTEND_OPAQUE_SUFFIXES:
        candidates.append(candidate)
    elif candidate.suffix:
        candidates.extend(
            Path(f"{candidate}{suffix}") for suffix in _FRONTEND_CODE_SUFFIXES
        )
    else:
        candidates.extend(candidate.with_suffix(suffix) for suffix in _FRONTEND_CODE_SUFFIXES)
        candidates.extend(candidate / f"index{suffix}" for suffix in _FRONTEND_CODE_SUFFIXES)
    root = bms.resolve()
    source_root = (root / "platform/frontend/src").resolve()
    matches: list[Path] = []
    for relative_candidate in candidates:
        absolute = (root / relative_candidate).resolve()
        try:
            absolute.relative_to(source_root)
        except ValueError:
            continue
        if absolute.is_file():
            matches.append(absolute)
    unique_matches = sorted(set(matches), key=lambda path: path.as_posix())
    if len(unique_matches) == 1:
        return unique_matches[0].relative_to(root).as_posix()
    if len(unique_matches) > 1:
        raise DenominatorError(
            "mounted frontend module is ambiguous: "
            f"importer={importer_relative!r} module={module!r} "
            f"matches={[path.relative_to(root).as_posix() for path in unique_matches]!r}"
        )
    raise DenominatorError(
        f"mounted frontend module cannot be resolved: importer={importer_relative!r} module={module!r}"
    )


def _frontend_module_references(
    text: str,
) -> tuple[tuple[str, str, int, int], ...]:
    matches: list[tuple[int, str, str]] = []
    for kind, pattern in (
        ("import", _ES_IMPORT_RE),
        ("import", _ES_SIDE_EFFECT_IMPORT_RE),
        ("export", _ES_EXPORT_FROM_RE),
        ("import", _ES_DYNAMIC_IMPORT_RE),
    ):
        matches.extend(
            (match.start(), kind, match.group("module"))
            for match in pattern.finditer(text)
        )
    references: list[tuple[str, str, int, int]] = []
    for occurrence, (offset, kind, module) in enumerate(sorted(matches), start=1):
        references.append(
            (kind, module, text.count("\n", 0, offset) + 1, occurrence)
        )
    return tuple(references)


def _frontend_mount_graph(bms: Path) -> dict[str, Any]:
    queue = [_FRONTEND_APP_PATH]
    visited: set[str] = set()
    consumers: dict[str, tuple[str, ...]] = {}
    parents: dict[str, list[dict[str, Any]]] = {}
    module_edges: dict[str, list[dict[str, Any]]] = {}
    while queue:
        relative = queue.pop(0)
        if relative in visited:
            continue
        visited.add(relative)
        text = _source_text(_verified_path(bms, relative))
        edges: list[dict[str, Any]] = []
        for reference_kind, module, source_line, occurrence in _frontend_module_references(text):
            resolved = _resolve_frontend_module(bms, relative, module)
            if resolved is not None:
                edges.append(
                    {
                        "kind": reference_kind,
                        "module": module,
                        "source_line": source_line,
                        "occurrence": occurrence,
                        "target": resolved,
                    }
                )
        module_edges[relative] = edges
        for child_relative in sorted({str(edge["target"]) for edge in edges}):
            if child_relative not in visited:
                queue.append(child_relative)
        for component, module in sorted(_component_import_bindings(text)):
            mounts = list(re.finditer(rf"<\s*{re.escape(component)}(?=\s|/|>)", text))
            if not mounts:
                continue
            child_relative = _resolve_frontend_module(bms, relative, module)
            if child_relative is None:
                continue
            edge = {
                "component": component,
                "import_module": module,
                "parent_path": relative,
                "mount_lines": tuple(
                    text.count("\n", 0, mount.start()) + 1 for mount in mounts
                ),
            }
            child_parents = parents.setdefault(child_relative, [])
            if edge not in child_parents:
                child_parents.append(edge)
            if child_relative not in visited:
                queue.append(child_relative)
    reexporters: set[str] = set()
    changed = True
    while changed:
        changed = False
        for relative, edges in module_edges.items():
            if relative in reexporters:
                continue
            if any(
                edge["kind"] == "export"
                and (
                    edge["target"] == _FRONTEND_CLIENT_PATH
                    or edge["target"] in reexporters
                )
                for edge in edges
            ):
                reexporters.add(relative)
                changed = True
    for relative, edges in module_edges.items():
        client_modules = sorted(
            str(edge["module"])
            for edge in edges
            if edge["kind"] == "import"
            and (
                edge["target"] == _FRONTEND_CLIENT_PATH
                or edge["target"] in reexporters
            )
        )
        if client_modules:
            consumers[relative] = tuple(client_modules)
    canonical_edges = tuple(
        {
            "source": source,
            **edge,
        }
        for source in sorted(module_edges)
        for edge in module_edges[source]
    )
    return {
        "reachable": visited,
        "consumers": consumers,
        "parents": parents,
        "reexporters": reexporters,
        "edges": canonical_edges,
        "sha256": _sha256_bytes(_canonical_bytes(canonical_edges)),
    }


def _frontend_consumer_rows(
    bms: Path,
    digests: Mapping[tuple[str, str], str],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    text_cache: dict[str, str] = {}
    graph = _frontend_mount_graph(bms)
    discovered_consumers = set(graph["consumers"])
    expected_consumers = set(FRONTEND_CONSUMER_POLICY)
    if discovered_consumers != expected_consumers:
        missing = sorted(expected_consumers - discovered_consumers)
        extra = sorted(discovered_consumers - expected_consumers)
        raise DenominatorError(
            f"mounted BioXP frontend consumer inventory changed: missing={missing} extra={extra}"
        )
    for relative, policy in FRONTEND_CONSUMER_POLICY.items():
        text = text_cache.setdefault(relative, _source_text(_verified_path(bms, relative)))
        component = str(policy["component"])
        declarations = [
            match
            for match in _FRONTEND_COMPONENT_RE.finditer(text)
            if match.group("name") == component
        ]
        if len(declarations) != 1:
            raise DenominatorError(
                f"frontend consumer component inventory changed for {relative}: "
                f"expected={component!r} declarations={len(declarations)}"
            )

        client_module = str(policy["client_module"])
        imports = _named_imports(text, client_module)
        expected_imports = set(policy["client_imports"])
        actual_imports = set(imports)
        if actual_imports != expected_imports:
            missing = sorted(expected_imports - actual_imports)
            extra = sorted(actual_imports - expected_imports)
            raise DenominatorError(
                f"BioXP frontend client consumer identities changed for {relative}: "
                f"missing={missing} extra={extra}"
            )
        expected_type_imports = set(policy["client_type_imports"])
        actual_type_imports = {
            name for name, (_, type_only) in imports.items() if type_only
        }
        if actual_type_imports != expected_type_imports:
            missing = sorted(expected_type_imports - actual_type_imports)
            extra = sorted(actual_type_imports - expected_type_imports)
            raise DenominatorError(
                f"BioXP frontend client type identities changed for {relative}: "
                f"missing={missing} extra={extra}"
            )

        body = _text_without_es_imports(text)
        identifiers = set(re.findall(r"\b[A-Za-z_][A-Za-z0-9_]*\b", body))
        unused_imports = sorted(
            imported
            for imported, (local, _) in imports.items()
            if local not in identifiers
        )
        if unused_imports:
            raise DenominatorError(
                f"BioXP frontend client imports are not consumed in {relative}: {unused_imports}"
            )
        call_identifiers = set(re.findall(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*\(", body))
        actual_client_calls = {
            imported
            for imported, (local, _) in imports.items()
            if local in call_identifiers
        }
        expected_client_calls = set(policy["client_calls"])
        if actual_client_calls != expected_client_calls:
            missing = sorted(expected_client_calls - actual_client_calls)
            extra = sorted(actual_client_calls - expected_client_calls)
            raise DenominatorError(
                f"BioXP frontend client call identities changed for {relative}: "
                f"missing={missing} extra={extra}"
            )

        component_edges = [
            edge
            for edge in graph["parents"].get(relative, ())
            if edge["component"] == component
        ]
        if len(component_edges) != 1:
            raise DenominatorError(
                f"frontend consumer mount graph changed for {relative}: "
                f"component={component} edges={len(component_edges)}"
            )
        component_edge = component_edges[0]
        parent_relative = str(policy["parent_path"])
        if (
            component_edge["parent_path"] != parent_relative
            or component_edge["import_module"] != str(policy["parent_import_module"])
            or len(component_edge["mount_lines"]) != 1
        ):
            raise DenominatorError(
                f"frontend consumer parent policy changed for {relative}: "
                f"expected={(parent_relative, policy['parent_import_module'])!r} "
                f"actual={component_edge!r}"
            )
        parent_text = text_cache.setdefault(
            parent_relative,
            _source_text(_verified_path(bms, parent_relative)),
        )
        parent_component = str(policy["parent_component"])
        parent_declarations = re.findall(
            rf"^(?:export\s+)?function\s+{re.escape(parent_component)}\s*\(",
            parent_text,
            re.MULTILINE,
        )
        if len(parent_declarations) != 1:
            raise DenominatorError(
                f"frontend consumer parent component changed for {relative}: {parent_component}"
            )
        declaration = declarations[0]
        rows.append(
            _row(
                family="frontend_consumer",
                repository="bms",
                source_file=relative,
                source_sha256=digests[("bms", relative)],
                source_line=text.count("\n", 0, declaration.start()) + 1,
                identifier=component,
                final_callable=component,
                control_class="frontend_consumer",
                transport_effect="mounted_typed_client_consumer",
                physical_effect_possible=False,
                requires_durable_claim=False,
                dispatch_path="mounted_typed_bms_client_consumer",
                verification_id=f"RA.RW0.UI.CONSUMER.{component}",
                mutation_gate="typed_bms_api_and_connection_generation",
                correlation_fields=("expected_connection_generation",),
                report_projection="mounted_cockpit_consumer",
                bms_exposure={
                    "bioxp_client_imports": sorted(actual_imports),
                    "bioxp_client_type_imports": sorted(actual_type_imports),
                    "bioxp_client_calls": sorted(actual_client_calls),
                    "parent_mount": {
                        "component": parent_component,
                        "path": parent_relative,
                        "source_line": component_edge["mount_lines"][0],
                        "source_sha256": digests[("bms", parent_relative)],
                    },
                },
                cockpit_control=f"mounted:{component}",
            )
        )
    return rows


def _normalize_frontend_endpoint(path: str) -> str:
    def replace_expression(match: re.Match[str]) -> str:
        identifiers = [
            name
            for name in re.findall(r"[A-Za-z_][A-Za-z0-9_]*", match.group(1))
            if name not in {"encodeURIComponent", "String"}
        ]
        if not identifiers:
            raise DenominatorError(f"frontend endpoint template expression is not resolvable: {match.group(1)!r}")
        return "{" + identifiers[0] + "}"

    return re.sub(r"\$\{(.*?)\}", replace_expression, path)


def _typescript_structure_mask(text: str) -> tuple[str, tuple[int, ...]]:
    masked = list(text)
    depths = [0] * (len(text) + 1)
    depth = 0
    state = "normal"
    index = 0
    while index < len(text):
        character = text[index]
        next_character = text[index + 1] if index + 1 < len(text) else ""
        depths[index] = depth
        if state == "normal":
            if character == "/" and next_character == "/":
                masked[index] = masked[index + 1] = " "
                state = "line_comment"
                index += 2
                continue
            if character == "/" and next_character == "*":
                masked[index] = masked[index + 1] = " "
                state = "block_comment"
                index += 2
                continue
            if character in {"'", '"', "`"}:
                masked[index] = " "
                state = {"'": "single", '"': "double", "`": "template"}[character]
            elif character == "{":
                depth += 1
            elif character == "}":
                depth -= 1
                if depth < 0:
                    raise DenominatorError("frontend source has an unmatched closing brace")
        elif state == "line_comment":
            if character == "\n":
                state = "normal"
            else:
                masked[index] = " "
        elif state == "block_comment":
            if character == "*" and next_character == "/":
                masked[index] = masked[index + 1] = " "
                state = "normal"
                index += 2
                continue
            if character != "\n":
                masked[index] = " "
        else:
            if character == "\\":
                masked[index] = " "
                if index + 1 < len(text):
                    if text[index + 1] != "\n":
                        masked[index + 1] = " "
                    index += 2
                    continue
            terminator = {"single": "'", "double": '"', "template": "`"}[state]
            if character == terminator:
                state = "normal"
            if character != "\n":
                masked[index] = " "
        index += 1
    depths[len(text)] = depth
    if state in {"single", "double", "block_comment", "template"} or depth != 0:
        raise DenominatorError("frontend source has an unterminated literal/comment or unbalanced braces")
    return "".join(masked), tuple(depths)


def _frontend_export_blocks(
    text: str,
) -> tuple[
    dict[str, tuple[str, int]],
    dict[str, str],
]:
    mask, depths = _typescript_structure_mask(text)
    top_level_exports = [
        match
        for match in re.finditer(r"^(?P<indent>[ \t]*)export\b", mask, re.MULTILINE)
        if depths[match.start()] == 0
    ]
    indented = [match.start() for match in top_level_exports if match.group("indent")]
    if indented:
        raise DenominatorError(f"frontend top-level export must start in column zero: offsets={indented!r}")
    boundaries = [
        match
        for match in _FRONTEND_TOP_LEVEL_DEFINITION_RE.finditer(mask)
        if depths[match.start()] == 0
    ]
    boundary_starts = {match.start() for match in boundaries}
    unparsed_exports = [match.start() for match in top_level_exports if match.start() not in boundary_starts]
    if unparsed_exports:
        raise DenominatorError(f"frontend export syntax is multiline or unresolved: offsets={unparsed_exports!r}")

    definitions: dict[str, tuple[str, int]] = {}
    exports: dict[str, str] = {}
    export_lists: list[tuple[str, int]] = []
    for index, boundary in enumerate(boundaries):
        end = boundaries[index + 1].start() if index + 1 < len(boundaries) else len(text)
        block = text[boundary.start():end]
        line = text.count("\n", 0, boundary.start()) + 1
        declaration = _FRONTEND_DEFINITION_RE.match(text, boundary.start())
        if declaration is not None:
            name = str(declaration.group("function") or declaration.group("const"))
            if name in definitions:
                raise DenominatorError(f"duplicate BioXP client definition: {name}")
            definitions[name] = (block, line)
            if boundary.group("export"):
                exports[name] = name
            continue
        if boundary.group("export_list"):
            if re.search(r"\}\s*from\s*['\"]", block):
                raise DenominatorError(
                    f"cross-file BioXP client export list is unsupported at line {line}"
                )
            export_lists.append((block, line))
            continue
        if boundary.group("export_star") or boundary.group("export_default"):
            raise DenominatorError(
                f"indirect BioXP client export authority is unsupported at line {line}"
            )
        if (
            _FRONTEND_ENDPOINT_RE.search(block)
            or re.search(r"\bapi\b", block)
            or "useQuery(" in block
            or "useMutation(" in block
            or "useRefreshMutation(" in block
        ):
            raise DenominatorError(
                f"unresolved frontend definition owns BioXP behavior at line {line}"
            )

    for block, line in export_lists:
        match = re.match(r"export\s*\{(?P<names>[^}]*)\}", block, re.DOTALL)
        if match is None:
            raise DenominatorError(f"malformed BioXP client export list at line {line}")
        for raw_entry in match.group("names").split(","):
            entry = raw_entry.strip()
            if not entry:
                continue
            parts = re.split(r"\s+as\s+", entry)
            if len(parts) > 2:
                raise DenominatorError(f"malformed BioXP client export identity: {raw_entry!r}")
            local = parts[0].strip()
            exported = parts[-1].strip()
            if local not in definitions:
                raise DenominatorError(
                    f"BioXP client export does not resolve to one local definition: {local}"
                )
            if exported in exports:
                raise DenominatorError(f"duplicate exported BioXP client identity: {exported}")
            exports[exported] = local
    return definitions, exports


def _frontend_reachable_definition_names(
    root: str,
    definitions: Mapping[str, tuple[str, int]],
) -> tuple[str, ...]:
    if root not in definitions:
        raise DenominatorError(f"exported BioXP client definition is missing: {root}")
    reachable: set[str] = set()
    queue = [root]
    while queue:
        current = queue.pop(0)
        if current in reachable:
            continue
        reachable.add(current)
        block = definitions[current][0]
        mask, _depths = _typescript_structure_mask(block)
        for candidate in definitions:
            if candidate in reachable or candidate == current:
                continue
            if re.search(rf"\b{re.escape(candidate)}\b", mask):
                queue.append(candidate)
    return tuple(sorted(reachable))


def _frontend_api_calls(
    name: str,
    block: str,
    endpoint_constants: Mapping[str, str],
) -> tuple[tuple[str, str], ...]:
    mask, _depths = _typescript_structure_mask(block)
    api_offsets = [match.start() for match in re.finditer(r"\bapi\b", mask)]
    direct_pattern = re.compile(
        r"\bapi\.(?P<method>get|post|put|patch|delete)\s*"
        r"(?:<[^;()]*>)?\s*\(\s*"
        r"(?P<arg>(?:[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*)|"
        r"(?:'/api/bioxp/(?:\\.|[^'\\])*'|"
        r"\"/api/bioxp/(?:\\.|[^\"\\])*\"|"
        r"`/api/bioxp/(?:\\.|[^`\\])*`))",
        re.DOTALL,
    )
    direct_calls = [match for match in direct_pattern.finditer(block) if match.start() in api_offsets]
    if len(direct_calls) != len(api_offsets) or len({match.start() for match in direct_calls}) != len(api_offsets):
        raise DenominatorError(f"frontend API identifier is aliased, rebound, computed, optional, or unresolved: {name}")

    calls: list[tuple[str, str]] = []
    for call in direct_calls:
        method = call.group("method").lower()
        argument = call.group("arg")
        if argument[0] in "'\"`":
            endpoint = _normalize_frontend_endpoint(argument[1:-1])
        elif argument in endpoint_constants:
            endpoint = endpoint_constants[argument]
        else:
            raise DenominatorError(f"frontend API endpoint expression is unresolved: {name}:{argument}")
        calls.append((method, endpoint))
    if len(set(calls)) != len(calls):
        raise DenominatorError(
            f"frontend definition repeats one direct API call identity: {name}:{calls!r}"
        )
    return tuple(calls)


def _frontend_rows(bms: Path, digests: Mapping[tuple[str, str], str]) -> list[dict[str, Any]]:
    relative = "platform/frontend/src/lib/bioxpClient.ts"
    path = _verified_path(bms, relative)
    text = _source_text(path)
    definitions, exports = _frontend_export_blocks(text)
    endpoint_constants: dict[str, str] = {}
    endpoint_constant_owners: set[str] = set()
    for match in re.finditer(
        r"^(?:export\s+)?const\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*(?P<quote>['\"`])(?P<path>/api/bioxp/.*?)(?P=quote)",
        text,
        re.MULTILINE,
    ):
        endpoint_constants[match.group("name")] = _normalize_frontend_endpoint(match.group("path"))
    for name, (block, _line) in definitions.items():
        for match in re.finditer(
            r"(?P<property>[A-Za-z_][A-Za-z0-9_]*)\s*:\s*(?P<quote>['\"`])"
            r"(?P<path>/api/bioxp/.*?)(?P=quote)",
            block,
            re.DOTALL,
        ):
            reference = f"{name}.{match.group('property')}"
            if reference in endpoint_constants:
                raise DenominatorError(f"duplicate frontend endpoint constant: {reference}")
            endpoint_constants[reference] = _normalize_frontend_endpoint(match.group("path"))
            endpoint_constant_owners.add(name)

    discovered: dict[str, tuple[tuple[tuple[str, ...], str, str, str, bool, bool], int]] = {}
    frontend_backend_calls: dict[str, tuple[tuple[str, str], ...]] = {}
    owned_definitions: set[str] = set()
    for name, local_name in exports.items():
        reachable_names = _frontend_reachable_definition_names(local_name, definitions)
        owned_definitions.update(reachable_names)
        behavior_names = tuple(
            reachable
            for reachable in reachable_names
            if reachable not in endpoint_constant_owners or reachable == local_name
        )
        block = "\n".join(definitions[reachable][0] for reachable in behavior_names)
        line = definitions[local_name][1]
        scoped_constants = dict(endpoint_constants)
        for match in re.finditer(
            r"^[ \t]+const\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*=\s*"
            r"(?P<quote>['\"`])(?P<path>/api/bioxp/.*?)(?P=quote)",
            block,
            re.MULTILINE,
        ):
            constant_name = match.group("name")
            endpoint = _normalize_frontend_endpoint(match.group("path"))
            prior = scoped_constants.get(constant_name)
            if prior is not None and prior != endpoint:
                raise DenominatorError(
                    f"frontend helper endpoint constant is ambiguous: {name}:{constant_name}"
                )
            scoped_constants[constant_name] = endpoint
        paths = {
            _normalize_frontend_endpoint(match.group("path"))
            for match in _FRONTEND_ENDPOINT_RE.finditer(block)
        }
        for reference, endpoint in scoped_constants.items():
            if re.search(rf"\b{re.escape(reference)}\b", block):
                paths.add(endpoint)
        api_calls = _frontend_api_calls(name, block, scoped_constants)
        api_methods = {method for method, _endpoint in api_calls}
        call_paths = {endpoint for _method, endpoint in api_calls}
        paths.update(call_paths)
        query_hook = "useQuery(" in block
        action_hook = "useMutation(" in block or "useRefreshMutation(" in block
        if not paths:
            if api_methods or query_hook or action_hook:
                raise DenominatorError(f"endpoint-bearing frontend definition has no resolvable URL: {name}")
            continue
        if local_name in endpoint_constant_owners and not query_hook and not action_hook and not api_methods:
            continue
        if query_hook and not action_hook:
            control_class, effect = "frontend_query", "query"
        elif action_hook and not query_hook:
            control_class, effect = "frontend_action", "action"
        elif api_methods and api_methods <= {"get"}:
            control_class, effect = "frontend_query", "direct_query"
        elif api_methods and "get" not in api_methods:
            control_class, effect = "frontend_action", "direct_action"
        elif not api_methods and "return" in block:
            control_class, effect = "frontend_query", "url_builder"
        else:
            raise DenominatorError(f"endpoint-bearing frontend definition is not uniquely classified: {name}")
        backend_calls = list(api_calls)
        uncalled_paths = sorted(paths - call_paths)
        if uncalled_paths:
            if control_class != "frontend_query":
                raise DenominatorError(
                    f"frontend action endpoint has no direct typed API call: {name}:{uncalled_paths!r}"
                )
            backend_calls.extend(("get", endpoint) for endpoint in uncalled_paths)
        if len(set(backend_calls)) != len(backend_calls):
            raise DenominatorError(
                f"frontend backend call identity is duplicated: {name}:{backend_calls!r}"
            )
        frontend_backend_calls[name] = tuple(sorted(backend_calls))
        discovered[name] = (
            (tuple(sorted(paths)), name, control_class, effect, False, False),
            line,
        )

    for local_name, (block, line) in definitions.items():
        if local_name in owned_definitions:
            continue
        if (
            _FRONTEND_ENDPOINT_RE.search(block)
            or re.search(r"\bapi\b", _typescript_structure_mask(block)[0])
            or "useQuery(" in block
            or "useMutation(" in block
            or "useRefreshMutation(" in block
        ):
            raise DenominatorError(
                f"BioXP client behavior is not reachable from one exported owner: {local_name}:{line}"
            )

    discovered_policy = {name: policy for name, (policy, _line) in discovered.items()}
    if discovered_policy != dict(FRONTEND_DEFINITION_POLICY):
        raise DenominatorError(
            "complete BioXP frontend definition policy changed: "
            f"expected={dict(FRONTEND_DEFINITION_POLICY)!r} actual={discovered_policy!r}"
        )
    backend_exposure = _verify_bms_frontend_backend_closure(
        bms,
        frontend_backend_calls,
    )
    non_endpoint_helpers = {
        "bioXpErrorPresentation",
        "bioXpErrorText",
        "bioXpReceiptIsNonTerminal",
    }
    mounted_calls = {
        str(call)
        for consumer in FRONTEND_CONSUMER_POLICY.values()
        for call in consumer["client_calls"]
    }
    unresolved_endpoint_calls = sorted(mounted_calls - set(discovered_policy) - non_endpoint_helpers)
    if unresolved_endpoint_calls:
        raise DenominatorError(
            f"mounted frontend endpoint calls do not resolve to exported client definitions: {unresolved_endpoint_calls}"
        )
    rows: list[dict[str, Any]] = []
    for name, policy in sorted(FRONTEND_DEFINITION_POLICY.items()):
        paths, final_callable, control_class, effect, physical, claim = policy
        line = discovered[name][1]
        rows.append(
            _row(
                family="frontend_definition",
                repository="bms",
                source_file=relative,
                source_sha256=digests[("bms", relative)],
                source_line=line,
                identifier=name,
                final_callable=final_callable,
                control_class=control_class,
                transport_effect=effect,
                physical_effect_possible=physical,
                requires_durable_claim=claim,
                dispatch_path="typed_bms_client",
                verification_id=f"RA.RW0.UI.{name}",
                mutation_gate="typed_bms_api_and_connection_generation",
                correlation_fields=("expected_connection_generation",),
                report_projection="cockpit_query" if control_class == "frontend_query" else "cockpit_action_receipt",
                bms_exposure={
                    "endpoints": list(paths),
                    "backend_closure": backend_exposure[name],
                },
                cockpit_control=name,
            )
        )
    if not rows or not any(row["control_class"] == "frontend_query" for row in rows) or not any(row["control_class"] == "frontend_action" for row in rows):
        raise DenominatorError("explicit BioXP frontend query/action definitions were not both discovered")
    consumer_rows = _frontend_consumer_rows(bms, digests)
    for consumer in consumer_rows:
        exposure = consumer.get("bms_exposure")
        calls = exposure.get("bioxp_client_calls") if isinstance(exposure, Mapping) else None
        if not isinstance(calls, list):
            raise DenominatorError(f"mounted frontend consumer has no resolved client call inventory: {consumer['id']}")
        for call in calls:
            if call in FRONTEND_DEFINITION_POLICY:
                resolved = discovered_policy.get(str(call))
                if resolved is None or not resolved[0]:
                    raise DenominatorError(
                        f"mounted endpoint-bearing frontend call does not resolve to an exported endpoint definition: {call}"
                    )
    rows.extend(consumer_rows)
    return rows


def _invariants(rows: Sequence[Mapping[str, Any]], manifest: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    identities = [row["id"] for row in rows]
    bypasses = [
        row["id"]
        for row in rows
        if row["requires_durable_claim"] is True and row["dispatch_path"] == "direct_transport"
    ]
    callbacks = {
        row["public_method_or_path"]
        for row in rows
        if row["control_class"] == "callback_event"
    }
    expected_frontend_consumers = {
        f"{relative}:{policy['component']}"
        for relative, policy in FRONTEND_CONSUMER_POLICY.items()
    }
    actual_frontend_consumers = {
        f"{row['source_file']}:{row['public_method_or_path']}"
        for row in rows
        if row["family"] == "frontend_consumer"
    }
    return {
        "row_count": len(rows),
        "source_count": len(manifest),
        "family_counts": dict(sorted(Counter(str(row["family"]) for row in rows).items())),
        "unclassified_count": sum(
            1
            for row in rows
            if row.get("control_class") not in _ALLOWED_CONTROL_CLASSES or not row.get("verification_id")
        ),
        "direct_transport_bypass_count": len(bypasses),
        "direct_transport_bypass_ids": sorted(bypasses),
        "duplicate_identity_count": len(identities) - len(set(identities)),
        "missing_private_callback_count": len(_PRIVATE_CALLBACK_IDENTITIES - callbacks),
        "missing_private_callbacks": sorted(_PRIVATE_CALLBACK_IDENTITIES - callbacks),
        "robot_liquid_route_count": sum(1 for row in rows if row["family"] == "robot_route"),
        "operator_catalog_dispatch_count": sum(1 for row in rows if row["family"] == "operator_catalog_dispatch"),
        "robot_command_plane_route_count": sum(
            1 for row in rows if row["family"] == "robot_command_plane_route"
        ),
        "robot_report_route_count": sum(
            1 for row in rows if row["family"] == "robot_report_route"
        ),
        "bms_default_robot_route_count": sum(1 for row in rows if row["family"] == "bms_default_robot_route"),
        "frontend_definition_count": sum(1 for row in rows if row["family"] == "frontend_definition"),
        "frontend_consumer_count": sum(1 for row in rows if row["family"] == "frontend_consumer"),
        "missing_frontend_consumer_count": len(expected_frontend_consumers - actual_frontend_consumers),
        "missing_frontend_consumers": sorted(expected_frontend_consumers - actual_frontend_consumers),
        "extra_frontend_consumer_count": len(actual_frontend_consumers - expected_frontend_consumers),
        "extra_frontend_consumers": sorted(actual_frontend_consumers - expected_frontend_consumers),
    }


def _validate_payload(payload: Mapping[str, Any]) -> None:
    if payload.get("schema") != SCHEMA:
        raise DenominatorError("denominator schema is not current")
    rows = payload.get("rows")
    authority = payload.get("authority")
    envelope = payload.get("canonical_payload")
    invariants = payload.get("invariants")
    if type(rows) is not list or not rows or type(authority) is not dict or type(envelope) is not dict or type(invariants) is not dict:
        raise DenominatorError("denominator has an incomplete closed shape")
    if rows != sorted(rows, key=lambda row: (row["family"], row["repository"], row["source_file"], row["source_line"], row["public_method_or_path"])):
        raise DenominatorError("denominator rows are not in canonical order")
    manifest = authority.get("source_manifest")
    if type(manifest) is not list or manifest != sorted(manifest, key=lambda row: (row["repository"], row["path"])):
        raise DenominatorError("source manifest is absent or not canonically sorted")
    if authority.get("source_manifest_sha256") != _sha256_bytes(_canonical_bytes(manifest)):
        raise DenominatorError("source manifest digest is invalid")
    import_graphs = authority.get("import_graphs")
    anchors = authority.get("manifest_anchors")
    if (
        type(import_graphs) is not dict
        or set(import_graphs) != {"robot_python", "bms_python", "bms_frontend"}
        or authority.get("import_graphs_sha256")
        != _sha256_bytes(_canonical_bytes(import_graphs))
        or type(anchors) is not list
    ):
        raise DenominatorError("source import-graph authority is absent or invalid")
    expected_anchors = [
        {"repository": "robot", "path": GENERATOR_PATH, "role": "generator"},
        {
            "repository": "bms",
            "path": BMS_MOUNT_ANCHOR_POLICY[0],
            "role": "mounted_application_anchor",
        },
    ]
    if anchors != expected_anchors:
        raise DenominatorError("source manifest anchor authority changed")
    covered_paths = {
        (str(anchor["repository"]), str(anchor["path"])) for anchor in anchors
    }
    for graph_name, graph in import_graphs.items():
        if (
            type(graph) is not dict
            or type(graph.get("repository")) is not str
            or type(graph.get("paths")) is not list
            or graph["paths"] != sorted(set(graph["paths"]))
            or graph.get("unresolved_local_count") != 0
            or graph.get("sha256")
            != _sha256_bytes(_canonical_bytes(graph.get("edges")))
        ):
            raise DenominatorError(
                f"source import graph is unresolved or noncanonical: {graph_name}"
            )
        covered_paths.update(
            (str(graph["repository"]), str(path)) for path in graph["paths"]
        )
    manifest_paths = {
        (str(row["repository"]), str(row["path"])) for row in manifest
    }
    if covered_paths != manifest_paths:
        raise DenominatorError(
            f"source manifest reachability is incomplete: "
            f"orphaned={sorted(manifest_paths - covered_paths)!r} "
            f"unmanifested={sorted(covered_paths - manifest_paths)!r}"
        )
    if envelope != {
        "algorithm": "sha256",
        "canonicalization": "utf8-json-sorted-keys-compact",
        "digest_scope": "complete_payload_with_canonical_payload_omitted",
        "sha256": canonical_payload_sha256(payload),
    }:
        raise DenominatorError("canonical payload digest envelope is invalid")
    expected_invariants = _invariants(rows, manifest)
    if invariants != expected_invariants:
        raise DenominatorError("denominator invariant projection is stale")
    if invariants["unclassified_count"] != 0:
        raise DenominatorError("denominator contains unclassified rows")
    if invariants["direct_transport_bypass_count"] != 0:
        raise DenominatorError("denominator contains a direct transport bypass")
    if invariants["duplicate_identity_count"] != 0:
        raise DenominatorError("denominator contains duplicate identities")
    if invariants["missing_private_callback_count"] != 0:
        raise DenominatorError("denominator omits a required private callback")
    if invariants["operator_catalog_dispatch_count"] != len(OPERATOR_CATALOG_ROUTE_POLICY):
        raise DenominatorError("denominator omits part of the complete mounted operator catalog")
    if invariants["robot_command_plane_route_count"] != len(COMMAND_PLANE_ROUTE_POLICY):
        raise DenominatorError("denominator omits part of the mounted command plane")
    if invariants["robot_report_route_count"] != len(REPORT_ROUTE_POLICY):
        raise DenominatorError("denominator omits part of the mounted report router")
    if invariants["missing_frontend_consumer_count"] != 0 or invariants["extra_frontend_consumer_count"] != 0:
        raise DenominatorError("denominator frontend consumer inventory changed")


def generate_denominator(
    robot_root: str | Path,
    *,
    robot_repository_url: str,
    robot_commit: str,
    robot_tree: str,
    bms_root: str | Path,
    bms_repository_url: str,
    bms_commit: str,
    bms_tree: str,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    robot = _verified_root(robot_root, "robot")
    bms = _verified_root(bms_root, "BMS")
    generator = _verified_path(robot, GENERATOR_PATH)
    if generator != Path(__file__).resolve():
        raise DenominatorError("the executing generator is not the supplied robot root's generator")
    repositories = {
        "robot": _validate_repository_identity("robot", robot, robot_repository_url, robot_commit, robot_tree),
        "bms": _validate_repository_identity("BMS", bms, bms_repository_url, bms_commit, bms_tree),
    }
    _verify_repository_root(robot, "robot", repositories["robot"])
    _verify_repository_root(bms, "BMS", repositories["bms"])
    manifest, digests, import_graphs = _source_manifest(robot, bms)
    source_commit_differences = _manifest_commit_differences(
        manifest,
        robot=robot,
        bms=bms,
        repositories=repositories,
    )
    route_rows, catalog_rows = _robot_route_rows(robot, digests)
    rows = [
        *route_rows,
        *catalog_rows,
        *_mounted_command_and_report_rows(robot, digests),
        *_protocol_rows(robot, digests),
        *_lifecycle_rows(robot, digests),
        *_transport_rows(robot, digests),
        *_bms_rows(bms, digests),
        *_bms_backend_route_rows(bms, digests),
        *_frontend_rows(bms, digests),
    ]
    rows.sort(key=lambda row: (row["family"], row["repository"], row["source_file"], row["source_line"], row["public_method_or_path"]))
    payload: dict[str, Any] = {
        "schema": SCHEMA,
        "artifact_path": CANONICAL_ARTIFACT_PATH,
        "authority": {
            "generator": {
                "repository": "robot",
                "path": GENERATOR_PATH,
                "sha256": digests[("robot", GENERATOR_PATH)],
            },
            "repositories": repositories,
            "source_manifest": manifest,
            "source_manifest_sha256": _sha256_bytes(_canonical_bytes(manifest)),
            "source_state": "working_candidate_over_base_commits",
            "source_commit_differences": source_commit_differences,
            "source_commit_differences_sha256": _sha256_bytes(
                _canonical_bytes(source_commit_differences)
            ),
            "import_graphs": import_graphs,
            "import_graphs_sha256": _sha256_bytes(_canonical_bytes(import_graphs)),
            "manifest_anchors": [
                {
                    "repository": "robot",
                    "path": GENERATOR_PATH,
                    "role": "generator",
                },
                {
                    "repository": "bms",
                    "path": BMS_MOUNT_ANCHOR_POLICY[0],
                    "role": "mounted_application_anchor",
                },
            ],
        },
        "rows": rows,
        "invariants": _invariants(rows, manifest),
    }
    payload["canonical_payload"] = {
        "algorithm": "sha256",
        "canonicalization": "utf8-json-sorted-keys-compact",
        "digest_scope": "complete_payload_with_canonical_payload_omitted",
        "sha256": canonical_payload_sha256(payload),
    }
    _validate_payload(payload)
    if output_path is not None:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(json.dumps(payload, indent=2, sort_keys=True).encode("utf-8") + b"\n")
    return payload


def verify_denominator(
    payload: Mapping[str, Any],
    robot_root: str | Path,
    *,
    robot_repository_url: str,
    robot_commit: str,
    robot_tree: str,
    bms_root: str | Path,
    bms_repository_url: str,
    bms_commit: str,
    bms_tree: str,
) -> None:
    _validate_payload(payload)
    expected = generate_denominator(
        robot_root,
        robot_repository_url=robot_repository_url,
        robot_commit=robot_commit,
        robot_tree=robot_tree,
        bms_root=bms_root,
        bms_repository_url=bms_repository_url,
        bms_commit=bms_commit,
        bms_tree=bms_tree,
    )
    if payload != expected:
        raise DenominatorError("denominator is not the exact pinned source regeneration")


def _identity_args(parser: argparse.ArgumentParser, prefix: str) -> None:
    parser.add_argument(f"--{prefix}-root", type=Path, required=True)
    parser.add_argument(f"--{prefix}-repository-url", required=True)
    parser.add_argument(f"--{prefix}-commit", required=True)
    parser.add_argument(f"--{prefix}-tree", required=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate the exact BioXP/BMS runtime audit entrypoint denominator")
    _identity_args(parser, "robot")
    _identity_args(parser, "bms")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = generate_denominator(
        args.robot_root,
        robot_repository_url=args.robot_repository_url,
        robot_commit=args.robot_commit,
        robot_tree=args.robot_tree,
        bms_root=args.bms_root,
        bms_repository_url=args.bms_repository_url,
        bms_commit=args.bms_commit,
        bms_tree=args.bms_tree,
        output_path=args.output,
    )
    print(payload["canonical_payload"]["sha256"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Public manual custody identities; motion remains in the OEM finite worker."""
from .oem_deck_movement import OEM_PLATE_NAME_ORDINALS
from .oem_compat.pathing import LOCATION_NAME_TO_ID

PLATE_ACTION_OPERATIONS = {
    "oem.deck.catch_plate": "catch_plate",
    "oem.deck.release_plate": "release_plate",
}


def plate_action_inputs(action_id):
    if action_id == "oem.deck.catch_plate":
        return [
            {"name": "plate", "required": True, "type": "integer",
             "enum": sorted(set(OEM_PLATE_NAME_ORDINALS.values()))},
            {"name": "run_in_parallel", "required": False, "type": "boolean", "default": True},
        ]
    return [
        {"name": "destination", "required": True, "type": "integer",
         "enum": sorted(set(LOCATION_NAME_TO_ID.values()))},
        {"name": "press_plate", "required": False, "type": "boolean", "default": False},
        {"name": "run_in_parallel", "required": False, "type": "boolean", "default": True},
    ]


def validate_plate_action_inputs(action_id, inputs):
    """Validate the source enum/types, not historical custody or sensor proof."""
    from fastapi import HTTPException
    for field in plate_action_inputs(action_id):
        name = field["name"]
        if name not in inputs and not field["required"]:
            continue
        value = inputs.get(name)
        valid_type = type(value) is (int if field["type"] == "integer" else bool)
        if not valid_type or ("enum" in field and value not in field["enum"]):
            raise HTTPException(422, detail={"error": "invalid_plate_action_input", "field": name})
    # Omitted defaults stay omitted in request identity; the existing compiler
    # owns their application, exactly as for native plate_catch/plate_release.
    return dict(inputs)

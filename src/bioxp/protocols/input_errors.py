"""Representation errors, not runtime hardware exception handling."""
from typing import Mapping


class ProtocolInputError(ValueError):
    def __init__(self, message, path=""):
        super().__init__(message)
        self.path = path

    def to_payload(self):
        return {"code": "invalid_protocol_input", "message": str(self),
                "path": self.path, "category": "representation"}


def validate_native_shape(data):
    def obj(value, path):
        if not isinstance(value, Mapping):
            raise ProtocolInputError("Expected an object", path)
    def seq(value, path):
        if not isinstance(value, (list, tuple)):
            raise ProtocolInputError("Expected an array", path)
    obj(data, "")
    if "metadata" in data and data["metadata"] is not None:
        obj(data["metadata"], "/metadata")
    if "operations" in data:
        seq(data["operations"], "/operations")
        for i, operation in enumerate(data["operations"]):
            obj(operation, f"/operations/{i}")
        return
    if "version" in data and (type(data["version"]) is not int):
        raise ProtocolInputError("Version must be an integer", "/version")
    seq(data.get("stages", ()), "/stages")
    if "metadata" in data and data["metadata"] is not None:
        obj(data["metadata"], "/metadata")
    for i, stage in enumerate(data.get("stages", ())):
        path = f"/stages/{i}"
        obj(stage, path)
        seq(stage.get("actions", ()), path + "/actions")
        if "metadata" in stage and stage["metadata"] is not None:
            obj(stage["metadata"], path + "/metadata")
        for j, action in enumerate(stage.get("actions", ())):
            ap = path + f"/actions/{j}"
            obj(action, ap)
            for key in ("params", "metadata"):
                if key in action and action[key] is not None:
                    obj(action[key], ap + "/" + key)

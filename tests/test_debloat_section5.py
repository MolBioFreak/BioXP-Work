"""Section 5 route retirement: real mounted OpenAPI, no app lifespan or USB."""

from fastapi import FastAPI


def test_retired_shadow_capture_is_not_mounted_and_cached_read_remains():
    from bioxp.oem_homing_routes import router

    app = FastAPI()
    app.include_router(router)
    paths = app.openapi()["paths"]
    assert "/motion/oem/shadow_readback/capture" not in paths
    assert "get" in paths["/motion/oem/shadow_readback"]


def test_supported_canonical_snapshot_collection_remains_mounted():
    from bioxp.api import app

    paths = app.openapi()["paths"]
    assert "post" in paths["/hardware/snapshot/collect"]
    assert "/motion/oem/shadow_readback/capture" not in paths

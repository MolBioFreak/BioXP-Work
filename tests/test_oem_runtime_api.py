from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.api import app


def test_runtime_api_exact_command_result_does_not_guess_unknown_ids(tmp_path):
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    response = TestClient(app).get("/oem/runtime/commands/not-a-real-command")

    assert response.status_code == 404
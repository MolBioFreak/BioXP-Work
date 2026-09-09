"""The actual lifespan migration connection must support retained named rows."""
import asyncio
from types import SimpleNamespace
import pytest
from bioxp import api
from bioxp.oem_runtime_store import OEMRuntimeStore


def test_retained_database_reaches_post_migration_startup(tmp_path, monkeypatch):
    store = OEMRuntimeStore(tmp_path)
    store.close()
    monkeypatch.setattr(api, 'configure_release_identity', lambda: {})
    monkeypatch.setattr(api, 'runtime_state_root', lambda: tmp_path)
    monkeypatch.setattr(api, '_pipette_receipts', SimpleNamespace(root=tmp_path))
    monkeypatch.setattr(api, '_operator_reports_installed', True)

    class AfterMigration(Exception):
        pass

    def reached(_store):
        raise AfterMigration()

    monkeypatch.setattr(api, 'reconcile_operator_report_exports', reached)

    async def start():
        async with api.lifespan(api.app):
            pytest.fail('test must stop before any hardware startup')

    with pytest.raises(AfterMigration):
        asyncio.run(start())

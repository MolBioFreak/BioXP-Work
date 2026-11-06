"""Home settlement runs on the dispatcher thread between lane-lease renewals.

Its candidate query must not rescan operator_plane_transitions per command:
on the robot that took ~17 s, the 5 s lease lapsed and a live workflow lost
authority mid-run (worker exited, job left dispatched).
"""
import inspect
import re

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore


def _candidate_sql():
    source = inspect.getsource(OperatorCommandStore.settle_deck_recovery_by_home)
    block = source.split("candidates = conn.execute(", 1)[1].split(").fetchall()", 1)[0]
    return "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', block))


def test_candidate_query_scans_transitions_once(tmp_path):
    init = OEMRuntimeStore(tmp_path); init.close()
    store = OperatorCommandStore(tmp_path)
    try:
        sql = _candidate_sql()
        assert "axes_homed_after_unknown_moves" in sql
        plan = [row[3] for row in store.connection.execute("EXPLAIN QUERY PLAN " + sql)]
        # A correlated subquery over transitions re-scans it once per command.
        for index, detail in enumerate(plan[:-1]):
            if detail.startswith("CORRELATED"):
                assert not plan[index + 1].startswith("SCAN t"), plan
        assert store.settle_deck_recovery_by_home() in (True, False)
    finally:
        store.stop()

import sqlite3

from bioxp.operator_command_plane import OperatorCommandPlane


def _db(hold=1):
    c = sqlite3.connect(":memory:")
    c.row_factory = sqlite3.Row
    c.executescript("""
    CREATE TABLE operator_plane_safety(singleton INTEGER, recovery_hold INTEGER);
    CREATE TABLE operator_plane_commands(command_id TEXT, action_id TEXT, status TEXT, finished_at REAL, updated_at REAL);
    CREATE TABLE operator_plane_deck_commands(command_id TEXT, ambiguity_state TEXT);
    CREATE TABLE operator_plane_deck_recovery_decisions(command_id TEXT);
    CREATE TABLE operator_plane_deck_semantic_state(singleton INTEGER, ambiguity_state TEXT);
    CREATE TABLE operator_commands(command_id TEXT, action_id TEXT, status TEXT, updated_at REAL);
    INSERT INTO operator_plane_deck_semantic_state VALUES(1,'none');
    INSERT INTO operator_plane_commands VALUES('m','oem.deck._finite_operation','ambiguous',100,100);
    """)
    c.execute("INSERT INTO operator_plane_safety VALUES(1,?)", (hold,))
    return c


def test_unknown_move_holds_until_x_y_and_z_home_after_it():
    c = _db()
    assert OperatorCommandPlane._deck_recovery_blocker(c) == "deck_recovery_hold"
    c.execute("INSERT INTO operator_commands VALUES('z0','oem.z.manual_home','completed',50)")
    c.execute("INSERT INTO operator_commands VALUES('z1','oem.z.manual_home','completed',150)")
    c.execute("INSERT INTO operator_commands VALUES('xy0','oem.xy.home','failed',160)")
    assert OperatorCommandPlane._deck_recovery_blocker(c) == "deck_recovery_hold"
    c.execute("INSERT INTO operator_commands VALUES('xy1','oem.xy.home','completed',170)")
    # Resolved by the homes; the stale hold flag is what settle_deck_recovery_by_home lifts.
    assert OperatorCommandPlane._deck_recovery_blocker(c) == "deck_recovery_state_inconsistent"
    c.execute("UPDATE operator_plane_safety SET recovery_hold=0")
    assert OperatorCommandPlane._deck_recovery_blocker(c) is None

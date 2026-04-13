import importlib.util
import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DECODER_PATH = ROOT / "scripts" / "decode_tmcl_frame.py"
FIXTURE_DIR = ROOT / "testdata" / "tmcl" / "fixtures"



def load_decoder_module():
    spec = importlib.util.spec_from_file_location("decode_tmcl_frame", DECODER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class DecodeTmclTests(unittest.TestCase):
    def test_single_frame_decode_accepts_raw_hex(self):
        decoder = load_decoder_module()

        decoded = decoder.decode_many([
            "7E 00 00 00 04 08 8A 00 01 00 00 00 00 00 97 7E",
        ])[0]["decoded"]

        self.assertEqual(decoded["kind"], "request")
        self.assertEqual(decoded["board"]["name"], "HEAD_BOARD")
        self.assertEqual(decoded["command"]["name"], "QUERY_MOTOR_STOP")
        self.assertEqual(decoded["interpretation"]["motion_axis"], "Z")
        self.assertEqual(decoded["interpretation"]["query_scope"], "single_axis")

        heartbeat = decoder.decode_many([
            "7E 00 00 00 00 08 04 81 00 00 00 00 00 00 8D 7E",
        ])[0]["decoded"]
        self.assertEqual(heartbeat["kind"], "heartbeat_long")
        self.assertEqual(heartbeat["board"]["name"], "HEAD_BOARD")

    def test_transaction_decode_correlates_gap_and_gio_fixture(self):
        decoder = load_decoder_module()
        session = decoder.decode_session_lines(
            (FIXTURE_DIR / "manual_gap_and_gio_session.jsonl").read_text(encoding="utf-8").splitlines()
        )

        self.assertEqual(session["summary"]["transaction_count"], 2)
        self.assertEqual(session["summary"]["unmatched_reply_count"], 0)
        self.assertEqual(session["summary"]["pending_request_count"], 0)

        gap_tx = session["transactions"][0]
        self.assertEqual(gap_tx["command"]["name"], "GAP")
        self.assertEqual(gap_tx["latency_ms"], 4.0)
        self.assertEqual(gap_tx["reply"]["interpretation"]["parameter_name"], "ACTUAL_POSITION")
        self.assertEqual(gap_tx["reply"]["interpretation"]["motion_axis"], "Z")
        self.assertEqual(gap_tx["reply"]["interpretation"]["reply_value"], 12345)

        gio_tx = session["transactions"][1]
        self.assertEqual(gio_tx["command"]["name"], "GIO")
        self.assertEqual(gio_tx["reply"]["interpretation"]["io_query"], "DOOR_SENSOR")
        self.assertIs(gio_tx["reply"]["interpretation"]["io_active"], True)
        self.assertEqual(gio_tx["reply"]["interpretation"]["reply_value"], 1)

    def test_transaction_decode_marks_heartbeat_and_enriches_motor_stop_reply(self):
        decoder = load_decoder_module()
        session = decoder.decode_session_lines(
            (FIXTURE_DIR / "manual_query_motor_stop_session.jsonl").read_text(encoding="utf-8").splitlines()
        )

        self.assertEqual(session["summary"]["frame_records"], 3)
        self.assertEqual(session["frames"][0]["decoded"]["kind"], "heartbeat_short")
        self.assertEqual(session["summary"]["transaction_count"], 1)

        tx = session["transactions"][0]
        self.assertEqual(tx["command"]["name"], "QUERY_MOTOR_STOP")
        self.assertEqual(tx["latency_ms"], 5.0)
        self.assertEqual(tx["reply"]["interpretation"]["motion_axis"], "Z")
        self.assertIs(tx["reply"]["interpretation"]["is_stopped"], True)
        self.assertEqual(tx["reply"]["request_context"]["motor"], 1)

    def test_decoder_cli_outputs_transaction_json_for_captured_fixture(self):
        fixture = FIXTURE_DIR / "manual_thermal_sensor_session.jsonl"
        result = subprocess.run(
            [sys.executable, str(DECODER_PATH), "--session", "--compact", "--input", str(fixture)],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        )

        payload = json.loads(result.stdout)
        self.assertEqual(payload["summary"]["transaction_count"], 1)
        tx = payload["transactions"][0]
        self.assertEqual(tx["command"]["name"], "READ_SENSOR_TEMPERATURE")
        self.assertEqual(tx["reply"]["interpretation"]["thermal_sensor"], "THERMAL_SENSOR_0")
        self.assertEqual(tx["reply"]["interpretation"]["temperature_c"], 25.75)
        self.assertEqual(tx["reply"]["request_context"]["motor"], 0)


if __name__ == "__main__":
    unittest.main()

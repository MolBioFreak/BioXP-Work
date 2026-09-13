"""Reduced retained test_operator_controls / test_z_stop_outer_lease fixtures.
Only configuration and USB endpoints are synthetic. No device is initialized.
"""
import queue
import threading

from fastapi import FastAPI
from bioxp import operator_controls
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.novo_usb_can import NovoUsbCanBus, novo_encode


def make_app(root, monkeypatch, *, z_stop_route=None):
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_ROOT', str(root))
    monkeypatch.setattr(operator_controls, 'current_release_identity', lambda: {
        'verified': True, 'release_id': 'test-release',
        'source': {'manifest_sha256': '1' * 64, 'aggregate_sha256': '2' * 64}})
    monkeypatch.setattr(operator_controls, 'current_authority_identity', lambda: {
        'evidence_lock_identity_verified': True, 'evidence_lock_sha256': '3' * 64})
    monkeypatch.setattr(operator_controls, 'current_registry_sha256', lambda: '4' * 64)
    OEMRuntimeStore(root).close()
    app = FastAPI()
    if z_stop_route is not None:
        app.add_api_route('/motion/oem/z/stop', z_stop_route, methods=['POST'])

    class HardwareState:
        ownership_epoch = 7

        def ownership_projection(self):
            return {'ownership_epoch': 7, 'ownership': {
                'transport': 'owned', 'usb': 'service', 'router': 'running', 'CAN_READY': True}}

        def project(self, *domains, independent_domains=False):
            observations = {'power': {'safety_valid': True},
                'latch': {'door_sensor': 1, 'latch_sensor': 1},
                'interlock': {'motion_arm': {'armed': True}}}
            return {'snapshot_id': 'test-snapshot',
                'freshness': {'state': 'fresh', 'age_s': 0.0, 'fresh_for_s': 30.0},
                'domains': {domain: {'status': 'observed', 'observation': observations.get(domain, {})}
                            for domain in domains}}

    monkeypatch.setattr(operator_controls, 'hardware_state', HardwareState())
    operator_controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {'motion_blocked': False, 'recovery_required': False},
        reference_state_provider=lambda: {'rows': {axis: {'state': 'referenced'} for axis in ('x','y','z','g','door')}},
        lifecycle_state_provider=lambda: {'operation_state': 'stopped', 'door': {'door_closed': True, 'latch_closed': True}},
        serial206_initialization_state_provider=lambda: {'bound': True, 'initialize_motors_live_available': True,
            'x_authority': {'state': 'referenced', 'reference_state': 'referenced', 'lifecycle': {
                'state': 'referenced_ready', 'board_lifecycle_generation_fresh': True}},
            'z_authority': {'state': 'referenced', 'reference_state': 'referenced', 'lifecycle': {'state': 'referenced_ready'}}})
    return app, []


class HardwareEndpoints:
    def __init__(self):
        self.replies = queue.Queue()
        self.stop_delivered = threading.Event()
        self.stop_writes = 0
        self.events = []
        self.gap_value = 1000
        self.gap_status = 100

    def write(self, frame, timeout=None):
        payload = bytes(frame)[1:-2]
        board, command = payload[3], payload[5]
        if command == 3:
            self.stop_writes += 1
            self.events.append(f'stop_write_{self.stop_writes}')
            if self.stop_writes == 2:
                self.stop_delivered.set()
        value = self.gap_value if command == 6 else 0
        status = self.gap_status if command == 6 else 100
        data = bytes([board, status, command]) + value.to_bytes(4, 'big') + b'\x00'
        self.replies.put(novo_encode(NovoUsbCanBus.build_payload(0, data, 8)))
        return len(frame)

    def read(self, size, timeout=None):
        try:
            return self.replies.get(timeout=(timeout or 10) / 1000)
        except queue.Empty:
            raise TimeoutError('synthetic USB receive timeout')

"""Real pressure transport for legacy source-order fixtures; raw CAN leaves only.

These fixtures are source sequencing checks, not SQLite authority acceptance.
The latter lives in test_pressure_empty_source_v1 with the real collection owner.
"""
import time
from types import SimpleNamespace
from bioxp.can_driver import BioXpCanDriver
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport


def pressure_source_transport(value=12.25, loaded_channel: int | None = 2):
    transports = []
    for channel in range(4):
        driver = BioXpCanDriver.__new__(BioXpCanDriver)
        driver.pipette_id = channel
        driver.bus = SimpleNamespace(router=SimpleNamespace(reader_generation=1))
        def exchange(command, *, address, ack_mode, command_name, channel=channel):
            assert address == 'report' and ack_mode == 'query'
            now = time.monotonic()
            if command_name == 'query_tip_status':
                assert command == '?31'
                data = [32, 96, 49 if channel == loaded_channel else 48]
            else:
                assert command_name == 'query_pressure' and command == '?57'
                data = [32, 96, *str(value).encode('ascii')]
            return {'ok': True, 'query_response_correlated': True,
                'ack': {'received': True, 'data': data},
                'provenance': {'channel': channel, 'owner_generation': 1,
                    'transaction_id': f'pressure-fixture:{channel}:{now}',
                    'tx_timestamp': now, 'receive_timestamp': now}}
        driver._send_pipette_command = exchange
        transports.append(CanPipetteTransport(driver_factory=lambda driver=driver: driver, pipette_id=channel))
    group = FourPipetteTransport(transports)
    group.query_tip_status_all()
    return group

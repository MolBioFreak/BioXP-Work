"""Optional, attempt-owned ADP stream control. No acquisition controls for pLLD."""
from collections.abc import Mapping

from .cavro_commands import compile_device_command

# Short bounded exchanges; never reuse the stroke completion or its timeout.
TIMEOUT_MS = 300


def authored_control(node):
    """Explicit configuration wins, even when it disables streaming. Ignore
    documentary Recipe provenance. Buffered programs may change settings or
    leave executable contents: don't inject automatic commands around them.
    """
    if isinstance(node, Mapping):
        if ('pressure_streaming' in node or node.get('operation') == 'device_program'
                or (node.get('command') == 'event_subscription' and node.get('event') == 'pressure')):
            return True
        return any(authored_control(v) for k,v in node.items() if k not in ('liquid_settings', 'metadata'))
    return isinstance(node, (tuple,list)) and any(authored_control(v) for v in node)


class StreamControl:
    def __init__(self, group, channels, router, epoch, authored, fence, ref):
        self.group, self.router, self.epoch = group, router, epoch
        self.generation = router.reader_generation
        self.fence = fence
        self.rows = ref['stream_control'] = [{'channel':ch} for ch in channels]
        self.stroke_revisions = {}
        self.drivers = {ch: group._transports[ch]._driver for ch in channels}
        self.authored = authored

    def check_authority(self):
        self.fence()
        if self.group._interrupt_epoch != self.epoch or self.group._forceabort():
            raise RuntimeError('workflow pressure control interrupted')

    def driver(self, ch):
        self.check_authority()
        d = self.drivers[ch]
        if (self.group._transports[ch]._driver is not d or d.bus.router is not self.router
                or d.bus.shared_usb.novo_router is not self.router or not self.router.running
                or self.router.reader_generation != self.generation):
            raise RuntimeError('pressure reader changed')
        if self.router.pipette_completion_pending(ch):
            raise ValueError('channel already has a completion owner')
        return d

    def report(self, ch, selector):
        spec = compile_device_command({'command':'report', 'selector':selector})
        result = self.driver(ch).send_cavro_command(spec, timeout_ms=TIMEOUT_MS)
        p = result.get('provenance') or {}
        if (result.get('delivery_verified') is not True or result.get('ok') is not True
                or result.get('semantic_query_response_verified') is not True
                or p.get('owner_generation') != self.generation or not p.get('transaction_id')
                or p.get('channel') != ch or p.get('observed_rx_id') != (0x506 | ch << 3)):
            raise ValueError(f'uncorrelated ?{selector} report')
        return result['report']['raw_ascii']

    def write(self, row, enabled):
        ch = row['channel']
        if self.report(ch,67) != '0':
            raise ValueError('command buffer not confirmed empty')
        spec = compile_device_command({'command':'event_subscription','event':'pressure',
                                       'mode':'always' if enabled else 'disabled'})
        d = self.driver(ch)
        label = 'enable' if enabled else 'restore'
        row[label] = 'unresolved'
        sent = d.send_cavro_command(spec, timeout_ms=TIMEOUT_MS)
        p = sent.get('provenance') or {}
        row[label + '_transaction_id'] = p.get('transaction_id')
        token = p.get('completion_owner_token')
        if not token or not p.get('completion_deferred'):
            raise ValueError('no owned stream command completion')
        end = d.wait_pipette_command_completion(TIMEOUT_MS/1000, owner_token=token)
        if (sent.get('delivery_verified') is not True or sent.get('ok') is not True
                or end.get('ok') is not True or end.get('owner_token') != token or end.get('channel') != ch
                or not p.get('transaction_id') or end.get('transaction_id') != p['transaction_id']
                or end.get('owner_generation') != self.generation):
            raise ValueError('stream command completion not confirmed')
        row[label] = 'completion_returned'

    def enable(self):
        for row in self.rows:
            try:
                if self.authored:
                    row['enable'] = 'authored_control'; continue
                ch = row['channel']
                self.group._transports[ch]._require_initialized()
                # 30053815-C p48: five pipe-separated ASCII fields; ?76 is protocol,
                # not enable state. Never infer OFF from absence of samples.
                fields = self.report(ch,77).split('|')
                if len(fields)!=5 or not all(fields) or fields[2] not in ('PSOFF','PSON'):
                    raise ValueError('unrecognized ?77 pressure_streaming field')
                row['prior'] = fields[2]
                if fields[2]=='PSOFF': self.write(row, True)
                else: row['enable'] = 'already_enabled'
            except Exception as exc:
                row['setup_error'] = str(exc)[:256]

    def restore(self, outcome):
        for row in reversed(self.rows):
            if row.get('enable') not in ('unresolved','completion_returned'): continue
            row['restore'] = 'unresolved'
            try:
                if row['enable'] == 'unresolved':
                    raise ValueError('stream change ownership unresolved')
                ch = row['channel']
                end = next((r.get('completion') or {} for r in (outcome or {}).get('channels', [])
                            if r['channel']==ch), {})
                if not end.get('ok') and end.get('outcome')!='device_error':
                    raise ValueError('stroke completion unresolved')
                with self.group._transaction_lock:
                    if self.stroke_revisions.get(ch) != self.driver(ch)._pipette_buffer_revision:
                        raise ValueError('intervening channel command')
                    self.write(row, False)
            except Exception as exc:
                row['restore_error'] = str(exc)[:256]

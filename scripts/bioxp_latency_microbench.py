#!/usr/bin/env python3
"""Synthetic offline state/reference benchmark; no hardware or live DB contact.

Run with the repository's development interpreter from an isolated checkout.
Results are host-specific microbenchmarks, not command-latency predictions.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import tempfile
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=200)
    args = parser.parse_args(argv)
    if not 1 <= args.iterations <= 5000:
        parser.error('iterations must be 1..5000')
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    from bioxp.services.reference_service import ReferenceStateStore, MarkAxisReferencedCommand
    with tempfile.TemporaryDirectory(prefix='bioxp-offline-latency-') as directory:
        root = Path(directory)
        store = OEMRuntimeStore(root)
        references = None
        try:
            provider = Serial206OemInitializationProvider(object(), state_store=store)
            state = provider._new_state()
            state['machine_status']['benchmark_padding'] = [
                {'key': i, 'text': 'synthetic-state-only-' + str(i) * 5} for i in range(1500)]
            provider._save_state(state)
            raw = store._db.execute('SELECT state_json FROM serial206_authority_snapshots ORDER BY sequence DESC LIMIT 1').fetchone()[0]
            provider._load_state()
            start = time.perf_counter()
            for _ in range(args.iterations): provider._load_state()
            provider_ms = (time.perf_counter() - start) * 1000 / args.iterations
            references = ReferenceStateStore(root / 'bioxp_runtime.db')
            assert references.mark_referenced(MarkAxisReferencedCommand('x', 0, source='offline-benchmark'))['ok']
            start = time.perf_counter()
            for _ in range(args.iterations): references.snapshot(('x',))
            reference_ms = (time.perf_counter() - start) * 1000 / args.iterations
            print(json.dumps({'scope': 'synthetic_offline_not_robot_latency',
                              'state_json_bytes': len(raw.encode()), 'iterations': args.iterations,
                              'provider_load_ms': provider_ms, 'reference_read_ms': reference_ms}, indent=2))
        finally:
            if references is not None and hasattr(references, 'close'): references.close()
            store.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

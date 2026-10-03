"""Export the actual native schemas under the offline guard."""
import json
import os
from pathlib import Path
from bioxp.protocols.method_contract import method_contract


def test_export_final_contract():
    contract = method_contract()
    assert contract['physical_qualification'] is False
    assert contract['bindings']['seal_separate']['execution'] == 'source_noop'
    Path(os.environ['CAVRO_EVIDENCE_ROOT'], 'native-contract.json').write_text(json.dumps(contract, indent=2) + '\n')
    import gzip
    from bioxp.protocols import compile_native_protocol
    root = Path(__file__).parents[1] / 'testdata'
    for name, count in [('final_method_documents', 87), ('final_recovery_documents', 4)]:
        documents = json.loads(gzip.decompress((root / (name + '.json.gz')).read_bytes()))
        assert len(documents) == count
        for doc in documents:
            parsed = compile_native_protocol(doc)
            assert parsed.to_payload()['metadata']['bms_method_run'] == doc['metadata']['bms_method_run']

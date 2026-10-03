"""Package verified final captures; fail if any matrix row or snapshot is missing."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = Path('/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4/contract-final')
OUT = ROOT / 'testdata/final_method_close'
OUT.mkdir(exist_ok=True)
sha = lambda b: hashlib.sha256(b).hexdigest()
documents = json.loads(gzip.decompress((ROOT / 'testdata/final_method_documents.json.gz').read_bytes()))
recovery = json.loads(gzip.decompress((ROOT / 'testdata/final_recovery_documents.json.gz').read_bytes()))
assert len(documents) == 87 and len(recovery) == 4
assert gzip.decompress((ROOT / 'testdata/final_method_documents.json.gz').read_bytes()) == (EVIDENCE / 'run-documents.json').read_bytes()
assert gzip.decompress((ROOT / 'testdata/final_recovery_documents.json.gz').read_bytes()) == (EVIDENCE / 'recovery-documents.json').read_bytes()

roster = []
for source in sorted(EVIDENCE.glob('matrix-*.xml')) + [EVIDENCE / 'recovery-final.xml', EVIDENCE / 'bms-final-combined.xml']:
    cases = list(ET.parse(source).iter('testcase'))
    failed = [c.attrib['name'] for c in cases if any(x.tag in ('error', 'failure', 'skipped') for x in c)]
    assert not failed, (source, failed)
    roster.append({'file': source.name, 'tests': len(cases), 'failed': 0, 'skipped': 0, 'sha256': sha(source.read_bytes())})
    target = OUT / 'junit' / source.name; target.parent.mkdir(exist_ok=True)
    shutil.copyfile(source, target)
assert sum(r['tests'] for r in roster if r['file'].startswith('matrix-')) == 87

rows, exports = [], []
# A superseded failing run may leave a held capture after a later success.
# Retain those in the audit directory, never include them in the final manifest.
if (OUT / 'exports').exists():
    shutil.rmtree(OUT / 'exports')
for source in sorted((EVIDENCE / 'exports').glob('*.json.gz')):
    capture = json.loads(gzip.decompress(source.read_bytes()))
    if source.name.startswith('final-') and source.name.endswith('-held.json.gz'):
        final = json.loads(gzip.decompress(source.with_name(source.name.replace('-held.', '-execution.')).read_bytes()))
        if capture.get('held') not in (final.get('held') or []):
            continue
    assert capture['document'] == capture['result']['protocol']['document']
    assert 'bms_method_run' in capture['document']['metadata']
    target = OUT / 'exports' / source.name; target.parent.mkdir(exist_ok=True)
    shutil.copyfile(source, target)
    exports.append({'file': str(target.relative_to(OUT)), 'sha256': sha(source.read_bytes()),
                    'uncompressed_sha256': sha(gzip.decompress(source.read_bytes())),
                    'job_id': capture['result']['job_id'], 'status': capture['result']['command']['status'],
                    'gate': capture['result']['execution']['runtime_state']['workflow']['gate']})
for i, document in enumerate(documents):
    name = f'final-{i:02d}-execution.json.gz'
    entry = next(x for x in exports if x['file'] == 'exports/' + name)
    capture = json.loads(gzip.decompress((OUT / entry['file']).read_bytes()))
    assert capture['document'] == document
    actions = [a for s in document['stages'] for a in s['actions']]
    rows.append({'index': i, 'compiler_digest': document['metadata']['bms_method']['digest'],
                 'document_sha256': sha(json.dumps(document, sort_keys=True, separators=(',', ':')).encode()),
                 'capture': entry['file'], 'native_status': entry['status'],
                 'actions': [{'action_id': a['action_id'], 'kind': a['kind'], 'params': a['params'],
                              'source_occurrence_id': a['source_occurrence_id']} for a in actions]})

from collections import Counter
manifest = {'schema': 'bioxp.final-method-matrix.v1',
            'native_product_commit': '960caec98518fb84e96dcd339bb514128f52d188',
            'bms_producer_commit': '33b40edd5a945f8b2be602ff2416551cd8e2d125',
            'final_model_documents': len(rows), 'recovery_control_documents': len(recovery),
            'native_status_counts': dict(Counter(r['native_status'] for r in rows)),
            'final_document_sha256': sha(gzip.decompress((ROOT / 'testdata/final_method_documents.json.gz').read_bytes())),
            'recovery_document_sha256': sha(gzip.decompress((ROOT / 'testdata/final_recovery_documents.json.gz').read_bytes())),
            'exports': exports, 'matrix': rows, 'junit': roster,
            'physical_qualification': False,
            'limits': ['Pipette has no clot/air classifier; none modelled',
                       'Native ordinary method documents reject OEM-only deferred pause/wake',
                       'Expected context failures retain exact original document and partial effects',
                       'No native product behavior, automatic Stop, cleanup or retry was changed']}
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(json.dumps({k: manifest[k] for k in ('final_model_documents', 'recovery_control_documents', 'native_status_counts')}, indent=2))
print('exports', len(exports), 'tests', sum(r['tests'] for r in roster))

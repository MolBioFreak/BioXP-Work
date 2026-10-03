import json
from pathlib import Path
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
root = Path(__file__).resolve().parents[2]
out = Path('/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4/contract-final')
results = []
def batch(start):
    names = [f'tests/test_final_method_matrix.py::test_final_document_execution[{i}]' for i in range(start, min(start + 8, 87))]
    with (out / f'matrix-{start}.log').open('w') as log:
        result = subprocess.run([sys.executable, str(Path(__file__).with_name('run-final-contract.py')), *names, '--junitxml=' + str(out / f'matrix-{start}.xml')], cwd=root, stdout=log, stderr=subprocess.STDOUT)
    return {'start': start, 'exit_code': result.returncode}
with ThreadPoolExecutor(max_workers=3) as pool:
    for future in as_completed([pool.submit(batch, start) for start in range(0, 87, 8)]):
        results.append(future.result())
        (out / 'matrix-results.json').write_text(json.dumps(sorted(results, key=lambda r: r['start']), indent=2))
        print(results[-1], flush=True)
raise SystemExit(any(r['exit_code'] for r in results))

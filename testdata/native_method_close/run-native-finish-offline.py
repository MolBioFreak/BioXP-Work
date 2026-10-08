import os, subprocess, sys
root = '/home/dalab/.hermes/profiles/fresh/cache/scratch/bioxp-methods-r4/robot-native-finish'
tmp = '/home/dalab/.hermes/profiles/fresh/cache/scratch/bioxp-methods-r4/native-finish-container-tmp'
evidence = '/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4'
fixture = '/home/dalab/.hermes/profiles/fresh/robot-audit/pipette-closeout-20261001/inputs/retained'
os.makedirs(tmp, exist_ok=True)
cmd = ['docker', 'run', '--rm', '--network', 'none', '--read-only', '--user', f'{os.getuid()}:{os.getgid()}']
for bind in ['/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/derived/oem_runtime_parity_spec_20260719/oem_machine_runtime_bundle_serial206:/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/derived/oem_runtime_parity_spec_20260719/oem_machine_runtime_bundle_serial206:ro', f'{root}:{root}:ro', f'{tmp}:{tmp}', f'{fixture}:{fixture}:ro', f'{evidence}:{evidence}', '/home/dalab/.hermes/profiles/fresh/robot-audit/responsive-fixes/venv/lib/python3.11/site-packages:/host-test-deps:ro']:
    cmd += ['-v', bind]
cmd += ['-w', root]
for env in [f'CAVRO_EVIDENCE_ROOT={evidence}', f'BIOXP_RETAINED_FIXTURE_ROOT={fixture}', f'DECK_RETAINED_BASELINE={fixture}', f'BMS_HOME={root}', f'HOME={tmp}', f'TMPDIR={tmp}', f'PYTHONPATH=src:tests:.:{evidence}/native-test-bootstrap', 'PYTHONDONTWRITEBYTECODE=1', 'PYTEST_DISABLE_PLUGIN_AUTOLOAD=1']:
    cmd += ['-e', env]
args = ['-q', '--tb=short', '-p', 'tests.z_stop_offline_guard', '-p', 'no:cacheprovider', *sys.argv[1:]]
cmd += ['sha256:d1f96045384c92c808560dc49112efa2bac436fde879daa8f39dbebd736b2f93', 'python', '-c', f"import sys; sys.path.append('/host-test-deps'); import pytest; raise SystemExit(pytest.main({args!r}))"]
raise SystemExit(subprocess.run(cmd).returncode)

"""Run one isolated offline module/process; no network or device mounts."""
import os
from pathlib import Path
import subprocess
import sys
root = Path(__file__).resolve().parents[2]
runner = Path(__file__).with_name('run-native-finish-offline.py').read_text()
runner = runner.replace("root = '/home/dalab/.hermes/profiles/fresh/cache/scratch/bioxp-methods-r4/robot-native-finish'", f'root = {str(root)!r}')
runner = runner.replace('/native-finish-container-tmp', '/final-contract-container-tmp')
runner = runner.replace("evidence = '/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4'", "evidence = '/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4/contract-final'")
Path('/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4/contract-final').mkdir(exist_ok=True)
# Bootstrap retains the already established dependency-only path.
runner = runner.replace('{evidence}/native-test-bootstrap', '/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4/native-test-bootstrap')
runner = runner.replace("for bind in [", "for bind in ['/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4/native-test-bootstrap:/home/dalab/.hermes/profiles/fresh/robot-audit/bioxp-workflow-implementation-r4/native-test-bootstrap:ro', ")
exec(compile(runner, __file__, 'exec'))

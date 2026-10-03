"""Offline image runner for the sole native-close worktree."""
from pathlib import Path
source = Path(__file__).with_name('run-native-finish-offline.py')
exec(compile(source.read_text().replace('/robot-native-finish', '/robot-native-close').replace('/native-finish-container-tmp', '/native-close-container-tmp'), str(source), 'exec'))

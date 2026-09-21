"""Launcher selection regression; no application import or hardware access."""
import subprocess
import tempfile
from pathlib import Path


def test_release_container_reuse():
    source = (Path(__file__).resolve().parents[1] / 'scripts/bioxp_release_container_run.sh').read_text()
    block = '# uDocker run IMAGE' + source.split('# uDocker run IMAGE', 1)[1].split('\nSOURCE_VOLUME=', 1)[0]
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        store = root / 'store'
        (store / 'containers').mkdir(parents=True)
        repo = store / 'repos/repo/tag'
        repo.mkdir(parents=True)
        (repo / 'container.json').write_text('{"identity":"test"}')
        stub = root / 'udocker'
        stub.write_text("""#!/usr/bin/python3
import sys
from pathlib import Path
s = Path(sys.argv[1].split('=', 1)[1])
name = sys.argv[3].split('=', 1)[1]
c = s / 'containers/created'
c.mkdir()  # A second creation would fail this test.
(c / 'ROOT').mkdir()
(c / 'imagerepo.name').write_text('repo:tag')
(c / 'container.json').write_text((s / 'repos/repo/tag/container.json').read_text())
(s / 'containers' / name).symlink_to('created')
""")
        stub.chmod(0o700)
        script = f'set -eu\nUDOCKER_ROOT={root}\nUDOCKER_BIN={stub}\nIMAGE_ID=sha256:test\nIMAGE_REF=repo:tag\nfail() {{ exit 78; }}\n' + block

        def run():
            return subprocess.run(['bash', '-c', script], capture_output=True, text=True)

        cold = run()
        assert cold.returncode == 0, cold.stderr
        assert run().returncode == 0, 'warm reuse attempted creation'
        container = store / 'containers/created'
        (container / 'imagerepo.name').write_text('other:tag')
        assert run().returncode != 0
        (container / 'imagerepo.name').write_text('repo:tag')
        (container / 'container.json').write_text('{}')
        assert run().returncode != 0
        (store / 'containers/bioxp-test').unlink()
        (store / 'containers/bioxp-test').symlink_to(root)
        assert run().returncode != 0


if __name__ == '__main__':
    test_release_container_reuse()
    print('5/5: cold create, warm reuse, wrong image, wrong metadata, path escape')

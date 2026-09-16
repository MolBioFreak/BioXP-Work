"""Image inspection follows OCI base-to-top layers, not uDocker ancestry."""
import hashlib
import io
import json
import tarfile
from pathlib import Path

import pytest
from scripts import bioxp_udocker_image_inspector as inspector


def layer(store, files):
    data = io.BytesIO()
    with tarfile.open(fileobj=data, mode='w') as archive:
        for name, raw in files.items():
            member = tarfile.TarInfo(name)
            member.size = len(raw)
            archive.addfile(member, io.BytesIO(raw))
    raw = data.getvalue()
    digest = hashlib.sha256(raw).hexdigest()
    (store / 'layers' / (digest + '.layer')).write_bytes(raw)
    return digest


def manifest(content):
    digest = hashlib.sha256(content).hexdigest()
    row = {'path': 'src/example.py', 'size': len(content), 'sha256': digest}
    aggregate = hashlib.sha256(b'src/example.py\0' + str(len(content)).encode() + b'\0' + digest.encode() + b'\n').hexdigest()
    return inspector.canonical_bytes({'schema': 'bioxp.release.source_manifest.v1', 'status': 'verified',
        'commit': 'a' * 40, 'tree': 'b' * 40, 'root': '/app',
        'inventory_algorithm': 'sorted-utf8-path-size-sha256-v1',
        'aggregate_algorithm': 'sha256(path_utf8+nul+decimal_size+nul+file_sha256+lf)',
        'file_count': 1, 'total_size': len(content), 'aggregate_sha256': aggregate, 'files': [row]})


def image(tmp_path):
    store = tmp_path / 'store'
    (store / 'layers').mkdir(parents=True)
    tag = store / 'repos/example/latest'
    tag.mkdir(parents=True)
    current = b'current-source\n'
    source_manifest = manifest(current)
    lower = layer(store, {'app/src/example.py': b'old-source\n',
        'usr/share/bioxp-release/source-manifest.json': b'old-manifest\n',
        'app/removed.py': b'removed\n'})
    middle = layer(store, {'usr/share/bioxp-release/.wh.source-manifest.json': b'',
        'app/.wh.removed.py': b''})
    upper = layer(store, {'app/src/example.py': current,
        'usr/share/bioxp-release/source-manifest.json': source_manifest})
    order = [lower, middle, upper]
    config = {'rootfs': {'type': 'layers', 'diff_ids': ['sha256:' + d for d in order]},
        'config': {'Labels': dict(zip(inspector.REQUIRED_LABELS,
            ['a' * 40, 'b' * 40, hashlib.sha256(source_manifest).hexdigest()]))}}
    raw = inspector.canonical_bytes(config)
    (tag / 'container.json').write_bytes(raw)
    (tag / 'ancestry').write_text(json.dumps(list(reversed(order))))
    external = tmp_path / 'source-manifest.json'
    external.write_bytes(source_manifest)
    return store, tag, config, 'sha256:' + hashlib.sha256(raw).hexdigest(), external


def test_measure_authenticates_current_overlay_and_manifest(tmp_path):
    store, tag, config, image_id, external = image(tmp_path)
    actual, labels = inspector._measure(store, image_id, external)
    assert actual == json.loads(external.read_bytes())
    assert labels[inspector.REQUIRED_LABELS[0]] == 'a' * 40
    dest = tmp_path / 'materialized'
    inspector._materialize_udocker_rootfs(store, tag, dest, config)
    assert (dest / 'app/src/example.py').read_bytes() == b'current-source\n'
    assert not (dest / 'app/removed.py').exists()
    assert not list(dest.rglob('.wh.*'))


@pytest.mark.parametrize('mutation', ['missing', 'extra', 'duplicate'])
def test_rejects_mismatched_ancestry_membership(tmp_path, mutation):
    store, tag, config, image_id, external = image(tmp_path)
    ancestry = json.loads((tag / 'ancestry').read_text())
    if mutation == 'missing':
        ancestry.pop()
    elif mutation == 'extra':
        ancestry.append('f' * 64)
    else:
        ancestry[0] = ancestry[1]
    (tag / 'ancestry').write_text(json.dumps(ancestry))
    with pytest.raises(RuntimeError, match='layer membership'):
        inspector._measure(store, image_id, external)


def test_rejects_corrupt_layer_bytes(tmp_path):
    store, tag, config, image_id, external = image(tmp_path)
    digest = config['rootfs']['diff_ids'][0][7:]
    (store / 'layers' / (digest + '.layer')).write_bytes(b'corrupt')
    with pytest.raises(RuntimeError, match='absent or corrupt'):
        inspector._measure(store, image_id, external)

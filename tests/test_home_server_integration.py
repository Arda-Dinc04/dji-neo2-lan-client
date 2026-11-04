"""Offline source installation refuses unknown changes before any writes."""
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

spec=importlib.util.spec_from_file_location('home_server_integration',
    Path(__file__).resolve().parents[1]/'tools/apply_home_server_control.py')
integration=importlib.util.module_from_spec(spec)
spec.loader.exec_module(integration)


@pytest.fixture
def source(tmp_path,monkeypatch):
    package=tmp_path/'package';repo=tmp_path/'repo';target=tmp_path/'target'
    (package/'files').mkdir(parents=True);target.mkdir()
    for name in ('favicon.svg','camera.css','flight.css','flight.js','flight.html'):
        path=repo/'src/neo2_control/web'/name;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text('fixture-ui' if name!='flight.html' else
            '__NEO2_TOKEN__ <meta name="neo2-auth" content="local"><meta name="neo2-api-prefix" content="/api/flight">')
    original='apps/web/src/routes/drone/+page.svelte'
    path=target/original;path.parent.mkdir(parents=True);path.write_text('legacy-video-page')
    path=package/'files'/original;path.parent.mkdir(parents=True);path.write_text('new-control-page')
    (package/'files/new-controller.ts').write_text('new-controller')
    baseline={'files':{original:hashlib.sha256(b'legacy-video-page').hexdigest()},
        'preserve':{original:original.replace('/drone/','/drone/video/')}}
    (package/'baseline.json').write_text(json.dumps(baseline))
    monkeypatch.setattr(integration,'REPO',repo);monkeypatch.setattr(integration,'PACKAGE',package)
    return target,package,original


def test_install_and_update_preserve_the_original_video_page(source):
    root,package,original=source
    integration.apply(root)
    preserved=root/original.replace('/drone/','/drone/video/')
    assert preserved.read_text()=='legacy-video-page'
    (package/'files'/original).write_text('updated-control-page')
    integration.apply(root)
    assert preserved.read_text()=='legacy-video-page'
    assert (root/original).read_text()=='updated-control-page'


def test_check_is_inert_and_installation_is_idempotent(source):
    root,_,_=source
    before={str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}
    assert integration.apply(root,check=True)['modified'] is False
    assert before=={str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}
    integration.apply(root);integration.apply(root)
    state=json.loads((root/integration.STATE).read_text())
    assert state['format']==integration.FORMAT
    assert all(hashlib.sha256((root/name).read_bytes()).hexdigest()==value for name,value in state['files'].items())


def test_changed_baseline_is_refused_before_creating_any_new_file(source):
    root,_,original=source;(root/original).write_text('unrelated-user-change')
    with pytest.raises(ValueError):integration.apply(root)
    assert not (root/'new-controller.ts').exists() and not (root/integration.STATE).exists()


def test_preexisting_unknown_new_file_is_preserved(source):
    root,_,_=source;(root/'new-controller.ts').write_text('unrelated-controller')
    with pytest.raises(ValueError):integration.apply(root)
    assert (root/'new-controller.ts').read_text()=='unrelated-controller'
    assert not (root/integration.STATE).exists()


def test_symlink_escape_is_refused_before_source_writes(source,tmp_path):
    root,_,_=source;outside=tmp_path/'outside';outside.mkdir()
    (root/'apps/web/static').symlink_to(outside,target_is_directory=True)
    with pytest.raises(ValueError,match='symlink'):integration.apply(root)
    assert not list(outside.iterdir()) and not (root/'new-controller.ts').exists()

"""CLI credential and client wiring; mission runs against an in-memory fixture."""
import json

import pytest

from neo2_control.cli import build_parser,main
import neo2_control.cloud_operator_client as cloud
import neo2_control.program_mission as mission
from test_program_mission import Client,PROGRAM


def test_cloud_cli_uses_private_token_without_printing_or_journaling_it(tmp_path,monkeypatch,capsys):
    token='fixture.cli.secret';credentials=tmp_path/'token';credentials.write_text(token);credentials.chmod(0o600)
    program=tmp_path/'program.json';program.write_text(json.dumps(PROGRAM))
    journal=tmp_path/'mission.jsonl';client=Client();client.server_offset=1_000_000.
    received=[]
    def construct(url,value):received.append((url,value));return client
    monkeypatch.setattr(cloud,'CloudFlightOperatorClient',construct)
    run=mission.run_program_mission
    monkeypatch.setattr(mission,'run_program_mission',lambda c,p,**kw:run(c,p,
        clock=lambda:client.now,wait=client.wait,**kw))
    code=main(['run-program',str(program),'--cloud-api','https://server.invalid/home-server/api/drone/control',
        '--token-file',str(credentials),'--operator-ready','--journal',str(journal)])
    captured=capsys.readouterr()
    assert code==2 and json.loads(captured.out)['motion_complete']
    assert received==[('https://server.invalid/home-server/api/drone/control',token)]
    assert token not in captured.out+captured.err+journal.read_text()
    assert journal.stat().st_mode&0o777==0o600


@pytest.mark.parametrize('options,reason',[
    (['--cloud-api','https://server.invalid/api/control'],'cloud_token_file_required'),
    (['--token-file','unused-token'],'cloud_token_file_requires_cloud_api')])
def test_missing_credential_pair_fails_before_file_or_network(tmp_path,capsys,options,reason):
    journal=tmp_path/'never-created.jsonl'
    assert main(['run-program','missing.json',*options,'--operator-ready','--journal',str(journal)])==1
    assert reason in capsys.readouterr().err and not journal.exists()


def test_cloud_and_loopback_url_cannot_be_selected_together():
    with pytest.raises(SystemExit):
        build_parser().parse_args(['run-program','p.json','--url','http://127.0.0.1:8794',
            '--cloud-api','https://server.invalid/api/control','--operator-ready','--journal','j.jsonl'])

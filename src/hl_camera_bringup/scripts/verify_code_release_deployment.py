"""Verify code-only transfer, failed-start rollback and restart persistence on OAK."""
import argparse
import json
from pathlib import Path
import shutil
import time

from deploy_code_release import APP, ROOT, Remote, build_bundle
from release_runtime import code_files, RUNTIME_FILES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app-id',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    remote=Remote('C:/Program Files/oakctl/bin/oakctl.exe','192.168.201.2',args.app_id)
    initial=remote.control('status')
    bundle=args.output/'code1.zip'
    manifest=build_bundle(APP,'0.33.10-code1',bundle)
    started=time.monotonic()
    deployed=remote.deploy(bundle)
    deployed['elapsed_s']=time.monotonic()-started
    (args.output/'deployed.json').write_text(json.dumps(deployed,indent=2))
    # Fault injection changes only a staged code release, never installed libraries.
    fixture=args.output/'failed-start-fixture';fixture.mkdir(exist_ok=True)
    for name in set(code_files(APP))|set(RUNTIME_FILES)|{'oakapp.toml'}:
        shutil.copy2(APP/name,fixture/name)
    (fixture/'main.py').write_text("raise RuntimeError('Intentional code-release rollback verification')\n")
    failed_bundle=args.output/'failed-start.zip'
    failed_manifest=build_bundle(fixture,'intentional-failed-start',failed_bundle)
    try:
        remote.deploy(failed_bundle)
    except RuntimeError as exc:
        failure=str(exc)
    else:raise AssertionError('An intentionally failing release was accepted')
    recovered=remote.wait(manifest['release'])
    assert recovered['state']['active']==manifest['release']
    assert failed_manifest['release'] in recovered['state']['last_failure']
    remote.restart()
    restarted=remote.wait(manifest['release'])
    assert restarted['frame']['runtime_fingerprint']==initial['runtime']
    result=dict(ok=True,initial=initial,bundle_bytes=bundle.stat().st_size,
                deploy_elapsed_s=deployed['elapsed_s'],release=manifest['release'],
                expected_failure=failure,recovered=recovered,restarted=restarted)
    (args.output/'verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(ok=True,bundle_bytes=result['bundle_bytes'],
                         deploy_elapsed_s=result['deploy_elapsed_s'],release=manifest['release'])))


if __name__=='__main__':main()

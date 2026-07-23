#!/usr/bin/env python3
"""Verify the static BioXP OEM movement source/binary/config registry."""
from __future__ import annotations
import argparse, hashlib, json, re, sys
from pathlib import Path

DEFAULT_REGISTRY = Path(__file__).resolve().parents[1] / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"
REQUIRED_ANCHORS = {
    "initializeEnvironment", "motion_thread_process", "initializeSystem",
    "initialCheck", "checkDoorStatus", "initializeMotion", "initializeMotorsWithoutMotion",
    "initializeMotors", "queryTipStatus", "ejectAllTips", "verifyEjectTip", "initiateGroup",
    "selftest", "TCSelfTest", "RCSelfTest", "OCSelfTest", "CheckCamera", "AdjustCamera",
    "SnapshotImage", "inspectCover", "InspectOutputLocation", "checkRCCover",
    "checkCoverStorage", "checkChillerCover", "catchPlate", "releasePlate", "parkGantry",
    "doorOpen", "unlockDoor", "openThermalDoor", "closeThermalDoor", "scriptmoveTo",
    "moveTo", "moveX", "moveY", "moveZ", "moveSteps", "HomeAxis", "HomeXY",
    "goHome", "axisSearchHome", "doorSearchHome", "queryHome", "queryRightSensor",
    "stopMotor", "setHome", "setSpeedAcc", "setMaxCurrent", "setStallGuardThreshold",
    "TransmitMessage", "sendCommand", "transmitCommand", "ProcessReceivedTrafficPacket",
    "handleReturnMessageNovo", "setChillerCoolRate", "setChillerHeatRate",
    "readChillerTemperature", "setGP", "readGP", "checkLabel", "matchPattern",
    "loadConfig", "loadOperationParameters", "ForceToHighHome",
    "getCurrentPosition", "CloseGripper", "setChillerPWM",
    "GetLidTemperature", "GetThermalTemperature", "getNestTemp", "GetTCPedestalTemp",
    "turnOffHeater", "SetTemperature", "readRCTemperature", "readOCTemperature",
    "setLIDTemperature", "setThermalTemperature", "setChilerTemperature",
    "KeepTip", "terminatecommands", "queryIndividualTipStatus",
}

def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def strip_line(line: str, in_block: bool) -> tuple[str, bool]:
    out=[]; i=0; quote=None
    while i < len(line):
        if in_block:
            j=line.find('*/', i)
            if j < 0: return ''.join(out), True
            i=j+2; in_block=False; continue
        if quote:
            c=line[i]
            if c=='\\': i+=2; continue
            if c==quote: quote=None
            i+=1; continue
        if line.startswith('//', i): break
        if line.startswith('/*', i): in_block=True; i+=2; continue
        c=line[i]
        if c in ('"', "'"): quote=c; i+=1; continue
        out.append(c); i+=1
    return ''.join(out), in_block

def brace_end(lines: list[str], start: int) -> int:
    depth=0; opened=False; in_block=False
    for idx in range(start-1, len(lines)):
        clean,in_block=strip_line(lines[idx],in_block)
        for c in clean:
            if c=='{': depth+=1; opened=True
            elif c=='}' and opened:
                depth-=1
                if depth==0: return idx+1
        if not opened and ';' in clean: return idx+1
    return len(lines)

def check_field_line(record: dict, files: dict[str, Path], errors: list[str]) -> None:
    artifact=record['artifact']; path=files.get(artifact)
    if path is None: errors.append(f"configuration artifact not found in live corpus: {artifact}"); return
    lines=path.read_text(errors='replace').splitlines()
    if 'line' not in record: return
    line_no=record['line']
    if not (1 <= line_no <= len(lines)):
        errors.append(f"{artifact}:{line_no} outside file"); return
    line=lines[line_no-1]
    value=record['value']
    values=[]
    if isinstance(value,dict): values=list(value.values())
    elif isinstance(value,list): values=value
    elif isinstance(value,(str,int,float,bool)): values=[value]
    for v in values:
        token=str(v)
        if isinstance(v,bool): token='True' if v else 'False'
        if token not in line and value != 'selected camera-control block':
            errors.append(f"{artifact}:{line_no} lacks selected value {token!r} for {record['field']}")

def verify(registry_path: Path) -> dict:
    errors=[]; d=json.loads(registry_path.read_text())
    source_by={s['source_id']:s for s in d['sources']}
    binary_ids={b['binary_id'] for b in d['binaries']}
    for b in d['binaries']:
        p=Path(b['path'])
        if not p.is_file(): errors.append(f"missing binary: {p}"); continue
        if p.stat().st_size != b['size']: errors.append(f"binary size mismatch: {p}")
        if sha256(p) != b['sha256']: errors.append(f"binary hash mismatch: {p}")
    lines_by_source={}
    for s in d['sources']:
        p=Path(s['absolute_path'])
        if not p.is_file(): errors.append(f"missing source: {p}"); continue
        if s['binary_id'] not in binary_ids: errors.append(f"unknown binary for source {s['source_id']}")
        if sha256(p) != s['sha256']: errors.append(f"source hash mismatch: {p}")
        lines=p.read_text(errors='replace').splitlines(); lines_by_source[s['source_id']]=lines
        if len(lines) != s['line_count']: errors.append(f"line count mismatch: {p}")
    seen=set(); names=set()
    for m in d['methods']:
        if m['method_id'] in seen: errors.append(f"duplicate method_id: {m['method_id']}")
        seen.add(m['method_id']); names.add(m['name'])
        s=source_by.get(m['source_id'])
        if s is None: errors.append(f"unknown source_id: {m['source_id']}"); continue
        if m['binary_id'] != s['binary_id']: errors.append(f"method/source binary mismatch: {m['method_id']}")
        lines=lines_by_source.get(m['source_id']); start=m['start_line']; end=m['end_line']
        if not lines or not (1 <= start <= end <= len(lines)):
            errors.append(f"invalid range: {m['method_id']} {start}-{end}"); continue
        if m['kind'] in {'method','constructor'}:
            if lines[start-1].strip() != m['declaration']:
                errors.append(f"declaration mismatch: {m['method_id']}")
            actual_end=brace_end(lines,start)
            if actual_end != end: errors.append(f"brace end mismatch: {m['method_id']} expected {end}, got {actual_end}")
    missing=sorted(REQUIRED_ANCHORS-names)
    if missing: errors.append('missing required call-graph anchors: '+', '.join(missing))
    lock_path=Path(d['authority']['evidence_lock_path']); lock=json.loads(lock_path.read_text())
    acq_root=Path(lock['acquisition']['root']); live_files={}
    records=d['live_machine_corpus']['records']
    if len(records) != 19: errors.append(f"live corpus count is {len(records)}, expected 19")
    for r in records:
        p=acq_root/r['relative_path']; live_files[Path(r['logical_relative_path']).name]=p
        if not p.is_file(): errors.append(f"missing live corpus artifact: {p}"); continue
        if p.stat().st_size != r['size']: errors.append(f"live artifact size mismatch: {p}")
        if sha256(p) != r['sha256']: errors.append(f"live artifact hash mismatch: {p}")
    config=d['selected_machine_configuration']
    expected_hashes={'config.xml':config['config_sha256'],'Operation_parameters.xml':config['operation_parameters_sha256'],'InspectionSettings.xml':config['inspection_settings_sha256']}
    for name,expected in expected_hashes.items():
        p=live_files.get(name)
        if not p or sha256(p)!=expected: errors.append(f"selected artifact hash mismatch: {name}")
    for r in config['field_records']: check_field_line(r,live_files,errors)
    config_xml=live_files.get('config.xml')
    if config_xml:
        pos_lines=config_xml.read_text(errors='replace').splitlines()[30:59]
        if len(pos_lines)!=29 or not all(re.match(r'\s*<[A-Z0-9_]+\s',x) for x in pos_lines):
            errors.append('selected PositionTable is not exactly 29 entries at config.xml lines 31-59')
    return {'ok':not errors,'errors':errors,'counts':d['counts'],'required_anchor_count':len(REQUIRED_ANCHORS),'registry':str(registry_path)}

def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument('registry',nargs='?',type=Path,default=DEFAULT_REGISTRY); ap.add_argument('--json',action='store_true')
    args=ap.parse_args(); result=verify(args.registry)
    if args.json: print(json.dumps(result,indent=2,sort_keys=True))
    else:
        print('OEM_MOVEMENT_REGISTRY=' + ('PASS' if result['ok'] else 'FAIL'))
        print(json.dumps(result['counts'],sort_keys=True))
        print(f"required_anchors={result['required_anchor_count']}")
        for e in result['errors']: print('ERROR:',e)
    return 0 if result['ok'] else 1
if __name__=='__main__': sys.exit(main())

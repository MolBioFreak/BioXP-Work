#!/usr/bin/env python3
"""Verify the static BioXP OEM movement source/binary/config registry."""
from __future__ import annotations
import argparse, hashlib, json, re, sys
import xml.etree.ElementTree as ET
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
    "m_canControl_handleLatchEvent", "m_canControl_handleEnclosureDoorEvent",
    "m_canControl_handleEnclosureDoorEventProcess", "wakefrompause",
    "queryDoorStatus", "rehome", "startup", "ReadBarcode", "homeGripper",
    "moveXY", "getMidPoint", "setZaxisCurrentmax", "OpenGripper",
    "SetTCTemperature", "SetLidTemperature", "setOutputChillerTemp",
    "setReagentChillerTemp", "resumeTemperature", "waitForBoard",
    "SaveImage", "locateCover", "setExposure", "setAutoExposure",
    "createSetting", "loadMasterPosition", "readParameters", "GetCameraSettings",
    "InspectionItems", "locationID", "positionStruct", "wellID", "plateName", "OperationMode",
    "MainWindowInitialize", "loadStatus", "WritePacket", "RxThread", "Encode", "Decode",
    "ToBytes", "SendDataViaBulkTransfer", "ReceiveDataViaBulkTransfer", "ScreenResolutionHigh",
}
REQUIRED_SOURCE_IDS = {
    "main", "control", "cci", "pipettes", "pipette", "ican", "novo", "novousb",
    "caninterfaceboard", "novoencoding", "canpacket", "icanpacket", "winusb",
    "base", "io", "deck", "head", "thermal", "chiller", "thermalctl", "motor",
    "settings", "statuslog", "camerasettings", "inspectionitems", "locationid",
    "positionstruct", "wellid", "platename", "operationmode", "defaults", "vision",
}
REQUIRED_EDGE_IDS = {
    "app_load_status", "app_start_motion_worker", "app_initial_environment",
    "environment_initial_check", "motion_update_gate", "motion_dispatch_initialize",
    "initialize_ship_state", "initialize_fresh_initial_check", "initialize_saved_state",
    "initialize_motion", "initialize_selftest", "selftest_delegate", "initialize_camera",
    "initialize_cover", "initialize_park", "initialize_job", "prepare_job_startup",
    "motion_initialize_motors", "motion_query_tips", "motion_eject_tips",
    "motion_pipette_initiate", "motion_pipette_check", "nomotion_wait_boards",
    "wait_activate_boards", "camera_profile", "camera_label", "camera_park",
    "cover_selected_lowres", "cover_profile", "cover_native_location", "cover_barcode",
    "novo_owner_to_usb", "novo_send_to_transmit", "novo_transmit_packet",
    "novo_transmit_wire", "novo_direct_send_wire", "wire_packet_bytes", "wire_encode",
    "wire_bulk_send", "wire_bulk_receive", "wire_decode", "wire_receive_packet",
    "settings_load_config", "settings_load_operation", "settings_select_screen_profile",
    "settings_derive_x_selftest", "status_restore_saved_state",
}
REQUIRED_SEQUENCE_IDS = {"initialize_motors_direct_oem", "initialize_motion_stale_tip", "novo_usb_tx_frame"}
REQUIRED_HAZARDS = {
    "NOVO_PIPETTE_MASK_IMPOSSIBLE_PROJECTION", "MOTOR_NULL_SUCCESS_CONFLATION",
    "THERMAL_NULL_AND_STATUS_CONFLATION", "ROUTE_ALWAYS_TRUE_LOCATION_TEST",
    "MOVE_XY_WRONG_NULL_BRANCH", "DOOR_NULL_AND_CACHED_STATE_SUCCESS",
    "SETTINGS_SERIAL_SETTER_BYPASS", "SETTINGS_SELFTEST_AXIS_CROSS_ASSIGNMENT",
    "SETTINGS_PERMISSIVE_FALLBACK", "MOTION_WORKER_NO_FINALLY",
    "INITIALIZE_GRIPPER_NULL_GUARD_ORDER", "TURN_OFF_HEATER_DUPLICATE_TARGET",
    "SELFTEST_ASYNC_REPORTING_AND_TIMEOUT_MODE", "VISION_INVALID_IL_REGIONS",
    "CAMERA_GAIN_UNUSED_BY_ADJUSTER", "DOOR_24V_POLARITY_PROJECTION",
    "NOVO_DECODE_NO_CHECKSUM_VALIDATION",
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

def scalar_token(value: object) -> str:
    if isinstance(value, bool): return 'true' if value else 'false'
    return str(value)

def check_field_line(record: dict, files: dict[str, Path], errors: list[str]) -> None:
    artifact=record['artifact']; path=files.get(artifact)
    if path is None: errors.append(f"configuration artifact not found in live corpus: {artifact}"); return
    lines=path.read_text(errors='replace').splitlines()
    if 'line' in record:
        line_no=record['line']
        if not (1 <= line_no <= len(lines)):
            errors.append(f"{artifact}:{line_no} outside file"); return
        line=lines[line_no-1]
        location=str(line_no)
    elif 'line_range' in record:
        start,end=record['line_range']
        if not (1 <= start <= end <= len(lines)):
            errors.append(f"{artifact}:{start}-{end} outside file"); return
        line='\n'.join(lines[start-1:end])
        location=f"{start}-{end}"
    else:
        errors.append(f"configuration record lacks line provenance: {record['field']}"); return
    selector=record.get('selector','')
    value=record['value']
    # Attribute selectors are verified as an exact attribute/value pair, not a loose token.
    if '/@' in selector and not isinstance(value,(dict,list)):
        attr=selector.rsplit('/@',1)[1]
        token=scalar_token(value)
        if re.search(rf'\b{re.escape(attr)}\s*=\s*["\']{re.escape(token)}["\']',line,re.I) is None:
            errors.append(f"{artifact}:{location} lacks exact selector attribute {attr}={token!r} for {record['field']}")
        element=selector.rsplit('/@',1)[0].rstrip('/').rsplit('/',1)[-1]
        if element and f'<{element}' not in line:
            errors.append(f"{artifact}:{location} lacks selector element <{element} for {record['field']}")
        return
    # Camera-profile selectors must identify the exact profile key and every named field.
    key_match=re.search(r'\[Key=([^\]]+)\]',selector)
    if key_match:
        key=key_match.group(1)
        ns=r'(?:[A-Za-z0-9_]+:)?'
        if re.search(rf'<{ns}Key>\s*{re.escape(key)}\s*</{ns}Key>',line,re.I) is None:
            errors.append(f"{artifact}:{location} lacks profile key {key!r} for {record['field']}")
        if isinstance(value,dict):
            for name,v in value.items():
                token=scalar_token(v)
                direct=re.search(rf'<{re.escape(name)}>\s*{re.escape(token)}\s*</{re.escape(name)}>',line,re.I)
                parameter=re.search(rf'<{ns}Key>\s*{re.escape(name)}\s*</{ns}Key>.*?<{ns}Value[^>]*>\s*{re.escape(token)}\s*</{ns}Value>',line,re.I|re.S)
                if direct is None and parameter is None:
                    errors.append(f"{artifact}:{location} lacks exact profile field {name}={token!r} for {record['field']}")
        return
    # Non-derived path selectors must resolve to the exact XML element name.
    if selector.startswith('/'):
        element=selector.rstrip('/').rsplit('/',1)[-1]
        if element and not element.startswith('*') and f'<{element}' not in line:
            errors.append(f"{artifact}:{location} lacks selector element <{element} for {record['field']}")
    if record['field']=='PositionTable': return
    values=list(value.values()) if isinstance(value,dict) else value if isinstance(value,list) else [value]
    for v in values:
        token=scalar_token(v)
        if token.lower() not in line.lower():
            errors.append(f"{artifact}:{location} lacks selected value {token!r} for {record['field']}")

def verify(registry_path: Path) -> dict:
    errors=[]; d=json.loads(registry_path.read_text())
    if d.get('schema_id') != 'bioxp.oem_movement_method_source_binary_registry.v3':
        errors.append(f"unexpected registry schema: {d.get('schema_id')}")
    lock_path=Path(d['authority']['evidence_lock_path']); lock=json.loads(lock_path.read_text())
    lock_map={p:b for b,paths in lock['decompile_binary_map'].items() for p in paths}
    source_by={s['source_id']:s for s in d['sources']}
    missing_sources=sorted(REQUIRED_SOURCE_IDS-set(source_by))
    if missing_sources: errors.append('missing required source IDs: '+', '.join(missing_sources))
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
        status=s.get('binary_map_status'); evidence=s.get('binary_mapping_evidence',{})
        rel=s['relative_path']
        if rel in lock_map:
            if lock_map[rel] != s['binary_id']:
                errors.append(f"canonical source/binary mismatch: {rel} -> {lock_map[rel]}, registry {s['binary_id']}")
            if status!='canonical_evidence_lock' or evidence.get('kind')!='evidence_lock_decompile_binary_map':
                errors.append(f"canonical mapping evidence not used: {s['source_id']}")
            if evidence.get('source_relative_path')!=rel or evidence.get('binary_id')!=s['binary_id']:
                errors.append(f"canonical mapping evidence fields mismatch: {s['source_id']}")
        else:
            if status!='registry_extension_from_captured_assembly' or evidence.get('kind')!='decompiled_project_assembly_name':
                errors.append(f"extension mapping lacks project AssemblyName evidence: {s['source_id']}")
            project=Path(evidence.get('path','')); line_no=evidence.get('line')
            if not project.is_file() or not isinstance(line_no,int):
                errors.append(f"invalid project mapping evidence path/line: {s['source_id']}")
            else:
                project_lines=project.read_text(errors='replace').splitlines()
                assembly=evidence.get('assembly_name')
                expected=f'<AssemblyName>{assembly}</AssemblyName>'
                if not (1 <= line_no <= len(project_lines)) or expected not in project_lines[line_no-1]:
                    errors.append(f"project AssemblyName evidence mismatch: {s['source_id']}")
                expected_binary=assembly + ('.exe' if s['binary_id'].endswith('.exe') else '.dll')
                if expected_binary != s['binary_id'] or evidence.get('binary_id')!=s['binary_id']:
                    errors.append(f"project assembly/binary identity mismatch: {s['source_id']}")
    seen=set(); names=set(); method_by={}
    for m in d['methods']:
        if m['method_id'] in seen: errors.append(f"duplicate method_id: {m['method_id']}")
        seen.add(m['method_id']); names.add(m['name']); method_by[m['method_id']]=m
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
    edge_ids=set()
    for edge in d.get('required_call_edges',[]):
        edge_id=edge.get('edge_id')
        if edge_id in edge_ids: errors.append(f"duplicate required edge: {edge_id}")
        edge_ids.add(edge_id)
        caller=method_by.get(edge.get('caller_method_id')); callee=method_by.get(edge.get('callee_method_id'))
        if caller is None: errors.append(f"edge caller not registered: {edge_id} -> {edge.get('caller_method_id')}"); continue
        if callee is None: errors.append(f"edge callee/member not registered: {edge_id} -> {edge.get('callee_method_id')}")
        call_line=edge.get('call_line'); lines=lines_by_source.get(caller['source_id'],[])
        if not isinstance(call_line,int) or not (caller['start_line'] <= call_line <= caller['end_line']):
            errors.append(f"edge call line outside exact caller: {edge_id}"); continue
        token=edge.get('callee_token','')
        if call_line > len(lines) or token not in lines[call_line-1]:
            errors.append(f"edge source token mismatch: {edge_id} at {caller['source_id']}:{call_line}")
    missing_edges=sorted(REQUIRED_EDGE_IDS-edge_ids)
    if missing_edges: errors.append('missing required exact call edges: '+', '.join(missing_edges))
    sequence_ids=set()
    for sequence in d.get('ordered_call_sequences',[]):
        sequence_id=sequence.get('sequence_id')
        if sequence_id in sequence_ids: errors.append(f"duplicate ordered sequence: {sequence_id}")
        sequence_ids.add(sequence_id)
        caller=method_by.get(sequence.get('caller_method_id'))
        if caller is None: errors.append(f"ordered sequence caller not registered: {sequence_id}"); continue
        lines=lines_by_source.get(caller['source_id'],[]); previous=caller['start_line']-1
        for step in sequence.get('steps',[]):
            line_no=step.get('line'); token=step.get('token','')
            if not isinstance(line_no,int) or line_no <= previous or not (caller['start_line'] <= line_no <= caller['end_line']):
                errors.append(f"ordered sequence line/order mismatch: {sequence_id} {line_no}"); continue
            previous=line_no
            if line_no > len(lines) or token not in lines[line_no-1]:
                errors.append(f"ordered sequence source token mismatch: {sequence_id} at {caller['source_id']}:{line_no}")
    missing_sequences=sorted(REQUIRED_SEQUENCE_IDS-sequence_ids)
    if missing_sequences: errors.append('missing required ordered sequences: '+', '.join(missing_sequences))
    missing=sorted(REQUIRED_ANCHORS-names)
    if missing: errors.append('missing required call-graph anchors: '+', '.join(missing))
    hazard_ids={h.get('hazard_id') for h in d.get('known_hazards',[])}
    missing_hazards=sorted(REQUIRED_HAZARDS-hazard_ids)
    if missing_hazards: errors.append('missing required source hazards: '+', '.join(missing_hazards))
    if d.get('counts',{}).get('binaries') != len(d['binaries']): errors.append('binary count mismatch')
    if d.get('counts',{}).get('sources') != len(d['sources']): errors.append('source count mismatch')
    if d.get('counts',{}).get('methods_and_members') != len(d['methods']): errors.append('method/member count mismatch')
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
        root=ET.parse(config_xml).getroot(); position_table=root.find('PositionTable'); axis_limits=root.find('AxisLimits')
        if position_table is None or axis_limits is None:
            errors.append('selected config lacks PositionTable or AxisLimits for self-test derivation')
        else:
            x=y=z=0
            for element in list(position_table):
                nx=int(element.attrib.get('x',0)); ny=int(element.attrib.get('y',0)); nz=int(element.attrib.get('zLow',0))
                x=x if x>nx else nx
                # Preserve the literal OEM cross-assignment projection at ClassBioXPSettings.cs:3449-3450.
                y=x if y>ny else ny
                z=x if z>nz else nz
            limit_by={element.tag:int(element.attrib['maxSteps']) for element in list(axis_limits)}
            raw={'X':x,'Y':y,'Z':z}
            clamped={'X':min(x,limit_by['X_limit']),'Y':min(y,limit_by['Y_limit']),'Z':min(z,limit_by['Z_limit'])}
            fields={r['field']:r for r in config['field_records']}
            raw_record=fields.get('SelfTestTravel.raw_source_projection',{})
            clamp_record=fields.get('SelfTestTravel.clamped',{})
            if raw_record.get('value')!=raw or raw_record.get('derivation_source')!='ClassBioXPSettings.cs:3387-3450':
                errors.append(f"self-test raw derivation mismatch: expected {raw}")
            if clamp_record.get('value')!=clamped or clamp_record.get('derivation_source')!='ClassBioXPSettings.cs:1277-1281,3387-3450':
                errors.append(f"self-test clamped derivation mismatch: expected {clamped}")
            settings_lines=lines_by_source.get('settings',[])
            exact_settings={
                1277:'Math.Min(m_xSelfTestTravel, XHighLimit)', 1279:'Math.Min(m_ySelfTestTravel, YHighLimit)',
                1281:'Math.Min(m_zSelfTestTravel, ZHighLimit)', 3448:'m_xSelfTestTravel',
                3449:'m_ySelfTestTravel = ((m_ySelfTestTravel > num16) ? m_xSelfTestTravel : num16)',
                3450:'m_zSelfTestTravel = ((m_zSelfTestTravel > num17) ? m_xSelfTestTravel : num17)',
            }
            for line_no,token in exact_settings.items():
                if line_no>len(settings_lines) or token not in settings_lines[line_no-1]:
                    errors.append(f"self-test derivation source mismatch: settings:{line_no}")
    return {'ok':not errors,'errors':errors,'counts':d['counts'],'required_anchor_count':len(REQUIRED_ANCHORS),'required_edge_count':len(REQUIRED_EDGE_IDS),'ordered_sequence_count':len(REQUIRED_SEQUENCE_IDS),'registry':str(registry_path)}

def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument('registry',nargs='?',type=Path,default=DEFAULT_REGISTRY); ap.add_argument('--json',action='store_true')
    args=ap.parse_args(); result=verify(args.registry)
    if args.json: print(json.dumps(result,indent=2,sort_keys=True))
    else:
        print('OEM_MOVEMENT_REGISTRY=' + ('PASS' if result['ok'] else 'FAIL'))
        print(json.dumps(result['counts'],sort_keys=True))
        print(f"required_anchors={result['required_anchor_count']}")
        print(f"required_edges={result['required_edge_count']}")
        print(f"ordered_sequences={result['ordered_sequence_count']}")
        for e in result['errors']: print('ERROR:',e)
    return 0 if result['ok'] else 1
if __name__=='__main__': sys.exit(main())

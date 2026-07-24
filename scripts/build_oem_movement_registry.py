#!/usr/bin/env python3
from __future__ import annotations
import hashlib, json, re
from pathlib import Path

ROOT=Path('/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup')
LOCK_PATH=Path('/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/derived/oem_runtime_parity_spec_20260719/OEM_EVIDENCE_LOCK.json')
OUT=Path('/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/bioxp_historical_run_forensics_20260717/derived/linux_runtime_acquisition_20260718/bioxp_re_working_copy/docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json')
index=json.loads(Path('/tmp/bioxp_method_index.json').read_text())+json.loads(Path('/tmp/default_parameters_index.json').read_text())
lock=json.loads(LOCK_PATH.read_text())

BINARIES={x['name']:x['sha256'] for x in lock['captured_clickonce_binaries']}
click_root=Path(lock['acquisition']['root'])/lock['clickonce_binary_root_relative']

SOURCE_DEFS={
 'main':('decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs','GenBotApp.exe','application_lifecycle'),
 'control':('decompiled_src/BioXPControlLib/ControlLib.cs','BioXPControlLib.dll','control_lifecycle'),
 'cci':('decompiled_src/BioXPControlLib/ClassControlInterface.cs','BioXPControlLib.dll','movement_orchestration'),
 'pipettes':('decompiled_src/BioXPControlLib/ClassPipetteCollection.cs','BioXPControlLib.dll','pipette_collection'),
 'pipette':('decompiled_src_can/BioXPControlLib/ClassPipette.cs','BioXPControlLib.dll','pipette_channel'),
 'ican':('decompiled_src_can/ClassCanLib/InterfaceCAN.cs','ClassCanLib.dll','transport_contract'),
 'novo':('decompiled_src_can/ClassCanLib/ClassNovo.cs','ClassCanLib.dll','transport_owner'),
 'novousb':('decompiled_src_novo/NovoCANUSBLib/ClassNovoCANUSB.cs','NovoCANUSBLib.dll','usb_can_transport'),
 'caninterfaceboard':('decompiled_src_novodevices/Novo/Devices/CanInterfaceBoard.cs','Novo.Devices.dll','novo_wire_interface'),
 'novoencoding':('decompiled_src_novodevices/Novo/Devices/NovoEncoding.cs','Novo.Devices.dll','novo_wire_framing'),
 'canpacket':('decompiled_src_novodevices/Novo/Devices/CAN/Interfaces/CanPacket.cs','Novo.Devices.dll','novo_can_packet'),
 'icanpacket':('decompiled_src_novodevices/Novo/Devices/CAN/Interfaces/ICanPacket.cs','Novo.Devices.dll','novo_can_packet_contract'),
 'winusb':('decompiled_src_novodevices/WinUsb/WinUsbCommunications.cs','Novo.Devices.dll','novo_usb_bulk_transport'),
 'base':('decompiled_src_can/ClassCanLib/ClassBaseBoard.cs','ClassCanLib.dll','board_base'),
 'io':('decompiled_src_can/ClassCanLib/ClassIOControl.cs','ClassCanLib.dll','interlock_io'),
 'deck':('decompiled_src_can/ClassCanLib/ClassDeckBoard.cs','ClassCanLib.dll','deck_axis_board'),
 'head':('decompiled_src_can/ClassCanLib/ClassHeadBoard.cs','ClassCanLib.dll','head_axis_board'),
 'thermal':('decompiled_src_can/ClassCanLib/ClassThermalBoard.cs','ClassCanLib.dll','thermal_axis_board'),
 'chiller':('decompiled_src_can/ClassCanLib/ClassChillerBoard.cs','ClassCanLib.dll','chiller_board'),
 'thermalctl':('decompiled_src_can/ClassCanLib/ClassThermalControl.cs','ClassCanLib.dll','thermal_chiller_control'),
 'motor':('decompiled_src_can/ClassCanLib/ClassMotor.cs','ClassCanLib.dll','motor_primitive'),
 'settings':('decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs','BioXPCommonLib.dll','machine_settings'),
 'statuslog':('decompiled_src_bioxpcommon/BioXPCommonLib/ClassStatusLog.cs','BioXPCommonLib.dll','persistent_oem_application_state'),
 'camerasettings':('decompiled_src_bioxpcommon/BioXPCommonLib/CameraSettings.cs','BioXPCommonLib.dll','camera_profile_selection'),
 'inspectionitems':('decompiled_src_bioxpcommon/BioXPCommonLib/InspectionItems.cs','BioXPCommonLib.dll','camera_inspection_ordinal_map'),
 'locationid':('decompiled_src_commonlib/CommonLib/locationID.cs','CommonLib.dll','route_location_identifier_map'),
 'positionstruct':('decompiled_src_commonlib/CommonLib/positionStruct.cs','CommonLib.dll','route_coordinate_payload'),
 'wellid':('decompiled_src_commonlib/CommonLib/wellID.cs','CommonLib.dll','well_identifier_map'),
 'platename':('decompiled_src_bioxpcommon/BioXPCommonLib/plateName.cs','BioXPCommonLib.dll','cover_plate_state_map'),
 'operationmode':('decompiled_src_bioxpcommon/BioXPCommonLib/OperationMode.cs','BioXPCommonLib.dll','terminal_start_mode_map'),
 'defaults':('decompiled_src_bioxpcommon/BioXPCommonLib/DefaultParameters.cs','BioXPCommonLib.dll','global_motion_policy'),
 'vision':('decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs','CVisionLib.dll','vision_primitive'),
}

NAMES={
 'main':['BioXPMainWindow','MainWindowInitialize','frmMain_Loaded','initializeEnvironment','initializeSystem','SelfTest','PrepareToRunJob','motion_thread_process','UpdateCheck','m_pageWarning_buttonclicked','m_canControl_handleLatchEvent','m_canControl_handleEnclosureDoorEvent','m_canControl_handleEnclosureDoorEventProcess','wakefrompause','m_pageSoftwareUpdate_Cancel_Click','btnFetch_Click'],
 'control':['AdjustCamera','AllLEDOff','CheckCamera','SnapshotImage','inspectCover','InspectOutputLocation','checkRCCover','checkCoverStorage','checkChillerCover','catchPlate','releasePlate','parkGantry','doorOpen','checkDoorStatus','queryDoorStatus','initialCheck','initializeMotion','unlockDoor','setLEDColor','selftest','TCSelfTest','RCSelfTest','OCSelfTest','waitforcompletion','forceAbortMotion','setLIDTemperature','setThermalTemperature','setChilerTemperature','rehome','startup','ReadBarcode','homeGripper'],
 'cci':['ClassControlInterface','openThermalDoor','closeThermalDoor','confirmAxis','initializeMotorsWithoutMotion','initializeMotors','setChillerCoolRate','activateBoard','deactivateBoard','moveTo','scriptmoveTo','moveSteps','moveX','moveY','moveZ','moveXY','moveAxis','MoveZHome','HomeAxis','HomeXY','setGripperCurrent','setColor','query24voltage','querydoorsensor','querylatchsensor','querySolenoidControl','setSolenoidControl','getCurrentPosition','getMidPoint','setZaxisCurrentmax','OpenGripper','CloseGripper','setChillerPWM','GetLidTemperature','GetThermalTemperature','getNestTemp','GetTCPedestalTemp','turnOffHeater','SetTemperature','SetTCTemperature','SetLidTemperature','setOutputChillerTemp','setReagentChillerTemp','readRCTemperature','readOCTemperature','resumeTemperature','forceAbortMotion','waitForBoard'],
 'pipettes':['ClassPipetteCollection','initiateGroup','checkedPipetteStatus','ejectAllTips','ejectTip','KeepTip','verifyEjectTip','terminatecommands','queryTipStatus','queryIndividualTipStatus','waitforcompletion','enablePressureStream'],
 'pipette':['ClassPipette','initiate','ejectTip','QueryStatus','QueryTipStatus','enablePressureStream','SendCommand','processMessage'],
 'novo':['ClassNovo','startService','close','MessageProcessingThread','ClassNovo_GotMessage','GotMessageProcess','TransmitMessage','calculatePressureoffset'],
 'novousb':['ClassNovoCANUSB','UpdateDevices','OnReplyPacketAvailable','SendData','SendRaw','sendCommand','transmitCommand','ProcessReceivedTrafficPacket','IsAMatch','IsAMatchPipette','IsAMatchPipetteUIM','filterMassage','calculatePressureoffset','closeDevice'],
 'caninterfaceboard':['CanInterfaceBoard','WritePacket','Reply','StartReadingPackets','StopReadingPackets','Connect','Disconnect','Dispose','RxThread'],
 'novoencoding':['Encode','Decode','EscapesFound','EscapesNeeded','EscapeNeeded','Checksum'],
 'canpacket':['CanPacket','ToBytes'],
 'winusb':['CloseDeviceHandle','GetDeviceHandle','InitializeDevice','ReceiveDataViaBulkTransfer','SendDataViaBulkTransfer'],
 'base':['ClassBaseBoard','deactivateBoard','activateBoard','handleReturnMessageNovo','forceAbortMotion','setStallGuardThreshold','doorSearchHome','goHome','moveToAbs','setSpeedAcc','setSpeed','setMaxCurrent','disableRightSwitch','setAxisLimits','stopMotor','setRdivPdiv','disableLeftSwitch','queryHome','queryRightSensor','axisSearchHome','setHome','readMaxCurrent','moveSteps','GetWaitEventhandle','setMaxAcc','startFanService','stopFanService'],
 'io':['ClassIOControl','setLED','query24VSensor','querDoorSensor','querySolenoidControl','querLatchSensor','setSolenoidControl'],
 'deck':['ClassDeckBoard','goHome','setHome','setStallGuardThreshold','setMaxCurrent','readMaxCurrent','setSpeedAcc','setRdivPdiv','moveToAbs','moveSteps','stopMotor','axisSearchHome','queryHome','queryRightSensor','checkMotorStopped','setAxisLimits','setSpeed','disableRightSwitch','disableLeftSwitch','setColor'],
 'head':['ClassHeadBoard','goHome','setHome','setStallGuardThreshold','setMaxCurrent','readMaxCurrent','setSpeedAcc','setRdivPdiv','moveToAbs','moveSteps','stopMotor','axisSearchHome','queryHome','queryRightSensor','checkMotorStopped','setAxisLimits','setSpeed','disableRightSwitch','disableLeftSwitch'],
 'thermal':['ClassThermalBoard','setHome','setStallGuardThreshold','setMaxCurrent','readMaxCurrent','setSpeedAcc','setRdivPdiv','moveToAbs','moveSteps','stopMotor','doorSearchHome','axisSearchHome','queryHome','queryRightSensor','checkMotorStopped','setAxisLimits','setSpeed','disableRightSwitch','disableLeftSwitch'],
 'chiller':['ClassChillerBoard','activateBoard','setChillerTemp','chillerFan','readChillerFanSpeed','readChillerTemperature','readChillerPedestalTemperature','setChillerCoolRate','setChillerHeatRate','readChillerCoolRate','readChillerHeatRate','readChillerCurrent','readChillerNestTemp','setChillerPWM','setChillerPedTemp','setChillerMaxCurrent','setChillerProportional','setChillerFeedForward','handleReturnMessageNovo','startFanService','stopFanService','SetTimerInterval','resumeTempSetting'],
 'thermalctl':['ClassThermalControl','setChillerTemperature','turnChillerFanOnOff','readChillerTemperature','setGP','readGP','getCoolRampRate','getHeatRampRate','getTECCurrent','readNestTemp','getLidHeatRampRate','startFanService'],
 'motor':['ClassMotor','MoveLeft','MoveRight','StopMotor','disableRightLimitSwitch','disableLeftLimitSwitch','MovetoRelPosition','MoveHome','moveToAbs','setMaxSpeed','setMaxAcc','setMaxCurrent','readMaxCurrent','setHome','setRampMode','setStallGuardThreshold','queryActualPosition','queryMotorSpeed','queryReachedPosition','queryLeftSwitchStatus','queryRightSwitchStatus','queryMotorStop','setPDIV','setRDIV','setLimits','beyondLimit','atTarget'],
 'settings':['ClassBioXPSettings','createSetting','loadOperationParameters','loadConfig','loadMasterPosition','readParameters'],
 'statuslog':['setStatus','disableWriting','updateSN','updateShipMode','updateAccessKey','updateMACAddress','updateStartMode','updateSelfTestDate','updatePreviousJobId','updatePreviousJobName','saveStatus','saveStatusAsJson','loadStatus'],
 'camerasettings':['GetCameraSettings'],
 'defaults':['ForceToHighHome'],
 'vision':['ClassFrameGrabber','SaveImage','locateCover','checkLabel','matchPattern','setExposure','setAutoExposure'],
}

MANUAL=[
 ('cci','ClassControlInterface.m_AxisIODesignater','data_member',29,111,'Logical axis/board/axis designator map'),
 ('ican','InterfaceCAN','interface',3,32,'Interface and events'),
 ('ican','InterfaceCAN.CAN_READY','property',5,5,'Transport readiness property'),
 ('ican','InterfaceCAN.TransmitMessage','interface_method',19,19,'Serialized CAN request/reply contract'),
 ('ican','InterfaceCAN.startService','interface_method',21,21,'Transport service start'),
 ('ican','InterfaceCAN.trigDoorOpenEvent','interface_method',23,23,'Door event projection'),
 ('ican','InterfaceCAN.trigBoardErrorEvent','interface_method',25,25,'Board error projection'),
 ('ican','InterfaceCAN.trigLatchEvent','interface_method',27,27,'Latch event projection'),
 ('ican','InterfaceCAN.calculatePressureoffset','interface_method',29,29,'Pipette pressure offset dispatch'),
 ('ican','InterfaceCAN.close','interface_method',31,31,'Transport close'),
 ('icanpacket','ICanPacket','interface',5,14,'CAN packet data-length/module/payload contract'),
 ('icanpacket','ICanPacket.Data','interface_member',7,7,'Payload bytes'),
 ('icanpacket','ICanPacket.DataLength','interface_member',9,9,'Wire payload length'),
 ('icanpacket','ICanPacket.ModuleId','interface_member',11,11,'Wire module identifier'),
 ('icanpacket','ICanPacket.ToBytes','interface_method',13,13,'Payload serialization contract'),
 ('settings','ClassBioXPSettings.SerialNumber','property',768,788,'Serial-dependent door defaults'),
 ('settings','ClassBioXPSettings.movement_configuration_fields','data_member',946,970,'Selected door/Z movement property surface'),
 ('settings','ClassBioXPSettings.StartMode','property',995,1005,'Selected terminal startup mode'),
 ('settings','ClassBioXPSettings.axis_limits_and_selftest','data_member',1277,1299,'Selected self-test maxima and axis-limit properties'),
 ('settings','ClassBioXPSettings.PositionTable','property',1389,1389,'Selected route table property'),
 ('settings','ClassBioXPSettings.ScreenResolutionHigh','property',1135,1145,'Selected camera/cover algorithm profile'),
 ('statuslog','ClassStatusLog.ShipMode','property',74,74,'Runtime shipping-mode branch input'),
 ('statuslog','ClassStatusLog.LatestStatus','property',82,82,'Current application status'),
 ('statuslog','ClassStatusLog.SelfTestDate','property',86,86,'Daily self-test branch input'),
 ('statuslog','ClassStatusLog.SavedStatus','property',94,94,'Recovered startup-state branch input'),
 ('defaults','DefaultParameters.PSUDO_Z_HOME','data_member',47,59,'Pseudo-Z constants and current selected pseudo-home'),
 ('inspectionitems','InspectionItems','enum',3,24,'Camera inspection ordinal map'),
 ('locationid','locationID','enum',3,52,'Logical route/location identifier map'),
 ('positionstruct','positionStruct','struct',3,18,'Route coordinate payload'),
 ('wellid','wellID','enum',3,102,'Well-to-row/column identifier map'),
 ('platename','plateName','enum',3,27,'Cover/plate state identifier map'),
 ('operationmode','OperationMode','enum',3,9,'StartMode terminal branch values'),
 ('pipettes','ClassPipetteCollection.TipExist','property',90,101,'Four-channel tip predicate'),
 ('pipette','ClassPipette.TipLoaded','property',117,127,'Per-channel tip state'),
 ('pipette','ClassPipette.CommandCompleted','property',129,139,'Per-channel completion state'),
]

def rel_of(index_path:str)->str:
 p=Path(index_path)
 return p.relative_to(ROOT).as_posix()
idx_by_rel={rel_of(x['path']):x for x in index}
lock_map={p:b for b,paths in lock['decompile_binary_map'].items() for p in paths}
PROJECT_FILES={
 'decompiled_src/':'decompiled_src/BioXPControlLib.csproj',
 'decompiled_src_genbotapp/':'decompiled_src_genbotapp/GenBotApp.exe.csproj',
 'decompiled_src_can/':'decompiled_src_can/ClassCanLib.dll.csproj',
 'decompiled_src_novo/':'decompiled_src_novo/NovoCANUSBLib.dll.csproj',
 'decompiled_src_novodevices/':'decompiled_src_novodevices/Novo.Devices.dll.csproj',
 'decompiled_src_bioxpcommon/':'decompiled_src_bioxpcommon/BioXPCommonLib.dll.csproj',
 'decompiled_src_commonlib/':'decompiled_src_commonlib/CommonLib.dll.csproj',
 'decompiled_src_vision/':'decompiled_src_vision/CVisionLib.dll.csproj',
}

sources=[]
for key,(rel,binary,role) in SOURCE_DEFS.items():
 p=ROOT/rel
 assert p.exists(), p
 sha=hashlib.sha256(p.read_bytes()).hexdigest()
 expected=idx_by_rel[rel]['sha256'] if rel in idx_by_rel else sha
 assert sha==expected
 map_status='canonical_evidence_lock' if lock_map.get(rel)==binary else 'registry_extension_from_captured_assembly'
 if map_status=='canonical_evidence_lock':
  mapping_evidence={'kind':'evidence_lock_decompile_binary_map','path':str(LOCK_PATH),'binary_id':binary,'source_relative_path':rel}
 else:
  prefix=next((x for x in PROJECT_FILES if rel.startswith(x)),None)
  assert prefix is not None,(rel,'no project evidence')
  project_rel=PROJECT_FILES[prefix]; project=ROOT/project_rel
  assembly=binary.removesuffix('.dll').removesuffix('.exe')
  project_lines=project.read_text(errors='replace').splitlines()
  assembly_line=next(i for i,x in enumerate(project_lines,1) if f'<AssemblyName>{assembly}</AssemblyName>' in x)
  mapping_evidence={'kind':'decompiled_project_assembly_name','path':str(project),'relative_path':project_rel,'line':assembly_line,'assembly_name':assembly,'binary_id':binary}
 sources.append({
  'source_id':key,'relative_path':rel,'absolute_path':str(p),'sha256':sha,
  'line_count':len(p.read_text(errors='replace').splitlines()),'binary_id':binary,
  'binary_map_status':map_status,'binary_mapping_evidence':mapping_evidence,
  'role':role,
 })

binary_records=[]
for name,sha in BINARIES.items():
 p=click_root/name
 assert p.exists(),p
 actual=hashlib.sha256(p.read_bytes()).hexdigest()
 assert actual==sha,(name,actual,sha)
 binary_records.append({'binary_id':name,'path':str(p),'sha256':sha,'size':p.stat().st_size})

methods=[]
for source_id,names in NAMES.items():
 rel=SOURCE_DEFS[source_id][0]
 f=idx_by_rel[rel]
 lines=(ROOT/rel).read_text(errors='replace').splitlines()
 for m in f['methods']:
  if m['name'] not in names: continue
  seg=lines[m['start_line']-1:m['end_line']]
  warnings=[m['start_line']+i for i,line in enumerate(seg) if re.search(r'IL_|Unknown result type|Invalid comparison|Expected [OI]',line)]
  sleeps=[]
  for i,line in enumerate(seg):
   for hit in re.finditer(r'Thread\.Sleep\((\d+)\)',line): sleeps.append({'line':m['start_line']+i,'milliseconds':int(hit.group(1))})
  mid=f"{'.'.join(f['classes'][:1])}.{m['name']}@{m['start_line']}"
  methods.append({
   'method_id':mid,'source_id':source_id,'binary_id':SOURCE_DEFS[source_id][1],
   'name':m['name'],'kind':m['kind'],'start_line':m['start_line'],'end_line':m['end_line'],
   'declaration':m['declaration'],'literal_sleeps':sleeps,
   'decompiler_warning_lines':warnings,'status':'present_hash_locked_and_line_bounded',
  })
for source_id,member,kind,start,end,role in MANUAL:
 methods.append({'method_id':member,'source_id':source_id,'binary_id':SOURCE_DEFS[source_id][1],
  'name':member.split('.')[-1],'kind':kind,'start_line':start,'end_line':end,'declaration':role,
  'literal_sleeps':[],'decompiler_warning_lines':[],'status':'present_hash_locked_and_line_bounded'})

# Ensure every requested name resolved unless it was manually represented.
for sid,names in NAMES.items():
 found={m['name'] for m in methods if m['source_id']==sid}
 missing=sorted(set(names)-found)
 assert not missing,(sid,missing)

required_call_edges=[
 ('app_load_status','BioXPMainWindow.MainWindowInitialize@375','ClassStatusLog.loadStatus@249',581,'ClassStatusLog.loadStatus'),
 ('app_start_motion_worker','BioXPMainWindow.MainWindowInitialize@375','BioXPMainWindow.motion_thread_process@2030',600,'motion_thread_process'),
 ('app_initial_environment','BioXPMainWindow.MainWindowInitialize@375','BioXPMainWindow.initializeEnvironment@973',821,'initializeEnvironment'),
 ('environment_initial_check','BioXPMainWindow.initializeEnvironment@973','ControlLib.initialCheck@8728',978,'initialCheck'),
 ('environment_can_ready','BioXPMainWindow.initializeEnvironment@973','InterfaceCAN.CAN_READY',976,'CAN_READY'),
 ('environment_queue_initialize','BioXPMainWindow.initializeEnvironment@973','BioXPMainWindow.initializeSystem@1046',994,'name = "initializeSystem"'),
 ('motion_update_gate','BioXPMainWindow.motion_thread_process@2030','BioXPMainWindow.UpdateCheck@4264',2046,'UpdateCheck'),
 ('motion_dispatch_initialize','BioXPMainWindow.motion_thread_process@2030','BioXPMainWindow.initializeSystem@1046',2048,'initializeSystem'),
 ('initialize_ship_state','BioXPMainWindow.initializeSystem@1046','ClassStatusLog.ShipMode',1127,'ClassStatusLog.ShipMode'),
 ('initialize_fresh_initial_check','BioXPMainWindow.initializeSystem@1046','ControlLib.initialCheck@8728',1143,'initialCheck'),
 ('initialize_saved_state','BioXPMainWindow.initializeSystem@1046','ClassStatusLog.SavedStatus',1144,'ClassStatusLog.SavedStatus'),
 ('initialize_saved_recovery_motion','BioXPMainWindow.initializeSystem@1046','ControlLib.initializeMotion@8797',1148,'initializeMotion'),
 ('initialize_motion','BioXPMainWindow.initializeSystem@1046','ControlLib.initializeMotion@8797',1159,'initializeMotion'),
 ('initialize_selftest','BioXPMainWindow.initializeSystem@1046','BioXPMainWindow.SelfTest@1562',1167,'SelfTest'),
 ('selftest_delegate','BioXPMainWindow.SelfTest@1562','ControlLib.selftest@10688',1564,'selftest'),
 ('initialize_camera','BioXPMainWindow.initializeSystem@1046','ControlLib.CheckCamera@1929',1172,'CheckCamera'),
 ('initialize_cover','BioXPMainWindow.initializeSystem@1046','ControlLib.inspectCover@3663',1184,'inspectCover'),
 ('initialize_park','BioXPMainWindow.initializeSystem@1046','ControlLib.parkGantry@7071',1188,'parkGantry'),
 ('initialize_job','BioXPMainWindow.initializeSystem@1046','BioXPMainWindow.PrepareToRunJob@1567',1253,'PrepareToRunJob'),
 ('prepare_job_startup','BioXPMainWindow.PrepareToRunJob@1567','ControlLib.startup@8872',1579,'startup'),
 ('motion_initialize_motors','ControlLib.initializeMotion@8797','ClassControlInterface.initializeMotors@3348',8803,'initializeMotors'),
 ('warning_update_failure_queue_initialize','BioXPMainWindow.m_pageWarning_buttonclicked@2611','BioXPMainWindow.initializeSystem@1046',2622,'name = "initializeSystem"'),
 ('warning_initial_check','BioXPMainWindow.m_pageWarning_buttonclicked@2611','ControlLib.initialCheck@8728',2628,'initialCheck'),
 ('warning_initialize_motion','BioXPMainWindow.m_pageWarning_buttonclicked@2611','ControlLib.initializeMotion@8797',2635,'initializeMotion'),
 ('door_event_queue_wakefrompause','BioXPMainWindow.m_canControl_handleEnclosureDoorEventProcess@2428','BioXPMainWindow.wakefrompause@2103',2448,'wakefrompause'),
 ('wake_initial_check','BioXPMainWindow.wakefrompause@2103','ControlLib.initialCheck@8728',2105,'initialCheck'),
 ('wake_rehome','BioXPMainWindow.wakefrompause@2103','ControlLib.rehome@8784',2106,'rehome'),
 ('rehome_initialize_motors','ControlLib.rehome@8784','ClassControlInterface.initializeMotors@3348',8787,'initializeMotors'),
 ('door_event_queue_initialize','BioXPMainWindow.m_canControl_handleEnclosureDoorEventProcess@2428','BioXPMainWindow.initializeSystem@1046',2493,'initializeSystem'),
 ('software_update_cancel_queue_initialize','BioXPMainWindow.m_pageSoftwareUpdate_Cancel_Click@2861','BioXPMainWindow.initializeSystem@1046',2866,'name = "initializeSystem"'),
 ('fetch_queue_initialize','BioXPMainWindow.btnFetch_Click@4050','BioXPMainWindow.initializeSystem@1046',4056,'name = "initializeSystem"'),
 ('initmotors_z_home','ClassControlInterface.initializeMotors@3348','ClassBaseBoard.axisSearchHome@204',3352,'axisSearchHome'),
 ('initmotors_gripper_current','ClassControlInterface.initializeMotors@3348','ClassControlInterface.setGripperCurrent@5392',3354,'setGripperCurrent'),
 ('initmotors_gripper_clear','ClassControlInterface.initializeMotors@3348','ClassControlInterface.moveSteps@4165',3355,'moveSteps'),
 ('initmotors_x_home','ClassControlInterface.initializeMotors@3348','ClassBaseBoard.axisSearchHome@204',3369,'axisSearchHome'),
 ('initmotors_x_set_home','ClassControlInterface.initializeMotors@3348','ClassBaseBoard.setHome@209',3371,'setHome'),
 ('initmotors_x_speed','ClassControlInterface.initializeMotors@3348','ClassBaseBoard.setSpeed@166',3372,'setSpeed'),
 ('initmotors_x_park','ClassControlInterface.initializeMotors@3348','ClassControlInterface.moveX@4206',3374,'moveX'),
 ('initmotors_y_home','ClassControlInterface.initializeMotors@3348','ClassBaseBoard.axisSearchHome@204',3378,'axisSearchHome'),
 ('initmotors_door_home','ClassControlInterface.initializeMotors@3348','ClassBaseBoard.doorSearchHome@148',3382,'doorSearchHome'),
 ('initmotors_door_predicate','ClassControlInterface.initializeMotors@3348','ClassControlInterface.confirmAxis@2714',3384,'confirmAxis'),
 ('initmotors_door_failure_open','ClassControlInterface.initializeMotors@3348','ClassControlInterface.openThermalDoor@2651',3386,'openThermalDoor'),
 ('initmotors_y_set_home','ClassControlInterface.initializeMotors@3348','ClassBaseBoard.setHome@209',3391,'setHome'),
 ('initmotors_output_chiller','ClassControlInterface.initializeMotors@3348','ClassControlInterface.setChillerCoolRate@3423',3414,'setChillerCoolRate'),
 ('initmotors_reagent_chiller','ClassControlInterface.initializeMotors@3348','ClassControlInterface.setChillerCoolRate@3423',3415,'setChillerCoolRate'),
 ('initmotors_status','ClassControlInterface.initializeMotors@3348','ClassStatusLog.setStatus@96',3416,'ClassStatusLog.setStatus'),
 ('initmotors_gripper_idle','ClassControlInterface.initializeMotors@3348','ClassControlInterface.setGripperCurrent@5392',3419,'setGripperCurrent'),
 ('motion_query_tips','ControlLib.initializeMotion@8797','ClassPipetteCollection.queryTipStatus@1336',8805,'queryTipStatus'),
 ('motion_eject_tips','ControlLib.initializeMotion@8797','ClassPipetteCollection.ejectAllTips@1176',8814,'ejectAllTips'),
 ('motion_pipette_initiate','ControlLib.initializeMotion@8797','ClassPipetteCollection.initiateGroup@677',8832,'initiateGroup'),
 ('motion_pipette_check','ControlLib.initializeMotion@8797','ClassPipetteCollection.checkedPipetteStatus@726',8833,'checkedPipetteStatus'),
 ('nomotion_wait_boards','ClassControlInterface.initializeMotorsWithoutMotion@3181','ClassControlInterface.waitForBoard@3507',3183,'waitForBoard'),
 ('wait_activate_boards','ClassControlInterface.waitForBoard@3507','ClassControlInterface.activateBoard@3474',3528,'activateBoard'),
 ('camera_profile','ControlLib.CheckCamera@1929','CameraSettings.GetCameraSettings@318',1933,'GetCameraSettings'),
 ('camera_label','ControlLib.CheckCamera@1929','ClassFrameGrabber.checkLabel@9284',1938,'checkLabel'),
 ('camera_park','ControlLib.CheckCamera@1929','ControlLib.parkGantry@7071',1958,'parkGantry'),
 ('cover_selected_lowres','ControlLib.inspectCover@3663','ControlLib.checkChillerCover@3891',3682,'checkChillerCover'),
 ('cover_profile','ControlLib.checkChillerCover@3891','CameraSettings.GetCameraSettings@318',3901,'GetCameraSettings'),
 ('cover_native_location','ControlLib.checkChillerCover@3891','ClassFrameGrabber.locateCover@5376',3924,'locateCover'),
 ('cover_barcode','ControlLib.checkChillerCover@3891','ControlLib.ReadBarcode@9088',3928,'ReadBarcode'),
 ('novo_owner_to_usb','ClassNovo.TransmitMessage@194','ClassNovoCANUSB.sendCommand@474',204,'sendCommand'),
 ('novo_send_to_transmit','ClassNovoCANUSB.sendCommand@474','ClassNovoCANUSB.transmitCommand@565',477,'transmitCommand'),
 ('novo_transmit_packet','ClassNovoCANUSB.transmitCommand@565','CanPacket.CanPacket@28',588,'new CanPacket'),
 ('novo_transmit_wire','ClassNovoCANUSB.transmitCommand@565','CanInterfaceBoard.WritePacket@43',619,'WritePacket'),
 ('novo_direct_send_wire','ClassNovoCANUSB.SendData@430','CanInterfaceBoard.WritePacket@43',444,'WritePacket'),
 ('wire_packet_bytes','CanInterfaceBoard.WritePacket@43','CanPacket.ToBytes@34',52,'ToBytes'),
 ('wire_encode','CanInterfaceBoard.WritePacket@43','NovoEncoding.Encode@11',53,'NovoEncoding.Encode'),
 ('wire_bulk_send','CanInterfaceBoard.WritePacket@43','WinUsbCommunications.SendDataViaBulkTransfer@637',56,'SendDataViaBulkTransfer'),
 ('wire_bulk_receive','CanInterfaceBoard.RxThread@137','WinUsbCommunications.ReceiveDataViaBulkTransfer@596',144,'ReceiveDataViaBulkTransfer'),
 ('wire_decode','CanInterfaceBoard.RxThread@137','NovoEncoding.Decode@49',147,'NovoEncoding.Decode'),
 ('wire_receive_packet','CanInterfaceBoard.RxThread@137','CanPacket.CanPacket@28',154,'new CanPacket'),
 ('settings_load_config','ClassBioXPSettings.createSetting@1491','ClassBioXPSettings.loadConfig@2798',1669,'loadConfig'),
 ('settings_load_operation','ClassBioXPSettings.createSetting@1491','ClassBioXPSettings.loadOperationParameters@2516',1702,'loadOperationParameters'),
 ('settings_select_screen_profile','ClassBioXPSettings.loadOperationParameters@2516','ClassBioXPSettings.ScreenResolutionHigh',2702,'ScreenResolutionHigh'),
 ('settings_derive_x_selftest','ClassBioXPSettings.loadConfig@2798','ClassBioXPSettings.axis_limits_and_selftest',3448,'m_xSelfTestTravel'),
 ('status_restore_saved_state','ClassStatusLog.loadStatus@249','ClassStatusLog.SavedStatus',311,'m_savedstatus'),
]
required_call_edges=[{'edge_id':e,'caller_method_id':c,'callee_method_id':d,'call_line':line,'callee_token':token} for e,c,d,line,token in required_call_edges]

# Mandatory branch/terminal semantics for startup admission. These are source facts,
# not permissions to execute the corresponding physical operation.
required_branch_outcomes=[
 {'branch_id':'environment_can_ready_admission','caller_method_id':'BioXPMainWindow.initializeEnvironment@973','line':976,'token':'m_control.m_canControl.CAN_READY','outcome':'only_CAN_READY_enters_initialCheck_and_door_branching'},
 {'branch_id':'environment_not_ready_nonmanual_return','caller_method_id':'BioXPMainWindow.initializeEnvironment@973','line':1007,'token':'(int)m_settingsWindow.StartMode != 0','outcome':'CAN_not_ready_nonmanual_returns_without_initialization'},
 {'branch_id':'environment_not_ready_manual_profile','caller_method_id':'BioXPMainWindow.initializeEnvironment@973','line':1011,'token':'m_settingsWindow.ScreenResolutionHigh','outcome':'CAN_not_ready_manual_selects_menu_by_resolution'},
 {'branch_id':'environment_not_ready_manual_gantry','caller_method_id':'BioXPMainWindow.initializeEnvironment@973','line':1025,'token':'m_control.GantryAvailable = true','outcome':'CAN_not_ready_manual_marks_gantry_available_after_menu_navigation'},
 {'branch_id':'initial_check_can_wait','caller_method_id':'ControlLib.initialCheck@8728','line':8732,'token':'while (!m_canControl.CAN_READY)','outcome':'waits_for_CAN_readiness_before_board_or_door_operations'},
 {'branch_id':'initial_check_can_timeout','caller_method_id':'ControlLib.initialCheck@8728','line':8735,'token':'if (num > 10)','outcome':'returns_false_after_more_than_ten_200ms_wait_iterations'},
 {'branch_id':'initial_check_door_failure','caller_method_id':'ControlLib.initialCheck@8728','line':8751,'token':'if (!checkDoorStatus())','outcome':'returns_false_before_board_reset_when_door_check_fails'},
]

ordered_call_sequences=[
 {'sequence_id':'initialize_motors_direct_oem','caller_method_id':'ClassControlInterface.initializeMotors@3348','steps':[
  {'line':3350,'token':'m_Boards[m_AxisIODesignater["MotorZ"].board] != null','kind':'board_guard'},
  {'line':3352,'token':'axisSearchHome'}, {'line':3354,'token':'setGripperCurrent(31)'}, {'line':3355,'token':'moveSteps'},
  {'line':3356,'token':'m_Boards[m_AxisIODesignater["MotorGrip"].board] != null','kind':'board_guard'},
  {'line':3358,'token':'GripperVersion == 0','kind':'branch_predicate'}, {'line':3360,'token':'axisSearchHome'}, {'line':3364,'token':'axisSearchHome'},
  {'line':3367,'token':'m_Boards[m_AxisIODesignater["MotorX"].board] != null','kind':'board_guard'},
  {'line':3369,'token':'axisSearchHome'}, {'line':3370,'token':'Thread.Sleep(20)'}, {'line':3371,'token':'setHome'},
  {'line':3372,'token':'setSpeed'}, {'line':3373,'token':'Thread.Sleep(40)'}, {'line':3374,'token':'moveX(6000)'},
  {'line':3376,'token':'m_Boards[m_AxisIODesignater["MotorY"].board] != null','kind':'board_guard'},
  {'line':3378,'token':'axisSearchHome'}, {'line':3380,'token':'m_Boards[m_AxisIODesignater["ThermalDoor"].board] != null','kind':'board_guard'},
  {'line':3382,'token':'doorSearchHome'}, {'line':3384,'token':'Convert.ToInt32(m_settingsWindow.SerialNumber) > 9','kind':'branch_predicate'},
  {'line':3386,'token':'openThermalDoor'}, {'line':3387,'token':'throw new Exception','kind':'failure_path'}, {'line':3389,'token':'m_Boards[m_AxisIODesignater["MotorY"].board] != null','kind':'board_guard'},
  {'line':3391,'token':'setHome'}, {'line':3393,'token':'m_settingsWindow.Calibrated','kind':'branch_predicate'},
  {'line':3414,'token':'setChillerCoolRate("OC")'}, {'line':3415,'token':'setChillerCoolRate("RC")'},
  {'line':3416,'token':'ClassStatusLog.setStatus'}, {'line':3417,'token':'m_settingsWindow.GripperVersion == 1','kind':'branch_predicate'},
  {'line':3419,'token':'setGripperCurrent(10)'},]},
 {'sequence_id':'initialize_motion_stale_tip','caller_method_id':'ControlLib.initializeMotion@8797','steps':[
  {'line':8803,'token':'initializeMotors'}, {'line':8805,'token':'queryTipStatus'}, {'line':8806,'token':'Thread.Sleep(500)'},
  {'line':8809,'token':'openThermalDoor'}, {'line':8812,'token':'scriptmoveTo'}, {'line':8814,'token':'ejectAllTips'},
  {'line':8815,'token':'moveZ(80000)'}, {'line':8816,'token':'moveX(79000)'}, {'line':8817,'token':'queryTipStatus'},
  {'line':8818,'token':'Thread.Sleep(100)'}, {'line':8831,'token':'Thread.Sleep(2)'}, {'line':8832,'token':'initiateGroup'},
  {'line':8833,'token':'checkedPipetteStatus'},]},
 {'sequence_id':'novo_usb_tx_frame','caller_method_id':'CanInterfaceBoard.WritePacket@43','steps':[
  {'line':45,'token':'DataLength + 5'}, {'line':46,'token':'GetBytes(packet.ModuleId)'},
  {'line':47,'token':'bytes[3]'}, {'line':48,'token':'bytes[2]'}, {'line':49,'token':'bytes[1]'}, {'line':50,'token':'bytes[0]'},
  {'line':51,'token':'packet.DataLength'}, {'line':52,'token':'ToBytes'}, {'line':53,'token':'NovoEncoding.Encode'},
  {'line':56,'token':'SendDataViaBulkTransfer'},]},
]

# Exact selected live-machine field records.
field_records=[
  {'field':'SerialNumber','value':206,'artifact':'config.xml','line':4,'selector':'/BioXPCommonLib/GenBot/SerialNumber/@GenBot'},
 {'field':'ConfigVersion','value':3,'artifact':'config.xml','line':5,'selector':'/BioXPCommonLib/GenBot/Config/@Version'},
 {'field':'GripperVersion','value':1,'artifact':'config.xml','line':5,'selector':'/BioXPCommonLib/GenBot/Config/@GripperVersion'},
 {'field':'TroughVersion','value':1,'artifact':'config.xml','line':5,'selector':'/BioXPCommonLib/GenBot/Config/@TroughVersion'},
 {'field':'Calibrated','value':1,'artifact':'config.xml','line':6,'selector':'/BioXPCommonLib/GenBot/Calibration/@Calibrated'},
 {'field':'CameraInstalled','value':1,'artifact':'config.xml','line':7,'selector':'/BioXPCommonLib/GenBot/CameraInstalled/@Camera'},
 {'field':'CameraCalibrated','value':True,'artifact':'config.xml','line':7,'selector':'/BioXPCommonLib/GenBot/CameraInstalled/@Cameracalibrated'},
 {'field':'OriginOffsetG','value':4450,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_originOffsetG'},
 {'field':'GripperClosePOS','value':27350,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_GripperClosePOS'},
 {'field':'GripperOpenPOS','value':31400,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_GripperOpenPOS'},
 {'field':'GripperOpenWide','value':32400,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_GripperOpenWide'},
 {'field':'TCDoorOpen','value':18500,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_TCDoorOpen'},
 {'field':'TCDoorStallGuardThreshold','value':6,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_TCDoorStallGuardThreshold'},
 {'field':'TC_DOOR_VELOCITY','value':50,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_TC_DOOR_VELOCITY'},
 {'field':'TC_DOOR_ACCELERATION','value':20,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_TC_DOOR_ACCELERATION'},
 {'field':'TC_DOOR_MAX_CURRENT','value':31,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_TC_DOOR_MAX_CURRENT'},
 {'field':'Z_MOTOR_MAX_CURRENT_DOWN','value':25,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_Z_MOTOR_MAX_CURRENT_DOWN'},
 {'field':'Z_MOTOR_MAX_CURRENT_UP','value':31,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_Z_MOTOR_MAX_CURRENT_UP'},
 {'field':'Z_MOTOR_STALL_GUARD_THRESHOLD','value':3,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@m_Z_MOTOR_STALL_GUARD_THRESHOLD'},
 {'field':'OutPutBufferatMS_Zlow','value':0,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@OutPutBufferatMS_Zlow'},
 {'field':'OutlierRangeFactor','value':4,'artifact':'config.xml','line':15,'selector':'/BioXPCommonLib/CalibrationFactors/Offsets/@OutlierRangeFactor'},
 {'field':'ReagentChiller','value':{'MaxHeatingCurrent':5,'MaxCoolingCurrent':-5,'Proportional':1.5,'FeedForward':400},'artifact':'config.xml','line':17,'selector':'/BioXPCommonLib/CalibrationFactors/ReagentChiller'},
 {'field':'OutputChiller','value':{'MaxHeatingCurrent':5,'MaxCoolingCurrent':-5,'Proportional':1.5,'FeedForward':400},'artifact':'config.xml','line':18,'selector':'/BioXPCommonLib/CalibrationFactors/OutputChiller'},
 {'field':'X_limits','value':[0,90263],'artifact':'config.xml','line':25,'selector':'/BioXPCommonLib/AxisLimits/X_limit'},
 {'field':'Y_limits','value':[0,102956],'artifact':'config.xml','line':26,'selector':'/BioXPCommonLib/AxisLimits/Y_limit'},
 {'field':'Z_limits','value':[0,160000],'artifact':'config.xml','line':27,'selector':'/BioXPCommonLib/AxisLimits/Z_limit'},
 {'field':'G_limits','value':[0,15000],'artifact':'config.xml','line':28,'selector':'/BioXPCommonLib/AxisLimits/G_limit'},
 {'field':'PositionTable','value':'29 entries','artifact':'config.xml','line_range':[30,60],'selector':'/BioXPCommonLib/PositionTable/*'},
 {'field':'CameraOffset','value':{'x':3499,'y':-7744,'zLow':3145,'zDelta':6842},'artifact':'config.xml','line':59,'selector':'/BioXPCommonLib/PositionTable/CAMERA_OFFSET'},
 {'field':'Mode','value':'WebMode','artifact':'Operation_parameters.xml','line':4,'selector':'/OperationParameters/Mode/@Mode'},
 {'field':'DeckInspection','value':True,'artifact':'Operation_parameters.xml','line':8,'selector':'/OperationParameters/DeckInspection/@DeckInspection'},
 {'field':'InspectionLogOnly','value':False,'artifact':'Operation_parameters.xml','line':10,'selector':'/OperationParameters/InspectionLogOnly/@InspectionLogOnly'},
 {'field':'SelfTest','value':True,'artifact':'Operation_parameters.xml','line':11,'selector':'/OperationParameters/SelfTest/@SelfTest'},
 {'field':'CheckCamera','value':True,'artifact':'Operation_parameters.xml','line':17,'selector':'/OperationParameters/CheckCamera/@CheckCamera'},
 {'field':'ScreenResolutionHigh','value':False,'artifact':'Operation_parameters.xml','line':19,'selector':'/OperationParameters/ScreenResolutionHigh/@ScreenResolutionHigh'},
 {'field':'SelfTestTravel.raw_source_projection','value':{'X':92049,'Y':92049,'Z':92049},'artifact':'config.xml','line_range':[31,59],'selector':'derived:ClassBioXPSettings.cs:3448-3450','derivation_source':'ClassBioXPSettings.cs:3387-3450'},
 {'field':'SelfTestTravel.clamped','value':{'X':90263,'Y':92049,'Z':92049},'artifact':'config.xml','line_range':[25,59],'selector':'derived:ClassBioXPSettings.cs:1277-1281,3448-3450','derivation_source':'ClassBioXPSettings.cs:1277-1281,3387-3450'},
 {'field':'InspectionSettings3200.CamereInitialCheck','value':{'Exposure':1000,'Gain':1000,'LED1':False,'LED2':False,'LED3':False},'artifact':'InspectionSettings.xml','line_range':[32,41],'selector':'CameraSettings/Settings3200/*[Key=CamereInitialCheck]'},
 {'field':'InspectionSettings3200.CoverInspection','value':{'Exposure':1000,'Gain':1000,'LED1':False,'LED2':True,'LED3':False},'artifact':'InspectionSettings.xml','line_range':[59,68],'selector':'CameraSettings/Settings3200/*[Key=CoverInspection]'},
 {'field':'InspectionSettings3200.OutputPlateInspection','value':{'Exposure':1000,'Gain':1000,'LED1':False,'LED2':False,'LED3':False,'threshold':60,'pixelCount':1000},'artifact':'InspectionSettings.xml','line_range':[108,126],'selector':'CameraSettings/Settings3200/*[Key=OutputPlateInspection]'},
 {'field':'InspectionSettings3250.CamereInitialCheck','value':{'Exposure':-3,'Gain':1000,'LED1':False,'LED2':False,'LED3':False},'artifact':'InspectionSettings.xml','line_range':[279,288],'selector':'CameraSettings/Settings3250/*[Key=CamereInitialCheck]'},
 {'field':'InspectionSettings3250.CoverInspection','value':{'Exposure':-2,'Gain':1000,'LED1':False,'LED2':True,'LED3':False},'artifact':'InspectionSettings.xml','line_range':[306,315],'selector':'CameraSettings/Settings3250/*[Key=CoverInspection]'},
 {'field':'InspectionSettings3250.CoverStorageInspection','value':{'Exposure':-1,'Gain':1000,'LED1':True,'LED2':True,'LED3':True},'artifact':'InspectionSettings.xml','line_range':[333,342],'selector':'CameraSettings/Settings3250/*[Key=CoverStorageInspection]'},
 {'field':'InspectionSettings3250.OutputPlateInspection','value':{'Exposure':-2,'Gain':1000,'LED1':True,'LED2':True,'LED3':True,'threshold':30,'pixelCount':1000},'artifact':'InspectionSettings.xml','line_range':[366,384],'selector':'CameraSettings/Settings3250/*[Key=OutputPlateInspection]'},
]
selected_fields={r['field']:r['value'] for r in field_records}
known_hazards=[
 {'hazard_id':'NOVO_PIPETTE_MASK_IMPOSSIBLE_PROJECTION','source_ref':'ClassNovoCANUSB.cs:481-484; compare 691','disposition':'mandatory_raw_IL_resolution_before_transport_fixture_or_runtime_binding','detail':'Projected (ID & 7) cannot equal 259/260; coherent later mask uses ID & 0x107.'},
 {'hazard_id':'MOTOR_NULL_SUCCESS_CONFLATION','source_ref':'ClassMotor.cs:492-517,641-689; board queryHome implementations','disposition':'linux_must_require_controller_reply_and_physical_postcondition','detail':'Null replies and success can collapse to the same scalar; setHome mutates host state after failed/null transport; uninitialized/null switch paths can be interpreted as home.'},
 {'hazard_id':'THERMAL_NULL_AND_STATUS_CONFLATION','source_ref':'ClassThermalControl.cs:384-464,638-646,1079-1084,1165-1210','disposition':'linux_must_fail_closed_and_preserve_reply_status','detail':'Null reads can return cached/default values; set target can return success-like zero on null; lid null branch dereferences array; wrapper PWM status is discarded.'},
 {'hazard_id':'ROUTE_ALWAYS_TRUE_LOCATION_TEST','source_ref':'ClassControlInterface.cs:3869,3877','disposition':'mandatory_raw_IL_resolution_or_explicit_safety_deviation','detail':'Projected (location != A || location != B) predicates are always true.'},
 {'hazard_id':'MOVE_XY_WRONG_NULL_BRANCH','source_ref':'ClassControlInterface.cs:4292-4295','disposition':'mandatory_raw_IL_resolution_or_explicit_safety_deviation','detail':'Projected null-X-board branch calls moveX(y) instead of moving Y.'},
 {'hazard_id':'DOOR_NULL_AND_CACHED_STATE_SUCCESS','source_ref':'ClassControlInterface.cs:2651-2712; ControlLib.cs:8541-8610','disposition':'linux_requires_sensor_confirmed_postcondition','detail':'Null door board can return true and ControlLib can cache requested state despite failed confirmation; close recovery homes without retrying close.'},
 {'hazard_id':'SETTINGS_SERIAL_SETTER_BYPASS','source_ref':'ClassBioXPSettings.cs:768-788,2861-2885,3128-3190','disposition':'pin_explicit_serial_206_xml_values','detail':'loadConfig assigns backing serial directly; selected XML door-open 18500 differs from setter default 16000.'},
 {'hazard_id':'SETTINGS_SELFTEST_AXIS_CROSS_ASSIGNMENT','source_ref':'ClassBioXPSettings.cs:3449-3450','disposition':'preserve_source_semantics_in_fixture_and_record_any_safety_deviation','detail':'Projected Y/Z existing-value branches use m_xSelfTestTravel.'},
 {'hazard_id':'SETTINGS_PERMISSIVE_FALLBACK','source_ref':'ClassBioXPSettings.cs:2855-2858,3542-3669,4059-4080','disposition':'linux_live_path_must_fail_closed','detail':'OEM logs and continues on missing config and suppresses master-position/offset errors; Linux may not silently activate defaults for serial 206.'},
 {'hazard_id':'MOTION_WORKER_NO_FINALLY','source_ref':'BioXPMainWindow.cs:2030-2101','disposition':'persistent_state_machine_must_terminalize_or_block','detail':'Exception can bypass GantryAvailable restoration.'},
 {'hazard_id':'INITIALIZE_GRIPPER_NULL_GUARD_ORDER','source_ref':'ClassControlInterface.cs:3355-3356','disposition':'linux_must_admit_board_before_command','detail':'Projected gripper +10000 move occurs before later null guard.'},
 {'hazard_id':'TURN_OFF_HEATER_DUPLICATE_TARGET','source_ref':'ClassControlInterface.cs:3057-3067','disposition':'fixture_exact_duplicate_call_and_require_acknowledgements','detail':'Both calls are identical setTCorLidPWM(0,0), not separate nest/lid selectors.'},
 {'hazard_id':'SELFTEST_ASYNC_REPORTING_AND_TIMEOUT_MODE','source_ref':'ControlLib.cs:10688-11022','disposition':'ledger_reports_submission_separately_from_completion_and_pins_wait_mode','detail':'Finished messages are emitted after queueing; STA applies timeout per handle while non-STA applies one WaitAll timeout.'},
 {'hazard_id':'VISION_INVALID_IL_REGIONS','source_ref':'ControlLib.cs:3665-3671,3780-3782,3827,3852-3858,3893-3899,7073-7078','disposition':'mandatory_binary_disposition_before_executable_port','detail':'Unsafe enum/cast regions are projection evidence, not recompilable authority.'},
 {'hazard_id':'CAMERA_GAIN_UNUSED_BY_ADJUSTER','source_ref':'ControlLib.cs:1883-1920; CameraSettings.cs:318-325','disposition':'fixture_selected_gain_but_do_not_claim_it_was_applied','detail':'CameraControlParameter.Gain is loaded from the selected profile but AdjustCamera does not apply it.'},
 {'hazard_id':'DOOR_24V_POLARITY_PROJECTION','source_ref':'ControlLib.cs:8670-8726','disposition':'mandatory_raw_IL_and_controller_signal_resolution_before_physical_admission','detail':'Projected nonzero query24voltage path clears EnclosureDoorClosed and returns false; only zero reaches true.'},
 {'hazard_id':'NOVO_DECODE_NO_CHECKSUM_VALIDATION','source_ref':'NovoEncoding.cs:49-75,112-120','disposition':'explicit_fail_closed_linux_frame_validation_deviation','detail':'The projected Decode path removes framing/checksum bytes, unescapes payload, and returns true without visibly checking delimiters or checksum. Preserve OEM vectors, but Linux must validate frame boundaries and checksum before accepting a reply and label that behavior as a reviewed safety deviation.'},
]
registry={
 'schema_id':'bioxp.oem_movement_method_source_binary_registry.v3',
 'created_date':'2026-07-23',
 'claim_boundary':'Static source/binary/configuration registry only; no deployment, transport, controller, or physical parity claim.',
 'scope':'Complete OEM startup movement envelope: application admission and worker; no-motion motor setup; initializeMotors; initializeMotion stale-tip remediation; optional self-test movement; camera/cover movement; park; door/interlocks; terminal startup branches. General post-admission job execution is excluded.',
 'authority':{
  'evidence_lock_path':str(LOCK_PATH),'evidence_lock_schema':lock['schema_id'],'evidence_lock_sha256':hashlib.sha256(LOCK_PATH.read_bytes()).hexdigest(),
  'acquisition_session_id':lock['acquisition']['session_id'],
  'frozen_ssd_manifest_sha256':lock['frozen_ssd_corpus']['manifest_sha256'],
  'authority_order':lock['authority_order_for_conflict_resolution'],
 },
 'binaries':sorted(binary_records,key=lambda x:x['binary_id']),
 'sources':sorted(sources,key=lambda x:x['source_id']),
 'selected_machine_configuration':{
  'config_sha256':'33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475',
  'operation_parameters_sha256':'d032d58c08312706892a2d5c7a9a319f817a6d9ef73e75d30dd933ae110c9685',
  'inspection_settings_sha256':'d38220177e7e01b3d6d50892e0ffbbe27b1eb46087c4623cd6ca4757cc80b2d7',
  'values':selected_fields,
  'field_records':field_records,
 },
 'live_machine_corpus':lock['live_machine_corpus'],
 'known_hazards':known_hazards,
 'required_call_edges':required_call_edges,
 'required_branch_outcomes':required_branch_outcomes,
 'ordered_call_sequences':ordered_call_sequences,
 'methods':sorted(methods,key=lambda x:(x['source_id'],x['start_line'],x['method_id'])),
 'counts':{'binaries':len(binary_records),'sources':len(sources),'methods_and_members':len(methods),'live_machine_records':len(lock['live_machine_corpus']['records']),'required_call_edges':len(required_call_edges),'required_branch_outcomes':len(required_branch_outcomes),'ordered_call_sequences':len(ordered_call_sequences),'known_hazards':len(known_hazards)},
 'status_legend':{
  'present_hash_locked_and_line_bounded':'Source bytes hashed, captured binary pinned, canonical source-to-binary evidence-lock mapping present, and declaration/member has an exact inclusive line range.',
 },
}
OUT.parent.mkdir(parents=True,exist_ok=True)
OUT.write_text(json.dumps(registry,indent=2,sort_keys=True)+'\n')
print(json.dumps(registry['counts'],sort_keys=True))
print(OUT)

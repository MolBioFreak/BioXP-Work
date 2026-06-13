## GenBotApp/BioXPMainWindow.cs:1128-1165
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

```csharp
1128: 		{
1129: 			ClassMessageLog.LogSystemMessage("System preparing for shipping.", (SystemLogMessageType)0);
1130: 			m_control.doorOpen(false, false);
1131: 			MessageBox.Show("Power system off with rocker switch on right side of unit", "Attention!", (MessageBoxButton)0);
1132: 			Process.Start("shutdown", "/s /f");
1133: 			return;
1134: 		}
1135: 		int num = 0;
1136: 		showScreen("Inspection", messages, num);
1137: 		ErrorStatus val = (ErrorStatus)0;
1138: 		try
1139: 		{
1140: 			if (!skipInitializeMotion)
1141: 			{
1142: 				bool flag = true;
1143: 				m_control.initialCheck();
1144: 				if ((int)ClassStatusLog.SavedStatus == 3 || (int)ClassStatusLog.SavedStatus == 4)
1145: 				{
1146: 					showScreen("Warning", "_warning", "_msg80", null, null, WarningSituation.WAIT_INITIALIZATION.ToString());
1147: 					ClassMessageLog.LogSystemMessage("*** Unexpected shutdown occurred ***", (SystemLogMessageType)0);
1148: 					m_control.initializeMotion();
1149: 					val = m_control.inspectCover();
1150: 					flag = false;
1151: 				}
1152: 				if (!flag)
1153: 				{
1154: 					m_control.unlockDoor();
1155: 					showScreen("Warning", "_warning", "_msg37", null, null, WarningSituation.WAIT_INITIALIZATION.ToString());
1156: 					return;
1157: 				}
1158: 				showScreen(num);
1159: 				m_control.initializeMotion();
1160: 				m_control.m_PipetteControl.pipetteError -= new pipetteErrorEvent(m_PipetteControl_pipetteError);
1161: 				m_control.m_PipetteControl.pipetteError += new pipetteErrorEvent(m_PipetteControl_pipetteError);
1162: 			}
1163: 			if (m_settingsWindow.SelfTest && (DateTime.Now - ClassStatusLog.SelfTestDate).TotalDays > 1.0)
1164: 			{
1165: 				num++;
```

## GenBotApp/BioXPMainWindow.cs:1240-1288
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

```csharp
1240: 					oCommunicationEvent.WaitOne(5000);
1241: 				}
1242: 				if (!m_communication.CheckNetworkStatus() && (int)m_settingsWindow.StartMode != 2)
1243: 				{
1244: 					m_control.unlockDoor();
1245: 					ClassMessageLog.LogSystemMessage("No network connection has been detected", (SystemLogMessageType)2);
1246: 					ClassMessageLog.LogSystemMessage("No network connection has been detected", (SystemLogMessageType)1);
1247: 					showScreen("Error", "_error", "_NoNetworkConnection", "_BtnConfig", null, ErrorSituation.SERVER_COMMUNICATION_ERROR.ToString());
1248: 				}
1249: 				else
1250: 				{
1251: 					num++;
1252: 					showScreen(num);
1253: 					val = PrepareToRunJob(out jobid_barcode, out reagent_barcode);
1254: 					m_reagentBarcode = reagent_barcode;
1255: 					m_control.parkGantry(false);
1256: 					switch ((int)val)
1257: 					{
1258: 					case 0:
1259: 						m_bsc_barcode = jobid_barcode;
1260: 						try
1261: 						{
1262: 							if ((int)m_settingsWindow.StartMode == 2)
1263: 							{
1264: 								if (!m_communication.GetJobLocal(jobid_barcode, reagent_barcode))
1265: 								{
1266: 									m_control.unlockDoor();
1267: 									showError("_error", "_msg82", null, null, ErrorSituation.JOB_LOAD_ERROR);
1268: 								}
1269: 							}
1270: 							else if (m_settingsWindow.Host.Contains("AWS"))
1271: 							{
1272: 								if (!m_communication.GetJobAWS(jobid_barcode, reagent_barcode))
1273: 								{
1274: 									m_control.unlockDoor();
1275: 									showError("_error", "_msg46", null, null, ErrorSituation.JOB_LOAD_ERROR);
1276: 									postFilesToAWS();
1277: 								}
1278: 							}
1279: 							else
1280: 							{
1281: 								m_communication.GetJobFromGoogle(jobid_barcode);
1282: 							}
1283: 						}
1284: 						catch (Exception ex)
1285: 						{
1286: 							ClassMessageLog.LogErrorMessage(ex.Message);
1287: 							m_control.unlockDoor();
1288: 							showStart("_attention", "_msg18");
```

## GenBotApp/BioXPMainWindow.cs:2040-2120
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

```csharp
2040: 		{
2041: 			motionCommands motionCommands = m_commandQueue.Take();
2042: 			m_control.GantryAvailable = false;
2043: 			switch (motionCommands.name)
2044: 			{
2045: 			case "initializeSystem":
2046: 				if (!UpdateCheck())
2047: 				{
2048: 					initializeSystem();
2049: 					ClassStatusLog.setStatus((system_status)1, true);
2050: 				}
2051: 				break;
2052: 			case "unlockProcess":
2053: 				unlockProcess();
2054: 				ClassStatusLog.setStatus((system_status)1, true);
2055: 				break;
2056: 			case "PrepareToRunJob":
2057: 				ClassCommonFunctions.Async((DispatcherObject)(object)m_pageRunningLarge, (Action)delegate
2058: 				{
2059: 					m_pageRunningLarge.showInitalizedForUse();
2060: 				});
2061: 				if ((int)m_settingsWindow.StartMode == 1 || (int)m_settingsWindow.StartMode == 2)
2062: 				{
2063: 					if (!m_bsc_barcode.ToLower().StartsWith("tp"))
2064: 					{
2065: 						if (!checkReagentBarcode())
2066: 						{
2067: 							ClassStatusLog.setStatus((system_status)1, true);
2068: 						}
2069: 						else
2070: 						{
2071: 							PrepareToRunJob();
2072: 						}
2073: 					}
2074: 					else
2075: 					{
2076: 						PrepareToRunJob();
2077: 					}
2078: 				}
2079: 				else
2080: 				{
2081: 					PrepareToRunJob();
2082: 				}
2083: 				break;
2084: 			case "abortjob":
2085: 			{
2086: 				WarningSituation action = (WarningSituation)Enum.Parse(typeof(WarningSituation), motionCommands.para[0].ToString());
2087: 				abortjob(action);
2088: 				ClassStatusLog.setStatus((system_status)1, true);
2089: 				break;
2090: 			}
2091: 			case "validateJob":
2092: 				validateJob();
2093: 				break;
2094: 			case "wakefrompause":
2095: 				wakefrompause();
2096: 				ClassStatusLog.setStatus((system_status)3, true);
2097: 				break;
2098: 			}
2099: 			m_control.GantryAvailable = true;
2100: 		}
2101: 	}
2102: 
2103: 	private void wakefrompause()
2104: 	{
2105: 		m_control.initialCheck();
2106: 		m_control.rehome();
2107: 		if (m_settingsWindow.ScreenResolutionHigh)
2108: 		{
2109: 			ClassCommonFunctions.Async((DispatcherObject)(object)m_pageRunningLarge, (Action)delegate
2110: 			{
2111: 				((ContentControl)m_pageRunningLarge.btnPause).Content = "Continue";
2112: 			});
2113: 			ClassCommonFunctions.Async((DispatcherObject)(object)m_pageRunningLarge.btnPause, (Action)delegate
2114: 			{
2115: 				((UIElement)m_pageRunningLarge.btnPause).IsEnabled = true;
2116: 			});
2117: 		}
2118: 		else
2119: 		{
2120: 			ClassCommonFunctions.Async((DispatcherObject)(object)m_pageRunning, (Action)delegate
```

## GenBotApp/BioXPMainWindow.cs:1370-1425
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs`

```csharp
1370: 		{
1371: 			m_control.parkGantry(false);
1372: 		}
1373: 		return flag;
1374: 	}
1375: 
1376: 	private void validateJob()
1377: 	{
1378: 		//IL_0099: Unknown result type (might be due to invalid IL or missing references)
1379: 		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
1380: 		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
1381: 		//IL_00c8: Expected I4, but got Unknown
1382: 		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
1383: 		//IL_00dc: Invalid comparison between Unknown and I4
1384: 		oCommunicationEvent.WaitOne(5000);
1385: 		if (!m_communication.CheckNetworkStatus())
1386: 		{
1387: 			m_control.unlockDoor();
1388: 			ClassMessageLog.LogSystemMessage("No network connection has been detected", (SystemLogMessageType)2);
1389: 			ClassMessageLog.LogSystemMessage("No network connection has been detected", (SystemLogMessageType)1);
1390: 			showScreen("Error", "_error", "_NoNetworkConnection", "_BtnConfig", null, ErrorSituation.SERVER_COMMUNICATION_ERROR.ToString());
1391: 			return;
1392: 		}
1393: 		showScreen("Warning", "_warning", "_recheckJob", null, null, WarningSituation.NETWORK_PROBLEM.ToString());
1394: 		string jobid_barcode;
1395: 		string reagent_barcode;
1396: 		ErrorStatus val = PrepareToRunJob(out jobid_barcode, out reagent_barcode);
1397: 		m_reagentBarcode = reagent_barcode;
1398: 		m_control.parkGantry(false);
1399: 		switch ((int)val)
1400: 		{
1401: 		case 0:
1402: 			m_bsc_barcode = jobid_barcode;
1403: 			if ((int)m_settingsWindow.StartMode == 2)
1404: 			{
1405: 				if (!m_communication.GetJobLocal(jobid_barcode, reagent_barcode))
1406: 				{
1407: 					m_control.unlockDoor();
1408: 					showError("_error", "_msg82", null, null, ErrorSituation.JOB_LOAD_ERROR);
1409: 				}
1410: 			}
1411: 			else if (m_settingsWindow.Host.Contains("AWS"))
1412: 			{
1413: 				if (!m_communication.GetJobAWS(jobid_barcode, reagent_barcode))
1414: 				{
1415: 					m_control.unlockDoor();
1416: 					showError("_error", "_msg46", null, null, ErrorSituation.JOB_LOAD_ERROR);
1417: 					postFilesToAWS();
1418: 				}
1419: 			}
1420: 			else
1421: 			{
1422: 				m_communication.GetJobFromGoogle(jobid_barcode);
1423: 			}
1424: 			break;
1425: 		case 1:
```

## ControlLib.cs:8790-8845
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ControlLib.cs`

```csharp
8790: 		{
8791: 			m_machineStatus.ThermalDoorOpen = false;
8792: 		}
8793: 		doorOpen(thermalDoorOpen);
8794: 		m_ControlInterface.resumeTemperature();
8795: 	}
8796: 
8797: 	public void initializeMotion()
8798: 	{
8799: 		m_stopScripts = true;
8800: 		forceabort = false;
8801: 		try
8802: 		{
8803: 			m_ControlInterface.initializeMotors();
8804: 			m_machineStatus.ThermalDoorOpen = false;
8805: 			m_PipetteControl.queryTipStatus(-1);
8806: 			Thread.Sleep(500);
8807: 			if (m_PipetteControl.TipExist)
8808: 			{
8809: 				m_ControlInterface.openThermalDoor();
8810: 				m_machineStatus.ThermalDoorOpen = true;
8811: 				m_machineStatus.TipLoaded = true;
8812: 				m_ControlInterface.scriptmoveTo((locationID)28, (wellID)0, (locationID)6, 0, 0);
8813: 				m_machineStatus.updateLocation((locationID)6, (wellID)0);
8814: 				m_PipetteControl.ejectAllTips(false, true);
8815: 				m_ControlInterface.moveZ(80000);
8816: 				m_ControlInterface.moveX(79000);
8817: 				m_PipetteControl.queryTipStatus(-1);
8818: 				Thread.Sleep(100);
8819: 				if (m_PipetteControl.TipExist)
8820: 				{
8821: 					m_pauseScripts = true;
8822: 					if (this.errorEvent != null)
8823: 					{
8824: 						this.errorEvent("Eject tip failed");
8825: 						throw new Exception("Eject tip failed");
8826: 					}
8827: 					return;
8828: 				}
8829: 				m_machineStatus.TipDirty = false;
8830: 				m_machineStatus.TipLoaded = false;
8831: 				Thread.Sleep(2);
8832: 				m_PipetteControl.initiateGroup();
8833: 				if (!m_PipetteControl.checkedPipetteStatus())
8834: 				{
8835: 					m_PipetteControl.initiateGroup();
8836: 					if (!m_PipetteControl.checkedPipetteStatus())
8837: 					{
8838: 						this.errorEvent("Eject tip failed");
8839: 						throw new Exception("Eject tip failed");
8840: 					}
8841: 				}
8842: 			}
8843: 			else
8844: 			{
8845: 				m_machineStatus.TipLoaded = false;
```

## ClassControlInterface.cs:3180-3410
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
3180: 
3181: 	public void initializeMotorsWithoutMotion()
3182: 	{
3183: 		waitForBoard();
3184: 		turnOffHeater();
3185: 		setChillerPWM();
3186: 		Thread.Sleep(1);
3187: 		if (m_Boards[m_AxisIODesignater["MotorX"].board] != null)
3188: 		{
3189: 			m_Boards[m_AxisIODesignater["MotorX"].board].setSpeedAcc(m_AxisIODesignater["MotorX"].axis, 1700, 350);
3190: 			Thread.Sleep(2);
3191: 			m_Boards[m_AxisIODesignater["MotorX"].board].setMaxCurrent(m_AxisIODesignater["MotorX"].axis, 31);
3192: 			Thread.Sleep(2);
3193: 			m_Boards[m_AxisIODesignater["MotorX"].board].setStallGuardThreshold(m_AxisIODesignater["MotorX"].axis, 16);
3194: 			Thread.Sleep(2);
3195: 		}
3196: 		if (m_Boards[m_AxisIODesignater["MotorY"].board] != null)
3197: 		{
3198: 			m_Boards[m_AxisIODesignater["MotorY"].board].setSpeedAcc(m_AxisIODesignater["MotorY"].axis, 1800, 400);
3199: 			Thread.Sleep(2);
3200: 			m_Boards[m_AxisIODesignater["MotorY"].board].setMaxCurrent(m_AxisIODesignater["MotorY"].axis, 31);
3201: 			Thread.Sleep(2);
3202: 			m_Boards[m_AxisIODesignater["MotorY"].board].setStallGuardThreshold(m_AxisIODesignater["MotorY"].axis, 16);
3203: 			Thread.Sleep(2);
3204: 			m_Boards[m_AxisIODesignater["MotorY"].board].disableRightSwitch(m_AxisIODesignater["MotorY"].axis);
3205: 			Thread.Sleep(2);
3206: 		}
3207: 		if (m_Boards[m_AxisIODesignater["MotorZ"].board] != null)
3208: 		{
3209: 			m_Boards[m_AxisIODesignater["MotorZ"].board].setSpeedAcc(m_AxisIODesignater["MotorZ"].axis, 1791, 576);
3210: 			Thread.Sleep(2);
3211: 			m_Boards[m_AxisIODesignater["MotorZ"].board].setMaxCurrent(m_AxisIODesignater["MotorZ"].axis, m_settingsWindow.Z_MOTOR_MAX_CURRENT_UP);
3212: 			Thread.Sleep(2);
3213: 			m_zCurrent = m_Boards[m_AxisIODesignater["MotorZ"].board].readMaxCurrent(m_AxisIODesignater["MotorZ"].axis);
3214: 			m_Boards[m_AxisIODesignater["MotorZ"].board].setStallGuardThreshold(m_AxisIODesignater["MotorZ"].axis, m_settingsWindow.Z_MOTOR_STALL_GUARD_THRESHOLD);
3215: 			Thread.Sleep(2);
3216: 		}
3217: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] != null)
3218: 		{
3219: 			if (m_settingsWindow.GripperVersion == 0)
3220: 			{
3221: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setSpeedAcc(m_AxisIODesignater["MotorGrip"].axis, 600, 5);
3222: 			}
3223: 			else
3224: 			{
3225: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setSpeedAcc(m_AxisIODesignater["MotorGrip"].axis, 1500, 20);
3226: 			}
3227: 			Thread.Sleep(2);
3228: 			if (m_settingsWindow.GripperVersion == 0)
3229: 			{
3230: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setMaxCurrent(m_AxisIODesignater["MotorGrip"].axis, 31);
3231: 				Thread.Sleep(2);
3232: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setStallGuardThreshold(m_AxisIODesignater["MotorGrip"].axis, 5);
3233: 			}
3234: 			else
3235: 			{
3236: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setMaxCurrent(m_AxisIODesignater["MotorGrip"].axis, 10);
3237: 				Thread.Sleep(2);
3238: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setStallGuardThreshold(m_AxisIODesignater["MotorGrip"].axis, 20);
3239: 			}
3240: 			Thread.Sleep(2);
3241: 			m_Boards[m_AxisIODesignater["MotorGrip"].board].setRdivPdiv(m_AxisIODesignater["MotorGrip"].axis, 6, 2);
3242: 			Thread.Sleep(2);
3243: 		}
3244: 		if (m_Boards[m_AxisIODesignater["ThermalDoor"].board] != null)
3245: 		{
3246: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].setSpeedAcc(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TC_DOOR_VELOCITY, m_settingsWindow.TC_DOOR_ACCELERATION);
3247: 			Thread.Sleep(2);
3248: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].setMaxCurrent(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TC_DOOR_MAX_CURRENT);
3249: 			Thread.Sleep(2);
3250: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].setStallGuardThreshold(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TCDoorStallGuardThreshold);
3251: 			Thread.Sleep(2);
3252: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].disableRightSwitch(m_AxisIODesignater["ThermalDoor"].axis);
3253: 			Thread.Sleep(2);
3254: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].disableLeftSwitch(m_AxisIODesignater["ThermalDoor"].axis);
3255: 			Thread.Sleep(2);
3256: 		}
3257: 		setChillerCoolRate("OC");
3258: 		setChillerCoolRate("RC");
3259: 		if (m_Boards[m_AxisIODesignater["ThermalCycler"].board] != null)
3260: 		{
3261: 			m_Boards[m_AxisIODesignater["ThermalCycler"].board].setTCHeatRate(2.5);
3262: 			m_Boards[m_AxisIODesignater["ThermalCycler"].board].setTCCoolRate(-2.0);
3263: 		}
3264: 		setColor(255, 255, 255);
3265: 	}
3266: 
3267: 	public void initializeforBoardTest()
3268: 	{
3269: 		if (m_Boards[m_AxisIODesignater["MotorX"].board] != null)
3270: 		{
3271: 			m_Boards[m_AxisIODesignater["MotorX"].board].setSpeedAcc(m_AxisIODesignater["MotorX"].axis, 1700, 350);
3272: 			Thread.Sleep(2);
3273: 			m_Boards[m_AxisIODesignater["MotorX"].board].setMaxCurrent(m_AxisIODesignater["MotorX"].axis, 31);
3274: 			Thread.Sleep(2);
3275: 			m_Boards[m_AxisIODesignater["MotorX"].board].setStallGuardThreshold(m_AxisIODesignater["MotorX"].axis, 16);
3276: 			Thread.Sleep(2);
3277: 		}
3278: 		if (m_Boards[m_AxisIODesignater["MotorY"].board] != null)
3279: 		{
3280: 			m_Boards[m_AxisIODesignater["MotorY"].board].setSpeedAcc(m_AxisIODesignater["MotorY"].axis, 1800, 400);
3281: 			Thread.Sleep(2);
3282: 			m_Boards[m_AxisIODesignater["MotorY"].board].setMaxCurrent(m_AxisIODesignater["MotorY"].axis, 31);
3283: 			Thread.Sleep(2);
3284: 			m_Boards[m_AxisIODesignater["MotorY"].board].setStallGuardThreshold(m_AxisIODesignater["MotorY"].axis, 16);
3285: 			Thread.Sleep(2);
3286: 			m_Boards[m_AxisIODesignater["MotorY"].board].disableRightSwitch(m_AxisIODesignater["MotorY"].axis);
3287: 			Thread.Sleep(2);
3288: 		}
3289: 		if (m_Boards[m_AxisIODesignater["MotorZ"].board] != null)
3290: 		{
3291: 			m_Boards[m_AxisIODesignater["MotorZ"].board].setSpeedAcc(m_AxisIODesignater["MotorZ"].axis, 1791, 576);
3292: 			Thread.Sleep(2);
3293: 			m_Boards[m_AxisIODesignater["MotorZ"].board].setMaxCurrent(m_AxisIODesignater["MotorZ"].axis, m_settingsWindow.Z_MOTOR_MAX_CURRENT_UP);
3294: 			Thread.Sleep(2);
3295: 			m_zCurrent = m_Boards[m_AxisIODesignater["MotorZ"].board].readMaxCurrent(m_AxisIODesignater["MotorZ"].axis);
3296: 			m_Boards[m_AxisIODesignater["MotorZ"].board].setStallGuardThreshold(m_AxisIODesignater["MotorZ"].axis, m_settingsWindow.Z_MOTOR_STALL_GUARD_THRESHOLD);
3297: 			Thread.Sleep(2);
3298: 		}
3299: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] != null)
3300: 		{
3301: 			if (m_settingsWindow.GripperVersion == 0)
3302: 			{
3303: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setSpeedAcc(m_AxisIODesignater["MotorGrip"].axis, 600, 5);
3304: 			}
3305: 			else
3306: 			{
3307: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setSpeedAcc(m_AxisIODesignater["MotorGrip"].axis, 1500, 20);
3308: 			}
3309: 			Thread.Sleep(2);
3310: 			if (m_settingsWindow.GripperVersion == 0)
3311: 			{
3312: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setMaxCurrent(m_AxisIODesignater["MotorGrip"].axis, 31);
3313: 				Thread.Sleep(2);
3314: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setStallGuardThreshold(m_AxisIODesignater["MotorGrip"].axis, 5);
3315: 			}
3316: 			else
3317: 			{
3318: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setMaxCurrent(m_AxisIODesignater["MotorGrip"].axis, 10);
3319: 				Thread.Sleep(2);
3320: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].setStallGuardThreshold(m_AxisIODesignater["MotorGrip"].axis, 20);
3321: 			}
3322: 			Thread.Sleep(2);
3323: 			m_Boards[m_AxisIODesignater["MotorGrip"].board].setRdivPdiv(m_AxisIODesignater["MotorGrip"].axis, 6, 2);
3324: 			Thread.Sleep(2);
3325: 		}
3326: 		if (m_Boards[m_AxisIODesignater["ThermalDoor"].board] != null)
3327: 		{
3328: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].setSpeedAcc(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TC_DOOR_VELOCITY, m_settingsWindow.TC_DOOR_ACCELERATION);
3329: 			Thread.Sleep(2);
3330: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].setMaxCurrent(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TC_DOOR_MAX_CURRENT);
3331: 			Thread.Sleep(2);
3332: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].setStallGuardThreshold(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TCDoorStallGuardThreshold);
3333: 			Thread.Sleep(2);
3334: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].disableRightSwitch(m_AxisIODesignater["ThermalDoor"].axis);
3335: 			Thread.Sleep(2);
3336: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].disableLeftSwitch(m_AxisIODesignater["ThermalDoor"].axis);
3337: 			Thread.Sleep(2);
3338: 		}
3339: 		setChillerCoolRate("OC");
3340: 		setChillerCoolRate("RC");
3341: 		if (m_Boards[m_AxisIODesignater["ThermalCycler"].board] != null)
3342: 		{
3343: 			m_Boards[m_AxisIODesignater["ThermalCycler"].board].setTCHeatRate(2.5);
3344: 			m_Boards[m_AxisIODesignater["ThermalCycler"].board].setTCCoolRate(-2.0);
3345: 		}
3346: 	}
3347: 
3348: 	public void initializeMotors()
3349: 	{
3350: 		if (m_Boards[m_AxisIODesignater["MotorZ"].board] != null)
3351: 		{
3352: 			m_Boards[m_AxisIODesignater["MotorZ"].board].axisSearchHome(m_AxisIODesignater["MotorZ"].axis, 1791);
3353: 		}
3354: 		setGripperCurrent(31);
3355: 		m_Boards[m_AxisIODesignater["MotorGrip"].board].moveSteps(m_AxisIODesignater["MotorGrip"].axis, 10000, true);
3356: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] != null)
3357: 		{
3358: 			if (m_settingsWindow.GripperVersion == 0)
3359: 			{
3360: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].axisSearchHome(m_AxisIODesignater["MotorGrip"].axis, 600);
3361: 			}
3362: 			else
3363: 			{
3364: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].axisSearchHome(m_AxisIODesignater["MotorGrip"].axis, 200);
3365: 			}
3366: 		}
3367: 		if (m_Boards[m_AxisIODesignater["MotorX"].board] != null)
3368: 		{
3369: 			m_Boards[m_AxisIODesignater["MotorX"].board].axisSearchHome(m_AxisIODesignater["MotorX"].axis, 250);
3370: 			Thread.Sleep(20);
3371: 			m_Boards[m_AxisIODesignater["MotorX"].board].setHome(m_AxisIODesignater["MotorX"].axis);
3372: 			m_Boards[m_AxisIODesignater["MotorX"].board].setSpeed(m_AxisIODesignater["MotorX"].axis, 1700);
3373: 			Thread.Sleep(40);
3374: 			moveX(6000);
3375: 		}
3376: 		if (m_Boards[m_AxisIODesignater["MotorY"].board] != null)
3377: 		{
3378: 			m_Boards[m_AxisIODesignater["MotorY"].board].axisSearchHome(m_AxisIODesignater["MotorY"].axis, 250);
3379: 		}
3380: 		if (m_Boards[m_AxisIODesignater["ThermalDoor"].board] != null)
3381: 		{
3382: 			m_Boards[m_AxisIODesignater["ThermalDoor"].board].doorSearchHome(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TC_DOOR_VELOCITY, m_settingsWindow.TCDoorStallGuardThreshold);
3383: 		}
3384: 		if (Convert.ToInt32(m_settingsWindow.SerialNumber) > 9 && !confirmAxis("tcDoorClosed") && m_settingsWindow.CameraCalibrated)
3385: 		{
3386: 			openThermalDoor();
3387: 			throw new Exception("Cannot close thermal cycler door!");
3388: 		}
3389: 		if (m_Boards[m_AxisIODesignater["MotorY"].board] != null)
3390: 		{
3391: 			m_Boards[m_AxisIODesignater["MotorY"].board].setHome(m_AxisIODesignater["MotorY"].axis);
3392: 		}
3393: 		if (m_settingsWindow.Calibrated)
3394: 		{
3395: 			PageMotionControl mc = m_controlLib.m_diagnosticPanel.m_MotionControl;
3396: 			if (((DispatcherObject)mc).Dispatcher.CheckAccess())
3397: 			{
3398: 				mc.txtAbsPosX.Text = "0";
3399: 				mc.txtAbsPosY.Text = "0";
3400: 				mc.txtAbsPosZ.Text = "0";
3401: 				mc.txtAbsPosZ.Text = "0";
3402: 			}
3403: 			else
3404: 			{
3405: 				((DispatcherObject)mc).Dispatcher.Invoke((DispatcherPriority)9, (Delegate)(Action)delegate
3406: 				{
3407: 					mc.txtAbsPosX.Text = "0";
3408: 					mc.txtAbsPosY.Text = "0";
3409: 					mc.txtAbsPosZ.Text = "0";
3410: 					mc.txtAbsPosZ.Text = "0";
```

## ClassControlInterface.cs:5050-5070
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
5050: 		}
5051: 		return result;
5052: 	}
5053: 
5054: 	internal int[] HomeXY()
5055: 	{
5056: 		if (m_Boards[m_AxisIODesignater["MotorX"].board] != null && m_Boards[m_AxisIODesignater["MotorY"].board] != null)
5057: 		{
5058: 			m_Boards[m_AxisIODesignater["MotorX"].board].setSpeedAcc(m_AxisIODesignater["MotorX"].axis, 200, 200);
5059: 			m_Boards[m_AxisIODesignater["MotorY"].board].setSpeedAcc(m_AxisIODesignater["MotorY"].axis, 200, 200);
5060: 			int x = 0;
5061: 			int y = 0;
5062: 			Task task = Task.Run(() => x = m_Boards[m_AxisIODesignater["MotorX"].board].goHome(false, m_AxisIODesignater["MotorX"].axis, 200, true));
5063: 			Task task2 = Task.Run(() => y = m_Boards[m_AxisIODesignater["MotorY"].board].goHome(false, m_AxisIODesignater["MotorY"].axis, 200, true));
5064: 			Task.WaitAll(task, task2);
5065: 			m_Boards[m_AxisIODesignater["MotorX"].board].setSpeedAcc(m_AxisIODesignater["MotorX"].axis, 1700, 350);
5066: 			m_Boards[m_AxisIODesignater["MotorY"].board].setSpeedAcc(m_AxisIODesignater["MotorY"].axis, 1800, 400);
5067: 			return new int[2] { x, y };
5068: 		}
5069: 		return null;
5070: 	}
```

## ClassControlInterface.cs:1218-1240
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
1218: 
1219: 	private void btnNewLocation_Click(object sender, RoutedEventArgs e)
1220: 	{
1221: 		m_controlLib.m_diagnosticPanel.clearXYZ();
1222: 	}
1223: 
1224: 	private void btnDHome_Click(object sender, RoutedEventArgs e)
1225: 	{
1226: 		//IL_00db: Unknown result type (might be due to invalid IL or missing references)
1227: 		try
1228: 		{
1229: 			if (m_Boards[m_AxisIODesignater["ThermalDoor"].board] != null)
1230: 			{
1231: 				m_Boards[m_AxisIODesignater["ThermalDoor"].board].doorSearchHome(m_AxisIODesignater["ThermalDoor"].axis, m_settingsWindow.TC_DOOR_VELOCITY, m_settingsWindow.TCDoorStallGuardThreshold);
1232: 			}
1233: 			bool closed = confirmAxis("tcDoorClosed");
1234: 			bool opened = confirmAxis("tcDoorOpened");
1235: 			PageMotionControl mc = m_controlLib.m_diagnosticPanel.m_MotionControl;
1236: 			mc.txtAbsPosD.Text = "0";
1237: 			ClassCommonFunctions.Async((DispatcherObject)(object)mc, (Action)delegate
1238: 			{
1239: 				mc.UpdateDoorStatus(opened, closed);
1240: 			});
```

## ClassControlInterface.cs:1960-2018
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
1960: 		}
1961: 		catch (Exception ex)
1962: 		{
1963: 			MessageBox.Show(ex.Message);
1964: 		}
1965: 	}
1966: 
1967: 	private void btnDoorOpen_Click(object sender, RoutedEventArgs e)
1968: 	{
1969: 		openThermalDoor();
1970: 	}
1971: 
1972: 	private void btnDoorClose_Click(object sender, RoutedEventArgs e)
1973: 	{
1974: 		closeThermalDoor();
1975: 	}
1976: 
1977: 	private void btnSetColor_Click(object sender, RoutedEventArgs e)
1978: 	{
1979: 		string text = m_controlLib.m_diagnosticPanel.m_IOpage.txtRedIntensity.Text;
1980: 		string text2 = m_controlLib.m_diagnosticPanel.m_IOpage.txtGreenIntensity.Text;
1981: 		string text3 = m_controlLib.m_diagnosticPanel.m_IOpage.txtBlueIntensity.Text;
1982: 		if (int.TryParse(text, out var result) && int.TryParse(text2, out var result2) && int.TryParse(text3, out var result3))
1983: 		{
1984: 			setColor((byte)result, (byte)result2, (byte)result3);
1985: 		}
1986: 	}
1987: 
1988: 	private void btnTest_Click(object sender, RoutedEventArgs e)
1989: 	{
1990: 	}
1991: 
1992: 	private void btnOpen_Click(object sender, RoutedEventArgs e)
1993: 	{
1994: 		//IL_0090: Unknown result type (might be due to invalid IL or missing references)
1995: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] != null)
1996: 		{
1997: 			try
1998: 			{
1999: 				setGripperCurrent(31);
2000: 				OpenGripper(recover: true);
2001: 				m_controlLib.m_diagnosticPanel.m_MotionControl.txtAbsPosG.Text = m_Boards[m_AxisIODesignater["MotorGrip"].board].getCurrentPosition(m_AxisIODesignater["MotorGrip"].axis).ToString();
2002: 			}
2003: 			catch (Exception ex)
2004: 			{
2005: 				MessageBox.Show(ex.Message);
2006: 			}
2007: 		}
2008: 	}
2009: 
2010: 	private void btnOpenWide_Click(object sender, RoutedEventArgs e)
2011: 	{
2012: 		//IL_0090: Unknown result type (might be due to invalid IL or missing references)
2013: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] != null)
2014: 		{
2015: 			try
2016: 			{
2017: 				setGripperCurrent(31);
2018: 				OpenGripper(recover: true, openwide: true);
```

## ClassControlInterface.cs:4620-4665
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
4620: 		return 0;
4621: 	}
4622: 
4623: 	public int MoveZHome(bool rehome = true)
4624: 	{
4625: 		if (m_Boards[m_AxisIODesignater["MotorZ"].board] != null)
4626: 		{
4627: 			m_Boards[m_AxisIODesignater["MotorZ"].board].setMaxCurrent(m_AxisIODesignater["MotorZ"].axis, 31);
4628: 			m_zCurrent = m_Boards[m_AxisIODesignater["MotorZ"].board].readMaxCurrent(m_AxisIODesignater["MotorZ"].axis);
4629: 			return m_Boards[m_AxisIODesignater["MotorZ"].board].goHome(rehome, m_AxisIODesignater["MotorZ"].axis, 1791, true);
4630: 		}
4631: 		return 0;
4632: 	}
4633: 
4634: 	public bool IsDevelopmentMachine()
4635: 	{
4636: 		bool result = false;
4637: 		string[] directories = Directory.GetDirectories("c:\\program files", "Microsoft Visual Studio*");
4638: 		int num = 0;
4639: 		if (num < directories.Length)
4640: 		{
4641: 			_ = directories[num];
4642: 			result = true;
4643: 		}
4644: 		if (Directory.Exists("C:\\Program Files (x86)"))
4645: 		{
4646: 			directories = Directory.GetDirectories("C:\\Program Files (x86)", "Microsoft Visual Studio*");
4647: 			num = 0;
4648: 			if (num < directories.Length)
4649: 			{
4650: 				_ = directories[num];
4651: 				result = true;
4652: 			}
4653: 		}
4654: 		return result;
4655: 	}
4656: 
4657: 	public void homeGZ(int delay)
4658: 	{
4659: 		if (m_Boards[m_AxisIODesignater["MotorZ"].board] == null)
4660: 		{
4661: 			HomeAxis("G");
4662: 			return;
4663: 		}
4664: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] == null)
4665: 		{
```

## ClassBaseBoard.cs:140-170
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_can/ClassCanLib/ClassBaseBoard.cs`

```csharp
140: 	public virtual void forceAbortMotion()
141: 	{
142: 	}
143: 
144: 	public virtual void setStallGuardThreshold(int axis, int threshold)
145: 	{
146: 	}
147: 
148: 	public virtual void doorSearchHome(int axis, int speed, int stallThread)
149: 	{
150: 	}
151: 
152: 	public virtual int goHome(bool rehome, int axis, int speed, bool waitforstop = true)
153: 	{
154: 		return 0;
155: 	}
156: 
157: 	public virtual int moveToAbs(int axis, int position, bool waitforstop = true, bool stallRecover = false, bool gripperRecover = false)
158: 	{
159: 		return 0;
160: 	}
161: 
162: 	public virtual void setSpeedAcc(int axis, int speed, int acc)
163: 	{
164: 	}
165: 
166: 	public virtual void setSpeed(int axis, int velocity)
167: 	{
168: 	}
169: 
170: 	public virtual void setMaxCurrent(int axis, int current)
```

## ClassHeadBoard.cs:50-80
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_can/ClassCanLib/ClassHeadBoard.cs`

```csharp
50: 		{
51: 			ClassMessageLog.logMessage("activateBoard returned null for " + m_boardAssy.ToString() + " ****************", false);
52: 			ClassMessageLog.LogSystemMessage("activateBoard returned null for " + m_boardAssy, (SystemLogMessageType)0);
53: 		}
54: 		else if (array[1] == 100)
55: 		{
56: 			m_isinitialized = true;
57: 		}
58: 	}
59: 
60: 	public override int goHome(bool rehome, int axis, int speed, bool waitforstop = true)
61: 	{
62: 		if (ClassBaseBoard.m_24Vdropped)
63: 		{
64: 			throw new Exception("Lost 24V power go home");
65: 		}
66: 		if (!m_isinitialized)
67: 		{
68: 			return 1;
69: 		}
70: 		if (m_Motors[axis].MotorHome && m_Motors[axis].CurrentPosition == 0)
71: 		{
72: 			return 0;
73: 		}
74: 		m_searccHomeSpeed[axis] = speed;
75: 		int result = 0;
76: 		if (rehome)
77: 		{
78: 			result = moveToAbs(axis, 10000);
79: 		}
80: 		moveLeft(axis, speed);
```

## ClassThermalBoard.cs:110-130
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_can/ClassCanLib/ClassThermalBoard.cs`

```csharp
110: 			}
111: 			finally
112: 			{
113: 				Thread.Sleep(500);
114: 			}
115: 		}
116: 	}
117: 
118: 	public override int goHome(bool rehome, int axis, int speed, bool waitforstop = true)
119: 	{
120: 		if (ClassBaseBoard.m_24Vdropped)
121: 		{
122: 			throw new Exception("Lost 24V power go home");
123: 		}
124: 		if (!m_isinitialized)
125: 		{
126: 			return 1;
127: 		}
128: 		if (m_Motors[axis].MotorHome && m_Motors[axis].CurrentPosition == 0)
129: 		{
130: 			return 0;
```

## ClassBioXPSettings.cs:220-275
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`

```csharp
220: 
221: 	private string[] m_languageTagList;
222: 
223: 	private string m_currentLanguage;
224: 
225: 	private string m_curranteLanguageTag;
226: 
227: 	private int m_originOffsetG;
228: 
229: 	private int m_GripperClosePOS;
230: 
231: 	private int m_GripperOpenPOS;
232: 
233: 	private int m_GripperOpenWide;
234: 
235: 	private int m_calibrated;
236: 
237: 	private double m_outlierRangeFactor = 4.0;
238: 
239: 	private string[] m_tipType = new string[3] { "TypeA", "TypeB", "TypeC" };
240: 
241: 	private Color[] m_tipColor = (Color[])(object)new Color[3]
242: 	{
243: 		Colors.Red,
244: 		Colors.Green,
245: 		Colors.Blue
246: 	};
247: 
248: 	private int[] m_tipCapacity = new int[3] { 10, 25, 50 };
249: 
250: 	private int m_maxXsteps = 80000;
251: 
252: 	private int m_minXsteps;
253: 
254: 	private int m_maxYsteps = 80000;
255: 
256: 	private int m_minYsteps;
257: 
258: 	private int m_maxZsteps = 160000;
259: 
260: 	private int m_minZsteps;
261: 
262: 	private int m_maxGsteps = 15000;
263: 
264: 	private int m_minGsteps;
265: 
266: 	private int m_TCDoorOpen = 16000;
267: 
268: 	private int m_xSelfTestTravel;
269: 
270: 	private int m_ySelfTestTravel;
271: 
272: 	private int m_zSelfTestTravel;
273: 
274: 	private int m_cut_gclose_offset;
275: 
```

## ClassBioXPSettings.cs:3138-3160
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`

```csharp
3138: 						m_GripperClosePOS = Convert.ToInt32(list2[num2]);
3139: 						break;
3140: 					case "m_GripperOpenPOS":
3141: 						m_GripperOpenPOS = Convert.ToInt32(list2[num2]);
3142: 						break;
3143: 					case "m_GripperOpenWide":
3144: 						m_GripperOpenWide = Convert.ToInt32(list2[num2]);
3145: 						break;
3146: 					case "m_TCDoorOpen":
3147: 						m_TCDoorOpen = Convert.ToInt32(list2[num2]);
3148: 						break;
3149: 					case "m_TCDoorStallGuardThreshold":
3150: 						m_TCDoorStallGuardThreshold = Convert.ToInt32(list2[num2]);
3151: 						break;
3152: 					case "m_TC_DOOR_VELOCITY":
3153: 						m_TC_DOOR_VELOCITY = Convert.ToInt32(list2[num2]);
3154: 						break;
3155: 					case "m_TC_DOOR_ACCELERATION":
3156: 						m_TC_DOOR_ACCELERATION = Convert.ToInt32(list2[num2]);
3157: 						break;
3158: 					case "m_TC_DOOR_MAX_CURRENT":
3159: 						m_TC_DOOR_MAX_CURRENT = Convert.ToInt32(list2[num2]);
3160: 						break;
```

## ClassBioXPSettings.cs:3828-3855
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`

```csharp
3828: 				array2 = new string[1] { m_remoteLid.Value.ToString() };
3829: 				val.SetAttribute("GenBot", "Remote", array, array2);
3830: 			}
3831: 			val.SetAttribute("LanguageSetting", "Languages", m_languageList, m_languageTagList);
3832: 			array = new string[1] { m_currentLanguage };
3833: 			array2 = new string[1] { m_curranteLanguageTag };
3834: 			val.SetAttribute("LanguageSetting", "CurrentLanguage", array, array2);
3835: 			array = new string[14]
3836: 			{
3837: 				"m_originOffsetG", "m_GripperClosePOS", "m_GripperOpenPOS", "m_GripperOpenWide", "m_TCDoorOpen", "m_TCDoorStallGuardThreshold", "m_TC_DOOR_VELOCITY", "m_TC_DOOR_ACCELERATION", "m_TC_DOOR_MAX_CURRENT", "m_Z_MOTOR_MAX_CURRENT_DOWN",
3838: 				"m_Z_MOTOR_MAX_CURRENT_UP", "m_Z_MOTOR_STALL_GUARD_THRESHOLD", "OutPutBufferatMS_Zlow", "OutlierRangeFactor"
3839: 			};
3840: 			array2 = new string[14]
3841: 			{
3842: 				m_originOffsetG.ToString(),
3843: 				m_GripperClosePOS.ToString(),
3844: 				m_GripperOpenPOS.ToString(),
3845: 				m_GripperOpenWide.ToString(),
3846: 				m_TCDoorOpen.ToString(),
3847: 				m_TCDoorStallGuardThreshold.ToString(),
3848: 				m_TC_DOOR_VELOCITY.ToString(),
3849: 				m_TC_DOOR_ACCELERATION.ToString(),
3850: 				m_TC_DOOR_MAX_CURRENT.ToString(),
3851: 				m_Z_MOTOR_MAX_CURRENT_DOWN.ToString(),
3852: 				m_Z_MOTOR_MAX_CURRENT_UP.ToString(),
3853: 				m_Z_MOTOR_STALL_GUARD_THRESHOLD.ToString(),
3854: 				m_OPBufferLow.ToString(),
3855: 				m_outlierRangeFactor.ToString()
```

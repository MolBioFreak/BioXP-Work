## ClassControlInterface.cs:40-70
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
40: 			"MotorY",
41: 			new boardaxis
42: 			{
43: 				board = 0,
44: 				axis = 0
45: 			}
46: 		},
47: 		{
48: 			"MotorZ",
49: 			new boardaxis
50: 			{
51: 				board = 0,
52: 				axis = 1
53: 			}
54: 		},
55: 		{
56: 			"MotorGrip",
57: 			new boardaxis
58: 			{
59: 				board = 0,
60: 				axis = 2
61: 			}
62: 		},
63: 		{
64: 			"MotorLid",
65: 			new boardaxis
66: 			{
67: 				board = 2,
68: 				axis = 0
69: 			}
70: 		},
```

## ClassControlInterface.cs:2028-2055
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
2028: 	private void btnClose_Click(object sender, RoutedEventArgs e)
2029: 	{
2030: 		//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
2031: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] != null)
2032: 		{
2033: 			try
2034: 			{
2035: 				PageMotionControl motionControl = m_controlLib.m_diagnosticPanel.m_MotionControl;
2036: 				m_Boards[m_AxisIODesignater["MotorGrip"].board].moveToAbs(m_AxisIODesignater["MotorGrip"].axis, m_settingsWindow.m_gripperposition[(GripperStatus)2], true, false, false);
2037: 				motionControl.txtAbsPosG.Text = m_Boards[m_AxisIODesignater["MotorGrip"].board].getCurrentPosition(m_AxisIODesignater["MotorGrip"].axis).ToString();
2038: 			}
2039: 			catch (Exception ex)
2040: 			{
2041: 				MessageBox.Show(ex.Message);
2042: 			}
2043: 		}
2044: 	}
2045: 
2046: 	private void btnGripperHome_Click(object sender, RoutedEventArgs e)
2047: 	{
2048: 		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
2049: 		if (m_Boards[m_AxisIODesignater["MotorGrip"].board] == null)
2050: 		{
2051: 			return;
2052: 		}
2053: 		try
2054: 		{
2055: 			setGripperCurrent(31);
```

## ClassControlInterface.cs:2728-2755
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
2728: 			}
2729: 			return true;
2730: 		case "z":
2731: 			if (m_Boards[m_AxisIODesignater["MotorZ"].board] != null)
2732: 			{
2733: 				return m_Boards[m_AxisIODesignater["MotorZ"].board].queryHome(m_AxisIODesignater["MotorZ"].axis);
2734: 			}
2735: 			return true;
2736: 		case "g":
2737: 		case "gripper":
2738: 			if (m_Boards[m_AxisIODesignater["MotorGrip"].board] != null)
2739: 			{
2740: 				bool flag = m_Boards[m_AxisIODesignater["MotorGrip"].board].queryHome(m_AxisIODesignater["MotorGrip"].axis);
2741: 				if (!flag && getG() < 50)
2742: 				{
2743: 					flag = true;
2744: 				}
2745: 				return flag;
2746: 			}
2747: 			return true;
2748: 		case "tcDoorOpened":
2749: 			if (m_Boards[m_AxisIODesignater["ThermalDoor"].board] != null)
2750: 			{
2751: 				return m_Boards[m_AxisIODesignater["ThermalDoor"].board].queryRightSensor(m_AxisIODesignater["ThermalDoor"].axis);
2752: 			}
2753: 			return true;
2754: 		case "tcDoorClosed":
2755: 			if (m_Boards[m_AxisIODesignater["ThermalDoor"].board] != null)
```

## ClassControlInterface.cs:3330-3410
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs`

```csharp
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

## ControlLib.cs:1208-1230
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ControlLib.cs`

```csharp
1208: 		//IL_0273: Unknown result type (might be due to invalid IL or missing references)
1209: 		m_ControlInterface.UpdatePositionText();
1210: 		if ((int)ClassStatusLog.SavedStatus == 1)
1211: 		{
1212: 			((UIElement)m_diagnosticPanel.m_TPCalibration.btnFluidDetection).IsEnabled = true;
1213: 		}
1214: 		else
1215: 		{
1216: 			((UIElement)m_diagnosticPanel.m_TPCalibration.btnFluidDetection).IsEnabled = false;
1217: 		}
1218: 		bool num = m_ControlInterface.confirmAxis("tcDoorClosed");
1219: 		bool flag = m_ControlInterface.confirmAxis("tcDoorOpened");
1220: 		bool flag2 = m_ControlInterface.confirmAxis("gripper");
1221: 		bool flag3 = m_ControlInterface.confirmAxis("x");
1222: 		bool flag4 = m_ControlInterface.confirmAxis("y");
1223: 		bool flag5 = m_ControlInterface.confirmAxis("z");
1224: 		if (num)
1225: 		{
1226: 			m_diagnosticPanel.m_MotionControl.txtDoorClose.Text = "X";
1227: 			m_diagnosticPanel.m_MotionControl.txtDoorClose.FontWeight = FontWeights.Bold;
1228: 		}
1229: 		else
1230: 		{
```

## ControlLib.cs:8228-8245
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ControlLib.cs`

```csharp
8228: 			ClassMessageLog.LogStackTrace("releasePlate() error: " + text2);
8229: 		}
8230: 	}
8231: 
8232: 	private void sendZandGripperHome(bool runInParallel = true)
8233: 	{
8234: 		int g = m_ControlInterface.getG();
8235: 		int num = 0;
8236: 		if (g > m_settingsWindow.m_gripperposition[(GripperStatus)2])
8237: 		{
8238: 			num = 1;
8239: 		}
8240: 		if (runInParallel)
8241: 		{
8242: 			Task task = Task.Run(delegate
8243: 			{
8244: 				m_ControlInterface.moveZ(DefaultParameters.PSUDO_Z_HOME);
8245: 			});
```

## ClassHeadBoard.cs:90-122
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_can/ClassCanLib/ClassHeadBoard.cs`

```csharp
90: 				{
91: 					stopwatch.Stop();
92: 					string text = ((!(AxisName(axis) == "Gripper")) ? (m_boardAssy.ToString() + ": Axis: " + AxisName(axis) + " Move to left") : (m_boardAssy.ToString() + ": Axis: " + AxisName(axis) + " Move to home"));
93: 					ClassMessageLog.logMessage(text + " time out ****************", false);
94: 					ClassMessageLog.LogSystemMessage(text + " time out ****************", (SystemLogMessageType)1);
95: 					if (!(AxisName(axis) == "Gripper"))
96: 					{
97: 						stopMotor(axis, waitforstop: false);
98: 						throw new Exception(text + " time out");
99: 					}
100: 					int position = m_settings.m_gripperposition[(GripperStatus)4];
101: 					moveToAbs(axis, position, waitforstop: true, stallRecover: false, gripperRecover: true);
102: 				}
103: 			}
104: 			stopwatch.Stop();
105: 			bool flag = false;
106: 			while (!flag && DateTime.Now - now < timeSpan)
107: 			{
108: 				if (queryHome(axis))
109: 				{
110: 					stopMotor(axis, waitforstop: false);
111: 					flag = true;
112: 					result = -m_Motors[axis].queryActualPosition();
113: 					setHome(axis);
114: 					break;
115: 				}
116: 			}
117: 		}
118: 		return result;
119: 	}
120: 
121: 	public override void setHome(int axis)
122: 	{
```

## ClassBioXPSettings.cs:220-236
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
```

## ClassBioXPSettings.cs:650-675
Path: `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`

```csharp
650: 		},
651: 		{
652: 			(locationID)31,
653: 			new positionStruct
654: 			{
655: 				x = 0,
656: 				y = 0,
657: 				zLow = 0,
658: 				zDelta = 0,
659: 				inc_factor = 0
660: 			}
661: 		}
662: 	};
663: 
664: 	public Dictionary<locationID, positionStruct> m_positionTableOriginal;
665: 
666: 	public Dictionary<GripperStatus, int> m_gripperposition = new Dictionary<GripperStatus, int>();
667: 
668: 	private int m_gripperversion = 1;
669: 
670: 	private int m_troughversion = 1;
671: 
672: 	private string m_current_tool = "";
673: 
674: 	private DateTime m_current_cal_date;
675: 
```

資料に基づき、Real-time Feedbackのデータ構造（ビットテーブル）を作成しました。各パケットは1440バイトで構成され、リトルエンディアン（下位バイト優先）形式で格納されます。

| Meaning | Data type | Number of values | Size in bytes | Byte position value | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| MessageSize | unsigned short | 1 | 2 | 0000 – 0001 | Total message length in bytes |
| N/A | N/A | N/A | 6 | 0002 – 0007 | Reserved |
| DigitalInputs | uint64 | 1 | 8 | 0008 – 0015 | Current status of digital inputs. See DI/DO description. |
| DigitalOutputs | uint64 | 1 | 8 | 0016 – 0023 | Current status of digital outputs. See DI/DO description. |
| RobotMode | uint64 | 1 | 8 | 0024 – 0031 | Robot mode. See RobotMode. |
| TimeStamp | uint64 | 1 | 8 | 0032 – 0039 | Unix timestamp (unit: ms) |
| RunTime | uint64 | 1 | 8 | 0040 – 0047 | Robot running time (unit: ms) |
| TestValue | uint64 | 1 | 8 | 0048 – 0055 | Memory structure test standard value 0x0123 4567 89AB CDEF |
| N/A | N/A | N/A | 8 | 0056 – 0063 | Reserved |
| SpeedScaling | double | 1 | 8 | 0064 – 0071 | Speed ratio |
| N/A | N/A | N/A | 16 | 0072 – 0087 | Reserved |
| VRobot | double | 1 | 8 | 0088 – 0095 | Robot voltage |
| IRobot | double | 1 | 8 | 0096 – 0103 | Robot current |
| ProgramState | double | 1 | 8 | 0104 – 0111 | Script running status |
| SafetyIOIn | char | 2 | 2 | 0112 – 0113 | Safety IO input status |
| SafetyIOOut | char | 2 | 2 | 0114 – 0115 | Safety IO output status |
| N/A | N/A | N/A | 76 | 0116 – 0191 | Reserved |
| QTarget | double | 6 | 48 | 0192 – 0239 | Target joint position |
| QDTarget | double | 6 | 48 | 0240 – 0287 | Target joint speed |
| QDDTarget | double | 6 | 48 | 0288 – 0335 | Target joint acceleration |
| ITarget | double | 6 | 48 | 0336 – 0383 | Target joint current |
| MTarget | double | 6 | 48 | 0384 – 0431 | Target joint torque |
| QActual | double | 6 | 48 | 0432 – 0479 | Actual joint position |
| QDActual | double | 6 | 48 | 0480 – 0527 | Actual joint speed |
| IActual | double | 6 | 48 | 0528 – 0575 | Actual joint current |
| ActualTCPForce | double | 6 | 48 | 0576 – 0623 | TCP axes force (calculated by six-axis force original value) |
| ToolVectorActual | double | 6 | 48 | 0624 – 0671 | TCP actual Cartesian coordinates |
| TCPSpeedActual | double | 6 | 48 | 0672 – 0719 | TCP actual speed in Cartesian coordinate system |
| TCPForce | double | 6 | 48 | 0720 – 0767 | TCP force (calculate through joint current) |
| ToolVectorTarget | double | 6 | 48 | 0768 – 0815 | TCP target Cartesian coordinates |
| TCPSpeedTarget | double | 6 | 48 | 0816 – 0863 | TCP target Cartesian speed |
| MotorTemperatures| double | 6 | 48 | 0864 – 0911 | Joint temperature |
| JointModes | double | 6 | 48 | 0912 – 0959 | Joint control mode. 8: Position mode; 10: Torque mode |
| VActual | double | 6 | 48 | 0960 – 1007 | Joint voltage |
| HandType | char | 4 | 4 | 1008 – 1011 | Hand system (alternate parameter) |
| User | char | 1 | 1 | 1012 | User coordinate system |
| Tool | char | 1 | 1 | 1013 | Tool coordinate system |
| RunQueuedCmd | char | 1 | 1 | 1014 | Algorithm queue running flag |
| PauseCmdFlag | char | 1 | 1 | 1015 | Algorithm queue pause flag |
| VelocityRatio | char | 1 | 1 | 1016 | Joint speed ratio (0 – 100) |
| AccelerationRatio| char | 1 | 1 | 1017 | Joint acceleration ratio (0 – 100) |
| N/A | N/A | N/A | 1 | 1018 | Reserved |
| XYZVelocityRatio | char | 1 | 1 | 1019 | Cartesian position speed ratio (0 – 100) |
| RVelocityRatio | char | 1 | 1 | 1020 | Cartesian posture speed ratio (0 – 100) |
| XYZAccelerationRatio | char | 1 | 1 | 1021 | Cartesian position acceleration ratio (0 – 100) |
| RAccelerationRatio | char | 1 | 1 | 1022 | Cartesian posture acceleration ratio (0 – 100) |
| N/A | N/A | N/A | 2 | 1023 – 1024 | Reserved |
| BrakeStatus | char | 1 | 1 | 1025 | Brake status. See BrakeStatus description. |
| EnableStatus | char | 1 | 1 | 1026 | Enabling status |
| DragStatus | char | 1 | 1 | 1027 | Drag status. 0: Not in drag status; 1: Joint drag status; 2: Force-control drag status |
| RunningStatus | char | 1 | 1 | 1028 | Running status |
| ErrorStatus | char | 1 | 1 | 1029 | Alarm status |
| JogStatusCR | char | 1 | 1 | 1030 | Jog status |
| CRRobotType | char | 1 | 1 | 1031 | Robot type. See RobotType description. |
| DragButtonSignal | char | 1 | 1 | 1032 | Drag signal |
| EnableButtonSignal | char | 1 | 1 | 1033 | Enabling signal |
| RecordButtonSignal | char | 1 | 1 | 1034 | Recording signal |
| ReappearButtonSignal | char | 1 | 1 | 1035 | Playback signal |
| JawButtonSignal | char | 1 | 1 | 1036 | Gripper control signal |
| SixForceOnline | char | 1 | 1 | 1037 | Six-axis force sensor online status. 0: Offline; 1: Online; 2: Abnormal |
| CollisionState | char | 1 | 1 | 1038 | Collision status |
| ArmApproachState | char | 1 | 1 | 1039 | Forearm SafeSkin-approach-pause |
| J4ApproachState | char | 1 | 1 | 1040 | J4 SafeSkin-approach-pause |
| J5ApproachState | char | 1 | 1 | 1041 | J5 SafeSkin-approach-pause |
| J6ApproachState | char | 1 | 1 | 1042 | J6 SafeSkin-approach-pause |
| N/A | N/A | N/A | 61 | 1043 – 1103 | Reserved |
| VibrationDisZ | double | 1 | 8 | 1104 – 1111 | Z-axis jitter displacement measured by accelerometer |
| CurrentCommandId | uint64 | 1 | 8 | 1112 – 1119 | Current queue id |
| MActual | double | 6 | 48 | 1120 – 1167 | Actual torque of six joints |
| Load | double | 1 | 8 | 1168 – 1175 | Payload (kg) |
| CenterX | double | 1 | 8 | 1176 – 1183 | Eccentric distance in X-direction (mm) |
| CenterY | double | 1 | 8 | 1184 – 1191 | Eccentric distance in Y-direction (mm) |
| CenterZ | double | 1 | 8 | 1192 – 1199 | Eccentric distance in Z-direction (mm) |
| User | double | 6 | 48 | 1200 – 1247 | User coordinates |
| Tool | double | 6 | 48 | 1248 – 1295 | Tool coordinates |
| N/A | N/A | N/A | 8 | 1296 – 1303 | Reserved |
| SixForceValue | double | 6 | 48 | 1304 – 1351 | Six-axis force original value |
| TargetQuaternion | double | 4 | 32 | 1352 – 1383 | [qw,qx,qy,qz] Target quaternion |
| ActualQuaternion | double | 4 | 32 | 1384 – 1415 | [qw,qx,qy,qz] Actual quaternion |
| AutoManualMode | char | 1 | 2 | 1416 – 1417 | Manual/Automatic mode |
| ExportStatus | unsigned short | 1 | 2 | 1418 – 1419 | USB export status |
| SafetyState | char | 1 | 1 | 1420 | 1420 Safety status (0: Emergency stop, 1: Protective stop, 2: Reduced mode, 3: Non-stop status, 4: In motion, 5: System emergency stop, 6: User emergency stop, 7: Safety home output) |
| SafeState N/A | char | 1 | 1 | 1421 | Reserved for safety status |
| N/A | N/A | N/A | 18 | 1422 – 1439 | Reserved |
| **TOTAL** | | | **1440**| | **1440byte package** |

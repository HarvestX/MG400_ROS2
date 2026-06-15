Dobot TCP/IPリモートコントロールインターフェースで利用可能なコマンドをカテゴリ別にまとめました。

### 1. 制御コマンド (Control command)
デバイスの制御モード変更、起動・有効化、ドラッグモードの切り替えなど、ロボットの基本的な制御を行うコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **RequestControl** | Request to change the device control mode to TCP mode | Immediate command |
| **PowerOn** | Power on the robot | Immediate command |
| **EnableRobot** | Enable the robot | Immediate command |
| **DisableRobot** | Disable the robot | Immediate command |
| **ClearError** | Clear alarms of robot | Immediate command |
| **RunScript** | Run the project | Immediate command |
| **Stop** | Stop moving (or stop running the project) | Immediate command |
| **Pause** | Pause moving (or pause running the project) | Immediate command |
| **Continue** | Continue moving (or continue running the paused project) | Immediate command |
| **EmergencyStop** | Stop the robot in an emergency | Immediate command |
| **BrakeControl** | Control the brake of specified joint | Immediate command |
| **StartDrag** | Robot enters the drag mode | Immediate command |
| **StopDrag** | Robot exits the drag mode | Immediate command |

### 2. 設定コマンド (Settings command)
速度や加速度の比率、座標系、ペイロード、衝突検知、SafeSkinなどの各種パラメータを設定するコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **SpeedFactor** | Set global speed ratio | Immediate command |
| **User** | Set the global user coordinate system | Queue command |
| **SetUser** | Modify the specified user coordinate system | Immediate command |
| **CalcUser** | Calculate the user coordinate system | Immediate command |
| **Tool** | Set the global tool coordinate system | Queue command |
| **SetTool** | Modify the specified tool coordinate system | Immediate command |
| **CalcTool** | Calculate the tool coordinate system | Immediate command |
| **SetPayload** | Set payload | Queue command |
| **AccJ** | Set the acceleration ratio for joint motion | Immediate command |
| **AccL** | Set the acceleration ratio for linear and arc motion | Immediate command |
| **VelJ** | Set the velocity ratio for joint motion | Immediate command |
| **VelL** | Set the velocity ratio for linear and arc motion | Immediate command |
| **CP** | Set CP ratio | Immediate command |
| **SetCollisionLevel** | Set collision detection level | Queue command |
| **SetBackDistance** | Set collision backoff distance | Queue command |
| **SetPostCollisionMode** | Set post-collision handling mode | Queue command |
| **DragSensivity** | Set drag sensitivity | Immediate command |
| **EnableSafeSkin** | Enable or disable SafeSkin | Queue command |
| **SetSafeSkin** | Set the sensitivity for each part of the SafeSkin | Queue command |
| **SetSafeWallEnable** | Enable or disable the specified safety wall | Queue command |
| **SetWorkZoneEnable** | Enable or disable the specified safety zone | Queue command |

### 3. 計算および取得コマンド (Calculating and obtaining command)
ロボットの現在のステータスやエラーコード、座標・姿勢の取得、トレイの作成などを行うコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **RobotMode** | Get current status of robot | Immediate command |
| **PositiveKin** | Forward kinematics | Immediate command |
| **InverseKin** | Inverse kinematics | Immediate command |
| **GetAngle** | Get joint coordinates of current posture | Immediate command |
| **GetPose** | Get Cartesian coordinates of current posture under the specific coordinate system | Immediate command |
| **GetErrorID** | Get current error code of robot | Immediate command |
| **CreateTray** | Create tray | Immediate command |
| **GetTrayPoint** | Get tray point | Immediate command |

### 4. IOコマンド (IO command)
デジタル入出力（DO/DI）やアナログ入出力（AO/AI）、エンドツールのRS485や電源状態などを設定・取得するコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **DO** | Set status of DO port | Queue command |
| **DOInstant** | Set status of DO port | Immediate command |
| **GetDO** | Get status of DO port | Immediate command |
| **DOGroup** | Set status of multiple DO ports | Queue command |
| **GetDOGroup** | Get status of multiple DO ports | Immediate command |
| **ToolDO** | Set status of tool DO port | Queue command |
| **ToolDOInstant** | Set status of tool DO port | Immediate command |
| **GetToolDO** | Get status of tool DO port | Immediate command |
| **AO** | Set value of AO port | Queue command |
| **AOInstant** | Set value of AO port | Immediate command |
| **GetAO** | Get value of AO port | Immediate command |
| **DI** | Get status of DI port | Immediate command |
| **DIGroup** | Get status of multiple DI ports | Immediate command |
| **ToolDI** | Get status of tool DI port | Immediate command |
| **AI** | Get value of AI port | Immediate command |
| **ToolAI** | Get value of tool AI port | Immediate command |
| **SetTool485** | Set data type corresponding to RS485 interface of end tool | Immediate command |
| **SetToolPower** | Set power status of end tool | Immediate command |
| **SetToolMode** | Set the mode of tool multiplex terminal | Immediate command |

### 5. Modbusコマンド (Modbus command)
Modbusマスターの作成や、スレーブとの通信（レジスタやコイルの読み書き）を行うコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **ModbusCreate** | Create Modbus master | Immediate command |
| **ModbusRTUCreate** | Create Modbus master based on RS485 | Immediate command |
| **ModbusClose** | Disconnect with Modbus slave | Immediate command |
| **GetInBits** | Read contact registers | Immediate command |
| **GetInRegs** | Read input registers | Immediate command |
| **GetCoils** | Read coil registers | Immediate command |
| **SetCoils** | Write to coil registers | Immediate command |
| **GetHoldRegs** | Read holding registers | Immediate command |
| **SetHoldRegs** | Write to holding registers | Immediate command |

### 6. バスレジスタコマンド (Bus register command)
ProfinetまたはEthernet/IPバスレジスタの値（Bool, Int, Float）を読み書きするためのコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **GetInputBool** | Get boolean value from the specified input register address | Immediate command |
| **GetInputInt** | Get int value from the specified input register address | Immediate command |
| **GetInputFloat**| Get float value from the specified input register address | Immediate command |
| **GetOutputBool**| Get boolean value from the specified output register address | Immediate command |
| **GetOutputInt** | Get int value from the specified output register address | Immediate command |
| **GetOutputFloat**| Get float value from the specified output register address | Immediate command |
| **SetOutputBool**| Set boolean value at the specified output register address | Immediate command |
| **SetOutputInt** | Set int value at the specified output register address | Immediate command |
| **SetOutputFloat**| Set float value at the specified output register address | Immediate command |

### 7. モーションコマンド (Motion command)
各ジョイントや直交座標系に基づく移動、円弧補間、軌跡の再生、相対移動などを実行するコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **MovJ** | Joint motion | Queue command |
| **MovL** | Linear motion | Queue command |
| **MovLIO** | Move in linear mode and output DO | Queue command |
| **MovJIO** | Move in joint mode and output DO | Queue command |
| **Arc** | Arc motion | Queue command |
| **Circle** | Circle motion | Queue command |
| **ServoJ** | Dynamic following command based on joint space | Queue command |
| **ServoP** | Dynamic following command based on Cartesian space | Queue command |
| **MoveJog** | Jog the robot | Immediate command |
| **RunTo** | Move to the specified point | Immediate command |
| **GetStartPose** | Get start point of specified trajectory | Immediate command |
| **StartPath** | Play back recorded trajectory | Queue command |
| **RelMovJTool** | Perform relative joint motion along the tool coordinate system | Queue command |
| **RelMovLTool** | Perform relative linear motion along the tool coordinate system | Queue command |
| **RelMovJUser** | Perform relative joint motion along the user coordinate system | Queue command |
| **RelMovLUser** | Perform relative linear motion along the user coordinate system | Queue command |
| **RelJointMovJ** | Perform relative joint motion along the joint coordinate system | Queue command |
| **RelPointTool** | Perform Cartesian point offset along the tool coordinate system | Immediate command |
| **RelPointUser** | Perform Cartesian point offset along the user coordinate system | Immediate command |
| **RelJoint** | Perform relative joint position offset | Immediate command |
| **GetCurrentCommandID** | Get algorithm queue ID of current command | Immediate command |

### 8. 軌跡復元コマンド (Trajectory recovery command)
一時停止したプロジェクトを再開する前に、ロボットの姿勢を一時停止時の状態に復元するためのコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **SetResumeOffset** | Set backoff distance for trajectory recovery | Immediate command |
| **PathRecovery** | Start trajectory recovery | Immediate command |
| **PathRecoveryStop** | Stop the robot during trajectory recovery | Immediate command |
| **PathRecoveryStatus** | Query the trajectory recovery status | Immediate command |

### 9. ログエクスポートコマンド (Log export command)
ロボットのログをUSBにエクスポートし、そのステータスを確認するコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **LogExportUSB** | Export robot logs to USB | Immediate command |
| **GetExportStatus**| Get log export status | Immediate command |

### 10. 力制御コマンド (Force control command)
6軸力センサーを利用し、力制御ドラッグモードや各種コンプライアンス制御パラメータを設定・操作するコマンドです。

| Command | Function | Command type |
| :--- | :--- | :--- |
| **EnableFTSensor** | Switch on/off the force sensor | Immediate command |
| **SixForceHome** | Zero the force sensor | Immediate command |
| **GetForce** | Get force sensor data | Immediate command |
| **ForceDriveMode** | Enter the force-control drag mode | Immediate command |
| **ForceDriveSpeed**| Set the force-control drag speed | Immediate command |
| **StopDrag** | Exit the drag mode | Immediate command |
| **FCForceMode** | Enable force control with user-specified parameters | Queue command |
| **FCSetDeviation** | Set the offset and posture deviation in force control mode | Immediate command |
| **FCSetForceLimit**| Set the maximum force limit | Immediate command |
| **FCSetMass** | Set the mass coefficients for each direction in force control mode| Immediate command |
| **FCSetStiffness** | Set the stiffness coefficients for each direction in force control mode | Immediate command |
| **FCSetDamping** | Set the damping coefficients for each direction in force control mode | Immediate command |
| **FCOff** | Exit the force control mode | Queue command |
| **FCSetForceSpeedLimit** | Set the speed limits for force control adjustment in each direction | Immediate command |
| **FCSetForce** | Adjust the constant force settings in real time | Immediate command |

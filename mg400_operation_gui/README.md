# MG400 ServoJ Operation GUI

MG400_ROS2のServoJ APIを操作するQt 5製のROS 2 GUIです。

## 機能

- RobotMode、ControlState、Lease IDの表示
- Enable、Disable、ClearErrorの呼び出し
- realtime feedbackの現在関節角の表示とtargetへの読込
- ServoJのenable/disableとactive leaseの表示
- ServoJ targetの周期送信開始と停止
- targetに対する一次ローパスフィルタ

ServoJ開始時は、急な移動を避けるため`realtime_feedback`の
`q_actual[0..3]`をtargetの初期値として読み込みます。Servoモードの開始だけでは
targetを送信しません。`Target送信開始`を押すと周期送信を開始します。

## ビルド

```bash
colcon build --packages-select mg400_operation_gui
source install/setup.bash
```

## 起動

```bash
ros2 run mg400_operation_gui mg400_operation_gui
```

MG400 nodeのnamespaceが`/mg400`以外の場合はremapしてください。

```bash
ros2 run mg400_operation_gui mg400_operation_gui --ros-args \
  -r __ns:=/robot1/mg400 \
  -p command_period_ms:=30
```

## 操作手順

1. MG400 nodeをconfigure、activateします。
2. 必要に応じてEnableを押し、RobotModeが`ENABLE`になることを確認します。
3. `現在値をスライダーへ読込`を押します。
4. `Servoモード開始`を押してLease IDを取得します。
5. `Target送信開始`を押し、関節スライダーを操作します。
6. 終了時は`Target送信停止`、`Servoモード終了`の順に押します。

ウィンドウを閉じる際にleaseを保持している場合、GUIは先に停止要求を送ります。

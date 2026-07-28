# mg400_operation_gui

MG400_ROS2 の ServoJ / ServoP APIを操作するQt 5製のROS 2 GUIです。

## 機能

- MG400の `Enable`、`Disable`、`ClearError`
- ServoJ / ServoPの選択
- Servo leaseの取得と返却（Servoモード開始 / 終了）
- 4軸関節角、またはX / Y / Z / Yawのスライダー操作
- `realtime_feedback` による現在関節角・TCP poseの逐次表示と読込
- RobotMode、ControlState、lease IDの表示
- ServoJ / ServoP targetの周期送信開始 / 停止
- スライダーtargetに対する一次ローパスフィルタとカットオフ周波数のUI設定

Servoモード開始時は、急な移動を避けるため `realtime_feedback` の最新値を読み込みます。ServoJには `q_actual[0..3]`、ServoPには `tool_vector_actual[0..3]`（X / Y / Z / R）を使用します。これらはMG400コントローラ単位のdegとmmなので、GUIへ直接設定します。明示的に現在値読込ボタンを押していない場合も、Servoモード開始時に自動で読み込みます。Servoモードの開始だけではtargetを送信せず、`Target送信開始` を押すと周期送信を開始します。送信targetは現在値を初期値とする一次ローパスフィルタを通るため、スライダー値が大きく変化しても滑らかに追従します。

## ビルド

MG400_ROS2のServo対応ブランチ（`ServoJ.msg`、`ServoP.msg`、`RealtimeFeedback.msg`、`ChangeControlState.srv`を含むもの）と同じcolcon workspaceでビルドしてください。

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to mg400_operation_gui --symlink-install
source install/setup.bash
```

## 起動

ROS namespaceはデフォルトで `/mg400` です。

```bash
ros2 run mg400_operation_gui mg400_operation_gui
```

別のROS namespaceを使用する場合は上書きできます。たとえばルートnamespaceで起動する場合は次のとおりです。

```bash
ros2 run mg400_operation_gui mg400_operation_gui --ros-args -r __ns:=/
```

ServoP targetのframeとGUIからのtarget更新周期はROSパラメータで変更できます。

```bash
ros2 run mg400_operation_gui mg400_operation_gui --ros-args \
  -p servo_p_frame_id:=mg400_origin_link \
  -p command_period_ms:=30
```

`command_period_ms` は10〜90 msへクランプされます。MG400ノード側の `servo.target_watchdog_ms` のデフォルトは30000 msです。Servoモード開始後、または最後のtarget受信後に30秒間targetが届かなければ、watchdogがServo leaseを解放してIDLEへ戻します。

`Target LPF cutoff` はUI上で0.1〜10.0 Hzに変更でき、デフォルトは1.0 Hzです。小さい値ほど滑らかになりますがtargetへの追従は遅くなり、大きい値ほど素早く追従します。送信中に変更した場合も次の周期から反映されます。

## 操作手順

1. MG400ノードを起動し、GUIにRobotModeとControlStateが表示され、操作ログに `realtime_feedback` の受信開始が出ることを確認します。
2. エラーがある場合は `ClearError`、続いて `Enable` を押します。
3. ServoJまたはServoPを選び、`現在値をスライダーへ読込` を押します。
4. 周囲の安全を確認して `Servoモード開始（lease取得）` を押します。この時点ではtargetは送信されません。
5. 必要に応じて `Target LPF cutoff` を調整してから `Target送信開始` を押し、スライダーでtargetを変更します。カットオフ周波数は送信中も変更できます。
6. targetの送信だけを止める場合は `Target送信停止` を押します。Servoモードは維持されますが、30秒間無指令が続くとwatchdogでIDLEへ戻ります。
7. 終了時は `Servoモード終了（IDLE）`、必要に応じて `Disable` の順に押します。

ServoJとServoPを直接切り替えることはできません。一度Servoモードを終了し、ControlStateがIDLEになったことを確認してから別モードを選択してください。

## 安全上の注意

- 実機周辺を立入禁止にし、非常停止をすぐ押せる状態で操作してください。
- GUIを閉じる際は指令送信を止め、保有中のleaseを返却してから終了します。通信断やプロセス異常時はMG400_ROS2側のtarget watchdogが安全停止を行います。
- ServoPの表示単位は位置がmm、Yawがdegです。ROS messageへはmとquaternionへ変換して送信します。
- スライダー範囲はGUI上の入力範囲であり、姿勢の到達可能性や周辺設備との干渉を保証しません。

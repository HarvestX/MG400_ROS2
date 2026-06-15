# upstream/humble 差分まとめ

作成日: 2026-06-12

## 比較条件

- 比較元: `upstream/humble` (`1a8c7344be44c52030f7216c0f387873ed91fd11`)
- 比較先: `breakingchange` / `HEAD` (`1519378bf874b1e3ed95d3382d5e62b87080fa05`)
- 比較コマンド: `git diff upstream/humble..HEAD`
- 差分規模: 39 files changed, 1618 insertions(+), 1637 deletions(-)

## 全体概要

このブランチでは、`upstream/humble` に対してリアルタイムフィードバックの ROS 化、ターミナル UI による監視・コマンド送信、URDF/RViz 構成の更新、TCP コマンド API の変更が追加されています。

一方で、従来 `mg400_node` にあった Qt ベースの joint state / command publisher GUI は削除されています。

## 主要変更

### リアルタイムフィードバック

- `mg400_msgs/msg/RealtimeFeedback.msg` を追加し、MG400 のリアルタイムデータを ROS メッセージとして扱えるようにしています。
- 追加された主なフィールドは、ロボットモード、速度倍率、DI/DO、目標/実測関節位置・速度・加速度・電流、TCP 姿勢・速度、TCP 力、ペイロード情報です。
- `mg400_node` 本体がリアルタイムフィードバック TCP から 10 ms 周期でデータを取得し、`realtime_feedback` トピックへ publish します。
- スタンドアロンの `realtime_feedback_publisher` 実行ファイルは持たず、publisher は `mg400_node` に一本化します。
- TCP 位置は mm から m、TCP 角度は deg から rad に変換して publish します。

### 監視・コマンド送信用 TUI

- 新規パッケージ `mg400_monitor` が追加されています。
- FTXUI を使った `realtime_feedback_monitor` が追加され、`/realtime_feedback` を購読して状態、関節値、TCP、I/O、電流、目標値などを表示します。
- `motion_commander_chat` と `dashboard_commander_chat` が追加され、Motion ポート / Dashboard ポートへチャット風 UI から直接コマンドを送信できます。
- `mg400_monitor/CMakeLists.txt` は FTXUI v5.0.0 を `FetchContent` で取得します。ビルド時にネットワークアクセスが必要になる可能性があります。

### TCP コマンド API

- `DashboardTcpInterface::sendCommand()` と `MotionTcpInterface::sendCommand()` が `void` から `bool` 戻り値に変わっています。
- コマンド文字列は ASCII かつ `MessageName(Param1,Param2,...)` 形式か検証され、不正な場合は送信せず `false` を返します。
- `recvResponse()` が追加され、`;` 区切りのレスポンスを 3 秒タイムアウトで受信します。タイムアウト時は `"TIMEOUT ERROR"` を返します。
- `TcpSocketHandler::recvDelimited()` が追加され、区切り文字までの TCP 受信に対応しています。
- `CommandBuilder` が追加され、Dashboard / Motion コマンド文字列を組み立てる API が定義されています。

### URDF / RViz / launch

- `mg400_description` に `launch/display.launch.py` と `rviz/display.rviz` が追加され、`launch` と `rviz` が install 対象に追加されています。
- 既存の `mg400_bringup/launch/display.launch.py` と `mg400_bringup/rviz/display.rviz` は削除されています。
- ルート側のリンク名が `arm_frame_link_offset` から `base_link` に変わっています。
- `mg400.xacro` に mimic joint と virtual link が追加され、MoveIt2 で平行リンクを扱いやすい構成に更新されています。
- `mg400_bringup/rviz/mg400.rviz` は Fixed Frame を `base_link` に変更し、追加された virtual link を表示対象に含めています。

### GUI 削除

- `mg400_node/src/joint_state_publisher_gui` と `mg400_node/src/joint_command_publisher_gui` が削除されています。
- `mg400_node/package.xml` から `qtbase5-dev` と `qt5-qmake` の build depend が削除されています。
- `mg400_node/CMakeLists.txt` からも Qt 検出は削除されています。

### ドキュメント

- `doc/mg400_hardware_manual.pdf` と `doc/mg400_tcp_ip_manual.pdf` が追加されています。
- `doc/mg400_state.puml` が追加され、MG400 の状態遷移を PlantUML で表現しています。

## 影響範囲

### ビルド

- 新規パッケージとして `mg400_monitor` が増えています。
- `mg400_monitor` は CMake configure 時に FTXUI を GitHub から取得する構成です。
- `mg400_node` は Qt GUI と Qt 検出を削除しているため、Qt 未インストール環境でも `mg400_node` 側では止まらない想定です。
- `CommandBuilder` は `mg400_interface/src/command_builder.cpp` として追加されていますが、現状の `mg400_interface/CMakeLists.txt` のライブラリソース一覧には含まれていません。利用開始時にはリンク対象に追加する必要があります。

### ランタイム

- Motion / Dashboard への直接コマンド送信は `MessageName(...)` 形式である必要があります。
- `sendCommand()` の戻り値が `bool` に変わったため、呼び出し側は送信失敗を扱えます。
- `recvResponse()` は `;` を終端として扱います。レスポンス仕様が異なるコマンドではタイムアウト扱いになる可能性があります。
### ROS インターフェース

- 新規トピック `realtime_feedback` が追加されます。
- `RealtimeFeedback.msg` では TCP 位置が m、TCP 角度が rad として publish されます。
- モニタ側では表示用に m から mm、rad から deg に戻しています。

## 変更ファイル一覧

### 追加

- `doc/mg400_hardware_manual.pdf`
- `doc/mg400_state.puml`
- `doc/mg400_tcp_ip_manual.pdf`
- `mg400_description/launch/display.launch.py`
- `mg400_description/rviz/display.rviz`
- `mg400_interface/include/mg400_interface/command_builder.hpp`
- `mg400_interface/src/command_builder.cpp`
- `mg400_monitor/CMakeLists.txt`
- `mg400_monitor/include/mg400_test/realtime_feedback_monitor.hpp`
- `mg400_monitor/package.xml`
- `mg400_monitor/src/dashboard_commander_chat.cpp`
- `mg400_monitor/src/motion_commander_chat.cpp`
- `mg400_monitor/src/realtime_feedback_monitor.cpp`
- `mg400_msgs/msg/RealtimeFeedback.msg`

### 変更

- `mg400_bringup/rviz/mg400.rviz`
- `mg400_description/CMakeLists.txt`
- `mg400_description/urdf/mg400.urdf.xacro`
- `mg400_description/urdf/mg400.xacro`
- `mg400_interface/include/mg400_interface/tcp_interface/dashboard_tcp_interface.hpp`
- `mg400_interface/include/mg400_interface/tcp_interface/motion_tcp_interface.hpp`
- `mg400_interface/include/mg400_interface/tcp_interface/tcp_socket_handler.hpp`
- `mg400_interface/src/joint_handler.cpp`
- `mg400_interface/src/tcp_interface/dashboard_tcp_interface.cpp`
- `mg400_interface/src/tcp_interface/motion_tcp_interface.cpp`
- `mg400_interface/src/tcp_interface/realtime_feedback_tcp_interface.cpp`
- `mg400_interface/src/tcp_interface/tcp_socket_handler.cpp`
- `mg400_interface/test/src/commander/test_dashboard_commander.cpp`
- `mg400_interface/test/src/commander/test_motion_commander.cpp`
- `mg400_node/CMakeLists.txt`
- `mg400_node/package.xml`

### 削除

- `mg400_bringup/launch/display.launch.py`
- `mg400_bringup/rviz/display.rviz`
- `mg400_node/src/joint_command_publisher_gui/main.cpp`
- `mg400_node/src/joint_command_publisher_gui/main_window.cpp`
- `mg400_node/src/joint_command_publisher_gui/main_window.hpp`
- `mg400_node/src/joint_command_publisher_gui/main_window.ui`
- `mg400_node/src/joint_state_publisher_gui/main.cpp`
- `mg400_node/src/joint_state_publisher_gui/main_window.cpp`
- `mg400_node/src/joint_state_publisher_gui/main_window.hpp`
- `mg400_node/src/joint_state_publisher_gui/main_window.ui`
- `mg400_node/include/mg400_node/realtime_feedback_publisher.hpp`
- `mg400_node/src/realtime_feedback_publisher.cpp`

## 代表的な独自コミット

- `edcce50` feat: add realtime feedback monitor using FTXUI
- `5a49229` feat: implement RealtimeFeedbackPublisher for comprehensive robot feedback
- `8a94adc` refactor: remove joint state and command publisher GUIs; add realtime feedback publisher
- `c5285e1` Add Commander Chat interface for MG400 robot control
- `7e9d285` feat: add MG400 robot URDF with mimic joints and joint limits for MoveIt2 compatibility
- `792556a` feat: update MotionTcpInterface to return success status and add response handling
- `b7e3a4e` feat: update sendCommand method to return success status and add message format validation
- `9cb7f2f` feat: add standalone realtime feedback monitor implementation and CMake configuration
- `83f38f5` feat: implement J4 admittance control with initialization and toggling functionality
- `1989b5f` feat: add collision detection level reset and robot enabling sequence

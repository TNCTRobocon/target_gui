# target_gui

タッチパネル向けの射出電流設定GUIです。ボタン操作ではモーターへの指令を送らず、次回の射出に使う設定値だけを更新します。

## 画面レイアウト

画面上部の操作領域は左・中央・右の3列構成です。左列に「高」「中」「低」「移動」を縦に並べ、中央列は上半分が「ブレーキ」、下半分が「リセット」、右列は上半分が「UP」、下半分が「DOWN」です。中央列と右列の最下端は左列の「移動」の下端に揃えます。電流設定値とERROR / WAIT / OKは、操作領域の下にある画面下部全幅の領域へ左右に分けて表示します。

- 左側の「高」「中」「低」ボタン：それぞれ `manual_bucket_1_current_a`、`manual_bucket_2_current_a`、`manual_bucket_3_current_a` の値を設定（値は `shoot_belt/config/shoot_belt.yaml` で設定）
- 左側の「移動」ボタン：「低」と同じ設定更新処理で `manual_move_current_a` の値を設定（値は `shoot_belt/config/shoot_belt.yaml` で設定）
- 右上の「UP」ボタン：現在の指令値を YAML の増減幅だけ増加
- 右列下側の「DOWN」ボタン：現在の指令値を YAML の増減幅だけ減少
- 中央列上側の「ブレーキ」ボタン：GUI表示・ログ確認用。クリックしてもROSトピック、Action、CAN指令を送信しない
- 中央列下側の「リセット」ボタン：GUI表示・ログ確認用。クリックしてもROSトピック、Action、CAN指令を送信しない
- 画面下部全幅：左半分に現在の射出電流設定値、右半分に射出Action受付状態 `ERROR` / `WAIT` / `OK` を大きく表示

状態表示は `shoot_belt` の `/shoot_belt/operation_state` (`std_msgs/msg/UInt8`) を使用します。状態通知が `operation_state_timeout_sec` 秒以上途絶えた場合は、古い状態を残さず `ERROR` を表示します。

表示値は実測電流ではなく、次回の射出に使う設定値です。「ブレーキ」「リセット」は現在GUIのみのボタンで、クリック時にはログを出すだけです。制御処理やROS通信は実装していません。ボタン操作そのものでは、モーターの制御モード変更や電流・CAN指令送信を行いません。

## 起動

まず SocketCAN と shoot_belt を起動してください。GUI はモーターの CAN フレームを直接送らず、/shoot_belt/manual_current サービスを通じて制御ノードへ要求します。

実行するコマンド:

    # 端末1・2: 既存の ros2_socketcan sender/receiver を起動
    # 端末3:
    source /opt/ros/jazzy/setup.bash
    cd ~/ros2_ws
    colcon build --packages-up-to target_gui --symlink-install
    source install/setup.bash
    ros2 launch shoot_belt shoot_belt.launch.py

    # 端末4:
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch target_gui target_gui.launch.py

GUI の full_screen は config/target_gui.yaml で変更できます。Esc で終了、F11 で全画面を切り替えます。

## 設定

増減幅と制限は shoot_belt/config/shoot_belt.yaml にあります。

    manual_fixed_bucket_current_a: 1.0
    manual_current_step_a: 1.0
    manual_min_current_a: 0.1
    manual_max_current_a: 10.0

これらは初期設定例です。manual_max_current_a を含む上下限は、実際のモーター、VESC、電源、伝達機構の許容値を確認してから設定してください。現在の上限値 10 A が実機で安全であることを保証するものではありません。

## 通信

- サービス /shoot_belt/manual_current (shoot_belt/srv/ManualCurrent)：射出電流設定値の変更のみ
- 状態トピック /shoot_belt/manual_current_state (std_msgs/msg/Float32, transient-local)：射出電流設定値
- 設定値の変更時にモーターの制御モード・CAN指令には触れません。シーケンス実行中に変更しても、実行中の射出は開始時に固定した電流値を使い、変更後の設定は次回以降の射出に反映します。

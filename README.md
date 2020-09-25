# ros_plain_serial

Arduino等の非OS環境下とRosの間の通信をUARTで簡易的に行うパッケージ。

## 動作環境

次の環境で動作させています。

- OS        : Ubuntu 18.04 LTS  on WSL, Server,

- Ros       : Melodic ver.1.14.7

- Platforms : Arduino 1.8.12

## 説明

Arduino UnoでRosSerialを運用するのは難しいと感じたので軽量で最低限な通信でノードを構成できるパッケージを作りました。
現在配信しているものは、オドメトリと速度指示値のみです。

## 前提パッケージ

- "ros-melodic-geometry2"


## 配信メッセージ

- **"plain_serial_TF2" ノード** 

    実用を想定したノードです。tf2を利用して姿勢計算を行います。

    - "/plain_serial/cmd_vel" : geometry_msgs.msg/Twist

        送信する指示値[m/s,ras/s]。(x, y, yaw, のみに軸を制限)

    - "/plain_serial/odometry" : geometry_msgs.msg/Odometry

        受信した現在のオドメトリ値[m,rad(四元数)]。(x, y, yaw, のみに軸を制限)

    - "/tf" : tfメッセージ

        現在の姿勢を、"ps_odom"ローカル座標の"base_link"に表現しています。

## 配信サービス

- "BoolCommand.srv"

    8つ分のture/falseを送信します。リセットフラグを送信するときなどに使います。

## 備考

Arduino側のプロジェクト
https://github.com/TaiyouKomazawa16/PlainSerial

TF2について
http://wiki.ros.org/tf2


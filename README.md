# ROS2 seed_robot_ros2_pkgセットアップ

## 参考
- [ROS2 JazzyJalisco : Installation Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 1.事前準備
1. 全てのパッケージをupgradeする
    ```
    $ sudo apt update
    $ sudo apt upgrade
    ```
* __注意事項 :__  
      ros2_controlのバージョン不整合によるクラッシュ（26/7/16 時点）を確認しております。  
      26/8/7より前の当パッケージを使用されている場合は``git pull``でファイルを更新するか再度 "3.seed_robot_ros2_pkgインストール"を実行してください
   
2. pip3インストール
    ```terminal
    $ sudo apt install python3-pip
    ```

## 2. ROS2 jazzyインストール
公式サイトの手順に従いROS2 jazzyをインストールする
1. インストール  
[ROS2 jazzy : Installation Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. 環境設定  
[Configuring environment](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)  
※「3.1 The ROS_DOMAIN_ID variable」の設定は不要, 

3. ビルドの確認  
[Using colcon to build packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)  
4. colcon cleanをインストール
    ```terminal
    $ sudo apt install python3-colcon-clean 
    ```
5. rosdep初期化
    ```terminal
    $ sudo rosdep init
    $ rosdep update
    ```
    ※colcon cleanをインストールすると以下のコマンドでビルドによる出力内容を削除することが出来る 
    ワークスペース以外で誤ってビルドを行った場合などビルド結果を削除するのに便利
    ```
    $ colcon clean workspace
    ```

## 3.seed_robot_ros2_pkgインストール
1. ワークスペース作成 
    (ワークスペース名は任意で良いですがここでは"ros2_ws"として説明します)
    ```
    $ mkdir -p ~/ros2_ws/src
    ```
2. seed_ros2_pkgをインストール
    ```terminal
    $ cd ~/ros2_ws/src
    $ git clone --recurse-submodules https://github.com/thkrrc1/seed_robot_ros2_pkg.git
    ```
3. その他パッケージインストール
    ```
    $ sudo apt install ros-jazzy-nav2-bringup
    $ sudo apt-get install ros-jazzy-laser-proc
    $ sudo apt-get install ros-jazzy-laser-filters
    $ sudo apt-get install ros-jazzy-ros2-control-cmake
    $ sudo apt-get install ros-jazzy-tf-transformations
    $ sudo apt-get install ros-jazzy-moveit
    $ sudo apt install ros-jazzy-moveit-ros-planning-interface
    $ sudo apt install ros-jazzy-moveit-core ros-jazzy-moveit-common ros-jazzy-moveit-ros-planning
    $ sudo apt install ros-jazzy-moveit-visual-tools
    $ sudo apt install mplayer
    $ sudo apt install ros-jazzy-joy-linux
    $ sudo apt install ros-jazzy-joy-linux-dbgsym
    ```
4. パッチの適用
    ```
    $ cd ~/ros2_ws/src/seed_robot_ros2_pkg
    $ patch -p0 < patch/urg_node2.patch
    ```
5. ロボットプロジェクトのクローン
    ```
    $ cd ~/ros2_ws/src/seed_robot_ros2_pkg/robots
    $ python3 clone_robots.py
    ```
    実行すると下記メッセージが表示されるので、ロボット名を入力してください。（例：lifter_mover）  
    プロジェクトが存在する場合はクローンが開始されます。
    ```
    クローンしたいロボット名を入力してください：　lifter_mover
    ```
    
    ※　下記メッセージが表示されている場合はクローンに失敗しています。
    ```
    RuntimeError: コマンド失敗
    ```
6. ビルド
    ```
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    $ cd ~/ros2_ws
    $ source install/setup.bash
    ```
## 4.Udevの設定（ロボットを持っている場合）
ロボットのUSBをPCにまだ登録していない場合は、登録する必要があります。
(``/etc/udev/rules.d/90-aero.rules``がすでにある場合、こちらの作業は必要ありません。)
1. スクリプトの実行
    ```terminal
    $ cd ~/ros2_ws/src/seed_robot_ros2_pkg/scripts
    $ ./make_udev_install.sh
    ```
    実行すると下記メッセージが表示されます。
    ```
    udevファイルをコピーします
    完了しました
    ```

## その他 : 細かいインストール(必要に応じて)
- ros2_control関係
```terminal
$ sudo apt install ros-jazzy-ros2-controllers ros-jazzy-ros2-control-test-assets ros-jazzy-ros2-control 
```

## 注意事項  
- ファームウェアのバージョンについて  
  実機に書き込まれているファームウェアのバージョンによっては取得できない情報がありますので予めご了承下さい。  
  また、バージョンによってはパラメータの変更が必要となる場合がございますのでご注意下さい。  
  ご不明点ございましたらTHKまでお問い合わせ下さい。  
  バージョンの確認方法は以下のコマンドを実行して確認して下さい。
  ```
  $ ros2 service call /aero_controller/get_version aero_controller_msgs/srv/GetVersion
  ```
- msid=1, version='0001072500'以降である場合  
  "seed_robot_ros2_pkg/robots/(ロボット名)/config/controllers/controller_settings_mechanum.yaml"内の"encoder_reset"-"enable"の"true" → "false"にして下さい。
  ```
  encoder_reset:
   enable: false  #encoder reset      
   msid: 1       #mover
  ```

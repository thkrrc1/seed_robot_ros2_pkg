# ROS2 seed_robot_ros2_pkgセットアップ

## 参考
- [ROS2 JazzyJalisco : Installation Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 1.事前準備
1. 全てのパッケージをupgradeする
    ```
    $ sudo apt update
    $ sudo apt upgrade
    ```
2. pip3インストール
    ```terminal
    $ sudo apt install python3-pip
    ```

## 2. ROS2 jazzyインストール
公式サイトの手順に従いROS2 jazzyをインストールする
1. インストール  
[ROS2 jazzy : Installation Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

1. 環境設定  
[Configuring environment](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)  
※「3.1 The ROS_DOMAIN_ID variable」の設定は不要, 

2. ビルドの確認  
[Using colcon to build packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)  
3. colcon cleanをインストール
    ```terminal
    $ sudo apt install python3-colcon-clean 
    ```
4. rosdep初期化
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
    $ git clone -b robots/lifter_mover --recurse-submodules https://github.com/thkrrc1/seed_robot_ros2_pkg.git
    ```
3. その他パッケージインストール
    ```
    $ sudo apt install ros-jazzy-nav2-bringup
    $ sudo apt-get install ros-jazzy-laser-proc
    $ sudo apt-get install ros-jazzy-laser-filters
    $ sudo apt-get install ros-jazzy-gz-ros2-control
    $ sudo apt-get install ros-jazzy-ros2-control-cmake
    ```
4. パッチの適用
    ```
    $ cd ~/ros2_ws/src/seed_robot_ros2_pkg
    $ patch -p0 < patch/urg_node2.patch
    ```
5. ビルド
    ```
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    $ cd ~/ros2_ws
    $ source install/setup.bash
    ```
    
以降は用途に応じて下記の作業に従ってください。  
※起動に失敗する場合は一度Ctrl+Cによる終了後、プロセスが残っていないかを確認してください。

## 4.slamによる地図作成
1a. Gazeboによるシミュレーション環境でslamを実行する場合
```terminal
 $ ros2 launch lifter_mover bringup_gazebo.launch.py slam:=true
```

1b. 実機を使用してslamを実行する場合
```terminal
 $ ros2 launch lifter_mover bringup_robot.launch.py slam:=true
```
一定時間経過するとLiDARの情報と地図が表示されるので、そこからロボットを動作させて地図作成を開始してください。

2. 地図を保存する場合
```terminal
 $ ros2 run lifter_mover save_map_client_node --ros-args -p map_topic:=map_nav -p map_url:=（.map/.yamlのファイル名）
```
※地図ファイルはlaunchファイルを起動した場所に保存されます。


## 5.amclを利用した自律移動
1a. Gazeboによるシミュレーション環境で自律移動を実行する場合
```terminal
 $ ros2 launch lifter_mover bringup_gazebo.launch.py slam:=false
```

1b. 実機を使用して自律移動を実行する場合
```terminal
 $ ros2 launch lifter_mover bringup_robot.launch.py slam:=false
```
一定時間経過するとLiDARの情報と地図が表示されるので、そこからロボットを動作させてください。

※読み込ませる地図ファイルを変更する場合は、launchファイルで地図名を指定している箇所がありますので該当箇所を編集してください。
デフォルトでは下記のように設定されています。
  - bringup_gazebo.launch.py　→　gz_test_map.yaml
  - bringup_robot.launch.py　→　scan_map.yaml


## その他 : 細かいインストール(必要に応じて)

- rqt-joint-trajectory-controller（lifterの関節角度を変化させる）
```terminal
$ sudo apt install ros-jazzy-rqt-joint-trajectory-controller 
```
- ros2_control関係
```terminal
$ sudo apt install ros-jazzy-ros2-controllers ros-jazzy-ros2-control-test-assets ros-jazzy-ros2-control 
```


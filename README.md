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
    $ git clone -b （ロボット名） --recurse-submodules https://github.com/thkrrc1/seed_robot_ros2_pkg.git
    ```
3. その他パッケージインストール
    ```
    $ sudo apt install ros-jazzy-nav2-bringup
    $ sudo apt-get install ros-jazzy-laser-proc
    $ sudo apt-get install ros-jazzy-laser-filters
    $ sudo apt-get install ros-jazzy-gz-ros2-control
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
    ```

## 4.実行
```terminal
 $ cd ~/ros2_ws
 $ source install/setup.bash
 $ ros2 launch (ロボット名) bringup_robot.launch.py
```
rvizにロボットが表示されればok  

以上

## その他 : 細かいインストール(必要に応じて)

- rqt-joint-trajectory-controller
```terminal
$ sudo apt install ros-jazzy-rqt-joint-trajectory-controller 
```
- ros2_control関係
```terminal
$ sudo apt install ros-jazzy-ros2-controllers ros-jazzy-ros2-control-test-assets ros-jazzy-ros2-control 
```


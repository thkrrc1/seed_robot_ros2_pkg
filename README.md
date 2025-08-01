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
    $ git clone --recurse-submodules https://github.com/thkrrc1/seed_robot_ros2_pkg.git
    ```
3. その他パッケージインストール
    ```
    $ sudo apt install ros-jazzy-nav2-bringup
    $ sudo apt-get install ros-jazzy-laser-proc
    $ sudo apt-get install ros-jazzy-laser-filters
    $ sudo apt-get install ros-jazzy-gz-ros2-control
    $ sudo apt-get install ros-jazzy-ros2-control-cmake
    $ sudo apt-get install ros-jazzy-tf-transformations
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
7. ビルド
    ```
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    $ cd ~/ros2_ws
    $ source install/setup.bash
    ```

## その他 : 細かいインストール(必要に応じて)
- ros2_control関係
```terminal
$ sudo apt install ros-jazzy-ros2-controllers ros-jazzy-ros2-control-test-assets ros-jazzy-ros2-control 
```

- Gazebo関係（sim<#>にはバージョンが入ります。）
```terminal
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install libgz-sim<#>-dev
```
    


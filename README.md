# ROS2 seed_robot_ros2_pkgセットアップ

## 参考
- [ROS2 JazzyJalisco : Installation Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 1.事前準備
1. 全てのパッケージをupgradeする
    ```
    $ sudo apt update
    $ sudo apt upgrade
    ```

2. ディスプレイサーバーをwaylandからX11に変更する
    1. 設定されているディスプレイサーバー確認
        ※ 実行結果が"wayland"ではなく"X11"の場合はそのままで良いです
        ``` terminal
        $ echo $XDG_SESSION_TYPE
        ```
    1. X11を使うために以下をインストール
        ```terminal
        $ sudo apt install xorg openbox
        ```
    1. 設定ファイルを書き換える
        ```terminal
        $ sudo nano /etc/gdm3/custom.conf
        ```
        ファイル内でコメントの箇所を探し"WaylandEnable=false"に設定
        ```
        [daemon]
        # Uncomment the line below to force the login screen to use Xorg
        WaylandEnable=false
        ```
    1. 再起動
        ```terminal
        $ reboot
        ```
        再度以下のコマンドを実行し"X11"と表示されればok
        ``` terminal
        $ echo $XDG_SESSION_TYPE
        ```
3. pip3インストール
    ```terminal
    $ sudo apt install python3-pip
    ```

4. .bashrc追記
    ```terminal
    $ echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    $ echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
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
3. slam_toolboxをインストール
    ```terminal
    $ cd ~/ros2_ws/src
    $ git clone -b jazzy https://github.com/SteveMacenski/slam_toolbox.git
    $ cd slam_toolbox
    $ git checkout 9b37f20c38890cc340cbabb1777b0d8af6c06f4d
    ```

4. その他パッケージインストール
    ```
    sudo apt install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-laser-proc \
    ros-jazzy-laser-filters \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-control-cmake \
    ros-jazzy-ros2-control-test-assets \
    ros-jazzy-ros2-controllers \
    ros-jazzy-tf-transformations \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-core \
    ros-jazzy-moveit-common \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-srdfdom \
    ros-jazzy-joy-linux \
    ros-jazzy-realsense2-camera \
    mplayer
    ```
5. パッチの適用
    ```
    $ cd ~/ros2_ws/src/seed_robot_ros2_pkg
    $ patch -p0 < patch/urg_node2.patch
    ```
6. ロボットプロジェクトのクローン
    ```
    $ cd ~/ros2_ws/src/seed_robot_ros2_pkg/robots
    $ python3 clone_robots.py
    ```
    実行すると下記メッセージが表示されるので、ロボット名を入力してください。（例：noid_lifter_mover）  
    プロジェクトが存在する場合はクローンが開始されます。
    ```
    クローンしたいロボット名を入力してください：　noid_lifter_mover
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

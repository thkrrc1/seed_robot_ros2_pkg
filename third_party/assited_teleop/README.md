# assisted_teleop for ROS2 Jazzy

ROS1 の `assisted_teleop` を ROS2 Jazzy / Nav2 / mecanum 向けに置き換えるための最小パッケージです。

## 目的

Joy / teleop から来る `cmd_vel` をそのまま `mechanum_controller` に入れず、Nav2 の local costmap を参照して安全側に制限します。

```text
teleop_twist_joy
  -> /cmd_vel_joy_raw
  -> assisted_teleop_node
  -> /mechanum_controller/cmd_vel_teleop
  -> mechanum_controller
```

## ROS1版との対応

| ROS1 | ROS2 Jazzy |
|---|---|
| `cmd_vel_recv` | `/cmd_vel_joy_raw` |
| `cmd_vel_send` | `/mechanum_controller/cmd_vel_teleop` |
| `TrajectoryPlannerROS::checkTrajectory()` | `checkTrajectory()` で local costmap 上の footprint cost を評価 |
| `costmap_2d::Costmap2DROS` | `/local_costmap/costmap` 購読 + `nav2_costmap_2d::Costmap2D` |
| `control_thread` | `rclcpp::TimerBase` |

## ビルド

```bash
cd ~/ros2/ros2_ws7/src
 # この assisted_teleop ディレクトリを配置
cd ~/ros2/ros2_ws7
colcon build --symlink-install --packages-select assisted_teleop
source install/setup.bash
```

## 起動

```bash
ros2 launch assisted_teleop assisted_teleop.launch.py
```

## 注意

- `teleop_twist_joy` の出力は `/cmd_vel` ではなく `/cmd_vel_joy_raw` にしてください。
- `mechanum_controller` の teleop 購読先が実際に `/mechanum_controller/cmd_vel_teleop` になっているか確認してください。

```bash
ros2 topic list | grep cmd_vel_teleop
```

- `fail_open_without_costmap: false` の場合、local costmap / footprint / TF が取れないと停止側になります。
- local costmap の `OccupancyGrid` は 0-100 表現で来るため、内部で Nav2 cost 0-254 に変換しています。

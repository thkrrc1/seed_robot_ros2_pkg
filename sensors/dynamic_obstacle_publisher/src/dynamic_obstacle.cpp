#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>

class DynamicObstaclePublisher : public rclcpp::Node
{
public:
    DynamicObstaclePublisher()
        : Node("dynamic_obstacle_publisher"), time_(0.0)
    {
        // 障害物の位置情報を配信するパブリッシャー
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("dynamic_obstacle", 10);

        // 0.1秒ごとに障害物の位置を更新
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DynamicObstaclePublisher::publishObstacle, this));
    }

private:
    void publishObstacle()
    {
        // 障害物の位置情報を格納するためのメッセージ
        auto obstacle_positions = std_msgs::msg::Float32MultiArray();
        
        // 現在の時刻（time_）を使って動的に障害物の位置を決定
        float x_pos = std::sin(time_) * 5.0;  // x座標
        float y_pos = std::cos(time_) * 5.0;  // y座標
        float z_pos = 0.1;                   // z座標（固定）

        // Float32MultiArrayに位置情報を格納
        obstacle_positions.data.push_back(x_pos);
        obstacle_positions.data.push_back(y_pos);
        obstacle_positions.data.push_back(z_pos);

        // メッセージにタイムスタンプを追加
        obstacle_positions.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());//配列の次元情報を設定する
        obstacle_positions.layout.dim[0].size = 3;          //要素数
        obstacle_positions.layout.dim[0].stride = 1;        //メモリの間隔（通常は１）

        // 障害物の位置情報をパブリッシュ
        publisher_->publish(obstacle_positions);

        // 時間の更新
        time_ += 0.1;
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObstaclePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

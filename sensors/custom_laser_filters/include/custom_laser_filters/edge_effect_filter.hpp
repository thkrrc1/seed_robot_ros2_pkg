#pragma once

#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <angles/angles.h>

namespace custom_laser_filters
{
class EdgeEffectFilter: public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
    double min_angle_ = 0;
    int neighbors_ = 0;

    ////////////////////////////////////////////////////////////////////////////////
    EdgeEffectFilter()
    {
    }

    bool configure() override
    {
        if (!filters::FilterBase < sensor_msgs::msg::LaserScan > ::getParam(std::string("min_angle"), min_angle_))
        {
            RCLCPP_ERROR(logging_interface_->get_logger(), "min_angle was not specified.\n");
            return false;
        }

        if (min_angle_ < 0 || 90 < min_angle_)
        {
            RCLCPP_ERROR(logging_interface_->get_logger(), "min_angle must be 0 <= min_angle <= 90\n");
            return false;
        }

        if (!filters::FilterBase < sensor_msgs::msg::LaserScan > ::getParam(std::string("neighbors"), neighbors_))
        {
            RCLCPP_ERROR(logging_interface_->get_logger(), "neighbors was not specified.\n");
            return false;
        }


        return true;
    }

    virtual ~EdgeEffectFilter()
    {
    }

    bool update(const sensor_msgs::msg::LaserScan &scan_in, sensor_msgs::msg::LaserScan &scan_out) override
    {
        scan_out = scan_in;

        std::vector<int> indices_edge;
        std::vector<int> edge_direction;

        for (unsigned int i = 0; i < scan_in.ranges.size(); i++)
        {
            int far_idx = i;
            int near_idx = i+1;
            if (near_idx >= (int) scan_in.ranges.size())
            {
                continue;
            }

            if (scan_in.ranges[far_idx] < scan_in.ranges[near_idx]) {
                std::swap(far_idx, near_idx);
            }

            float ang = fabs(scan_in.angle_increment);
            const float &r1 = scan_in.ranges[near_idx];
            const float &r2 = scan_in.ranges[far_idx];
            float diff_ang = std::atan2(r2 * sin(ang), r2 * cos(ang) - r1);
            if (diff_ang < angles::from_degrees(min_angle_)) { // エッジノイズの開始地点
                indices_edge.push_back(far_idx);
                edge_direction.push_back(far_idx > near_idx ? 1 : -1);
            }
        }

        //小領域であれば、取り除く
        size_t idx_size = indices_edge.size() > 0 ? indices_edge.size() -1 : 0;
        for (size_t idx = 0; idx < idx_size; ++idx) {
                int start_idx = indices_edge[idx];
                int end_idx = indices_edge[idx + 1] - 1;
                if (edge_direction[idx] < 0) {//近い方向にちらつきが出る
                    start_idx = indices_edge[idx]+1;
                }
                if(edge_direction[idx+1] < 0){
                    end_idx = indices_edge[idx + 1];
                }
                if (end_idx - start_idx <= neighbors_) {
                    for (int idx2 = start_idx; idx2 <= end_idx; ++idx2) {
                        scan_out.ranges[idx2] = std::numeric_limits<float>::quiet_NaN();
                    }
                }
        }

        //最後の要素だけ別途処理
        if (indices_edge.size() != 0) {
            int start_idx = indices_edge.back();
            int end_idx = scan_out.ranges.size() - 1;
            if (edge_direction.back() < 0) {
                start_idx = indices_edge.back() + 1;
            }
            if (end_idx - start_idx <= neighbors_) {
                for (int idx2 = start_idx; idx2 <= end_idx; ++idx2) {
                    scan_out.ranges[idx2] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        return true;
    }

};
}


#pragma once

#include <rcpputils/rolling_mean_accumulator.hpp>

namespace mechanum_controller {

struct Velocity{
    double vx = 0;
    double vy = 0;
    double vth = 0;
};

struct Pose{
    double x = 0;
    double y = 0;
    double th= 0;
};

class Odometry{

public:
    Odometry():
        vx_acc(rcpputils::RollingMeanAccumulator<double>(vel_rolling_window_size)),
        vy_acc(rcpputils::RollingMeanAccumulator<double>(vel_rolling_window_size)),
        vth_acc(rcpputils::RollingMeanAccumulator<double>(vel_rolling_window_size)){
    }

    Pose get_pose(){
        return global_pose;
    }

    Pose update(const Velocity &local_vel, double dt) {
        vx_acc.accumulate(local_vel.vx);
        vy_acc.accumulate(local_vel.vy);
        vth_acc.accumulate(local_vel.vth);
        local_vel_mean.vx = vx_acc.getRollingMean();
        local_vel_mean.vy = vy_acc.getRollingMean();
        local_vel_mean.vth = vth_acc.getRollingMean();

        auto &mean_vel = local_vel_mean; // 一つ前と今回の、平均速度を使う = 2次ルンゲクッタもどき

        //グローバル座標から見た位置
        Pose global_pose_next;
        global_pose_next.th = global_pose.th + mean_vel.vth * dt;

        if (fabs(mean_vel.vth) < 1e-6) {
            //回転成分がない場合
            global_pose_next.x = global_pose.x + (mean_vel.vx * cos(global_pose_next.th) - mean_vel.vy * sin(global_pose_next.th)) * dt;
            global_pose_next.y = global_pose.y + (mean_vel.vx * sin(global_pose_next.th) + mean_vel.vy * cos(global_pose_next.th)) * dt;
        } else {
            //xy成分の積分
            global_pose_next.x = global_pose.x + (mean_vel.vx / mean_vel.vth) * (sin(global_pose_next.th) - sin(global_pose.th)) + (mean_vel.vy / mean_vel.vth) * (cos(global_pose_next.th) - cos(global_pose.th));
            global_pose_next.y = global_pose.y - (mean_vel.vx / mean_vel.vth) * (cos(global_pose_next.th) - cos(global_pose.th)) + (mean_vel.vy / mean_vel.vth) * (sin(global_pose_next.th) - sin(global_pose.th));
        }

        global_pose = global_pose_next;

        //グローバル座標から見た速度
        global_vel.vth = mean_vel.vth;
        global_vel.vx = mean_vel.vx * cos(global_pose.th) - mean_vel.vy * sin(global_pose.th);
        global_vel.vy = mean_vel.vx * sin(global_pose.th) + mean_vel.vy * cos(global_pose.th);

        return global_pose;
    }


private:
    //グローバル座標から見た位置

    //速度移動平均
    size_t vel_rolling_window_size = 2;
    rcpputils::RollingMeanAccumulator<double> vx_acc;
    rcpputils::RollingMeanAccumulator<double> vy_acc;
    rcpputils::RollingMeanAccumulator<double> vth_acc;

    Pose global_pose;
    Velocity global_vel;
    Velocity local_vel_mean;

};
}

#pragma once

#include <seed_ros2_controller/stroke_converter/stroke_converter_base.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace seed_converter {

class LifterMover: public StrokeConverter {
public:

    LifterMover() {
    }
    void makeTables() override;
    void Angle2Stroke(std::vector<int16_t> &_strokes, const std::vector<double> &_angles) override;
    void Stroke2Angle(std::vector<double> &_angles, const std::vector<int16_t> &_strokes) override;
    void setJointNames(const std::vector<std::string> &names) override;

    void calcActuatorVel(std::vector<int16_t> &actuator_vel, const std::vector<double> &joint_vel) override;
    void calcJointVel(std::vector<double> &joint_vel, const std::vector<int16_t> &actuator_vel) override;

private:

    int16_t calcStroke1(double angle, double scale, double offset = 0);

    int16_t calcStroke2(double angle, double scale, int dir, double offset, const std::vector<seed_converter::StrokeMap> &_table);

    void calcStroke3(double angle1, double angle2, double scale, int dir1, int dir2, const std::vector<seed_converter::StrokeMap> &_table1, const std::vector<seed_converter::StrokeMap> &_table2, bool is_pitch, int16_t &out1, int16_t &out2);

    int16_t calcWheelAngDeg(int idx, int16_t raw_ang);

private:
    ConvertTable leg;
    std::string lifter_csv_dir;

    int16_t zero_ofst[4] = {0}; //ホイール角度ゼロのオフセット
    int16_t prev_wheel[4] = {0}; //ホイール角度ゼロのオフセット
    int16_t cur_wheel[4] = {0}; //ホイール角度ゼロのオフセット
    
    int idx_knee = 0;
    int idx_ankle = 0;
    int idx_wheel_front_left = 0;
    int idx_wheel_front_right = 0;
    int idx_wheel_rear_left = 0;
    int idx_wheel_rear_right = 0;

};

}


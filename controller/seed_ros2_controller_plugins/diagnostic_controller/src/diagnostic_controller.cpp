#include "diagnostic_controller.hpp"
#include "rt_logger/logger.hpp"

#include <aero_controller_msgs/srv/servo.hpp>

namespace diagnostic_controller {


controller_interface::CallbackReturn DiagnosticController::on_init(){
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(get_node());
    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add("Robot status", this, &DiagnosticController::setDiagnostics);
    auto_declare("auto_svon",true);
    return controller_interface::CallbackReturn::SUCCESS;
}

void DiagnosticController::activate_hardware(seed_ros2_controller::command_interface::StatusInterface& hw) {
    auto names = hw.get_names();
    if (names.size() != 1) {
        LOG_ERROR_STREAM() << "robot status handle was not specified." << LOG_END;
        return;
    }
    handle = hw.get_handle(names[0]);


    //svon_client = root_nh.serviceClient < aero_controller::Servo > ("aero_controller/servo");

    return;
}

void DiagnosticController::release_hardware(){
}

controller_interface::CallbackReturn DiagnosticController::on_configure(const rclcpp_lifecycle::State &previous_state){
    servo_client_ = get_node()->create_client < aero_controller_msgs::srv::Servo > ("/aero_controller/servo");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type DiagnosticController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
    auto status = handle.getCommandPtr();
    swp_buff_recv.writeFromA(*status, true);
    return controller_interface::return_type::OK;
}

void DiagnosticController::setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    struct MSSummary {
        union {
            SingleCANErrorInfo info;
            uint8_t info_u8;
        };
    };

    Status *cur_status = swp_buff_recv.readFromB();
    if (!cur_status) {
        return;
    }

    bool usb_err = 0;
    uint16_t ms_err = 0;

    for (int idx = 0; idx < cur_status->usb_num; ++idx) {
        usb_err |= cur_status->usb_status[idx].disconnect;
    }

    for (int idx = 0; idx < cur_status->ms_num; ++idx) {
        ms_err |= cur_status->ms_status[idx].err_u16;
    }

    MSSummary ms_sum;
    ms_sum.info_u8 = reinterpret_cast<uint8_t*>(&ms_err)[0] | reinterpret_cast<uint8_t*>(&ms_err)[1];
    bool estop = ms_sum.info.connection && ms_sum.info.calibration;

    if (estop) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "E-Stop switch is pushed, please release it");
    } else if (usb_err) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Now calibrating, or the USB cable is plugged out");
    } else if (ms_sum.info.power) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Power failed, please check the battery");
    } else if (ms_sum.info.connection) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Connection error occurred, please check the cable");
    } else if (ms_sum.info.temperature) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Motor driver is high temperature, please reboot the robot");
    } else if (ms_sum.info.protective_stopped) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Protective stopped, please release it");
    } else if (ms_sum.info.calibration) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Calibration error occurred, please re-calibration");
    } else if (ms_sum.info.step_out) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Step-out has occurred");
    } else if (ms_sum.info.response) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Response error has occurred");
    } else if (ms_sum.info.motor_stat) {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor servo is off");
    } else {
        stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "System all green");
    }

    stat.add("Emergency Stopped", estop);
    stat.add("Communication Error", usb_err);
    stat.add("Power Failed", ms_sum.info.power);
    stat.add("Connection Error", ms_sum.info.connection);
    stat.add("Temperature Error", ms_sum.info.temperature);
    stat.add("Protective Stopped", ms_sum.info.protective_stopped);
    stat.add("Calibration Error", ms_sum.info.calibration);
    stat.add("Step Out Occurred", ms_sum.info.step_out);
    stat.add("Response Error", ms_sum.info.response);
    stat.add("Motor Servo OFF", ms_sum.info.motor_stat);

    bool auto_svon = get_node()->get_parameter("auto_svon").as_bool();
    if (auto_svon && ms_sum.info.motor_stat) {
        //サーボOFFであれば、サーボONにする。
        using namespace std::chrono_literals;
        auto servo_req = std::make_shared<aero_controller_msgs::srv::Servo::Request>();
        servo_req->type = aero_controller_msgs::srv::Servo::Request::SERVO_ON;
        if (!servo_client_->wait_for_service(100ms)) {
            return;
        }
        servo_client_->async_send_request(servo_req);
        LOG_INFO_STREAM() << "exec servo on" << LOG_END;
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(diagnostic_controller::DiagnosticController, controller_interface::ControllerInterface)

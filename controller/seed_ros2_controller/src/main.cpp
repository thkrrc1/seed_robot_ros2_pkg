#include "main.hpp"
#include <map>
#include <hardware_interface/handle.hpp>

namespace seed_ros2_controller {

template<class SinkType>
void setLogLevel(SinkType &sink, const std::string &level) {
    std::map < std::string, spdlog::level::level_enum > levelList { { "trace", spdlog::level::trace }, { "debug", spdlog::level::debug }, { "info", spdlog::level::info }, { "warn", spdlog::level::warn }, { "error", spdlog::level::err }, { "fatal", spdlog::level::critical }, };

    if (levelList.count(level) != 0) {
        sink->set_level(levelList[level]);
    }
}

RobotHardware::~RobotHardware() {
    if (driver_) {
        delete driver_;
        driver_ = nullptr;
    }
}

hardware_interface::CallbackReturn RobotHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string uname = std::string(std::getenv("USER"));
    auto loggerCore = std::make_shared<rtlogger::SpdLogLogger>();
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto rotating_sink = std::make_shared < spdlog::sinks::rotating_file_sink_mt > ("logs/" + uname + "_log.txt", 1024 * 1024 * 10, 3, true);

    //ログ出力レベルを設定
    std::string file_log_level = "trace";
    std::string console_log_level = "debug";
    setLogLevel(console_sink, console_log_level);
    setLogLevel(rotating_sink, file_log_level);

    loggerCore->addSink(console_sink);
    loggerCore->addSink(rotating_sink);
    loggerCore->set_pattern("[%D %T.%e][%^%l%$][%!][pid:%P]%v");
    rtlogger::setLoggerCore(loggerCore);
    rtlogger::setProcessName("seed_ros2_controller");

    driver_ = new RobotDriver(info_.hardware_parameters["driver_settings"]);
    while(rclcpp::ok() && !driver_->open()){
        using namespace std::literals::chrono_literals;
        rclcpp::sleep_for(500ms);
    }

    int numJoints = driver_->getNumJoints();
    pos_recv_.resize(numJoints, std::numeric_limits<double>::quiet_NaN());
    vel_recv_.resize(numJoints, std::numeric_limits<double>::quiet_NaN());
    pos_send_.resize(numJoints, std::numeric_limits<double>::quiet_NaN());
    vel_send_.resize(numJoints, std::numeric_limits<double>::quiet_NaN());

    int numPorts = driver_->getNumPorts();
    int numMs = driver_->getNumMs();
    other_cmds_recv_.resize(numMs);
    other_cmds_send_.resize(numMs);
    status_recv_.usb_num = numPorts;
    status_recv_.ms_num = numMs;
    if (numPorts > status_recv_.get_usb_capacity()) {
        throw std::runtime_error("exceed usb capacity.");
    }
    if (numMs > status_recv_.get_ms_capacity()) {
        throw std::runtime_error("exceed ms capacity.");
    }


    for (int idx = 0; idx < numMs; ++idx) {
        int msid = driver_->getMsId(idx);
        auto joint_names = driver_->getJointNamesMessageOrder(msid);
        auto protocol = driver_->getProtocol(msid);
        command_interface::OtherCommandHandle oc_handle = command_interface::OtherCommandHandle(msid, joint_names, protocol, &other_cmds_recv_[idx], &other_cmds_send_[idx]);
        other_interf.register_handle(oc_handle);
    }

    command_interface::StatusHandle status_handle = command_interface::StatusHandle("status", &status_recv_);
    status_interf.register_handle(status_handle);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardware::on_configure(const rclcpp_lifecycle::State &previous_state) {    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (int idx = 0; idx < driver_->getNumJoints(); idx++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(driver_->getJointName(idx), hardware_interface::HW_IF_POSITION, &pos_recv_[idx]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(driver_->getJointName(idx), hardware_interface::HW_IF_VELOCITY, &vel_recv_[idx]));
    }
    other_interf.export_interfaces(state_interfaces);
    status_interf.export_interfaces(state_interfaces);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (int idx = 0; idx < driver_->getNumJoints(); idx++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(driver_->getJointName(idx), hardware_interface::HW_IF_POSITION, &pos_send_[idx]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(driver_->getJointName(idx), hardware_interface::HW_IF_VELOCITY, &vel_send_[idx]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn RobotHardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
    //有効になったら、現在の状態を拾ってきて、送信コマンドとする
    for (int idx = 0; idx < driver_->getNumJoints(); idx++) {
        pos_recv_[idx] = 0;
        vel_recv_[idx] = 0;
        pos_send_[idx] = std::numeric_limits<double>::quiet_NaN();
        vel_send_[idx] = 0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    driver_->read(time, period);
    driver_->getPos(pos_recv_);
    driver_->getVel(vel_recv_);
    driver_->getStatus(status_recv_);
    driver_->getOtherCommands(other_cmds_recv_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    driver_->sendPosition(pos_send_, 0);
    driver_->sendVelocity(vel_send_);
    driver_->sendOtherCommands(other_cmds_send_);
    driver_->sendIdleCommand();
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(seed_ros2_controller::RobotHardware, hardware_interface::SystemInterface)

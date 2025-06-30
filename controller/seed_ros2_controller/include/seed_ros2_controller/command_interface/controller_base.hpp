#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace seed_ros2_controller {
namespace command_interface {

template<class CommandInterface>
class ControllerBase  : public controller_interface::ControllerInterface{
public:
    virtual ~ControllerBase() = default;
    virtual void activate_hardware(CommandInterface &hw) = 0;
    virtual void release_hardware() = 0;

private:

    controller_interface::InterfaceConfiguration state_interface_configuration() const override final {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::ALL;
        return config;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override final {
        return controller_interface::InterfaceConfiguration { controller_interface::interface_configuration_type::NONE };
    }

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override final {
        std::map<std::string, double*> data_list;
        for (size_t idx = 0; idx < state_interfaces_.size(); ++idx) {
            hardware_interface::LoanedStateInterface &loaned_interface = state_interfaces_[idx];
            if (loaned_interface.get_interface_name() != interf_.get_interface_name()) {
                continue;
            }

            loaned_interface.delegate([&](const std::string &name, double *data) {
                data_list.emplace(name, data);
            });
        }
        for (auto& [name, data] : data_list) {
            interf_.register_handle(data);
        }
        activate_hardware(interf_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override final {
        release_hardware();
        interf_.release_handle();

        release_interfaces();
        return controller_interface::CallbackReturn::SUCCESS;
    }

private:
    CommandInterface interf_;
};

}
}

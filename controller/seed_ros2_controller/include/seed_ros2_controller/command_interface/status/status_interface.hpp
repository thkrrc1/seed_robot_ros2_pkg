#pragma once

#include <seed_ros2_controller/command_interface/command_interface.hpp>
#include <seed_ros2_controller/command_interface/status/robot_status.hpp>
#include <seed_ros2_controller/command_interface/status/value_handle.hpp>

namespace seed_ros2_controller{
namespace command_interface{

using StatusHandle = ValueHandle<Status>;

class StatusInterface : public CommandInterface<StatusHandle>{
public:
    std::string get_interface_name() const override{
        return "status";
    }
};

}
}

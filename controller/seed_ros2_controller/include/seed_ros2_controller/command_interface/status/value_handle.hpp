#pragma once

namespace seed_ros2_controller{
namespace command_interface{

template<class T>
class ValueHandle
{
public:
    ValueHandle() = default;

    ValueHandle(const std::string &name, T* cmd)
    : name_(name), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw std::runtime_error("Cannot create handle '" + name + "'. Command data pointer is null.");
    }
  }

  void setCommand(T command) {assert(cmd_); *cmd_ = command;}
  T getCommand() const {assert(cmd_); return *cmd_;}
  const T* getCommandPtr() const {assert(cmd_); return cmd_;}

  std::string get_name() const{
      return name_;
  }

private:
  std::string name_;
  T* cmd_ = nullptr;
};
}
}


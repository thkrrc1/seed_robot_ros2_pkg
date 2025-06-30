#pragma once

#include <hardware_interface/handle.hpp>
#include <map>

namespace seed_ros2_controller{
namespace command_interface{

template<class ResourceHandle>
class CommandInterface{
public:
    virtual ~CommandInterface(){}

    std::vector<std::string> get_names() const {
        std::vector < std::string > ret;
        for (const auto& [name, handle] : resource_map_) {
            ret.push_back(name);
        }
        return ret;
    }

    void register_handle(ResourceHandle handle) {
        resource_map_.emplace(handle.get_name(), handle);
    }

    void register_handle(void *handle) {
        auto handle_ptr = reinterpret_cast<ResourceHandle*>(handle);
        resource_map_.emplace(handle_ptr->get_name(), *handle_ptr);
    }

    void release_handle(){
        resource_map_.clear();
    }

    void export_interfaces(std::vector<hardware_interface::StateInterface> &command_interfaces){
        for (auto& [name, handle] : resource_map_) {
            command_interfaces.push_back(hardware_interface::StateInterface(name, get_interface_name(), reinterpret_cast<double*>(&handle)));
        }
    }

    virtual std::string get_interface_name() const = 0;

    ResourceHandle get_handle(const std::string& name){
        auto itr = resource_map_.find(name);
        if(itr == resource_map_.end()){
            throw std::logic_error("could not find resource");
        }

        return itr->second;
    }

    const ResourceHandle& get_handle(const std::string &name) const {
        auto itr = resource_map_.find(name);
        if (itr == resource_map_.end()) {
            throw std::logic_error("could not find resource");
        }

        return itr->second;
    }

    std::map<std::string, ResourceHandle> get_resource_map(){
        return resource_map_;
    }

private:
    std::map<std::string, ResourceHandle> resource_map_;
};

}
}

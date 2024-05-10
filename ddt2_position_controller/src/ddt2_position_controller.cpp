//
// Created by lsy on 24-3-22.
//

#include "ddt2_position_controller/ddt2_position_controller.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace ddt2_position_controller {
    using hardware_interface::LoanedCommandInterface;
    using hardware_interface::LoanedStateInterface;

    DDT2PositionController::DDT2PositionController()
            : controller_interface::ControllerInterface(),
              rt_command_ptr_(nullptr),
              joints_command_subscriber_(nullptr),
              velocity_limits_subscriber_(nullptr),
              velocity_limits_position_subscriber_(nullptr){}

    controller_interface::return_type
    DDT2PositionController::init(const std::string &controller_name) {
        auto ret = ControllerInterface::init(controller_name);
        if (ret != controller_interface::return_type::OK)
            return ret;
        try {
            auto_declare<std::vector<std::string>>("joints",
                                                   std::vector<std::string>());
            auto_declare<std::vector<std::string>>("command_interfaces",
                                                   command_interface_types_);
            auto_declare<std::vector<std::string>>("state_interfaces",
                                                   state_interface_types_);
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n",
                    e.what());
            return controller_interface::return_type::ERROR;
        }
        return controller_interface::return_type::OK;
    }

    CallbackReturn DDT2PositionController::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_names_ = node_->get_parameter("joints").as_string_array();
        if (joint_names_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
            return CallbackReturn::ERROR;
        }
        std::string bus_name = node_->get_parameter("bus_name").as_string();
        while (!socket_can_.open(bus_name, 95) &&
               rclcpp::ok()) {
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "!!!!!!!!!!!!!!!!");
        }
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "!!!!!!!!!!!!!!!!");
        // Specialized, child controllers set interfaces before calling configure
        // function.
        if (command_interface_types_.empty() && state_interface_types_.empty()) {
            command_interface_types_ =
                    node_->get_parameter("command_interfaces").as_string_array();
            state_interface_types_ =
                    node_->get_parameter("state_interfaces").as_string_array();
        }

        if (command_interface_types_.empty() || state_interface_types_.empty()) {
            RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "'command_interface' of 'state_interface' parameter was empty");
            return CallbackReturn::ERROR;
        }
        joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
                "~/commands", rclcpp::SystemDefaultsQoS(),
                [this](const CmdType::SharedPtr msg) {
                    rt_command_ptr_.writeFromNonRT(msg);
                });
        velocity_limits_subscriber_ = get_node()->create_subscription<CmdType>(
                "~/velocity_limits", rclcpp::SystemDefaultsQoS(),
                [this](const CmdType::SharedPtr msg) {
                    setLimitVelocity(msg);
                });

        velocity_limits_position_subscriber_ = get_node()->create_subscription<CmdType>(
                "~/velocity_limits_position", rclcpp::SystemDefaultsQoS(),
                [this](const CmdType::SharedPtr msg) {
                    double position_msg=msg->data[1];
                    setLimitVelocity_position(msg);
                    msg->data.resize(1);
                    msg->data[0]=position_msg;
                    rt_command_ptr_.writeFromNonRT(msg);
                });

        RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    DDT2PositionController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        //  Only use one command type so use INDIVIDUAL
        command_interfaces_config.type =
                controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto &joint: joint_names_) {
            //      TODO now the controller is only suppose effort control
            command_interfaces_config.names.push_back(joint + "/" +
                                                      command_interface_types_[0]);
        }
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration
    DDT2PositionController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type =
                controller_interface::interface_configuration_type::INDIVIDUAL;
        state_interfaces_config.names.reserve(joint_names_.size() *
                                              state_interface_types_.size());
        for (const auto &joint: joint_names_) {
            for (const auto &interface_type: state_interface_types_)
                state_interfaces_config.names.push_back(joint + "/" + interface_type);
        }
        return state_interfaces_config;
    }

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
    template<typename T>
    bool get_ordered_interfaces(
            std::vector<T> &unordered_interfaces,
            const std::vector<std::string> &joint_names,
            const std::string &interface_type,
            std::vector<std::reference_wrapper<T>> &ordered_interfaces) {
        for (const auto &joint_name: joint_names) {
            for (auto &interface: unordered_interfaces) {
                if ((interface.get_name() == joint_name) &&
                    (interface.get_interface_name() == interface_type)) {
                    ordered_interfaces.emplace_back(std::ref(interface));
                }
            }
        }

        return joint_names.size() == ordered_interfaces.size() ||
               joint_names.size() * 3 == ordered_interfaces.size();
    }

    CallbackReturn DDT2PositionController::on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        //  check if we have all resources defined in the "points" parameter
        //  also verify that we *only* have the resources defined in the "points"
        //  parameter
        std::vector<std::reference_wrapper<LoanedCommandInterface>>
                command_ordered_interfaces;
        std::vector<std::reference_wrapper<LoanedStateInterface>>
                state_ordered_interfaces;
        for (const auto &interface: command_interface_types_) {
            if (!get_ordered_interfaces(command_interfaces_, joint_names_, interface,
                                        command_ordered_interfaces) ||
                command_interfaces_.size() != command_ordered_interfaces.size()) {
                RCLCPP_ERROR(
                        node_->get_logger(), "Expected %zu '%s' command interfaces, got %zu.",
                        joint_names_.size(), interface.c_str(), command_interfaces_.size());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                CallbackReturn::ERROR;
            }
        }

        // reset command buffer if a command came through callback when controller was
        // inactive
        rt_command_ptr_ =
                realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DDT2PositionController::on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        // reset command buffer
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
        return CallbackReturn::SUCCESS;
    }

// Except for update() and config get, everything else is almost the same
    controller_interface::return_type DDT2PositionController::update() {
        if (last_time_.seconds() == 0 && last_time_.nanoseconds() == 0)
            last_time_ = rclcpp::Clock().now();
        auto joint_commands = rt_command_ptr_.readFromRT();
        init_positions_.resize(joint_names_.size());
        // no command received yet
        if (!joint_commands || !(*joint_commands)) {
//            if (!is_init_pos_) {
//                for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
//                    for (const auto &state_interface: state_interfaces_) {
//                        if (state_interface.get_full_name().find("position") !=
//                            std::string::npos &&
//                            state_interface.get_full_name().find(
//                                    command_interfaces_[index].get_name()) != std::string::npos)
//                            init_positions_[index] = state_interface.get_value();
//                    }
//                    command_interfaces_[index].set_value(init_positions_[index]);
//                    RCLCPP_INFO_STREAM(rclcpp::get_logger("!!!!!!!!!!!!!"), init_positions_[index]);
//                }
//                return controller_interface::return_type::OK;
//            }
            for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
                command_interfaces_[index].set_value(0.);
            }
            return controller_interface::return_type::OK;
        }
        if ((*joint_commands)->data.size() != command_interfaces_.size()) {
            RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(), *node_->get_clock(), 1000,
                    "command size (%zu) does not match number of interfaces (%zu)",
                    (*joint_commands)->data.size(), command_interfaces_.size());
            return controller_interface::return_type::ERROR;
        }
        for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
            command_interfaces_[index].set_value((*joint_commands)->data[index]);
        }
        last_time_ = rclcpp::Clock().now();
        return controller_interface::return_type::OK;
    }

    void DDT2PositionController::setLimitVelocity(const CmdType::SharedPtr msg) {
        can_frame limit_velocity_{};
        limit_velocity_.data[0] = 0x01;
        limit_velocity_.data[1] = 0x4D;

        double velocity_limit = msg->data[0];
        int velocity_limit_int = static_cast<int>(velocity_limit);
        unsigned char hex_value = static_cast<unsigned char>(velocity_limit_int & 0xFF);

        limit_velocity_.data[2] = hex_value;
        limit_velocity_.data[3] = 0x00;
        limit_velocity_.data[4] = 0x00;
        limit_velocity_.data[5] = 0x00;
        limit_velocity_.data[6] = 0xFF;
        limit_velocity_.data[7] = 0xFF;
        limit_velocity_.can_id = 0x036;
        limit_velocity_.can_dlc = 8;
        socket_can_.write(&limit_velocity_);
    }
    void DDT2PositionController::setLimitVelocity_position(const CmdType::SharedPtr msg) {
        can_frame limit_velocity_{};
        double velocity_limit = msg->data[0];
        const CmdType::SharedPtr position_msg;
        int velocity_limit_int = static_cast<int>(velocity_limit);
        unsigned char hex_value = static_cast<unsigned char>(velocity_limit_int & 0xFF);
        limit_velocity_.data[0] = 0x01;
        limit_velocity_.data[1] = 0x4D;
        limit_velocity_.data[2] = hex_value;
        limit_velocity_.data[3] = 0x00;
        limit_velocity_.data[4] = 0x00;
        limit_velocity_.data[5] = 0x00;
        limit_velocity_.data[6] = 0xFF;
        limit_velocity_.data[7] = 0xFF;
        limit_velocity_.can_id = 0x036;
        limit_velocity_.can_dlc = 8;
        socket_can_.write(&limit_velocity_);
        std::cout << "finish set limit" << msg->data[0]<<std::endl;
    }
} // namespace ddt2_position_controller



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ddt2_position_controller::DDT2PositionController,
                       controller_interface::ControllerInterface)

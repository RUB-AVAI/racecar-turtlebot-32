// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE__BUILDER_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vesc_msgs/msg/detail/vesc_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vesc_msgs
{

namespace msg
{

namespace builder
{

class Init_VescState_fault_code
{
public:
  explicit Init_VescState_fault_code(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  ::vesc_msgs::msg::VescState fault_code(::vesc_msgs::msg::VescState::_fault_code_type arg)
  {
    msg_.fault_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_distance_traveled
{
public:
  explicit Init_VescState_distance_traveled(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_fault_code distance_traveled(::vesc_msgs::msg::VescState::_distance_traveled_type arg)
  {
    msg_.distance_traveled = std::move(arg);
    return Init_VescState_fault_code(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_displacement
{
public:
  explicit Init_VescState_displacement(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_distance_traveled displacement(::vesc_msgs::msg::VescState::_displacement_type arg)
  {
    msg_.displacement = std::move(arg);
    return Init_VescState_distance_traveled(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_energy_regen
{
public:
  explicit Init_VescState_energy_regen(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_displacement energy_regen(::vesc_msgs::msg::VescState::_energy_regen_type arg)
  {
    msg_.energy_regen = std::move(arg);
    return Init_VescState_displacement(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_energy_drawn
{
public:
  explicit Init_VescState_energy_drawn(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_energy_regen energy_drawn(::vesc_msgs::msg::VescState::_energy_drawn_type arg)
  {
    msg_.energy_drawn = std::move(arg);
    return Init_VescState_energy_regen(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_charge_regen
{
public:
  explicit Init_VescState_charge_regen(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_energy_drawn charge_regen(::vesc_msgs::msg::VescState::_charge_regen_type arg)
  {
    msg_.charge_regen = std::move(arg);
    return Init_VescState_energy_drawn(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_charge_drawn
{
public:
  explicit Init_VescState_charge_drawn(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_charge_regen charge_drawn(::vesc_msgs::msg::VescState::_charge_drawn_type arg)
  {
    msg_.charge_drawn = std::move(arg);
    return Init_VescState_charge_regen(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_duty_cycle
{
public:
  explicit Init_VescState_duty_cycle(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_charge_drawn duty_cycle(::vesc_msgs::msg::VescState::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return Init_VescState_charge_drawn(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_speed
{
public:
  explicit Init_VescState_speed(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_duty_cycle speed(::vesc_msgs::msg::VescState::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_VescState_duty_cycle(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_current_input
{
public:
  explicit Init_VescState_current_input(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_speed current_input(::vesc_msgs::msg::VescState::_current_input_type arg)
  {
    msg_.current_input = std::move(arg);
    return Init_VescState_speed(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_current_motor
{
public:
  explicit Init_VescState_current_motor(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_current_input current_motor(::vesc_msgs::msg::VescState::_current_motor_type arg)
  {
    msg_.current_motor = std::move(arg);
    return Init_VescState_current_input(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_temperature_pcb
{
public:
  explicit Init_VescState_temperature_pcb(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_current_motor temperature_pcb(::vesc_msgs::msg::VescState::_temperature_pcb_type arg)
  {
    msg_.temperature_pcb = std::move(arg);
    return Init_VescState_current_motor(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_voltage_input
{
public:
  Init_VescState_voltage_input()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VescState_temperature_pcb voltage_input(::vesc_msgs::msg::VescState::_voltage_input_type arg)
  {
    msg_.voltage_input = std::move(arg);
    return Init_VescState_temperature_pcb(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vesc_msgs::msg::VescState>()
{
  return vesc_msgs::msg::builder::Init_VescState_voltage_input();
}

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE__BUILDER_HPP_

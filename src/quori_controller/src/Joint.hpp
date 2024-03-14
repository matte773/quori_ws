#pragma once

#include <ostream>
#include <memory>
#include <cstdint>
#include <string>

namespace quori_controller
{
  struct Joint
  {
    Joint() = default;
    explicit Joint(const std::string &name);

    using ConstPtr = std::shared_ptr<const Joint>;
    using Ptr = std::shared_ptr<Joint>;

    enum class Mode : std::uint8_t
    {
      Position,
      Velocity
    };

    std::string name;

    double command = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;

    Mode mode = Mode::Position;
  };
}

std::ostream &operator<<(std::ostream &o, const quori_controller::Joint &joint)
{
  o << "Joint [Name: " << joint.name
    << ", Command: " << joint.command
    << ", Position: " << joint.position
    << ", Velocity: " << joint.velocity
    << ", Effort: " << joint.effort
    << ", Mode: " << static_cast<std::underlying_type<quori_controller::Joint::Mode>::type>(joint.mode)
    << "]";
  return o;
}

std::ostream &operator<<(std::ostream &o, const quori_controller::Joint::Ptr &joint)
{
  if (joint) o << *joint;
  else o << "Joint [null]";
  return o;
}

std::ostream &operator<<(std::ostream &o, const quori_controller::Joint::ConstPtr &joint)
{
  if (joint) o << *joint;
  else o << "Joint [null]";
  return o;
}

/* Copyright 2016 Barrett Technology support@barrett.com
 *
 * This file is part of proficio_toolbox.
 *
 * This version of proficio_toolbox is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of proficio_toolbox is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of proficio_toolbox.  If not, see
 * http://www.gnu.org/licenses/.
 */

/** @file haptic_floor.h
 *
 *  Creates a haptic floor on which the user can rest their arm.
 *  
 *  I/O:
 *    positionInput     Cartesian tool position
 *    velocityInput     Cartesian tool velocity
 *    output            Cartesian tool force
 * 
 *  Parameters:
 *    @param sys_name   Name of the system
 */
// @TODO(ab): Remove this and use the standard safety walls

#ifndef PROFICIO_SYSTEMS_UTILITIES_HAPTIC_FLOOR_H_
#define PROFICIO_SYSTEMS_UTILITIES_HAPTIC_FLOOR_H_

#include <string>

#include <barrett/systems.h>                    // NOLINT(build/include_order)
#include <barrett/systems/abstract/system.h>    // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)

namespace proficio {
namespace systems {
class HapticFloor : public barrett::systems::System {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  Input<cp_type> positionInput;
  Input<cv_type> velocityInput;
  Output<cf_type> output;
  explicit HapticFloor(const std::string& sys_name = "HapticFloor") :
    System(sys_name), positionInput(this), velocityInput(this),
    output(this, &outputValue), kp_(0), kd_(0.5), tool_radius_(0.0) {
      object_center_.setZero();
      force_sum_.setZero();
    }

  void setCenter(const cp_type& center) { object_center_ = center; }
  void setToolRadius(const double& radius) { tool_radius_ = radius; }
  void setKp(const double& gain) { kp_ = gain; }
  void setKd(const double& gain) { kd_ = gain; }

 protected:
  Output<cf_type>::Value* outputValue;
  cp_type object_center_;
  double kp_, kd_, tool_radius_;
  cf_type force_sum_;  // must be outside operate() for output to work
  void operate() {
    cp_type position = positionInput.getValue();
    cv_type velocity = velocityInput.getValue();
    force_sum_.setZero();
    if (position[2] <= (object_center_[2] + tool_radius_)) {
      force_sum_[2] += kp_ * ((object_center_[2] + tool_radius_) - position[2]);
      force_sum_[2] -= kd_ * (velocity[2]);
    }
    outputValue->setData(&force_sum_);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(HapticFloor);
};
}  // namespace systems
}  // namespace proficio

#endif  // PROFICIO_SYSTEMS_UTILITIES_HAPTIC_FLOOR_H_

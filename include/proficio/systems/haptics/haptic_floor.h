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
 *  Defines the haptic floor.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#ifndef PROFICIO_SYSTEMS_HAPTICS_HAPTIC_FLOOR_H_
#define PROFICIO_SYSTEMS_HAPTICS_HAPTIC_FLOOR_H_

#include <proficio/systems/haptics/abstract/haptic_object.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// This creates a rigid floor defined by its center and its normal vector(axis)
// This can be configured as a magnetic floor if required
// Example:
//   HapticFloor* hf = new HapticFloor(center, axis);
//   hf->SetKp(kp);
//   hf->SetAxis(axis);
//   hf->SetMagField(mag_field); // typically mag_field = 0.02
//   forces = hf->CalcForces(tool_pos, tool_vel);
class HapticFloor: public HapticObject {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  HapticFloor(const cp_type& ctr, const cp_type& axs,
              const double& magfield = 0.0) :
              HapticObject(ctr, magfield), axis_(axs) {}
  // Resets the axis
  void SetAxis(const cp_type& ax) {
    axis_ = ax;
  }
  // Returns the axis
  cp_type GetAxis() const {
    return axis_;
  }
  // Checks if the tool is in contact with the floor
  bool InContact(const cp_type& tool);
  // Rotates the plane along X
  void RotX(const double&);
  // Rotates the plane along Y
  void RotY(const double&);
  // Rotates the plane along Z
  void RotZ(const double&);
  // Calculates the forces acting on the plane
  cf_type CalcForces(const cp_type& tool_center, const cv_type& wam_vel);
  ~HapticFloor() {}

 protected:
  cp_type axis_;
};

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_HAPTICS_HAPTIC_FLOOR_H_


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

/** @file shrinking_sphere.h
 *
 *  Defines class for the shrinking sphere assistive force.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#ifndef PROFICIO_SYSTEMS_HAPTICS_SHRINKING_SPHERE_H_
#define PROFICIO_SYSTEMS_HAPTICS_SHRINKING_SPHERE_H_

#include <proficio/systems/haptics/abstract/haptic_object.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// This creates a sphere whose radius keeps reducing with forces acting inwards
// thereby pushing the tool(if inside the sphere) towards its center
// Example:
//   ShrinkingSphere* ss = new ShrinkingSphere(center, rate);
//   ss->SetKp(kp);
//   ss->SetRadius(rad);
//   ss->SetSpeed(speed); // unit is in mts every 0.002(500Hz) secs
//   ss->SetDelay(delay); // unit is in mts every 0.002(500Hz) secs
//   forces = ca->CalcForces(tool_pos, tool_vel);
//   once the target is reached
//   ss->SetCenter(new_center);
class ShrinkingSphere : public HapticObject {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  ShrinkingSphere(const cp_type& ctr, const double& rad = 0.0,
                  const double& toolr = 0.05, const double& stiff = 500,
                  const double& damp = 0.0, const double& mf = 0.0)
                  : HapticObject(ctr, mf, toolr, stiff, damp), radius_(rad) {}
  // Sets the speed at which the shrinking sphere collapses
  void SetSpeed(const double& speed) {
    speed_ = speed;
  }
  // Sets the offset before impacting the shrinking sphere collapses
  void SetDelay(const double& delay) {
    delay_ = delay;
  }
  // Sets the offset before impacting the shrinking sphere collapses
  double GetRadius() const {
    return radius_;
  }
  // Checks if the shrinking sphere is in contact to the tool
  bool InContact(const cp_type& tool) const {
    if (radius_ - (tool - center_).norm() <= tool_radius_)
      return true;
    return false;
  }
  // Sets the magnetic object's center
  void SetCenter(const cp_type&);
  // Calculates the net force from the shrinking sphere
  cf_type CalcForces(const cp_type&, const cv_type&);

 private:
  // Speed at which the sphere converges and the time delay before it
  // impacts the tool
  double speed_, delay_;
  // Radius of the shrinking sphere
  double radius_;
  // Indicates if the shrinking sphere's radius needs to be updated
  bool update_radius_;
};

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_HAPTICS_SHRINKING_SPHERE_H_

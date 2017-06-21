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

/** @file haptic_sphere.h
 *
 *  Defines the haptic sphere.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#ifndef PROFICIO_SYSTEMS_HAPTICS_HAPTIC_SPHERE_H_
#define PROFICIO_SYSTEMS_HAPTICS_HAPTIC_SPHERE_H_

#include <proficio/systems/haptics/abstract/haptic_object.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// This creates a rigid sphere at the specified position with desired radius
// This can be configured as a magnetic sphere if required
// Example:
//   HapticSphere* hs = new HapticSphere(center, radius);
//   hs->SetKp(kp);
//   hs->SetRadius(rad);
//   hs->SetMagField(mag_field); // typically mag_field = 0.02
//   forces = hs->CalcForces(tool_pos, tool_vel);
class HapticSphere: public HapticObject {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  HapticSphere(const cp_type& ctr, const double& rad = 0.0,
               const double& mf= 0.0) : HapticObject(ctr, mf),
               radius_(rad){}
  // Sets the radius of the sphere
  void SetRadius(const double& r) { radius_ = r; }
  // Checks if the tool is touching the haptic sphere
  bool InContact(const cp_type& tool) const {
    if ((tool - center_).norm() < tool_radius_ + radius_)
      return true;
    return false;
  }
  // Calculates the forces acting on the plane
  cf_type CalcForces(const cp_type&, const cv_type&);
  ~HapticSphere() {}

 protected:
  // Radius of the haptic sphere
  double radius_;

};
}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio
#endif // PROFICIO_SYSTEMS_HAPTICS_HAPTIC_SPHERE_H_

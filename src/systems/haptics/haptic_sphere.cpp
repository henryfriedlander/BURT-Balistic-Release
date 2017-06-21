/* Copyright 2016 Barrett Technology support@barrett.com
 *
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

/** @file haptic_sphere.cpp
 *
 *  Defines the haptic sphere.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#include <proficio/systems/haptics/haptic_sphere.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// Calculates the distance between the tool's center and the sphere's center.
// It then checks if that distance is less than the total of the radius of
// the tool, the sphere and the magnetic field, in which case it means that
// the tool is inside the magnetic field. If so, it applies a force to pull
// the tool towards the surface of the sphere
HapticSphere::cf_type HapticSphere::CalcForces(const cp_type& tool_pos,
                                               const cv_type& wam_vel) {
  // Find the location of the tool's center relative to the sphere
  cp_type error = tool_pos - center_;
  double mag = error.norm();  // Distance between the tool and the sphere
  // The distance at which the tool resides with respect to the nearest
  // point(to the tool) on the sphere's surface
  double depth = mag - radius_;
  // Unit vector from the sphere's center towards the tool's center
  cp_type direction = error / mag;
  cf_type forces(0.0);
  // If the tool's surface is within the magnetic field or if it is inside
  // the surface of the sphere
  if (depth <= mag_field_ + tool_radius_ && depth > 0) {
    // The position along the direction of the tool on the surface of the ball
    cp_type proxy_center = center_ + ((radius_ + tool_radius_) * direction);
    forces += kp_ * (proxy_center - tool_pos);  // Stiffness
    forces -= kd_ * wam_vel;  // Damping portion
  }
  return forces;
}

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio

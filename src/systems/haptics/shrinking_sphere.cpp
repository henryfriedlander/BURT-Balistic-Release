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

/** @file shrinking_sphere.cpp
 *
 *  Defines class for the shrinking sphere assistive force.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#include <proficio/systems/haptics/shrinking_sphere.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// The center of the shrinking sphere is updated and this
// change is recorded to update the radius next time the
// forces are calculated
void ShrinkingSphere::SetCenter(const cp_type& ctr) {
  center_ = ctr;
  update_radius_ = true;
}

// If the center is updated then the radius is modified as the distance
// between the target and tool's centers plus the delay desired. The radius
// is then reduced at rate specified by the user(m every 0.002 secs(500Hz)).
// The distance between the tool and the target vs radius of the shrinking
// sphere is compared to find if the shrinking sphere is touching the outer
// outer surface of the tool. If so, forces are applied inwards to push the
// tool towards the shrinking sphere's center(also the target).
ShrinkingSphere::cf_type ShrinkingSphere::CalcForces(const cp_type& tool_pos,
                                                     const cv_type& wam_vel) {
  cp_type error = (center_ - tool_pos);
  if (update_radius_) {  // This indicates that the target's position is changed
    radius_ = error.norm() + delay_;
    update_radius_ = false;
  }
  radius_ = (radius_ > 0) ? (radius_ - speed_) : 0;
  double mag = error.norm();
  cf_type forces(0.0);
  if ((radius_ - mag) < tool_radius_ && (radius_ - mag) > 0) {
    // Apply force 0 when just touching or inside sphereRad
    // Apply increasing force when between touching and proxy center
    // Apply large force when proxy center is at sphereRad
    // Apply force 0 when proxy center is beyond sphereRad
    cp_type direction = error / mag;
    // On the inner surface of the shrinking sphere
    cp_type proxy_center = center_ - (radius_ - tool_radius_) * direction;
    forces += kp_ * (proxy_center - tool_pos);
    forces -= kd_ * wam_vel;  // damping
  }
  return forces;
}

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio

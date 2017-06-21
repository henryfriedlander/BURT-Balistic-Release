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

/** @file cylindrical_assist.cpp
 *
 *  Defines a cylinder shaped assistance for the user to reach the target.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#include <proficio/systems/haptics/cylindrical_assist.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

void CylindricalAssist::SetFacePts(const cp_type& p1, const cp_type& p2) {
  pt1_ = p1;
  pt2_ = p2;
}

// Calculates the point on the cylinder's inner surface that is closest
// to the tool's center and checks if that point is lying on the outer surface
// of the tool. If so, it applies a resistive force to keep the tool from
// penetrating into the cylinder
CylindricalAssist::cf_type CylindricalAssist::CalcForces (
    const cp_type& tool_pos,
    const cv_type& wam_vel) {
  // Unit vector of the normal from the tool's center to the cylinder
  cp_type unit_vec = (pt1_ - pt2_)/(pt1_ - pt2_).norm();
  // The point on the cylinder along the normal
  cp_type pt = pt2_ + dot((tool_pos - pt2_), unit_vec)*(unit_vec);
  cp_type vec = (tool_pos - pt);
  double dist = vec.norm();  // Distance between tool's center and the pt above
  cp_type proxy_center;
  cf_type forces(0.0);
  // If tool's outer surface touches the surface of the cylinder from inside
  if (dist > radius_ - tool_radius_) {
    proxy_center = ((radius_ - tool_radius_) * (vec/dist)) + pt;
    forces += kp_ * (proxy_center - tool_pos);
  }
  return forces;
}

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio

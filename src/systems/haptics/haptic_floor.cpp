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

/** @file haptic_floor.cpp
 *
 *  Defines the haptic floor.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#include <proficio/systems/haptics/haptic_floor.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// The normal vector(axis) and the center point of the plane
// are transformed with respect to x by the specified angle
void HapticFloor::RotX(const double &ang) {
  // Transform the normal vector with the rotation matrix
  axis_[1] = axis_[1] * cos(ang) - axis_[2] * sin(ang);
  axis_[2] = axis_[1] * sin(ang) + axis_[2] * cos(ang);
}

// The normal vector(axis) and the center point of the plane
// are transformed with respect to y by the specified angle
void HapticFloor::RotY(const double &ang) {
  // Transform the normal vector with the rotation matrix
  axis_[0] = axis_[0] * cos(ang) + axis_[2] * sin(ang);
  axis_[2] = - axis_[0] * sin(ang) + axis_[2] * cos(ang);
}

// The normal vector(axis) and the center point of the plane
// are transformed with respect to z by the specified angle
void HapticFloor::RotZ(const double &ang) {
  // Transform the normal vector with the rotation matrix
  axis_[0] = axis_[0] * cos(ang) - axis_[1] * sin(ang);
  axis_[1] = axis_[0] * sin(ang) + axis_[1] * cos(ang);
}

// Calculates the distance between the tool and the closest point on the plane.
// Then checks if the distance is zero to record that it is in contact with
// the plane.
bool HapticFloor::InContact(const cp_type& tool_pos) {
  // unit vector of normal to the plane
  cp_type plane_norm_unit = axis_/axis_.norm();
  // The point of intersection of the vector from the tool's center parallel
  // to the axis of the plane is found
  cp_type vec = center_ - tool_pos;
  double t = dot(plane_norm_unit, vec);
  cp_type intersect_pt;
  intersect_pt = tool_pos + (t * plane_norm_unit);
  if (-t <= tool_radius_)
    return true;
  return false;
}

// Calculates the point on the plane that is along the vector parallel to the
// plane's normal and passing through the tool's center. Then computes
// the distance between that point and the tool's center to check if the tool
// is within the plane's magnetic field. If so, it pulls the tool to keep it
// exactly on top of the plane.
HapticFloor::cf_type HapticFloor::CalcForces(const cp_type& tool_pos,
                                             const cv_type& wam_vel) {
  // unit vector of normal to the plane
  cp_type plane_norm_unit = axis_/axis_.norm();
  // The point of intersection of the vector from the tool's center parallel
  // to the axis of the plane is found
  cp_type vec = center_ - tool_pos;
  double t = dot(plane_norm_unit, vec);
  cp_type intersect_pt;
  intersect_pt = tool_pos + (t * plane_norm_unit);
  cf_type forces(0.0);
  // If the tool is touching/pushing against the plane or if the tool
  // is beneath the plane in which case the direction of the
  // vector will be opposite to that of the plane's unit normal
  if (-t <= tool_radius_ + mag_field_) {
    cp_type proxy_center = intersect_pt +
                           ((tool_radius_ + mag_field_) * plane_norm_unit);
    forces += kp_ * (proxy_center - tool_pos);  // Magnetic Portion
    forces -= kd_ * wam_vel;  // Damping portion
  }
  return forces;
}

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio

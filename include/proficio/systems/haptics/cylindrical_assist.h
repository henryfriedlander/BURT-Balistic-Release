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

/** @file cylindrical_assist.h
 *
 *  Defines class for the cylindrical assistive force.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#ifndef PROFICIO_SYSTEMS_HAPTICS_CYLINDRICAL_ASSIST_H_
#define PROFICIO_SYSTEMS_HAPTICS_CYLINDRICAL_ASSIST_H_

#include <proficio/systems/haptics/abstract/haptic_object.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// This creates a rigid cylinder with the tool on end face and the target at
// the other end face, thereby making the tool only to move within the cylinder
// towards the target
// Example:
//   CylindricalAssist* ca = new CylindricalAssist(center, face_pt1, face_pt2);
//   ca->SetKp(kp);
//   ca->SetRadius(rad);
//   forces = ca->CalcForces(tool_pos, tool_vel);
class CylindricalAssist : public HapticObject {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  CylindricalAssist(const cp_type& ctr, const double& p1, const double& p2,
                    const double& rad = 100000.0, const double& toolr = 0.05,
                    const double& stiff = 500, const double& damp = 0.0,
                    const double& mf = 0.0) : HapticObject(ctr, mf, toolr,
                    stiff, damp), pt1_(p1), pt2_(p2), radius_(0.0) {
                    SetRadius(rad); }

  // Sets the radius associated with the cylinder
  void SetRadius(const double& rad) {
    if(rad < 2 * tool_radius_)
      radius_ = 2 * tool_radius_;
    else
      radius_ = rad;
  }
  // Checks if tool is in contact with the cylinder
  bool InContact(const cp_type& tool) const {
  if ((tool - center_).norm() < tool_radius_ + radius_)
    return true;
  return false;
  }
  // Updates the axis of the cylinder
  void SetFacePts(const cp_type&, const cp_type&);
  // Calculates the net force from the cylindrical assist
  cf_type CalcForces(const cp_type&, const cv_type&);

 private:
  // The center points at either end of the cylinder
  cp_type pt1_, pt2_;
  // The radius of the cylinder
  double radius_;
};

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_HAPTICS_ASSISTIVE_FORCES_H_

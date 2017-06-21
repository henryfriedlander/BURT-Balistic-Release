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

/** @file haptic_object.h
 *
 *  Defines a template class for the haptic objects.
 *
 *  NOTICE: This file does not follow the barrett coding standards and is not
 *  approved for use in clinical software.
 *
 *  THIS FILE HAS NOT BEEN TESTED.
 */

#ifndef PROFICIO_SYSTEMS_HAPTICS_ABSTRACT_HAPTIC_OBJECT_H_
#define PROFICIO_SYSTEMS_HAPTICS_ABSTRACT_HAPTIC_OBJECT_H_

#include <Eigen/Core>
#include <cmath>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>

namespace proficio {
namespace systems {
namespace haptictoolbox {

// Template class that defines the magnetic objects
// This class can be inherited by all other classes which needs to define a
// haptic object of various geometries, size and forces
class HapticObject {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  HapticObject(const cp_type& ctr, const double& magfield = 0.0,
               const double& toolr = 0.05, const double& kp = 0.0,
               const double& kd = 0.0) : center_(ctr),
    mag_field_(magfield), kp_(kp), kd_(kd), tool_radius_(toolr) {}
  // Sets the haptic object's stiffness
  void SetKp(const double& gain) {
    kp_ = gain;
  }
  // Sets the haptic object's damping
  void SetKd(const double& gain) {
    kd_ = gain;
  }
  // Sets the tool's radius
  void SetToolRadius(const double& r) {
    tool_radius_ = r;
  }
  // Sets the haptic field size
  void SetMagField(const double& mf) {
    mag_field_ = mf;
  }
  // Sets the haptic object's center
  void SetCenter(const cp_type& c) {
    center_ = c;
  }
  // Returns the forces to be applied on the tool's center
  virtual cf_type CalcForces(const cp_type&, const cv_type&) = 0;
  virtual ~HapticObject() {}

 protected:
  cp_type center_;
  double mag_field_, kp_, kd_, tool_radius_;
  // Performs the dot product between two vectors of cp_type
  // Dot Product between two cp_types
  double dot(cp_type vec1 , cp_type vec2) {
    return ((vec1[0]*vec2[0])+(vec1[1]*vec2[1])+(vec1[2]*vec2[2]));
  }
};

}  // namespace haptictoolbox
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_HAPTICS_ABSTRACT_HAPTIC_OBJECT_H_


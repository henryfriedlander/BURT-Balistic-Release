/* Copyright 2016 Barrett Technology <support@barrett.com>
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
 * <http://www.gnu.org/licenses/>.
 */

/** @file user_gravity_compensation.h
 *  
 *  Calculate and apply a torque to provide assitive gravity compensation
 *  for the weight of a user's arm.
 *
 *  Torque is calculated by reading in calibration data from a file, 
 *  interpolating for the robot's current position and torque, and then
 *  multiplying by a gain to scale to the desired amount of assistance for the 
 *  specific user.
 *
 *  I/O      | Description
 *  ---------|----------------------
 *    input  | joint positions
 *    output | joint torques
 *
 *  Parameters:
 *    @param config_file_name  name of file with calibration data 
 *    @param sys_name          name of the system
 */

#ifndef PROFICIO_SYSTEMS_UTILITIES_USER_GRAVITY_COMPENSATION_H_
#define PROFICIO_SYSTEMS_UTILITIES_USER_GRAVITY_COMPENSATION_H_

#include <iostream>
#include <string>

#include <boost/tuple/tuple.hpp>                // NOLINT(build/include_order)

#include <barrett/systems.h>                    // NOLINT(build/include_order)
#include <barrett/systems/abstract/system.h>    // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)
#include <proficio/tools/natural_neighbor_interpolate.h>  // NOLINT(build/include_order)

namespace proficio {
namespace systems {

template <size_t DOF>
class UserGravityCompensation : public barrett::systems::SingleIO<
    typename barrett::units::JointPositions<DOF>::type,
    typename barrett::units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  /** Gets calibration data from config file.
   *
   *  The order of calibration points are fixed such that points 2 through 5
   *  are traveling clockwise around the edges of the convex hull and
   *  point 1 is on the interior. This simplifies the process of projecting 
   *  an external point onto the surface of the hull.
   *   * 0: start position (approx 0.25, 0.00, 1.57)
   *   * 1: q1 low, q2 low (arm up, out to side, towards user)
   *   * 2: q1 low, q2 high (arm up, in front of user)
   *   * 3: q1 high, q2 high (arm down, in front of user, away)
   *   * 4: q1 high, q2 low (arm down, out to side)
   *   * 5-14: whatever you want, or empty
   */
  explicit UserGravityCompensation(const std::string& config_file_name,
                                   const std::string& sys_name =
                                       "UserGravityCompensation")
      : barrett::systems::SingleIO<jp_type, jt_type>(sys_name),
        interpolate_(config_file_name) {
    torque_previous_ << 0, 0, 0;
    gain_ = 0;
  }

  virtual ~UserGravityCompensation() { this->mandatoryCleanUp(); }

  void decrementGain() {
    gain_ -= 0.2;
    if (gain_ < 0) gain_ = 0;
  }

  void incrementGain() {
    gain_ += 0.2;
    if (gain_ > 2) gain_ = 2;
  }

  double resetGain() {
    gain_ = 1.0;
    return gain_;
  }

  double getGain() { return gain_; }

  void setGainZero() { gain_ = 0; }

 protected:
  proficio::tools::NaturalNeighborInterpolate<DOF> interpolate_;
  jt_type torque_previous_;  ///< torque applied the previous control loop
  double gain_;  ///< scale amount of assistive gravity compensation

  /** Interpolate gravity compensation for current position 
    * and set a torque scaled by the gain */
  virtual void operate() {
    // get current robot position from input
    jp_type position = this->input.getValue();
    // default to applying the same torque as last time
    jt_type torque(torque_previous_);

    if (interpolate_.natural_neighbor_interpolation(position, &torque)) {
      // Multiply by gain for the current user.
      for (size_t i = 0; i < DOF; i++) {
        torque[i] = gain_ * torque[i];
      }
    }
    torque_previous_ = torque;  // Save torque output value.
    {
      BARRETT_SCOPED_LOCK(this->getEmMutex());
      this->outputValue->setData(&torque_previous_);
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(UserGravityCompensation);
};
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_UTILITIES_USER_GRAVITY_COMPENSATION_H_

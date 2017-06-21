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

/** @file user_gravity_compensation_linear.h
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

#ifndef PROFICIO_SYSTEMS_UTILITIES_USER_GRAVITY_COMPENSATION_LINEAR_H_
#define PROFICIO_SYSTEMS_UTILITIES_USER_GRAVITY_COMPENSATION_LINEAR_H_

#include <iostream>
#include <string>

#include <boost/tuple/tuple.hpp>                // NOLINT(build/include_order)

#include <barrett/systems.h>                    // NOLINT(build/include_order)
#include <barrett/systems/abstract/system.h>    // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)
#include <proficio/tools/linear_interpolate.h>  // NOLINT(build/include_order)

namespace proficio {
namespace systems {

template <size_t DOF>
class UserGravityCompensationLinear : public barrett::systems::SingleIO<
    typename barrett::units::JointPositions<DOF>::type,
    typename barrett::units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  /** Initializes and gets calibration data from config file. */
  explicit UserGravityCompensationLinear(const std::string& config_file_name,
                                         const std::string& sys_name =
                                             "UserGravityCompensationLinear")
      : barrett::systems::SingleIO<jp_type, jt_type>(sys_name) {
    interpolate_.init(config_file_name);
    torque_.setZero();
    gain_ = 0;
  }

  virtual ~UserGravityCompensationLinear() { this->mandatoryCleanUp(); }

  void decrementGain() {
    gain_ -= 0.2;
    if (gain_ < 0) gain_ = 0;
  }

  void incrementGain() {
    gain_ += 0.2;
    if (gain_ > 2) gain_ = 2;
  }

  void resetGain() { gain_ = 1.0; }

  double getGain() { return gain_; }

  void setGainZero() { gain_ = 0; }

 protected:
  proficio::tools::LinearInterpolate<DOF> interpolate_;
  jt_type torque_;  // must be outside operate to output correctly
  double gain_;  ///< scale amount of assistive gravity compensation

  /** Interpolate gravity compensation for current position 
    * and set a torque scaled by the gain */
  virtual void operate() {
    jp_type position = this->input.getValue();
    torque_ = gain_ * interpolate_.linear_interpolation(position);
    {
      BARRETT_SCOPED_LOCK(this->getEmMutex());
      this->outputValue->setData(&torque_);
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(UserGravityCompensationLinear);
};
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_UTILITIES_USER_GRAVITY_COMPENSATION_LINEAR_H_

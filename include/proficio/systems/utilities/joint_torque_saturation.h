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
 *
 */

/** @file joint_torque_saturation.h
 *
 *  Joint Torques Saturation system that limits the torques sent to the robot.
 *  
 *  I/O:
 *    input     Raw torque commands
 *    output    Scaled torque commands (same as input if under specified limits)
 * 
 *  Parameters:
 *    @param limits    Torque limits that specify at what torque to saturate
 *    @param sys_name   Name of the system
 */

#ifndef PROFICIO_SYSTEMS_UTILITIES_JOINT_TORQUE_SATURATION_H_
#define PROFICIO_SYSTEMS_UTILITIES_JOINT_TORQUE_SATURATION_H_

#include <string>

#include <barrett/systems.h>                    // NOLINT(build/include_order)
#include <barrett/systems/abstract/system.h>    // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)

namespace proficio {
namespace systems {

template<size_t DOF>
class JointTorqueSaturation
    : public barrett::systems::SingleIO<
        typename barrett::units::JointTorques<DOF>::type,
        typename barrett::units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  JointTorqueSaturation(const jt_type &limits,
                        const std::string &sys_name = "JointTorqueSaturation")
    : barrett::systems::SingleIO<jt_type, jt_type>(sys_name),
      torque_limits_(limits) {}

  virtual ~JointTorqueSaturation() {
    this->mandatoryCleanUp();
  }

  void setJointTorqueLimit(const jt_type &joint_torque_limits) {
    torque_limits_ = joint_torque_limits;
  }

 protected:
  jt_type torque_limits_;
  jt_type output_torques_;

  /** Scales down all torques if any joint's torque is over the limit */
  virtual void operate() {
    jt_type joint_torques = this->input.getValue();
    int index;  /// index of joint with most torque
    // finds ratio for joint whose torque is closest to the torque limit
    double min_ratio;  // minumum ratio of torque_limit/raw_joint_torque
    min_ratio = (torque_limits_.array() /
      (joint_torques.cwiseAbs()).array()).minCoeff(&index);
    if (min_ratio < 1.0) {  // if torque of any joint is over the limit
      output_torques_ = min_ratio * joint_torques;
    } else {
      output_torques_ = joint_torques;
    }
    this->outputValue->setData(&output_torques_);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(JointTorqueSaturation);
};
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_UTILITIES_JOINT_TORQUE_SATURATION_H_

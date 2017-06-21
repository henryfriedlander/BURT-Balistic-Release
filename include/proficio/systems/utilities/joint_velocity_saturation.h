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

/** @file joint_velocity_saturation.h
 *
 *  Joint velocity damping system that applies damping when joint velocity 
 *  gets above a certain threshold.
 * 
 *  I/O:
 *    input             Raw joint velocities
 *    output            Joint torques to apply damping if applicable
 *
 *  parameters:
 *    @param damping           Damping constants for the joints
 *    @param limits            Velocity at which damping starts
 *    @param mode              Choose damping type. 1: linear, 2: quadratic
 *    @param sys_name           Name of the system
 */

#ifndef PROFICIO_SYSTEMS_UTILITIES_JOINT_VELOCITY_SATURATION_H_
#define PROFICIO_SYSTEMS_UTILITIES_JOINT_VELOCITY_SATURATION_H_

#include <string>

#include <barrett/systems.h>                    // NOLINT(build/include_order)
#include <barrett/systems/abstract/system.h>    // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)

namespace proficio {
namespace systems {

template <size_t DOF>
class JointVelocitySaturation
    : public barrett::systems::SingleIO<
          typename barrett::units::JointVelocities<DOF>::type,
          typename barrett::units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  JointVelocitySaturation(const v_type damping, const jv_type limits,
                          const int mode = 1, const std::string &sys_name =
                                                  "JointVelocitySaturation")
      : barrett::systems::SingleIO<jv_type, jt_type>(sys_name),
        damping_constants_(damping),
        velocity_limits_(limits),
        multiplier_(1.0),
        torque_limit_(35.0),
        mode_(mode) {}

  virtual ~JointVelocitySaturation() { this->mandatoryCleanUp(); }

  void enable() { multiplier_ = 1.0; }

  void disable() { multiplier_ = 0.0; }

  void setDampingConstant(const v_type &val) { damping_constants_ = val; }

  void setJointVelocityLimit(const jv_type &val) { velocity_limits_ = val; }

 protected:
  v_type damping_constants_;
  jv_type velocity_limits_;
  jt_type output_torques_;  /// output torque of the system
  double multiplier_;    /// enable, disable, and scale damping
  const double torque_limit_;  /// limit for torques used in velocity damping
  const int mode_;  /// type of damping -- 1: linear, 2: quadratic


  /** Damps any joint velocities that exceed specified limit */
  virtual void operate() {
    jv_type velocity = this->input.getValue();
    jt_type calculated_torques;
    for (size_t i = 0; i < DOF; i++) {
      if (fabs(velocity[i]) > velocity_limits_[i]) {
        if (mode_ == 1) {  // use linear damping
          double error = velocity[i] -
                         velocity_limits_[i] * velocity[i] / fabs(velocity[i]);
          calculated_torques[i] = multiplier_ * -damping_constants_[i] * error;
        } else if (mode_ == 2) {  // use quadratic damping
          double error = velocity[i] -
                         velocity_limits_[i] * velocity[i] / fabs(velocity[i]);
          calculated_torques[i] = multiplier_ * -damping_constants_[i] * error*fabs(error);
        } else {  // output zero torque
          calculated_torques[i] = 0;
        }
        if (fabs(calculated_torques[i]) > torque_limit_) {
          calculated_torques[i] = torque_limit_ * calculated_torques[i] / 
                              fabs(calculated_torques[i]);
        }
      } else {
        calculated_torques[i] = 0;
      }
    }
    output_torques_ = calculated_torques;
    this->outputValue->setData(&output_torques_);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(JointVelocitySaturation);
};

}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_UTILITIES_JOINT_VELOCITY_SATURATION_H_

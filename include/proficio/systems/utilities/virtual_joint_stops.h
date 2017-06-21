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

/** @file virtual_joint_stops.h
 *  
 *  Virtual joint damper system that damps velocity when nearing mechanical 
 *  joint limits
 *
 *  I/O         | Description
 *  ----------- | ---------------------------------------------
 *  posInput    | current joint angle positions
 *  velInput    | current joint velocities
 *  output      | joint torques to apply damping if applicable
 * 
 *  Parameters:
 *    @param joint_mins         hardware minimum angle for the joints
 *    @param joint_maxes        hardware maximum angle for the joints
 *    @param damping_constants  damping constants of joints
 *    @param joint_offset       how far away from the joint the damper activates
 *    @param sys_name           name of the system
 */

#ifndef PROFICIO_SYSTEMS_UTILITIES_VIRTUAL_JOINT_STOPS_H_
#define PROFICIO_SYSTEMS_UTILITIES_VIRTUAL_JOINT_STOPS_H_

#include <string>

#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/units.h>

namespace proficio {
namespace systems {

template<size_t DOF>
class VirtualJointStops : public barrett::systems::System {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
  Input<jp_type> posInput;
  Input<jv_type> velInput;
  Output<jt_type> output;

  explicit VirtualJointStops(jp_type joint_mins, jp_type joint_maxes,
                            const v_type damping_constants,
                            const jp_type joint_offset = 0.4,
                            const std::string& sys_name = "VirtualJointStops")
      : barrett::systems::System(sys_name), posInput(this), velInput(this), 
          output(this, &outputValue), damping_constants_(damping_constants),
          damper_depth_(joint_offset), output_torques_(0.0) {
      joint_damper_min_ = joint_mins + joint_offset; // where to activate damper
      joint_damper_max_ = joint_maxes - joint_offset;
    }

  virtual ~VirtualJointStops() { this->mandatoryCleanUp(); }

protected:
  typename Output<jt_type>::Value *outputValue; 
  v_type damping_constants_; 
  jp_type joint_damper_min_;  ///< where the dampers activate
  jp_type joint_damper_max_;  ///< where the dampers activate
  jp_type damper_depth_;      ///< size of damping zone
  jt_type output_torques_;    ///< output torque of the system


  /** Get the position and velocity of the joints. Check each joint to see
   *  if it is within the damping range, and that the velocity is toward
   *  the mechanical joint stop. If so, damp the velocity by 
   *    -kp * velocity * depth_gain^2
   *  
   *  The quadratic damping makes the damping force lower when it is first
   *  applied to avoid any jerking motion, and then ramps up to prevent the
   *  user from hitting the mechanical joint stop with appreciable velocity,
   *  which would run the risk of damaging the robot.
   */
  virtual void operate() {
    jp_type curr_pos = this->posInput.getValue();   // current joint position
    jv_type curr_vel = this->velInput.getValue();  // current joint velocity

    for (size_t i = 0; i < DOF; i++) {  // loop through each joint
      if ((curr_pos[i] < joint_damper_min_[i]) &&  // within pos range
                 (curr_vel[i] < 0)) {  // vel toward joint stop
        double depth = joint_damper_min_[i] - curr_pos[i];
        double depth_gain = depth / damper_depth_[i];
        output_torques_[i] = -damping_constants_[i] * curr_vel[i] *
                             depth_gain * depth_gain;
      
      } else if ((curr_pos[i] > joint_damper_max_[i]) && (curr_vel[i] > 0)) {
        double depth = curr_pos[i] - joint_damper_max_[i];
        double depth_gain =  depth / damper_depth_[i];
        output_torques_[i] = -damping_constants_[i] * curr_vel[i] *
                             depth_gain * depth_gain;
      }
      else {
        output_torques_[i] = 0;
      }
    }
    this->outputValue->setData(&output_torques_);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(VirtualJointStops);
};
}  // namespace systems
}  // namespace proficio
#endif  // PROFICIO_SYSTEMS_UTILITIES_VIRTUAL_JOINT_STOPS_H_

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

/** @file state_toggle.cpp
 *
 *  Toggles the robot state between Activate (turned on and gravity
 *  compensated) and Sleep (E-Stop state: powered off, resistive braking
 *  active), use with the Proficio GUI.
 *
 *  This program accepts "Activate" or "Sleep" as an argument. Both options
 *  wake up the robot and create a virtual floor on which the user can rest
 *  their arm. The virtual floor starts at a very low position and rises with
 *  the user's arm until a pre-defined position close to the expected height
 *  of the patient's sternum. The Sleep option additionally powers off the
 *  robot when it is moved to its home position. With the "Sleep" option, the
 *  robot is considered to be in Sleep state when the program ends and the
 *  robot is powered off.
 *
 * This program is not intended to be used in clinical versions of Proficio.
 *
 *  Return code | Description
 *  -------------------------
 *   0          | No compatible robot was found
 *   1          | Success! In this case, success is either reaching the home
 *              | state or receiving an external kill signal, assumed to come
 *              | from the GUI.
 *   2          | Reached end of program (should never happen)
 *   3          | Detected fault or stop from pendant (ESTOP or IDLE state)
 */

// @TODO(ab): Update return codes to make more sense. Requires corresponding
// changes to proficio_gui.

#include <signal.h>  // signal, raise, sig_atomic_t
#include <string>

#include <barrett/math.h>                            // NOLINT(build/include_order)
#include <barrett/systems.h>                         // NOLINT(build/include_order)
#include <barrett/units.h>                           // NOLINT(build/include_order)
#include <barrett/detail/stl_utils.h>                // NOLINT(build/include_order)
#include <barrett/products/product_manager.h>        // NOLINT(build/include_order)
#include <barrett/products/safety_module.h>          // NOLINT(build/include_order)
#include <barrett/systems/wam.h>                     // NOLINT(build/include_order)

#define BARRETT_SMF_VALIDATE_ARGS
#define NO_CONTROL_PENDANT
#include <proficio/standard_proficio_main.h>         // NOLINT(build/include_order)
#include <proficio/systems/utilities.h>              // NOLINT(build/include_order)
#include <proficio/tools/configure_safety_module.h>  // NOLINT(build/include_order)

using barrett::systems::connect;
using barrett::btsleep;

namespace proficio {
std::string requested_state = "";
}  // namespace proficio

/** Checks that the caller of the program specified Activate or Sleep state. */
bool validate_args(int argc, char** argv) {
  if ((argc != 2) ||
      !(((std::string)argv[1] == "Activate") ||
        ((std::string)argv[1] == "Sleep"))) {
    printf("Usage: %s <Activate/Sleep> \n", argv[0]);
    return false;
  }
  proficio::requested_state = argv[1];
  return true;
}

namespace proficio {
bool exit_program = false;

/** When killed from outside (by GUI), this allows a graceful exit. */
void exit_program_callback(int signum) {
  exit_program = true;
}
}  // namespace proficio

template<size_t DOF>
int proficio_main(int argc, char** argv,
                  barrett::ProductManager& product_manager,  // NOLINT(runtime/references)
                  barrett::systems::Wam<DOF>& wam,           // NOLINT(runtime/references)
                  const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  wam.gravityCompensate(true);

  barrett::SafetyModule* safety_module = product_manager.getSafetyModule();
  safety_module->setVelocityLimit(1.5);
  safety_module->setTorqueLimit(4.0);

  // Set up joint velocity damping for movement in freespace. These gains are
  // tuned for P3 pucks.
  const v_type kDampingConstants(32.0, 22.0, 17.0);
  const jv_type kVelocityLimits(1.3);
  proficio::systems::JointVelocitySaturation<DOF> joint_velocity_saturation(
      kDampingConstants, kVelocityLimits, 2);

  const jv_type kJointVelocityFilterFrequency(20);
  barrett::systems::FirstOrderFilter<jv_type> joint_velocity_filter;
  joint_velocity_filter.setLowPass(kJointVelocityFilterFrequency);

  connect(wam.jvOutput, joint_velocity_filter.input);
  connect(joint_velocity_filter.output, joint_velocity_saturation.input);

  // Catch kill signals if possible for a graceful exit. This is intended for
  // use with the Proficio GUI, which currently calls this program between
  // games and kills this program before starting a new game.
  signal(SIGINT, proficio::exit_program_callback);
  signal(SIGTERM, proficio::exit_program_callback);
  signal(SIGKILL, proficio::exit_program_callback);

  const cv_type kToolVelocityFilterFrequency(20);
  const double kGainNoFloor = 100, kMaxFloorHeight = 0.15;

  // Check robot configuration. If joint 3 is negative, the robot is in left
  // configuration. If joint 3 is positive, it is in right configuration. This
  // check will become unnecessary when hardware support is available for
  // configuration detection.
  jp_type home;
  jp_type joint_positions = wam.getJointPositions();
  if (joint_positions[2] < 0) {
    home << -1.01, 0.96, -2.84;
  } else if (joint_positions[2] > 0) {
    home << -1.01, -0.96, 2.84;
  } else {
    printf("No compatible robot was found.");
    return 0;
  }

  // Check whether the robot endpoint is under the maximum floor height. If
  // yes, the floor height will start at the current tool position and rise
  // with the tool.
  cp_type center, tool_position;
  bool below_max_floor_height = false;
  tool_position = wam.getToolPosition();
  if (tool_position[2] <= kMaxFloorHeight) {
    center << 0, 0, tool_position[2];
    below_max_floor_height = true;
  } else {
    center << 0, 0, kMaxFloorHeight;
  }

  // Set up the virtual floor. The floor height starts at the lower of the max
  // floor height and the current tool height. It then rises with the tool
  // until it reaches the max floor height.
  // @TODO(ab): Use the standard safety walls instead of HapticFloor.
  proficio::systems::HapticFloor virtual_floor;
  virtual_floor.setCenter(center);
  virtual_floor.setKp(kGainNoFloor);
  connect(wam.toolPosition.output, virtual_floor.positionInput);

  barrett::systems::FirstOrderFilter<cv_type> tool_velocity_filter;
  tool_velocity_filter.setLowPass(kToolVelocityFilterFrequency);
  connect(wam.toolVelocity.output, tool_velocity_filter.input);
  connect(tool_velocity_filter.output, virtual_floor.velocityInput);

  barrett::systems::ToolForceToJointTorques<DOF> tf2jt;
  connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
  connect(virtual_floor.output, tf2jt.input);

  // Sum joint torques from virtual floor and joint velocity saturation.
  barrett::systems::Summer<jt_type> joint_torque_sum;
  connect(tf2jt.output, joint_torque_sum.getInput(0));
  connect(joint_velocity_saturation.output, joint_torque_sum.getInput(1));

  // Saturate joint torques for safety.
  jt_type joint_torque_limits(35.0);
  proficio::systems::JointTorqueSaturation<DOF>
      joint_torque_saturation(joint_torque_limits);
  connect(joint_torque_sum.output, joint_torque_saturation.input);

  safety_module->setMode(barrett::SafetyModule::ACTIVE);
  safety_module->waitForMode(barrett::SafetyModule::ACTIVE);
  connect(joint_torque_saturation.output, wam.input);
  wam.idle();

  // Update the virtual floor position if necessary. Check for quit signal,
  // reaching home, or robot stopped running; and follow the appropriate exit
  // sequence.
  while (true) {
    tool_position = wam.getToolPosition();
    joint_positions = wam.getJointPositions();

    // If the tool position was below the max floor height at the last
    // timestep, check whether the tool position has risen and increase it if
    // yes, to a maximum of the max floor height specified earlier.
    if (below_max_floor_height && (tool_position[2] > center[2])) {
      center[2] = tool_position[2];
      virtual_floor.setCenter(center);
      if (tool_position[2] > kMaxFloorHeight)
        below_max_floor_height = false;
    }

    // If a quit signal has been received (should come from Proficio GUI), idle
    // the robot and exit.
    if (proficio::exit_program) {
      wam.idle();
      product_manager.getPuck(1)->setProperty(
          product_manager.getPuck(1)->getBus(), 1, 8, 3);
      product_manager.getPuck(2)->setProperty(
          product_manager.getPuck(2)->getBus(), 2, 8, 3);
      product_manager.getPuck(3)->setProperty(
          product_manager.getPuck(3)->getBus(), 3, 8, 3);
      barrett::systems::disconnect(wam.input);
      return 1;
    }

    // If trying to Sleep, check whether robot is close enough to home position.
    const double kThreshold = 0.05;
    if (((proficio::requested_state == "Sleep") &&
         (fabs(joint_positions[0] - home[0]) <= kThreshold &&
          fabs(joint_positions[1] - home[1]) <= kThreshold &&
          fabs(joint_positions[2] - home[2]) <= kThreshold))) {
      // @TODO(hm): Send a CAN message asking the Pucks to power off
      safety_module->setMode(barrett::SafetyModule::ESTOP);
      wam.getLowLevelWam().getPuckGroup().setProperty(barrett::Puck::MODE,
          barrett::MotorPuck::MODE_IDLE);
      return 1;
    }

    // If the robot has stopped running due to fault or user stop, quit with
    // error code.
    if (safety_module->getMode() == barrett::SafetyModule::ESTOP ||
        safety_module->getMode() == barrett::SafetyModule::IDLE) {
      return 3;
    }
    btsleep(0.001);  // sleep time in seconds
  }

  return 2;
}

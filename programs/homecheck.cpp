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

/** @file homecheck.cpp
 *
 *  Automatically activates the proficio and checks for home position.
 *
 *  This program accepts "left" or "right" as an argument and checks whether
 *  the Proficio is in the specified configuration. It requires the Proficio to
 *  start in its extended position, and checks for this position by driving the
 *  robot into the expected position of the joint stops. If the configuration is
 *  correct, the program then moves the robot to the home position and sets that
 *  position as home. If the configuration is not correct, the program quits
 *  without setting a new home position.
 *
 *  Note that the home position is read from
 *     ~/.barrett/calibration_data/wam3/zerocal.conf
 *  so you must run leftConfig or rightConfig before running this program to
 *  ensure that the home position is set correctly.
 *
 *  This program is not intended to be used in clinical versions of Proficio.
 *
 *  Return code | Description
 *  -------------------------
 *   0          | <undefined>
 *   1          | Success!
 *   2          | Robot did not achieve ACTIVE state -OR- homecheck fail
 *   3          | Invalid configuration from command line input (validate_args
 *              | should fail before reaching here)
 *   4          | Unsupported puck type set in this program's source code
 *   5          | Invalid configuration type (should return 3 before reaching
 *              | here)
 *   6          | Fault detected during startup procedure
 *   7          | Robot movement detected when it should be stationary
 */

// @TODO(ab): Update return codes to make more sense. Requires corresponding
// changes to proficio_gui.
// @TODO(ab): Read home position from ~/.barrett/proficio_left or
// ~/.barrett_proficio_right, as appropriate.

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <string>

#include <barrett/exception.h>                       // NOLINT(build/include_order)
#include <barrett/math.h>                            // NOLINT(build/include_order)
#include <barrett/units.h>                           // NOLINT(build/include_order)
#include <barrett/systems.h>                         // NOLINT(build/include_order)
#include <barrett/detail/stl_utils.h>                // NOLINT(build/include_order)
#include <barrett/products/product_manager.h>        // NOLINT(build/include_order)
#include <barrett/products/puck.h>                   // NOLINT(build/include_order)
#include <barrett/products/detail/property_list.h>   // NOLINT(build/include_order)
#include <barrett/systems/wam.h>                     // NOLINT(build/include_order)

#define BARRETT_SMF_VALIDATE_ARGS
#define NO_CONTROL_PENDANT
#include <proficio/standard_proficio_main.h>         // NOLINT(build/include_order)
#include <proficio/systems/utilities.h>              // NOLINT(build/include_order)
#include <proficio/tools/configure_safety_module.h>  // NOLINT(build/include_order)

using barrett::systems::connect;
using barrett::btsleep;

namespace proficio {
// Since P2 and P3 gains are tuned differently, different torque limits are
// required.
enum PuckVersion {
  P2,
  P3
};
PuckVersion puck_type = P3;
}  // namespace proficio

/** Validates that the caller of the program specified whether to check for
 * left or right configuration.
 */
bool validate_args(int argc, char** argv) {
  if ((argc != 2) ||
      (!(((std::string)argv[1] == "left") ||
         ((std::string)argv[1] == "right")))) {
      printf("Usage: %s <left/right>\n", argv[0]);
      return false;
  }
  return true;
}

/** Drives the robot into the expected position of the joint stops with a
 * specified torque. If movement is less than a specified threshold, it
 * considers joint stops to be found. If all joint stops are found in the
 * expected locations, it moves the robot to the home position and sends
 * commands to the safety board to save the home position.
 */
template<size_t DOF>
int proficio_main(int argc, char** argv,
                  barrett::ProductManager& product_manager,  // NOLINT(runtime/references)
                  barrett::systems::Wam<DOF>& wam,           // NOLINT(runtime/references)
                  const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  barrett::SafetyModule* safety_module = product_manager.getSafetyModule();

  // Save initial position for later checks. Do this before activating.
  jp_type initial_position = wam.getJointPositions();
  usleep(250000);  // sleep time in microseconds
  safety_module->setMode(barrett::SafetyModule::ACTIVE);
  usleep(250000);  // sleep time in microseconds

  // If not active, try to put in active state. Try twice because the first
  // attempt sometimes fails due to timing.
  if (safety_module->getMode() != barrett::SafetyModule::ACTIVE) {
    safety_module->setMode(barrett::SafetyModule::ACTIVE);
    usleep(500000);  // sleep time in microseconds
    if (safety_module->getMode() != barrett::SafetyModule::ACTIVE) {
      return 2;
    }
  }

  Config arm;
  // strcmp() returns 0 if strings are equal
  if (!strcmp(argv[1], "left")) {
    arm = LEFT;
  } else if (!strcmp(argv[1], "right")) {
    arm = RIGHT;
  } else {
    // This should never happen because validate_args should catch a bad
    // configuration argument first.
    safety_module->setMode(barrett::SafetyModule::IDLE);
    safety_module->waitForMode(barrett::SafetyModule::IDLE, false);
    return 3;
  }

  // Set up torque limits for testing. P2 and P3 pucks have different gains, so
  // they require different maximum torque limits.
  safety_module->setTorqueLimit(3.0);
  jt_type max_torque;
  if (proficio::puck_type == proficio::P2) {
    max_torque << 11.0, 9.0, 9.0;
  } else if (proficio::puck_type == proficio::P3) {
    max_torque << 20.0, 22.0, 12.0;
  } else {
    // This should never happen unless the variable gets corrupted somehow.
    safety_module->setMode(barrett::SafetyModule::IDLE);
    safety_module->waitForMode(barrett::SafetyModule::IDLE, false);
    return 4;
  }

  // Define velocity commands for driving into joint stops.
  double command_speed;  // magnitude of velocity for each joint
  if (proficio::puck_type == proficio::P2) {
    command_speed = 0.3;
  } else {  // P3
    command_speed = 0.4;
  }
  jv_type command_direction;  // direction of velocity for each joint
  if (arm == LEFT) {
    // Home is -1.01, 0.96, -2.84, and extended position is -1.01, 0.96, -0.40.
    // So velocities to look for joint stops in extended position are -++.
    command_direction << -1, 1, 1;
  } else if (arm == RIGHT) {
    // Home is -1.01, -0.96, 2.84, and extended position is -1.01, -0.96, 0.40.
    // So velocities to look for joint stops in extended position are ---.
    command_direction << -1, -1, -1;
  } else {  // This should never happen because of previous checks.
    safety_module->setMode(barrett::SafetyModule::IDLE);
    safety_module->waitForMode(barrett::SafetyModule::IDLE, false);
    return 5;
  }
  jv_type command_velocity = command_speed * command_direction;

  // Set up controllers for driving into joint stops. Joints are tested one at
  // a time. The joint being tested is driven under velocity control while the
  // other joints are held stationary under position control.

  // Set up velocity controller for driving into joint stops.
  barrett::systems::ExposedOutput<jv_type> reference_velocity;
  barrett::systems::PIDController<jv_type, jt_type> velocity_pid_controller;
  jv_type zero_vel(0.0);
  jv_type kp_vel = zero_vel;  // current proportional gain for velocity control
  jv_type ki_vel = zero_vel;  // current integral gain for velocity control
  jv_type kp_vel_on;      // proportional gains when velocity control is on
  jv_type ki_vel_on;      // integral gains when velocity control is on
  // Since P2 and P3 pucks have different gain settings in firmware, we need
  // different software gains here.
  if (proficio::puck_type == proficio::P2) {
    kp_vel_on << 15, 35, 10;
    ki_vel_on << 250, 55, 50;
  } else {  // P3
    kp_vel_on << 20, 45, 13;
    ki_vel_on << 450, 130, 80;
  }
  velocity_pid_controller.setKp(kp_vel);
  velocity_pid_controller.setKd(zero_vel);
  velocity_pid_controller.setKi(ki_vel);
  reference_velocity.setValue(zero_vel);
  connect(wam.jvOutput, velocity_pid_controller.feedbackInput);
  connect(reference_velocity.output, velocity_pid_controller.referenceInput);

  // Set up position controller for holding joints stationary.
  barrett::systems::ExposedOutput<jp_type> reference_position;
  barrett::systems::PIDController<jp_type, jt_type> position_pid_controller;
  jp_type zero_pos(0.0);
  jp_type kp_pos = zero_pos;  // current proportional gain for position control
  jp_type kd_pos = zero_pos;  // current derivative gain for position control
  jp_type kp_pos_on;          // proportional gains when position control is on
  jp_type kd_pos_on;          // derivative gains when position control is on
  // Since P2 and P3 pucks have different gain settings in firmware, we need
  // different software gains here.
  if (proficio::puck_type == proficio::P2) {
    kp_pos_on << 500, 400, 400;
    kd_pos_on << 2, 2, 2;
  } else {
    kp_pos_on << 2000, 1200, 600;
    kd_pos_on << 5, 5, 5;
  }
  position_pid_controller.setKp(kp_pos);
  position_pid_controller.setKd(kd_pos);
  position_pid_controller.setKi(zero_pos);
  reference_position.setValue(wam.getJointPositions());
  connect(wam.jpOutput, position_pid_controller.feedbackInput);
  connect(reference_position.output, position_pid_controller.referenceInput);

  // Sum torques from position and velocity controllers. Note that each joint
  // should only have non-zero gains in one of the two controllers at any given
  // time.
  barrett::systems::Summer<jt_type, 2> joint_torque_sum("++");
  connect(velocity_pid_controller.controlOutput, joint_torque_sum.getInput(0));
  connect(position_pid_controller.controlOutput, joint_torque_sum.getInput(1));

  // Saturate torques for safety
  proficio::systems::JointTorqueSaturation<DOF>
      joint_torque_saturation(max_torque);
  connect(joint_torque_sum.output, joint_torque_saturation.input);

  // Final status checks before activating torques
  if (safety_module->getMode() == barrett::SafetyModule::IDLE) {
    // Fault detected. Set the mode to idle again because the safety module may
    // need to send commands to the motor pucks to shut down properly.
    safety_module->setMode(barrett::SafetyModule::IDLE);
    safety_module->waitForMode(barrett::SafetyModule::IDLE, false);
    return 6;
  } else if (((wam.getJointPositions()-initial_position).norm()) > 0.1) {
    // Robot has moved significantly during the startup process, so homecheck
    // may fail in interesting ways. Quit, so the user needs to try again.
    safety_module->setMode(barrett::SafetyModule::IDLE);
    safety_module->waitForMode(barrett::SafetyModule::IDLE, false);
    return 7;
  }

  // Activate torques to robot (should be zero right now)
  wam.idle();  // deactivate previous hold position commands (should be none)
  connect(joint_torque_saturation.output, wam.input);

  // Test joints one at a time, in the order 2,3,1. Testing in this order is
  // necessary because the J1 torque needed to lift the arm in a fully extended
  // position is higher than the torque that drives J1 into joint stop so far
  // that plastic hits metal. When J2 is home, the required J1 torque is
  // acceptable.

  bool quit = false;
  const int kTestOrderArray[3] = {2, 3, 1};
  const std::vector<int> kTestOrder(kTestOrderArray, kTestOrderArray +
      sizeof(kTestOrderArray) / sizeof(kTestOrderArray[0]));

  for (size_t k = 0; k < kTestOrder.size(); k++) {
    int joint_number = kTestOrder[k] - 1;  // joint numbers are indexed from 0
    jv_type velocity;
    velocity = zero_vel;
    reference_velocity.setValue(velocity);

    // turn on/off appropriate gains
    for (int i = 0; i < 3; i++) {
      if (i == joint_number) {
        // turn on velocity control, turn off position control
        kp_pos[i] = 0;
        kd_pos[i] = 0;
        kp_vel[i] = kp_vel_on[i];
        ki_vel[i] = ki_vel_on[i];
      } else {
        // turn off velocity control, turn on position control
        kp_vel[i] = 0;
        ki_vel[i] = 0;
        kp_pos[i] = kp_pos_on[i];
        kd_pos[i] = kd_pos_on[i];
      }
    }
    velocity_pid_controller.setKp(kp_vel);
    velocity_pid_controller.setKi(ki_vel);
    velocity_pid_controller.setKd(zero_vel);
    position_pid_controller.setKp(kp_pos);
    position_pid_controller.setKd(kd_pos);
    position_pid_controller.setKi(zero_pos);

    // Hold current position for joints not being tested.
    reference_position.setValue(wam.getJointPositions());

    int n_steps = 100;
    jv_type velocity_increment = command_velocity / static_cast<double>(n_steps);

    // Ramp up commanded velocity.
    for (int step = 0; step < n_steps; step++) {
      velocity[joint_number] += velocity_increment[joint_number];
      reference_velocity.setValue(velocity);
      if (step % 10 == 0) {
        if (safety_module->getMode() == barrett::SafetyModule::IDLE) {
          // Fault detected
          return 2;
        }
      }
      btsleep(0.001);
    }

    // Try to move at constant velocity for a short time.
    for (int i = 0; i < 50; i++) {
      if (safety_module->getMode() == barrett::SafetyModule::IDLE) {
        // Fault detected
        return 2;
      }
      btsleep(0.01);
    }

    // Check joint velocity. Non-negligible velocity means that there is no
    // joint stop to slow down movement, so the robot is not in the expected
    // position.
    velocity = wam.getJointVelocities();
    double delta;
    // Since P2 and P3 pucks have different control gains and we are commanding
    // different torques, we should also expect different velocities. With more
    // careful tuning of the max torque values defined earlier, this delta
    // could probably be the same.
    if (proficio::puck_type == proficio::P2) {
      delta = 0.1;
    } else {
      delta = 0.2;
    }
    if (fabs(velocity[joint_number]) >= delta) {
      quit = true;
    }

    // Ramp commanded velocity back down for a graceful exit. This is important
    // because a sudden stop often leads to the robot springing back off the
    // joint stops, which then prevents it from reaching the home position
    // correctly.
    for (int step = 0; step < n_steps; step++) {
      velocity[joint_number] -= velocity_increment[joint_number];
      reference_velocity.setValue(velocity);
      if (step % 10 == 0) {
        if (safety_module->getMode() == barrett::SafetyModule::IDLE) {
          return 2;
        }
      }
      btsleep(0.001);
    }
    reference_velocity.setValue(zero_vel);
    btsleep(0.2);
  }

  // If robot is in correct position, (J1 home, J2 home, J3 extended), then
  // slowly move J3 to home position. If J3 doesn't make it to home, that means
  // the cradle is in the way, so return a failure. Put limits on the torque
  // here to avoid breaking the cradle if it is in the way.

  if (!quit) {
    // Turn on position control and turn off velocity control for all joints.
    velocity_pid_controller.setKp(zero_vel);
    velocity_pid_controller.setKi(zero_vel);
    reference_velocity.setValue(zero_vel);
    jp_type start = wam.getJointPositions();
    reference_position.setValue(start);
    position_pid_controller.setKp(kp_pos_on);
    position_pid_controller.setKd(kd_pos_on);
    // Lower the torque saturation limit now to prevent possible breaking the
    // gravity cradle.
    max_torque << 8, 8, 4;
    joint_torque_saturation.setJointTorqueLimit(max_torque);

    // Set expected distance to travel in J3 based on published joint limits.
    // @TODO(ab): Get joint limits from a configuration file.
    double joint3_range = 2.84 - 0.40;
    double joint3_velocity = -1.0 * command_velocity[2];
    double dt = 0.01;
    jp_type position = start;

    bool home = false;
    while (!home) {
      if (safety_module->getMode() == barrett::SafetyModule::IDLE) {
        // Fault detected
        return 2;
      }
      position[2] += joint3_velocity * dt;
      reference_position.setValue(position);
      btsleep(dt);
      // Command movement until commanded position is a little bit beyond
      // expected joint stop location.
      if (joint3_range - fabs(position[2] - start[2]) <= -0.1) {
        home = true;
      }
    }
    btsleep(0.2);  // wait for position to settle
    // Compare actual position to commanded position. If the error is large,
    // that probably means the gravity cradle got in the way and the arm is
    // stuck. In that case, quit without setting the home position.
    if (fabs(position[2] - (wam.getJointPositions())[2]) > 0.05) {
      quit = true;
    }
  }

  // Finished! Set the home position if successful, or just quit.
  barrett::systems::disconnect(wam.input);
  btsleep(0.5);
  if (!quit) {
    // All the checks were successful. Set the current position as home.
    // Requires cycling the safety board state to set the new home position.
    // wam.getHomePosition() returns the home position from
    //   ~/.barrett/calibration_data/wam3/zerocal.conf
    // Then wam.getLowLevelWam().definePosition() associates the current puck
    // positions with those joint positions and tells the pucks that they have
    // been zeroed.
    safety_module->getPuck()->setProperty(barrett::Puck::ZERO, 0);
    safety_module->setMode(barrett::SafetyModule::IDLE);
    safety_module->waitForMode(barrett::SafetyModule::IDLE, false);
    wam.getLowLevelWam().definePosition(wam.getHomePosition());
    safety_module->setMode(barrett::SafetyModule::ACTIVE);
    safety_module->waitForMode(barrett::SafetyModule::ACTIVE, false);
    wam.gravityCompensate(true);
    safety_module->setMode(barrett::SafetyModule::ESTOP);
    wam.getLowLevelWam().getPuckGroup().setProperty(
        barrett::Puck::MODE, barrett::MotorPuck::MODE_IDLE);
    return 1;
  } else {
    safety_module->setMode(barrett::SafetyModule::IDLE);
    safety_module->waitForMode(barrett::SafetyModule::IDLE, false);
    safety_module->setMode(barrett::SafetyModule::ESTOP);
    wam.getLowLevelWam().getPuckGroup().setProperty(
        barrett::Puck::MODE, barrett::MotorPuck::MODE_IDLE);
    return 2;
  }
  // Should never get to here!
  return 0;
}


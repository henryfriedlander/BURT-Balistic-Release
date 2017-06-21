/*  Copyright 2016 Barrett Technology <support@barrett.com>
 *
 *  This file is part of proficio_toolbox.
 *
 *  This version of proficio_toolbox is free software: you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This version of proficio_toolbox is distributed in the hope that it will be
 *  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this version of proficio_toolbox.  If not, see
 *  <http://www.gnu.org/licenses/>.
 */

/** @file cube_sphere_mod.cpp
 *
 *  This is a demonstration of the capabilities of the Proficio robot for
 *  rehabilitation. In this demo, there is a box and sphere that provide haptic
 *  feedback when interacted with. The user can pop into and out of the objects
 *  if enough force is exerted. 
 *
 *  This file includes the haptics and logic behind the game. It must be run
 *  concurrently with the corresponding python visualization. To run the demo,
 *  run the following commands in two separate terminals:
 *
 *    ./cube_sphere_mod <IP address>
 *    python ex_10_haptics_visualization.py <IP address>
 *
 *  By default, the IP address is 127.0.0.1 for both commands
 *
 *  User assistance is available in this demo. Gravity assist aids the user in
 *  supporting the weight of their own arm. Movement assistance aids the user
 *  in moving towards the target. The degree of assistance can be adjusted with
 *  the following key presses:
 *
 *  key          | action
 *  ------------ | -----------------------
 *  <up arrow>   | Increase gravity assist
 *  <down arrow> | Decrease gravity assist
 *  <delete>     | Turn off gravity assist
 *
 *  NOTICE: This program is for demonstration purposes only.
 *  It is not approved for clinical use.
 */

#include "cube_sphere_mod.h"  // NOLINT (build/include_subdir)

#include <string>
#include <signal.h>

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/config.h>                     // NOLINT(build/include_order)
#include <barrett/exception.h>                  // NOLINT(build/include_order)
#include <barrett/products/product_manager.h>   // NOLINT(build/include_order)
#include <barrett/systems.h>                    // NOLINT(build/include_order)
#include <proficio/systems/utilities.h>         // NOLINT(build/include_order)
#include <proficio/systems/haptics.h>           // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)

#define BARRETT_SMF_VALIDATE_ARGS
//#define NO_CONTROL_PENDANT

#include <proficio/standard_proficio_main.h>    // NOLINT(build/include_order)

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

const char* remoteHost = NULL;
double kp = 3e3;
double kd = 3e1;
bool game_exit = false;

bool validate_args(int argc, char** argv) {
  switch (argc) {
    case 4:
      kd = atof(argv[3]);
    case 3:
      kp = atof(argv[2]);
    case 2:
      remoteHost = argv[1];
      break;
    default:
      remoteHost = "127.0.0.1";
      printf("Defaulting to 127.0.0.1\n");
  }
  printf("Gains: kp = %f; kd = %f\n", kp, kd);
  return true;
}

namespace cube_sphere_mod {
/** When killed from outside (by GUI), this allows a graceful exit. */
void exit_program_callback(int signum) { game_exit = true; }
}  // namespace cube_sphere_mod

cf_type scale(boost::tuple<cf_type, double> t) {
  return t.get<0>() * t.get<1>();
}

template <size_t DOF>
int proficio_main(int argc, char** argv,
                  barrett::ProductManager& product_manager,  // NOLINT
                  barrett::systems::Wam<DOF>& wam,           // NOLINT
                  const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  wam.gravityCompensate();
  barrett::SafetyModule* safety_module = product_manager.getSafetyModule();
  barrett::SafetyModule::PendantState ps;
  safety_module->getPendantState(&ps);
  std::string filename = "calibration_data/wam3/";
  if (side == LEFT) {
    filename = filename + "LeftConfig.txt";
  } else if (side == RIGHT) {
    filename = filename + "RightConfig.txt";
  }

  // Catch kill signals if possible for a graceful exit.
  signal(SIGINT, cube_sphere_mod::exit_program_callback);
  signal(SIGTERM, cube_sphere_mod::exit_program_callback);
  signal(SIGKILL, cube_sphere_mod::exit_program_callback);

  proficio::systems::UserGravityCompensation<DOF> gravity_comp(
      barrett::EtcPathRelative(filename).c_str());
  gravity_comp.setGainZero();

  // Instantiate Systems
  NetworkHaptics<DOF> network_haptics(product_manager.getExecutionManager(),
                                      remoteHost, &gravity_comp);
  const cp_type cylinder_center(0.4, -0.15, 0.0);
  const double face_pt1 = 0.12;
  const double face_pt2 = 0.30;
  proficio::systems::haptictoolbox::CylindricalAssist cylinder(cylinder_center, face_pt1, face_pt2);
  cylinder.SetRadius(0);
  const cp_type box_center(0.35, 0.2, 0.0);
  const barrett::math::Vector<3>::type box_size(0.2, 0.2, 0.2);
  barrett::systems::HapticBox box(box_center, box_size);
  barrett::systems::Summer<cf_type> direction_sum;
  barrett::systems::Summer<double> depth_sum;
  barrett::systems::PIDController<double, double> pid_controller;
  barrett::systems::Constant<double> zero(0.0);
  barrett::systems::TupleGrouper<cf_type, double> tuple_grouper;
  barrett::systems::Callback<boost::tuple<cf_type, double>, cf_type> mult(  // NOLINT
      scale);
  barrett::systems::ToolForceToJointTorques<DOF> tf2jt;
  barrett::systems::Summer<jt_type, 3> joint_torque_sum("+++");
  jt_type jtLimits(35.0);
  proficio::systems::JointTorqueSaturation<DOF> joint_torque_saturation(
      jtLimits);
  v_type dampingConstants(20.0);
  dampingConstants[2] = 10.0;
  dampingConstants[0] = 30.0;
  jv_type velocity_limits(1.4);
  proficio::systems::JointVelocitySaturation<DOF> velsat(dampingConstants,
                                                         velocity_limits);

  jv_type joint_vel_filter_freq(20.0);
  barrett::systems::FirstOrderFilter<jv_type> joint_vel_filter;
  joint_vel_filter.setLowPass(joint_vel_filter_freq);

  // configure Systems
  pid_controller.setKp(kp);
  pid_controller.setKd(kd);

  // line up coordinate axis with python visualization
  barrett::systems::modXYZ<cp_type> mod_axes;
  mod_axes.negX();
  mod_axes.negY();
  mod_axes.xOffset(0.85);
  if (side == LEFT) {
    mod_axes.yOffset(0.27);
  } else if (side == RIGHT) {
    mod_axes.yOffset(-0.27);
  }
  mod_axes.zOffset(-0.2);

  // line up forces so that they correlate correctly with python visualization
  barrett::systems::modXYZ<cf_type> mod_force;
  mod_force.negX();
  mod_force.negY();
  barrett::systems::connect(wam.jpOutput, gravity_comp.input);
  barrett::systems::connect(wam.jvOutput, joint_vel_filter.input);
  barrett::systems::connect(joint_vel_filter.output, velsat.input);

  barrett::systems::connect(wam.toolPosition.output, mod_axes.input);
  barrett::systems::connect(mod_axes.output, network_haptics.input);
  barrett::systems::connect(mod_axes.output, cylinder.input);
  barrett::systems::connect(mod_axes.output, box.input);

  barrett::systems::connect(mult.output, mod_force.input);
  barrett::systems::connect(mod_force.output, tf2jt.input);

  // This summing only works because it's not possible to be in contact with 
  // both haptic objects at the same time, so only one of the inputs to each 
  // summer will be non-zero
  barrett::systems::connect(cylinder.directionOutput, direction_sum.getInput(0));
  barrett::systems::connect(cylinder.depthOutput, depth_sum.getInput(0));
  barrett::systems::connect(box.directionOutput, direction_sum.getInput(1));
  barrett::systems::connect(box.depthOutput, depth_sum.getInput(1));

  barrett::systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
  barrett::systems::connect(direction_sum.output, tuple_grouper.getInput<0>());

  barrett::systems::connect(depth_sum.output, pid_controller.referenceInput);
  barrett::systems::connect(zero.output, pid_controller.feedbackInput);
  barrett::systems::connect(pid_controller.controlOutput, tuple_grouper.getInput<1>());

  barrett::systems::connect(tuple_grouper.output, mult.input);
  barrett::systems::connect(tf2jt.output, joint_torque_sum.getInput(0));
  barrett::systems::connect(gravity_comp.output, joint_torque_sum.getInput(1));
  barrett::systems::connect(velsat.output, joint_torque_sum.getInput(2));
  barrett::systems::connect(joint_torque_sum.output,
                            joint_torque_saturation.input);
  // adjust velocity fault limit
  product_manager.getSafetyModule()->setVelocityLimit(1.5);
  product_manager.getSafetyModule()->setTorqueLimit(3.0);
  wam.idle();
  barrett::systems::connect(joint_torque_saturation.output, wam.input);

  while (true) {  // Allow the user to stop and resume with pendant buttons
#ifndef NO_CONTROL_PENDANT
    product_manager.getSafetyModule()->waitForMode(
        barrett::SafetyModule::IDLE);  // block until the user Shift-idles
#endif
    if (game_exit) {
      product_manager.getPuck(1)
          ->setProperty(product_manager.getPuck(1)->getBus(), 1, 8, 3);
      product_manager.getPuck(2)
          ->setProperty(product_manager.getPuck(2)->getBus(), 2, 8, 3);
      product_manager.getPuck(3)
          ->setProperty(product_manager.getPuck(3)->getBus(), 3, 8, 3);
      barrett::systems::disconnect(wam.input);
      return 0;
    }
    barrett::btsleep(0.02);
#ifndef NO_CONTROL_PENDANT
    product_manager.getSafetyModule()->waitForMode(
        barrett::SafetyModule::ACTIVE);  // block until the user Shift-activates
#endif
  }
  return 0;
}

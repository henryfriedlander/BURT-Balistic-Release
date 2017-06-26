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

/** @file cube_sphere.cpp
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
 *    ./cube_sphere <IP address>
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
 
 /* TODOs
  * Create a system inside this file for HapticLine that has an input that can handle 
  * Create a callback for the hapticLine
  * 	(create something similar to the HapticCalc line 128 in haptic_world)
  */ 

#include "cube_sphere.h"  // NOLINT (build/include_subdir)
#include "haptic_line.h" // NOlINT (build/include_subdir)
#include "normalize.h" // NOlINT (build/include_subdir)

#include <string>
#include <signal.h>
#include <fstream>
#include <typeinfo>

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/config.h>                     // NOLINT(build/include_order)
#include <barrett/exception.h>                  // NOLINT(build/include_order)
#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/products/product_manager.h>   // NOLINT(build/include_order)
#include <barrett/systems.h>                    // NOLINT(build/include_order)
#include <proficio/systems/utilities.h>         // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)

// Networking
#include <netinet/in.h>
#include <sys/types.h>

#include <proficio/systems/utilities.h>
#define BARRETT_SMF_VALIDATE_ARGS
//#define NO_CONTROL_PENDANT

#include <proficio/standard_proficio_main.h>    // NOLINT(build/include_order)

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(4);  // defines v_type to have length 4

const char* remoteHost = NULL;
double kp = 3e3;
double kd = 3e1;
bool game_exit = false;
barrett::systems::ExposedOutput<v_type> message;

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


namespace cube_sphere {
/** When killed from outside (by GUI), this allows a graceful exit. */
void exit_program_callback(int signum) { game_exit = true; }
}  // namespace cube_sphere

cf_type scale(boost::tuple<cf_type, double> t) {
  return t.get<0>() * t.get<1>();
}

barrett::systems::ExposedOutput<v_type> append(cf_type axes) {
	v_type msg_tmp;
	for (int i = 0; i < 3; i++){
		msg_tmp[i] = axes[i];
	}
	msg_tmp[3] = 0.0; // needs to be randomly generated and used as an index in the cpp file
	message.setValue(msg_tmp);
	return message.output;
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
  
  // 
  // it is seperated by newlines
  // first is direction where the target is which is a number from 1-6
  // 
  /*
   * reads from file which is generated by dragonfly for communication
   * file is seperated by \n
   * 
   * first is the direction where the target is (a number from 1-6)
   * 1 is +z direction
   * 2 is -z direction
   * 3 is +y direction
   * 4 is -y direction
   * 5 is +x direction
   * 6 is -x direction
   * 
   * second line has 4 threshholds (10, 20, 30, 40)
   * 
   * thrid line has one of four targets 
   * 
   * 5 cm
   * 10 cm
   * 15 cm
   * 20 cm
  
  ifstream data_input;
  data_input.open("data_input.txt");
  char output[100];
  if (myReadFile.is_open()) {
  while (!myReadFile.eof()) {
    myReadFile >> output;
    cout<<output;
    }
  }
  myReadFile.close();
  */
  
  // Catch kill signals if possible for a graceful exit.
  signal(SIGINT, cube_sphere::exit_program_callback);
  signal(SIGTERM, cube_sphere::exit_program_callback);
  signal(SIGKILL, cube_sphere::exit_program_callback);

  proficio::systems::UserGravityCompensation<DOF> gravity_comp(
      barrett::EtcPathRelative(filename).c_str());
  gravity_comp.setGainZero();

  // Instantiate Systems
  NetworkHaptics<DOF> network_haptics(product_manager.getExecutionManager(),
                                      remoteHost, &gravity_comp);
  
  std::vector<cp_type> path;
  double start_point[] = {0.4, -0.15, 0.12};
  double end_point[] = {0.4, -0.15, -0.12};
  for (double t = 0.0; t <=1.0; t += 0.05){
	  double path_x = (1 - t) * start_point[0] + t * end_point[0];
	  double path_y = (1 - t) * start_point[1] + t * end_point[1];
	  double path_z = (1 - t) * start_point[2] + t * end_point[2];
	  cp_type path_point(path_x, path_y, path_z);
	  path.push_back(path_point);
  }
  
  //const cp_type transformVector(0.85, -0.27, -0.2);
  const cp_type ball_center(0.4, -0.15, 0.0);
  const cp_type system_center(0.440, -0.109, 0.2);
  wam.moveTo(system_center);
  printf("Done Moving Arm! \n");
  //const double radius = 0.02;
  //barrett::systems::HapticBall ball(ball_center, radius);
  const cp_type box_center(0.35, 0.2, 0.0);
  const barrett::math::Vector<3>::type box_size(0.2, 0.2, 0.2);
  const int XorYorZ = 1;
  barrett::systems::HapticBox ball(ball_center, box_size);
  barrett::systems::HapticLine line(ball_center, XorYorZ);
  
  barrett::systems::Summer<cf_type> direction_sum;
  barrett::systems::Summer<double> depth_sum;
  barrett::systems::PIDController<double, double> pid_controller;
  barrett::systems::Constant<double> zero(0.0);
  barrett::systems::TupleGrouper<cf_type, double> tuple_grouper;
  barrett::systems::Callback<boost::tuple<cf_type, double>, cf_type> mult(  // NOLINT
      scale);
  barrett::systems::Callback< cf_type, barrett::systems::ExposedOutput<v_type> > nhAppend(  // NOLINT
      append);
  barrett::systems::ToolForceToJointTorques<DOF> tf2jt;
  barrett::systems::Summer<jt_type, 3> joint_torque_sum("+++");
  jt_type jtLimits(35.0);
  proficio::systems::JointTorqueSaturation<DOF> joint_torque_saturation(
      jtLimits);
  v_type dampingConstants(20.0);
  dampingConstants[2] = 10.0;
  dampingConstants[0] = 30.0;
  jv_type velocity_limits(1.7);
  proficio::systems::JointVelocitySaturation<DOF> velsat(dampingConstants,
                                                         velocity_limits);

  barrett::systems::Normalize<cf_type> normalizeHelper;
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
  barrett::systems::connect(mod_axes.output, nhAppend.input);
  barrett::systems::forceConnect(nhAppend.output,  network_haptics.input);
  barrett::systems::connect(mod_axes.output, ball.input);
  barrett::systems::connect(mod_axes.output, line.input);

  barrett::systems::connect(mult.output, mod_force.input);
  barrett::systems::connect(mod_force.output, tf2jt.input);

  // This summing only works because it's not possible to be in contact with 
  // both haptic objects at the same time, so only one of the inputs to each 
  // summer will be non-zero
  barrett::systems::connect(ball.directionOutput, direction_sum.getInput(0));
  barrett::systems::connect(ball.depthOutput, depth_sum.getInput(0));
  barrett::systems::connect(line.directionOutput, direction_sum.getInput(1));
  barrett::systems::connect(line.depthOutput, depth_sum.getInput(1));

  barrett::systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
  barrett::systems::connect(direction_sum.output, normalizeHelper.input);
  barrett::systems::connect(normalizeHelper.output,tuple_grouper.getInput<0>());
  
  //std::cout << typeid(direction_sum.output).name() << std::endl;
  
  barrett::systems::connect(depth_sum.output, pid_controller.referenceInput);
  barrett::systems::connect(zero.output, pid_controller.feedbackInput);
  barrett::systems::connect(pid_controller.controlOutput, tuple_grouper.getInput<1>());

  barrett::systems::connect(tuple_grouper.output, mult.input);
  barrett::systems::connect(tf2jt.output, joint_torque_sum.getInput(0));
  barrett::systems::connect(gravity_comp.output, joint_torque_sum.getInput(1));
  barrett::systems::connect(joint_torque_sum.output, joint_torque_saturation.input);
  barrett::systems::connect(velsat.output, joint_torque_sum.getInput(2));

  // adjust velocity fault limit
  product_manager.getSafetyModule()->setVelocityLimit(1.5);
  product_manager.getSafetyModule()->setTorqueLimit(3.0);
  wam.idle();
  barrett::systems::connect(joint_torque_saturation.output, wam.input);
  int counter = 0; 
  int timer = 0;
  cp_type cp;
  cp_type target_center;
  target_center[0] = 0.439;
  target_center[1] = 0.417;
  target_center[2] = 0.366;
  double target_radius = 0.03;
  while (true) {  // Allow the user to stop and resume with pendant buttons
	cp = barrett::math::saturate(wam.getToolPosition(), 9.999);
    if ( counter % 100 == 0 ) {
		printf("[%6.3f, %6.3f, %6.3f]\n", cp[0], cp[1], cp[2]);
	}
	counter++;
	if ( pow((cp[0]-target_center[0]),2) + pow((cp[1]-target_center[1]),2) + pow((cp[2]-target_center[2]),2) < pow(target_radius,2)){
		if (timer > 10){
			wam.moveTo(system_center);
			// TODO: after the wam moves back to the center hand over a different set of instructions to the python visualization
			// this will allow a graceful transition between trials
			// (this may not be necessary if the system provides a text file for each trial)
			wam.idle();
			timer = 0;
		}
		else{
			timer++;
		}
	}else{
		timer = 0;
	}

#ifndef NO_CONTROL_PENDANT
    product_manager.getSafetyModule()->waitForMode(
        barrett::SafetyModule::IDLE);  // block until the user Shift-idles
#endif

    if (product_manager.getSafetyModule()->getMode() == barrett::SafetyModule::IDLE) {
		/*
		product_manager.getPuck(1)
		  ->setProperty(product_manager.getPuck(1)->getBus(), 1, 8, 3);
		product_manager.getPuck(2)
		  ->setProperty(product_manager.getPuck(2)->getBus(), 2, 8, 3);
		product_manager.getPuck(3)
		  ->setProperty(product_manager.getPuck(3)->getBus(), 3, 8, 3);
		barrett::systems::disconnect(wam.input);
		* */
		wam.moveHome();
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

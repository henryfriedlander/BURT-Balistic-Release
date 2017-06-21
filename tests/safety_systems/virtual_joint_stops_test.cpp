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

/** @file virtual_joint_stops_test.cpp
 *
 *  Test the virtual joint damper system by running it on the robot or
 *  by logging values.
 * 
 *  This program can either run the robot with joint dampers or log values
 *  for validation. Which option to use is specified with a command line
 *  argument, either "robot" or "log"
 *  Example: `./virtual_joint_stops robot`
 *  will run the robot with joint stops.
 */

// #include <proficio/systems/utilities/virtual_joint_stops.h">

#include <cstdio>
#include <cstdlib>

#include <iostream>
#include <string>
#include <syslog.h>

#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/units.h>

//#define NO_CONTROL_PENDANT
#include <proficio/standard_proficio_main.h>
#include <proficio/systems/utilities.h>

using barrett::detail::waitForEnter;

template<size_t DOF>
int proficio_main(int argc, char** argv, barrett::ProductManager& pm,
           barrett::systems::Wam<DOF>& wam, const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  if (DOF != 3) { 
    printf("This program is supported only for the Proficio robot");
    return 0;
  }

  jp_type joint_mins;
  jp_type joint_maxes; 
  joint_mins << -1.01, -0.96, 0.0;
  joint_maxes << 0.56, 0.96, 0.0;
  if (side == RIGHT) {
    joint_mins [2] = 0.40;
    joint_maxes [2] = 2.84;
  } else if (side == LEFT) {
    joint_mins [2] = -2.84;
    joint_maxes [2] = -0.40;
  } else {
    printf("No known robot found");
    return 1;
  }

  v_type damping_constants(0.0);
  damping_constants << 60.0, 50.0, 50.0;
  jp_type offset(0.3);
  proficio::systems::VirtualJointStops<DOF> joint_stops(joint_mins,
                                                       joint_maxes,
                                                       damping_constants,
                                                       offset);
  bool useRobot = false;
  if ((argc >= 2) && !(strcmp(argv[1], "log"))) {  // strcmp returns 0 if equal
    useRobot = false;
  } else if ((argc >= 2) && !(strcmp(argv[1], "robot"))) {  
    useRobot = true;
  } else {
    printf("Command line arguments are invalid.\n"
           "Usage: ./virtual_joint_stops <robot/log>\n");
    return 1;
  }
  pm.getSafetyModule()->setVelocityLimit(2.0);
  pm.getSafetyModule()->setTorqueLimit(4.0);

  if (useRobot) {  
    wam.gravityCompensate();
    printf("Press [Enter] to add joint dampers.");
    waitForEnter();

    jv_type freq;
    freq << 10, 10, 4;
    barrett::systems::FirstOrderFilter<jv_type> filter_vel;
    filter_vel.setLowPass(freq);
    // Connect joint limit spring system
    barrett::systems::connect(wam.jvOutput, filter_vel.input); 
    barrett::systems::connect(wam.jpOutput, joint_stops.posInput);
    barrett::systems::connect(filter_vel.output, joint_stops.velInput);
    wam.trackReferenceSignal(joint_stops.output);
    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
  } else {  // set position and velocity and log
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
      printf("ERROR: Couldn't create temporary file!\n");
      return 1;
    }
    // set position and velocity for joint limit spring
    jp_type curr_pos(0.0);
    jv_type curr_vel(0.0);
    barrett::systems::ExposedOutput<jp_type> pos(curr_pos);
    barrett::systems::ExposedOutput<jv_type> vel(curr_vel);
    barrett::systems::connect(pos.output, joint_stops.posInput);
    barrett::systems::connect(vel.output, joint_stops.velInput);

    barrett::systems::Ramp time(pm.getExecutionManager(), 1.0);
    barrett::systems::TupleGrouper<double, jp_type, jv_type, jt_type> tgLog;
    barrett::systems::connect(time.output, tgLog.template getInput<0>());
    barrett::systems::connect(pos.output, tgLog.template getInput<1>());
    barrett::systems::connect(vel.output, tgLog.template getInput<2>());
    barrett::systems::connect(joint_stops.output, tgLog.template getInput<3>());

    typedef boost::tuple<double, jp_type, jv_type, jt_type> log_tuple_type;
    const size_t kPeriodMultiplier = 50;
    barrett::systems::PeriodicDataLogger<log_tuple_type> logger(
        pm.getExecutionManager(),
        new barrett::log::RealTimeWriter<log_tuple_type>(tmpFile,
            kPeriodMultiplier * pm.getExecutionManager()->getPeriod()),
        kPeriodMultiplier);

    time.start();
    barrett::systems::connect(tgLog.output, logger.input);
    printf("Logging started.\n\n");

    curr_pos[0] = 0.6;
    curr_vel[0] = 0.9;
    pos.setValue(curr_pos);
    vel.setValue(curr_vel);
    barrett::btsleep(0.1);

    for (int j = 0; j < 3; j++) {  // loop through joints
      printf("Joint %d\n-------\n", j);
      // loop through vel +/-(0, 0.5, 1) with pos = limit +/- offset*0.5
      for (double v = 1; v >= 0; v -= 0.5) {  
        curr_pos[j] = joint_maxes[j] - (0.5 * offset[j]);
        curr_vel[j] = v;  // toward max pos
        pos.setValue(curr_pos);
        vel.setValue(curr_vel);
        barrett::btsleep(0.1);
        printf("Pos: %6.3f  Vel: %6.1f  Torque: %6.1f\n",
               (joint_stops.posInput.getValue())[j],            // position
               (joint_stops.velInput.getValue())[j],            // velocity
               (tgLog.template getInput<3>().getValue())[j]);   // torque

        curr_pos[j] = joint_mins[j] + (0.5 * offset[j]);
        curr_vel[j] = -1 * v;  // toward min pos
        pos.setValue(curr_pos);
        vel.setValue(curr_vel);
        barrett::btsleep(0.1);
        printf("Pos: %6.3f  Vel: %6.1f  Torque: %6.1f\n",
               (joint_stops.posInput.getValue())[j],            // position
               (joint_stops.velInput.getValue())[j],            // velocity
               (tgLog.template getInput<3>().getValue())[j]);   // torque
      }
      curr_vel[j] = 1.0;
      vel.setValue(curr_vel);
      // loop through pos = limit +/- offset*(0.25, 0.5, 0.75), vel = +/-1
      for (double  p=0.25; p<1; p+=0.25) {
        curr_pos[j] = joint_mins[j] + (p*offset[j]);
        pos.setValue(curr_pos);
        barrett::btsleep(0.1);
        printf("Pos: %6.3f  Vel: %6.1f  Torque: %6.1f\n",
               (joint_stops.posInput.getValue())[j],            // position
               (joint_stops.velInput.getValue())[j],            // velocity
               (tgLog.template getInput<3>().getValue())[j]);   // torque

        curr_pos[j] = joint_maxes[j] - (p*offset[j]);
        pos.setValue(curr_pos);
        barrett::btsleep(0.1);
        printf("Pos: %6.3f  Vel: %6.1f  Torque: %6.1f\n",
               (joint_stops.posInput.getValue())[j],            // position
               (joint_stops.velInput.getValue())[j],            // velocity
               (tgLog.template getInput<3>().getValue())[j]);   // torque
      }
      // leave joint in the middle, away from joint stops 
      curr_pos[j] = (joint_mins[j] + joint_maxes[j]) / 2;
      printf("-------------------------\n\n");
    }
    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);

    disconnect(wam.input);
    logger.closeLog();
    printf("Logging stopped.\n");

    barrett::log::Reader<log_tuple_type> lr(tmpFile);
    std::string logfile("log.txt");
    lr.exportCSV(logfile.c_str());
    printf("Output written to %s.\n",logfile.c_str());
    std::remove(tmpFile);
  }
  return 0;
}

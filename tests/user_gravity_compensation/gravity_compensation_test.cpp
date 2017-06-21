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

/** @file gravity_compensation_test.cpp
 *
 * Compare natural neighbor interpolation with linear interpolation for user
 * gravity compensation. Save output of both to log files and allow toggling
 * between the two to compare how they feel with the robot.
 */

#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()
#include <string>

#include <boost/tuple/tuple.hpp>               // NOLINT(build/include_order)

#include <barrett/config.h>                    // NOLINT(build/include_order)
#include <barrett/log.h>                       // NOLINT(build/include_order)
#include <barrett/products/product_manager.h>  // NOLINT(build/include_order)
#include <barrett/systems.h>                   // NOLINT(build/include_order)
#include <barrett/units.h>                     // NOLINT(build/include_order)

#define BARRETT_SMF_VALIDATE_ARGS
#define NO_CONTROL_PENDANT
#include <proficio/standard_proficio_main.h>   // NOLINT(build/include_order)

#include <proficio/systems/utilities.h>        // NOLINT(build/include_order)

using barrett::systems::connect;

bool validate_args(int argc, char** argv) {
  if (argc != 2) {
    printf("Usage: %s <fileName>\n", argv[0]);
    return false;
  }
  return true;
}

void printMenu() {
  printf("Commands:\n");
  printf("  n  Turn on gravity comp with natural neighbor interpolation\n");
  printf("  l  Turn on gravity comp with linear interpolation\n");
  printf("  k  Increase gravity comp gain\n");
  printf("  j  Decrease gravity comp gain\n");
  printf("  z  Turn off gravity comp (reset gains to zero)\n");
  printf("  m  Print menu again\n");
  printf("  q  Quit\n");
}

template<size_t DOF>
int proficio_main(int argc, char** argv,
                  barrett::ProductManager& product_manager,  // NOLINT
                  barrett::systems::Wam<DOF>& wam,           // NOLINT
                  const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  char tmpFile[] = "/tmp/btXXXXXX";
  if (mkstemp(tmpFile) == -1) {
    printf("ERROR: Couldn't create temporary file!\n");
    return 1;
  }

  std::string filename_nn = "calibration_data/wam3/";  // nn = natural neighbor
  std::string handedness_name = "";
  if (side == LEFT) {
    filename_nn = filename_nn + "LeftConfig.txt";
    std::string handedness_name = "proficio_left";
  } else if (side == RIGHT) {
    filename_nn = filename_nn + "RightConfig.txt";
    std::string handedness_name = "proficio_right";
  }
  std::string filename_linear = handedness_name + "/calibration_data/wam3/" +
                             "user_gravity_compensation_grid.conf";

  wam.gravityCompensate();

  barrett::systems::Ramp time(product_manager.getExecutionManager(), 1.0);
  proficio::systems::UserGravityCompensation<DOF> user_grav_comp_nn(
      barrett::EtcPathRelative(filename_nn).c_str());  // natural neighbor
                                                       // interpolation
  user_grav_comp_nn.setGainZero();
  proficio::systems::UserGravityCompensationLinear<DOF> user_grav_comp_linear(
      barrett::EtcPathRelative(filename_linear).c_str());  // linear
                                                           // interpolation
  user_grav_comp_linear.setGainZero();

  connect(wam.jpOutput, user_grav_comp_nn.input);
  connect(wam.jpOutput, user_grav_comp_linear.input);

  barrett::systems::TupleGrouper<double, jp_type,
                                 jt_type, jt_type> tuple_grouper;
  connect(time.output, tuple_grouper.template getInput<0>());
  connect(wam.jpOutput, tuple_grouper.template getInput<1>());
  connect(user_grav_comp_nn.output, tuple_grouper.template getInput<2>());
  connect(user_grav_comp_linear.output, tuple_grouper.template getInput<3>());

  typedef boost::tuple<double, jp_type, jt_type, jt_type> tuple_type;  // NOLINT
  const size_t kPeriodMultiplier = 1;
  barrett::systems::PeriodicDataLogger<tuple_type> logger(
      product_manager.getExecutionManager(),
      new barrett::log::RealTimeWriter<tuple_type>(tmpFile, kPeriodMultiplier *
        product_manager.getExecutionManager()->getPeriod()),
      kPeriodMultiplier);

  time.start();
  connect(tuple_grouper.output, logger.input);
  printf("Logging started.\n\n");

  printMenu();
  std::string line;
  bool going = true;
  while (going) {
    printf(">>> ");
    std::getline(std::cin, line);
    switch (line[0]) {
    case 'n':
      forceConnect(user_grav_comp_nn.output, wam.input);
      break;
    case 'l':
      forceConnect(user_grav_comp_linear.output, wam.input);
      break;
    case 'k':
      user_grav_comp_nn.incrementGain();
      user_grav_comp_linear.incrementGain();
      std::cout << "New gain: " << user_grav_comp_linear.getGain() << std::endl;
      break;
    case 'j':
      user_grav_comp_nn.decrementGain();
      user_grav_comp_linear.decrementGain();
      std::cout << "New gain: " << user_grav_comp_linear.getGain() << std::endl;
      break;
    case 'z':
      user_grav_comp_nn.setGainZero();
      user_grav_comp_linear.setGainZero();
      disconnect(wam.input);
      break;
    case 'm':
      printMenu();
      break;
    case 'q':
      printf("Quitting.\n");
      disconnect(wam.input);
      going = false;
      break;
    default:
      if (line.size() != 0) {
        printf("Unrecognized option.\n");
        printMenu();
      }
      break;
    }
  }
  logger.closeLog();
  printf("Logging stopped.\n");
  barrett::log::Reader<tuple_type> lr(tmpFile);
  lr.exportCSV(argv[1]);
  printf("Output written to %s.\n", argv[1]);
  std::remove(tmpFile);

  product_manager.getSafetyModule()->setMode(barrett::SafetyModule::IDLE);
  product_manager.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
  return 0;
}

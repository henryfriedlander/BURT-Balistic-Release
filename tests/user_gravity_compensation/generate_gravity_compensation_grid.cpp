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

/** @file generate_regular_gravity_compensation_grid.cpp
 *
 *  Take a separation distance and handedness configuration as command line
 *  arguments, and create a corresponding GenerateGravityCompensationGrid object.
 *  Print the dimensions and contents of the grid, and write the contents to
 *  a configuration file.
 *
 *  Usage:
 *  generate_regular_gravity_compensation_grid <separation_dist> <right/left>
 */

#include <cstdio>

#include <string>

#include <proficio/tools/generate_gravity_compensation_grid.h>  // NOLINT(build/include_order)

int main(int argc, char** argv) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(3);
  if (argc < 3) {
    printf("Usage:\n\t%s <separation_dist> <right/left>\n", argv[0]);
    return 1;
  }
  const double separation_dist = atof(argv[1]);
  const std::string side = argv[2];
  printf(">>> Generating matrix with separation distance %2.3f"
         " for %s configuration.\n", separation_dist, side.c_str());

  std::string fname = "calibration_data/wam3/";
  jp_type joint_mins;
  jp_type joint_maxes;
  joint_mins << -1.01, -0.96, 0.0;
  joint_maxes << 0.56, 0.96, 0.0;

  if (!side.compare("right")) {  // if side == right
    joint_mins[2] = 0.40;
    joint_maxes[2] = 2.84;
    fname = fname + "RightConfig.txt";
  } else if (!side.compare("left")) {  // if side == left
    joint_mins[2] = -2.84;
    joint_maxes[2] = -0.40;
    fname = fname + "LeftConfig.txt";
  } else {
    printf("Usage:\n\t%s <separation_dist> <right/left>\n", argv[0]);
    return 1;
  }

  const std::string interp_filename = barrett::EtcPathRelative(fname).c_str();
  proficio::tools::GenerateGravityCompensationGrid<3> grid(joint_mins,
                                                           joint_maxes,
                                                           separation_dist,
                                                           interp_filename,
                                                           side);
  printf(">>> Grid dimensions are: %dx%dx%d\n\n",
         grid.getQ1NumPoints(), grid.getQ2NumPoints(), grid.getQ3NumPoints());
  grid.printGrid();
  if (grid.writeToFile()) {
    return 0;
  } else {
    return 1;
  }
}

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

/** @file linear_interpolation_test.cpp
 *
 *  Get an array of torque commands measured on a grid in jointspace. Test run
 *  user gravity compensation on those points.
 */

#include <cstdio>
#include <string>
#include <vector>

#include <barrett/detail/stl_utils.h>           // NOLINT(build/include_order)
#include <barrett/units.h>                      // NOLINT(build/include_order)
#include <barrett/config.h>                     // NOLINT(build/include_order)

#include <proficio/tools/linear_interpolate.h>  // NOLINT(build/include_order)

int calculateNumPoints(double min, double max, double separation_dist) {
  double range = (max + separation_dist - min);
  int num_points = range / separation_dist + 1;  // correct off by one error
  printf("min %f, max %f, sep dist %f, range %f, n %d\n",
      min, max, separation_dist, range, num_points);
  return num_points;
}

int main() {
  const int DOF = 3;
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  const jp_type joint_mins(-1.01, -0.96, -2.84);  // left config
  const jp_type joint_maxes(0.56, 0.96, -0.40);  // left config
  const double separation_dist = 1.0;
  printf("Joint 1 number of points calculation:\n");
  int q1_num_pts = calculateNumPoints(joint_mins[0], joint_maxes[0],
                                      separation_dist);
  printf("Joint 2 number of points calculation:\n");
  int q2_num_pts = calculateNumPoints(joint_mins[1], joint_maxes[1],
                                      separation_dist);
  printf("Joint 3 number of points calculation:\n");
  int q3_num_pts = calculateNumPoints(joint_mins[2], joint_maxes[2],
                                      separation_dist);

  std::vector<std::vector<std::vector<std::vector<double> > > > torque_matrix;
  std::vector<double> temp_torque(DOF, 0.0);
  std::vector<std::vector<double> > temp_q3(q3_num_pts, temp_torque);
  std::vector<std::vector<std::vector<double> > > temp_q2(q2_num_pts, temp_q3);
  torque_matrix.resize(q1_num_pts, temp_q2);

  printf("\nPosition grid:\n");
  printf("[");
  for (int i = 0; i < q1_num_pts; i++) {
    printf("[");
    for (int j = 0; j < q2_num_pts; j++) {
      printf("[");
      for (int k = 0; k < q3_num_pts; k++) {
        printf("(%1.2f %1.2f %1.2f ), ",
            joint_mins[0] + static_cast<double>(i) * separation_dist,
            joint_mins[1] + static_cast<double>(j) * separation_dist,
            joint_mins[2] + static_cast<double>(k) * separation_dist);
      }
      printf("]\n");
    }
    printf("]\n");
  }
  printf("]\n\n");

  // set test values and print torques
  for (int i = 0; i < q1_num_pts; i++) {
    for (int j = 0; j < q2_num_pts; j++) {
      for (int k = 0; k < q3_num_pts; k++) {
        jp_type curr_pos;
        curr_pos << joint_mins[0] + (separation_dist * i),
                    joint_mins[1] + (separation_dist * j),
                    joint_mins[2] + (separation_dist * k);
        jt_type calculated_torque(0.0);
        double temp = i + j + k;
        calculated_torque << temp, 2 * temp, 3 * temp;
        for (int t = 0; t < DOF; t++) {  // extract each element of the torque
          torque_matrix[i][j][k][t] = calculated_torque[t];
        }
      }
    }
  }

  proficio::tools::LinearInterpolate<3> interpolate;
  std::string handedness_name = "proficio_left";
  const std::string kConfigFilePath = std::string(getenv("HOME")) +
                                      "/.barrett/" + handedness_name +
                                      "/calibration_data/wam3/" +
                                      "user_gravity_compensation_grid.conf";
  interpolate.init(kConfigFilePath);

  printf("Test reading from config file. Torque values:\n");
  interpolate.printGrid();

  // @TODO(ab): Print expected results for these and automate the comparison.
  printf("Test interpolation points!\n\n");
  std::vector<jp_type> test_points;
  jp_type point(0.0);
  point << -1.5, -1.2, -3.0;  // below the range in all joints
  test_points.push_back(point);
  point << 1.0, 2.0, 1.0;  // above the range in all joints
  test_points.push_back(point);
  point << 0.0, -1.2, -1.0;  // below the range in joint 2
  test_points.push_back(point);
  point << 0.0, 0.0, 1.0;  // above the range in joint 3
  test_points.push_back(point);
  point << 0.0, -1.2, 1.0;  // below the range in joint 2 and above in joint 3
  test_points.push_back(point);
  point << -0.2, 0.2, -1.0;  // inside the grid range
  test_points.push_back(point);
  point << 0.2, 0.2, -2.5;  // inside the grid range
  test_points.push_back(point);
  point << -0.01, -0.96, -1.84;  // exactly on a grid point
  test_points.push_back(point);
  for (size_t i = 0; i < test_points.size(); i++) {
    point = test_points[i];
    jt_type torque = interpolate.linear_interpolation(point);
    std::cout << "Position: " << point << std::endl <<
                 "Torque: " << torque << std::endl << std::endl;
  }

  std::cout << "Press [Enter] to continue." << std::endl;
  barrett::detail::waitForEnter();

  // re-initialize to test the init() function
  interpolate.init(torque_matrix, joint_mins, joint_maxes, separation_dist);
  printf("re-initialization test successful.\n");  // b/c it didn't seg fault!

  return 0;
}

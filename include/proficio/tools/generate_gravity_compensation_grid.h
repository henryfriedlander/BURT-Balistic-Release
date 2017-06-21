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

/** @file generate_gravity_compensation_grid.h
 *
 *  Do natural neighbor interpolation on the sparsely populated grid of
 *  position:torque pairs and generate a regularly spaced grid that can be used
 *  for linear interpolation. The regularly spaced grid can be accessed via
 *  `printGrid`, `getPositionTorqueMatrix`, or the appropriate config file.
 * 
 *  The grid does not encode position, rather, it uses the following formulas
 *  to relate index and position:
 *  * `position = joint_minimum + (separation_distance * index)`
 *  * `index = (position - joint_minimum) / separation_distance`
 */

#ifndef INCLUDE_PROFICIO_TOOLS_GENERATE_GRAVITY_COMPENSATION_GRID_H_
#define INCLUDE_PROFICIO_TOOLS_GENERATE_GRAVITY_COMPENSATION_GRID_H_

#include <stdlib.h>

#include <cstdio>
#include <vector>
#include <string>

#include <barrett/units.h>   // NOLINT(build/include_order)
#include <barrett/config.h>  // NOLINT(build/include_order)

#include <libconfig.h++>

#include <proficio/tools/natural_neighbor_interpolate.h>  // NOLINT(build/include_order)

namespace proficio {
namespace tools {

template <size_t DOF>
class GenerateGravityCompensationGrid {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  /** Set const member variable values and calculate the number of points
   *  needed for each joint. Initialize the position_torque_matrix_ to the size
   *  needed to store the calculated number of points, and call
   *  generatePositionTorqueMatrix to populate it with torques.
   *
   *  @param joint_mins                 minimum positions of each joint
   *  @param joint_maxes                maximum positions of each joint
   *  @param separation_dist            distance between points in regularly
   *                                    spaced grid
   *  @param interp_filename            name of file with user gravity
   *                                    calibration data
   *  @param side                       "left" or "right" to specify handedness
   */
  explicit GenerateGravityCompensationGrid(const jp_type& joint_mins,
                                           const jp_type& joint_maxes,
                                           double separation_dist,
                                           const std::string& interp_filename,
                                           const std::string& side)
      : interpolate_(interp_filename),
        joint_mins_(joint_mins),
        joint_maxes_(joint_maxes),
        separation_dist_(separation_dist),
        q1_num_pts_(calculateNumPoints(joint_mins[0],
                                       joint_maxes[0],
                                       separation_dist)),
        q2_num_pts_(calculateNumPoints(joint_mins[1],
                                       joint_maxes[1],
                                       separation_dist)),
        q3_num_pts_(calculateNumPoints(joint_mins[2],
                                       joint_maxes[2],
                                       separation_dist)),
        side_(side) {
    std::vector<double> temp_torque(DOF, 0.0);
    std::vector<std::vector<double> > temp_q3(q3_num_pts_, temp_torque);
    std::vector<std::vector<std::vector<double> > > temp_q2(q2_num_pts_,
                                                            temp_q3);
    position_torque_matrix_.resize(q1_num_pts_, temp_q2);
    generatePositionTorqueMatrix();
  }

  /** Given a position, calculate the torque by calling natural neighbor
   *  interpolation.
   *
   *  @param position     the position at which to calculate torque
   *  @param torque_out   a pointer to a jt_type variable in which to
   *                      store the calculated torque value.
   */
  bool getTorque(const jp_type& position, jt_type* torque_out) {
    jt_type torque;
    bool success =
        interpolate_.natural_neighbor_interpolation(position, &torque);
    *torque_out = torque;
    return success;
  }

  /** Generate a matrix of positions in the given range that are
   *  "separation_dist" apart. Calculate the torque at each position
   *  and set the corresponding matrix element to the calculated torque
   *
   *  The position corresponding to an index is:
   *  `joint_min + (separation_dist * index)`
   *
   *  We need to cover at least the possible range, so the positions may include
   *  values up to just under `joint_max + separation_dist`
   */
  void generatePositionTorqueMatrix() {
    for (int i = 0; i < q1_num_pts_; i++) {
      for (int j = 0; j < q2_num_pts_; j++) {
        for (int k = 0; k < q3_num_pts_; k++) {
          jp_type position;
          position << joint_mins_[0] + (separation_dist_ * i),
              joint_mins_[1] + (separation_dist_ * j),
              joint_mins_[2] + (separation_dist_ * k);
          jt_type calculated_torque;
          bool res = getTorque(position, &calculated_torque);
          for (size_t t = 0; t < DOF;
               t++) {  // extract each element of the torque
            position_torque_matrix_[i][j][k][t] = calculated_torque[t];
          }
        }
      }
    }
  }

  bool writeToFile() {
    std::string handedness_name = "";
    if (!side_.compare("right")) {  // if side == right
      handedness_name = "proficio_right";
    } else if (!side_.compare("left")) {  // if side == left
      handedness_name = "proficio_left";
    } else {
      printf("Error: Invalid handedness\n");
      return false;
    }

    const std::string kConfigFilePath = std::string(getenv("HOME")) +
                                        "/.barrett/" + handedness_name +
                                        "/calibration_data/wam3/" +
                                        "user_gravity_compensation_grid.conf";
    const char* kOutputFile = kConfigFilePath.c_str();
    libconfig::Config data_config;
    libconfig::Setting& setting_group = data_config.getRoot()
        .add("user_gravity_compensation", libconfig::Setting::TypeGroup);
    libconfig::Setting& separation_dist_setting = setting_group
        .add("separation_distance", libconfig::Setting::TypeFloat) =
          separation_dist_;
    libconfig::Setting& joint_mins_setting = setting_group
        .add("joint_mins", libconfig::Setting::TypeList);
    libconfig::Setting& joint_maxes_setting = setting_group
        .add("joint_maxes", libconfig::Setting::TypeList);
    for (size_t i = 0; i < DOF; i++) {
      joint_mins_setting.add(libconfig::Setting::TypeFloat) = joint_mins_[i];
    }
    for (size_t i = 0; i < DOF; i++) {
      joint_maxes_setting.add(libconfig::Setting::TypeFloat) = joint_maxes_[i];
    }

    libconfig::Setting& q1 = setting_group
        .add("position_torque_grid", libconfig::Setting::TypeList);
    for (int i = 0; i < q1_num_pts_; i++) {
      libconfig::Setting& q2 = q1.add(libconfig::Setting::TypeList);
      for (int j = 0; j < q2_num_pts_; j++) {
      libconfig::Setting& q3 = q2.add(libconfig::Setting::TypeList);
        for (int k = 0; k < q3_num_pts_; k++) {
        libconfig::Setting& torque = q3.add(libconfig::Setting::TypeList);
          for (int t = 0; t < 3; t++) {
            torque.add(libconfig::Setting::TypeFloat) =
              position_torque_matrix_[i][j][k][t];
          }
        }
      }
    }
    data_config.writeFile(kOutputFile);
    printf(">>> Data written to: %s\n", kOutputFile);
    return true;
  }

  void printGrid() {
    printf("[");
    for (int i = 0; i < q1_num_pts_; i++) {
      printf("[");
      for (int j = 0; j < q2_num_pts_; j++) {
        printf("[");
        for (int k = 0; k < q3_num_pts_; k++) {
          printf("(");
          for (int t = 0; t < 3; t++) {
            printf("%1.2f ", position_torque_matrix_[i][j][k][t]);
          }
          printf("), ");
        }
        printf("]\n");
      }
      printf("]\n");
    }
    printf("]\n\n");
  }

  int getQ1NumPoints() { return q1_num_pts_; }

  int getQ2NumPoints() { return q2_num_pts_; }

  int getQ3NumPoints() { return q3_num_pts_; }

  std::vector<std::vector<std::vector<std::vector<double> > > >
      getPositionTorqueMatrix() {
    return position_torque_matrix_;
  }

 protected:
  /** Calculate the number of points separation_dist apart needed to
   * entirely cover the range between min and max.
   *
   *  The points need to entirely cover the range and can go over. They may go
   *  up to, but not including, max + separation_dist.
   *
   *  @param min                the minumum required value of a point
   *  @param max                the maximum required value of a point
   *  @param separation_dist    distance between points
   */
  int calculateNumPoints(double min, double max, double separation_dist) {
    double range = (max + separation_dist - min);
    int num_points = range / separation_dist + 1;  // correct off by one error
    return num_points;
  }

 private:
  proficio::tools::NaturalNeighborInterpolate<DOF> interpolate_;
  const jp_type joint_mins_;
  const jp_type joint_maxes_;
  const double separation_dist_;
  const int q1_num_pts_;
  const int q2_num_pts_;
  const int q3_num_pts_;
  std::vector<std::vector<std::vector<std::vector<double> > > >
      position_torque_matrix_;  ///< 3D grid of joint torques
  const std::string side_;
};
}  // namespace tools
}  // namespace proficio
#endif  // INCLUDE_PROFICIO_TOOLS_GENERATE_GRAVITY_COMPENSATION_GRID_H_

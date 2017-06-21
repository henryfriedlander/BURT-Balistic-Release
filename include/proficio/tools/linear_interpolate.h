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

/** @file linear_interpolate.h
 *
 *  Interpolate necessary torque for a given position using data from a config
 *  file with a regular grid of torque:position pairs.
 *  
 *  Position and index are related using the following formulas:  
 *  * `position = joint_minimum + (separation_distance * index)`
 *  * `index = (position - joint_minimum) / separation_distance`
 */

#ifndef PROFICIO_TOOLS_LINEAR_INTERPOLATE_H_
#define PROFICIO_TOOLS_LINEAR_INTERPOLATE_H_

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <barrett/units.h>  // NOLINT(build/include_order)

namespace proficio {
namespace tools {

template <size_t DOF>
class LinearInterpolate {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  LinearInterpolate() : initialized_(false) {}

  /** Reads a libconfig file with separation distance, joint mins, joint maxes,
   *  and position_torque_grid, all in a user_gravity_compensation group. 
   *
   *  Can be called more than once to allow changing the config file on the fly.
   *
   *  @param kConfigFilePath     the filepath to the libconfig file
   */
  bool init(const std::string& kConfigFilePath) {
    torque_matrix_.clear();
    libconfig::Config config;
    config.readFile(kConfigFilePath.c_str());
    printf(">>> Reading in file from %s\n", kConfigFilePath.c_str());

    libconfig::Setting& separation_dist_setting = config.lookup(
        "user_gravity_compensation.separation_distance");
    separation_dist_ = separation_dist_setting;
    libconfig::Setting& joint_mins_setting = config.lookup(
        "user_gravity_compensation.joint_mins");
    joint_mins_ = joint_mins_setting;
    libconfig::Setting& joint_maxes_setting = config.lookup(
        "user_gravity_compensation.joint_maxes");
    joint_maxes_ = joint_maxes_setting;

    for (size_t index = 0; index < DOF; index++) {
      num_pts_[index] = calculateNumPoints(joint_mins_[index],
                                           joint_maxes_[index],
                                           separation_dist_);
    }

    libconfig::Setting& torque_matrix_setting = config.lookup(
        "user_gravity_compensation.position_torque_grid");
    std::vector<double> temp_torque(3, 0.0);
    std::vector<std::vector<double> > temp_q3(num_pts_[2], temp_torque);
    std::vector<std::vector<std::vector<double> > > temp_q2(num_pts_[1],
                                                            temp_q3);
    torque_matrix_.resize(num_pts_[0], temp_q2);
    for (int i = 0; i < num_pts_[0]; i++) {
      for (int j = 0; j < num_pts_[1]; j++) {
        for (int k = 0; k < num_pts_[2]; k++) {
          for (int t = 0; t < 3; t++) {
            torque_matrix_[i][j][k][t] = torque_matrix_setting[i][j][k][t];
          }
        }
      }
    }
    initialized_ = true;
    return initialized_;
  }

  /** Initializes from a torque array, which presumably was developed from a
   *  config file read elsewhere. Can be called more than once to allow changing
   *  the config on the fly.
   */
  bool init(const std::vector<std::vector<std::vector<std::vector<double> > > >&
              torque_matrix,
            const jp_type& joint_mins,
            const jp_type& joint_maxes,
            const double separation_dist ) {
    joint_mins_ = joint_mins;
    joint_maxes_ = joint_maxes;
    separation_dist_ = separation_dist;
    torque_matrix_.clear();
    torque_matrix_ = torque_matrix;
    for (size_t index = 0; index < DOF; index++) {
      num_pts_[index] = calculateNumPoints(joint_mins[index],
                                           joint_maxes[index],
                                           separation_dist);
    }
    initialized_ = true;
    return initialized_;
  }

  /** Performs linear interpolation on the 3D grid. If the point is outside the
   *  range of the grid, uses the point on the edge.
   *
   *  Starts by finding the eight grid points surrounding the position, or using
   *  the min or max calibrated points if the position is outside the calibrated
   *  range.
   */
  jt_type linear_interpolation(const jp_type& position) {
    jt_type torque(0.0);
    if (initialized_) {
      int indices[DOF][2];  // indices of the surrounding grid points
      for (size_t joint_num = 0; joint_num < DOF; joint_num++) {
        if (position[joint_num] < joint_mins_[joint_num]) {  // below calibrated
                                                             // range
          indices[joint_num][0] = 0;
          indices[joint_num][1] = 0;
        } else if (position[joint_num] >
            (joint_mins_[joint_num] + (num_pts_[joint_num] - 1) *
            separation_dist_)) {  // above calibrated range
          indices[joint_num][0] = num_pts_[joint_num] - 1;
          indices[joint_num][1] = num_pts_[joint_num] - 1;
        } else {
          indices[joint_num][0] = (position[joint_num] - joint_mins_[joint_num])
                                / separation_dist_;
          indices[joint_num][1] = indices[joint_num][0] + 1;
        }
      }

      std::vector<std::vector<std::vector<std::vector<double> > > > temp;
      temp.resize(2);
      for (int i = 0; i < 2; i++) {
        temp[i].resize(2);
        for (int j = 0; j < 2; j++) {
          temp[i][j].resize(2);
          for (int k = 0; k < 2; k++) {
            temp[i][j][k].resize(3, 0.0);
            for (size_t t = 0; t < DOF; t++) {
              temp[i][j][k][t] = torque_matrix_[indices[0][i]] [indices[1][j]]
                                               [indices[2][k]] [t];
            }
          }
        }
      }

      // interpolate in q3 - eight points to four points
      std::vector<std::vector<std::vector<double> > > temp2;
      temp2.resize(2);
      for (int i = 0; i < 2; i++) {
        temp2[i].resize(2);
        for (int j = 0; j < 2; j++) {
          temp2[i][j].resize(3, 0.0);
          for (size_t t = 0; t < DOF; t++) {
            if (indices[2][0] == indices[2][1]) {  // outside q3 range, both
                                                   // points are same
              temp2[i][j][t] = temp[i][j][0][t];
            } else {  // interpolate!
              double temp_lower_pos = joint_mins_[2] +
                                      separation_dist_ * indices[2][0];
              temp2[i][j][t] = (temp[i][j][1][t] - temp[i][j][0][t]) /
                               separation_dist_ *
                               (position[2] - temp_lower_pos) +
                               temp[i][j][0][t];
            }
          }
        }
      }

      // interpolate in q2 - four points to two points
      std::vector<std::vector<double> > temp3;
      temp3.resize(2);
      for (int i = 0; i < 2; i++) {
        temp3[i].resize(3, 0.0);
        for (size_t t = 0; t < DOF; t++) {
          if (indices[1][0] == indices[1][1]) {  // outside q2 range, both
                                                 // points are same
            temp3[i][t] = temp2[i][0][t];
          } else {  // interpolate!
            double temp_lower_pos = joint_mins_[1] +
                                    separation_dist_ * indices[1][0];
            temp3[i][t] = (temp2[i][1][t] - temp2[i][0][t]) /
                          separation_dist_ *
                          (position[1] - temp_lower_pos) +
                          temp2[i][0][t];
          }
        }
      }

      // interpolate in q1 - two points to one point
      for (size_t t = 0; t < DOF; t++) {
        if (indices[0][0] == indices[0][1]) {  // outside q1 range, both points
                                               // are same
          torque[t] = temp3[0][t];
        } else {  // interpolate!
          double temp_lower_pos = joint_mins_[0] +
                                  separation_dist_ * indices[0][0];
          torque[t] = (temp3[1][t] - temp3[0][t]) /
                      separation_dist_ *
                      (position[0] - temp_lower_pos) +
                      temp3[0][t];
        }
      }
    } else {  // not initialized
      torque.setZero();
    }
    return torque;
  }

  void printGrid() {
    printf("[");
    for (int i = 0; i < num_pts_[0]; i++) {
      printf("[");
      for (int j = 0; j < num_pts_[1]; j++) {
        printf("[");
        for (int k = 0; k < num_pts_[2]; k++) {
          printf("(");
          for (int t = 0; t < 3; t++) {
            printf("%1.2f ", torque_matrix_[i][j][k][t]);
          }
          printf("), ");
        }
        printf("]\n");
      }
      printf("]\n");
    }
    printf("]\n\n");
  }

 protected:
  /** Calculate the number of points separation_dist apart needed to
   *  entirely cover the range between min and max.
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
  bool initialized_;  ///< Flag indicating whether config file has been read
  jp_type joint_mins_;
  jp_type joint_maxes_;
  double separation_dist_;  ///< Distance between grid points
  int num_pts_[3];
  std::vector<std::vector<std::vector<std::vector<double> > > >
    torque_matrix_;  ///< Torque values from config file
};
}  // namespace tools
}  // namespace proficio
#endif  // PROFICIO_TOOLS_LINEAR_INTERPOLATE_H_

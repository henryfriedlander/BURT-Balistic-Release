/*
 * Copyright 2016 Barrett Technology <support@barrett.com>
 *
 * This file is part of proficio_toolbox.
 *
 * This version of proficio_toolbox is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This version of proficio_toolbox is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General 
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this version of proficio_toolbox. If not, see <http://www.gnu.org/licenses/>.
 */

/**@file forward_kinematics.h
 *
 * A class that evaluates the forward kinematics of the robot and provides
 * functions to return either the entire transformation matrix or just the
 * Cartesian position of the endpoint.
 *
 * Parameters:
 *    kin       A libbarrett kinematics structure of type bt_kinematics
 *    world     A pointer to a config_setting_t (type defined in libconfig.h)
 */

#include <math.h>
#include <vector>

#include <barrett/cdlbt/calgrav.h>
#include <barrett/cdlbt/gsl.h>
#include <barrett/cdlbt/kinematics.h>
#include <barrett/config.h>
#include <barrett/units.h>

#ifndef PROFICIO_TOOLS_FORWARD_KINEMATICS_H_
#define PROFICIO_TOOLS_FORWARD_KINEMATICS_H_

namespace proficio {

/** Returns the forward kinematics of the robot.
 * 
 *  Can be used to get the entire transformation matrix or just the translation
 *  vector.
 *
 *  Example:
 *    ForwardKinematics<DOF> forward_kinematics(kin, world);
 *    jp_type q = wam.getJointPositions();
 *    cp_type position = forward_kinematics.findFwKinPosition(q);
 *    Eigen::Matrix4f transform;
 *    // findFwKinMatrix stores the result in transform
 *    forward_kinematics.findFwKinMatrix(q, *transform);
 */
template<size_t DOF>
class ForwardKinematics {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  ForwardKinematics(const struct bt_kinematics& kin, config_setting_t * world) :
      kin_(kin), world_(world) {
    // Populate world to base transform
    config_setting_t * world_row;
    for (int i = 0; i < 4; ++i) {
      world_row = config_setting_get_elem(world_, i);
      for (int j = 0; j < 4; ++j) {
        double val;
        // Get the i,j element of world to base transform
        bt_gsl_config_get_double(config_setting_get_elem(world_row, j), &val);
        transform_base_to_world_(i, j) = val;
      }
    }
  }

  /** Returns the 4x4 transformation matrix that describes the position and
   * orientation of the tool (robot endpoint) relative to the world frame.
   * @param ang             input vector of joint positions
   */
  Eigen::Matrix4f findFwKinMatrix(const jp_type& ang) {
    std::vector<Eigen::Matrix4f> all_transforms;
    all_transforms.resize(DOF);
    Eigen::Matrix4f transform_result = Eigen::MatrixXf::Identity(4, 4);

    // Construct transformation matrices for each frame
    for (size_t i = 0; i < DOF; ++i) {
      all_transforms[i] << cos(ang[i]), -sin(ang[i]) * cos(kin_.link[i]->alpha),  sin(ang[i]) * sin(kin_.link[i]->alpha), kin_.link[i]->a * cos(ang[i]),  // NOLINT(whitespace/line_length) - matrix
                           sin(ang[i]),  cos(ang[i]) * cos(kin_.link[i]->alpha), -cos(ang[i]) * sin(kin_.link[i]->alpha), kin_.link[i]->a * sin(ang[i]),  // NOLINT(whitespace/line_length)
                           0,            sin(kin_.link[i]->alpha),                cos(kin_.link[i]->alpha),               kin_.link[i]->d,                // NOLINT(whitespace/line_length)
                           0,            0,                                       0,                                      1;                              // NOLINT(whitespace/line_length)
    }

    // Homogeneous transformation from Frame 0 to Frame n
    for (size_t i = 0; i < DOF; ++i) {
      transform_result = transform_result * all_transforms[i];
    }

    // Transform the coordinates to the base frame
    transform_result = transform_base_to_world_ * transform_result;

    return transform_result;
  }

  /** Returns the position of the tool (robot endpoint) relative to the world
   * frame.
   * @param ang             input vector of joint positions
   */
  cp_type findFwKinPosition(const jp_type& ang) {
    Eigen::Matrix4f transform_matrix = findFwKinMatrix(ang);
    cp_type cart_position(transform_matrix(0, 3),
                          transform_matrix(1, 3),
                          transform_matrix(2, 3));
    return cart_position;
  }

 protected:
  struct bt_kinematics kin_;
  config_setting_t * world_;
  Eigen::Matrix4f transform_base_to_world_;
};
}  // namespace proficio

#endif  // PROFICIO_TOOLS_FORWARD_KINEMATICS_H_

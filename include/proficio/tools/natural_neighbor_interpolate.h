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

/** @file natural_neighbor_interpolate.h
 *
 *  Interpolate necessary torque for a given position using data from a config
 *  file with a sparesely populated matrix of measured torque:position pairs.
 */

#ifndef PROFICIO_TOOLS_NATURAL_NEIGHBOR_INTERPOLATE_H_
#define PROFICIO_TOOLS_NATURAL_NEIGHBOR_INTERPOLATE_H_

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <barrett/units.h>  // NOLINT(build/include_order)

#include <CGAL/Delaunay_triangulation_2.h>        // NOLINT(build/include_order)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>  // NOLINT(build/include_order)
#include <CGAL/natural_neighbor_coordinates_2.h>  // NOLINT(build/include_order)

namespace proficio {
namespace tools {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay_triangulation;
typedef std::vector<std::pair<K::Point_2, K::FT> > Point_coordinate_vector;

template <size_t DOF>
class NaturalNeighborInterpolate {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  /** Store calibration file data in member variables and create a 
   *  Delaunay triangulation from the calibration data.
   */
  explicit NaturalNeighborInterpolate(const std::string& config_file_name) {
    if (readConfig(&calibration_joint_positions_, &calibration_torques_,
                   config_file_name)) {
    } else {
      std::cout << "Config file read failed.\n\n";
    }
    for (int i = 1; i < 5; i++) {
      calibration_center_ += calibration_joint_positions_[i] / 4;
    }

    // create a Delaunay triangulation from the calibration data
    for (size_t i = 0; i < calibration_joint_positions_.size(); i++) {
      delaunay_triangulation_.insert(K::Point_2(
          calibration_joint_positions_[i][0], calibration_joint_positions_[i][1]));
    }
  }

  /** Does natural neighbor interpolation with the following procedure:
   *
   *  (1)  Check whether point is inside or outside convex hull.
   *       Project the point onto each segment of the hull. Then check whether
   *       the point is inside or outside by multiplying by the normal pointing
   *       into the hull. If the point is outside for any segment, then it is
   *       outside the hull. Save distances between point and projections.
   *
   *  (2)  If the point is outside for any segment, check whether its
   *       projection is on the segment. If yes, then the projection is the
   *       closest point on the hull. Perturb the point towards the hull center
   *       and calculate the nn coordinates. The perturbation prevents trying to
   *       calculate nn coordinates for a point too close to the edge, which
   *       takes a long time and sometimes results in inf or nan weights.
   *
   *   (2a)  Use natural neighbor (nn) coordinates to complete the
   *         interpolation. The nn coordinates stored in coords.second are the 
   *         weights to use for the corresponding points in coords.first.
   *         Those weights need to be normalized by the sum of weights stored in
   *         result.second.
   *         Since the output in coords is not in the same order as the original
   *         list of points, search the list for the points in coords, in order,
   *         and pull out the appropriate torque to multiply by the listed nn
   *         coordinate.
   *
   *  (3)  If the point is inside the hull, make sure it is some minimum
   *       distance from all segments. If not, perturb it a small amount towards
   *       the hull center, again to prevent long calculation times and inf/nan
   *       weights. Then calculate the nn coordinates.
   *
   *   (3a)  Use nn coordinates to complete the interpolation as in (2a).
   *
   *  (4)  If the point is not found in an edge section, then it must be in a
   *       corner section. Calculate the distance to each corner. The closest
   *       corner is the closest point on the surface of the hull. Use the
   *       calibration torque from that point.
   *
   *
   *  Assumptions:
   *
   *  (1)  Points were collected in the specified order (qi = joint angle i):
   *         1: start position (approx 0.25, 0.00, 1.57)
   *         2: q1 low, q2 low (arm up, out to side, towards user)
   *         3: q1 low, q2 high (arm up, in front of user)
   *         4: q1 high, q2 high (arm down, in front of user, away)
   *         5: q1 high, q2 low (arm down, out to side)
   *         6-20: whatever you want, or empty
   *
   *  (2)  Points are at least 0.003 apart.
   *
   *  (3)  Start point is set at (0.2647750, -0.0058371, 1.5773100), or at
   *       least 0.002 away from (0,0).
   *
   *  @param position       position to interpolate for
   *  @param torque_out     calculated necessary torque for position
   */
  bool natural_neighbor_interpolation(const jp_type& position,
                                      jt_type* torque_out) {
    const K::Point_2 p(position[0], position[1]);
    const K::Vector_2 p_vec(p[0], p[1]);
    jt_type torque;  // empty vector for torque computation
    torque.setZero();

    // coordinate computation
    const K::Point_2 center(calibration_center_[0], calibration_center_[1]);
    bool point_found = false;
    bool outside = false;
    bool near_edge = false;
    bool success = true;

    // Check if point is inside or outside convex hull created by calibration
    // points.
    for (int seg = 1; seg < 5; seg++) {
      // define the line segment
      K::Point_2 p1(calibration_joint_positions_[seg][0],
                    calibration_joint_positions_[seg][1]);
      double p2x, p2y;
      if (seg < 4) {
        p2x = calibration_joint_positions_[seg + 1][0];
        p2y = calibration_joint_positions_[seg + 1][1];
      } else {
        p2x = calibration_joint_positions_[1][0];
        p2y = calibration_joint_positions_[1][1];
      }
      K::Point_2 p2(p2x, p2y);

      // get a unit vector in the direction of the line segment
      K::Vector_2 segment(p1, p2);
      segment = segment / sqrt(segment.squared_length());
      // project the point of interest onto that vector
      K::Point_2 proj = p1 - ((p1 - p) * segment) * segment;
      // find the normal pointing into the hull
      K::Vector_2 normal(segment[1], -segment[0]);
      // check whether point is outside the hull on this side
      if (normal * (proj - p) > 0) {
        // point is outside this segment. check if the projection is on the
        // segment.
        outside = true;
        if ((proj[0] >= std::min(p1[0], p2[0])) &&
            (proj[0] <= std::max(p1[0], p2[0])) &&
            (proj[1] >= std::min(p1[1], p2[1])) &&
            (proj[1] <= std::max(p1[1], p2[1]))) {
          // This is the section containing the point.
          // Perturb the projection towards center.
          K::Vector_2 to_center(proj, center);
          proj = proj + 0.000001 * to_center;
          // Get the nn coordinates
          Point_coordinate_vector coords;
          CGAL::Triple<std::back_insert_iterator<Point_coordinate_vector>,
                       K::FT, bool> result =
              CGAL::natural_neighbor_coordinates_2(
                  delaunay_triangulation_, proj, std::back_inserter(coords));
          // check success
          if (result.third) {
            // Calculate the torque
            for (size_t i = 0; i < coords.size(); i++) {
              int index = findPoint(coords[i].first, calibration_joint_positions_,
                                    calibration_joint_positions_.size());
              if (index < 0) {
                index = calibration_torques_.size();  // zero torques
              }
              for (int j = 0; j < 3; j++) {
                torque[j] += coords[i].second * calibration_torques_[index][j] /
                             result.second;
              }
            }
          } else {
            // Failed. Use torque from previous timestep.
            success = false;
            //TODO make this warning more helpful
            std::cout << "WARNING: NN coordinate calculation failed.\n";
          }
          // don't bother checking the other edge sections
          point_found = true;
          break;
        }
      } else {
        // point is inside this segment. check distance to edge.
        if (sqrt((proj - p).squared_length()) < 0.000001) {
          near_edge = true;
        }
      }
    }
    if (!outside) {  // point is inside the convex hull.
      // check distances to edges.
      K::Point_2 p_new(p);
      if (near_edge) {
        // perturb towards center
        K::Vector_2 to_center(p, center);
        p_new = p + 0.000001 * to_center;
      }
      // Get the natural neighbor coordinates
      Point_coordinate_vector coords;
      CGAL::Triple<std::back_insert_iterator<Point_coordinate_vector>, K::FT,
                   bool> result =
          CGAL::natural_neighbor_coordinates_2(delaunay_triangulation_, p_new,
                                               std::back_inserter(coords));
      // check success
      if (result.third) {
        // Calculate the torque
        for (size_t i = 0; i < coords.size(); i++) {
          int index = findPoint(coords[i].first, calibration_joint_positions_,
                                calibration_joint_positions_.size());
          if (index < 0) {
            index = calibration_torques_.size();  // zero torques
          }
          for (int j = 0; j < 3; j++) {
            torque[j] += coords[i].second * calibration_torques_[index][j] /
                         result.second;
          }
        }
      } else {
        // Failed. Use torque from previous timestep.
        success = false;
        //TODO make this warning more helpful
        std::cout << "WARNING: NN coordinate calculation failed.\n";
      }
      point_found = true;
    } else if (!point_found) {
      // point is outside the convex hull but not in an edge section.
      // point must be in a corner section.
      // find the closest corner point and use that.
      int min_i = 1;
      double min_dist = sqrt(pow(p[0] - calibration_joint_positions_[1][0], 2) +
                             pow(p[1] - calibration_joint_positions_[1][1], 2));
      double dist;
      for (int i = 2; i < 5; i++) {
        dist = sqrt(pow(p[0] - calibration_joint_positions_[i][0], 2) +
                    pow(p[1] - calibration_joint_positions_[i][1], 2));
        if (dist < min_dist) {
          min_dist = dist;
          min_i = i;
        }
      }
      for (int i = 0; i < 3; i++) {
        torque[i] = calibration_torques_[min_i][i];
      }
    }
    if (success) {
      *torque_out = torque;
    }
    return success;
  }

 protected:
  Delaunay_triangulation delaunay_triangulation_;
  std::vector<jt_type> calibration_torques_;
  jp_type calibration_center_;
  std::vector<jp_type> calibration_joint_positions_;

  /** Read in data from a configuration file of joint positions and torques.
   *  Store the data in calibration_joint_positions_ and calibration_torques_ 
   *
   *  @param calibration_joint_positions_     store joint positions
   *  @param calibration_torques_             store torques
   *  @param fname                            name of the file
   */
  bool readConfig(std::vector<jp_type>* calibration_joint_positions_,
                  std::vector<jt_type>* calibration_torques_,
                  const std::string& fname) {
    std::ifstream myfile(fname.c_str());
    std::string line;
    size_t i = 0;
    size_t size = 20;  // max number of points to read
    jp_type data_joints;
    data_joints.setZero();
    jt_type data_torques;
    data_torques.setZero();
    if (myfile.is_open()) {
      while (getline(myfile, line) && i < size) {
        if (!parseDoubles(&data_joints, &data_torques, line)) {
          std::cout << "WARNING: Calibration data parsing failed. " << 
            line << std::endl;
          return false;
        }
        calibration_joint_positions_->push_back(data_joints);
        calibration_torques_->push_back(data_torques);
        i++;
      }
      myfile.close();
    }
    jt_type data_zero;
    data_zero.setZero();
    calibration_torques_->push_back(data_zero);
    return true;
  }

  /** Searches point_list for point and returns the index when found. */
  int findPoint(const K::Point_2& point, const std::vector<jp_type>& point_list,
                const size_t length, double min_separation = 0.001) {
    for (size_t i = 0; i < length; i++) {
      if (fabs(point[0] - point_list[i][0]) < min_separation &&
          fabs(point[1] - point_list[i][1]) < min_separation) {
        return i;
      }
    }
    return -1;  // none found
  }

  /** Given a string of numbers separated by spaces, returns two 3-vectors of
   *  doubles. The first vector is joint positions, the second is joint torques.
   *
   *  Based on parseDoubles function from ex03.
   */
  bool parseDoubles(jp_type* x, jt_type* y, const std::string& str) {
    const char* cur = str.c_str();
    const char* next = cur;

    for (int i = 0; i < 3; ++i) {
      (*x)[i] = strtod(cur, (char**)&next);  // convert string to double
      if (cur == next) {
        return false;
      } else {
        cur = next;
      }
    }

    for (int i = 0; i < 3; ++i) {
      (*y)[i] = strtod(cur, (char**)&next);
      if (cur == next) {
        return false;
      } else {
        cur = next;
      }
    }

    // Make sure there are no extra numbers in the string.
    double ignore = strtod(cur, (char**)&next);
    (void)ignore;  // Prevent unused variable warnings

    if (cur != next) {
      return false;
    }
    return true;
  }
};
}  // namespace tools
}  // namespace proficio
#endif  // PROFICIO_TOOLS_NATURAL_NEIGHBOR_INTERPOLATE_H_

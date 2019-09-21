/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_BOIDS_PLUGINS_AUTONOMY_DEFENDERS_DEFENDERS_H_
#define INCLUDE_BOIDS_PLUGINS_AUTONOMY_DEFENDERS_DEFENDERS_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class Defenders : public scrimmage::Autonomy {
public:
   void init(std::map<std::string, std::string> &params) override;
   bool step_autonomy(double t, double dt) override;

protected:
  void animate_FOV(double sense_range,double fov_width_angle, double fov_height_angle);
  void show_FOV();
  void show_attack_FOV();
  std::vector<ID> get_neighbors_in_range(double range);
  void update_goal();
  void update_desired_state(Eigen::Vector3d &v);
  void check_attack(double t, double dt);
   int follow_id_ = -1;
   double initial_speed_ = 0;
   double max_speed_;
  double attack_sense_range_ = 30;
  double attack_duration_ = 3;
  double attack_time_ = 0;
  double attack_end_time_ = 0;
  bool pre_in_view_ = false; // true if the enemy previously followed still in FOV
  double sense_range_;
  double fov_width_angle_;
  double fov_height_angle_;
  bool attacking_ = false;
  scrimmage::PublisherPtr killed_ids_ad;
  //weights
   double w_align_;
   double w_avoid_team_;
   double w_centroid_;
   double w_avoid_nonteam_;
   double w_goal_;


   double minimum_team_range_;
   double minimum_nonteam_range_;
   double sphere_of_influence_;
   Eigen::Vector3d goal_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_BOIDS_PLUGINS_AUTONOMY_DEFENDERS_DEFENDERS_H_

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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MBOID_MBOID_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MBOID_MBOID_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <string>
#include <map>
#include <vector>

namespace scrimmage {
namespace autonomy {
class MBoid: public scrimmage::Autonomy {
  public:
    virtual void init(std::map<std::string, std::string> &params) override;
    virtual bool step_autonomy(double t, double dt) override;
 protected:
   void show_FOV(double sense_range,double fov_width_angle, double fov_height_angle);
   virtual void get_neighbors_in_range(double range,std::vector<ID> &neighbors);
   virtual Eigen::Vector3d step_goal() = 0;
   void update_desired_state(Eigen::Vector3d &v);
   double initial_speed_ = 0;
   double max_speed_;
   double sense_range_;
   double fov_width_angle_;
   double fov_height_angle_;
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
#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_MBOID_MBOID_H_

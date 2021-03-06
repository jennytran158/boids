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
#include <boids/plugins/autonomy/MBoid/MBoid.h>


namespace scrimmage {
namespace autonomy {
class Defenders : public MBoid {
  public:
    virtual void init(std::map<std::string, std::string> &params) override;
    virtual bool step_autonomy(double t, double dt) override;
  protected:
    virtual Eigen::Vector3d step_goal() override;
    virtual void get_neighbors_in_range(double range,std::vector<ID> &neighbors) override;
    void show_attack_FOV();
    void step_attack(double t, double dt);
    int follow_id_ = -1;
    double attack_sense_range_ = 30;
    double attack_duration_ = 3;
    double attack_time_ = 0;
    double attack_end_time_ = 0;
    bool attacking_ = false;
    scrimmage::PublisherPtr killed_ids_ad;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_BOIDS_PLUGINS_AUTONOMY_DEFENDERS_DEFENDERS_H_

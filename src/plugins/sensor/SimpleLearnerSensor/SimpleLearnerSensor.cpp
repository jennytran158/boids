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

#include <boids/plugins/sensor/SimpleLearnerSensor/SimpleLearnerSensor.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/Quaternion.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Sensor,
                scrimmage::sensor::SimpleLearnerSensor,
                SimpleLearnerSensor_plugin)

namespace scrimmage {
namespace sensor {

void SimpleLearnerSensor::get_observation(double *data, uint32_t beg_idx, uint32_t /*end_idx*/) {
    double sense_range = 200;
    auto pos = parent_->state()->pos();
    Eigen::Vector3d v_goal = (Eigen::Vector3d(0,0,100) - pos);
    auto contacts = parent_->contacts();
    Eigen::Vector3d v_non_team = Eigen::Vector3d(0,0,0);
    for (auto it = contacts->begin();
          it != contacts->end(); it++) {
          auto contact = (*it).second;
          if (contact.id().id() != parent_->id().id()) {
               v_non_team += (contact.state()->pos() - pos);
          }
     }
    data[beg_idx] = parent_->state()->pos()(0);
    data[beg_idx + 1] = parent_->state()->pos()(1);
    data[beg_idx + 2] = parent_->state()->pos()(2);
    if (v_goal.norm() > sense_range) {
      v_goal = v_goal.normalized() * sense_range;
    }
    data[beg_idx + 3] = v_goal(0);
    data[beg_idx + 4] = v_goal(1);
    data[beg_idx + 5] = v_goal(2);

    // data[beg_idx + 6] = v_non_team(0);
    // data[beg_idx + 7] = v_non_team(1);
    // data[beg_idx + 8] = v_non_team(2);
}

void SimpleLearnerSensor::set_observation_space() {
    const double inf = std::numeric_limits<double>::infinity();
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    // observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    // observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
    // observation_space.continuous_extrema.push_back(std::make_pair(-inf, inf));
}
} // namespace sensor
} // namespace scrimmage

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

#include <boids/plugins/autonomy/SimpleLearner/SimpleLearner.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::SimpleLearner,
                SimpleLearner_plugin)

namespace scrimmage {
namespace autonomy {

void SimpleLearner::init_helper(std::map<std::string, std::string> &params) {
    using Type = scrimmage::VariableIO::Type;
    using Dir = scrimmage::VariableIO::Direction;

    output_vel_x_idx_ = vars_.declare(Type::velocity_x, Dir::Out);
    output_vel_y_idx_ = vars_.declare(Type::velocity_y, Dir::Out);
    output_vel_z_idx_ = vars_.declare(Type::velocity_z, Dir::Out);

    vars_.output(output_vel_x_idx_, 0);
    vars_.output(output_vel_y_idx_, 0);
    vars_.output(output_vel_z_idx_, 0);
    radius_ = std::stod(params.at("radius"));
    auto callback = [&] (scrimmage::MessagePtr<int> msg) {
      dead = true;
    };
    subscribe<int>("GlobalNetwork", "Killed_IDs", callback);
}
void SimpleLearner::set_environment() {
    reward_range = std::make_pair(-50, 100);
    // action_space.discrete_count.push_back(2);
    action_space.continuous_extrema.push_back(std::make_pair(-30,30));//goal
    // action_space.continuous_extrema.push_back(std::make_pair(-1,1));// enemy
    // action_space.continuous_extrema.push_back(std::make_pair(-1,1));
    // action_space.continuous_extrema.push_back(std::make_pair(-1,1));
    // action_space.continuous_extrema.push_back(std::make_pair(-1,1));
}
std::tuple<bool, double, pybind11::dict> SimpleLearner::calc_reward() {
    // auto contact = (*contacts_)[parent_->id().id()];
    const double x = state_->pos()(0);
    double reward = dead ? -500 : -1;
    bool done = dead;
    auto pos = state_->pos();
    if (pos(0) < 25 && pos(1) < 25) {
      reward = 1000;
      done = true;
    }
    printf("reward: %f\n",reward);
    // here we setup the debugging info.
    pybind11::dict info;
    return std::make_tuple(done, reward, info);
}
bool SimpleLearner::step_helper() {
  auto pos = parent_->state()->pos();
  Eigen::Vector3d v_goal = (Eigen::Vector3d(0,0,100) - pos).normalized() *30;
  Eigen::Vector3d v_non_team = Eigen::Vector3d(0,0,0);
  int cont_idx = 0;
  auto getter = [&]() -> double {
    for (auto i = action.continuous.begin(); i != action.continuous.end(); ++i)
    return action.continuous[cont_idx++];
  };
  // for (auto it = contacts_->begin();
  //       it != contacts_->end(); it++) {
  //         auto contact = (*it).second;
  //       if (contact.id().id() != parent_->id().id()) {
  //            v_non_team += (contact.state()->pos() - pos);
  //       }
  //  }
  double action1 = getter();
  Eigen::Vector3d vel =  (action1 * v_goal);
  printf("action in env: %f\n", action1);
  printf("pos in env: %f %f %f\n", pos(0),pos(1),pos(2));
  // double action2 = getter();
  // Eigen::Vector3d vel =  action1 * v_goal + action2 * v_non_team;
  // printf("action in env: %f %f\n", action1, action2);
  vars_.output(output_vel_x_idx_, vel[0]);
  vars_.output(output_vel_y_idx_, vel[1]);
  vars_.output(output_vel_z_idx_, vel[2]);
  return true;
}
} // namespace autonomy
} // namespace scrimmage

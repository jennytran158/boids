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

#include <boids/plugins/autonomy/Charmer/Charmer.h>

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
                scrimmage::autonomy::Charmer,
                Charmer_plugin)

namespace scrimmage {
namespace autonomy {

void Charmer::init(std::map<std::string, std::string> &params) {
    initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);
    sense_range_ = sc::get<double>("sense_range", params, sense_range_); //assumes universal sense range for all drones
    attack_sense_range_ = sc::get<double>("attack_sense_range", params, attack_sense_range_);
    w_avoid_nonteam_ = sc::get<double>("avoid_nonteam_weight", params, 1.0);
    w_goal_ = sc::get<double>("goal_weight", params, 1.0);

    goal_ = Eigen::Vector3d(0,0,100); //hardcoded
}

void Charmer::update_desired_state(Eigen::Vector3d &v) {
  double desired_heading = atan2(v(1), v(0));
  Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1000;
  Eigen::Vector3d temp = state_->pos() + v * rel_pos.norm();
  desired_state_->pos() = Eigen::Vector3d(temp(0),temp(1),state_->pos()(2) + v(2));
  desired_state_->quat().set(0, 0,desired_heading);
}

bool Charmer::following_me(int id){
    sc::StatePtr ent_state = contacts_->at(id).state();
    Eigen::Vector3d towards_me = (state_->pos() - ent_state->pos());
    cout << id << ": " << towards_me.normalized().dot(ent_state->vel().normalized()) << endl;
    return (bool) (towards_me.normalized().dot(ent_state->vel().normalized()) > 0.8 && (ent_state->pos() - state_->pos()).norm() < sense_range_);
}

bool Charmer::is_dangerous(int id){
    sc::StatePtr ent_state = contacts_->at(id).state();
    Eigen::Vector3d towards_me = (state_->pos() - ent_state->pos());
    //cout << id << ": " << towards_me.normalized().dot(ent_state->vel().normalized()) << endl;
    return (bool) ((ent_state->pos() - state_->pos()).norm() < 3 * attack_sense_range_);
}

bool Charmer::step_autonomy(double t, double dt) {
    // Find nearest entity on other team. Loop through each contact, calculate
    // distance to entity, save the ID of the entity that is closest.
    double min_dist = std::numeric_limits<double>::infinity();
    std::vector<int> following; //vector of IDs of enemy contacts following me
    Eigen::Vector3d sum_velocity;
    sum_velocity.setZero();
    //cout << "sum_velocity init: " << sum_velocity << endl;
    for (auto it = contacts_->begin(); it != contacts_->end(); it++) {

        // Skip if this contact is on the same team
        if (it->second.id().team_id() == parent_->id().team_id()) {
            continue;
        }
        //
        // // Calculate distance to entity
        // double dist = (it->second.state()->pos() - state_->pos()).norm();
        //
        // if (dist < min_dist) {
        //     // If this is the minimum distance, save distance and reference to
        //     // entity
        //     min_dist = dist;
        //     follow_id_ = it->first;
        // }

        if (is_dangerous(it->second.id().id())){
            following.push_back(it->second.id().id());
        }
    }

    for (auto it = following.begin(); it != following.end(); it++){
        sc::StatePtr ent_state = contacts_->at(*it).state();
        Eigen::Vector3d towards_me = (state_->pos() - ent_state->pos());
        double dist = towards_me.norm();
        sum_velocity += (1 / dist) * ent_state->vel();
    }

    cout << following.size() << endl;
    Eigen::Vector3d v_goal = goal_ - state_->pos();
    Eigen::Vector3d avg_vec = w_avoid_nonteam_ * sum_velocity.normalized() + w_goal_ * v_goal.normalized();
    update_desired_state(avg_vec);

    // // Head toward entity on other team
    // if (contacts_->count(follow_id_) > 0) {
    //     // Get a reference to the entity's state.
    //     sc::StatePtr ent_state = contacts_->at(follow_id_).state();
    //
    //     //Eigen::Vector3d towards_me = (state_->pos() - ent_state->pos());
    //     Eigen::Vector3d towards_it = (ent_state->pos() - state_->pos());
    //     //cout << "ent_vel: " << ent_state->vel().normalized() << endl;
    //     //cout << "Dot Product: " << towards_me.normalized().dot(ent_state->vel().normalized()) << endl;
    //
    //     // if(following.size() >= 2){
    //     //     cout << "running!" << endl;
    //     //     update_desired_state(sum_velocity);
    //     // }
    //     // else{
    //     //     cout << "following!" << endl;
    //     //     update_desired_state(towards_it);
    //     // }
    // }
    return true;
}
} // namespace autonomy
} // namespace scrimmage

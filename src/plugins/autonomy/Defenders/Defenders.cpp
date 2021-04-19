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

#include <boids/plugins/autonomy/Defenders/Defenders.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/common/RTree.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/Shape.h>

#include <iostream>
#include <limits>

using std::list;
using std::cout;
using std::endl;


REGISTER_PLUGIN(scrimmage::Autonomy,
  scrimmage::autonomy::Defenders,
  Defenders_plugin)

  namespace scrimmage {
    namespace autonomy {
      void Defenders::init(std::map<std::string, std::string> &params) {
        MBoid::init(params);
        attack_sense_range_ = scrimmage::get<double>("attack_sense_range", params, attack_sense_range_);
        killed_ids_ad = advertise("GlobalNetwork", "Killed_IDs",100);
        auto killed_callback = [&] (scrimmage::MessagePtr<int> msg) {
          if (msg->data == follow_id_) {
            attack_time_ = 0;
            attack_end_time_ = 0;
            follow_id_ = -1;
          }
        };
        subscribe<int>("GlobalNetwork", "Killed_IDs", killed_callback);
      }

      void Defenders::show_attack_FOV() {
        show_FOV(attack_sense_range_, fov_width_angle_, fov_height_angle_);
      }
      Eigen::Vector3d Defenders::step_goal() {
        Eigen::Vector3d goal = Eigen::Vector3d(0,0,100);
        if (follow_id_ > 0) {
          StatePtr enemy_state = (*contacts_)[follow_id_].state();
          try {
            goal = enemy_state->pos();
          } catch (const std::bad_typeid &e) {
            follow_id_ = -1;
          }
        }
        return goal;
      }
      void Defenders::step_attack(double t, double dt) {
        Eigen::Vector3d &pos = state_->pos();
        auto ent = contacts_->at(follow_id_);
        scrimmage::StatePtr ent_state = ent.state();
        Eigen::Vector3d &ent_pos = ent_state->pos();
        Eigen::Vector2d p = (ent_pos - pos).head<2>();
        double dist = p.norm();

        // Red line: indicates a pair of drone and its target
        auto line = scrimmage::shape::make_line(state_->pos(), goal_,Eigen::Vector3d(255, 0, 0),0.75);
        line->set_persistent(false);
        line->set_ttl(1);
        draw_shape(line);

        if (dist <= attack_sense_range_) {
          attacking_ = true;
          if (attack_time_ == 0){
            attack_time_ = t;
            attack_end_time_ = attack_time_ + dt * attack_duration_;
          }
          if (attack_time_ >= attack_end_time_){
            attack_time_ = 0;
            attack_end_time_ = 0;
            //broadcast attacked ids
            auto msg = std::make_shared<scrimmage::Message<int>>();
            msg->data = follow_id_;
            killed_ids_ad->publish(msg);
          }

          if (attack_time_ == t) {
            attack_time_ = t + dt;
          } else {
            attack_time_ = 0;
          }
          show_attack_FOV();
        }
      }
      void Defenders::get_neighbors_in_range(double range,std::vector<ID> &neighbors) {
        MBoid::get_neighbors_in_range(range,neighbors);
        follow_id_ = -1;
        double min_dist = 10e9;
        for (auto it = neighbors.begin(); it != neighbors.end();/* no inc */) {
          auto other = (*contacts_)[it->id()];
          auto dist = (other.state()->pos() - state_->pos()).norm();
          if (other.id().team_id() != parent_->id().team_id()&& dist < min_dist ) {
              follow_id_ = it->id();
              min_dist = dist;
          }
          ++it;
        }
      }
      bool Defenders::step_autonomy(double t, double dt) {
        MBoid::step_autonomy(t,dt);
        if (goal_ != Eigen::Vector3d(0,0,100)) {
          step_attack(t, dt);
        }
        return true;
      }
    } // namespace autonomy
  } // namespace scrimmage

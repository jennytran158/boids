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

#include <boids/plugins/interaction/Base/Base.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>


#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
  scrimmage::interaction::Base,
  Base_plugin)

  namespace scrimmage {
    namespace interaction {

      Base::Base() {
      }

      bool Base::init(std::map<std::string, std::string> &mission_params,
        std::map<std::string, std::string> &plugin_params) {
	  pub_boundary_ = advertise("GlobalNetwork", "Attacked");
          return true;
        }


        bool Base::step_entity_interaction(std::list<sc::EntityPtr> &ents,
          double t, double dt) {
            if (ents.empty()) {
              return true;
            }
            double  base_width = 50;
            double  base_height = 50;
            double  base_depth =  200;
            // Account for entities "colliding"
            if (green_base_ == NULL) {
              green_base_ = std::make_shared<scrimmage_proto::Shape>();
              green_base_->set_opacity(0.25);
              green_base_->set_persistent(true);
              sc::set(green_base_->mutable_color(), 0, 255, 0); // r, g, b
              //our base measurements
              green_base_->mutable_cuboid()->set_x_length(base_width);
              green_base_->mutable_cuboid()->set_y_length(base_height);
              green_base_->mutable_cuboid()->set_z_length(base_depth);
              sc::set(green_base_->mutable_cuboid()->mutable_center(), 0, 0, base_depth/2);
              draw_shape(green_base_);
            }
            for (EntityPtr ent1 : ents) {
              Eigen::Vector3d p1 = ent1->state_truth()->pos();
              // ignore distance between itself
              if (ent1->id().team_id() != 1) continue;

              // ignore collisions that have already occurred this time-step
              if (!ent1->is_alive()) continue;


              if (p1(0) < (base_width/2 - 1.5) && p1(0) > -(base_width/2 -1.5) &&
              p1(1) < (base_height/2 - 1.5) && p1(1) > -(base_height/2 - 1.5) &&
              p1(2) < base_depth -1.5) {
                // ent1->collision();

                if (red_base_ == NULL) {
                  red_base_ = std::make_shared<scrimmage_proto::Shape>();
                  red_base_->set_opacity(0.25);
                  red_base_->set_persistent(true);
                  sc::set(red_base_->mutable_color(), 255, 0, 0); // r, g, b
                  //our base measurements
                  red_base_->mutable_cuboid()->set_x_length(base_width);
                  red_base_->mutable_cuboid()->set_y_length(base_height);
                  red_base_->mutable_cuboid()->set_z_length(base_depth);
                  sc::set(red_base_->mutable_cuboid()->mutable_center(), 0, 0, base_depth/2);
                  draw_shape(red_base_);

		  //Send message that base is now red
		  base_attacked = 1;


		  auto msg = std::make_shared<sc::Message<int>>();
		  msg->data = base_attacked;
		  pub_boundary_->publish(msg);
                }
              }
            }
            return true;
            return true;
          }
        } // namespace interaction
      } // namespace scrimmage

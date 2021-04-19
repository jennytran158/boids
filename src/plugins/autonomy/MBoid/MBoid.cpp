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

#include <boids/plugins/autonomy/MBoid/MBoid.h>

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

namespace scrimmage {
  namespace autonomy {


    void MBoid::init(std::map<std::string, std::string> &params) {
      sense_range_ = scrimmage::get<double>("sense_range", params, sense_range_);

      max_speed_ = get<double>("max_speed", params, 21);

      w_align_ = get("align_weight", params, 0.01);
      w_avoid_team_ = get("avoid_team_weight", params, 0.95);
      w_centroid_ = get("centroid_weight", params, 0.05);
      w_avoid_nonteam_ = get("avoid_nonteam_weight", params, 1.0);
      w_goal_ = get<double>("goal_weight", params, 1.0);

      fov_height_angle_ = M_PI*get("fov_height_angle", params, 90)/180;
      fov_width_angle_ = M_PI*get("fov_width_angle", params, 90)/180;
      sense_range_ = get("sense_range", params, 1000);


      sphere_of_influence_ = get<double>("sphere_of_influence", params, 10);
      minimum_team_range_ = get<double>("minimum_team_range", params, 5);
      minimum_nonteam_range_ = get<double>("minimum_nonteam_range", params, 10);
      std::vector<double> goal_vec;
      if (get_vec<double>("goal", params, " ", goal_vec, 3)) {
        goal_ = vec2eigen(goal_vec);
      } else {
        goal_ = state_->pos() ;
      }

    }
    void MBoid::show_FOV(double sense_range,double fov_width_angle, double fov_height_angle) {
      //animate field of view
      double fov_width = 2 * (sense_range / tan((M_PI - fov_width_angle)/2));
      double fov_height = 2 * (sense_range / tan((M_PI - fov_width_angle)/2));
      list<Eigen::Vector3d> points;
      points.push_back(Eigen::Vector3d(0,0,0));
      points.push_back(Eigen::Vector3d(-sense_range,  fov_width/2, fov_height/2));
      points.push_back(Eigen::Vector3d(-sense_range, -fov_width/2, fov_height/2));
      points.push_back(Eigen::Vector3d(-sense_range, -fov_width/2,  -fov_height/2));
      points.push_back(Eigen::Vector3d(-sense_range, fov_width/2, -fov_height/2));
      points.push_back(Eigen::Vector3d(-sense_range,  fov_width/2, fov_height/2));
      points.push_back(Eigen::Vector3d(0,0,0));
      list<Eigen::Vector3d> points_transformed;
      for (auto iter = points.begin(); iter != points.end();iter++){
        points_transformed.push_back(state_->pos() - (state_->quat().normalized() * *iter));
      }
      auto FOV = scrimmage::shape::make_polygon(points_transformed,  Eigen::Vector3d(0, 0, 0),0.1);
      FOV->set_persistent(false);
      FOV->set_ttl(1);
      draw_shape(FOV);
    }
    void MBoid::get_neighbors_in_range(double range,std::vector<ID> &neighbors) {
      rtree_->neighbors_in_range(state_->pos(), neighbors, range);
      for (auto it = neighbors.begin(); it != neighbors.end();
    /* no inc */) {
    // Ignore own position / id
    if (it->id() == parent_->id().id()) {
      it = neighbors.erase(it);
    } else {
      auto other = (*contacts_)[it->id()];
      scrimmage::StatePtr other_state = other.state();
      //keep team members in sense_range_
      if (other.id().team_id() == parent_->id().team_id()) {
        ++it;
      } else if (state_->InFieldOfView(*other_state, fov_width_angle_, fov_height_angle_)) {
        ++it;
      } else {
        // The enemy is "behind." Remove it.
        it = neighbors.erase(it);
      }
    }
  }

}
void MBoid::update_desired_state(Eigen::Vector3d &v) {
  double desired_heading = atan2(v(1), v(0));
  Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1000;
  Eigen::Vector3d temp = state_->pos() + v * rel_pos.norm();
  desired_state_->pos() = Eigen::Vector3d(temp(0),temp(1),state_->pos()(2) + v(2));
  desired_state_->quat().set(0, 0,desired_heading);
}
bool MBoid::step_autonomy(double t, double dt) {
  // Remove non-team members not within FOV and team members not within sense_range_
  std::vector<ID> rtree_neighbors;
  get_neighbors_in_range(sense_range_,rtree_neighbors);
  goal_ = step_goal();

  // Boids
  //move to goal
  Eigen::Vector3d v_goal = (goal_ - state_->pos()).normalized();

  double heading = 0;
  Eigen::Vector3d align(0, 0, 0);
  Eigen::Vector3d centroid(0, 0, 0);

  // repulsion vectors
  std::vector<Eigen::Vector3d> O_team_vecs;
  std::vector<Eigen::Vector3d> O_nonteam_vecs;


  double min_dist = 1e10;
  double team_size = 0;

  for (ID id : rtree_neighbors) {
    bool is_team = (id.team_id() == parent_->id().team_id());
    StatePtr other_state = (*contacts_)[id.id()].state();
    auto other_pos = other_state->pos();

    Eigen::Vector3d diff = other_pos - state_->pos();
    double dist = diff.norm();
    if (!is_team && dist < min_dist ) {
      min_dist = dist;
    }

    double min_range = is_team ? minimum_team_range_ : minimum_nonteam_range_;
    double O_mag = 0;
    if (dist > sphere_of_influence_) {
      O_mag = 0;
    } else if (min_range < dist && dist <= sphere_of_influence_) {
      O_mag = (sphere_of_influence_ - dist) /
      (sphere_of_influence_ - min_range);
    } else if (dist <= min_range) {
      O_mag = 1e10;
    }

    Eigen::Vector3d O_dir = - O_mag * diff.normalized();
    if (is_team) {
      O_team_vecs.push_back(O_dir);
    } else {
      O_nonteam_vecs.push_back(O_dir);
    }

    // find centroid of team members and heading alignment
    if (is_team) {
      centroid = centroid + other_state->pos();
      align += other_state->vel().normalized();
      heading += other_state->quat().yaw();
      team_size += 1;
    }
  }

  Eigen::Vector3d align_vec(0, 0, 0);
  if (team_size > 0) {
    centroid = centroid / team_size;
    align = align / team_size;
    heading /= team_size;
    align_vec << cos(heading), sin(heading), 0;
  }

  // make sure vectors are well-behaved

  align = align_vec;
  Eigen::Vector3d v_align_normed = align.normalized();
  double v_align_norm = align.norm();
  if (v_align_normed.hasNaN()) {
    v_align_normed = Eigen::Vector3d::Zero();
    v_align_norm = 0;
  }

  // Normalize each team repulsion vector and sum
  Eigen::Vector3d O_team_vec(0, 0, 0);
  for (Eigen::Vector3d v : O_team_vecs) {
    if (v.hasNaN()) {
      continue; // ignore misbehaved vectors
    }
    O_team_vec += v;
  }

  // Normalize each nonteam repulsion vector and sum
  Eigen::Vector3d O_nonteam_vec(0, 0, 0);
  for (Eigen::Vector3d v : O_nonteam_vecs) {
    if (v.hasNaN()) {
      continue; // ignore misbehaved vectors
    }
    O_nonteam_vec += v;
  }

  // Apply weights
  Eigen::Vector3d v_goal_w_gain = v_goal * w_goal_;
  Eigen::Vector3d O_team_vec_w_gain = O_team_vec * w_avoid_team_;
  Eigen::Vector3d O_nonteam_vec_w_gain = O_nonteam_vec * w_avoid_nonteam_;
  Eigen::Vector3d v_centroid_w_gain = (centroid - state_->pos()).normalized() * w_centroid_;
  Eigen::Vector3d v_align_w_gain = v_align_normed * w_align_;

  double sum_norms = v_goal_w_gain.norm() + O_team_vec_w_gain.norm() +
  O_nonteam_vec_w_gain.norm() + v_centroid_w_gain.norm() +
  v_align_norm;

  Eigen::Vector3d v_sum = (v_goal_w_gain + O_team_vec_w_gain +
    O_nonteam_vec_w_gain + v_centroid_w_gain +
    v_align_w_gain) / sum_norms;

    // Scale
    Eigen::Vector3d vel_result = v_sum * max_speed_;
    if (rtree_neighbors.size() > 0) {
      update_desired_state(vel_result);
      // Draw resultant velocity vector:
      auto line = scrimmage::shape::make_line(state_->pos(), vel_result + state_->pos(),Eigen::Vector3d(255, 255, 0),0.75);
      line->set_persistent(false);
      line->set_ttl(1);
      draw_shape(line);
    } else {
      update_desired_state(v_goal);
    }
    return true;
  }
} // namespace autonomy
} // namespace scrimmage

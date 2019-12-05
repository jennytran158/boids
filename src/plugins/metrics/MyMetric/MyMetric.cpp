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

#include <boids/plugins/metrics/MyMetric/MyMetric.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/metrics/Metrics.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
                scrimmage::metrics::MyMetric,
                MyMetric_plugin)

namespace scrimmage {
namespace metrics {

MyMetric::MyMetric() {
}


void MyMetric::init(std::map<std::string, std::string> &params) {
    auto attacked_cb = [&] (scrimmage::MessagePtr<int> msg) {
        scores_[0].increment_base_attacked();
    };
    subscribe<int>("GlobalNetwork", "Attacked", attacked_cb);

    auto killed_cb = [&] (scrimmage::MessagePtr<int> msg) {
        scores_[0].increment_attackers_killed();
    };
    subscribe<int>("GlobalNetwork", "Killed_IDs", killed_cb);
}

bool MyMetric::step_metrics(double t, double dt) {
    return true;
}

void MyMetric::calc_team_scores() {
    for (auto &kv : scores_) {
        Score &score = kv.second;

        int team_id = (*id_to_team_map_)[kv.first];

        // Create the score, if necessary
        if (team_coll_scores_.count(team_id) == 0) {
            Score score;
            score.set_weights(params_);
            team_coll_scores_[team_id] = score;
        }
        team_coll_scores_[team_id].add_base_attacked(score.base_attacked());
        team_coll_scores_[team_id].add_attackers_killed(score.attackers_killed());
    }

    for (auto &kv : team_coll_scores_) {
        int team_id = kv.first;
        Score &score = kv.second;
        team_metrics_[team_id]["base_attacked"] = score.base_attacked();
        team_metrics_[team_id]["attackers_killed"] = score.attackers_killed();
        team_scores_[team_id] = score.score();
    }

    // list the headers we want put in the csv file
    headers_.push_back("base_attacked");
}

void MyMetric::print_team_summaries() {
    for (std::map<int, Score>::iterator it = team_coll_scores_.begin();
         it != team_coll_scores_.end(); ++it) {

        cout << "Score: " << it->second.score() << endl;
        cout << "Base Attacks: " << it->second.base_attacked() << endl;
        cout << "Attackers Killed: " << it->second.attackers_killed() << endl;
        cout << sc::generate_chars("-", 70) << endl;
    }
}
} // namespace metrics
} // namespace scrimmage

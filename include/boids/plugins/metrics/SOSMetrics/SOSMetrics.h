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

#ifndef INCLUDE_BOIDS_PLUGINS_METRICS_SOSMETRICS_SOSMETRICS_H_
#define INCLUDE_BOIDS_PLUGINS_METRICS_SOSMETRICS_SOSMETRICS_H_

#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/parse/ParseUtils.h>

#include <map>
#include <string>

namespace sc = scrimmage;

namespace scrimmage {
namespace metrics {

class Score {
 public:
    Score() {
        ground_collisions_ = 0;
        killed_ = 0;
    }

    bool set_weights(std::map<std::string, std::string> &params) {
        ground_collisions_w_ = sc::get<double>("ground_collisions_w", params, 0.0);
        killed_w_ = sc::get<double>("killed_w", params, 0.0);
        return true;
    }

    void increment_ground_collisions() { ground_collisions_++; }
    void add_ground_collisions(int c) { ground_collisions_ += c; }
    int ground_collisions() { return ground_collisions_; }
    void set_ground_collisions(int ground_collisions) {
        ground_collisions_ = ground_collisions;
    }

    void increment_killed() { killed_++; }
    void add_killed(int c) { killed_ += c; }
    int killed() { return killed_; }
    void set_killed(int killed)
    {
        killed_ = killed;
    }

    double score() {
        double s = ground_collisions() * ground_collisions_w_ + killed() * killed_w_;
        return s;
    }

 protected:
    int ground_collisions_ = 0;
    double ground_collisions_w_ = 0.0;

    int killed_ = 0;
    double killed_w_ = 0.0;
};


class SOSMetrics : public scrimmage::Metrics {
 public:
    SOSMetrics();
    virtual std::string name() { return std::string("SOSMetrics"); }
    virtual void init(std::map<std::string, std::string> &params);
    virtual bool step_metrics(double t, double dt);
    virtual void calc_team_scores();
    virtual void print_team_summaries();
 protected:
    std::map<int, Score> scores_;
    std::map<int, Score> team_coll_scores_;
    std::map<std::string, std::string> params_;
 private:
};

} // namespace metrics
} // namespace scrimmage
#endif // INCLUDE_BOIDS_PLUGINS_METRICS_SOSMETRICS_SOSMETRICS_H_

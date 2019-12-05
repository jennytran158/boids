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

#ifndef INCLUDE_BOIDS_PLUGINS_METRICS_MYMETRIC_MYMETRIC_H_
#define INCLUDE_BOIDS_PLUGINS_METRICS_MYMETRIC_MYMETRIC_H_

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
        base_attacked_ = 0;
    }

    bool set_weights(std::map<std::string, std::string> &params) {
        base_attacked_w_ = sc::get<double>("base_attacked_w", params, 50.0);
	attackers_killed_w_ = sc::get<double>("attackers_killed_w", params, -1.0);
        return true;
    }

    void increment_base_attacked() { base_attacked_++; }
    void add_base_attacked(int c) { base_attacked_ += c; }
    int base_attacked() { return base_attacked_; }
    void base_attacked(int base_attacked) {
        base_attacked_ = base_attacked;
    }


    void increment_attackers_killed() { attackers_killed_++; }
    void add_attackers_killed(int c) { attackers_killed_ += c; }
    int attackers_killed() { return attackers_killed_; }
    void attackers_killed(int attackers_killed) {
        attackers_killed_ = attackers_killed;
    }

    double score() {
        double s = base_attacked() * base_attacked_w_ + attackers_killed_ * attackers_killed_w_;
        return s;
    }

 protected:
    int base_attacked_ = 0;
    double base_attacked_w_ = 50.0;

    int attackers_killed_ = 0;
    double attackers_killed_w_ = -1.0;

};


class MyMetric : public scrimmage::Metrics {
 public:
    MyMetric();
    virtual std::string name() { return std::string("MyMetric"); }
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
#endif // INCLUDE_BOIDS_PLUGINS_METRICS_MYMETRIC_MYMETRIC_H_

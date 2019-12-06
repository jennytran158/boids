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

#ifndef INCLUDE_BOIDS_PLUGINS_SENSOR_SIMPLELEARNERSENSOR_SIMPLELEARNERSENSOR_H_
#define INCLUDE_BOIDS_PLUGINS_SENSOR_SIMPLELEARNERSENSOR_SIMPLELEARNERSENSOR_H_

#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/entity/Contact.h>
#include <scrimmage/pubsub/Publisher.h>

#include <scrimmage/plugins/sensor/ScrimmageOpenAISensor/ScrimmageOpenAISensor.h>
#include <random>
#include <vector>
#include <map>
#include <string>

namespace scrimmage {
namespace sensor {
class SimpleLearnerSensor :public scrimmage::sensor::ScrimmageOpenAISensor {
 public:
   void set_observation_space() override;
   void get_observation(double* data, uint32_t beg_idx, uint32_t end_idx) override;
};
} // namespace sensor
} // namespace scrimmage
#endif // INCLUDE_BOIDS_PLUGINS_SENSOR_SIMPLELEARNERSENSOR_SIMPLELEARNERSENSOR_H_

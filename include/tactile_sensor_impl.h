/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Luca Lach
*/

#ifndef KD45_CONTROLLER_TACTILE_SENSOR_IMPL_H

#include <tactile_sensor.h>

namespace kd45_controller {
TactileSensorBase::TactileSensorBase(ros::NodeHandle& nh, std::shared_ptr<std::vector<float>> forces, bool simulation) : nh_(nh), forces_(forces), sim(simulation){}

TactileSensorSim::TactileSensorSim(ros::NodeHandle& nh, std::shared_ptr<std::vector<float>> forces) : TactileSensorBase(nh, forces, true) {
    sub_ = nh.subscribe("/kd45_tactile", 0, &TactileSensorSim::sensor_cb_, this);
    ROS_INFO("Registered subscriber for \"/kd45_tactile\"");
}

void TactileSensorSim::sensor_cb_(const tactile_msgs::TactileStateConstPtr ts) {
    for (int i = 0; i < forces_->size(); i++){ // lock here?
        (*forces_)[i] = ts->sensors[i].values[0];
    }
}

    TactileSensorReal::TactileSensorReal(ros::NodeHandle& nh, std::shared_ptr<std::vector<float>> forces) : TactileSensorBase(nh, forces, false) {}
}

#endif  // KD45_CONTROLLER_TACTILE_SENSOR_IMPL_H

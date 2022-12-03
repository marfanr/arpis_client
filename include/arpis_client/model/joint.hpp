#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <string>

namespace arpis_client {

std::string joint_id_[20] = {
    "right_shoulder_pitch", // 1
    "left_shoulder_pitch",  // 2
    "right_shoulder_roll",  // 3
    "left_shoulder_roll",   // 4
    "right_elbow",          // 5
    "left_elbow",           // 6
    "right_hip_yaw",        // 7
    "left_hip_yaw",         // 8
    "right_hip_pitch",      // 9
    "left_hip_pitch",       // 10
    "right_hip_roll",       // 11
    "left_hip_roll",        // 12
    "right_knee",           // 13
    "left_knee",            // 14
    "right_ankle_roll",     // 15
    "left_ankle_roll",      // 16
    "right_ankle_pitch",    // 17
    "left_ankle_pitch",     // 18
    "head_pan",             // 19
    "head_tilt"             // 20
};

}

#endif
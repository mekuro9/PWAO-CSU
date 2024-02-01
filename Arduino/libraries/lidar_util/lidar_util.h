/*==============================================================
// Filename :       lidar_util.h
// Authors :        Mrinal Magar (s2689529)
// Version :        v1.0
// License :  
// Description :    Utility functions for lidar obstacle avoidance
// 
==============================================================*/

#if !defined(lidar_util_h)
#define lidar_util_h

#include<Arduino.h>
#include<vector>

void make_blind(std::vector<float>& angle, std::vector<float>& distance, int start, int end); // make the lidar ignore areas for obstacle avoidanceS

int find_closest(const std::vector<float>& angle, float target); // returns the index value of the closest angle to the target angle

#endif // lidar_util_h

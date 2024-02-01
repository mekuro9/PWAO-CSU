/*==============================================================
// Filename :       lidar_util.cpp
// Authors :        Mrinal Magar (s2689529)
// Version :        v1.0
// License :  
// Description :    Utility functions for lidar obstacle avoidance
// 
==============================================================*/

/*
    Lidar angle ranges from  approximately -179deg to +179deg. 
    These values are stored in an array of 3240 values(the resolution is approximately 0.12 deg)
    first element of array corresponds to -179 and last corresponds to +179
*/

#include<lidar_util.h>


/*
    This function takes the pointer to vectors (distance and angle) and creates a blind spot (ignores obstacles at)
    in the specified angle range which is given by the max and min angle values
*/
void make_blind(std::vector<float>& angle, std::vector<float>& distance, int start, int end){

    //erase the elements from int start to int end
     angle.erase (angle.begin()+start,angle.begin()+end);
     distance.erase(distance.begin()+start,distance.begin()+end);

}

int find_closest(const std::vector<float>& angle, float target){

    int n = sizeof(angle) / sizeof(angle[0]);
    int left = 0, right = n - 1;
    while (left < right) {
        if (abs(angle[left] - target)
            <= abs(angle[right] - target)) {
            right--;
        }
        else {
            left++;
        }
    }
    return left;
}
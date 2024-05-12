#include <iostream>
#include <vector>
#include <utility> 
#include <obstacle_detection/LidarUtility.h>

// Constructor implementation
LidarUtility::LidarUtility(float* angles, float* distances) : scanAngles(angles), scanDistances(distances) {}


std::vector<std::pair<float, float>> LidarUtility::getScanDataInRange(float minAngle, float maxAngle, int ARRAY_SIZE) {
    
    std::vector<std::pair<float, float>> result;
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        if (scanAngles[i] >= minAngle && scanAngles[i] <= maxAngle) {
            result.emplace_back(scanAngles[i], scanDistances[i]);
        }
    }
    return result;
}

bool LidarUtility::checkForObstacle(float minAngle, float maxAngle, float threshold, int ARRAY_SIZE)
{
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        if (scanAngles[i] >= minAngle && scanAngles[i] <= maxAngle) {
            if (scanDistances[i] < threshold) {
                return true; 
            }
        }
    }
    return false;
}

bool LidarUtility::checkObstacleInFilteredData(const std::vector<std::pair<float, float>>& filteredData, float threshold) {
    for (const auto& data : filteredData) {
        if (data.second < threshold) {
            return true;  
        }
    }
    return false;  
}
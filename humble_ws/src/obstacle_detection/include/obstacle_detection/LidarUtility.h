#include <iostream>
#include <vector>
#include <utility> 

#ifndef LIDARUTILITY_H
#define LIDARUTILITY_H


class LidarUtility {

public:
    LidarUtility(float* angles, float* distances);
    std::vector<std::pair<float, float>> getScanDataInRange(float minAngle, float maxAngle, int array_size);
    bool checkForObstacle(float minAngle, float maxAngle, float threshold, int ARRAY_SIZE);
    bool checkObstacleInFilteredData(const std::vector<std::pair<float, float>>& filteredData, float threshold);

private:
    float* scanAngles;
    float* scanDistances;
};


#endif // lidarutility
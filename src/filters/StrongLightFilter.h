#ifndef STRONGLIGHTFILTER_H
#define STRONGLIGHTFILTER_H
#include "FilterInterface.h"


// High-intensity light filter (trailing filter)
class YDLIDAR_API StrongLightFilter : public FilterInterface
{
public:
    StrongLightFilter();
    virtual ~StrongLightFilter();
    
    virtual void filter(const LaserScan &in,
                int lidarType,
                int version,
                LaserScan &out);
    void setMaxDist(float dist) {maxDist = dist;}
    void setMaxAngle(float angle) {maxAngle = angle;}
    void setMinNoise(int noise) {minNoise = noise;}

protected:
    struct Point
    {
        float x = .0;
        float y = .0;

        Point(float x = .0, float y = .0);

        static Point angular2Polar(const Point &p); // Rectangular coordinates to polar coordinates
        static Point polar2Angular(const Point &p); // Polar coordinates to rectangular coordinates
        // Calculate the distance from a point in a rectangular coordinate system to a line.
        static float calcDist(
            const Point &p,
            const Point &p1,
            const Point &p2);
        // Calculate the length of the vector
        static float calcLen(
            const Point &v);
        // Calculate the product of vectors
        static float calcDot(
            const Point &v1,
            const Point &v2);
        // Calculate the angle between two line segments forming a straight line in a rectangular coordinate system.
        static float calcAngle(
            const Point &p1,
            const Point &p2,
            const Point &p3,
            const Point &p4);
    };

    float maxDist = 0.05; // Maximum distance threshold, in meters (this value can be modified as needed).
    float maxAngle = 12.0; // Maximum angle threshold, in degrees (this value can be modified as needed).
    int minNoise = 2; //Minimum number of consecutive noise points (this value can be modified as needed)
};

#endif // STRONGLIGHTFILTER_H
